#include "mono_camera_capture/mono_camera_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "sensor_msgs/image_encodings.hpp"

namespace mono_camera_capture
{
namespace
{

double captureFps(bool enable_color, int color_fps, bool enable_depth, int depth_fps)
{
  double fps = 1.0;
  if (enable_color) {
    fps = std::max(fps, static_cast<double>(color_fps));
  }
  if (enable_depth) {
    fps = std::max(fps, static_cast<double>(depth_fps));
  }
  return fps;
}

std::string deviceInfo(const rs2::device & device, rs2_camera_info info)
{
  return device.supports(info) ? device.get_info(info) : "unknown";
}

}  // namespace

MonoCameraNode::MonoCameraNode(const rclcpp::NodeOptions & options)
: Node("d435_camera_node", options)
{
  declareParameters();
  config_ = loadConfigFromParameters();

  std::string reason;
  if (!validateConfig(config_, reason)) {
    throw std::invalid_argument("Invalid D435 configuration: " + reason);
  }

  const auto qos = rclcpp::SensorDataQoS();
  color_image_pub_ = create_publisher<sensor_msgs::msg::Image>("color/image_raw", qos);
  color_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("color/camera_info", qos);
  depth_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
    "aligned_depth_to_color/image_raw", qos);
  depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
    "aligned_depth_to_color/camera_info", qos);

  last_reconnect_attempt_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  fps_window_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

  {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    startPipelineLocked();
  }

  recreateCaptureTimer(
    captureFps(config_.enable_color, config_.color_fps, config_.enable_depth, config_.depth_fps));
  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&MonoCameraNode::onParameters, this, std::placeholders::_1));
}

MonoCameraNode::~MonoCameraNode()
{
  std::lock_guard<std::mutex> lock(pipeline_mutex_);
  stopPipelineLocked();
}

void MonoCameraNode::declareParameters()
{
  declare_parameter<std::string>("serial_no", "");
  declare_parameter<int>("color_width", 1280);
  declare_parameter<int>("color_height", 720);
  declare_parameter<int>("color_fps", 30);
  declare_parameter<int>("depth_width", 848);
  declare_parameter<int>("depth_height", 480);
  declare_parameter<int>("depth_fps", 30);
  declare_parameter<bool>("enable_color", true);
  declare_parameter<bool>("enable_depth", true);
  declare_parameter<bool>("align_depth_to_color", true);
  declare_parameter<bool>("enable_emitter", true);
  declare_parameter<double>("laser_power", -1.0);
  declare_parameter<std::string>("color_frame_id", "camera_color_optical_frame");
  declare_parameter<std::string>(
    "depth_frame_id", "camera_aligned_depth_to_color_frame");
  declare_parameter<bool>("publish_camera_info", true);
  declare_parameter<bool>("log_fps", true);
  declare_parameter<bool>("reconnect_on_failure", true);
  declare_parameter<int>("reconnect_period_ms", 1000);
  declare_parameter<int>("frame_timeout_ms", 1000);
}

MonoCameraNode::CameraConfig MonoCameraNode::loadConfigFromParameters() const
{
  CameraConfig config;
  config.serial_no = get_parameter("serial_no").as_string();
  config.color_width = static_cast<int>(get_parameter("color_width").as_int());
  config.color_height = static_cast<int>(get_parameter("color_height").as_int());
  config.color_fps = static_cast<int>(get_parameter("color_fps").as_int());
  config.depth_width = static_cast<int>(get_parameter("depth_width").as_int());
  config.depth_height = static_cast<int>(get_parameter("depth_height").as_int());
  config.depth_fps = static_cast<int>(get_parameter("depth_fps").as_int());
  config.enable_color = get_parameter("enable_color").as_bool();
  config.enable_depth = get_parameter("enable_depth").as_bool();
  config.align_depth_to_color = get_parameter("align_depth_to_color").as_bool();
  config.enable_emitter = get_parameter("enable_emitter").as_bool();
  config.laser_power = get_parameter("laser_power").as_double();
  config.color_frame_id = get_parameter("color_frame_id").as_string();
  config.depth_frame_id = get_parameter("depth_frame_id").as_string();
  config.publish_camera_info = get_parameter("publish_camera_info").as_bool();
  config.log_fps = get_parameter("log_fps").as_bool();
  config.reconnect_on_failure = get_parameter("reconnect_on_failure").as_bool();
  config.reconnect_period_ms = static_cast<int>(
    get_parameter("reconnect_period_ms").as_int());
  config.frame_timeout_ms = static_cast<int>(get_parameter("frame_timeout_ms").as_int());
  return config;
}

bool MonoCameraNode::startPipelineLocked()
{
  stopPipelineLocked();

  try {
    pipeline_ = std::make_unique<rs2::pipeline>();
    rs2::config rs_config;
    if (!config_.serial_no.empty()) {
      rs_config.enable_device(config_.serial_no);
    }
    if (config_.enable_color) {
      rs_config.enable_stream(
        RS2_STREAM_COLOR, config_.color_width, config_.color_height,
        RS2_FORMAT_BGR8, config_.color_fps);
    }
    if (config_.enable_depth) {
      rs_config.enable_stream(
        RS2_STREAM_DEPTH, config_.depth_width, config_.depth_height,
        RS2_FORMAT_Z16, config_.depth_fps);
    }

    const auto profile = pipeline_->start(rs_config);
    pipeline_started_ = true;
    const auto device = profile.get_device();
    configureDepthSensor(device, config_);

    if (config_.align_depth_to_color) {
      align_to_color_ = std::make_unique<rs2::align>(RS2_STREAM_COLOR);
    } else {
      align_to_color_.reset();
    }

    RCLCPP_INFO(
      get_logger(),
      "Opened RealSense %s serial=%s; color=%s %dx%d@%d, depth=%s %dx%d@%d, align=%s, depth_scale=%.6f m",
      deviceInfo(device, RS2_CAMERA_INFO_NAME).c_str(),
      deviceInfo(device, RS2_CAMERA_INFO_SERIAL_NUMBER).c_str(),
      config_.enable_color ? "on" : "off",
      config_.color_width, config_.color_height, config_.color_fps,
      config_.enable_depth ? "on" : "off",
      config_.depth_width, config_.depth_height, config_.depth_fps,
      config_.align_depth_to_color ? "on" : "off",
      static_cast<double>(depth_scale_m_));
    return true;
  } catch (const rs2::error & error) {
    RCLCPP_ERROR(
      get_logger(), "Failed to start RealSense pipeline: %s (%s)",
      error.what(), error.get_failed_function().c_str());
    stopPipelineLocked();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Failed to start RealSense pipeline: %s", error.what());
    stopPipelineLocked();
  }
  return false;
}

void MonoCameraNode::stopPipelineLocked()
{
  if (pipeline_started_ && pipeline_) {
    try {
      pipeline_->stop();
    } catch (const rs2::error & error) {
      RCLCPP_WARN(get_logger(), "RealSense pipeline stop failed: %s", error.what());
    }
  }
  pipeline_started_ = false;
  align_to_color_.reset();
  pipeline_.reset();
}

void MonoCameraNode::configureDepthSensor(
  const rs2::device & device, const CameraConfig & config)
{
  if (!config.enable_depth) {
    return;
  }

  for (const auto & sensor : device.query_sensors()) {
    if (!sensor.is<rs2::depth_sensor>()) {
      continue;
    }

    const auto depth_sensor = sensor.as<rs2::depth_sensor>();
    depth_scale_m_ = depth_sensor.get_depth_scale();

    if (sensor.supports(RS2_OPTION_EMITTER_ENABLED)) {
      sensor.set_option(RS2_OPTION_EMITTER_ENABLED, config.enable_emitter ? 1.0F : 0.0F);
    }
    if (config.laser_power >= 0.0 && sensor.supports(RS2_OPTION_LASER_POWER)) {
      const auto range = sensor.get_option_range(RS2_OPTION_LASER_POWER);
      const float value = std::clamp(
        static_cast<float>(config.laser_power), range.min, range.max);
      sensor.set_option(RS2_OPTION_LASER_POWER, value);
      RCLCPP_INFO(get_logger(), "D435 laser power set to %.1f", static_cast<double>(value));
    }
    return;
  }

  throw std::runtime_error("The selected RealSense device does not expose a depth sensor");
}

void MonoCameraNode::captureOnce()
{
  rs2::frame color_frame;
  rs2::frame depth_frame;
  CameraConfig config_snapshot;
  const auto stamp = now();

  {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    config_snapshot = config_;

    if (!pipeline_started_) {
      if (!config_snapshot.reconnect_on_failure || restarting_.exchange(true)) {
        return;
      }
      if (last_reconnect_attempt_.nanoseconds() != 0 &&
        (stamp - last_reconnect_attempt_).nanoseconds() <
        static_cast<int64_t>(config_snapshot.reconnect_period_ms) * 1000000LL)
      {
        restarting_ = false;
        return;
      }

      last_reconnect_attempt_ = stamp;
      const std::string reconnect_target = config_snapshot.serial_no.empty() ?
        " to the first D435" : " to serial " + config_snapshot.serial_no;
      RCLCPP_WARN(
        get_logger(), "RealSense pipeline is stopped; reconnecting%s", reconnect_target.c_str());
      startPipelineLocked();
      restarting_ = false;
      return;
    }

    try {
      auto frames = pipeline_->wait_for_frames(
        static_cast<unsigned int>(config_snapshot.frame_timeout_ms));
      if (align_to_color_) {
        frames = align_to_color_->process(frames);
      }
      if (config_snapshot.enable_color) {
        color_frame = frames.get_color_frame();
      }
      if (config_snapshot.enable_depth) {
        depth_frame = frames.get_depth_frame();
      }
    } catch (const rs2::error & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), config_snapshot.reconnect_period_ms,
        "RealSense frame acquisition failed: %s", error.what());
      if (config_snapshot.reconnect_on_failure) {
        stopPipelineLocked();
        last_reconnect_attempt_ = stamp;
      }
      return;
    }
  }

  if (color_frame) {
    publishColorFrame(color_frame.as<rs2::video_frame>(), stamp, config_snapshot);
  }
  if (depth_frame) {
    publishDepthFrame(depth_frame.as<rs2::depth_frame>(), stamp, config_snapshot);
  }
  if (color_frame || depth_frame) {
    updateFpsStats(stamp, config_snapshot.log_fps);
  }
}

void MonoCameraNode::publishColorFrame(
  const rs2::video_frame & frame,
  const rclcpp::Time & stamp,
  const CameraConfig & config)
{
  sensor_msgs::msg::Image image;
  image.header.stamp = stamp;
  image.header.frame_id = config.color_frame_id;
  image.height = static_cast<uint32_t>(frame.get_height());
  image.width = static_cast<uint32_t>(frame.get_width());
  image.encoding = sensor_msgs::image_encodings::BGR8;
  image.is_bigendian = false;
  image.step = image.width * 3U;
  image.data.resize(static_cast<size_t>(image.step) * image.height);
  std::memcpy(image.data.data(), frame.get_data(), image.data.size());
  color_image_pub_->publish(image);

  if (config.publish_camera_info) {
    const auto profile = frame.get_profile().as<rs2::video_stream_profile>();
    color_info_pub_->publish(makeCameraInfo(profile, stamp, config.color_frame_id));
  }
}

void MonoCameraNode::publishDepthFrame(
  const rs2::depth_frame & frame,
  const rclcpp::Time & stamp,
  const CameraConfig & config)
{
  sensor_msgs::msg::Image image;
  image.header.stamp = stamp;
  image.header.frame_id = config.depth_frame_id;
  image.height = static_cast<uint32_t>(frame.get_height());
  image.width = static_cast<uint32_t>(frame.get_width());
  image.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  image.is_bigendian = false;
  image.step = image.width * sizeof(uint16_t);
  image.data.resize(static_cast<size_t>(image.step) * image.height);

  const auto * source = static_cast<const uint16_t *>(frame.get_data());
  const size_t pixel_count = static_cast<size_t>(image.width) * image.height;
  const double raw_to_mm = static_cast<double>(depth_scale_m_) * 1000.0;
  std::vector<uint16_t> converted(pixel_count);
  for (size_t index = 0; index < pixel_count; ++index) {
    if (source[index] == 0U) {
      converted[index] = 0U;
      continue;
    }
    const double millimeters = static_cast<double>(source[index]) * raw_to_mm;
    converted[index] = static_cast<uint16_t>(std::clamp(
      std::lround(millimeters), 0L,
      static_cast<long>(std::numeric_limits<uint16_t>::max())));
  }
  std::memcpy(image.data.data(), converted.data(), image.data.size());
  depth_image_pub_->publish(image);

  if (config.publish_camera_info) {
    const auto profile = frame.get_profile().as<rs2::video_stream_profile>();
    depth_info_pub_->publish(makeCameraInfo(profile, stamp, config.depth_frame_id));
  }
}

sensor_msgs::msg::CameraInfo MonoCameraNode::makeCameraInfo(
  const rs2::video_stream_profile & profile,
  const rclcpp::Time & stamp,
  const std::string & frame_id) const
{
  const auto intrinsics = profile.get_intrinsics();
  sensor_msgs::msg::CameraInfo info;
  info.header.stamp = stamp;
  info.header.frame_id = frame_id;
  info.width = static_cast<uint32_t>(intrinsics.width);
  info.height = static_cast<uint32_t>(intrinsics.height);

  if (intrinsics.model == RS2_DISTORTION_KANNALA_BRANDT4 ||
    intrinsics.model == RS2_DISTORTION_FTHETA)
  {
    info.distortion_model = "equidistant";
    info.d.assign(intrinsics.coeffs, intrinsics.coeffs + 4);
  } else {
    info.distortion_model = "plumb_bob";
    info.d.assign(intrinsics.coeffs, intrinsics.coeffs + 5);
  }

  info.k = {
    intrinsics.fx, 0.0, intrinsics.ppx,
    0.0, intrinsics.fy, intrinsics.ppy,
    0.0, 0.0, 1.0};
  info.r = {
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0};
  info.p = {
    intrinsics.fx, 0.0, intrinsics.ppx, 0.0,
    0.0, intrinsics.fy, intrinsics.ppy, 0.0,
    0.0, 0.0, 1.0, 0.0};
  return info;
}

void MonoCameraNode::recreateCaptureTimer(double fps)
{
  if (capture_timer_) {
    capture_timer_->cancel();
  }
  const auto period_ms = static_cast<int>(
    std::max(1.0, std::round(1000.0 / std::max(fps, 1.0))));
  capture_timer_ = create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&MonoCameraNode::captureOnce, this));
}

void MonoCameraNode::updateFpsStats(const rclcpp::Time & stamp, bool log_fps)
{
  if (!log_fps) {
    return;
  }
  if (fps_window_start_.nanoseconds() == 0) {
    fps_window_start_ = stamp;
    fps_frame_count_ = 0;
  }

  ++fps_frame_count_;
  const double elapsed = (stamp - fps_window_start_).seconds();
  if (elapsed >= 1.0) {
    RCLCPP_INFO(
      get_logger(), "RealSense frameset FPS: %.2f",
      static_cast<double>(fps_frame_count_) / elapsed);
    fps_window_start_ = stamp;
    fps_frame_count_ = 0;
  }
}

bool MonoCameraNode::validateConfig(
  const CameraConfig & config, std::string & reason) const
{
  if (!config.enable_color && !config.enable_depth) {
    reason = "at least one of enable_color or enable_depth must be true";
    return false;
  }
  if (config.align_depth_to_color && (!config.enable_color || !config.enable_depth)) {
    reason = "align_depth_to_color requires both color and depth streams";
    return false;
  }
  if (config.enable_color &&
    (config.color_width <= 0 || config.color_height <= 0 || config.color_fps <= 0))
  {
    reason = "color width, height and fps must be positive";
    return false;
  }
  if (config.enable_depth &&
    (config.depth_width <= 0 || config.depth_height <= 0 || config.depth_fps <= 0))
  {
    reason = "depth width, height and fps must be positive";
    return false;
  }
  if (config.reconnect_period_ms < 100) {
    reason = "reconnect_period_ms must be at least 100";
    return false;
  }
  if (config.frame_timeout_ms < 100) {
    reason = "frame_timeout_ms must be at least 100";
    return false;
  }
  if (config.color_frame_id.empty() || config.depth_frame_id.empty()) {
    reason = "frame IDs must not be empty";
    return false;
  }
  return true;
}

rcl_interfaces::msg::SetParametersResult MonoCameraNode::onParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  CameraConfig next_config;
  {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    next_config = config_;
  }

  bool restart_required = false;
  bool timer_required = false;
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();
    if (name == "serial_no") {
      next_config.serial_no = parameter.as_string();
      restart_required = true;
    } else if (name == "color_width") {
      next_config.color_width = static_cast<int>(parameter.as_int());
      restart_required = true;
    } else if (name == "color_height") {
      next_config.color_height = static_cast<int>(parameter.as_int());
      restart_required = true;
    } else if (name == "color_fps") {
      next_config.color_fps = static_cast<int>(parameter.as_int());
      restart_required = true;
      timer_required = true;
    } else if (name == "depth_width") {
      next_config.depth_width = static_cast<int>(parameter.as_int());
      restart_required = true;
    } else if (name == "depth_height") {
      next_config.depth_height = static_cast<int>(parameter.as_int());
      restart_required = true;
    } else if (name == "depth_fps") {
      next_config.depth_fps = static_cast<int>(parameter.as_int());
      restart_required = true;
      timer_required = true;
    } else if (name == "enable_color") {
      next_config.enable_color = parameter.as_bool();
      restart_required = true;
    } else if (name == "enable_depth") {
      next_config.enable_depth = parameter.as_bool();
      restart_required = true;
    } else if (name == "align_depth_to_color") {
      next_config.align_depth_to_color = parameter.as_bool();
      restart_required = true;
    } else if (name == "enable_emitter") {
      next_config.enable_emitter = parameter.as_bool();
      restart_required = true;
    } else if (name == "laser_power") {
      next_config.laser_power = parameter.as_double();
      restart_required = true;
    } else if (name == "color_frame_id") {
      next_config.color_frame_id = parameter.as_string();
    } else if (name == "depth_frame_id") {
      next_config.depth_frame_id = parameter.as_string();
    } else if (name == "publish_camera_info") {
      next_config.publish_camera_info = parameter.as_bool();
    } else if (name == "log_fps") {
      next_config.log_fps = parameter.as_bool();
    } else if (name == "reconnect_on_failure") {
      next_config.reconnect_on_failure = parameter.as_bool();
    } else if (name == "reconnect_period_ms") {
      next_config.reconnect_period_ms = static_cast<int>(parameter.as_int());
    } else if (name == "frame_timeout_ms") {
      next_config.frame_timeout_ms = static_cast<int>(parameter.as_int());
    }
  }

  std::string reason;
  if (!validateConfig(next_config, reason)) {
    result.successful = false;
    result.reason = reason;
    return result;
  }

  {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    config_ = next_config;
    fps_window_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    fps_frame_count_ = 0;
    if (restart_required && !startPipelineLocked()) {
      RCLCPP_WARN(
        get_logger(),
        "D435 parameter update was accepted, but the pipeline could not restart; reconnect is enabled if configured");
    }
  }

  if (timer_required) {
    recreateCaptureTimer(captureFps(
      next_config.enable_color, next_config.color_fps,
      next_config.enable_depth, next_config.depth_fps));
  }
  return result;
}

}  // namespace mono_camera_capture
