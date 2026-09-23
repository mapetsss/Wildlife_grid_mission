#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include "std_msgs/msg/header.hpp"
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <opencv2/imgproc.hpp>

#include "rclcpp/rclcpp.hpp"
#include "ros2_yolos_cpp/adapters/detector_adapter.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "wildlife_vision/msg/animal_count.hpp"
#include "wildlife_vision/msg/animal_report.hpp"
#include "wildlife_vision/msg/vision_trigger.hpp"

using namespace std::chrono_literals;

namespace
{
std::string trim_copy(const std::string & input)
{
  const auto begin = std::find_if_not(
    input.begin(), input.end(), [](unsigned char c) {return std::isspace(c) != 0;});
  const auto end = std::find_if_not(
    input.rbegin(), input.rend(), [](unsigned char c) {return std::isspace(c) != 0;}).base();
  if (begin >= end) {
    return "";
  }
  return std::string(begin, end);
}

const char * state_name(int state)
{
  switch (state) {
    case 0:
      return "WAIT_TRIGGER";
    case 1:
      return "CAPTURE";
    case 2:
      return "INFERENCE";
    case 3:
      return "STATISTIC";
    case 4:
      return "PUBLISH";
    default:
      return "UNKNOWN";
  }
}
}  // namespace

class WildlifeVisionNode : public rclcpp::Node
{
public:
  WildlifeVisionNode()
  : Node("wildlife_vision_node")
  {
    load_parameters();
    initialize_detector();

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&WildlifeVisionNode::image_callback, this, std::placeholders::_1));
    trigger_sub_ = create_subscription<wildlife_vision::msg::VisionTrigger>(
      "/vision/trigger", 10,
      std::bind(&WildlifeVisionNode::trigger_callback, this, std::placeholders::_1));
    report_pub_ = create_publisher<wildlife_vision::msg::AnimalReport>(
      "/vision/animal_report", 10);
    debug_image_pub_ = create_publisher<sensor_msgs::msg::Image>("/vision/debug_image", 10);

    timer_ = create_wall_timer(20ms, std::bind(&WildlifeVisionNode::run_state_machine, this));

    RCLCPP_INFO(
      get_logger(),
      "wildlife_vision_node ready. image_topic=%s model=%s classes=%s detect_frames=%d",
      image_topic_.c_str(), model_path_.c_str(), classes_path_.c_str(), detect_frames_);
  }

private:
  enum class VisionState
  {
    WAIT_TRIGGER = 0,
    CAPTURE = 1,
    INFERENCE = 2,
    STATISTIC = 3,
    PUBLISH = 4
  };

  void load_parameters()
  {
    model_path_ = declare_parameter<std::string>("model_path", "");
    classes_path_ = declare_parameter<std::string>("classes_path", "");
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/color/image_raw");
    confidence_threshold_ = static_cast<float>(
      declare_parameter<double>("confidence_threshold", 0.5));
    nms_threshold_ = static_cast<float>(declare_parameter<double>("nms_threshold", 0.45));
    settle_time_s_ = declare_parameter<double>("settle_time_s", 0.5);
    capture_timeout_s_ = declare_parameter<double>("capture_timeout_s", 3.0);
    detect_frames_ = declare_parameter<int>("detect_frames", 3);
    debug_stream_enabled_ = declare_parameter<bool>("debug_stream_enabled", false);
    debug_stream_rate_hz_ = declare_parameter<double>("debug_stream_rate_hz", 3.0);
    use_gpu_ = declare_parameter<bool>("use_gpu", false);
    yolo_version_ = declare_parameter<std::string>("yolo_version", "auto");

    confidence_threshold_ = std::clamp(confidence_threshold_, 0.0f, 1.0f);
    nms_threshold_ = std::clamp(nms_threshold_, 0.0f, 1.0f);
    settle_time_s_ = std::clamp(settle_time_s_, 0.0, 5.0);
    capture_timeout_s_ = std::max(0.5, capture_timeout_s_);
    detect_frames_ = std::clamp(detect_frames_, 1, 10);
    debug_stream_rate_hz_ = std::clamp(debug_stream_rate_hz_, 0.2, 15.0);
  }

  void initialize_detector()
  {
    if (model_path_.empty() || classes_path_.empty()) {
      RCLCPP_ERROR(get_logger(), "model_path and classes_path are required.");
      return;
    }

    ros2_yolos_cpp::YolosConfig config;
    config.model_path = model_path_;
    config.labels_path = classes_path_;
    config.use_gpu = use_gpu_;
    config.conf_threshold = confidence_threshold_;
    config.nms_threshold = nms_threshold_;
    config.yolo_version = yolo_version_;

    detector_initialized_ = detector_.initialize(config);
    if (!detector_initialized_) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize YOLO detector.");
      return;
    }

    RCLCPP_INFO(
      get_logger(), "YOLO detector initialized with %zu classes.",
      detector_.getClassNames().size());
  }

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    try {
      cv::Mat bgr = image_to_bgr(*msg);
      std::lock_guard<std::mutex> lock(image_mutex_);
      latest_frame_ = std::move(bgr);
      latest_header_ = msg->header;
      latest_frame_version_++;
      has_latest_frame_ = true;
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to convert RGB image: %s", e.what());
    }
  }

  static cv::Mat image_to_bgr(const sensor_msgs::msg::Image & msg)
  {
    if (msg.encoding == sensor_msgs::image_encodings::BGR8) {
      return cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    }
    if (msg.encoding == sensor_msgs::image_encodings::RGB8) {
      cv::Mat bgr;
      cv::cvtColor(
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8)->image,
        bgr, cv::COLOR_RGB2BGR);
      return bgr;
    }
    if (msg.encoding == sensor_msgs::image_encodings::BGRA8) {
      cv::Mat bgr;
      cv::cvtColor(
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGRA8)->image,
        bgr, cv::COLOR_BGRA2BGR);
      return bgr;
    }
    if (msg.encoding == sensor_msgs::image_encodings::RGBA8) {
      cv::Mat bgr;
      cv::cvtColor(
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGBA8)->image,
        bgr, cv::COLOR_RGBA2BGR);
      return bgr;
    }
    return cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
  }

  void trigger_callback(const wildlife_vision::msg::VisionTrigger::SharedPtr msg)
  {
    const std::string grid = trim_copy(msg->grid);
    if (!msg->enable) {
      if (grid == active_grid_) {
        reset_to_wait("trigger disabled");
      }
      return;
    }

    if (grid.empty()) {
      RCLCPP_WARN(get_logger(), "Ignoring vision trigger with empty grid.");
      return;
    }

    if (!detector_initialized_) {
      publish_report(grid, false, "detector is not initialized");
      return;
    }

    if (state_ != VisionState::WAIT_TRIGGER) {
      RCLCPP_WARN(
        get_logger(), "Ignoring trigger for %s while busy with %s in %s.",
        grid.c_str(), active_grid_.c_str(), state_name(static_cast<int>(state_)));
      return;
    }

    active_grid_ = grid;
    max_counts_.clear();
    processed_frames_ = 0;
    trigger_time_ = now();
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      last_processed_frame_version_ = latest_frame_version_;
    }
    state_ = VisionState::CAPTURE;

    RCLCPP_INFO(get_logger(), "Vision trigger accepted for grid %s.", active_grid_.c_str());
  }

  void run_state_machine()
  {
    if (state_ == VisionState::WAIT_TRIGGER) {
      maybe_publish_debug_stream(now());
      return;
    }

    const auto stamp = now();
    if ((stamp - trigger_time_).seconds() < settle_time_s_) {
      return;
    }

    cv::Mat frame;
    std_msgs::msg::Header frame_header;
    std::uint64_t frame_version = 0;
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      if (has_latest_frame_ && latest_frame_version_ != last_processed_frame_version_) {
        frame = latest_frame_.clone();
        frame_header = latest_header_;
        frame_version = latest_frame_version_;
      }
    }

    if (frame.empty()) {
      if ((stamp - trigger_time_).seconds() > settle_time_s_ + capture_timeout_s_) {
        publish_report(active_grid_, false, "no fresh RGB image after trigger");
        reset_to_wait("capture timeout");
      }
      return;
    }

    last_processed_frame_version_ = frame_version;
    state_ = VisionState::INFERENCE;

    std::map<std::string, int> frame_counts;
    const auto detections = detector_.detect(frame, confidence_threshold_, nms_threshold_);
    publish_debug_image(frame, frame_header, detections);
    for (const auto & detection : detections) {
      if (!detection.class_name.empty()) {
        frame_counts[detection.class_name]++;
      }
    }

    state_ = VisionState::STATISTIC;
    for (const auto & [name, count] : frame_counts) {
      const auto existing = max_counts_.find(name);
      if (existing == max_counts_.end() || count > existing->second) {
        max_counts_[name] = count;
      }
    }
    processed_frames_++;

    if (processed_frames_ >= detect_frames_) {
      state_ = VisionState::PUBLISH;
      publish_report(active_grid_, true, "ok");
      reset_to_wait("report published");
    } else {
      state_ = VisionState::CAPTURE;
    }
  }

  void maybe_publish_debug_stream(const rclcpp::Time & stamp)
  {
    if (!debug_stream_enabled_ || !detector_initialized_) {
      return;
    }
    if (last_debug_stream_time_.nanoseconds() != 0 &&
      (stamp - last_debug_stream_time_).seconds() < 1.0 / debug_stream_rate_hz_)
    {
      return;
    }

    cv::Mat frame;
    std_msgs::msg::Header frame_header;
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      if (!has_latest_frame_) {
        return;
      }
      frame = latest_frame_.clone();
      frame_header = latest_header_;
    }

    if (frame.empty()) {
      return;
    }

    last_debug_stream_time_ = stamp;
    const auto detections = detector_.detect(frame, confidence_threshold_, nms_threshold_);
    publish_debug_image(frame, frame_header, detections);
  }

  void publish_report(const std::string & grid, bool success, const std::string & message)
  {
    wildlife_vision::msg::AnimalReport report;
    report.grid = grid;
    report.success = success;
    report.message = message;

    for (const auto & [name, count] : max_counts_) {
      wildlife_vision::msg::AnimalCount animal;
      animal.name = name;
      animal.count = static_cast<std::uint16_t>(
        std::min(count, static_cast<int>(std::numeric_limits<std::uint16_t>::max())));
      report.animals.push_back(std::move(animal));
    }

    report_pub_->publish(report);

    std::ostringstream summary;
    for (const auto & animal : report.animals) {
      if (summary.tellp() > 0) {
        summary << ", ";
      }
      summary << animal.name << "=" << animal.count;
    }
    RCLCPP_INFO(
      get_logger(), "Published animal report: grid=%s success=%s animals=[%s] message=%s",
      report.grid.c_str(), success ? "true" : "false", summary.str().c_str(),
      report.message.c_str());
  }

  void publish_debug_image(
    const cv::Mat & frame,
    const std_msgs::msg::Header & header,
    const std::vector<ros2_yolos_cpp::DetectionResult> & detections)
  {
    if (!debug_image_pub_ || frame.empty()) {
      return;
    }

    cv::Mat debug = frame.clone();
    detector_.drawDetections(debug, detections);
    if (detections.empty()) {
      cv::putText(
        debug, "no detection", cv::Point(16, 32), cv::FONT_HERSHEY_SIMPLEX,
        0.9, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }

    auto debug_msg = cv_bridge::CvImage(
      header, sensor_msgs::image_encodings::BGR8, debug).toImageMsg();
    debug_image_pub_->publish(*debug_msg);
  }

  void reset_to_wait(const std::string & reason)
  {
    RCLCPP_DEBUG(get_logger(), "Vision state reset: %s.", reason.c_str());
    state_ = VisionState::WAIT_TRIGGER;
    active_grid_.clear();
    processed_frames_ = 0;
    max_counts_.clear();
  }

  ros2_yolos_cpp::DetectorAdapter detector_;
  bool detector_initialized_{false};

  std::string model_path_;
  std::string classes_path_;
  std::string image_topic_;
  std::string yolo_version_{"auto"};
  float confidence_threshold_{0.5f};
  float nms_threshold_{0.45f};
  double settle_time_s_{0.5};
  double capture_timeout_s_{3.0};
  double debug_stream_rate_hz_{3.0};
  int detect_frames_{3};
  bool debug_stream_enabled_{false};
  bool use_gpu_{false};

  std::mutex image_mutex_;
  cv::Mat latest_frame_;
  std_msgs::msg::Header latest_header_;
  std::uint64_t latest_frame_version_{0};
  std::uint64_t last_processed_frame_version_{0};
  bool has_latest_frame_{false};

  VisionState state_{VisionState::WAIT_TRIGGER};
  std::string active_grid_;
  rclcpp::Time trigger_time_{0, 0, RCL_SYSTEM_TIME};
  rclcpp::Time last_debug_stream_time_{0, 0, RCL_SYSTEM_TIME};
  int processed_frames_{0};
  std::map<std::string, int> max_counts_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<wildlife_vision::msg::VisionTrigger>::SharedPtr trigger_sub_;
  rclcpp::Publisher<wildlife_vision::msg::AnimalReport>::SharedPtr report_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WildlifeVisionNode>());
  rclcpp::shutdown();
  return 0;
}
