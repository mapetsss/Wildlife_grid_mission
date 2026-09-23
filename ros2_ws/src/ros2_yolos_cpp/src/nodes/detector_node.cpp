// Copyright 2024 YOLOs-CPP Team
// SPDX-License-Identifier: AGPL-3.0

#include "ros2_yolos_cpp/nodes/detector_node.hpp"

#include "ros2_yolos_cpp/conversion/detection_converter.hpp"
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

#include <sensor_msgs/image_encodings.hpp>

namespace ros2_yolos_cpp {

YolosDetectorNode::YolosDetectorNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("yolos_detector", options) {
    RCLCPP_INFO(get_logger(), "YolosDetectorNode created for RealSense aligned depth");
    declareParameters();
}

void YolosDetectorNode::declareParameters() {
    declare_parameter("model_path", rclcpp::PARAMETER_STRING);
    declare_parameter("labels_path", rclcpp::PARAMETER_STRING);
    declare_parameter("use_gpu", false);
    declare_parameter("conf_threshold", 0.4);
    declare_parameter("nms_threshold", 0.45);
    declare_parameter("yolo_version", "auto");
    declare_parameter("publish_timing", false);
    declare_parameter("use_depth", true);
    declare_parameter("depth_sample_radius_px", 3);
    declare_parameter("min_depth_m", 0.15);
    declare_parameter("max_depth_m", 10.0);
    declare_parameter("max_depth_age_s", 0.20);
}

YolosConfig YolosDetectorNode::loadConfig() {
    YolosConfig config;
    config.model_path = get_parameter("model_path").as_string();
    config.labels_path = get_parameter("labels_path").as_string();
    config.use_gpu = get_parameter("use_gpu").as_bool();
    config.conf_threshold = static_cast<float>(get_parameter("conf_threshold").as_double());
    config.nms_threshold = static_cast<float>(get_parameter("nms_threshold").as_double());
    config.yolo_version = get_parameter("yolo_version").as_string();

    model_path_ = config.model_path;
    labels_path_ = config.labels_path;
    use_gpu_ = config.use_gpu;
    conf_threshold_ = config.conf_threshold;
    nms_threshold_ = config.nms_threshold;
    yolo_version_ = config.yolo_version;
    publish_timing_ = get_parameter("publish_timing").as_bool();
    use_depth_ = get_parameter("use_depth").as_bool();
    depth_sample_radius_px_ = static_cast<int>(get_parameter("depth_sample_radius_px").as_int());
    min_depth_m_ = get_parameter("min_depth_m").as_double();
    max_depth_m_ = get_parameter("max_depth_m").as_double();
    max_depth_age_s_ = get_parameter("max_depth_age_s").as_double();
    return config;
}

YolosDetectorNode::CallbackReturn YolosDetectorNode::on_configure(const rclcpp_lifecycle::State&) {
    RCLCPP_INFO(get_logger(), "Configuring...");
    try {
        auto config = loadConfig();
        if (config.model_path.empty() || config.labels_path.empty()) {
            RCLCPP_ERROR(get_logger(), "model_path and labels_path are required");
            return CallbackReturn::FAILURE;
        }
        if (depth_sample_radius_px_ < 0 || min_depth_m_ <= 0.0 ||
            max_depth_m_ <= min_depth_m_ || max_depth_age_s_ < 0.0) {
            RCLCPP_ERROR(
                get_logger(),
                "Invalid depth parameters: radius must be >= 0, 0 < min_depth_m < max_depth_m, max_depth_age_s >= 0");
            return CallbackReturn::FAILURE;
        }

        detector_ = createDetectorAdapter();
        if (!detector_->initialize(config)) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize detector");
            return CallbackReturn::FAILURE;
        }
        inference_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        det_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>("~/detections", 10);
        offset_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("~/offset_px", 10);
        offset_m_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/offset_m", 10);
        distance_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/distance_m", 10);
        if (publish_timing_) {
            timing_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/timing", 10);
        }
        RCLCPP_INFO(get_logger(), "Configured successfully");
        return CallbackReturn::SUCCESS;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Configuration failed: %s", e.what());
        return CallbackReturn::FAILURE;
    }
}

YolosDetectorNode::CallbackReturn YolosDetectorNode::on_activate(const rclcpp_lifecycle::State&) {
    RCLCPP_INFO(get_logger(), "Activating...");
    det_pub_->on_activate();
    if (offset_pub_) offset_pub_->on_activate();
    if (offset_m_pub_) offset_m_pub_->on_activate();
    if (distance_pub_) distance_pub_->on_activate();
    if (timing_pub_) timing_pub_->on_activate();

    auto options = rclcpp::SubscriptionOptions();
    options.callback_group = inference_cb_group_;
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "~/image_raw", rclcpp::SensorDataQoS(),
        std::bind(&YolosDetectorNode::imageCallback, this, std::placeholders::_1), options);
    if (use_depth_) {
        depth_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "~/depth_image", rclcpp::SensorDataQoS(),
            std::bind(&YolosDetectorNode::depthImageCallback, this, std::placeholders::_1), options);
        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "~/camera_info", rclcpp::SensorDataQoS(),
            std::bind(&YolosDetectorNode::cameraInfoCallback, this, std::placeholders::_1), options);
    }

    detect_service_ = create_service<srv::DetectImage>(
        "~/detect",
        std::bind(&YolosDetectorNode::detectServiceCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, inference_cb_group_);

    RCLCPP_INFO(
        get_logger(),
        "%s",
        use_depth_ ?
        "Activated in D435 RGB-D mode; inference runs on ~/detect requests" :
        "Activated in D435 RGB-only mode; metric outputs are disabled");
    return CallbackReturn::SUCCESS;
}

YolosDetectorNode::CallbackReturn YolosDetectorNode::on_deactivate(const rclcpp_lifecycle::State&) {
    image_sub_.reset();
    depth_image_sub_.reset();
    camera_info_sub_.reset();
    detect_service_.reset();
    {
        std::lock_guard<std::mutex> lock(latest_frame_mutex_);
        latest_frame_.release();
        latest_depth_m_.release();
        latest_header_ = std_msgs::msg::Header();
        latest_depth_header_ = std_msgs::msg::Header();
        latest_intrinsics_ = CameraIntrinsics();
        has_latest_frame_ = false;
        has_latest_depth_ = false;
    }
    det_pub_->on_deactivate();
    if (offset_pub_) offset_pub_->on_deactivate();
    if (offset_m_pub_) offset_m_pub_->on_deactivate();
    if (distance_pub_) distance_pub_->on_deactivate();
    if (timing_pub_) timing_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
}

YolosDetectorNode::CallbackReturn YolosDetectorNode::on_cleanup(const rclcpp_lifecycle::State&) {
    if (detector_) {
        detector_->shutdown();
        detector_.reset();
    }
    image_sub_.reset();
    depth_image_sub_.reset();
    camera_info_sub_.reset();
    detect_service_.reset();
    {
        std::lock_guard<std::mutex> lock(latest_frame_mutex_);
        latest_frame_.release();
        latest_depth_m_.release();
        latest_header_ = std_msgs::msg::Header();
        latest_depth_header_ = std_msgs::msg::Header();
        latest_intrinsics_ = CameraIntrinsics();
        has_latest_frame_ = false;
        has_latest_depth_ = false;
    }
    det_pub_.reset();
    offset_pub_.reset();
    offset_m_pub_.reset();
    distance_pub_.reset();
    timing_pub_.reset();
    return CallbackReturn::SUCCESS;
}

YolosDetectorNode::CallbackReturn YolosDetectorNode::on_shutdown(const rclcpp_lifecycle::State& state) {
    return on_cleanup(state);
}

void YolosDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    if (!msg) return;
    try {
        auto cv_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        std::lock_guard<std::mutex> lock(latest_frame_mutex_);
        latest_frame_ = cv_image->image.clone();
        latest_header_ = msg->header;
        has_latest_frame_ = true;
    } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Color image caching failed: %s", e.what());
    }
}

void YolosDetectorNode::depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    if (!msg) return;
    try {
        cv::Mat depth_m;
        if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
            msg->encoding == sensor_msgs::image_encodings::MONO16) {
            auto cv_depth = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            cv_depth->image.convertTo(depth_m, CV_32FC1, 0.001);
        } else if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            auto cv_depth = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            depth_m = cv_depth->image.clone();
        } else {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Unsupported D435 depth encoding '%s'; expected 16UC1 millimeters or 32FC1 meters",
                msg->encoding.c_str());
            return;
        }

        std::lock_guard<std::mutex> lock(latest_frame_mutex_);
        latest_depth_m_ = std::move(depth_m);
        latest_depth_header_ = msg->header;
        has_latest_depth_ = true;
    } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Depth image caching failed: %s", e.what());
    }
}

void YolosDetectorNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
    if (!msg) return;
    CameraIntrinsics intrinsics;
    intrinsics.fx = msg->k[0];
    intrinsics.fy = msg->k[4];
    intrinsics.cx = msg->k[2];
    intrinsics.cy = msg->k[5];
    intrinsics.width = static_cast<int>(msg->width);
    intrinsics.height = static_cast<int>(msg->height);
    intrinsics.valid = intrinsics.fx > 0.0 && intrinsics.fy > 0.0 &&
                       intrinsics.width > 0 && intrinsics.height > 0;
    if (!intrinsics.valid) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Received invalid D435 CameraInfo");
        return;
    }
    std::lock_guard<std::mutex> lock(latest_frame_mutex_);
    latest_intrinsics_ = intrinsics;
}

void YolosDetectorNode::detectServiceCallback(
    const std::shared_ptr<srv::DetectImage::Request> request,
    std::shared_ptr<srv::DetectImage::Response> response) {
    if (!response) return;
    response->distance_m.data.clear();
    if (!detector_ || !detector_->isInitialized()) {
        RCLCPP_WARN(get_logger(), "Detector is not initialized, returning an empty result");
        return;
    }
    if (!request) {
        RCLCPP_WARN(get_logger(), "Received null detect request");
        return;
    }
    // DetectImage keeps its legacy Image request for API compatibility.  D435
    // inference uses the latest synchronized color/depth cache instead.
    (void)request;

    cv::Mat frame;
    cv::Mat depth_m;
    std_msgs::msg::Header header;
    std_msgs::msg::Header depth_header;
    CameraIntrinsics intrinsics;
    {
        std::lock_guard<std::mutex> lock(latest_frame_mutex_);
        if (!has_latest_frame_ || latest_frame_.empty()) {
            RCLCPP_WARN(get_logger(), "No D435 color image cached yet");
            return;
        }
        frame = latest_frame_.clone();
        header = latest_header_;
        intrinsics = latest_intrinsics_;
        if (has_latest_depth_ && !latest_depth_m_.empty()) {
            depth_m = latest_depth_m_.clone();
            depth_header = latest_depth_header_;
        }
    }

    if (!depth_m.empty() && max_depth_age_s_ > 0.0) {
        const rclcpp::Time color_stamp(header.stamp);
        const rclcpp::Time depth_stamp(depth_header.stamp);
        const double age = std::abs((color_stamp - depth_stamp).seconds());
        if (age > max_depth_age_s_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "D435 color/depth timestamps differ by %.3f s; metric output skipped", age);
            depth_m.release();
        }
    }

    auto detection_msg = processFrame(frame, depth_m, intrinsics, header);
    for (const auto& detection : detection_msg.detections) {
        Point3D point;
        if (!deprojectDetection(detection, depth_m, intrinsics, point)) continue;
        const double line_distance_m = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        response->distance_m.data = {point.z, line_distance_m};
        return;
    }
}

vision_msgs::msg::Detection2DArray YolosDetectorNode::processFrame(
    const cv::Mat& frame,
    const cv::Mat& depth_m,
    const CameraIntrinsics& intrinsics,
    const std_msgs::msg::Header& header) {
    vision_msgs::msg::Detection2DArray detection_msg;
    detection_msg.header = header;
    if (!detector_ || !detector_->isInitialized()) return detection_msg;

    const auto t_start = std::chrono::high_resolution_clock::now();
    try {
        const auto t_preprocess = std::chrono::high_resolution_clock::now();
        const auto detections = detector_->detect(frame, conf_threshold_, nms_threshold_);
        const auto t_inference = std::chrono::high_resolution_clock::now();
        detection_msg = conversion::toDetection2DArray(detections, header, frame.cols, frame.rows);
        if (det_pub_ && det_pub_->is_activated()) {
            det_pub_->publish(detection_msg);
        }
        const auto t_postprocess = std::chrono::high_resolution_clock::now();

        const int image_center_x = frame.cols / 2;
        const int image_center_y = frame.rows / 2;
        for (size_t index = 0; index < detections.size(); ++index) {
            const auto& det = detections[index];
            const int center_x = det.bbox.x + det.bbox.width / 2;
            const int center_y = det.bbox.y + det.bbox.height / 2;
            const int offset_x = center_x - image_center_x;
            const int offset_y = center_y - image_center_y;

            if (offset_pub_ && offset_pub_->is_activated()) {
                std_msgs::msg::Int32MultiArray offset_msg;
                offset_msg.data = {offset_x, offset_y};
                offset_pub_->publish(offset_msg);
            }

            Point3D point;
            const bool has_depth = use_depth_ && index < detection_msg.detections.size() &&
                deprojectDetection(detection_msg.detections[index], depth_m, intrinsics, point);
            if (!has_depth) {
                if (use_depth_) {
                    RCLCPP_INFO(
                        get_logger(), "det=[%s] center=(%d,%d) offset_px=(%d,%d) depth=invalid",
                        det.class_name.c_str(), center_x, center_y, offset_x, offset_y);
                } else {
                    RCLCPP_INFO(
                        get_logger(), "det=[%s] center=(%d,%d) offset_px=(%d,%d) RGB-only",
                        det.class_name.c_str(), center_x, center_y, offset_x, offset_y);
                }
                continue;
            }

            RCLCPP_INFO(
                get_logger(),
                "det=[%s] center=(%d,%d) point_m=(x:%.3f,y:%.3f,z:%.3f)",
                det.class_name.c_str(), center_x, center_y, point.x, point.y, point.z);

            if (offset_m_pub_ && offset_m_pub_->is_activated()) {
                std_msgs::msg::Float64MultiArray offset_m_msg;
                offset_m_msg.data = {point.x, point.y};
                offset_m_pub_->publish(offset_m_msg);
            }
            if (distance_pub_ && distance_pub_->is_activated()) {
                const double line_distance_m = std::sqrt(
                    point.x * point.x + point.y * point.y + point.z * point.z);
                std_msgs::msg::Float64MultiArray distance_msg;
                distance_msg.data = {point.z, line_distance_m};
                distance_pub_->publish(distance_msg);
            }
        }

        if (publish_timing_ && timing_pub_ && timing_pub_->is_activated()) {
            std_msgs::msg::Float64MultiArray timing;
            timing.data = {
                std::chrono::duration<double, std::milli>(t_preprocess - t_start).count(),
                std::chrono::duration<double, std::milli>(t_inference - t_preprocess).count(),
                std::chrono::duration<double, std::milli>(t_postprocess - t_inference).count()};
            timing_pub_->publish(timing);
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Detection failed: %s", e.what());
    }
    return detection_msg;
}

bool YolosDetectorNode::deprojectDetection(
    const vision_msgs::msg::Detection2D& detection,
    const cv::Mat& depth_m,
    const CameraIntrinsics& intrinsics,
    Point3D& point) const {
    if (depth_m.empty() || depth_m.type() != CV_32FC1 || !intrinsics.valid) return false;
    if (depth_m.cols != intrinsics.width || depth_m.rows != intrinsics.height) return false;

    const int center_x = static_cast<int>(std::lround(detection.bbox.center.position.x));
    const int center_y = static_cast<int>(std::lround(detection.bbox.center.position.y));
    const double z = sampleDepthMeters(depth_m, center_x, center_y);
    if (!std::isfinite(z)) return false;

    point.x = (static_cast<double>(center_x) - intrinsics.cx) * z / intrinsics.fx;
    point.y = (static_cast<double>(center_y) - intrinsics.cy) * z / intrinsics.fy;
    point.z = z;
    return true;
}

double YolosDetectorNode::sampleDepthMeters(const cv::Mat& depth_m, int center_x, int center_y) const {
    if (center_x < 0 || center_y < 0 || center_x >= depth_m.cols || center_y >= depth_m.rows) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    const int x_min = std::max(0, center_x - depth_sample_radius_px_);
    const int x_max = std::min(depth_m.cols - 1, center_x + depth_sample_radius_px_);
    const int y_min = std::max(0, center_y - depth_sample_radius_px_);
    const int y_max = std::min(depth_m.rows - 1, center_y + depth_sample_radius_px_);
    std::vector<float> valid_depths;
    valid_depths.reserve(static_cast<size_t>((x_max - x_min + 1) * (y_max - y_min + 1)));

    for (int y = y_min; y <= y_max; ++y) {
        for (int x = x_min; x <= x_max; ++x) {
            const float depth = depth_m.at<float>(y, x);
            if (std::isfinite(depth) && depth >= min_depth_m_ && depth <= max_depth_m_) {
                valid_depths.push_back(depth);
            }
        }
    }
    if (valid_depths.empty()) return std::numeric_limits<double>::quiet_NaN();

    const size_t middle = valid_depths.size() / 2;
    std::nth_element(valid_depths.begin(), valid_depths.begin() + middle, valid_depths.end());
    return static_cast<double>(valid_depths[middle]);
}

}  // namespace ros2_yolos_cpp

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_yolos_cpp::YolosDetectorNode)
