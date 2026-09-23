#ifndef MONO_CAMERA_CAPTURE__MONO_CAMERA_NODE_HPP_
#define MONO_CAMERA_CAPTURE__MONO_CAMERA_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <librealsense2/rs.hpp>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace mono_camera_capture
{

// The package and executable names are kept for backwards compatibility.  The
// implementation is a native librealsense2 driver for Intel RealSense D435.
class MonoCameraNode : public rclcpp::Node
{
public:
  explicit MonoCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MonoCameraNode() override;

private:
  struct CameraConfig
  {
    std::string serial_no;
    std::string color_frame_id{"camera_color_optical_frame"};
    std::string depth_frame_id{"camera_aligned_depth_to_color_frame"};
    int color_width{640};
    int color_height{480};
    int color_fps{30};
    int depth_width{640};
    int depth_height{480};
    int depth_fps{30};
    bool enable_color{true};
    bool enable_depth{true};
    bool align_depth_to_color{true};
    bool enable_emitter{true};
    double laser_power{-1.0};
    bool publish_camera_info{true};
    bool log_fps{true};
    bool reconnect_on_failure{true};
    int reconnect_period_ms{1000};
    int frame_timeout_ms{1000};
  };

  void declareParameters();
  CameraConfig loadConfigFromParameters() const;
  bool startPipelineLocked();
  void stopPipelineLocked();
  void configureDepthSensor(const rs2::device & device, const CameraConfig & config);
  void captureOnce();
  void publishColorFrame(
    const rs2::video_frame & frame,
    const rclcpp::Time & stamp,
    const CameraConfig & config);
  void publishDepthFrame(
    const rs2::depth_frame & frame,
    const rclcpp::Time & stamp,
    const CameraConfig & config);
  sensor_msgs::msg::CameraInfo makeCameraInfo(
    const rs2::video_stream_profile & profile,
    const rclcpp::Time & stamp,
    const std::string & frame_id) const;
  void recreateCaptureTimer(double fps);
  void updateFpsStats(const rclcpp::Time & stamp, bool log_fps);
  bool validateConfig(const CameraConfig & config, std::string & reason) const;

  rcl_interfaces::msg::SetParametersResult onParameters(
    const std::vector<rclcpp::Parameter> & parameters);

  CameraConfig config_;
  std::mutex pipeline_mutex_;
  std::unique_ptr<rs2::pipeline> pipeline_;
  std::unique_ptr<rs2::align> align_to_color_;
  bool pipeline_started_{false};
  float depth_scale_m_{0.001F};

  rclcpp::TimerBase::SharedPtr capture_timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  std::atomic<bool> restarting_{false};
  rclcpp::Time last_reconnect_attempt_;
  rclcpp::Time fps_window_start_;
  int fps_frame_count_{0};
};

}  // namespace mono_camera_capture

#endif  // MONO_CAMERA_CAPTURE__MONO_CAMERA_NODE_HPP_
