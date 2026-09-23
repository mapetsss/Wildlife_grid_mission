#include <memory>

#include "mono_camera_capture/mono_camera_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mono_camera_capture::MonoCameraNode>());
  rclcpp::shutdown();
  return 0;
}
