#include <chrono>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "wildlife_vision/msg/animal_report.hpp"
#include "wildlife_vision/msg/vision_trigger.hpp"

using namespace std::chrono_literals;

class WildlifeVisionTestNode : public rclcpp::Node
{
public:
  WildlifeVisionTestNode()
  : Node("wildlife_vision_test_node")
  {
    grid_ = declare_parameter<std::string>("grid", "A3B5");
    repeat_ = declare_parameter<bool>("repeat", false);
    publish_period_s_ = std::max(0.5, declare_parameter<double>("publish_period_s", 2.0));

    trigger_pub_ = create_publisher<wildlife_vision::msg::VisionTrigger>("/vision/trigger", 10);
    report_sub_ = create_subscription<wildlife_vision::msg::AnimalReport>(
      "/vision/animal_report", 10,
      std::bind(&WildlifeVisionTestNode::report_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(publish_period_s_)),
      std::bind(&WildlifeVisionTestNode::publish_trigger, this));

    startup_timer_ = create_wall_timer(500ms, [this]() {
      publish_trigger();
      startup_timer_->cancel();
    });

    RCLCPP_INFO(
      get_logger(), "wildlife_vision_test_node ready. grid=%s repeat=%s",
      grid_.c_str(), repeat_ ? "true" : "false");
  }

private:
  void publish_trigger()
  {
    if (published_once_ && !repeat_) {
      timer_->cancel();
      return;
    }

    wildlife_vision::msg::VisionTrigger trigger;
    trigger.grid = grid_;
    trigger.enable = true;
    trigger_pub_->publish(trigger);
    published_once_ = true;

    RCLCPP_INFO(get_logger(), "Published vision trigger for grid %s.", grid_.c_str());

    if (!repeat_) {
      timer_->cancel();
    }
  }

  void report_callback(const wildlife_vision::msg::AnimalReport::SharedPtr msg)
  {
    std::ostringstream animals;
    for (const auto & animal : msg->animals) {
      if (animals.tellp() > 0) {
        animals << ", ";
      }
      animals << animal.name << "=" << animal.count;
    }

    RCLCPP_INFO(
      get_logger(), "Received animal report: grid=%s success=%s animals=[%s] message=%s",
      msg->grid.c_str(), msg->success ? "true" : "false", animals.str().c_str(),
      msg->message.c_str());
  }

  std::string grid_{"A3B5"};
  bool repeat_{false};
  bool published_once_{false};
  double publish_period_s_{2.0};

  rclcpp::Publisher<wildlife_vision::msg::VisionTrigger>::SharedPtr trigger_pub_;
  rclcpp::Subscription<wildlife_vision::msg::AnimalReport>::SharedPtr report_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WildlifeVisionTestNode>());
  rclcpp::shutdown();
  return 0;
}
