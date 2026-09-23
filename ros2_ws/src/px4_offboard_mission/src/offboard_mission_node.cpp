#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace
{
double yaw_from_pose(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & q = pose.pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}
}  // namespace

class OffboardMissionNode : public rclcpp::Node
{
public:
  OffboardMissionNode()
  : Node("px4_offboard_mission")
  {
    takeoff_height_m_ = declare_parameter<double>("takeoff_height_m", 1.4);
    takeoff_climb_rate_mps_ = declare_parameter<double>("takeoff_climb_rate_mps", 0.35);
    hover_time_s_ = declare_parameter<double>("hover_time_s", 3.0);
    left_distance_m_ = declare_parameter<double>("left_distance_m", 1.0);
    position_tolerance_m_ = declare_parameter<double>("position_tolerance_m", 0.15);
    setpoint_rate_hz_ = declare_parameter<double>("setpoint_rate_hz", 20.0);
    prestream_time_s_ = declare_parameter<double>("prestream_time_s", 3.0);
    pose_timeout_s_ = declare_parameter<double>("pose_timeout_s", 2.5);
    takeoff_timeout_s_ = declare_parameter<double>("takeoff_timeout_s", 25.0);
    move_timeout_s_ = declare_parameter<double>("move_timeout_s", 25.0);
    service_retry_period_s_ = declare_parameter<double>("service_retry_period_s", 1.0);
    land_mode_ = declare_parameter<std::string>("land_mode", "AUTO.LAND");
    setpoint_frame_id_ = declare_parameter<std::string>("setpoint_frame_id", "map");

    sanitize_parameters();

    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10,
      [this](mavros_msgs::msg::State::SharedPtr msg) {
        current_state_ = *msg;
        has_state_ = true;
      });

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        latest_pose_ = *msg;
        last_pose_time_ = get_clock()->now();
        has_pose_ = true;
      });

    setpoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/setpoint_position/local", 10);
    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    const auto now = get_clock()->now();
    phase_started_at_ = now;
    last_mode_request_time_ = now - rclcpp::Duration::from_seconds(service_retry_period_s_);
    last_pose_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

    const auto period = std::chrono::duration<double>(1.0 / setpoint_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&OffboardMissionNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "Mission ready: takeoff %.2f m, hover %.2f s, move left %.2f m, tolerance %.2f m.",
      takeoff_height_m_, hover_time_s_, left_distance_m_, position_tolerance_m_);
  }

private:
  enum class Phase
  {
    WAIT_CONNECTION,
    WAIT_POSE,
    PRESTREAM,
    SET_OFFBOARD,
    WAIT_ARM,
    TAKEOFF,
    HOVER,
    MOVE_LEFT,
    LAND,
    LANDING,
    FINISHED
  };

  void sanitize_parameters()
  {
    takeoff_height_m_ = std::max(0.1, takeoff_height_m_);
    takeoff_climb_rate_mps_ = std::max(0.1, takeoff_climb_rate_mps_);
    hover_time_s_ = std::max(0.0, hover_time_s_);
    left_distance_m_ = std::max(0.0, left_distance_m_);
    position_tolerance_m_ = std::max(0.03, position_tolerance_m_);
    setpoint_rate_hz_ = std::max(5.0, setpoint_rate_hz_);
    prestream_time_s_ = std::max(1.0, prestream_time_s_);
    pose_timeout_s_ = std::max(0.2, pose_timeout_s_);
    takeoff_timeout_s_ = std::max(5.0, takeoff_timeout_s_);
    move_timeout_s_ = std::max(5.0, move_timeout_s_);
    service_retry_period_s_ = std::max(0.2, service_retry_period_s_);
  }

  void on_timer()
  {
    const auto now = get_clock()->now();

    if (should_publish_setpoint()) {
      publish_target(now);
    }

    if (requires_fresh_pose() && !pose_is_fresh(now)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for fresh /mavros/local_position/pose before progressing.");
      return;
    }

    switch (phase_) {
      case Phase::WAIT_CONNECTION:
        wait_for_connection();
        break;
      case Phase::WAIT_POSE:
        wait_for_pose();
        break;
      case Phase::PRESTREAM:
        if (elapsed_s(now) >= prestream_time_s_) {
          transition_to(Phase::SET_OFFBOARD, "setpoint stream is warm");
        }
        break;
      case Phase::SET_OFFBOARD:
        if (current_state_.mode == "OFFBOARD") {
          transition_to(Phase::WAIT_ARM, "PX4 is in OFFBOARD; arm manually with RC");
        } else {
          request_mode("OFFBOARD");
        }
        break;
      case Phase::WAIT_ARM:
        if (current_state_.armed) {
          transition_to(Phase::TAKEOFF, "manual arm detected");
        } else {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "Waiting for manual arm from RC.");
        }
        break;
      case Phase::TAKEOFF:
        run_takeoff(now);
        break;
      case Phase::HOVER:
        run_hover(now);
        break;
      case Phase::MOVE_LEFT:
        run_move_left(now);
        break;
      case Phase::LAND:
        request_land();
        break;
      case Phase::LANDING:
        wait_for_disarm();
        break;
      case Phase::FINISHED:
        break;
    }
  }

  void wait_for_connection()
  {
    if (has_state_ && current_state_.connected) {
      transition_to(Phase::WAIT_POSE, "MAVROS connected to FCU");
      return;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Waiting for /mavros/state connected=true.");
  }

  void wait_for_pose()
  {
    if (!has_pose_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Waiting for /mavros/local_position/pose.");
      return;
    }

    start_pose_ = latest_pose_;
    const double yaw = yaw_from_pose(start_pose_);
    set_target(
      start_pose_.pose.position.x,
      start_pose_.pose.position.y,
      start_pose_.pose.position.z,
      yaw);

    RCLCPP_INFO(
      get_logger(),
      "Start pose xyz=(%.3f, %.3f, %.3f), target takeoff z=%.3f.",
      start_pose_.pose.position.x,
      start_pose_.pose.position.y,
      start_pose_.pose.position.z,
      takeoff_target_z());
    transition_to(Phase::PRESTREAM, "local pose is available");
  }

  void run_takeoff(const rclcpp::Time & now)
  {
    const double ramp_z = std::min(
      takeoff_target_z(),
      start_pose_.pose.position.z + takeoff_climb_rate_mps_ * elapsed_s(now));
    set_target(
      start_pose_.pose.position.x,
      start_pose_.pose.position.y,
      ramp_z,
      yaw_from_pose(start_pose_));

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Auto takeoff climb: current z=%.3f, command z=%.3f, final z=%.3f.",
      latest_pose_.pose.position.z,
      target_pose_.pose.position.z,
      takeoff_target_z());

    if (takeoff_target_reached()) {
      set_target(
        start_pose_.pose.position.x,
        start_pose_.pose.position.y,
        takeoff_target_z(),
        yaw_from_pose(start_pose_));
      transition_to(Phase::HOVER, "takeoff target reached");
      return;
    }

    if (elapsed_s(now) > takeoff_timeout_s_) {
      RCLCPP_ERROR(
        get_logger(),
        "Takeoff target was not reached within %.1f s; requesting land.",
        takeoff_timeout_s_);
      transition_to(Phase::LAND, "takeoff timeout");
    }
  }

  void run_hover(const rclcpp::Time & now)
  {
    if (elapsed_s(now) < hover_time_s_) {
      return;
    }

    const double yaw = yaw_from_pose(latest_pose_);
    const double left_x = -std::sin(yaw);
    const double left_y = std::cos(yaw);
    set_target(
      latest_pose_.pose.position.x + left_distance_m_ * left_x,
      latest_pose_.pose.position.y + left_distance_m_ * left_y,
      takeoff_target_z(),
      yaw);

    RCLCPP_INFO(
      get_logger(),
      "Moving left %.2f m: left_unit=(%.3f, %.3f), target xyz=(%.3f, %.3f, %.3f), yaw %.1f deg.",
      left_distance_m_,
      left_x,
      left_y,
      target_pose_.pose.position.x,
      target_pose_.pose.position.y,
      target_pose_.pose.position.z,
      yaw * 180.0 / M_PI);
    transition_to(Phase::MOVE_LEFT, "hover complete");
  }

  void run_move_left(const rclcpp::Time & now)
  {
    if (target_reached()) {
      transition_to(Phase::LAND, "left target reached");
      return;
    }

    if (elapsed_s(now) > move_timeout_s_) {
      RCLCPP_ERROR(
        get_logger(),
        "Left target was not reached within %.1f s; requesting land.",
        move_timeout_s_);
      transition_to(Phase::LAND, "left move timeout");
    }
  }

  void request_land()
  {
    if (current_state_.mode == land_mode_) {
      transition_to(Phase::LANDING, "land mode accepted");
      return;
    }

    request_mode(land_mode_);
  }

  void wait_for_disarm()
  {
    if (!current_state_.armed) {
      transition_to(Phase::FINISHED, "vehicle disarmed");
      RCLCPP_INFO(get_logger(), "Mission finished.");
      return;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Landing in progress; waiting for disarm.");
  }

  void set_target(double x, double y, double z, double yaw)
  {
    target_pose_.header.frame_id = setpoint_frame_id_;
    target_pose_.pose.position.x = x;
    target_pose_.pose.position.y = y;
    target_pose_.pose.position.z = z;
    target_pose_.pose.orientation = quaternion_from_yaw(yaw);
    has_target_ = true;
  }

  void publish_target(const rclcpp::Time & now)
  {
    if (!has_target_) {
      return;
    }

    target_pose_.header.stamp = now;
    setpoint_pub_->publish(target_pose_);
  }

  bool should_publish_setpoint() const
  {
    switch (phase_) {
      case Phase::PRESTREAM:
      case Phase::SET_OFFBOARD:
      case Phase::WAIT_ARM:
      case Phase::TAKEOFF:
      case Phase::HOVER:
      case Phase::MOVE_LEFT:
      case Phase::LAND:
        return has_target_;
      default:
        return false;
    }
  }

  bool requires_fresh_pose() const
  {
    switch (phase_) {
      case Phase::WAIT_POSE:
      case Phase::PRESTREAM:
      case Phase::SET_OFFBOARD:
      case Phase::WAIT_ARM:
      case Phase::TAKEOFF:
      case Phase::HOVER:
      case Phase::MOVE_LEFT:
        return true;
      default:
        return false;
    }
  }

  bool pose_is_fresh(const rclcpp::Time & now) const
  {
    if (!has_pose_) {
      return false;
    }
    return (now - last_pose_time_).seconds() <= pose_timeout_s_;
  }

  bool target_reached() const
  {
    if (!has_pose_ || !has_target_) {
      return false;
    }

    const double dx = latest_pose_.pose.position.x - target_pose_.pose.position.x;
    const double dy = latest_pose_.pose.position.y - target_pose_.pose.position.y;
    const double dz = latest_pose_.pose.position.z - target_pose_.pose.position.z;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    return distance <= position_tolerance_m_;
  }

  double takeoff_target_z() const
  {
    return start_pose_.pose.position.z + takeoff_height_m_;
  }

  bool takeoff_target_reached() const
  {
    if (!has_pose_) {
      return false;
    }

    const double dx = latest_pose_.pose.position.x - start_pose_.pose.position.x;
    const double dy = latest_pose_.pose.position.y - start_pose_.pose.position.y;
    const double dz = latest_pose_.pose.position.z - takeoff_target_z();
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    return distance <= position_tolerance_m_;
  }

  void request_mode(const std::string & mode)
  {
    const auto now = get_clock()->now();
    if (mode_request_pending_ ||
      (now - last_mode_request_time_).seconds() < service_retry_period_s_)
    {
      return;
    }

    if (!set_mode_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Waiting for /mavros/set_mode service.");
      return;
    }

    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->base_mode = 0;
    request->custom_mode = mode;

    mode_request_pending_ = true;
    last_mode_request_time_ = now;
    RCLCPP_INFO(get_logger(), "Requesting mode: %s", mode.c_str());
    set_mode_client_->async_send_request(
      request,
      [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
        mode_request_pending_ = false;
        const auto response = future.get();
        if (response->mode_sent) {
          RCLCPP_INFO(get_logger(), "Mode request sent: %s", mode.c_str());
        } else {
          RCLCPP_WARN(get_logger(), "Mode request was rejected before send: %s", mode.c_str());
        }
      });
  }

  void transition_to(Phase next, const std::string & reason)
  {
    phase_ = next;
    phase_started_at_ = get_clock()->now();
    RCLCPP_INFO(get_logger(), "Phase -> %s (%s).", phase_name(next), reason.c_str());
  }

  double elapsed_s(const rclcpp::Time & now) const
  {
    return (now - phase_started_at_).seconds();
  }

  const char * phase_name(Phase phase) const
  {
    switch (phase) {
      case Phase::WAIT_CONNECTION:
        return "WAIT_CONNECTION";
      case Phase::WAIT_POSE:
        return "WAIT_POSE";
      case Phase::PRESTREAM:
        return "PRESTREAM";
      case Phase::SET_OFFBOARD:
        return "SET_OFFBOARD";
      case Phase::WAIT_ARM:
        return "WAIT_ARM";
      case Phase::TAKEOFF:
        return "TAKEOFF";
      case Phase::HOVER:
        return "HOVER";
      case Phase::MOVE_LEFT:
        return "MOVE_LEFT";
      case Phase::LAND:
        return "LAND";
      case Phase::LANDING:
        return "LANDING";
      case Phase::FINISHED:
        return "FINISHED";
    }
    return "UNKNOWN";
  }

  double takeoff_height_m_{1.4};
  double takeoff_climb_rate_mps_{0.35};
  double hover_time_s_{3.0};
  double left_distance_m_{1.0};
  double position_tolerance_m_{0.15};
  double setpoint_rate_hz_{20.0};
  double prestream_time_s_{3.0};
  double pose_timeout_s_{2.5};
  double takeoff_timeout_s_{25.0};
  double move_timeout_s_{25.0};
  double service_retry_period_s_{1.0};
  std::string land_mode_{"AUTO.LAND"};
  std::string setpoint_frame_id_{"map"};

  Phase phase_{Phase::WAIT_CONNECTION};
  rclcpp::Time phase_started_at_;
  rclcpp::Time last_pose_time_;
  rclcpp::Time last_mode_request_time_;

  bool has_state_{false};
  bool has_pose_{false};
  bool has_target_{false};
  bool mode_request_pending_{false};

  mavros_msgs::msg::State current_state_;
  geometry_msgs::msg::PoseStamped latest_pose_;
  geometry_msgs::msg::PoseStamped start_pose_;
  geometry_msgs::msg::PoseStamped target_pose_;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardMissionNode>());
  rclcpp::shutdown();
  return 0;
}
