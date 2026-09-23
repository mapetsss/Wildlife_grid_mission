#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "wildlife_vision/msg/animal_report.hpp"
#include "wildlife_vision/msg/vision_trigger.hpp"

using namespace std::chrono_literals;

namespace
{
struct MissionPoint
{
  std::string grid_id;
  int a_index{0};
  int b_index{0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

double yaw_from_pose(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & q = pose.pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

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

std::string uppercase_copy(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) {return static_cast<char>(std::toupper(c));});
  return value;
}

std::vector<std::string> split_tokens(const std::string & text)
{
  std::vector<std::string> tokens;
  std::string token;
  for (const char c : text) {
    if (c == ',' || c == ';' || std::isspace(static_cast<unsigned char>(c)) != 0) {
      const auto cleaned = uppercase_copy(trim_copy(token));
      if (!cleaned.empty()) {
        tokens.push_back(cleaned);
      }
      token.clear();
    } else {
      token.push_back(c);
    }
  }
  const auto cleaned = uppercase_copy(trim_copy(token));
  if (!cleaned.empty()) {
    tokens.push_back(cleaned);
  }
  return tokens;
}

std::string json_escape(const std::string & input)
{
  std::ostringstream out;
  for (const char c : input) {
    switch (c) {
      case '"':
        out << "\\\"";
        break;
      case '\\':
        out << "\\\\";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << c;
        break;
    }
  }
  return out.str();
}
}  // namespace

class WildlifeMissionNode : public rclcpp::Node
{
public:
  WildlifeMissionNode()
  : Node("wildlife_mission_node")
  {
    load_parameters();
    sanitize_parameters();
    initialize_time_fields();

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
        last_pose_time_ = now();
        has_pose_ = true;
      });

    velocity_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/mavros/local_position/velocity_local", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        latest_velocity_ = *msg;
        has_velocity_ = true;
      });

    battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
      "/mavros/battery", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::BatteryState::SharedPtr msg) {
        latest_battery_ = *msg;
        has_battery_ = true;
      });

    route_upload_sub_ = create_subscription<std_msgs::msg::String>(
      "/mission/route_upload", 10,
      [this](std_msgs::msg::String::SharedPtr msg) {
        handle_route_upload(msg->data);
      });

    no_fly_sub_ = create_subscription<std_msgs::msg::String>(
      "/mission/no_fly_zones", 10,
      [this](std_msgs::msg::String::SharedPtr msg) {
        handle_no_fly_upload(msg->data);
      });

    animal_report_sub_ = create_subscription<wildlife_vision::msg::AnimalReport>(
      "/vision/animal_report", 10,
      [this](wildlife_vision::msg::AnimalReport::SharedPtr msg) {
        latest_animal_report_ = *msg;
        latest_animal_report_grid_ = msg->grid;
        if (waiting_for_vision_report_ && msg->grid == vision_hold_grid_) {
          has_vision_report_for_current_grid_ = true;
          waiting_for_vision_report_ = false;
          RCLCPP_INFO(
            get_logger(), "Vision report received for %s: success=%s animals=%zu message=%s",
            msg->grid.c_str(), msg->success ? "true" : "false", msg->animals.size(),
            msg->message.c_str());
        }
      });

    target_offset_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/vision/target_offset", 10,
      [this](geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        latest_target_offset_ = *msg;
        has_target_offset_ = true;
      });

    setpoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/setpoint_position/local", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/mission/status", 10);
    waypoint_reached_pub_ = create_publisher<std_msgs::msg::String>(
      "/mission/waypoint_reached", 10);
    done_pub_ = create_publisher<std_msgs::msg::Bool>("/mission/done", 10);
    vision_trigger_pub_ = create_publisher<wildlife_vision::msg::VisionTrigger>(
      "/vision/trigger", 10);

    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "/mission/start",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = start_mission();
        response->message = response->success ? "mission start accepted" : last_error_;
      });

    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "/mission/stop",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        stop_requested_ = true;
        response->success = true;
        response->message = "mission stop requested";
      });

    land_srv_ = create_service<std_srvs::srv::Trigger>(
      "/mission/land",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        land_requested_ = true;
        response->success = true;
        response->message = "land requested";
      });

    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    arm_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");

    parse_no_fly_list(no_fly_text_);

    const auto period = std::chrono::duration<double>(1.0 / setpoint_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&WildlifeMissionNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "wildlife_mission_node ready. Waiting for /mission/route_upload. target height %.2f m, auto_arm=%s.",
      target_height_m_, auto_arm_enabled_ ? "true" : "false");
  }

private:
  enum class Phase
  {
    IDLE,
    WAIT_FCU,
    OFFBOARD_PREPARE,
    ARM,
    TAKEOFF,
    FOLLOW_ROUTE,
    HOLD_AT_GRID,
    RETURN_HOME,
    LAND,
    LANDING,
    DONE,
    FAILSAFE
  };

  void load_parameters()
  {
    target_height_m_ = declare_parameter<double>("target_height_m", 1.2);
    grid_cell_size_m_ = declare_parameter<double>("grid_cell_size_m", 0.5);
    setpoint_rate_hz_ = declare_parameter<double>("setpoint_rate_hz", 20.0);
    status_rate_hz_ = declare_parameter<double>("status_rate_hz", 2.0);
    prestream_time_s_ = declare_parameter<double>("prestream_time_s", 2.5);
    waypoint_hold_time_s_ = declare_parameter<double>("waypoint_hold_time_s", 1.0);
    horizontal_tolerance_m_ = declare_parameter<double>("horizontal_tolerance_m", 0.10);
    vertical_tolerance_m_ = declare_parameter<double>("vertical_tolerance_m", 0.08);
    stable_time_s_ = declare_parameter<double>("stable_time_s", 0.4);
    pose_timeout_s_ = declare_parameter<double>("pose_timeout_s", 2.5);
    takeoff_timeout_s_ = declare_parameter<double>("takeoff_timeout_s", 25.0);
    waypoint_timeout_s_ = declare_parameter<double>("waypoint_timeout_s", 25.0);
    return_timeout_s_ = declare_parameter<double>("return_timeout_s", 25.0);
    vision_report_timeout_s_ = declare_parameter<double>("vision_report_timeout_s", 4.0);
    service_retry_period_s_ = declare_parameter<double>("service_retry_period_s", 1.0);
    offboard_retry_limit_ = declare_parameter<int>("offboard_retry_limit", 5);
    arm_retry_limit_ = declare_parameter<int>("arm_retry_limit", 3);
    auto_arm_enabled_ = declare_parameter<bool>("auto_arm_enabled", true);
    auto_start_ = declare_parameter<bool>("auto_start", false);
    failsafe_land_on_error_ = declare_parameter<bool>("failsafe_land_on_error", true);
    land_mode_ = declare_parameter<std::string>("land_mode", "AUTO.LAND");
    hold_mode_ = declare_parameter<std::string>("hold_mode", "AUTO.LOITER");
    setpoint_frame_id_ = declare_parameter<std::string>("setpoint_frame_id", "map");
    no_fly_text_ = declare_parameter<std::string>("no_fly_grids", "");
  }

  void sanitize_parameters()
  {
    target_height_m_ = std::clamp(target_height_m_, 0.2, 3.0);
    grid_cell_size_m_ = std::clamp(grid_cell_size_m_, 0.1, 2.0);
    setpoint_rate_hz_ = std::max(5.0, setpoint_rate_hz_);
    status_rate_hz_ = std::max(0.5, status_rate_hz_);
    prestream_time_s_ = std::max(2.0, prestream_time_s_);
    waypoint_hold_time_s_ = std::max(0.0, waypoint_hold_time_s_);
    horizontal_tolerance_m_ = std::clamp(horizontal_tolerance_m_, 0.03, 0.5);
    vertical_tolerance_m_ = std::clamp(vertical_tolerance_m_, 0.03, 0.5);
    stable_time_s_ = std::clamp(stable_time_s_, 0.1, 5.0);
    pose_timeout_s_ = std::max(0.2, pose_timeout_s_);
    takeoff_timeout_s_ = std::max(5.0, takeoff_timeout_s_);
    waypoint_timeout_s_ = std::max(5.0, waypoint_timeout_s_);
    return_timeout_s_ = std::max(5.0, return_timeout_s_);
    vision_report_timeout_s_ = std::max(0.5, vision_report_timeout_s_);
    service_retry_period_s_ = std::max(0.2, service_retry_period_s_);
    offboard_retry_limit_ = std::max(1, offboard_retry_limit_);
    arm_retry_limit_ = std::max(1, arm_retry_limit_);
  }

  void initialize_time_fields()
  {
    const auto clock_type = get_clock()->get_clock_type();
    phase_started_at_ = rclcpp::Time(0, 0, clock_type);
    last_pose_time_ = rclcpp::Time(0, 0, clock_type);
    last_status_time_ = rclcpp::Time(0, 0, clock_type);
    last_mode_request_time_ = rclcpp::Time(0, 0, clock_type);
    last_arm_request_time_ = rclcpp::Time(0, 0, clock_type);
    reached_since_ = rclcpp::Time(0, 0, clock_type);
    vision_trigger_time_ = rclcpp::Time(0, 0, clock_type);
  }

  void on_timer()
  {
    const auto stamp = now();

    if (auto_start_ && phase_ == Phase::IDLE && !auto_start_consumed_) {
      auto_start_consumed_ = true;
      start_mission();
    }

    if (stop_requested_ && mission_active()) {
      enter_failsafe("mission stop requested");
      stop_requested_ = false;
    }

    if (land_requested_ && mission_active()) {
      transition_to(Phase::LAND, "external land requested");
      land_requested_ = false;
    }

    if (should_publish_setpoint()) {
      publish_target(stamp);
    }

    if (requires_fresh_pose() && !pose_is_fresh(stamp)) {
      enter_failsafe("local position timeout");
    }

    switch (phase_) {
      case Phase::IDLE:
        break;
      case Phase::WAIT_FCU:
        run_wait_fcu();
        break;
      case Phase::OFFBOARD_PREPARE:
        run_offboard_prepare(stamp);
        break;
      case Phase::ARM:
        run_arm(stamp);
        break;
      case Phase::TAKEOFF:
        run_takeoff(stamp);
        break;
      case Phase::FOLLOW_ROUTE:
        run_follow_route(stamp);
        break;
      case Phase::HOLD_AT_GRID:
        run_hold_at_grid(stamp);
        break;
      case Phase::RETURN_HOME:
        run_return_home(stamp);
        break;
      case Phase::LAND:
        run_land();
        break;
      case Phase::LANDING:
        run_landing();
        break;
      case Phase::DONE:
        break;
      case Phase::FAILSAFE:
        run_failsafe();
        break;
    }

    publish_status_if_due(stamp);
  }

  bool start_mission()
  {
    if (phase_ != Phase::IDLE && phase_ != Phase::DONE) {
      last_error_ = "mission is already active";
      return false;
    }

    if (pending_route_.empty()) {
      last_error_ = "no valid ground station route uploaded";
      return false;
    }

    if (!route_is_allowed(pending_route_, last_error_)) {
      return false;
    }

    reset_mission_runtime();
    active_route_ = pending_route_;
    transition_to(Phase::WAIT_FCU, "mission start requested");
    return true;
  }

  void reset_mission_runtime()
  {
    route_index_ = 0;
    offboard_request_count_ = 0;
    arm_request_count_ = 0;
    mode_request_pending_ = false;
    arm_request_pending_ = false;
    reached_stable_ = false;
    current_grid_ = "";
    current_target_grid_ = "";
    last_error_.clear();
    has_target_ = false;
    mission_done_published_ = false;
    reset_vision_hold_state();
  }

  void run_wait_fcu()
  {
    if (!has_state_ || !current_state_.connected) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000, "Waiting for MAVROS FCU connection.");
      return;
    }
    if (!has_pose_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000, "Waiting for /mavros/local_position/pose.");
      return;
    }
    if (!pose_is_fresh(now())) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000, "Waiting for fresh /mavros/local_position/pose.");
      return;
    }

    home_pose_ = latest_pose_;
    locked_yaw_rad_ = yaw_from_pose(home_pose_);
    current_grid_ = "HOME/A9B1";
    resolve_route_targets();
    set_target(
      home_pose_.pose.position.x,
      home_pose_.pose.position.y,
      target_altitude(),
      "TAKEOFF");
    transition_to(Phase::OFFBOARD_PREPARE, "HOME/A9B1 and map yaw locked");
  }

  void run_offboard_prepare(const rclcpp::Time & stamp)
  {
    if (elapsed_s(stamp) < prestream_time_s_) {
      return;
    }
    if (current_state_.mode == "OFFBOARD") {
      transition_to(Phase::ARM, "OFFBOARD accepted");
      return;
    }
    if (offboard_request_count_ >= offboard_retry_limit_) {
      enter_failsafe("OFFBOARD mode retry limit exceeded");
      return;
    }
    if (request_mode("OFFBOARD")) {
      ++offboard_request_count_;
    }
  }

  void run_arm(const rclcpp::Time & stamp)
  {
    (void)stamp;
    if (current_state_.armed) {
      set_target(
        home_pose_.pose.position.x,
        home_pose_.pose.position.y,
        target_altitude(),
        "TAKEOFF");
      transition_to(Phase::TAKEOFF, "vehicle armed");
      return;
    }

    if (!auto_arm_enabled_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "auto_arm_enabled=false; waiting for manual arm.");
      return;
    }

    if (arm_request_count_ >= arm_retry_limit_) {
      enter_failsafe("arming retry limit exceeded");
      return;
    }

    if (request_arm(true)) {
      ++arm_request_count_;
    }
  }

  void run_takeoff(const rclcpp::Time & stamp)
  {
    if (target_reached_stably(stamp)) {
      transition_to(Phase::FOLLOW_ROUTE, "takeoff target reached");
      configure_current_route_target();
      return;
    }

    if (elapsed_s(stamp) > takeoff_timeout_s_) {
      enter_failsafe("takeoff timeout");
    }
  }

  void run_follow_route(const rclcpp::Time & stamp)
  {
    if (route_index_ >= active_route_.size()) {
      configure_return_home_target();
      transition_to(Phase::RETURN_HOME, "route complete");
      return;
    }

    if (target_reached_stably(stamp)) {
      current_grid_ = current_target_grid_;
      publish_waypoint_reached();
      transition_to(Phase::HOLD_AT_GRID, "waypoint reached");
      return;
    }

    if (elapsed_s(stamp) > waypoint_timeout_s_) {
      enter_failsafe("waypoint timeout at " + current_target_grid_);
    }
  }

  void run_hold_at_grid(const rclcpp::Time & stamp)
  {
    if (vision_hold_grid_ != current_grid_) {
      vision_hold_grid_ = current_grid_;
      vision_trigger_sent_ = false;
      waiting_for_vision_report_ = false;
      has_vision_report_for_current_grid_ = false;
      vision_trigger_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    }

    if (!vision_trigger_sent_) {
      publish_vision_trigger(current_grid_);
      vision_trigger_sent_ = true;
      waiting_for_vision_report_ = true;
      has_vision_report_for_current_grid_ = false;
      vision_trigger_time_ = stamp;
    }

    if (elapsed_s(stamp) < waypoint_hold_time_s_) {
      return;
    }

    if (!has_vision_report_for_current_grid_) {
      const double vision_elapsed_s = (stamp - vision_trigger_time_).seconds();
      if (vision_elapsed_s < vision_report_timeout_s_) {
        return;
      }
      if (waiting_for_vision_report_) {
        RCLCPP_WARN(
          get_logger(), "Vision report timeout at %s after %.1f s; continue route.",
          current_grid_.c_str(), vision_report_timeout_s_);
        waiting_for_vision_report_ = false;
      }
    }

    ++route_index_;
    reset_vision_hold_state();
    if (route_index_ >= active_route_.size()) {
      configure_return_home_target();
      transition_to(Phase::RETURN_HOME, "all route waypoints visited");
    } else {
      configure_current_route_target();
      transition_to(Phase::FOLLOW_ROUTE, "next waypoint");
    }
  }

  void publish_vision_trigger(const std::string & grid)
  {
    wildlife_vision::msg::VisionTrigger msg;
    msg.grid = grid;
    msg.enable = true;
    vision_trigger_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Vision trigger published for %s.", grid.c_str());
  }

  void reset_vision_hold_state()
  {
    vision_hold_grid_.clear();
    vision_trigger_sent_ = false;
    waiting_for_vision_report_ = false;
    has_vision_report_for_current_grid_ = false;
    vision_trigger_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }

  void run_return_home(const rclcpp::Time & stamp)
  {
    if (target_reached_stably(stamp)) {
      transition_to(Phase::LAND, "home reached");
      return;
    }
    if (elapsed_s(stamp) > return_timeout_s_) {
      enter_failsafe("return home timeout");
    }
  }

  void run_land()
  {
    if (current_state_.mode == land_mode_) {
      transition_to(Phase::LANDING, "land mode accepted");
      return;
    }
    request_mode(land_mode_);
  }

  void run_landing()
  {
    if (has_state_ && !current_state_.armed) {
      transition_to(Phase::DONE, "vehicle disarmed");
      publish_done(true);
      return;
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "Landing; waiting for disarm.");
  }

  void run_failsafe()
  {
    if (failsafe_land_on_error_) {
      request_mode(land_mode_);
      return;
    }
    request_mode(hold_mode_);
  }

  void configure_current_route_target()
  {
    if (route_index_ >= active_route_.size()) {
      return;
    }
    const auto & waypoint = active_route_[route_index_];
    set_target(waypoint.x, waypoint.y, waypoint.z, waypoint.grid_id);
  }

  void configure_return_home_target()
  {
    set_target(
      home_pose_.pose.position.x,
      home_pose_.pose.position.y,
      target_altitude(),
      "HOME/A9B1");
  }

  void set_target(double x, double y, double z, const std::string & grid_id)
  {
    target_pose_.header.frame_id = setpoint_frame_id_;
    target_pose_.pose.position.x = x;
    target_pose_.pose.position.y = y;
    target_pose_.pose.position.z = z;
    target_pose_.pose.orientation = quaternion_from_yaw(locked_yaw_rad_);
    has_target_ = true;
    current_target_grid_ = grid_id;
    reached_stable_ = false;
    reached_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }

  void publish_target(const rclcpp::Time & stamp)
  {
    if (!has_target_) {
      return;
    }
    target_pose_.header.stamp = stamp;
    setpoint_pub_->publish(target_pose_);
  }

  bool should_publish_setpoint() const
  {
    switch (phase_) {
      case Phase::OFFBOARD_PREPARE:
      case Phase::ARM:
      case Phase::TAKEOFF:
      case Phase::FOLLOW_ROUTE:
      case Phase::HOLD_AT_GRID:
      case Phase::RETURN_HOME:
      case Phase::LAND:
      case Phase::FAILSAFE:
        return has_target_;
      default:
        return false;
    }
  }

  bool requires_fresh_pose() const
  {
    switch (phase_) {
      case Phase::OFFBOARD_PREPARE:
      case Phase::ARM:
      case Phase::TAKEOFF:
      case Phase::FOLLOW_ROUTE:
      case Phase::HOLD_AT_GRID:
      case Phase::RETURN_HOME:
        return true;
      default:
        return false;
    }
  }

  bool mission_active() const
  {
    return phase_ != Phase::IDLE && phase_ != Phase::DONE && phase_ != Phase::FAILSAFE;
  }

  bool pose_is_fresh(const rclcpp::Time & stamp) const
  {
    return has_pose_ && (stamp - last_pose_time_).seconds() <= pose_timeout_s_;
  }

  bool target_reached_stably(const rclcpp::Time & stamp)
  {
    if (!target_reached()) {
      reached_stable_ = false;
      reached_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      return false;
    }
    if (!reached_stable_) {
      reached_stable_ = true;
      reached_since_ = stamp;
      return false;
    }
    return (stamp - reached_since_).seconds() >= stable_time_s_;
  }

  bool target_reached() const
  {
    if (!has_pose_ || !has_target_) {
      return false;
    }
    const double dx = latest_pose_.pose.position.x - target_pose_.pose.position.x;
    const double dy = latest_pose_.pose.position.y - target_pose_.pose.position.y;
    const double dz = latest_pose_.pose.position.z - target_pose_.pose.position.z;
    const double horizontal_error = std::sqrt(dx * dx + dy * dy);
    return horizontal_error <= horizontal_tolerance_m_ && std::fabs(dz) <= vertical_tolerance_m_;
  }

  double target_altitude() const
  {
    return home_pose_.pose.position.z + target_height_m_;
  }

  bool request_mode(const std::string & mode)
  {
    const auto stamp = now();
    if (mode_request_pending_ ||
      (stamp - last_mode_request_time_).seconds() < service_retry_period_s_)
    {
      return false;
    }
    if (!set_mode_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "Waiting for /mavros/set_mode service.");
      return false;
    }

    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->base_mode = 0;
    request->custom_mode = mode;
    mode_request_pending_ = true;
    last_mode_request_time_ = stamp;
    RCLCPP_INFO(get_logger(), "Requesting mode: %s", mode.c_str());
    set_mode_client_->async_send_request(
      request,
      [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
        mode_request_pending_ = false;
        const auto response = future.get();
        if (response->mode_sent) {
          RCLCPP_INFO(get_logger(), "Mode request sent: %s", mode.c_str());
        } else {
          RCLCPP_WARN(get_logger(), "Mode request rejected before send: %s", mode.c_str());
        }
      });
    return true;
  }

  bool request_arm(bool arm)
  {
    const auto stamp = now();
    if (arm_request_pending_ ||
      (stamp - last_arm_request_time_).seconds() < service_retry_period_s_)
    {
      return false;
    }
    if (!arm_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "Waiting for /mavros/cmd/arming service.");
      return false;
    }

    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = arm;
    arm_request_pending_ = true;
    last_arm_request_time_ = stamp;
    RCLCPP_INFO(get_logger(), "Requesting arm=%s.", arm ? "true" : "false");
    arm_client_->async_send_request(
      request,
      [this, arm](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
        arm_request_pending_ = false;
        const auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(get_logger(), "Arm request accepted: %s", arm ? "true" : "false");
        } else {
          RCLCPP_WARN(get_logger(), "Arm request failed: %s", arm ? "true" : "false");
        }
      });
    return true;
  }

  bool parse_grid_id(const std::string & input, int & a_index, int & b_index) const
  {
    const std::string grid = uppercase_copy(trim_copy(input));
    const auto b_pos = grid.find('B');
    if (grid.size() < 4 || grid.front() != 'A' || b_pos == std::string::npos || b_pos <= 1) {
      return false;
    }

    try {
      a_index = std::stoi(grid.substr(1, b_pos - 1));
      b_index = std::stoi(grid.substr(b_pos + 1));
    } catch (const std::exception &) {
      return false;
    }

    return a_index >= 1 && a_index <= 9 && b_index >= 1 && b_index <= 7;
  }

  bool grid_to_point(const std::string & grid_id, MissionPoint & point, std::string & error) const
  {
    int a_index = 0;
    int b_index = 0;
    if (!parse_grid_id(grid_id, a_index, b_index)) {
      error = "invalid grid id: " + grid_id;
      return false;
    }

    point.grid_id = uppercase_copy(trim_copy(grid_id));
    point.a_index = a_index;
    point.b_index = b_index;
    return true;
  }

  void resolve_route_targets()
  {
    const double cos_yaw = std::cos(locked_yaw_rad_);
    const double sin_yaw = std::sin(locked_yaw_rad_);
    for (auto & point : active_route_) {
      const double forward = (9.0 - static_cast<double>(point.a_index)) * grid_cell_size_m_;
      const double right = (static_cast<double>(point.b_index) - 1.0) * grid_cell_size_m_;
      const double left = -right;
      point.x = home_pose_.pose.position.x + cos_yaw * forward - sin_yaw * left;
      point.y = home_pose_.pose.position.y + sin_yaw * forward + cos_yaw * left;
      point.z = target_altitude();
    }
  }

  bool parse_route(
    const std::string & text,
    std::vector<MissionPoint> & route,
    std::string & error) const
  {
    route.clear();
    for (const auto & token : split_tokens(text)) {
      if (token == "HOME") {
        continue;
      }
      MissionPoint point;
      if (!grid_to_point(token, point, error)) {
        return false;
      }
      route.push_back(point);
    }

    if (route.empty()) {
      error = "route has no grid waypoints";
      return false;
    }
    return true;
  }

  void parse_no_fly_list(const std::string & text)
  {
    no_fly_grids_.clear();
    for (const auto & token : split_tokens(text)) {
      if (token == "HOME") {
        continue;
      }
      int a_index = 0;
      int b_index = 0;
      if (parse_grid_id(token, a_index, b_index)) {
        no_fly_grids_.push_back(token);
      } else {
        RCLCPP_WARN(get_logger(), "Ignoring invalid no-fly grid: %s", token.c_str());
      }
    }
  }

  bool route_is_allowed(const std::vector<MissionPoint> & route, std::string & error) const
  {
    for (const auto & waypoint : route) {
      if (std::find(no_fly_grids_.begin(), no_fly_grids_.end(), waypoint.grid_id) !=
        no_fly_grids_.end())
      {
        error = "route contains no-fly grid: " + waypoint.grid_id;
        return false;
      }
    }
    return true;
  }

  void handle_route_upload(const std::string & text)
  {
    if (route_upload_locked_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring route upload because the first route is already latched.");
      return;
    }

    if (phase_ != Phase::IDLE && phase_ != Phase::DONE) {
      last_error_ = "route upload rejected: mission is active";
      RCLCPP_WARN(get_logger(), "%s", last_error_.c_str());
      return;
    }

    std::vector<MissionPoint> uploaded_route;
    std::string error;
    if (!parse_route(text, uploaded_route, error)) {
      last_error_ = "route upload rejected: " + error;
      RCLCPP_WARN(get_logger(), "%s", last_error_.c_str());
      return;
    }
    if (!route_is_allowed(uploaded_route, error)) {
      last_error_ = "route upload rejected: " + error;
      RCLCPP_WARN(get_logger(), "%s", last_error_.c_str());
      return;
    }
    pending_route_ = uploaded_route;
    route_upload_locked_ = true;
    RCLCPP_INFO(get_logger(), "Route uploaded with %zu grid waypoints.", pending_route_.size());
  }

  void handle_no_fly_upload(const std::string & text)
  {
    parse_no_fly_list(text);
    RCLCPP_INFO(get_logger(), "No-fly grid list updated with %zu entries.", no_fly_grids_.size());
  }

  void enter_failsafe(const std::string & reason)
  {
    if (phase_ == Phase::FAILSAFE || phase_ == Phase::DONE) {
      return;
    }
    last_error_ = reason;
    if (has_pose_) {
      set_target(
        latest_pose_.pose.position.x,
        latest_pose_.pose.position.y,
        latest_pose_.pose.position.z,
        "FAILSAFE");
    }
    RCLCPP_ERROR(get_logger(), "FAILSAFE: %s", reason.c_str());
    transition_to(Phase::FAILSAFE, reason);
    publish_done(false);
  }

  void transition_to(Phase next, const std::string & reason)
  {
    phase_ = next;
    phase_started_at_ = now();
    reached_stable_ = false;
    reached_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    RCLCPP_INFO(get_logger(), "State -> %s (%s).", phase_name(next), reason.c_str());
  }

  double elapsed_s(const rclcpp::Time & stamp) const
  {
    return (stamp - phase_started_at_).seconds();
  }

  const char * phase_name(Phase phase) const
  {
    switch (phase) {
      case Phase::IDLE:
        return "IDLE";
      case Phase::WAIT_FCU:
        return "WAIT_FCU";
      case Phase::OFFBOARD_PREPARE:
        return "OFFBOARD_PREPARE";
      case Phase::ARM:
        return "ARM";
      case Phase::TAKEOFF:
        return "TAKEOFF";
      case Phase::FOLLOW_ROUTE:
        return "FOLLOW_ROUTE";
      case Phase::HOLD_AT_GRID:
        return "HOLD_AT_GRID";
      case Phase::RETURN_HOME:
        return "RETURN_HOME";
      case Phase::LAND:
        return "LAND";
      case Phase::LANDING:
        return "LANDING";
      case Phase::DONE:
        return "DONE";
      case Phase::FAILSAFE:
        return "FAILSAFE";
    }
    return "UNKNOWN";
  }

  void publish_waypoint_reached()
  {
    std_msgs::msg::String msg;
    std::ostringstream out;
    out << "{\"grid\":\"" << json_escape(current_grid_) << "\",\"index\":" << route_index_
        << ",\"x\":" << latest_pose_.pose.position.x
        << ",\"y\":" << latest_pose_.pose.position.y
        << ",\"z\":" << latest_pose_.pose.position.z << "}";
    msg.data = out.str();
    waypoint_reached_pub_->publish(msg);
  }

  void publish_done(bool success)
  {
    if (mission_done_published_) {
      return;
    }
    std_msgs::msg::Bool msg;
    msg.data = success;
    done_pub_->publish(msg);
    mission_done_published_ = true;
  }

  void publish_status_if_due(const rclcpp::Time & stamp)
  {
    if ((stamp - last_status_time_).seconds() < 1.0 / status_rate_hz_) {
      return;
    }
    last_status_time_ = stamp;

    std_msgs::msg::String msg;
    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    out << "{";
    out << "\"state\":\"" << phase_name(phase_) << "\",";
    out << "\"current_grid\":\"" << json_escape(current_grid_) << "\",";
    out << "\"target_grid\":\"" << json_escape(current_target_grid_) << "\",";
    out << "\"waypoint_index\":" << route_index_ << ",";
    out << "\"waypoint_count\":" <<
      (active_route_.empty() ? pending_route_.size() : active_route_.size()) << ",";
    out << "\"current_x\":" << (has_pose_ ? latest_pose_.pose.position.x : 0.0) << ",";
    out << "\"current_y\":" << (has_pose_ ? latest_pose_.pose.position.y : 0.0) << ",";
    out << "\"current_z\":" << (has_pose_ ? latest_pose_.pose.position.z : 0.0) << ",";
    out << "\"target_x\":" << (has_target_ ? target_pose_.pose.position.x : 0.0) << ",";
    out << "\"target_y\":" << (has_target_ ? target_pose_.pose.position.y : 0.0) << ",";
    out << "\"target_z\":" << (has_target_ ? target_pose_.pose.position.z : 0.0) << ",";
    out << "\"armed\":" << (current_state_.armed ? "true" : "false") << ",";
    out << "\"mode\":\"" << json_escape(current_state_.mode) << "\",";
    out << "\"connected\":" << (current_state_.connected ? "true" : "false") << ",";
    out << "\"waypoint_reached\":" << (target_reached() ? "true" : "false") << ",";
    out << "\"error\":\"" << json_escape(last_error_) << "\",";
    out << "\"battery_percent\":" << (has_battery_ ? latest_battery_.percentage : -1.0);
    out << "}";
    msg.data = out.str();
    status_pub_->publish(msg);
  }

  double target_height_m_{1.2};
  double grid_cell_size_m_{0.5};
  double setpoint_rate_hz_{20.0};
  double status_rate_hz_{2.0};
  double prestream_time_s_{2.5};
  double waypoint_hold_time_s_{1.0};
  double horizontal_tolerance_m_{0.10};
  double vertical_tolerance_m_{0.08};
  double stable_time_s_{0.4};
  double pose_timeout_s_{2.5};
  double takeoff_timeout_s_{25.0};
  double waypoint_timeout_s_{25.0};
  double return_timeout_s_{25.0};
  double vision_report_timeout_s_{4.0};
  double service_retry_period_s_{1.0};
  int offboard_retry_limit_{5};
  int arm_retry_limit_{3};
  bool auto_arm_enabled_{true};
  bool auto_start_{false};
  bool auto_start_consumed_{false};
  bool failsafe_land_on_error_{true};
  std::string land_mode_{"AUTO.LAND"};
  std::string hold_mode_{"AUTO.LOITER"};
  std::string setpoint_frame_id_{"map"};
  std::string no_fly_text_;

  Phase phase_{Phase::IDLE};
  rclcpp::Time phase_started_at_{0, 0, RCL_SYSTEM_TIME};
  rclcpp::Time last_pose_time_{0, 0, RCL_SYSTEM_TIME};
  rclcpp::Time last_status_time_{0, 0, RCL_SYSTEM_TIME};
  rclcpp::Time last_mode_request_time_{0, 0, RCL_SYSTEM_TIME};
  rclcpp::Time last_arm_request_time_{0, 0, RCL_SYSTEM_TIME};
  rclcpp::Time reached_since_{0, 0, RCL_SYSTEM_TIME};
  rclcpp::Time vision_trigger_time_{0, 0, RCL_SYSTEM_TIME};

  bool has_state_{false};
  bool has_pose_{false};
  bool has_velocity_{false};
  bool has_battery_{false};
  bool has_target_{false};
  bool has_target_offset_{false};
  bool mode_request_pending_{false};
  bool arm_request_pending_{false};
  bool reached_stable_{false};
  bool stop_requested_{false};
  bool land_requested_{false};
  bool mission_done_published_{false};
  bool route_upload_locked_{false};
  bool vision_trigger_sent_{false};
  bool waiting_for_vision_report_{false};
  bool has_vision_report_for_current_grid_{false};
  int offboard_request_count_{0};
  int arm_request_count_{0};
  size_t route_index_{0};
  double locked_yaw_rad_{0.0};

  std::string current_grid_;
  std::string current_target_grid_;
  std::string last_error_;
  std::string vision_hold_grid_;
  std::string latest_animal_report_grid_;
  std::vector<MissionPoint> pending_route_;
  std::vector<MissionPoint> active_route_;
  std::vector<std::string> no_fly_grids_;

  mavros_msgs::msg::State current_state_;
  geometry_msgs::msg::PoseStamped latest_pose_;
  geometry_msgs::msg::PoseStamped home_pose_;
  geometry_msgs::msg::PoseStamped target_pose_;
  geometry_msgs::msg::TwistStamped latest_velocity_;
  geometry_msgs::msg::Vector3Stamped latest_target_offset_;
  sensor_msgs::msg::BatteryState latest_battery_;
  wildlife_vision::msg::AnimalReport latest_animal_report_;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr route_upload_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr no_fly_sub_;
  rclcpp::Subscription<wildlife_vision::msg::AnimalReport>::SharedPtr animal_report_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr target_offset_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr waypoint_reached_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub_;
  rclcpp::Publisher<wildlife_vision::msg::VisionTrigger>::SharedPtr vision_trigger_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_srv_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WildlifeMissionNode>());
  rclcpp::shutdown();
  return 0;
}
