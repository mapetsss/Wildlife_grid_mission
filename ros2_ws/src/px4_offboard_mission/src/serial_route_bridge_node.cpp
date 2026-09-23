#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cstring>
#include <exception>
#include <fcntl.h>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "wildlife_vision/msg/animal_report.hpp"

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

std::string join_tokens(const std::vector<std::string> & tokens)
{
  std::ostringstream out;
  for (std::size_t i = 0; i < tokens.size(); ++i) {
    if (i > 0) {
      out << ',';
    }
    out << tokens[i];
  }
  return out.str();
}

bool parse_grid_id(const std::string & token)
{
  if (token.size() < 4 || token.front() != 'A') {
    return false;
  }

  const auto b_pos = token.find('B');
  if (b_pos == std::string::npos || b_pos <= 1 || b_pos + 1 >= token.size()) {
    return false;
  }

  int a_index = 0;
  int b_index = 0;
  try {
    std::size_t consumed = 0;
    a_index = std::stoi(token.substr(1, b_pos - 1), &consumed);
    if (consumed != b_pos - 1) {
      return false;
    }
    consumed = 0;
    b_index = std::stoi(token.substr(b_pos + 1), &consumed);
    if (consumed != token.size() - b_pos - 1) {
      return false;
    }
  } catch (const std::exception &) {
    return false;
  }

  return a_index >= 1 && a_index <= 9 && b_index >= 1 && b_index <= 7;
}

std::string short_line(const std::string & line)
{
  constexpr std::size_t kMax = 100;
  if (line.size() <= kMax) {
    return line;
  }
  return line.substr(0, kMax) + "...";
}

speed_t baud_to_constant(int baud)
{
  switch (baud) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      return B115200;
  }
}
}  // namespace

class SerialRouteBridgeNode : public rclcpp::Node
{
public:
  SerialRouteBridgeNode()
  : Node("serial_route_bridge_node")
  {
    port_ = declare_parameter<std::string>("port", "/dev/serial0");
    fallback_ports_ = declare_parameter<std::vector<std::string>>(
      "fallback_ports",
      std::vector<std::string>{
        "/dev/serial/by-id/usb-muselab-tech.com_CMSIS-DAP-MuseLab_0001A0000002-if00",
        "/dev/ttyACM0"});
    baud_ = declare_parameter<int>("baud", 115200);
    start_after_route_ = declare_parameter<bool>("start_after_route", true);
    report_tx_enabled_ = declare_parameter<bool>("report_tx_enabled", true);
    send_summary_each_report_ = declare_parameter<bool>("send_summary_each_report", false);
    send_summary_on_done_ = declare_parameter<bool>("send_summary_on_done", true);
    send_empty_reports_ = declare_parameter<bool>("send_empty_reports", false);
    empty_animal_name_ = declare_parameter<std::string>("empty_animal_name", "none");
    report_topic_ = declare_parameter<std::string>("report_topic", "/vision/animal_report");
    mission_done_topic_ = declare_parameter<std::string>("mission_done_topic", "/mission/done");
    start_delay_s_ = std::max(0.0, declare_parameter<double>("start_delay_s", 0.2));
    retry_period_s_ = std::max(0.2, declare_parameter<double>("retry_period_s", 1.0));
    const int max_line_length = declare_parameter<int>("max_line_length", 4096);
    max_line_length_ = static_cast<std::size_t>(std::max(32, max_line_length));

    route_pub_ = create_publisher<std_msgs::msg::String>("/mission/route_upload", 10);
    start_client_ = create_client<std_srvs::srv::Trigger>("/mission/start");
    if (report_tx_enabled_) {
      animal_report_sub_ = create_subscription<wildlife_vision::msg::AnimalReport>(
        report_topic_, 10,
        std::bind(&SerialRouteBridgeNode::handle_animal_report, this, std::placeholders::_1));
      mission_done_sub_ = create_subscription<std_msgs::msg::Bool>(
        mission_done_topic_, 10,
        std::bind(&SerialRouteBridgeNode::handle_mission_done, this, std::placeholders::_1));
    }

    read_timer_ = create_wall_timer(20ms, std::bind(&SerialRouteBridgeNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "serial_route_bridge_node ready: port=%s baud=%d start_after_route=%s report_tx=%s",
      port_.c_str(), baud_, start_after_route_ ? "true" : "false",
      report_tx_enabled_ ? "true" : "false");
  }

  ~SerialRouteBridgeNode() override
  {
    close_port();
  }

private:
  void on_timer()
  {
    if (fd_ < 0) {
      try_open_port();
      maybe_call_start();
      return;
    }

    read_available();
    maybe_call_start();
  }

  void try_open_port()
  {
    const auto stamp = now();
    if (last_open_attempt_.nanoseconds() != 0 &&
      (stamp - last_open_attempt_).seconds() < retry_period_s_)
    {
      return;
    }
    last_open_attempt_ = stamp;

    std::string first_error;
    for (const auto & candidate : serial_port_candidates()) {
      const int fd = ::open(candidate.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
      if (fd < 0) {
        if (first_error.empty()) {
          first_error = candidate + ": " + std::strerror(errno);
        }
        continue;
      }

      active_port_ = candidate;
      if (!configure_port(fd)) {
        active_port_.clear();
        ::close(fd);
        return;
      }

      fd_ = fd;
      line_buffer_.clear();
      RCLCPP_INFO(get_logger(), "Opened serial route port %s at %d 8N1.", active_port_.c_str(), baud_);
      return;
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Waiting for serial route port. Tried %s. First error: %s",
      join_tokens(serial_port_candidates()).c_str(),
      first_error.empty() ? "no configured port candidates" : first_error.c_str());
  }

  std::vector<std::string> serial_port_candidates() const
  {
    std::vector<std::string> candidates;
    auto add_candidate = [&candidates](const std::string & value) {
        const auto cleaned = trim_copy(value);
        if (cleaned.empty()) {
          return;
        }
        if (std::find(candidates.begin(), candidates.end(), cleaned) == candidates.end()) {
          candidates.push_back(cleaned);
        }
      };

    add_candidate(port_);
    for (const auto & fallback : fallback_ports_) {
      add_candidate(fallback);
    }
    return candidates;
  }

  bool configure_port(int fd)
  {
    termios options;
    if (tcgetattr(fd, &options) != 0) {
      RCLCPP_ERROR(
        get_logger(), "tcgetattr failed for %s: %s", active_port_.c_str(), std::strerror(errno));
      return false;
    }

    cfmakeraw(&options);
    const speed_t speed = baud_to_constant(baud_);
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
#ifdef CRTSCTS
    options.c_cflag &= ~CRTSCTS;
#endif
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &options) != 0) {
      RCLCPP_ERROR(
        get_logger(), "tcsetattr failed for %s: %s", active_port_.c_str(), std::strerror(errno));
      return false;
    }
    tcflush(fd, TCIOFLUSH);
    return true;
  }

  void read_available()
  {
    char buffer[256];
    while (true) {
      const ssize_t count = ::read(fd_, buffer, sizeof(buffer));
      if (count > 0) {
        append_bytes(buffer, static_cast<std::size_t>(count));
        continue;
      }
      if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
        return;
      }

      RCLCPP_WARN(
        get_logger(), "Serial read failed on %s: %s. Reopening.",
        active_port_.c_str(), std::strerror(errno));
      close_port();
      return;
    }
  }

  void append_bytes(const char * data, std::size_t size)
  {
    for (std::size_t i = 0; i < size; ++i) {
      const char c = data[i];
      if (c == '\n') {
        process_line(line_buffer_);
        line_buffer_.clear();
        continue;
      }

      if (line_buffer_.size() >= max_line_length_) {
        RCLCPP_WARN(
          get_logger(), "Serial line exceeded %zu bytes; dropping buffered data.",
          max_line_length_);
        line_buffer_.clear();
        continue;
      }
      line_buffer_.push_back(c);
    }
  }

  void process_line(const std::string & raw_line)
  {
    std::string line = trim_copy(raw_line);
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
      line = trim_copy(line);
    }
    if (line.empty()) {
      return;
    }

    if (route_latched_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring serial route because the first route is already latched.");
      return;
    }

    std::string normalized_route;
    std::string error;
    if (!normalize_route(line, normalized_route, error)) {
      RCLCPP_WARN(
        get_logger(), "Rejected serial route line: %s; line=\"%s\"",
        error.c_str(), short_line(line).c_str());
      return;
    }

    std_msgs::msg::String msg;
    msg.data = normalized_route;
    route_pub_->publish(msg);
    route_latched_ = true;
    RCLCPP_INFO(get_logger(), "Published serial route: %s", normalized_route.c_str());

    if (start_after_route_) {
      start_due_time_ = now() + rclcpp::Duration::from_seconds(start_delay_s_);
      start_pending_ = true;
    }
  }

  bool normalize_route(
    const std::string & line,
    std::string & normalized_route,
    std::string & error) const
  {
    std::string payload = uppercase_copy(trim_copy(line));
    if (payload.rfind("ROUTE", 0) == 0) {
      if (payload.size() == 5 || !std::isspace(static_cast<unsigned char>(payload[5]))) {
        error = "invalid ROUTE prefix";
        return false;
      }
      payload = trim_copy(payload.substr(5));
    }

    const auto tokens = split_tokens(payload);
    if (tokens.empty()) {
      error = "empty route";
      return false;
    }

    for (const auto & token : tokens) {
      if (!parse_grid_id(token)) {
        error = "invalid waypoint: " + token;
        return false;
      }
    }

    normalized_route = join_tokens(tokens);
    return true;
  }

  void maybe_call_start()
  {
    if (!start_pending_ || start_request_in_flight_) {
      return;
    }
    if (now() < start_due_time_) {
      return;
    }
    if (!start_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Waiting for /mission/start service before starting uploaded route.");
      return;
    }

    start_request_in_flight_ = true;
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    start_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        start_request_in_flight_ = false;
        start_pending_ = false;
        const auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(get_logger(), "/mission/start accepted: %s", response->message.c_str());
        } else {
          RCLCPP_WARN(get_logger(), "/mission/start rejected: %s", response->message.c_str());
        }
      });
  }

  void handle_animal_report(const wildlife_vision::msg::AnimalReport::SharedPtr msg)
  {
    if (!report_tx_enabled_) {
      return;
    }
    if (fd_ < 0) {
      try_open_port();
    }
    if (fd_ < 0) {
      RCLCPP_WARN(
        get_logger(), "Drop animal report for %s because serial port is not open.",
        msg->grid.c_str());
      return;
    }
    if (!msg->success) {
      RCLCPP_WARN(
        get_logger(), "Skip failed animal report for %s: %s",
        msg->grid.c_str(), msg->message.c_str());
      return;
    }

    if (msg->animals.empty()) {
      if (send_empty_reports_) {
        write_serial_line("REPORT " + msg->grid + " " + empty_animal_name_ + " 0");
      }
    } else {
      for (const auto & animal : msg->animals) {
        animal_totals_[animal.name] += animal.count;
        write_serial_line(
          "REPORT " + msg->grid + " " + animal.name + " " + std::to_string(animal.count));
      }
    }

    if (send_summary_each_report_) {
      send_summary();
    }
  }

  void handle_mission_done(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!report_tx_enabled_ || !send_summary_on_done_) {
      return;
    }
    if (!msg->data) {
      RCLCPP_WARN(get_logger(), "Mission done reported failure; not sending animal SUMMARY.");
      return;
    }
    send_summary();
  }

  void send_summary()
  {
    if (animal_totals_.empty()) {
      RCLCPP_INFO(get_logger(), "No animal totals available; skip SUMMARY.");
      return;
    }

    std::ostringstream line;
    line << "SUMMARY";
    for (const auto & [name, count] : animal_totals_) {
      line << " " << name << " " << count;
    }
    write_serial_line(line.str());
  }

  void write_serial_line(const std::string & line)
  {
    if (fd_ < 0) {
      try_open_port();
    }
    if (fd_ < 0) {
      RCLCPP_WARN(get_logger(), "Cannot send serial line because port is not open: %s", line.c_str());
      return;
    }

    const std::string payload = line + "\n";
    const ssize_t written = ::write(fd_, payload.data(), payload.size());
    if (written < 0 || static_cast<std::size_t>(written) != payload.size()) {
      RCLCPP_WARN(
        get_logger(), "Serial write failed on %s: %s. Reopening.",
        active_port_.c_str(), std::strerror(errno));
      close_port();
      return;
    }
    RCLCPP_INFO(get_logger(), "Serial line sent: %s", line.c_str());
  }

  void close_port()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
    active_port_.clear();
  }

  std::string port_;
  std::string active_port_;
  std::vector<std::string> fallback_ports_;
  int baud_{115200};
  bool start_after_route_{true};
  bool report_tx_enabled_{true};
  bool send_summary_each_report_{false};
  bool send_summary_on_done_{true};
  bool send_empty_reports_{false};
  double start_delay_s_{0.2};
  double retry_period_s_{1.0};
  std::size_t max_line_length_{4096};
  bool route_latched_{false};
  std::string report_topic_;
  std::string mission_done_topic_;
  std::string empty_animal_name_;
  std::map<std::string, std::uint32_t> animal_totals_;

  int fd_{-1};
  std::string line_buffer_;
  rclcpp::Time last_open_attempt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time start_due_time_{0, 0, RCL_ROS_TIME};
  bool start_pending_{false};
  bool start_request_in_flight_{false};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr route_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;
  rclcpp::Subscription<wildlife_vision::msg::AnimalReport>::SharedPtr animal_report_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_done_sub_;
  rclcpp::TimerBase::SharedPtr read_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialRouteBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
