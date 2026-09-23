#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <map>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "wildlife_vision/msg/animal_report.hpp"

using namespace std::chrono_literals;

namespace
{
speed_t baud_to_constant(const int baud)
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

std::string join_paths(const std::vector<std::string> & paths)
{
  std::ostringstream out;
  for (const auto & path : paths) {
    if (out.tellp() > 0) {
      out << ", ";
    }
    out << path;
  }
  return out.str();
}
}  // namespace

class WildlifeReportSerialNode : public rclcpp::Node
{
public:
  WildlifeReportSerialNode()
  : Node("wildlife_report_serial_node")
  {
    port_ = declare_parameter<std::string>("port", "/dev/serial0");
    fallback_ports_ = declare_parameter<std::vector<std::string>>(
      "fallback_ports",
      std::vector<std::string>{
        "/dev/serial/by-id/usb-muselab-tech.com_CMSIS-DAP-MuseLab_0001A0000002-if00",
        "/dev/ttyACM0"});
    baud_ = declare_parameter<int>("baud", 115200);
    report_topic_ = declare_parameter<std::string>("report_topic", "/vision/animal_report");
    send_summary_each_report_ = declare_parameter<bool>("send_summary_each_report", false);
    send_empty_reports_ = declare_parameter<bool>("send_empty_reports", false);
    empty_animal_name_ = declare_parameter<std::string>("empty_animal_name", "none");

    report_sub_ = create_subscription<wildlife_vision::msg::AnimalReport>(
      report_topic_, 10,
      std::bind(&WildlifeReportSerialNode::report_callback, this, std::placeholders::_1));

    retry_timer_ = create_wall_timer(1s, std::bind(&WildlifeReportSerialNode::ensure_port, this));

    RCLCPP_INFO(
      get_logger(), "wildlife_report_serial_node ready: topic=%s port=%s baud=%d",
      report_topic_.c_str(), port_.c_str(), baud_);
  }

  ~WildlifeReportSerialNode() override
  {
    close_port();
  }

private:
  void ensure_port()
  {
    if (fd_ >= 0) {
      return;
    }

    for (const auto & candidate : serial_port_candidates()) {
      const int fd = ::open(candidate.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
      if (fd < 0) {
        first_open_error_ = std::strerror(errno);
        continue;
      }

      if (!configure_port(fd)) {
        ::close(fd);
        continue;
      }

      fd_ = fd;
      active_port_ = candidate;
      RCLCPP_INFO(get_logger(), "Opened report serial port %s at %d 8N1.", active_port_.c_str(), baud_);
      return;
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Waiting for report serial port. Tried %s. First error: %s",
      join_paths(serial_port_candidates()).c_str(), first_open_error_.c_str());
  }

  std::vector<std::string> serial_port_candidates() const
  {
    std::vector<std::string> ports;
    if (!port_.empty()) {
      ports.push_back(port_);
    }
    for (const auto & fallback : fallback_ports_) {
      if (!fallback.empty() && std::find(ports.begin(), ports.end(), fallback) == ports.end()) {
        ports.push_back(fallback);
      }
    }
    return ports;
  }

  bool configure_port(const int fd)
  {
    termios attrs{};
    if (tcgetattr(fd, &attrs) != 0) {
      first_open_error_ = std::strerror(errno);
      return false;
    }

    cfmakeraw(&attrs);
    attrs.c_cflag |= CLOCAL | CREAD;
    attrs.c_cflag &= ~PARENB;
    attrs.c_cflag &= ~CSTOPB;
    attrs.c_cflag &= ~CSIZE;
    attrs.c_cflag |= CS8;

    const speed_t baud = baud_to_constant(baud_);
    cfsetispeed(&attrs, baud);
    cfsetospeed(&attrs, baud);

    if (tcsetattr(fd, TCSANOW, &attrs) != 0) {
      first_open_error_ = std::strerror(errno);
      return false;
    }
    return true;
  }

  void report_callback(const wildlife_vision::msg::AnimalReport::SharedPtr msg)
  {
    if (fd_ < 0) {
      ensure_port();
    }
    if (fd_ < 0) {
      RCLCPP_WARN(get_logger(), "Drop animal report for %s because serial port is not open.", msg->grid.c_str());
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
        write_line("REPORT " + msg->grid + " " + empty_animal_name_ + " 0");
      }
    } else {
      for (const auto & animal : msg->animals) {
        totals_[animal.name] += animal.count;
        write_line("REPORT " + msg->grid + " " + animal.name + " " + std::to_string(animal.count));
      }
    }

    if (send_summary_each_report_) {
      send_summary();
    }
  }

  void send_summary()
  {
    if (totals_.empty()) {
      return;
    }

    std::ostringstream line;
    line << "SUMMARY";
    for (const auto & [name, count] : totals_) {
      line << " " << name << " " << count;
    }
    write_line(line.str());
  }

  void write_line(const std::string & line)
  {
    const std::string payload = line + "\n";
    const ssize_t written = ::write(fd_, payload.data(), payload.size());
    if (written < 0 || static_cast<std::size_t>(written) != payload.size()) {
      RCLCPP_WARN(
        get_logger(), "Serial write failed on %s: %s",
        active_port_.c_str(), std::strerror(errno));
      close_port();
      return;
    }
    RCLCPP_INFO(get_logger(), "Serial report sent: %s", line.c_str());
  }

  void close_port()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
      active_port_.clear();
    }
  }

  int fd_{-1};
  int baud_{115200};
  bool send_summary_each_report_{false};
  bool send_empty_reports_{false};
  std::string port_;
  std::string report_topic_;
  std::string empty_animal_name_;
  std::string active_port_;
  std::string first_open_error_;
  std::vector<std::string> fallback_ports_;
  std::map<std::string, std::uint32_t> totals_;

  rclcpp::Subscription<wildlife_vision::msg::AnimalReport>::SharedPtr report_sub_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WildlifeReportSerialNode>());
  rclcpp::shutdown();
  return 0;
}
