#include "servebot_firmware/servebot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <algorithm>
#include <cmath>

namespace servebot_firmware
{

// ── Constructor / Destructor ───────────────────────────────────────────────

ServebotInterface::ServebotInterface()
  : serial_fd_(-1), baudrate_(115200)
{
  prev_enc_[0]      = 0;
  prev_enc_[1]      = 0;
  prev_position_[0] = 0.0;
  prev_position_[1] = 0.0;
}

ServebotInterface::~ServebotInterface()
{
  if (serial_fd_ >= 0) {
    ::write(serial_fd_, "S\n", 2);
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

// ── Private: open + configure POSIX serial port ───────────────────────────

int ServebotInterface::openSerial(const std::string & port, int baudrate)
{
  int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) return fd;

  struct termios tty;
  tcgetattr(fd, &tty);

  speed_t speed;
  switch (baudrate) {
    case 9600:   speed = B9600;   break;
    case 19200:  speed = B19200;  break;
    case 38400:  speed = B38400;  break;
    case 57600:  speed = B57600;  break;
    case 115200: speed = B115200; break;
    default:     speed = B115200; break;
  }
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
  tty.c_iflag &= ~(IGNBRK | IXON | IXOFF | IXANY);
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 1;

  tcsetattr(fd, TCSANOW, &tty);
  return fd;
}

// ── Private: parse one line received from ESP32 ───────────────────────────
//
// Joint mapping (from servebot_ros2_control.xacro):
//   index 0  =  wheel_right_joint  ←  enc2  (M2, RIGHT)
//   index 1  =  wheel_left_joint   ←  enc1  (M1, LEFT)

void ServebotInterface::parseSerialLine(const std::string & line,
                                        const rclcpp::Duration & period)
{
  // "E <enc1> <enc2>"  encoder ticks at 50 Hz
  if (line.size() > 2 && line[0] == 'E' && line[1] == ' ') {
    long e1 = 0, e2 = 0;
    if (sscanf(line.c_str() + 2, "%ld %ld", &e1, &e2) != 2) return;

    double dt = period.seconds();

    // ticks → radians
    double pos_right = static_cast<double>(e2) * TICKS_TO_RAD;  // enc2 → index 0
    double pos_left  = static_cast<double>(e1) * TICKS_TO_RAD;  // enc1 → index 1

    if (dt > 0.0) {
      velocity_states_[0] = (pos_right - prev_position_[0]) / dt;
      velocity_states_[1] = (pos_left  - prev_position_[1]) / dt;
    }

    position_states_[0] = pos_right;
    position_states_[1] = pos_left;
    prev_position_[0]   = pos_right;
    prev_position_[1]   = pos_left;
    prev_enc_[0]        = e2;
    prev_enc_[1]        = e1;
    return;
  }

  if (line.rfind("ERR", 0) == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ServebotInterface"), "ESP32: %s", line.c_str());
  } else {
    RCLCPP_INFO(rclcpp::get_logger("ServebotInterface"), "ESP32: %s", line.c_str());
  }
}

// ── on_init ───────────────────────────────────────────────────────────────

CallbackReturn ServebotInterface::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS) {
    return result;
  }

  try {
    port_ = info_.hardware_parameters.at("port");
  } catch (const std::out_of_range &) {
    RCLCPP_FATAL(rclcpp::get_logger("ServebotInterface"),
                 "No 'port' parameter in hardware_info! Aborting");
    return CallbackReturn::FAILURE;
  }

  try {
    baudrate_ = std::stoi(info_.hardware_parameters.at("baudrate"));
  } catch (...) {
    baudrate_ = 115200;
  }

  velocity_commands_.assign(info_.joints.size(), 0.0);
  position_states_.assign(info_.joints.size(), 0.0);
  velocity_states_.assign(info_.joints.size(), 0.0);

  last_run_ = rclcpp::Clock().now();

  RCLCPP_INFO(rclcpp::get_logger("ServebotInterface"),
              "on_init: port=%s  baudrate=%d  joints=%zu",
              port_.c_str(), baudrate_, info_.joints.size());

  return CallbackReturn::SUCCESS;
}

// ── export_state_interfaces ───────────────────────────────────────────────

std::vector<hardware_interface::StateInterface>
ServebotInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]);
    state_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]);
  }
  return state_interfaces;
}

// ── export_command_interfaces ─────────────────────────────────────────────

std::vector<hardware_interface::CommandInterface>
ServebotInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]);
  }
  return command_interfaces;
}

// ── on_activate ───────────────────────────────────────────────────────────

CallbackReturn ServebotInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("ServebotInterface"), "Starting robot hardware ...");

  velocity_commands_ = {0.0, 0.0};
  position_states_   = {0.0, 0.0};
  velocity_states_   = {0.0, 0.0};
  prev_enc_[0]       = 0;
  prev_enc_[1]       = 0;
  prev_position_[0]  = 0.0;
  prev_position_[1]  = 0.0;

  serial_fd_ = openSerial(port_, baudrate_);
  if (serial_fd_ < 0) {
    RCLCPP_FATAL(rclcpp::get_logger("ServebotInterface"),
                 "Failed to open %s: %s", port_.c_str(), strerror(errno));
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("ServebotInterface"),
              "Connected to ESP32 on %s at %d baud — ready for commands",
              port_.c_str(), baudrate_);
  return CallbackReturn::SUCCESS;
}

// ── on_deactivate ─────────────────────────────────────────────────────────

CallbackReturn ServebotInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("ServebotInterface"), "Stopping robot hardware ...");

  if (serial_fd_ >= 0) {
    ::write(serial_fd_, "S\n", 2);
    ::close(serial_fd_);
    serial_fd_ = -1;
  }

  RCLCPP_INFO(rclcpp::get_logger("ServebotInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}

// ── read ──────────────────────────────────────────────────────────────────
// Drains the serial RX buffer, assembles complete lines, calls parseSerialLine.

hardware_interface::return_type ServebotInterface::read(const rclcpp::Time &,
                                                        const rclcpp::Duration & period)
{
  if (serial_fd_ < 0) return hardware_interface::return_type::ERROR;

  char buf[256];
  int n = ::read(serial_fd_, buf, sizeof(buf) - 1);
  if (n <= 0) return hardware_interface::return_type::OK;

  read_buffer_.append(buf, n);

  size_t pos;
  while ((pos = read_buffer_.find('\n')) != std::string::npos) {
    std::string line = read_buffer_.substr(0, pos);
    read_buffer_.erase(0, pos + 1);
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (!line.empty()) {
      parseSerialLine(line, period);
    }
  }

  return hardware_interface::return_type::OK;
}

// ── write ─────────────────────────────────────────────────────────────────
// Converts wheel velocity commands (rad/s) to ESP32 J command.
//
//   velocity_commands_[0] = wheel_right  (rad/s)
//   velocity_commands_[1] = wheel_left   (rad/s)
//
// ESP32 tank-drive mixing:
//   L = (ly + rx) / 100  →  ly = (L_norm + R_norm) * 50
//   R = (ly - rx) / 100  →  rx = (L_norm - R_norm) * 50
//
// L_norm = vel_left  * RADS_TO_STEPS / MAX_SPEED_STEPS  (clamped ±1)
// R_norm = vel_right * RADS_TO_STEPS / MAX_SPEED_STEPS  (clamped ±1)

hardware_interface::return_type ServebotInterface::write(const rclcpp::Time &,
                                                         const rclcpp::Duration &)
{
  if (serial_fd_ < 0) return hardware_interface::return_type::ERROR;

  double L = std::clamp(velocity_commands_[1] * RADS_TO_STEPS / MAX_SPEED_STEPS, -1.0, 1.0);
  double R = std::clamp(velocity_commands_[0] * RADS_TO_STEPS / MAX_SPEED_STEPS, -1.0, 1.0);

  double ly = std::clamp((L + R) * 50.0, -100.0, 100.0);
  double rx = std::clamp((L - R) * 50.0, -100.0, 100.0);

  char cmd[32];
  int  len = snprintf(cmd, sizeof(cmd), "J %.2f %.2f\n", ly, rx);

  if (::write(serial_fd_, cmd, len) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ServebotInterface"),
                 "Serial write failed: %s", strerror(errno));
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace servebot_firmware

PLUGINLIB_EXPORT_CLASS(servebot_firmware::ServebotInterface, hardware_interface::SystemInterface)
