#ifndef SERVEBOT_INTERFACE_HPP
#define SERVEBOT_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

// POSIX serial — replaces libserial
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <vector>
#include <string>
#include <cmath>

namespace servebot_firmware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ServebotInterface : public hardware_interface::SystemInterface
{
public:
  ServebotInterface();
  virtual ~ServebotInterface();

  // Lifecycle
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  // hardware_interface::SystemInterface
  CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  // ── ESP32 drive-train constants (must match servebot_firmware.ino) ──────
  static constexpr double WHEEL_RADIUS_M       = 0.060;
  static constexpr long   STEPS_PER_WHEEL_REV  = 46000L;
  static constexpr double MAX_SPEED_STEPS      = 3000.0;  // steps/s

  // ticks → radians:  angle = ticks * 2π / STEPS_PER_WHEEL_REV
  static constexpr double TICKS_TO_RAD =
      (2.0 * M_PI) / static_cast<double>(STEPS_PER_WHEEL_REV);

  // rad/s → steps/s:  steps_per_s = omega * STEPS_PER_WHEEL_REV / (2π)
  static constexpr double RADS_TO_STEPS =
      static_cast<double>(STEPS_PER_WHEEL_REV) / (2.0 * M_PI);

  // ── Serial port ──────────────────────────────────────────────────────────
  int         serial_fd_;       // POSIX file descriptor (-1 when closed)
  std::string port_;            // e.g. "/dev/ttyAMA0"
  int         baudrate_;        // 115200
  std::string read_buffer_;     // accumulates partial lines from ESP32

  // ── ros2_control state & command mirrors ─────────────────────────────────
  // Index 0 = left wheel,  index 1 = right wheel
  std::vector<double> velocity_commands_;   // rad/s  (from controller → ESP32)
  std::vector<double> position_states_;     // rad    (from ESP32 encoder ticks)
  std::vector<double> velocity_states_;     // rad/s  (derived from position delta)

  // ── Encoder book-keeping ─────────────────────────────────────────────────
  long   prev_enc_[2];          // last encoder tick counts received
  double prev_position_[2];     // previous position in rad (for velocity calc)
  rclcpp::Time last_run_;

  // ── Helpers ──────────────────────────────────────────────────────────────

  // Opens and configures the serial port; returns fd or -1 on error.
  int openSerial(const std::string & port, int baudrate);

  // Parses one complete line received from the ESP32.
  // "E <t1> <t2>"  → updates position_states_ and velocity_states_
  // "READY" / "OK STOPPED" / "ERR ..." → logs only
  void parseSerialLine(const std::string & line, const rclcpp::Duration & period);
};

}  // namespace servebot_firmware

#endif  // SERVEBOT_INTERFACE_HPP
