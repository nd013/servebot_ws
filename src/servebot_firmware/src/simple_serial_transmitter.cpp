#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include <string>
#include <stdexcept>

class SimpleSerialTransmitter : public rclcpp::Node
{
public:
  SimpleSerialTransmitter() : Node("simple_serial_transmitter")
  {
    this->declare_parameter("port",     "/dev/ttyAMA0");
    this->declare_parameter("baudrate", 115200);

    port_     = this->get_parameter("port").as_string();
    baudrate_ = this->get_parameter("baudrate").as_int();

    serial_fd_ = openSerial(port_, baudrate_);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open %s: %s", port_.c_str(), strerror(errno));
      throw std::runtime_error("Failed to open serial port");
    }

    // Publishers — data coming FROM esp32
    enc_pub_    = this->create_publisher<std_msgs::msg::Int64MultiArray>("encoder_ticks", 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>("esp32_status", 10);

    // Subscribers — commands going TO esp32
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&SimpleSerialTransmitter::cmdVelCallback, this, std::placeholders::_1));

    stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "emergency_stop", 10,
      std::bind(&SimpleSerialTransmitter::stopCallback, this, std::placeholders::_1));

    // Timer reads serial at 100 Hz (faster than ESP32's 50 Hz encoder report)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&SimpleSerialTransmitter::readSerial, this));

    RCLCPP_INFO(this->get_logger(),
      "SimpleSerialTransmitter started on %s at %d baud", port_.c_str(), baudrate_);
  }

  ~SimpleSerialTransmitter()
  {
    if (serial_fd_ >= 0) close(serial_fd_);
  }

private:
  int openSerial(const std::string & port, int baudrate)
  {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
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

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    float ly = std::max(-100.0f, std::min(100.0f, (float)(msg->linear.x  * 100.0)));
    float rx = std::max(-100.0f, std::min(100.0f, (float)(msg->angular.z * 100.0)));
    char buf[32];
    int  n = snprintf(buf, sizeof(buf), "J %.2f %.2f\n", ly, rx);
    write(serial_fd_, buf, n);
  }

  void stopCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      write(serial_fd_, "S\n", 2);
      RCLCPP_INFO(this->get_logger(), "Emergency stop sent");
    }
  }

  void readSerial()
  {
    char buf[256];
    int n = read(serial_fd_, buf, sizeof(buf) - 1);
    if (n <= 0) return;

    read_buffer_.append(buf, n);

    size_t pos;
    while ((pos = read_buffer_.find('\n')) != std::string::npos) {
      std::string line = read_buffer_.substr(0, pos);
      read_buffer_.erase(0, pos + 1);
      if (!line.empty() && line.back() == '\r') line.pop_back();
      if (line.empty()) continue;

      // "E <enc1> <enc2>" — encoder ticks at 50 Hz
      if (line.size() > 2 && line[0] == 'E' && line[1] == ' ') {
        long e1 = 0, e2 = 0;
        if (sscanf(line.c_str() + 2, "%ld %ld", &e1, &e2) == 2) {
          auto msg = std_msgs::msg::Int64MultiArray();
          msg.data = {e1, e2};
          enc_pub_->publish(msg);
        }
        continue;
      }

      // All other messages — READY, OK STOPPED, BOOTING, ERR ...
      if (line.rfind("ERR", 0) == 0) {
        RCLCPP_ERROR(this->get_logger(), "ESP32: %s", line.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "ESP32: %s", line.c_str());
      }
      auto status = std_msgs::msg::String();
      status.data = line;
      status_pub_->publish(status);
    }
  }

  int         serial_fd_;
  std::string port_;
  int         baudrate_;
  std::string read_buffer_;

  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr enc_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr          status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr   cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr         stop_sub_;
  rclcpp::TimerBase::SharedPtr                                 timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleSerialTransmitter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
