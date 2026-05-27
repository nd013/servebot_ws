#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int64MultiArray, String
import serial


class SimpleSerialTransmitter(Node):
    def __init__(self):
        super().__init__("simple_serial_transmitter")

        self.declare_parameter("port", "/dev/ttyAMA0")
        self.declare_parameter("baudrate", 115200)

        self.port_     = self.get_parameter("port").value
        self.baudrate_ = self.get_parameter("baudrate").value

        self.esp32_ = serial.Serial(port=self.port_, baudrate=self.baudrate_, timeout=0.1)

        # Publishers — data coming FROM esp32
        self.enc_pub_    = self.create_publisher(Int64MultiArray, "encoder_ticks", 10)
        self.status_pub_ = self.create_publisher(String,          "esp32_status",  10)

        # Subscribers — commands going TO esp32
        self.cmd_sub_  = self.create_subscription(Twist, "cmd_vel",        self.cmdVelCallback, 10)
        self.stop_sub_ = self.create_subscription(Bool,  "emergency_stop", self.stopCallback,   10)

        # Timer reads serial at 100 Hz (faster than ESP32's 50 Hz encoder report)
        self.timer_ = self.create_timer(0.01, self.readSerial)

        self.get_logger().info(
            "SimpleSerialTransmitter started on %s at %d baud" % (self.port_, self.baudrate_))

    def cmdVelCallback(self, msg):
        ly = max(-100.0, min(100.0, msg.linear.x  * 100.0))
        rx = max(-100.0, min(100.0, msg.angular.z * 100.0))
        self.esp32_.write(("J %.2f %.2f\n" % (ly, rx)).encode("utf-8"))

    def stopCallback(self, msg):
        if msg.data:
            self.esp32_.write(b"S\n")
            self.get_logger().info("Emergency stop sent")

    def readSerial(self):
        if not self.esp32_.is_open or self.esp32_.in_waiting == 0:
            return

        line = self.esp32_.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            return

        # "E <enc1> <enc2>" — encoder ticks at 50 Hz
        if line.startswith("E "):
            parts = line.split()
            if len(parts) == 3:
                try:
                    msg = Int64MultiArray()
                    msg.data = [int(parts[1]), int(parts[2])]
                    self.enc_pub_.publish(msg)
                except ValueError:
                    pass
            return

        # All other messages — READY, OK STOPPED, BOOTING, ERR ...
        if line.startswith("ERR"):
            self.get_logger().error("ESP32: %s" % line)
        else:
            self.get_logger().info("ESP32: %s" % line)

        status = String()
        status.data = line
        self.status_pub_.publish(status)


def main():
    rclpy.init()
    simple_serial_transmitter = SimpleSerialTransmitter()
    rclpy.spin(simple_serial_transmitter)
    simple_serial_transmitter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
