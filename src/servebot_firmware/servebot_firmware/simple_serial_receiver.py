#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray, String
import serial


class SimpleSerialReceiver(Node):
    def __init__(self):
        super().__init__("simple_serial_receiver")

        self.declare_parameter("port", "/dev/ttyAMA0")
        self.declare_parameter("baudrate", 115200)

        self.port_     = self.get_parameter("port").value
        self.baudrate_ = self.get_parameter("baudrate").value

        self.esp32_ = serial.Serial(port=self.port_, baudrate=self.baudrate_, timeout=0.1)

        self.enc_pub_    = self.create_publisher(Int64MultiArray, "encoder_ticks", 10)
        self.status_pub_ = self.create_publisher(String,          "esp32_status",  10)

        self.timer_ = self.create_timer(0.01, self.timerCallback)

        self.get_logger().info(
            "SimpleSerialReceiver started on %s at %d baud" % (self.port_, self.baudrate_))

    def timerCallback(self):
        if not rclpy.ok() or not self.esp32_.is_open or self.esp32_.in_waiting == 0:
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
    simple_serial_receiver = SimpleSerialReceiver()
    rclpy.spin(simple_serial_receiver)
    simple_serial_receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
