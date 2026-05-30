#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistRelayNode(Node):
    def __init__(self):
        super().__init__("twist_relay")

        self.declare_parameter('use_sim_time', True)

        # Twist → TwistStamped: for tools like teleop_twist_keyboard that publish plain Twist
        self.controller_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10
        )
        self.controller_pub = self.create_publisher(
            TwistStamped,
            "/servebot_controller/cmd_vel",
            10
        )

        # TwistStamped → Twist: for nodes that need plain Twist from a stamped source
        self.joy_sub = self.create_subscription(
            TwistStamped,
            "/cmd_vel_stamped",
            self.cmd_vel_stamped_callback,
            10
        )
        self.joy_pub = self.create_publisher(
            Twist,
            "/cmd_vel_out",
            10
        )

    def cmd_vel_callback(self, msg):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = "base_footprint"
        twist_stamped.twist = msg
        self.controller_pub.publish(twist_stamped)

    def cmd_vel_stamped_callback(self, msg):
        self.joy_pub.publish(msg.twist)


def main(args=None):
    rclpy.init(args=args)
    node = TwistRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
