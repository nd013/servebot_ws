"""
ps_teleop.launch.py — PS4/PS5 joystick teleop for ros2_control.

Works for BOTH simulation and real robot — the diff_drive_controller
receives the same /servebot_controller/cmd_vel (TwistStamped) in both cases.

Usage:
  ros2 launch servebot_firmware ps_teleop.launch.py

Hold L1 to enable motion. Hold R1 for full speed.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory("servebot_firmware")

    # ── Arguments ─────────────────────────────────────────────────────────

    joy_dev_arg = DeclareLaunchArgument(
        "joy_dev",
        default_value="0",
        description="Joystick device ID (check ls /dev/input/js*)"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock"
    )

    # ── Nodes ─────────────────────────────────────────────────────────────

    # Reads PS controller → publishes sensor_msgs/Joy on /joy
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[{
            "device_id":       LaunchConfiguration("joy_dev"),
            "deadzone":        0.1,
            "autorepeat_rate": 20.0,
            "use_sim_time":    LaunchConfiguration("use_sim_time"),
        }]
    )

    # Converts /joy → TwistStamped on /servebot_controller/cmd_vel
    # Hold L1 (button 4) to enable — Left stick Y = forward, Right stick X = turn
    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[
            os.path.join(pkg, "config", "ps_teleop.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[
            ("cmd_vel", "/servebot_controller/cmd_vel"),
        ]
    )

    return LaunchDescription([
        joy_dev_arg,
        use_sim_time_arg,
        joy_node,
        teleop_node,
    ])
