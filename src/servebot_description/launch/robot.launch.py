"""
robot.launch.py — Unified launch for simulation and real robot.

  Simulation  (default):
    ros2 launch servebot_description robot.launch.py sim_mode:=true

  Real robot:
    ros2 launch servebot_description robot.launch.py sim_mode:=false

Both modes start:
  - robot_state_publisher  (URDF → /tf)
  - joint_state_broadcaster
  - servebot_controller    (diff_drive_controller → /odom)

Simulation additionally starts:
  - Gazebo (ign_ros2_control handles controller_manager internally)

Real robot additionally starts:
  - ros2_control_node  (controller_manager + ServebotInterface → ESP32)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    desc_pkg  = get_package_share_directory("servebot_description")
    ctrl_pkg  = get_package_share_directory("servebot_controller")

    # ── Arguments ─────────────────────────────────────────────────────────

    sim_mode_arg = DeclareLaunchArgument(
        "sim_mode",
        default_value="true",
        description="true = Gazebo simulation, false = real ESP32 robot"
    )

    port_arg = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyAMA0",
        description="Serial port for ESP32 (real robot only)"
    )

    sim_mode   = LaunchConfiguration("sim_mode")
    use_sim_time = PythonExpression(["'", sim_mode, "' == 'true'"])

    urdf_file  = os.path.join(desc_pkg, "urdf", "servebot_urdf.xacro")
    controllers_yaml = os.path.join(ctrl_pkg, "config", "servebot_controllers.yaml")

    robot_description = ParameterValue(
        Command(["xacro ", urdf_file, " sim_mode:=", sim_mode]),
        value_type=str
    )

    # ── robot_state_publisher (real robot only — gazebo.launch.py handles sim) ──

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": False,
        }],
        condition=UnlessCondition(sim_mode)
    )

    # ── SIMULATION: include gazebo.launch.py ──────────────────────────────
    # Gazebo starts ign_ros2_control which creates the controller_manager

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg, "launch", "gazebo.launch.py")),
        condition=IfCondition(sim_mode)
    )

    # ── REAL ROBOT: ros2_control_node (controller_manager + hardware) ─────

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_yaml,
            {"use_sim_time": False},
        ],
        output="screen",
        condition=UnlessCondition(sim_mode)
    )

    # ── Controller spawners (both modes, delayed to let CM start) ─────────

    joint_state_broadcaster_spawner = TimerAction(
        period=3.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster",
                       "--controller-manager", "/controller_manager"],
        )]
    )

    diff_drive_spawner = TimerAction(
        period=4.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["servebot_controller",
                       "--controller-manager", "/controller_manager"],
        )]
    )

    return LaunchDescription([
        sim_mode_arg,
        port_arg,
        robot_state_publisher,
        gazebo_launch,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        diff_drive_spawner,
    ])
