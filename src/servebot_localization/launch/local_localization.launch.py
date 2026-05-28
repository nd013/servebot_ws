from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
import os

def generate_launch_description():

    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="False",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",  # False for real robot (no /clock topic)
    )

    sim_mode_arg = DeclareLaunchArgument(
        "sim_mode",
        default_value="False",  # False = real robot, True = Gazebo simulation
    )

    use_python  = LaunchConfiguration("use_python")
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim_mode    = LaunchConfiguration("sim_mode")

    pkg = get_package_share_directory("servebot_localization")
    ekf_sim_config  = os.path.join(pkg, "config", "ekf.yaml")
    ekf_real_config = os.path.join(pkg, "config", "ekf_real.yaml")

    # ── Simulation EKF (base_footprint_ekf, odom_noisy) ──────────────────────
    robot_localization_sim = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_sim_config,
            {"use_sim_time": use_sim_time}
        ],
        condition=IfCondition(sim_mode),
    )

    # Static TF: base_footprint_ekf → imu_link_ekf (both sim and real robot)
    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0", "--y", "0", "--z", "0.103",
                   "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                   "--frame-id", "base_footprint_ekf",
                   "--child-frame-id", "imu_link_ekf"],
    )

    # ── Real Robot EKF (base_footprint_ekf, servebot_controller/odom) ────────
    robot_localization_real = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_real_config,
            {"use_sim_time": False}
        ],
        condition=UnlessCondition(sim_mode),
    )

    # ── IMU republisher (shared by both modes) ────────────────────────────────
    imu_republisher_py = Node(
        package="servebot_localization",
        executable="imu_republisher.py",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_python),
    )

    imu_republisher_cpp = Node(
        package="servebot_localization",
        executable="imu_republisher",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=UnlessCondition(use_python),
    )

    return LaunchDescription([
        use_python_arg,
        use_sim_time_arg,
        sim_mode_arg,
        static_transform_publisher,
        robot_localization_sim,
        robot_localization_real,
        imu_republisher_py,
        imu_republisher_cpp,
    ])
