from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # --- Packages ---
    unitree_pkg = get_package_share_directory("unitree_legged_real")
    conv_pkg = get_package_share_directory("go1_tools_low")

    # --- 1) Unitree low-level driver launch ---
    # Adjust filename if your launch is named differently
    unitree_low_launch = os.path.join(unitree_pkg, "launch", "low.launch.py")

    # --- 2) EKF config ---
    ekf_yaml = os.path.join(conv_pkg, "config", "ekf_imu_twist_srb.yaml")

    # --- Rosbag path ---
    bag_root = os.path.join(os.path.expanduser("~"), "rosbags_go1_low")
    os.makedirs(bag_root, exist_ok=True)
    bag_path = os.path.join(bag_root, "go1_low")

    enable_record = LaunchConfiguration("enable_record")
    record_delay_s = LaunchConfiguration("record_delay_s")

    # --- Include Unitree low.launch.py ---
    low = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(unitree_low_launch),
        launch_arguments={}.items(),
    )

    # --- EKF (robot_localization) ---
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_yaml],
    )

    # --- 3) Your Go1 low state converter node ---
    # Replace executable name if your entry-point is different
    converter = Node(
        package="go1_tools_low",
        executable="go1_low_state_converter.py",
        name="go1_state_converter",
        output="screen",
        parameters=[
            {
                # EKF output topic you will subscribe to inside the converter
                "ekf_odom_topic": "/odometry/filtered",
                "velocity_source": "srb",
                # Your existing topics
                "low_state_topic": "/low_state",
                "imu_topic": "/imu",
                "joint_states_topic": "/state/joint_states",
                "contacts_topic": "/contacts",
                "zero_imu_on_startup": False,
                "use_joint_bias": True,
                "joint_bias": [
                    0.01615, 0.0, 0.0,
                    0.01615, 0.0, 0.0,
                    0.01000, 0.0, 0.0,
                    0.01000, 0.0, 0.0,
                ],
            }
        ],
    )

    srb_vel = Node(
        package="go1_tools_low",
        executable="srb_velocity_estimator.py",
        name="srb_velocity_estimator",
        output="screen",
        parameters=[{
            "imu_topic": "/imu",
            "contacts_topic": "/contacts",
            "twist_topic": "/twist_srb",
            "world_frame": "odom",
            "min_contacts_for_zupt": 3,
            "alpha_zupt": 0.15,
            "use_window_filter": True,
            "velocity_window_size": 10,
        }]
    )


    record = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record",
            "-o", bag_path,
            "/low_state",
            "/imu",
            "/contacts",
            "/twist_srb",
            "/odometry/filtered",
            "/base_state",
            "/blind_state",
            "/quadruped_pympc_torques",
            "/trajectory_generator",
            "/mpc_grf",
            "/mpc_ref",
            "/tf",
            "/tf_static",
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("enable_record", default_value=TextSubstitution(text="false")),
        DeclareLaunchArgument("record_delay_s", default_value=TextSubstitution(text="1.0")),

        low,
        converter,
        srb_vel,
        ekf,

        TimerAction(
            period=record_delay_s,
            actions=[
                LogInfo(msg=TextSubstitution(text=f"[LOW] Recording rosbag to: {bag_path}")),
                record,
            ],
            condition=IfCondition(enable_record),
        ),

    ])
