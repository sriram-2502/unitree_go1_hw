#!/usr/bin/env python3
from __future__ import annotations

from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---------------- Bag path: ~/rosbags_go1_high ----------------
    bag_root = Path.home() / "rosbags_go1_high"
    bag_root.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_path = str(bag_root / f"go1_ekf_highcmd_loop_{stamp}")

    # ---------------- Launch args ----------------
    ekf_params = LaunchConfiguration("ekf_params")

    enable_record = LaunchConfiguration("enable_record")
    record_delay_s = LaunchConfiguration("record_delay_s")
    loop_delay_s = LaunchConfiguration("loop_delay_s")
    pj_delay_s = LaunchConfiguration("pj_delay_s")

    # Loop params
    max_loops = LaunchConfiguration("max_loops")
    vx_amp = LaunchConfiguration("vx_amp")
    vy_amp = LaunchConfiguration("vy_amp")
    wz_amp = LaunchConfiguration("wz_amp")
    t_hold = LaunchConfiguration("t_hold")
    ramp_in = LaunchConfiguration("ramp_in")
    ramp_out = LaunchConfiguration("ramp_out")
    rate_hz = LaunchConfiguration("rate_hz")
    log_dt = LaunchConfiguration("log_dt")
    start_delay_s = LaunchConfiguration("start_delay_s")

    # ---------------- Unitree bridge include ----------------
    unitree_share = FindPackageShare("unitree_legged_real")
    highlevel_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([unitree_share, "/launch/high.launch.py"])
    )

    # ---------------- PlotJuggler layout file ----------------
    tools_share = FindPackageShare("go1_tools_high")
    layout_file = PathJoinSubstitution([tools_share, "config", "go1_ekf_layout.xml"])

    plotjuggler = ExecuteProcess(
        cmd=[
            "ros2", "run", "plotjuggler", "plotjuggler",
            "--",
            "-l", layout_file,   # if your PlotJuggler uses --layout instead, replace "-l" with "--layout"
        ],
        output="screen",
    )

    # ---------------- State converter ----------------
    state_converter = Node(
        package="go1_tools_high",
        executable="go1_high_state_converter.py",
        name="go1_high_state_converter",
        output="screen",
    )

    # ---------------- EKF (robot_localization) ----------------
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params],
    )

    # ---------------- Rosbag record ----------------
    # NOTE: if /cmd_vel is not published, rosbag may warn. If you want zero warnings, remove it.
    record = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record",
            "-o", bag_path,
            "/high_cmd",
            "/cmd_vel",
            "/high_state",
            "/low_state",
            "/imu",
            "/twist",
            "/odometry/filtered",
            "/tf",
            "/tf_static",
        ],
        output="screen",
    )

    # ---------------- HighCmd loop node ----------------
    highcmd_loop = Node(
        package="go1_tools_high",
        executable="ros2_highcmd_loop",
        name="go1_highcmd_profile_loop",
        output="screen",
        parameters=[{
            "loop_forever": False,
            "max_loops": max_loops,
            "vx_amp": vx_amp,
            "vy_amp": vy_amp,
            "wz_amp": wz_amp,
            "t_hold": t_hold,
            "ramp_in": ramp_in,
            "ramp_out": ramp_out,
            "rate_hz": rate_hz,
            "log_dt": log_dt,
            "wait_for_enter": False,
            "start_delay_s": start_delay_s,
        }],
    )

    return LaunchDescription([
        # ---------------- Arguments ----------------
        DeclareLaunchArgument(
            "ekf_params",
            default_value=PathJoinSubstitution([tools_share, "config", "ekf_high_twist.yaml"]),
            description="Path to robot_localization EKF params YAML",
        ),

        DeclareLaunchArgument("enable_record", default_value=TextSubstitution(text="true")),
        DeclareLaunchArgument("record_delay_s", default_value=TextSubstitution(text="1.0")),
        DeclareLaunchArgument("loop_delay_s", default_value=TextSubstitution(text="3.0")),
        DeclareLaunchArgument("pj_delay_s", default_value=TextSubstitution(text="2.0")),
        DeclareLaunchArgument("start_delay_s", default_value=TextSubstitution(text="2.0")),

        DeclareLaunchArgument("max_loops", default_value=TextSubstitution(text="2")),
        DeclareLaunchArgument("vx_amp", default_value=TextSubstitution(text="0.30")),
        DeclareLaunchArgument("vy_amp", default_value=TextSubstitution(text="0.25")),
        DeclareLaunchArgument("wz_amp", default_value=TextSubstitution(text="0.80")),
        DeclareLaunchArgument("t_hold", default_value=TextSubstitution(text="1.5")),
        DeclareLaunchArgument("ramp_in", default_value=TextSubstitution(text="0.2")),
        DeclareLaunchArgument("ramp_out", default_value=TextSubstitution(text="0.2")),
        DeclareLaunchArgument("rate_hz", default_value=TextSubstitution(text="200.0")),
        DeclareLaunchArgument("log_dt", default_value=TextSubstitution(text="0.2")),

        # ---------------- Bringup ----------------
        LogInfo(msg=TextSubstitution(text="[EKF] Launching Unitree HIGH bridge...")),
        highlevel_launch,

        LogInfo(msg=TextSubstitution(text="[EKF] Launching go1_high_state_converter (publishes /twist)...")),
        state_converter,

        LogInfo(msg=[TextSubstitution(text="[EKF] Launching ekf_node with params: "), ekf_params]),
        ekf,

        # ---------------- PlotJuggler (delayed) ----------------
        TimerAction(
            period=pj_delay_s,
            actions=[
                LogInfo(msg=TextSubstitution(text="[EKF] Starting PlotJuggler with saved layout...")),
                plotjuggler,
            ],
        ),

        # ---------------- Rosbag (delayed) ----------------
        TimerAction(
            period=record_delay_s,
            actions=[
                LogInfo(msg=TextSubstitution(text=f"[EKF] Recording rosbag to: {bag_path}")),
                record,
            ],
            condition=IfCondition(enable_record),
        ),

        # ---------------- Loop (delayed) ----------------
        TimerAction(
            period=loop_delay_s,
            actions=[
                LogInfo(msg=TextSubstitution(text="[EKF] Starting HighCmd loop...")),
                highcmd_loop,
            ],
        ),
    ])
