#!/usr/bin/env python3
from __future__ import annotations

from typing import List

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import TwistWithCovarianceStamped

from ros2_unitree_legged_msgs.msg import HighState


class Go1HighStateConverter(Node):
    """
    Convert Unitree Go1 HighState -> standard ROS topics.

    Subscribes:
      /high_state (ros2_unitree_legged_msgs/HighState)

    Publishes:
      /imu                 (sensor_msgs/Imu)
      /twist               (geometry_msgs/TwistWithCovarianceStamped)
      /state/joint_states  (sensor_msgs/JointState)
      /contacts            (std_msgs/Int16MultiArray)  # [FL, FR, RL, RR] 0/1
    """

    def __init__(self) -> None:
        super().__init__("go1_high_state_converter")

        # ---------------------------
        # Topics
        # ---------------------------
        self.high_state_topic = self.declare_parameter("high_state_topic", "/high_state").value
        self.imu_topic = self.declare_parameter("imu_topic", "/imu").value
        self.twist_topic = self.declare_parameter("twist_topic", "/twist").value
        self.joint_states_topic = self.declare_parameter("joint_states_topic", "/state/joint_states").value
        self.contacts_topic = self.declare_parameter("contacts_topic", "/contacts").value

        # ---------------------------
        # Frames
        # ---------------------------
        self.imu_frame = self.declare_parameter("imu_frame", "imu").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.joint_state_frame = self.declare_parameter("joint_state_frame", "state/trunk").value

        # ---------------------------
        # IMU quaternion ordering
        # ---------------------------
        # "xyzw": q=[x,y,z,w], "wxyz": q=[w,x,y,z]
        self.imu_quat_order = str(self.declare_parameter("imu_quat_order", "xyzw").value).lower().strip()
        if self.imu_quat_order not in ("xyzw", "wxyz"):
            raise RuntimeError("imu_quat_order must be 'xyzw' or 'wxyz'.")

        # ---------------------------
        # Contacts
        # ---------------------------
        self.contact_threshold = float(self.declare_parameter("contact_threshold", 10.0).value)
        self.use_foot_force_est = bool(self.declare_parameter("use_foot_force_est", False).value)

        # ---------------------------
        # Joint mapping
        # ---------------------------
        # Must match URDF joint names
        self.joint_names: List[str] = list(
            self.declare_parameter(
                "joint_names",
                [
                    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
                    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
                ],
            ).value
        )
        self.motor_indices: List[int] = list(self.declare_parameter("motor_indices", list(range(12))).value)

        if len(self.joint_names) != 12:
            raise RuntimeError(f"Expected 12 joint_names, got {len(self.joint_names)}")
        if len(self.motor_indices) != 12:
            raise RuntimeError(f"Expected 12 motor_indices, got {len(self.motor_indices)}")

        # ---------------------------
        # Covariance knobs (simple defaults; tune later)
        # ---------------------------
        self.twist_cov_vxy = float(self.declare_parameter("twist_cov_vxy", 2e-3).value)
        self.twist_cov_vz = float(self.declare_parameter("twist_cov_vz", 5e-3).value)
        self.twist_cov_wz = float(self.declare_parameter("twist_cov_wz", 2e-3).value)

        self.imu_cov_ori_xy = float(self.declare_parameter("imu_cov_ori_xy", 1e-3).value)
        self.imu_cov_ori_z = float(self.declare_parameter("imu_cov_ori_z", 1e-2).value)
        self.imu_cov_w = float(self.declare_parameter("imu_cov_w", 1e-3).value)
        self.imu_cov_a = float(self.declare_parameter("imu_cov_a", 1e-2).value)

        # ---------------------------
        # Pub/Sub
        # ---------------------------
        self.sub = self.create_subscription(HighState, self.high_state_topic, self.cb_high_state, 10)

        self.pub_imu = self.create_publisher(Imu, self.imu_topic, 10)
        self.pub_twist = self.create_publisher(TwistWithCovarianceStamped, self.twist_topic, 10)
        self.pub_js = self.create_publisher(JointState, self.joint_states_topic, 10)
        self.pub_contacts = self.create_publisher(Int16MultiArray, self.contacts_topic, 10)

        self.get_logger().info(f"Sub: {self.high_state_topic}")
        self.get_logger().info(f"Pub IMU  : {self.imu_topic} (frame_id={self.imu_frame}, quat_order={self.imu_quat_order})")
        self.get_logger().info(f"Pub Twist: {self.twist_topic} (frame_id={self.base_frame})")
        self.get_logger().info(f"Pub JS   : {self.joint_states_topic} (frame_id={self.joint_state_frame})")
        self.get_logger().info(
            f"Pub contacts: {self.contacts_topic} (threshold={self.contact_threshold}, "
            f"source={'foot_force_est' if self.use_foot_force_est else 'foot_force'})"
        )

    def _fill_imu(self, msg: HighState, stamp) -> Imu:
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.imu_frame

        quat = msg.imu.quaternion
        if self.imu_quat_order == "xyzw":
            qx, qy, qz, qw = quat
        else:
            qw, qx, qy, qz = quat

        imu.orientation.x = float(qx)
        imu.orientation.y = float(qy)
        imu.orientation.z = float(qz)
        imu.orientation.w = float(qw)

        imu.angular_velocity.x = float(msg.imu.gyroscope[0])
        imu.angular_velocity.y = float(msg.imu.gyroscope[1])
        imu.angular_velocity.z = float(msg.imu.gyroscope[2])

        imu.linear_acceleration.x = float(msg.imu.accelerometer[0])
        imu.linear_acceleration.y = float(msg.imu.accelerometer[1])
        imu.linear_acceleration.z = float(msg.imu.accelerometer[2])

        # Covariances (diagonal defaults)
        imu.orientation_covariance = [
            self.imu_cov_ori_xy, 0.0, 0.0,
            0.0, self.imu_cov_ori_xy, 0.0,
            0.0, 0.0, self.imu_cov_ori_z,
        ]
        imu.angular_velocity_covariance = [
            self.imu_cov_w, 0.0, 0.0,
            0.0, self.imu_cov_w, 0.0,
            0.0, 0.0, self.imu_cov_w,
        ]
        imu.linear_acceleration_covariance = [
            self.imu_cov_a, 0.0, 0.0,
            0.0, self.imu_cov_a, 0.0,
            0.0, 0.0, self.imu_cov_a,
        ]
        return imu

    def _fill_twist(self, msg: HighState, stamp) -> TwistWithCovarianceStamped:
        tw = TwistWithCovarianceStamped()
        tw.header.stamp = stamp
        tw.header.frame_id = self.base_frame  # interpret as body-frame twist

        # HighState.velocity is float32[3]
        tw.twist.twist.linear.x = float(msg.velocity[0])
        tw.twist.twist.linear.y = float(msg.velocity[1])
        tw.twist.twist.linear.z = float(msg.velocity[2])

        # Only yaw rate is provided explicitly
        tw.twist.twist.angular.x = 0.0
        tw.twist.twist.angular.y = 0.0
        tw.twist.twist.angular.z = float(msg.yaw_speed)

        cov = [0.0] * 36
        cov[0] = self.twist_cov_vxy   # vx
        cov[7] = self.twist_cov_vxy   # vy
        cov[14] = self.twist_cov_vz   # vz
        cov[35] = self.twist_cov_wz   # wz
        tw.twist.covariance = cov
        return tw

    def _fill_joint_state(self, msg: HighState, stamp) -> JointState:
        js = JointState()
        js.header.stamp = stamp
        js.header.frame_id = self.joint_state_frame
        js.name = self.joint_names

        pos: List[float] = []
        vel: List[float] = []
        eff: List[float] = []

        for mi in self.motor_indices:
            ms = msg.motor_state[mi]
            pos.append(float(ms.q))
            vel.append(float(ms.dq))
            eff.append(float(ms.tau_est))

        js.position = pos
        js.velocity = vel
        js.effort = eff
        return js

    def _fill_contacts(self, msg: HighState) -> Int16MultiArray:
        src = msg.foot_force_est if self.use_foot_force_est else msg.foot_force
        contacts = Int16MultiArray()
        contacts.data = [1 if float(v) > self.contact_threshold else 0 for v in src]
        return contacts

    def cb_high_state(self, msg: HighState) -> None:
        stamp = self.get_clock().now().to_msg()

        self.pub_imu.publish(self._fill_imu(msg, stamp))
        self.pub_twist.publish(self._fill_twist(msg, stamp))
        self.pub_js.publish(self._fill_joint_state(msg, stamp))
        self.pub_contacts.publish(self._fill_contacts(msg))


def main() -> None:
    rclpy.init()
    node = Go1HighStateConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
