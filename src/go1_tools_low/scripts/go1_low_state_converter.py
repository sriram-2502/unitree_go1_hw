#!/usr/bin/env python3
from __future__ import annotations

from typing import List, Optional
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3Stamped
from nav_msgs.msg import Odometry

from ros2_unitree_legged_msgs.msg import LowState
from dls2_interface.msg import BaseState, BlindState


def quat_normalize_xyzw(qx: float, qy: float, qz: float, qw: float) -> List[float]:
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n < 1e-9:
        return [0.0, 0.0, 0.0, 1.0]
    return [qx/n, qy/n, qz/n, qw/n]

def quat_inverse_xyzw(qx: float, qy: float, qz: float, qw: float) -> List[float]:
    # For a unit quaternion, inverse is conjugate.
    return [-qx, -qy, -qz, qw]

def quat_multiply_xyzw(a: List[float], b: List[float]) -> List[float]:
    # Quaternion multiply (xyzw): result = a * b
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    rx = aw * bx + ax * bw + ay * bz - az * by
    ry = aw * by - ax * bz + ay * bw + az * bx
    rz = aw * bz + ax * by - ay * bx + az * bw
    rw = aw * bw - ax * bx - ay * by - az * bz
    return [rx, ry, rz, rw]

class Go1StateConverter(Node):
    """
    Go1 LowState -> PyMPC-friendly BaseState/BlindState (+ standard ROS topics).

    Inputs:
      - /low_state : joint states, raw IMU, foot forces
      - /twist_srb : world-frame linear velocity estimate (TwistWithCovarianceStamped)
      - /odometry/filtered : optional EKF output (nav_msgs/Odometry)

    Fusion logic for BaseState.velocity.linear (WORLD frame):
      prefer EKF odom twist if fresh,
      else SRB twist if fresh,
      else zero.

    Orientation + angular velocity always from LowState IMU.

    Base position:
      x,y = 0 (nominal)
      z   = nominal_base_z
    """

    def __init__(self) -> None:
        super().__init__("go1_state_converter")

        # ---------------------------
        # Topics
        # ---------------------------
        self.low_state_topic = self.declare_parameter("low_state_topic", "/low_state").value

        self.imu_topic = self.declare_parameter("imu_topic", "/imu").value
        self.contacts_topic = self.declare_parameter("contacts_topic", "/contacts").value

        self.base_state_topic = self.declare_parameter("base_state_topic", "/base_state").value
        self.blind_state_topic = self.declare_parameter("blind_state_topic", "/blind_state").value

        # Estimator topics
        self.twist_srb_topic = self.declare_parameter("twist_srb_topic", "/twist_srb").value
        self.ekf_odom_topic = self.declare_parameter("ekf_odom_topic", "/odometry/filtered").value

        # Which source to prefer for world linear velocity
        # Options: "ekf", "srb", "none"
        self.velocity_source = str(self.declare_parameter("velocity_source", "ekf").value)

        # Freshness timeout (seconds). If a message is older than this, ignore it.
        self.state_timeout = float(self.declare_parameter("state_timeout", 0.20).value)

        # ---------------------------
        # Frames
        # ---------------------------
        self.imu_frame = self.declare_parameter("imu_frame", "imu").value

        self.dls_frame_id = self.declare_parameter("dls_frame_id", "odom").value
        self.robot_name = self.declare_parameter("robot_name", "go1").value
        self.seq = 0

        # Quaternion ordering in LowState
        self.imu_quat_order = self.declare_parameter("imu_quat_order", "wxyz").value  # "wxyz" or "xyzw"
        self.zero_imu_on_startup = bool(self.declare_parameter("zero_imu_on_startup", False).value)
        self._imu_q0_inv: Optional[List[float]] = None

        # ---------------------------
        # Contacts
        # ---------------------------
        self.contact_threshold = float(self.declare_parameter("contact_threshold", 10.0).value)
        self.use_foot_force_est = bool(self.declare_parameter("use_foot_force_est", False).value)

        # ---------------------------
        # Joint mapping (PyMPC order)
        # ---------------------------
        self.joint_names: List[str] = list(
            self.declare_parameter(
                "joint_names",
                [
                    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
                    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
                ],
            ).value
        )

        self.motor_indices: List[int] = list(
            self.declare_parameter(
                "motor_indices",
                [3, 4, 5,  0, 1, 2,  9, 10, 11,  6, 7, 8],
            ).value
        )

        if len(self.joint_names) != 12:
            raise RuntimeError(f"Expected 12 joint_names, got {len(self.joint_names)}")
        if len(self.motor_indices) != 12:
            raise RuntimeError(f"Expected 12 motor_indices, got {len(self.motor_indices)}")

        # ---------------------------
        # Joint bias (PyMPC order)
        # ---------------------------
        self.use_joint_bias = bool(self.declare_parameter("use_joint_bias", False).value)
        self.joint_bias = list(
            self.declare_parameter(
                "joint_bias",
                [0.0] * 12,
            ).value
        )
        if len(self.joint_bias) != 12:
            raise RuntimeError(f"Expected 12 joint_bias entries, got {len(self.joint_bias)}")

        # ---------------------------
        # Base pose nominal
        # ---------------------------
        self.nominal_base_z = float(self.declare_parameter("nominal_base_z", 0.30).value)
        self.publish_zero_base_xy = bool(self.declare_parameter("publish_zero_base_xy", True).value)

        # ---------------------------
        # Cached estimator data
        # ---------------------------
        self._last_twist_srb: Optional[TwistWithCovarianceStamped] = None
        self._last_odom_ekf: Optional[Odometry] = None

        # ---------------------------
        # Pub/Sub
        # ---------------------------
        self.sub_low = self.create_subscription(LowState, self.low_state_topic, self.cb_low_state, 10)
        self.sub_srb = self.create_subscription(TwistWithCovarianceStamped, self.twist_srb_topic, self.cb_twist_srb, 10)
        self.sub_ekf = self.create_subscription(Odometry, self.ekf_odom_topic, self.cb_odom_ekf, 10)

        self.pub_imu = self.create_publisher(Imu, self.imu_topic, 10)
        self.pub_contacts = self.create_publisher(Int16MultiArray, self.contacts_topic, 10)

        self.pub_base = self.create_publisher(BaseState, self.base_state_topic, 10)
        self.pub_blind = self.create_publisher(BlindState, self.blind_state_topic, 10)
        self.pub_rpy = self.create_publisher(Vector3Stamped, "/base_state_rpy", 10)

        self.get_logger().info(f"Sub LowState: {self.low_state_topic}")
        self.get_logger().info(f"Sub SRB twist: {self.twist_srb_topic} (TwistWithCovarianceStamped)")
        self.get_logger().info(f"Sub EKF odom : {self.ekf_odom_topic} (Odometry)")
        self.get_logger().info(f"Velocity preference: {self.velocity_source} (timeout={self.state_timeout}s)")
        self.get_logger().info(f"DLS frame_id: {self.dls_frame_id}, nominal z={self.nominal_base_z}")

    # ---------------------------
    # Helper: time freshness
    # ---------------------------
    def _now_sec(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _is_fresh(self, msg_stamp) -> bool:
        if msg_stamp is None:
            return False
        age = self._now_sec() - self._stamp_to_sec(msg_stamp)
        return (age >= 0.0) and (age <= self.state_timeout)

    # ---------------------------
    # Estimator callbacks
    # ---------------------------
    def cb_twist_srb(self, msg: TwistWithCovarianceStamped) -> None:
        self._last_twist_srb = msg

    def cb_odom_ekf(self, msg: Odometry) -> None:
        self._last_odom_ekf = msg

    # ---------------------------
    # Main conversion callback
    # ---------------------------
    def cb_low_state(self, msg: LowState) -> None:
        self.seq += 1
        stamp = self.get_clock().now().to_msg()
        t = self._now_sec()

        # ---------------------------
        # Contacts
        # ---------------------------
        src = msg.foot_force_est if self.use_foot_force_est else msg.foot_force
        # Raw Unitree order: [FR, FL, RR, RL]
        contact_unitree = [bool(float(v) > self.contact_threshold) for v in src]

        # Remap to PyMPC order: [FL, FR, RL, RR]
        contact_bool = [contact_unitree[i] for i in [1, 0, 3, 2]]

        contact_int = [1 if c else 0 for c in contact_bool]


        # ---------------------------
        # IMU quaternion -> xyzw
        # ---------------------------
        q = list(msg.imu.quaternion)
        if self.imu_quat_order == "wxyz":
            qw, qx, qy, qz = q
        else:
            qx, qy, qz, qw = q
        qx, qy, qz, qw = quat_normalize_xyzw(float(qx), float(qy), float(qz), float(qw))
        if self.zero_imu_on_startup:
            if self._imu_q0_inv is None:
                self._imu_q0_inv = quat_inverse_xyzw(qx, qy, qz, qw)
                self.get_logger().info("Zeroed IMU orientation on startup")
            qx, qy, qz, qw = quat_multiply_xyzw(self._imu_q0_inv, [qx, qy, qz, qw])
            qx, qy, qz, qw = quat_normalize_xyzw(float(qx), float(qy), float(qz), float(qw))

        # ---------------------------
        # Publish ROS IMU
        # ---------------------------
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.imu_frame

        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw

        imu.angular_velocity.x = float(msg.imu.gyroscope[0])
        imu.angular_velocity.y = float(msg.imu.gyroscope[1])
        imu.angular_velocity.z = float(msg.imu.gyroscope[2])

        imu.linear_acceleration.x = float(msg.imu.accelerometer[0])
        imu.linear_acceleration.y = float(msg.imu.accelerometer[1])
        imu.linear_acceleration.z = float(msg.imu.accelerometer[2])

        self.pub_imu.publish(imu)

        # ---------------------------
        # Joint data
        # ---------------------------
        pos: List[float] = []
        vel: List[float] = []
        eff: List[float] = []
        q_acc: List[float] = []
        q_tau: List[float] = []
        q_temp: List[float] = []

        for mi in self.motor_indices:
            ms = msg.motor_state[mi]
            pos.append(float(ms.q))
            vel.append(float(ms.dq))
            eff.append(float(ms.tau_est))
            q_acc.append(float(ms.ddq))
            q_tau.append(float(ms.tau_est))
            q_temp.append(float(ms.temperature))

        if self.use_joint_bias:
            pos = [p - b for p, b in zip(pos, self.joint_bias)]

        # ---------------------------
        # Contacts topic
        # ---------------------------
        contacts = Int16MultiArray()
        contacts.data = contact_int
        self.pub_contacts.publish(contacts)

        # ---------------------------
        # BlindState
        # ---------------------------
        blind = BlindState()
        blind.frame_id = self.dls_frame_id
        blind.sequence_id = int(self.seq)
        blind.timestamp = float(t)
        blind.robot_name = self.robot_name

        blind.joints_name = self.joint_names
        blind.joints_position = pos
        blind.joints_velocity = vel
        blind.joints_acceleration = q_acc
        blind.joints_effort = q_tau
        blind.joints_temperature = q_temp

        blind.feet_contact = contact_bool
        blind.current_feet_positions = [0.0] * 12  # leave zero for now

        self.pub_blind.publish(blind)

        # ---------------------------
        # Pick WORLD linear velocity source
        # ---------------------------
        v_world = [0.0, 0.0, 0.0]

        ekf_ok = (self._last_odom_ekf is not None) and self._is_fresh(self._last_odom_ekf.header.stamp)
        srb_ok = (self._last_twist_srb is not None) and self._is_fresh(self._last_twist_srb.header.stamp)

        if self.velocity_source == "ekf":
            if ekf_ok:
                tw = self._last_odom_ekf.twist.twist
                v_world = [float(tw.linear.x), float(tw.linear.y), float(tw.linear.z)]
            elif srb_ok:
                tw = self._last_twist_srb.twist.twist
                v_world = [float(tw.linear.x), float(tw.linear.y), float(tw.linear.z)]

        elif self.velocity_source == "srb":
            if srb_ok:
                tw = self._last_twist_srb.twist.twist
                v_world = [float(tw.linear.x), float(tw.linear.y), float(tw.linear.z)]
            elif ekf_ok:
                tw = self._last_odom_ekf.twist.twist
                v_world = [float(tw.linear.x), float(tw.linear.y), float(tw.linear.z)]

        else:  # "none"
            v_world = [0.0, 0.0, 0.0]

        # ---------------------------
        # BaseState
        # ---------------------------
        base = BaseState()
        base.frame_id = self.dls_frame_id
        base.sequence_id = int(self.seq)
        base.timestamp = float(t)
        base.robot_name = self.robot_name

        # pose: nominal
        if self.publish_zero_base_xy:
            base.pose.position = [0.0, 0.0, float(self.nominal_base_z)]
        else:
            base.pose.position = [0.0, 0.0, float(self.nominal_base_z)]

        base.pose.orientation = [qx, qy, qz, qw]  # xyzw

        # WORLD linear velocity for MPC tracking
        base.velocity.linear = v_world

        # Angular velocity: gyro (body frame)
        base.velocity.angular = [
            float(msg.imu.gyroscope[0]),
            float(msg.imu.gyroscope[1]),
            float(msg.imu.gyroscope[2]),
        ]

        # raw accelerometer (includes gravity)
        base.acceleration.linear = [
            float(msg.imu.accelerometer[0]),
            float(msg.imu.accelerometer[1]),
            float(msg.imu.accelerometer[2]),
        ]
        base.acceleration.angular = [0.0, 0.0, 0.0]

        base.stance_status = contact_bool

        self.pub_base.publish(base)

        # ---------------------------
        # Debug RPY topic
        # ---------------------------
        rpy_msg = Vector3Stamped()
        rpy_msg.header.stamp = stamp
        rpy_msg.header.frame_id = self.dls_frame_id
        # Convert quaternion (xyzw) to roll/pitch/yaw
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        rpy_msg.vector.x = float(roll)
        rpy_msg.vector.y = float(pitch)
        rpy_msg.vector.z = float(yaw)
        self.pub_rpy.publish(rpy_msg)


def main() -> None:
    rclpy.init()
    node = Go1StateConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
