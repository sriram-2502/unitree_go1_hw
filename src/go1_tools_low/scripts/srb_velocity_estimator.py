#!/usr/bin/env python3
from __future__ import annotations

import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import TwistWithCovarianceStamped


def quat_to_rotmat_xyzw(q: np.ndarray) -> np.ndarray:
    """q = [x,y,z,w] -> R (world<-body)"""
    x, y, z, w = q
    # normalize
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-9:
        return np.eye(3)
    x, y, z, w = x/n, y/n, z/n, w/n
    # rotation
    R = np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])
    return R


class SRBVelocityEstimator(Node):
    """
    Simple SRB-style velocity estimate in WORLD frame.
    - Integrate gravity-compensated IMU accel (body) rotated to world using IMU orientation.
    - Apply ZUPT when enough contacts.
    - Moving-window average optional.
    Publishes: /twist_srb (TwistWithCovarianceStamped)
    """

    def __init__(self):
        super().__init__("srb_velocity_estimator")

        self.imu_topic = self.declare_parameter("imu_topic", "/imu").value
        self.contacts_topic = self.declare_parameter("contacts_topic", "/contacts").value
        self.twist_topic = self.declare_parameter("twist_topic", "/twist_srb").value
        self.world_frame = self.declare_parameter("world_frame", "odom").value

        self.min_contacts_for_zupt = int(self.declare_parameter("min_contacts_for_zupt", 3).value)
        self.alpha_zupt = float(self.declare_parameter("alpha_zupt", 0.15).value)

        self.use_window_filter = bool(self.declare_parameter("use_window_filter", True).value)
        self.velocity_window_size = int(self.declare_parameter("velocity_window_size", 10).value)

        # nominal covariances
        self.twist_lin_cov = float(self.declare_parameter("twist_lin_cov", 1e-2).value)  # (m/s)^2
        self.twist_ang_cov = float(self.declare_parameter("twist_ang_cov", 1e-2).value)  # (rad/s)^2

        self.sub_imu = self.create_subscription(Imu, self.imu_topic, self.cb_imu, 50)
        self.sub_contacts = self.create_subscription(Int16MultiArray, self.contacts_topic, self.cb_contacts, 50)

        self.pub_twist = self.create_publisher(TwistWithCovarianceStamped, self.twist_topic, 10)

        self.v_world = np.zeros(3)
        self.last_t = None
        self.contacts = np.array([1, 1, 1, 1], dtype=np.int32)

        self.v_hist = []

        self.get_logger().info(f"SRB vel est: imu={self.imu_topic}, contacts={self.contacts_topic}, pub={self.twist_topic} ({self.world_frame})")

    def cb_contacts(self, msg: Int16MultiArray):
        if len(msg.data) >= 4:
            self.contacts = np.array(msg.data[:4], dtype=np.int32)

    def cb_imu(self, msg: Imu):
        # time step from IMU stamps
        t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        if self.last_t is None:
            self.last_t = t
            return
        dt = t - self.last_t
        self.last_t = t
        if dt <= 0.0 or dt > 0.05:
            return

        # orientation (world<-body)
        q = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w], dtype=float)
        R_wb = quat_to_rotmat_xyzw(q)

        # accel in body
        a_b = np.array([msg.linear_acceleration.x,
                        msg.linear_acceleration.y,
                        msg.linear_acceleration.z], dtype=float)

        # rotate to world and remove gravity (assumes IMU already has gravity in it)
        g_w = np.array([0.0, 0.0, 9.81])
        a_w = R_wb @ a_b - g_w

        # integrate
        self.v_world = self.v_world + a_w * dt

        # ZUPT: if enough contacts, pull velocity toward 0
        if int(np.sum(self.contacts)) >= self.min_contacts_for_zupt:
            self.v_world = (1.0 - self.alpha_zupt) * self.v_world  # exponential decay to 0

        # optional moving window average
        v_out = self.v_world.copy()
        if self.use_window_filter:
            self.v_hist.append(v_out)
            if len(self.v_hist) > self.velocity_window_size:
                self.v_hist.pop(0)
            v_out = np.mean(np.stack(self.v_hist, axis=0), axis=0)

        # publish TwistWithCovarianceStamped
        tw = TwistWithCovarianceStamped()
        tw.header.stamp = msg.header.stamp
        tw.header.frame_id = self.world_frame

        tw.twist.twist.linear.x = float(v_out[0])
        tw.twist.twist.linear.y = float(v_out[1])
        tw.twist.twist.linear.z = float(v_out[2])

        # no angular estimate here (EKF should take gyro from IMU)
        tw.twist.twist.angular.x = 0.0
        tw.twist.twist.angular.y = 0.0
        tw.twist.twist.angular.z = 0.0

        # covariance: 6x6 row-major
        cov = np.zeros((6, 6), dtype=float)
        cov[0, 0] = self.twist_lin_cov
        cov[1, 1] = self.twist_lin_cov
        cov[2, 2] = self.twist_lin_cov
        cov[3, 3] = 1e3  # we are NOT providing angular vel here
        cov[4, 4] = 1e3
        cov[5, 5] = 1e3
        tw.twist.covariance = cov.reshape(-1).tolist()

        self.pub_twist.publish(tw)


def main():
    rclpy.init()
    node = SRBVelocityEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
