#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from ros2_unitree_legged_msgs.msg import LowCmd
from dls2_interface.msg import TrajectoryGenerator, ControlSignal

POS_STOP_F = 2.146e9
VEL_STOP_F = 16000.0

class Go1UnitreeStyleBridge(Node):
    def __init__(self):
        super().__init__("go1_pd_servo")

        # Same as C++ swap_joint_indices:
        # cmd index i (HW order FR,FL,RR,RL) pulls from ctrl index swap[i] (your order FL,FR,RL,RR)
        self.swap_joint_indices = [3,4,5, 0,1,2, 9,10,11, 6,7,8]
        
        # Validate swap_joint_indices
        assert len(self.swap_joint_indices) == 12, "swap_joint_indices must have 12 elements"
        assert all(0 <= i < 12 for i in self.swap_joint_indices), "All swap indices must be in range [0, 11]"

        self.declare_parameter("enabled", False)
        self.declare_parameter("kp", 0.0)
        self.declare_parameter("kd", 3.0)
        self.declare_parameter("use_ff_torque", True)
        self.declare_parameter("tau_limit", 20.0)

        self.last_traj = None
        self.last_tau = None
        self.shutdown_in_progress = False

        self.sub_traj = self.create_subscription(
            TrajectoryGenerator, "/trajectory_generator", self.on_traj, 10
        )
        self.sub_tau = self.create_subscription(
            ControlSignal, "/quadruped_pympc_torques", self.on_tau, 10
        )
        self.pub_low = self.create_publisher(LowCmd, "/low_cmd", 10)

        self.timer = self.create_timer(0.002, self.on_timer)  # 500Hz
        self.get_logger().info("Go1 bridge: Unitree-style send_cmd()/udp_init_send() behavior.")

    def on_traj(self, msg: TrajectoryGenerator):
        self.last_traj = msg

    def on_tau(self, msg: ControlSignal):
        # controller order expected = FL,FR,RL,RR (12)
        # Always reset to avoid stale torque data
        self.last_tau = None
        if msg.torques and len(msg.torques) >= 12:
            self.last_tau = list(msg.torques[:12])

    def build_udp_init_send(self) -> LowCmd:
        """Matches udp_init_send(): servo mode + PosStopF/VelStopF, Kp=Kd=0, tau=0."""
        cmd = LowCmd()
        cmd.head[0] = 0xFE
        cmd.head[1] = 0xEF
        cmd.level_flag = 0xFF
        cmd.frame_reserve = 0
        for m in cmd.motor_cmd:
            m.mode = 0x0A
            m.q = float(POS_STOP_F)
            m.dq = float(VEL_STOP_F)
            m.kp = 0.0
            m.kd = 0.0
            m.tau = 0.0
            m.reserve = [0,0,0]
        return cmd

    def on_timer(self):
        enabled = bool(self.get_parameter("enabled").value)

        # Disabled OR no trajectory => send udp_init_send style command
        if (not enabled) or (self.last_traj is None) or (len(self.last_traj.joints_position) < 12):
            self.pub_low.publish(self.build_udp_init_send())
            return

        kp = float(self.get_parameter("kp").value)
        kd = float(self.get_parameter("kd").value)
        use_ff = bool(self.get_parameter("use_ff_torque").value)
        tau_limit = float(self.get_parameter("tau_limit").value)

        # Direct pass-through of trajectory (matches legged_control HW interface)
        q_ctrl = list(self.last_traj.joints_position[:12])
        
        # Handle case where joints_velocity might be None or too short
        if (self.last_traj.joints_velocity is not None and 
            len(self.last_traj.joints_velocity) >= 12):
            dq_ctrl = list(self.last_traj.joints_velocity[:12])
        else:
            dq_ctrl = [0.0]*12

        # Default no FF torque
        tau_ctrl = [0.0]*12
        if use_ff and (self.last_tau is not None) and len(self.last_tau) >= 12:
            tau_ctrl = [max(-tau_limit, min(tau_limit, float(x))) for x in self.last_tau[:12]]

        cmd = self.build_udp_init_send()  # start with safe defaults for all motors

        # Match C++ send_cmd loop:
        # for each HW motor index i, use ctrl index swap_i
        for i_hw in range(12):
            i_ctrl = self.swap_joint_indices[i_hw]
            
            # Bounds check on control indices
            if not (0 <= i_ctrl < 12):
                self.get_logger().error(f"Invalid control index {i_ctrl} for hw index {i_hw}")
                self.pub_low.publish(self.build_udp_init_send())
                return

            m = cmd.motor_cmd[i_hw]
            m.mode = 0x0A
            m.q  = float(q_ctrl[i_ctrl])
            m.dq = float(dq_ctrl[i_ctrl])
            m.kp = float(kp)
            m.kd = float(kd)
            m.tau = float(tau_ctrl[i_ctrl])

        self.pub_low.publish(cmd)

    def __del__(self):
        """Destructor: sends final safe stop command to hardware."""
        try:
            self.get_logger().info("Shutting down Go1 bridge - sending final safe stop command")
            self.shutdown_in_progress = True
            
            # Send safe stop command multiple times to ensure hardware receives it
            safe_cmd = self.build_udp_init_send()
            for _ in range(5):  # Redundancy: send 5 times
                self.pub_low.publish(safe_cmd)
            
            self.get_logger().info("Safe stop command sent")
        except Exception as e:
            # Suppress exceptions in destructor
            pass

def main():
    rclpy.init()
    node = Go1UnitreeStyleBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        try:
            node.set_parameters([Parameter("enabled", Parameter.Type.BOOL, False)])
            node.get_logger().info("Disabled bridge on shutdown (enabled:=false)")
            safe_cmd = node.build_udp_init_send()
            for _ in range(10):
                node.pub_low.publish(safe_cmd)
        except Exception:
            pass
    finally:
        # Send final safe stop command before shutdown
        try:
            safe_cmd = node.build_udp_init_send()
            for _ in range(10):  # Send multiple times
                node.pub_low.publish(safe_cmd)
            node.get_logger().info("Final safe stop sent on shutdown")
        except:
            pass
        
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
