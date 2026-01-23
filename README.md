# Unitree Go1 – ROS2 Workspace

This workspace contains the high-level and low-level tools for Unitree Go1,
plus the ROS 2 integration required to run controllers from a PC.

## Workspace Install
```bash
source /opt/ros/humble/setup.bash
cd ~/unitree_ws
colcon build
source install/setup.bash
```

## Network Setup (Wired)
Default Unitree Go1 network:
- Robot: `192.168.123.161`
- PC: `192.168.123.162`

```bash
sudo ip addr flush dev <iface>
sudo ip addr add 192.168.123.162/24 dev <iface>
sudo ip link set <iface> up
ping 192.168.123.161
```

## Robot Setup
Use the wireless controller:
1) Power on
2) **L2 + A** to stand
3) Release sticks
4) (Recommended) turn the controller off

This puts the robot in **sport/high-level mode**.

## High-Level Mode
See: `src/go1_tools_high/README.md`

Quick start:
```bash
ros2 launch unitree_legged_real high.launch.py
ros2 run go1_tools_high ros2_highcmd_profile
```

## Low-Level Mode
See: `src/go1_tools_low/README.md`

Quick start:
```bash
ros2 launch unitree_legged_real low.launch.py
ros2 launch go1_tools_low go1_pympc_bringup.launch.py
python3 ~/Quadruped-PyMPC/ros2/run_controller.py
ros2 run go1_tools_low pympc_pd_bridge.py --ros-args -p enabled:=true -p use_ff_torque:=true -p kp:=0.0 -p kd:=3.0 -p tau_limit:=20.0
```

## Low-Level Framework
![Low-level framework](docs/images/low_tools_framework.jpg)

## Low-Level TODO Map
![Low-level TODOs](docs/images/low_tools_pain_tasks.jpg)

## Hardware Notes
- Low-level control is sensitive to latency. Prefer wired Ethernet.
- The PD bridge publishes at 500 Hz (Unitree servo loop).

## Stop the Robot
- Press `Ctrl+C` in the PD bridge terminal to stop publishing torques.

---
