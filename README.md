# Unitree Go1 – ROS2 to Real (PC → Hardware Quick Guide)

This guide shows the **minimal steps** to command a real **Unitree Go1**
from a **PC** using the **unitree_ros2** pipeline
(**high-level control**).

Tested on:
- Ubuntu 22.04
- ROS 2 Humble
- Wired Ethernet connection

---

## 1. Check network interfaces

```bash
ip a
```

Identify your **wired Ethernet interface**
(example: `enp0s31f6`, `eno1`, `enx...`).

---

## 2. Set static IP on the PC

Default Unitree Go1 network:
- **Robot IP:** `192.168.123.161`
- **PC IP:** `192.168.123.162`

Replace `<iface>` with your Ethernet interface name.

```bash
sudo ip addr flush dev <iface>
sudo ip addr add 192.168.123.162/24 dev <iface>
sudo ip link set <iface> up
```

Verify:
```bash
ip -4 addr show <iface>
ping 192.168.123.161
```

Ping **must succeed** before continuing.

---

## 3. Prepare the robot (IMPORTANT)

Using the **Unitree wireless controller**:

1. Power on the robot
2. Press **L2 + A** → robot stands (or it stands on power on automatically)
3. Release all sticks
4. (Recommended) **Turn the controller OFF** after standing

This puts the robot in **Sport / High-level API mode**, which allows
external commands from the PC.

---

## High-Level (Go1 Tools High)

### 4. Build the ROS2 workspace

```bash
source /opt/ros/humble/setup.bash
cd ~/unitree_ws
colcon build
source install/setup.bash
```

---

### 5. Launch the Unitree high-level bridge (from katie-hughes/unitree_ros2)

Terminal 1:
```bash
source ~/unitree_ws/install/setup.bash
ros2 launch unitree_legged_real high.launch.py
```

Expected:
- No errors
- Terminal keeps running
- Robot does **not** move yet

---

### 6. Run the official walking example (sanity check)

Terminal 2:
```bash
source ~/unitree_ws/install/setup.bash
ros2 run unitree_legged_real ros2_walk_example
```

Expected:
- Robot walks forward
- Confirms PC → robot command path works

---

### 7. Run the HighCmd profile test (single execution)
This test runs a fixed sequence of stand → walk → turn → stop.
Useful to verify high-level velocity tracking.

Terminal 2:
```bash
source ~/unitree_ws/install/setup.bash
ros2 run go1_tools_high ros2_highcmd_profile
```

Expected:
- Robot stands
- Walks forward
- Turns
- Stops 
- Program exits

---

### 8. Run the HighCmd looping profile test
This test continuously excites vx, vy, and wz in a loop.
Ideal for state-estimation (EKF) validation and data collection.

Terminal 2:
```bash
source ~/unitree_ws/install/setup.bash
ros2 run go1_tools_high ros2_highcmd_loop
```

Default behaviour:
- Runs infinite loops
- Publishes at 200 Hz
- Prints commanded velocities to terminal

```bash
ros2 run go1_tools_high ros2_highcmd_loop \
  --ros-args \
  -p loop_forever:=false \
  -p max_loops:=2 \
  -p vx_amp:=0.30 \
  -p vy_amp:=0.25 \
  -p wz_amp:=0.80 \
  -p t_hold:=1.5 \
  -p ramp_in:=0.2 \
  -p ramp_out:=0.2
```
---

---

### 9. Test EKF + loop + recording (PlotJuggler + optional rosbag)
This launch file loads the robot in high-level mode, runs the high command loop,
starts the EKF, launches PlotJuggler with the saved layout, and optionally records a rosbag.

```bash
ros2 launch go1_tools_high ekf_highcmd_loop_record.launch.py \
  max_loops:=2 \
  vx_amp:=0.20 \
  vy_amp:=0.15 \
  wz_amp:=0.40 \
  t_hold:=2.0 \
  ramp_in:=0.3 \
  ramp_out:=0.3 \
  start_delay_s:=3.0 \
  enable_record:=true
```

### 10. Verify ROS topics (optional)

```bash
ros2 topic list | grep high
ros2 topic echo /high_state --once
```

---

---

## Low-Level (Go1 Tools Low)

### Low-level testing (motor ping + joint mapping)
1) **Hang the robot** (legs off ground).
2) Launch low-level driver:
```bash
ros2 launch unitree_legged_real low.launch.py
```
3) Ping joints with the torque example (verify mapping):
```bash
ros2 run go1_tools_low ros2_torque_example --ros-args -p joint:=8 -p tau:=3.0 -p hold_sec:=0.3 -p rest_sec:=0.3
```
Repeat for other joints as needed.

### MPC controllers (hardware)
1) **Robot on the ground** (crouching on the floor mat, safe area).
2) Launch low-level bringup (EKF (and SRB based velocity estimator) + low-state converter + PlotJuggler):
```bash
ros2 launch go1_tools_low go1_pympc_bringup.launch.py
```
3) Check RViz/joint config; if joints are not updating, run a quick torque ping to start low-state publishing.
```bash
ros2 run go1_tools_low ros2_torque_example --ros-args -p joint:=8 -p tau:=3.0 -p hold_sec:=0.3 -p rest_sec:=0.3
```
4) Run the controller:
```bash
python3 ~/Quadruped-PyMPC/ros2/run_controller.py
```
5) In PlotJuggler, verify joint torques, MPC GRFs, references, and robot state are sane.
6) Start the PD bridge with the **hardware‑tested gains**:
```bash
ros2 run go1_tools_low pympc_pd_bridge.py --ros-args -p enabled:=true -p use_ff_torque:=true -p kp:=0.0 -p kd:=3.0 -p tau_limit:=20.0
```
Note: **Do not change `kp`** (0.0) for hardware; `kd=3.0` works best so far (aligned with legged_control).

## Stop the robot
- Press `Ctrl+C` in pd_bridge terminal to stop publishing torques

---
