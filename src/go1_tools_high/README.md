# go1_tools_high

ROS 2 tools for **Unitree Go1 high-level (sport) mode**. This package focuses on
velocity-profile testing and EKF validation in high-level control.

## What It Provides
- HighCmd profile node (`ros2_highcmd_profile`)
- HighCmd loop node (`ros2_highcmd_loop`) for continuous excitation
- High state converter (`go1_high_state_converter.py`) for /twist and high state outputs
- Launch: `ekf_highcmd_loop_record.launch.py` (EKF + loop + PlotJuggler + optional rosbag)

## Typical Usage
1) Launch Unitree high-level driver:
```
ros2 launch unitree_legged_real high.launch.py
```

2) Run a single high-level profile:
```
ros2 run go1_tools_high ros2_highcmd_profile
```

3) Run continuous loop test:
```
ros2 run go1_tools_high ros2_highcmd_loop
```

4) EKF + loop + logging:
```
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

## Notes
- High-level mode expects the robot to be in **sport/high mode** (standing).
- The loop nodes default to **200 Hz**.

## TODO 📝
- Add a high-level motion planner using **density functions**.
- Integrate camera + lidar for mapping and navigation.
- Follow https://github.com/katie-hughes/brne_social_nav to set up the ZED cam and publish obstacle position/velocity for the density-based planner.
