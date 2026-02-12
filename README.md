# DWA Planner

 DWA local planner node for TurtleBot3.

## Features
- Subscribes to:
  - `/odom`
  - `/scan`
- Accepts goal from RViz 2D Goal Pose topics:
  - `/goal_pose`
  - `/move_base_simple/goal`
- Publishes:
  - `/cmd_vel`
  - `/dwa_markers` (all sampled trajectories + best trajectory)
- Includes simple safety behavior:
  - stop-and-turn near front obstacle
  - recovery turn when no valid trajectory is found

## Build
```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select dwa_planner
source install/setup.bash
```

## Run
1. Start Gazebo and RViz (your normal TurtleBot3 simulation launch).
2. Run planner node:

```bash
ros2 run dwa_planner dwa_planner_node
```

3. In RViz, use **2D Goal Pose**.

## Useful checks
Verify node is receiving data:
```bash
ros2 topic echo /odom
ros2 topic echo /scan
ros2 topic echo /goal_pose
ros2 topic echo /move_base_simple/goal
```

Verify output command:
```bash
ros2 topic echo /cmd_vel
```

## Common runtime tuning
You can tune behavior without changing code:

```bash
ros2 run dwa_planner dwa_planner_node --ros-args \
  -p occdist_scale:=1.2 \
  -p turn_weight:=1.0 \
  -p max_rot_vel:=3.0 \
  -p acc_lim_theta:=5.0 \
  -p safety_stop_distance:=0.30 \
  -p safety_slow_distance:=0.90 \
  -p safety_turn_speed:=1.8
```

