# Bunker self-exploration startup

This document covers only the optional frontier self-exploration layer. It does
not replace the normal Bunker RTAB-Map/Nav2 startup. Start it only after the
manual dot-click Nav2 goal can move the robot correctly.

Use this after the stable multi-terminal stack is running: Terminal 1 combined
camera/hardware/EKF, RTAB-Map mapping or localization, and Nav2 drive mode.
The current preferred explorer is the upstream `frontier_exploration_ros2`
MRTSP/DP explorer, wrapped by
`frontier_exploration_ros2_bunker.launch.py`. The older local Python explorer
launches remain available for focused fallback/debugging.

The upstream explorer starts cold-idle by default. Then start movement:

```bash
ros2 run frontier_exploration_ros2 frontier_exploration_ctl start
```

The lower-level launch commands below are still useful for focused debugging.

## Required stack before launching

The following must already be running and healthy:

- Terminal 1 combined hardware: RealSense RGB, aligned depth, point cloud,
  Bunker driver raw `/wheel/odom`, EKF filtered `/odom`, `odom -> base_link`,
  robot TF.
- RTAB-Map mapping mode if creating a new map, or localization mode if using an
  existing map.
- Nav2 in `drive` mode with `allow_motion:=true`.
- `/safety_stop` is `false` when the route is clear.
- Bunker is in CAN control mode and has no blocking vehicle error.

Minimum checks:

```bash
ros2 topic hz /map
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo map base_link
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 topic echo --once /safety_stop
ros2 action list | grep navigate_to_pose
```

If a short manual clicked-dot goal does not move the robot, do not start
self-exploration yet.

## Install/build dependency

The upstream package needs `nav2_msgs` for the Nav2 `NavigateToPose` action.
If `ros2 pkg prefix nav2_msgs` fails inside the Docker shell, install it as root
or in the container image before building:

```bash
apt-get update
apt-get install -y ros-humble-nav2-msgs
```

Build the updated packages:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  frontier_exploration_ros2 \
  bunker_slam_bringup
source /ros2_ws/install/setup.bash
```

## Preferred upstream MRTSP explorer

The Bunker wrapper uses:

```text
map_topic: /map
costmap_topic: /global_costmap/costmap
local_costmap_topic: /local_costmap/costmap
navigate_to_pose_action_name: /navigate_to_pose
global_frame: map
robot_base_frame: base_link
autostart: false
control_service_enabled: true
front_sensor_frontier_filter_enabled: true
front_sensor_frontier_fov_deg: 87.0
front_sensor_frontier_allow_fallback: false
```

The front D435i pointcloud has a limited depth field of view. The wrapper uses
an 87 degree horizontal front cone for frontier selection and holds exploration
instead of selecting a rear/blindspot target when no front-facing frontier is
available.

Launch it as a sidecar:

```bash
ros2 launch bunker_slam_bringup frontier_exploration_ros2_bunker.launch.py \
  autostart:=false \
  control_service_enabled:=true
```

Start a short first run only after Nav2 is in drive mode and a manual goal
worked:

```bash
ros2 run frontier_exploration_ros2 frontier_exploration_ctl start
ros2 run frontier_exploration_ros2 frontier_exploration_ctl stop -t 120
```

Stop immediately:

```bash
ros2 run frontier_exploration_ros2 frontier_exploration_ctl stop
```

The tmux Nav-Man launcher starts this as `t13_frontier_mrtsp` in cold-idle mode.

## Clean restart helper

If the graph has stale Nav2, RTAB-Map, RViz, camera, showcase, or PiPER helper
nodes, stop them before restarting:

```bash
ros2 run bunker_slam_bringup pkill_bunker_nav2_stack.sh
```

This is a plain `pkill -f` cleanup script. It also restarts the ROS 2 daemon so
stale topic names disappear.

After cleanup, `/parameter_events` and `/rosout` are normal ROS 2 topics and do
not mean a robot node is still running.

Do not run the standalone URDF showcase at the same time as exploration. It
publishes visual-only TF/joint states and can duplicate `robot_state_publisher`.

Plain `pkill` cleanup, no ROS script:

```bash
pkill -TERM -f 'bunker_piper_3d_showcase.launch.py|showcase_static_map_to_base_link' || true
pkill -TERM -f 'terminal1_sensors.launch.py' || true
pkill -TERM -f 'rtabmap_front_camera.launch.py|nav2_bringup.launch.py|frontier_exploration.*launch.py' || true
pkill -TERM -f 'aruco_landmark_exploration.launch.py|landmark_navigator.launch.py|nav2_status_monitor.launch.py' || true
pkill -TERM -f 'amcl_localization.launch.py|ekf_h30.launch.py' || true
pkill -TERM -f 'realsense2_camera_node|rgbd_sync|rtabmap' || true
pkill -TERM -f 'controller_server|planner_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|smoother_server|lifecycle_manager' || true
pkill -TERM -f 'cmd_vel_mux_node|clicked_point_nav_goal|depth_route_monitor_node|sensor_fusion_node' || true
pkill -TERM -f 'frontier_explorer_node|aruco_landmark_node|landmark_navigator_node' || true
pkill -TERM -f 'bunker_base_node|robot_state_publisher|joint_state_prefixer|offline_initial_pose_publisher|static_transform_publisher' || true
pkill -TERM -f 'ekf_node|yesense_node_publisher|rviz2' || true
pkill -TERM -f 'piper_x_joint_preset|piper_navigation_pose|agx_arm_ctrl_single' || true
sleep 2
ros2 daemon stop || true
ros2 daemon start || true
```

## How the start button works

Launching the explorer does not immediately move the robot. The explorer waits
for:

```bash
ros2 service call /frontier_explorer/start std_srvs/srv/Trigger "{}"
```

That service call does two things:

1. Captures the current `map -> base_link` pose as `home`.
2. Starts the exploration timer or frontier-ratio completion logic.

Manual stop:

```bash
ros2 service call /frontier_explorer/stop std_srvs/srv/Trigger "{}"
```

## Legacy local Python explorer dry-run

This is the older local Python explorer. Dry-run logs selected frontier goals
but does not send them to Nav2.

```bash
ros2 launch bunker_slam_bringup frontier_exploration.launch.py \
  dry_run:=true \
  completion_mode:=time_limit \
  time_limit_sec:=120.0 \
  no_frontier_return_cycles:=2 \
  return_to_start:=true
```

Then start:

```bash
ros2 service call /frontier_explorer/start std_srvs/srv/Trigger "{}"
```

Check that selected frontier goals are reasonable before enabling movement.

## Mode 1: 2-minute time limit

Use this first for real movement.

```bash
ros2 launch bunker_slam_bringup frontier_exploration_drive.launch.py \
  completion_mode:=time_limit \
  time_limit_sec:=120.0 \
  frontier_cluster_min_cells:=3 \
  no_frontier_return_cycles:=2 \
  return_to_start:=true
```

Start:

```bash
ros2 service call /frontier_explorer/start std_srvs/srv/Trigger "{}"
```

Behavior:

```text
capture home -> explore frontier goals -> after 120 seconds return home -> stop
```

## Mode 2: frontier-ratio threshold

Use this after the time-limit mode is stable.

```bash
ros2 launch bunker_slam_bringup frontier_exploration_drive.launch.py \
  completion_mode:=frontier_ratio \
  frontier_stop_ratio:=0.20 \
  frontier_cluster_min_cells:=3 \
  no_frontier_return_cycles:=2 \
  return_to_start:=true
```

Start:

```bash
ros2 service call /frontier_explorer/start std_srvs/srv/Trigger "{}"
```

Behavior:

```text
capture home -> explore frontier goals -> stop when remaining frontier ratio is <= 20% -> return home -> stop
```

The 20% value is an initial tuning value. It may return early or late depending
on how RTAB-Map marks unknown cells in the current map.

## Current frontier failure behavior and proposed upgrade

Current behavior:

```text
find nearest usable frontier
-> send Nav2 NavigateToPose goal
-> if Nav2 fails with status=6, blacklist that frontier
-> immediately try another frontier
```

This can fail repeatedly if the robot is currently in a poor position, the local
costmap is blocked, or Nav2 cannot plan from the current pose. In the current
default direct command route, `/cmd_vel` will remain empty or zero. If Nav2 was
started with the legacy `use_cmd_vel_mux:=true` route, the command mux may hold
zero with `watchdog_timeout` because Nav2 stops sending fresh controller
commands.

Recommended next algorithm:

```text
find candidate frontiers
-> prefer candidates that Nav2 can plan to
-> if a frontier fails:
     blacklist it
     go to last_known_clear_pose or home
     wait until /safety_stop is false
     select a new frontier from the updated map
```

This is the safer behavior for the current robot. It avoids continuously
chasing new frontier points from a blocked/bad pose.

Implementation options:

| Option | What changes | Priority |
|---|---|---|
| Plan-verified frontier | Call Nav2 planner for several frontier candidates before sending the real movement goal | Highest |
| Recovery-to-clear-pose | On repeated status `6`, watchdog, or safety stop, send robot to home or last known-clear pose before retrying | Highest |
| Clearance-biased scoring | Prefer frontier goals surrounded by more known-free cells | Medium |
| Viewpoint scoring | Pick a free pose that faces the unknown area from a safe distance | Medium |
| Larger failed-goal blacklist | Avoid retrying goals near failed frontiers | Low, already partly implemented |

## Parameters

| Parameter | Default | Meaning |
|---|---:|---|
| `dry_run` | `true` | Log goals without sending motion goals |
| `completion_mode` | `time_limit` | `time_limit`, `frontier_ratio`, or `aruco_found` |
| `time_limit_sec` | `120.0` | Exploration duration for time-limit mode |
| `frontier_stop_ratio` | `0.20` | Stop threshold for frontier-ratio mode |
| `frontier_max_distance_m` | `4.0` | Maximum selected frontier distance |
| `frontier_min_distance_m` | `0.75` | Avoid tiny goals too close to the robot |
| `frontier_cluster_min_cells` | `3` in drive wrapper, `12` in base launch | Ignore small/noisy frontier clusters |
| `no_frontier_return_cycles` | `2` | Return home only after this many no-frontier scans |
| `return_to_start` | `true` | Return to captured home when complete |

## Mode 3: ArUco marker found

Use this when ArUco marker detection is stable from Terminal 1 or the combined
ArUco exploration launch.

```bash
ros2 launch bunker_slam_bringup aruco_landmark_exploration.launch.py \
  image_topic:=/front_camera/color/image_raw \
  camera_info_topic:=/front_camera/color/camera_info \
  target_marker_ids:=6 \
  marker_size_m:=0.05 \
  required_seen_count:=3 \
  completion_mode:=aruco_found \
  dry_run:=false \
  return_to_start:=true
```

Start:

```bash
ros2 service call /frontier_explorer/start std_srvs/srv/Trigger "{}"
```

Behavior:

```text
capture home -> explore frontier goals -> ArUco marker confirmed -> save landmark -> return home -> stop
```

For multiple doors, prefer `completion_mode:=time_limit` so the robot can record
more than one marker before selecting the furthest door later.

## Navigate to saved ArUco/home landmarks

After exploration has saved `/ros2_ws/maps/aruco_landmarks.json`, keep Nav2
running and start:

If you used `aruco_landmark_exploration.launch.py`, this is already running by
default because `start_landmark_navigator:=true`.

```bash
ros2 launch bunker_slam_bringup landmark_navigator.launch.py \
  landmark_path:=/ros2_ws/maps/aruco_landmarks.json
```

Go home:

```bash
ros2 service call /landmark_navigator/go_home std_srvs/srv/Trigger "{}"
```

Go to the furthest valid marker:

```bash
ros2 service call /landmark_navigator/go_furthest std_srvs/srv/Trigger "{}"
```

Go to marker ID `6`:

```bash
ros2 topic pub --once /landmark_navigator/go_marker std_msgs/msg/String "{data: 'aruco_6'}"
```

## Colored Nav2 status terminal

Open another pane while Nav2 or self-exploration is running:

```bash
ros2 launch bunker_slam_bringup nav2_status_monitor.launch.py
```

It shows:

- current Nav2 action state;
- `/safety_stop` and safety reason;
- Bunker control mode, vehicle state and error code;
- `/map`, `/odom`, and `/cmd_vel` freshness;
- raw Nav2 command, smoothed command and final command to Bunker;
- whether the robot is moving forward, reversing, turning left/right or stopped.

Colors:

```text
GREEN  healthy / moving / clear
YELLOW idle / stale / waiting
RED    blocked / missing / unsafe
```

## Why a white clicked area may not route

White in the occupancy grid means free in the map display. Nav2 still plans
against the costmap, which includes inflation, unknown-space policy, robot
footprint and live obstacles. A visually white cell can fail if:

- the inflated robot footprint overlaps an obstacle;
- unknown space blocks the corridor;
- the clicked cell is outside the active costmap;
- `map -> base_link` TF is stale or missing;
- `/safety_stop` blocks motion;
- the Bunker driver is not accepting `/cmd_vel`.

Self-exploration depends on the same planner and controller. It cannot fix a
manual Nav2 routing or Bunker CAN/control-mode problem.
