# Bunker RTAB-Map and Nav2 VS Code terminal-editor runbook

This is the maintained startup procedure for the Bunker + dual PiPER + dual D435i Nav2 workflow.
The URDF uses the two physical PiPER plate positions: front at X = +0.356 m and
rear at X = -0.356 m. The front PiPER base yaw is 180 degrees, and the rear
PiPER keeps its rear-facing -90 degree yaw.

Use this runbook for:

- Docker-based ROS 2 Humble startup.
- Front camera topics under `/front_camera/*`.
- Rear camera topics under `/rear_camera/*`.
- Front D435i serial `243322074578`.
- Rear D435i serial `261222077434`.
- RTAB-Map mapping and localization.
- Bunker `/odom` from the base driver.
- Safety-gated Nav2 dry-run and drive modes.
- RViz goal control.

This file is documentation only. It does not change launch behavior.

## Start Here: Clean Nav2 Startup Order

Follow this order from a fresh VS Code terminal session. Do not start mapping,
localization, Nav2, RViz, rqt, or exploration until the earlier checks pass.

### Step 0: Host Display And Docker Shell

On the robot desktop host:

```bash
export DISPLAY=:1
xhost +SI:localuser:root
code
```

Open a VS Code integrated terminal, then enter Docker:

```bash
docker exec -it \
  -e DISPLAY=:1 \
  -e QT_X11_NO_MITSHM=1 \
  -e QT_QPA_PLATFORM=xcb \
  trystan-bunker-navigation bash
```

Current RealSense serials checked with `rs-enumerate-devices -s`:

```text
front camera /front_camera  243322074578  old known D435i, mapping RGB-D camera
rear camera  /rear_camera   261222077434  newer second D435i
```

### Step 0.5: Rebuild After Any Source Change

For normal launch/config/RViz/URDF/script/documentation edits, use the fast
refresh path instead of a full build:

```bash
cd /ros2_ws
/ros2_ws/src/bunker_slam_bringup/scripts/refresh_without_colcon.sh
source /ros2_ws/install/setup.bash
```

Then restart the affected launch. Running ROS nodes do not live-reload launch
files, Python files, configs, or URDF files.

This refresh also fixes stale Docker install copies by relinking the active
`install/.../share/...` launch, config, RViz and URDF folders directly to
`/ros2_ws/src/...`. Edit files under `/ros2_ws/src`, not under `/ros2_ws/install`
or `/ros2_ws/build`.

You still need `colcon build` whenever you edit `setup.py`, `package.xml`,
`CMakeLists.txt`, package dependencies, console-script entry points, or when you
add a new file that must be installed as a ROS executable/resource:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  bunker_slam_bringup \
  bunker_dual_piper_nav2 \
  bunker_autonomy \
  --symlink-install
source /ros2_ws/install/setup.bash
```

For a full workspace rebuild after broader dependency changes:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source /ros2_ws/install/setup.bash
```

After every successful build, restart affected launches. A running ROS process
does not automatically reload changed launch files, Python code, configs, or
URDF files.

### Step 1: ROS Environment In Every Pane

Run this at the top of every Docker terminal pane:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=173
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

export DISPLAY=:1
export XAUTHORITY=/tmp/.docker.xauth
export QT_QPA_PLATFORM=xcb
export QT_X11_NO_MITSHM=1
export XDG_RUNTIME_DIR=/tmp/xdg-runtime
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"
```

Keep `ROS_DOMAIN_ID` and `ROS_LOCALHOST_ONLY` identical in every pane. If one
pane uses `ROS_LOCALHOST_ONLY=0` and another uses `1`, the graph can look empty
even while nodes are running.

### Step 2: Clean Old Processes

Preferred cleanup:

```bash
ros2 run bunker_slam_bringup pkill_bunker_nav2_stack.sh
ros2 topic list
```

If only PiPER/URDF/TF topics remain, such as
`/front_piper/feedback/joint_states`, `/rear_piper/feedback/joint_states`,
`/front_piper/robot_description`, `/rear_piper/robot_description`,
`/robot_description`, `/joint_states`, `/tf`, or `/tf_static`, use the targeted
PiPER description cleanup:

```bash
ros2 run bunker_slam_bringup pkill_piper_description_topics.sh
ros2 topic list
```

After a clean kill, this is normal:

```text
/parameter_events
/rosout
```

If the cleanup script is unavailable, use the manual cleanup block:

```bash
LAUNCHES='terminal1_sensors.launch.py|terminal2_robot.launch.py|terminal3_mapping.launch.py|system_bringup.launch.py|hardware_drivers.launch.py|description.launch.py|dual_realsense.launch.py|rtabmap_front_camera.launch.py|rtabmap.launch.py|nav2_bringup.launch.py|nav2_rtabmap.launch.py|amcl_localization.launch.py|amcl_bringup.launch.py|ekf_h30.launch.py|h30_imu.launch.py|bunker_piper_3d_showcase.launch.py|frontier_exploration.*launch.py|aruco_landmark_exploration.launch.py|landmark_navigator.launch.py|nav2_status_monitor.launch.py'
NODES='realsense2_camera_node|rgbd_sync|rgbdx_sync|rgbd_odometry|point_cloud_xyzrgb|rtabmap|map_server|amcl|controller_server|planner_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|smoother_server|lifecycle_manager|cmd_vel_mux_node|nav2_cmd_vel_safety_mux|clicked_point_nav_goal|nav2_arrival_manipulation_trigger|nav2_manipulation_handoff|nav2_status_monitor|abotclaw_handoff_service|abotclaw_manipulation_lifecycle_listener|front_piper_description_bridge|agent_server/server.py|front_piper_move_group_only|touch_marker_full_stack|search_marker_node|piper_touch_marker_api|depth_route_monitor_node|sensor_fusion_node|safety_monitor_node|frontier_explorer_node|aruco_landmark_node|landmark_navigator_node|bunker_base_node|robot_state_publisher|joint_state_prefixer|joint_state_publisher|static_transform_publisher|ekf_node|imu_bias_corrector.py|yesense_node_publisher|piper_x_joint_preset|piper_navigation_pose|agx_arm_ctrl_single|agx_arm_ctrl_single_node|detector_tracker_node|target_mux_node|depth_geometry_memory_node|semantic_memory_node|rviz2'
pkill -TERM -f "$LAUNCHES" || true
pkill -TERM -f "$NODES" || true
sleep 2
pkill -KILL -f "$LAUNCHES" || true
pkill -KILL -f "$NODES" || true
ros2 daemon stop || true
ros2 daemon start || true
ros2 node list
ros2 topic list
```

### Step 3: Verify CAN By Serial Before Launching ROS Drivers

Never trust only `can2`, `can3`, or `can4`; confirm serials:

```bash
for n in /sys/class/net/can*; do
  [ -e "$n" ] || continue
  n="$(basename "$n")"
  echo "=== $n ==="
  cat /sys/class/net/$n/device/../serial 2>/dev/null || true
  ip -details -statistics link show "$n" | sed -n '1,14p'
done
```

Current expected mapping:

```text
front PiPER  can2  serial 004E002B4148570A20343133  1000000 bit/s
rear PiPER   can3  serial 0036001F4148570A20343133  1000000 bit/s
Bunker       can4  serial 001D00255443570A20393433  500000 bit/s
```

Configure CAN:

```bash
ros2 run bunker_slam_bringup configure_can.sh can2 can4 can3
```

Check passive traffic:

```bash
timeout 5 candump -L can2
timeout 5 candump -L can3
timeout 5 candump -L can4
```

Do not continue to Nav2 if Bunker `can4` is `ERROR-PASSIVE`, `BUS-OFF`, has
`RTNETLINK answers: Broken pipe`, or shows no Bunker frames in `candump`.

### Step 4: Start Terminal 1 Hardware

Run this once:

```bash
ros2 launch bunker_slam_bringup terminal1_sensors.launch.py \
  arm_can:=can2 \
  bunker_can:=can4 \
  rear_piper_can:=can3 \
  front_piper_can:=can2 \
  launch_front_camera:=true \
  launch_rear_camera:=true \
  front_camera_serial:=243322074578 \
  rear_camera_serial:=261222077434 \
  configure_can:=true \
  start_piper_drivers:=true \
  piper_control_enabled:=true \
  run_piper_initial_pose:=true \
  allow_piper_motion:=true \
  h30_serial_port:=/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0003-if00 \
  h30_baud_rate:=460800 \
  h30_frame_id:=imu_link \
  start_ekf:=false
```

Terminal 1 keeps the PiPER external control gate enabled by default. The
front/rear services are `/front_piper/control_enable` and
`/rear_piper/control_enable`; both are started with `piper_control_enabled:=true`
unless you explicitly override that launch argument to `false`.

### Step 5: Hardware Health Checks

Run these from a separate diagnostic pane:

```bash
rs-enumerate-devices -s

ros2 topic list | grep -E '^/front_camera/color/image_raw$|^/front_camera/aligned_depth_to_color/image_raw$|^/front_camera/depth/color/points$|^/rear_camera/color/image_raw$|^/rear_camera/aligned_depth_to_color/image_raw$|^/rear_camera/depth/color/points$|^/front_rgbd_image$|^/rear_rgbd_image$|^/rgbd_images$'

ros2 topic info -v /front_camera/color/image_raw
ros2 topic info -v /rear_camera/color/image_raw

ros2 topic hz /front_camera/color/image_raw
ros2 topic hz /front_camera/aligned_depth_to_color/image_raw
ros2 topic info /front_camera/depth/color/points
ros2 topic hz /rear_camera/color/image_raw
ros2 topic hz /rear_camera/aligned_depth_to_color/image_raw
ros2 topic info /rear_camera/depth/color/points

ros2 topic info /wheel/odom
ros2 topic echo /wheel/odom --once
ros2 topic info /odom
ros2 topic hz /odom

ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link front_camera_color_optical_frame
ros2 run tf2_ros tf2_echo base_link rear_camera_color_optical_frame
ros2 run tf2_ros tf2_echo base_link front_piper_base_link
ros2 run tf2_ros tf2_echo base_link rear_piper_base_link
```

Do not start mapping, localization, or Nav2 until `/wheel/odom`, `/odom`, and
`odom -> base_link` work.

`rs-enumerate-devices -s` must show both D435i serials before the RealSense
launch can publish images. If it prints `No device detected. Is it plugged in?`,
Docker cannot see the cameras yet, so RViz will show `No Image` even when the
topic names are configured correctly.

If `/front_camera/color/image_raw` or `/rear_camera/color/image_raw` appears in
`ros2 topic list` but `ros2 topic info -v` shows `Publisher count: 0`, RViz will
show `No Image`. Restart the camera launch after confirming Step 1 used
`ROS_DOMAIN_ID=173` and `ROS_LOCALHOST_ONLY=1`.

### Step 5.1: Mapping And Manipulation Camera Readiness

Topic names alone are not enough. For mapping and manipulation, all raw camera
inputs must have real publishers and rates:

```bash
# Optional cleanup if another Docker stack was using/subscribing to camera topics.
docker exec iliyas-abot bash -lc \
  'pkill -TERM -f "aruco_ros/single|piper_touch_marker_api|touch_marker_full_stack|search_marker|wall_approach" || true' 2>/dev/null || true

# Stop old RealSense/RTAB-Map camera test processes in this Docker before a clean test.
ps -ef | grep -E 'realsense2_camera_node|dual_realsense|rgbd_sync|rtabmap' | grep -v grep
# Kill exact PIDs from the command above if any are stale.

rs-enumerate-devices -s

for t in \
  /front_camera/color/image_raw \
  /front_camera/color/camera_info \
  /front_camera/aligned_depth_to_color/image_raw \
  /front_camera/depth/color/points \
  /rear_camera/color/image_raw \
  /rear_camera/color/camera_info \
  /rear_camera/aligned_depth_to_color/image_raw \
  /rear_camera/depth/color/points \
  /front_rgbd_image \
  /rear_rgbd_image \
  /rgbd_images; do
  echo "=== $t ==="
  ros2 topic info "$t"
done

ros2 topic hz /front_camera/color/image_raw
ros2 topic hz /front_camera/aligned_depth_to_color/image_raw
ros2 topic hz /front_camera/depth/color/points
ros2 topic hz /rear_camera/color/image_raw
ros2 topic hz /rear_camera/aligned_depth_to_color/image_raw
ros2 topic hz /rear_camera/depth/color/points
ros2 topic hz /front_rgbd_image
ros2 topic hz /rear_rgbd_image
ros2 topic hz /rgbd_images
```

Pass criteria:

- `rs-enumerate-devices -s` lists both D435i serials.
- Front and rear RGB image, RGB camera info, aligned depth, and point cloud
  topics each show `Publisher count: 1` or higher.
- `/front_rgbd_image` and `/rear_rgbd_image` produce messages.
- RTAB-Map `/map`, `/grid_prob_map`, or `/cloud_map` produces messages after
  the robot has camera input and odometry.

If `/front_rgbd_image` or `/rear_rgbd_image` has a publisher but `ros2 topic hz`
shows no messages, the sync node is alive but missing raw RGB/depth/camera-info
inputs. Fix the raw RealSense publishers first.

Camera-only launch test:

```bash
ros2 launch bunker_dual_piper_nav2 dual_realsense.launch.py
```

In another pane, verify:

```bash
ros2 node list | grep -E '/front_camera$|/rear_camera$'
ros2 topic info /front_camera/color/image_raw
ros2 topic info /rear_camera/color/image_raw
ros2 topic hz /front_camera/color/image_raw
ros2 topic hz /rear_camera/color/image_raw
```

If both RealSense nodes exit with `wait for device timeout`, ROS is not the
problem yet. Check host/container USB enumeration first:

```bash
lsusb | grep -i -E 'intel|realsense|8086'
ls -l /dev/video*
rs-enumerate-devices -s
```

Live Docker check on 2026-08-14 in `trystan-bunker-navigation`:

- Terminal 1, both PiPER drivers, robot TF, Bunker `/odom`, RTAB-Map, and both
  RGB-D sync nodes were running at the same time.
- CAN links were healthy: `can2` and `can3` at 1 Mbit/s for PiPER, `can4` at
  500 kbit/s for Bunker, all `ERROR-ACTIVE`.
- `/odom` published around 30 Hz and `/joint_states` published from the dual
  PiPER prefixer.
- The camera topic names existed, but front/rear RGB, camera info, aligned
  depth, and point cloud topics all showed `Publisher count: 0`.
- `rs-enumerate-devices -s` printed `No device detected. Is it plugged in?`.
- Latest RealSense logs showed both `front_camera` and `rear_camera` exiting
  with `wait for device timeout of 10 secs expired`.

Conclusion for that check: the full startup can run together, but live RGB-D
mapping and manipulation camera perception were **not ready** because Docker
could not see either RealSense camera.

### Step 6: Start Mapping Or Localization

`/map` is published by the RTAB-Map node named `/rtabmap`, not by Nav2. In
this system, `rtabmap_front_camera.launch.py` supports
`mapping_camera:=front`, `mapping_camera:=rear`, or `mapping_camera:=both`.
The default is `both`.

With `mapping_camera:=both`, it starts:

- `/front_rgbd_sync`, subscribing to `/front_camera/color/image_raw`,
  `/front_camera/aligned_depth_to_color/image_raw`, and
  `/front_camera/color/camera_info`, then publishing `/front_rgbd_image`.
- `/rear_rgbd_sync`, subscribing to `/rear_camera/color/image_raw`,
  `/rear_camera/aligned_depth_to_color/image_raw`, and
  `/rear_camera/color/camera_info`, then publishing `/rear_rgbd_image`.
- `/rgbdx_sync`, subscribing to `/front_rgbd_image` and `/rear_rgbd_image`,
  then publishing the combined `/rgbd_images` stream.
- `/rtabmap`, subscribing to `/rgbd_images`
  plus `/odom`, then publishing `/map`, `/grid_prob_map`, `/cloud_map`,
  `/mapData`, `/mapGraph`, and `map -> odom`.

The RTAB-Map config sets `subscribe_rgbd: true` and `rgbd_cameras: 0`.
That makes `/rtabmap` use the `RGBDImages` interface from `/rgbd_images`.
This is required because the installed RTAB-Map build cannot directly
synchronize two separate RGB-D topics internally. If either per-camera RGB-D
stream has no messages, `/rgbd_images` will stop and `/map` may exist as a topic
but will not produce a useful live map.

With `mapping_camera:=front` or `mapping_camera:=rear`, it starts only that
camera's `rgbd_sync` node and lets `/rtabmap` subscribe directly to
`/front_rgbd_image` or `/rear_rgbd_image`. The rear mode uses the same
single-camera mapping path as the front mode, including
`Odom/ResetCountdown: 0`, so poor rear viewpoint/tracking pauses updates instead
of resetting the mapping session or deleting the database. Database deletion
only happens when `reset_database:=true`.

The config also sets `Vis/EstimationType: 0`, which means 3D->3D visual
registration. Keep this for dual-camera mapping on this Docker image. The
installed RTAB-Map binary was built without OpenGV, so multi-camera 3D->2D PnP
registration prints `Multi-camera 2D-3D PnP registration is only available if
rtabmap is built with OpenGV dependency`. Installing OpenGV after the fact is not
enough unless RTAB-Map itself is rebuilt against it.

For a new map:

```bash
ros2 launch bunker_dual_piper_nav2 rtabmap_front_camera.launch.py \
  mode:=mapping \
  database_path:=/ros2_ws/maps/bunker_dual_rgbd_v2.db \
  reset_database:=true \
  mapping_camera:=both

rviz2 -d /ros2_ws/install/bunker_slam_bringup/share/bunker_slam_bringup/rviz/camera_rgbd_mapping.rviz
```

For an existing map:

```bash
ros2 launch bunker_dual_piper_nav2 rtabmap_front_camera.launch.py \
  mode:=localization \
  database_path:=/ros2_ws/maps/bunker_dual_rgbd_v2.db \
  reset_database:=false \
  mapping_camera:=both
```

To extend an existing map using only the rear camera:

```bash
ros2 launch bunker_dual_piper_nav2 rtabmap_front_camera.launch.py \
  mode:=mapping \
  database_path:=/ros2_ws/maps/bunker_dual_rgbd_v2.db \
  reset_database:=false \
  mapping_camera:=rear
```

Check:

```bash
ros2 topic hz /front_rgbd_image
ros2 topic hz /rear_rgbd_image
ros2 node info /rtabmap | grep -E 'rgbd_image0|rgbd_image1|/odom' || true
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map base_link
```

### Step 7: Start Nav2 Dry Run Before Drive

Dry run first:

```bash
ros2 launch bunker_slam_bringup nav2_bringup.launch.py \
  mode:=dry_run \
  allow_motion:=false \
  use_rviz:=false \
  start_depth_safety:=true \
  start_clicked_goal:=true \
  start_landmark_navigator:=true \
  landmark_path:=/ros2_ws/maps/manual_nav_landmarks.json \
  start_manipulation_trigger:=false
```

Check:

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 topic echo --once /safety_stop
ros2 topic echo /cmd_vel_debug --once
```

Active Nav2 planner/control configuration:

```text
params_file: /ros2_ws/install/bunker_slam_bringup/share/bunker_slam_bringup/config/nav2_bunker.yaml
global planner: nav2_navfn_planner/NavfnPlanner with A*
controller: DWBLocalPlanner
recovery BT: navigate_to_pose_back_up_recovery.xml
recovery behavior: clear costmaps, wait
local costmap: front and rear RealSense point clouds
global costmap: static map plus front and rear RealSense point clouds
```

The active BT removes the old spin-first and backup recovery behavior. If Nav2
cannot follow a path, it clears costmaps and waits. During front-camera
exploration, DWB is forward-only; if no forward route stays in known-free space,
the goal should fail so frontier exploration can choose another area.

Confirm the planner/BT settings after a rebuild:

```bash
ros2 param get /planner_server planner_plugins
ros2 param get /planner_server GridBased.plugin
ros2 param get /planner_server GridBased.tolerance
ros2 param get /bt_navigator default_nav_to_pose_bt_xml
ros2 param get /controller_server FollowPath.min_vel_x
ros2 param get /velocity_smoother min_velocity
ros2 param get /behavior_server behavior_plugins
```

Expected:

```text
GridBased.plugin: nav2_navfn_planner/NavfnPlanner
GridBased.allow_unknown: false
GridBased.tolerance: 0.05
FollowPath.min_vel_x: 0.0
min_velocity: [0.0, 0.0, -0.375]
behavior_plugins: [drive_on_heading, wait]
```

The planner is configured to avoid unknown/purple map space and route through
known-free blue cells. The local and global costmaps use `inflation_radius:
0.35`, and DWB's `BaseObstacle.scale` is raised so local tracking avoids
inflated/purple cost where a free alternative exists. Reverse motion and the
BT backup behavior are disabled for front-camera exploration. If RViz still
shows a path through purple, confirm you are displaying `/global_costmap/costmap`
and that the active `/planner_server` parameters match the rebuilt config.

Only after dry-run goals look correct, stop the Nav2 pane and restart in drive
mode:

```bash
ros2 launch bunker_slam_bringup nav2_bringup.launch.py \
  mode:=drive \
  allow_motion:=true \
  use_rviz:=false \
  start_depth_safety:=true \
  start_clicked_goal:=true \
  start_landmark_navigator:=true \
  landmark_path:=/ros2_ws/maps/manual_nav_landmarks.json \
  start_manipulation_trigger:=false
```

If a manipulation task should start automatically after the robot reaches the
saved `door` landmark, set `start_manipulation_trigger:=true`. This starts
`/nav2_arrival_manipulation_trigger`, which watches
`/door_navigation/arrived` for a rising-edge `std_msgs/msg/Bool` `true` and
publishes a JSON arrival event on `/front_piper/task/start` plus a simple bool on
`/front_piper/task/start_bool`.

The communication path to `iliyas-abot` is ROS 2 topic based:

```text
/landmark_navigator -> /door_navigation/arrived
/landmark_navigator -> /home_navigation/arrived
/nav2_arrival_manipulation_trigger -> /front_piper/task/start
iliyas-abot manipulation lifecycle listener -> ABotClaw overlay
```

There is no direct Docker command or arm command from Nav2. The manipulation
process inside `iliyas-abot` must be running on the same ROS graph and must
subscribe to `/front_piper/task/start`. The listener installed in
`iliyas-abot` also accepts `/manipulation_task/start` for compatibility with the
older handoff note.

Start the Illiyas communication listener from the host before sending the door
goal:

```bash
/home/dase-orin/ros2_ws/src/bunker_slam_bringup/scripts/start_iliyas_manipulation_listener.sh
```

This starts only the communication listener inside `iliyas-abot`. When
`/front_piper/task/start` is published after the door arrival, the listener runs
the hardcoded ABotClaw lifecycle script and starts or repairs the manipulation
overlay. The no-agent path uses the low-level API on port `8892`; port `8893`
is optional.

The installed lifecycle entrypoints are:

```bash
/workspace/ABot-Claw-piperX/robot_layer/arm_piper_x/agent_server/start_manipulation_task_listener.sh
/workspace/ABot-Claw-piperX/robot_layer/arm_piper_x/agent_server/abotclaw_manipulation_lifecycle.sh
```

Check the trigger and manipulation readiness:

```bash
ros2 topic echo /door_navigation/arrived --field data | tr '[:upper:]' '[:lower:]'
ros2 topic echo /home_navigation/arrived --field data | tr '[:upper:]' '[:lower:]'
ros2 topic hz /door_navigation/arrived
ros2 topic hz /home_navigation/arrived
ros2 topic info -v /door_navigation/arrived
ros2 topic info -v /home_navigation/arrived
ros2 topic info /front_piper/task/start

docker exec iliyas-abot bash -lc '
source /opt/ros/humble/setup.bash
source /workspace/agx_arm_ws/install/setup.bash
export ROS_DOMAIN_ID=173
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=1
ros2 node list | grep -E "abotclaw_manipulation_lifecycle_listener|search_marker_node|piper_touch_marker_api"
ros2 service list | grep -E "/search_marker|/run_marker_task|/front_piper/enable_agx_arm"
curl -fsS http://127.0.0.1:8892/health; echo
'
```

Expected `/door_navigation/arrived` content:

```text
false
```

Then, after the `door` Nav2 goal succeeds, `/door_navigation/arrived` publishes
10 `true` messages at 2 Hz, then returns to `false`:

```text
true
true
...
false
```

Expected `/home_navigation/arrived` after the `home` Nav2 goal succeeds: 10
`true` messages at 2 Hz, then back to `false`.

```text
true
true
...
false
```

`--field data` keeps the output as a plain bool series instead of printing the
full ROS message field. The `tr` pipe forces lowercase `false`/`true` output.

The bridge only sends the start message. The manipulation node must still check
arm feedback, CAN state, safety stop, and workspace clearance before moving.
See `MANIPULATION_TASK_COMMUNICATION.md` for the full node-to-node contract,
payload format, manual test commands, and `iliyas-abot` debugging steps.

### Step 7.5: Manual Home And Door Landmark Layer

The Nav2 launch starts `/landmark_navigator` by default. It gives you a manual
landmark layer above Nav2, so you can save precise named map poses such as
`home` and `door`, see them as dots in RViz, and send Nav2 between those named
places later.

The landmark file is:

```bash
/ros2_ws/maps/manual_nav_landmarks.json
```

The default file contains placeholder `home` and `door` poses. Do not use those
placeholder coordinates for real driving. Save the real positions after mapping
or localization is working and `map -> base_link` is valid.

Drive or manually place the robot at the exact home position, then save:

```bash
ros2 topic pub --once /landmark_navigator/save_current_pose std_msgs/msg/String "{data: home}"
```

Move to the exact door position, then save:

```bash
ros2 topic pub --once /landmark_navigator/save_current_pose std_msgs/msg/String "{data: door}"
```

The saved dots and the connection between them publish on:

```bash
ros2 topic echo /landmark_navigator/markers --once
```

Send Nav2 to a saved landmark by name:

```bash
ros2 topic pub --once /landmark_navigator/go_marker std_msgs/msg/String "{data: home}"
ros2 topic pub --once /landmark_navigator/go_marker std_msgs/msg/String "{data: door}"
```

You can also call the home service:

```bash
ros2 service call /landmark_navigator/go_home std_srvs/srv/Trigger {}
```

When the robot is at the saved `door` landmark and you send it back to `home`,
`/landmark_navigator` now sends the home goal as a reverse-home trip. It keeps
the robot nose pointed back toward the door and lets Nav2 command negative
`linear.x`, so the Bunker backs from `door` toward `home` instead of turning
around first.

During that reverse-home trip, the active depth safety camera switches from the
front D435i serial `243322074578` to the rear D435i serial `261222077434`
through the rear camera topic:

```text
/rear_camera/aligned_depth_to_color/image_raw
```

Check the reverse-home mode and active camera:

```bash
ros2 topic echo /landmark_navigator/navigation_direction
ros2 topic echo /landmark_navigator/active_camera
ros2 topic echo /depth_route_status
```

Expected while going from `door` back to `home`:

```text
/landmark_navigator/navigation_direction: reverse
/landmark_navigator/active_camera: rear_camera
/depth_route_status depth_topic: /rear_camera/aligned_depth_to_color/image_raw
```

For normal forward landmark goals, the active camera returns to:

```text
/front_camera/aligned_depth_to_color/image_raw
```

Add more named places by saving another name:

```bash
ros2 topic pub --once /landmark_navigator/save_current_pose std_msgs/msg/String "{data: table}"
ros2 topic pub --once /landmark_navigator/go_marker std_msgs/msg/String "{data: table}"
```

To draw more RViz connection lines, edit the `connections` list in
`/ros2_ws/maps/manual_nav_landmarks.json`, then restart `/landmark_navigator` or
the Nav2 launch.

Example:

```json
"connections": [
  ["home", "door"],
  ["door", "table"]
]
```

### Step 8: Open RViz Or rqt For Checking

RViz:

```bash
rviz2 -d /ros2_ws/install/bunker_slam_bringup/share/bunker_slam_bringup/rviz/bunker_nav2.rviz
```

The Nav2 RViz config now includes:

```text
/landmark_navigator/markers       home/door dots, labels, and connection lines
/front_camera/color/image_raw     front RGB camera view
/rear_camera/color/image_raw      rear RGB camera view
```

For mapping and localization, RTAB-Map consumes front and rear RGB-D sync streams. RViz shows both RGB panels:

```text
/front_camera/color/image_raw
/rear_camera/color/image_raw
/front_rgbd_image
/rear_rgbd_image
```

If Terminal 1 fails with `launch configuration 'front_camera_serial' does not
exist`, the camera include was started from a launch scope that could not see
the declared camera serial arguments. The maintained launch now starts cameras
from the main launch context after an 8 second RealSense USB reset delay, and
the dual camera launch forwards its declared serial arguments into each camera
group. It also passes the serials as strings because the RealSense driver
rejects integer-typed `serial_no` values. Rebuild and source the workspace:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select bunker_slam_bringup bunker_dual_piper_nav2 --symlink-install
source /ros2_ws/install/setup.bash
ros2 launch bunker_slam_bringup terminal1_sensors.launch.py --show-args | grep camera_serial
```

rqt tools:

```bash
rqt_graph
rqt_image_view
rqt
```

rqt is for checking the graph and topics. If CAN, `/odom`, TF, or `/map` are
missing, fix those lower-level checks first.

## URDF, PiPER Joint-State, And TF Connection

For the dual-PiPER workflow, the URDF and arm feedback are connected through
topics, not through a ROS service.

Current data path:

```text
/front_piper/feedback/joint_states  \
                                      -> /dual_piper_joint_state_prefixer -> /joint_states
/rear_piper/feedback/joint_states   /

/robot_state_publisher:
  robot_description parameter/topic -> combined Bunker + front/rear PiPER URDF
  /joint_states subscription        -> front_piper_* and rear_piper_* joint values
  /tf and /tf_static publishers     -> live robot link transforms
```

Important details:

- `/robot_description` is the published URDF description. It is not a command
  service and it does not move the arms.
- `/front_piper/feedback/joint_states` and
  `/rear_piper/feedback/joint_states` are raw AGX/PiPER feedback topics.
- `/front_piper/feedback/tcp_pose` and `/rear_piper/feedback/tcp_pose` stay
  visible because the manipulation agent uses the front TCP pose for health and
  task checks.
- `/front_piper/control/joint_states` and `/rear_piper/control/joint_states`
  are command input topics used by the initial-pose helper. They are expected to
  have subscribers, not publishers.
- Unused PiPER command/status topics are remapped under each arm's hidden
  `/_unused/...` namespace so normal `ros2 topic list` stays readable. Use
  `ros2 topic list --include-hidden-topics` only if you need to inspect them.
- `/dual_piper_joint_state_prefixer` renames those raw joints into the URDF
  names, for example `front_piper_joint1` and `rear_piper_joint1`.
- `/joint_states` is the combined topic consumed by `robot_state_publisher`.
  It contains both front and rear Piper joints in one message.
- Services are used by higher-level behavior, for example starting exploration.
  They are not used to connect the URDF publisher to joint-state feedback.

Live verification:

```bash
ros2 topic echo --once /front_piper/feedback/joint_states
ros2 topic echo --once /rear_piper/feedback/joint_states
ros2 topic echo --once /joint_states
ros2 topic echo --once /robot_description
ros2 run tf2_ros tf2_echo base_link front_piper_base_link
ros2 run tf2_ros tf2_echo base_link rear_piper_base_link
```

If one physical arm moves, the matching prefixed joints in `/joint_states`
should change, and RViz/TF should update for that same arm.

Current two-PiPER bringup for URDF/RViz inspection:

```bash
ros2 launch bunker_dual_piper_nav2 dual_piper_bringup.launch.py \
  rear_piper_can:=can3 \
  front_piper_can:=can2 \
  run_initial_pose:=false \
  allow_piper_motion:=true \
  piper_control_enabled:=true \
  start_rviz:=true
```

This starts the two namespaced PiPER feedback drivers and the combined URDF
publisher, but does not command physical arm motion. The prefixer still seeds
the front and rear parked initial pose on `/joint_states` so the full model is
visible in RViz while waiting for live feedback.

For a headless two-PiPER node bringup without RViz and without arm motion:

```bash
ros2 launch bunker_dual_piper_nav2 dual_piper_bringup.launch.py \
  rear_piper_can:=can3 \
  front_piper_can:=can2 \
  configure_piper_can:=true \
  piper_control_enabled:=true \
  run_initial_pose:=false \
  allow_piper_motion:=true \
  start_rviz:=false
```

Expected PiPER nodes and topics:

```bash
ros2 node list | grep piper
ros2 topic echo --once /front_piper/feedback/joint_states
ros2 topic echo --once /rear_piper/feedback/joint_states
ros2 topic echo --once /joint_states
```

The low-level hardware driver defaults also use `rear_piper_can:=can3` and
`front_piper_can:=can2`, so this mapping is the default unless you override it.

Only after confirming the CAN/bus setup and keeping the area clear, the same
launch can command both arms to the documented initial pose:

```bash
ros2 launch bunker_dual_piper_nav2 dual_piper_bringup.launch.py \
  rear_piper_can:=can3 \
  front_piper_can:=can2 \
  piper_control_enabled:=true \
  run_initial_pose:=true \
  allow_piper_motion:=true \
  start_rviz:=true
```

Current robot mapping is front PiPER on `can2`, rear PiPER on `can3`, and
Bunker on `can4`.

## Stable Startup Layout

Use the stable lower-level launch files directly. Terminal 1 now owns all
always-on hardware: front camera, CAN setup, Bunker driver, robot TF, H30 IMU
and EKF. The remaining terminals depend on the mode you want to run.

YOLO is not part of this startup.

### 1. Local VS Code Terminal Editor And Docker Setup

Run this locally on the robot desktop with VS Code integrated terminal. The display is fixed to
`:1`.

```bash
export DISPLAY=:1
xhost +SI:localuser:root
code
```

In each VS Code terminal-editor pane, enter the ROS Docker container:

```bash
docker exec -it \
  -e DISPLAY=:1 \
  -e QT_X11_NO_MITSHM=1 \
  -e QT_QPA_PLATFORM=xcb \
  trystan-bunker-navigation bash
```

Open the first terminal with `Terminal: Create New Terminal`. The VS Code user
setting defaults new terminals to the editor area. Use the shortcuts below while
the terminal is focused.

Then paste the environment block from
[Environment block for every VS Code terminal-editor pane](#environment-block-for-every-vs-code-terminal-editor-pane)
before running that pane's ROS command.

Configured VS Code integrated-terminal shortcuts:

```text
Ctrl+Alt+Right  split the focused integrated terminal
Ctrl+Alt+Down   split terminal in the active workspace
Ctrl+Alt+Left   move to the previous split terminal pane
Ctrl+Alt+L      move to the next split terminal pane
Ctrl+Alt+Up     move to the previous integrated terminal
Ctrl+Alt+J      move to the next integrated terminal
Ctrl+R          bash reverse search inside the focused terminal
```

Validated configuration files:

```text
/home/dase-orin/.config/Code/User/keybindings.json
/home/dase-orin/.vscode-server/data/User/keybindings.json
/home/dase-orin/.config/Code/User/settings.json
/home/dase-orin/.vscode-server/data/User/settings.json
```

Validation performed from shell:

```bash
python3 -m json.tool /home/dase-orin/.config/Code/User/keybindings.json >/dev/null
python3 -m json.tool /home/dase-orin/.vscode-server/data/User/keybindings.json >/dev/null
```

This validates that the keybinding files are installed and parse correctly.
Final validation is a manual VS Code keypress test: focus an integrated
terminal, press `Ctrl+Alt+Right`, and confirm the terminal splits.

### 2. Terminal 1: All Hardware Startup

Run this once for every mode:

```bash
ros2 launch bunker_slam_bringup terminal1_sensors.launch.py \
  arm_can:=can2 \
  bunker_can:=can4 \
  rear_piper_can:=can3 \
  front_piper_can:=can2 \
  launch_front_camera:=true \
  launch_rear_camera:=true \
  front_camera_serial:=243322074578 \
  rear_camera_serial:=261222077434 \
  configure_can:=true \
  start_piper_drivers:=true \
  piper_control_enabled:=true \
  run_piper_initial_pose:=true \
  allow_piper_motion:=true \
  h30_serial_port:=/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0003-if00 \
  h30_baud_rate:=460800 \
  h30_frame_id:=imu_link \
  start_ekf:=false
```

Optional ArUco debug from the same Terminal 1 hardware launch:

```bash
ros2 launch bunker_slam_bringup terminal1_sensors.launch.py \
  arm_can:=can2 \
  bunker_can:=can4 \
  rear_piper_can:=can3 \
  front_piper_can:=can2 \
  launch_front_camera:=true \
  launch_rear_camera:=true \
  front_camera_serial:=243322074578 \
  rear_camera_serial:=261222077434 \
  configure_can:=true \
  start_piper_drivers:=true \
  piper_control_enabled:=true \
  run_piper_initial_pose:=true \
  allow_piper_motion:=true \
  h30_serial_port:=/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0003-if00 \
  h30_baud_rate:=460800 \
  h30_frame_id:=imu_link \
  start_ekf:=false \
  start_aruco_detector:=true \
  aruco_target_marker_ids:=6 \
  aruco_marker_size_m:=0.05
```

Expected core outputs:

```text
/front_camera/color/image_raw
/front_camera/aligned_depth_to_color/image_raw
/front_camera/depth/color/points
/rear_camera/color/image_raw
/rear_camera/aligned_depth_to_color/image_raw
/rear_camera/depth/color/points
/wheel/odom
/odom
/tf
/tf_static
```

### 3. Mode: Manual Mapping

Use this to create or extend an RTAB-Map database. The robot should be driven
manually with the physical controller; this mode does not start Nav2 movement.

```bash
ros2 launch bunker_dual_piper_nav2 rtabmap_front_camera.launch.py \
  mode:=mapping \
  database_path:=/ros2_ws/maps/bunker_dual_rgbd_v2.db \
  reset_database:=true \
  mapping_camera:=both

rviz2 -d /ros2_ws/install/bunker_slam_bringup/share/bunker_slam_bringup/rviz/camera_rgbd_mapping.rviz
```

Use `reset_database:=false` to continue an existing map. Use
`mapping_camera:=rear` when you want the rear camera to extend the database
without waiting for the combined front+rear RGB-D stream.

### 4. Mode: Already Mapped, Manual Nav2 Goal

Use this after a map database already exists. Start RTAB-Map localization, then
Nav2, then RViz.

```bash
# VS Code terminal-editor pane A: RTAB-Map localization from saved database
ros2 launch bunker_dual_piper_nav2 rtabmap_front_camera.launch.py \
  mode:=localization \
  database_path:=/ros2_ws/maps/bunker_dual_rgbd_v2.db \
  reset_database:=false \
  mapping_camera:=both
```

```bash
# VS Code terminal-editor pane B: Nav2 drive stack
ros2 launch bunker_slam_bringup nav2_bringup.launch.py \
  mode:=drive \
  allow_motion:=true \
  use_rviz:=false \
  start_depth_safety:=true \
  start_clicked_goal:=true
```

```bash
# VS Code terminal-editor pane C: RViz
rviz2 -d /ros2_ws/install/bunker_slam_bringup/share/bunker_slam_bringup/rviz/bunker_nav2.rviz
```

Use RViz `Publish Point` for dot-click goals. The clicked-goal bridge converts
the point to `/navigate_to_pose`.

### 5. Mode: Time-Limit Self-Exploration

Use this only after a short manual Nav2 goal works. Keep Terminal 1 and the
Nav2 stack from the previous mode running, then start:

```bash
ros2 launch bunker_slam_bringup frontier_exploration_drive.launch.py \
  completion_mode:=time_limit \
  time_limit_sec:=120.0 \
  frontier_cluster_min_cells:=3 \
  no_frontier_return_cycles:=2 \
  return_to_start:=true
```

The explorer does not move until you explicitly start it:

```bash
ros2 service call /frontier_explorer/start std_srvs/srv/Trigger "{}"
```

### 6. Mode: ArUco Landmark Exploration

Use this to explore until a configured marker is detected, or to record markers
while exploring for a time limit.

```bash
ros2 launch bunker_slam_bringup aruco_landmark_exploration.launch.py \
  completion_mode:=time_limit \
  time_limit_sec:=120.0 \
  target_marker_ids:=6 \
  marker_size_m:=0.05 \
  frontier_min_distance_m:=0.30 \
  frontier_max_distance_m:=2.5 \
  frontier_cluster_min_cells:=1 \
  no_frontier_return_cycles:=5 \
  dry_run:=false \
  return_to_start:=true
```

Then start:

```bash
ros2 service call /frontier_explorer/start std_srvs/srv/Trigger "{}"
```

### 7. Mode: Already Mapped And Got Landmark

After `/ros2_ws/maps/aruco_landmarks.json` exists, keep localization and Nav2
running, then start the landmark navigator if it is not already running:

```bash
ros2 launch bunker_slam_bringup landmark_navigator.launch.py \
  landmark_path:=/ros2_ws/maps/aruco_landmarks.json
```

Go home:

```bash
ros2 service call /landmark_navigator/go_home std_srvs/srv/Trigger "{}"
```

Go to the furthest saved marker:

```bash
ros2 service call /landmark_navigator/go_furthest std_srvs/srv/Trigger "{}"
```

Go to marker ID `6`:

```bash
ros2 topic pub --once /landmark_navigator/go_marker std_msgs/msg/String "{data: 'aruco_6'}"
```

## Operating modes

RTAB-Map and Nav2 use different `mode` arguments:

| Process | Mode | Purpose |
|---|---|---|
| RTAB-Map | `mapping` | Creates or extends the `.db` map |
| RTAB-Map | `localization` | Loads the `.db` and estimates `map -> odom` |
| Nav2 | `dry_run` | Plans goals but outputs only `/cmd_vel_debug` |
| Nav2 | `drive` | Sends safety-gated commands to the physical chassis |

Normal autonomous navigation is:

```text
RTAB-Map mode:=localization + Nav2 mode:=drive
```

Mapping is only for building the database. Localization is the mode used before sending Nav2 goals.

## Workflow Matrix

Use this table to decide which workflow pieces must be running. Do not run two
rows that own the same function at the same time, especially two RTAB-Map
instances or two ArUco detector nodes.

| Goal | Required workflow pieces | Notes |
|---|---|---|
| Camera/TF/odom health check | Hardware startup only | Confirms `/front_camera/*`, `/wheel/odom`, `/odom`, `/tf`, `/tf_static`. No Nav2. |
| Manual mapping | Hardware startup + RTAB-Map mapping | Drive the robot manually with the physical controller. |
| Existing-map localization | Hardware startup + RTAB-Map localization | Publishes `/map` and `map -> odom`. Required before Nav2 goals. |
| Manual Nav2 dry-run | Hardware startup + RTAB-Map localization + Nav2 in `mode:=dry_run` + optional RViz/status | Plans and publishes `/cmd_vel_debug`; chassis should not move. |
| Manual Nav2 drive | Hardware startup + RTAB-Map localization + Nav2 in `mode:=drive allow_motion:=true` + RViz/status | Publishes direct `/cmd_vel` by default. Use only after dry-run works. |
| Upstream MRTSP self-exploration while mapping | Hardware startup + RTAB-Map mapping + Nav2 drive + `frontier_exploration_ros2_bunker.launch.py` + optional RViz/status | Preferred current self-exploration path. Uses an 87 degree front D435i cone and will not choose rear/blindspot frontier targets. Start and stop with `frontier_exploration_ctl`; the node is cold-idle on launch. |
| Legacy time-limit self-exploration | Hardware startup + RTAB-Map mapping + Nav2 drive + `frontier_exploration_drive.launch.py` + optional RViz/status | Older local Python explorer. Start movement with `/frontier_explorer/start`. |
| ArUco stop-on-found exploration | Hardware startup without `start_aruco_detector`, RTAB-Map mapping/localization, Nav2 drive, `aruco_landmark_exploration.launch.py` | The ArUco exploration launch starts its own ArUco detector and landmark navigator by default. |
| Go to saved landmark/home | Hardware startup + localization + Nav2 drive + landmark navigator | Requires `/ros2_ws/maps/aruco_landmarks.json` from a previous marker detection run. |
| AMCL localization test | Hardware startup + `/scan` provider + AMCL + Nav2 | AMCL is practical only after a valid `/scan` topic and 2D map YAML exist. Do not run AMCL and RTAB-Map localization together for normal use. |

## VS Code Integrated Terminal Display Setup

Use this when you are physically using the Linux desktop attached to the robot.
Run this on the host before opening VS Code:

```bash
export DISPLAY=:1
xhost +SI:localuser:root
code
```

In each VS Code terminal-editor pane, enter Docker:

```bash
docker exec -it \
  -e DISPLAY=:1 \
  -e QT_X11_NO_MITSHM=1 \
  -e QT_QPA_PLATFORM=xcb \
  trystan-bunker-navigation bash
```

Use the VS Code terminal-editor panes and run one Docker shell per pane.

Configured VS Code terminal shortcuts:

```text
Ctrl+Alt+Right  split the focused integrated terminal
Ctrl+Alt+Down   split terminal in the active workspace
Ctrl+Alt+Left   move to the previous split terminal pane
Ctrl+Alt+L      move to the next split terminal pane
Ctrl+Alt+Up     move to the previous integrated terminal
Ctrl+Alt+J      move to the next integrated terminal
Ctrl+R          bash reverse search inside the focused terminal
```

These shortcuts are configured in both local VS Code user keybindings and the
Remote-SSH VS Code server keybindings. The JSON files have been validated from
the shell. The actual working test is to focus a VS Code integrated terminal
and press `Ctrl+Alt+Right`; it should split the current terminal.

In every VS Code terminal-editor pane inside Docker, paste the environment block below before running ROS
commands.

After all GUI programs are closed, remove the X11 permission if desired:

```bash
xhost -SI:localuser:root
```

## Environment block for every VS Code terminal-editor pane

Run this at the start of each pane inside the container:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=173
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

export DISPLAY=:1
export XAUTHORITY=/tmp/.docker.xauth
export QT_QPA_PLATFORM=xcb
export QT_X11_NO_MITSHM=1
export XDG_RUNTIME_DIR=/tmp/xdg-runtime
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"
```

This startup uses the robot desktop X11 display `:1` for VS Code, RViz, rqt,
and other GUI tools.

Domain notes:

- Every terminal on this robot must use the same `ROS_DOMAIN_ID`.
- Do not mix old domain `0` or `42` processes with this startup.
- `ROS_LOCALHOST_ONLY=1` keeps the graph local to this machine/container path.
  Use the same value in every pane. If one pane uses `0` and another uses `1`,
  `ros2 topic list` and rqt can look empty even when nodes are running.

To probe whether a candidate domain is quiet:

```bash
ROS_DOMAIN_ID=173 ros2 node list --no-daemon --spin-time 2
```

## Before starting: avoid duplicate nodes

Inspect existing processes:

```bash
pgrep -af 'realsense|rtabmap|nav2|rviz2|bunker_base|yesense'
```

Stop an existing launch with `Ctrl-C` in its original pane.

Preferred cleanup before a clean restart:

```bash
ros2 run bunker_slam_bringup pkill_bunker_nav2_stack.sh
```

This is a plain `pkill -f` script. It kills known Bunker/Nav2/RTAB-Map/camera/
RViz/showcase/PiPER helper processes, then restarts the ROS 2 daemon so stale
topic names disappear from `ros2 topic list`.

```bash
ros2 topic list
```

After a clean kill, seeing only these is normal:

```text
/parameter_events
/rosout
```

If costmap topics still appear after the script, a process is still alive or the
terminal is using a different `ROS_DOMAIN_ID` from the one you cleaned.

Plain `pkill` cleanup, no ROS script:

```bash
LAUNCHES='terminal1_sensors.launch.py|terminal2_robot.launch.py|terminal3_mapping.launch.py|system_bringup.launch.py|hardware_drivers.launch.py|description.launch.py|dual_realsense.launch.py|rtabmap_front_camera.launch.py|rtabmap.launch.py|nav2_bringup.launch.py|nav2_rtabmap.launch.py|amcl_localization.launch.py|amcl_bringup.launch.py|ekf_h30.launch.py|h30_imu.launch.py|bunker_piper_3d_showcase.launch.py|frontier_exploration.*launch.py|aruco_landmark_exploration.launch.py|landmark_navigator.launch.py|nav2_status_monitor.launch.py'
NODES='realsense2_camera_node|rgbd_sync|rgbdx_sync|rgbd_odometry|point_cloud_xyzrgb|rtabmap|map_server|amcl|controller_server|planner_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|smoother_server|lifecycle_manager|cmd_vel_mux_node|nav2_cmd_vel_safety_mux|clicked_point_nav_goal|nav2_arrival_manipulation_trigger|nav2_manipulation_handoff|nav2_status_monitor|abotclaw_handoff_service|abotclaw_manipulation_lifecycle_listener|front_piper_description_bridge|agent_server/server.py|front_piper_move_group_only|touch_marker_full_stack|search_marker_node|piper_touch_marker_api|depth_route_monitor_node|sensor_fusion_node|safety_monitor_node|frontier_explorer_node|aruco_landmark_node|landmark_navigator_node|bunker_base_node|robot_state_publisher|joint_state_prefixer|joint_state_publisher|static_transform_publisher|ekf_node|imu_bias_corrector.py|yesense_node_publisher|piper_x_joint_preset|piper_navigation_pose|agx_arm_ctrl_single|agx_arm_ctrl_single_node|detector_tracker_node|target_mux_node|depth_geometry_memory_node|semantic_memory_node|rviz2'
pkill -TERM -f "$LAUNCHES" || true
pkill -TERM -f "$NODES" || true
sleep 2
pkill -KILL -f "$LAUNCHES" || true
pkill -KILL -f "$NODES" || true
ros2 daemon stop || true
ros2 daemon start || true
ros2 node list
ros2 topic list
```

If processes are still alive after the normal cleanup, use the same block with
`pkill -KILL -f` for the specific process that remains. Do not use broad
`pkill -9 python3` because it can kill unrelated tools in the container.

Important: do not leave the visual-only URDF showcase running with the real
Nav2 stack. If you only need to stop the showcase:

```bash
pkill -f bunker_piper_3d_showcase.launch.py || true
pkill -f showcase_static_map_to_base_link || true
```

## CAN identity and startup

Do not choose CAN interfaces only by the `canN` number. Check the USB adapter serials.

The current robot mapping uses separate SocketCAN interfaces for the front
PiPER, rear PiPER, and Bunker base.

Canonical device roles by serial:

| Device | USB serial | Interface | Bitrate |
|---|---:|---:|---:|
| Front PiPER | `004E002B4148570A20343133` | `can2` | 1,000,000 bit/s |
| Rear PiPER | `0036001F4148570A20343133` | `can3` | 1,000,000 bit/s |
| Bunker | `001D00255443570A20393433` | `can4` | 500,000 bit/s |

Check the current mapping:

```bash
for n in /sys/class/net/can*; do
  [ -e "$n" ] || continue
  n="$(basename "$n")"
  echo "=== $n ==="
  readlink -f /sys/class/net/$n/device
  cat /sys/class/net/$n/device/../serial 2>/dev/null || true
  ip -details -statistics link show "$n" | sed -n '1,14p'
done
```

Configure CAN manually using the actual interface names from the serial check.
For the current observed mapping:

```bash
ros2 run bunker_slam_bringup configure_can.sh can2 can4 can3
```

Argument order:

```text
configure_can.sh FRONT_PIPER_CAN_INTERFACE BUNKER_CAN_INTERFACE REAR_PIPER_CAN_INTERFACE
```

So `configure_can.sh can2 can4 can3` means front PiPER on `can2`, Bunker on
`can4`, and rear PiPER on `can3`.
Do not use reversed order; that can put
the PiPER adapter at the Bunker bitrate and the Bunker adapter at the PiPER
bitrate.

The CAN setup script also checks the live CAN controller state. If a link is
already UP at the right bitrate but is `ERROR-PASSIVE`, `BUS-OFF`, or otherwise
not `ERROR-ACTIVE`, the script cycles the link instead of leaving stale state in
place.

Check passive traffic:

```bash
timeout 5 candump -L can2
timeout 5 candump -L can3
timeout 5 candump -L can4
```

`candump` is passive. It can run while other drivers are reading the same CAN interface.

If the Bunker driver reports `Detected protocol: UNKNOWN` or `/odom` is missing, verify that the Bunker side is really the 500 kbit/s bus. If the USB mapping changed, fix the command by serial identity, not by guessing.
If `can4` remains `ERROR-PASSIVE` after running the setup command, do not start
Nav2 yet. First make `candump -L can4` show Bunker status frames.

If launch aborts with:

```text
configure_can.sh can2 can4 can4
error: PiPER and Bunker must use distinct CAN interfaces
```

the rear PiPER argument is still wrong. That command means both Bunker and rear
PiPER were set to `can4`. Use `front_piper_can:=can2`, or omit the argument so
the current launch default supplies `can2`:

```bash
ros2 run bunker_slam_bringup configure_can.sh can2 can4 can3
```

The active startup environment only requires raw image transport for the
maintained `/front_camera/...` topics. Check it with:

```bash
ros2 run image_transport list_transports
```

If RealSense reports:

```text
Failed to load plugin image_transport/..._pub
rcl node's context is invalid
No plugins found!
```

look earlier in the same launch log first. In the 2026-08-13 failure, the
camera node context was already being torn down, so these plugin lines were
secondary shutdown noise after Terminal 1 had already aborted. The real software
bug was a duplicate PiPER launch action:

```text
ExecuteLocal action 'agx_arm_ctrl_single-5': executed more than once
```

That is fixed in `bunker_dual_piper_nav2/launch/hardware_drivers.launch.py` by
creating a separate included PiPER launch object for the front and rear arms.

If RealSense reports:

```text
xioctl(VIDIOC_S_FMT) failed, errno=16 Last Error: Device or resource busy
```

the D435i USB/V4L device is stuck or still recovering from a previous failed
camera launch. Terminal 1 now resets the connected D435i USB device before
starting RealSense by default:

```text
reset_front_camera_usb:=true
```

The reset uses `ros2 run bunker_autonomy realsense_usb_recover`, which supports
the active D435i product ID `8086:0b3a`.

If the watchdog reports:

```text
No RealSense USB device is currently enumerated
```

do not reboot the Jetson and do not restart Docker as the first fix. That means
Linux currently has no D435i USB device for the watchdog to reset. Keep the
container and the rest of the robot stack up, recover only the camera USB path
first, then let the RealSense node respawn or restart only the camera/Terminal 1
launch if needed:

```bash
rs-enumerate-devices -s
ls -l /dev/video* /dev/media*
ros2 run bunker_autonomy realsense_usb_recover
```

If `rs-enumerate-devices -s` still shows no D435i, the remaining action is
physical camera recovery: check cable, hub power, and replug the D435i. A full
Jetson reboot is only a last resort after the camera cannot re-enumerate.

## Optional: move both PiPER arms to crossed navigation pose first

This can physically move both arms at the same time. Keep the robot clear.

```bash
ros2 run bunker_slam_bringup configure_can.sh can2 can4 can3

ros2 launch bunker_dual_piper_nav2 dual_piper_bringup.launch.py \
  rear_piper_can:=can3 \
  front_piper_can:=can2 \
  configure_piper_can:=false \
  piper_control_enabled:=true \
  run_initial_pose:=true \
  allow_piper_motion:=true \
  piper_speed_percent:=25 \
  front_piper_parked_joint1:=-1.6 \
  rear_piper_parked_joint1:=1.6
```

When both arms reach the crossed navigation pose, press `Ctrl-C`. Then start Terminal 1.

## Terminal 1 Combined Hardware Launch

What it does:

- Starts the front and rear D435i cameras when `launch_front_camera:=true` and
  `launch_rear_camera:=true`.
- Publishes RGB, aligned depth, camera info, and point cloud topics for each
  enabled camera.
- Can optionally start the OpenCV ArUco detector for 5 cm marker testing.
- Configures front PiPER `can2` and rear PiPER `can3` at 1 Mbit/s, and Bunker `can4` at 500 kbit/s.
- Starts both PiPER drivers by default so `/front_piper/feedback/joint_states`
  and `/rear_piper/feedback/joint_states` feed the prefixed `/joint_states`
  stream used by the URDF in RViz.
- Starts the Bunker Mini driver on `can4`.
- Publishes raw Bunker odometry directly on `/odom` when `start_ekf:=false`.
- In this documented mode, EKF is disabled; H30 IMU remains available as a diagnostic stream.
- Publishes robot URDF and static camera/arm TF.
- Publishes H30/YESENSE IMU stream.
- Does not start YOLO.

Start all hardware:

```bash
ros2 launch bunker_slam_bringup terminal1_sensors.launch.py \
  arm_can:=can2 \
  bunker_can:=can4 \
  rear_piper_can:=can3 \
  front_piper_can:=can2 \
  launch_front_camera:=true \
  launch_rear_camera:=true \
  front_camera_serial:=243322074578 \
  rear_camera_serial:=261222077434 \
  configure_can:=true \
  start_piper_drivers:=true \
  piper_control_enabled:=true \
  run_piper_initial_pose:=true \
  allow_piper_motion:=true \
  h30_serial_port:=/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0003-if00 \
  h30_baud_rate:=460800 \
  h30_frame_id:=imu_link \
  start_ekf:=false
```

Equivalent single-line command:

```bash
ros2 launch bunker_slam_bringup terminal1_sensors.launch.py arm_can:=can2 bunker_can:=can4 rear_piper_can:=can3 front_piper_can:=can2 launch_front_camera:=true launch_rear_camera:=true front_camera_serial:=243322074578 rear_camera_serial:=261222077434 configure_can:=true start_piper_drivers:=true piper_control_enabled:=true run_piper_initial_pose:=true allow_piper_motion:=true start_ekf:=false
```

To command both PiPER arms to the documented initial/navigation pose, first
confirm the CAN roles and keep the area clear, then opt in explicitly:

```bash
ros2 launch bunker_slam_bringup terminal1_sensors.launch.py \
  arm_can:=can2 \
  bunker_can:=can4 \
  rear_piper_can:=can3 \
  front_piper_can:=can2 \
  configure_can:=true \
  start_piper_drivers:=true \
  piper_control_enabled:=true \
  run_piper_initial_pose:=true \
  allow_piper_motion:=true \
  h30_serial_port:=/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0003-if00 \
  h30_baud_rate:=460800 \
  h30_frame_id:=imu_link \
  start_ekf:=false
```

If the terminal shows a `>` prompt after the first line, Bash is waiting for the continued command because the previous line ended with `\`. Finish the command on the next line or cancel it with `Ctrl+C`.

If arrow-key escape text such as `^[[A`, `^[[B`, or `^[[C` appears, reset the terminal state:

```bash
Ctrl+C
reset
```

Then run the single-line command above. If using the multi-line form, the
backslash must be the last character on the line with no spaces after it.

Optional OpenCV ArUco test from Terminal 1:

```bash
ros2 launch bunker_slam_bringup terminal1_sensors.launch.py \
  arm_can:=can2 \
  bunker_can:=can4 \
  rear_piper_can:=can3 \
  front_piper_can:=can2 \
  configure_can:=true \
  start_piper_drivers:=true \
  piper_control_enabled:=true \
  run_piper_initial_pose:=true \
  allow_piper_motion:=true \
  start_ekf:=false \
  start_aruco_detector:=true \
  aruco_dictionary:=DICT_4X4_50 \
  aruco_target_marker_ids:=6 \
  aruco_marker_size_m:=0.05 \
  aruco_required_seen_count:=3 \
  aruco_publish_debug_image:=true
```

This starts the full hardware stack plus `aruco_landmark_node`. It does not use
YOLO and does not command motion. For a 5 cm marker, test slowly at short range
first. If the marker is not detected reliably, increase physical marker size
before changing the navigation logic.

Expected important topics:

| Topic | Publisher | Used by | Meaning |
|---|---|---|---|
| `/front_camera/color/image_raw` | RealSense | RTAB-Map RGB-D sync, RViz RGB image display | Front RGB image |
| `/front_camera/color/camera_info` | RealSense | RTAB-Map RGB-D sync | Camera calibration/intrinsics |
| `/front_camera/aligned_depth_to_color/image_raw` | RealSense | RTAB-Map RGB-D sync | Front depth image aligned to RGB |
| `/front_camera/depth/color/points` | RealSense | Depth safety, RViz diagnostic | Registered depth point cloud |
| `/front_camera/imu` | RealSense, only if D435i HID works | Diagnostic only in this workflow | Camera IMU stream |
| `/rear_camera/color/image_raw` | RealSense | RTAB-Map RGB-D sync, RViz RGB image display | Rear RGB image |
| `/rear_camera/color/camera_info` | RealSense | RTAB-Map RGB-D sync | Rear camera calibration/intrinsics |
| `/rear_camera/aligned_depth_to_color/image_raw` | RealSense | RTAB-Map RGB-D sync | Rear depth image aligned to RGB |
| `/rear_camera/depth/color/points` | RealSense | RViz diagnostic / optional rear depth safety | Rear registered depth point cloud |
| `/front_piper/feedback/joint_states` | Front PiPER driver on `can2` | Joint-state prefixer | Raw front arm feedback |
| `/rear_piper/feedback/joint_states` | Rear PiPER driver on `can3` | Joint-state prefixer | Raw rear arm feedback |
| `/joint_states` | Joint-state prefixer | robot_state_publisher, RViz | Prefixed front/rear PiPER joint states for the URDF |
| `/odom` | Bunker driver when `start_ekf:=false` | RTAB-Map, Nav2 | Raw Bunker odometry |
| `/wheel/odom` | Not used in this mode | N/A | Only used when EKF is enabled |
| `/tf` | Bunker driver, RTAB-Map, robot state publisher | RViz, Nav2 | Dynamic transforms |
| `/tf_static` | robot state/static publishers | RViz, Nav2 | Static transforms |
| `/aruco_landmarks/summary` | Optional OpenCV ArUco node | Operator/debug | JSON summary of marker IDs found |
| `/aruco_landmarks/found` | Optional OpenCV ArUco node | ArUco exploration mode | `true` after a marker is confirmed |
| `/aruco_landmarks/debug_image` | Optional OpenCV ArUco node | RViz/rqt_image_view | RGB image with detected marker boxes |

Checks:

```bash
rs-enumerate-devices -s

ros2 topic info /front_camera/color/image_raw
ros2 topic info /front_camera/aligned_depth_to_color/image_raw
ros2 topic info /front_camera/depth/color/points
ros2 topic info /rear_camera/color/image_raw
ros2 topic info /rear_camera/aligned_depth_to_color/image_raw
ros2 topic info /rear_camera/depth/color/points

ros2 topic hz /front_camera/color/image_raw
ros2 topic hz /front_camera/aligned_depth_to_color/image_raw
ros2 topic hz /front_camera/depth/color/points
ros2 topic hz /rear_camera/color/image_raw
ros2 topic hz /rear_camera/aligned_depth_to_color/image_raw
ros2 topic hz /rear_camera/depth/color/points
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
```

Publisher count alone is not enough. The topics must also produce messages.

Verified behavior on 2026-08-10:

- RGB image published around 20-30 Hz.
- Aligned depth published around 20 Hz.
- Raw depth published around 9-11 Hz during the sample.
- Point cloud was initially missing because this RealSense driver exposes the runtime pointcloud parameters as `pointcloud__neon_.*`. The launch now sets those actual parameters after startup, and `/front_camera/depth/color/points` was verified around 28 Hz.

## CAN Role Reminder

- The current observed launch mapping is `arm_can:=can2`, `bunker_can:=can4`,
  `rear_piper_can:=can3`, and `front_piper_can:=can2`. Re-check serials before
  every hardware session.

Expected important topics:

| Topic | Publisher | Used by | Meaning |
|---|---|---|---|
| `/odom` | Bunker base driver with `start_ekf:=false` | RTAB-Map, Nav2, RViz | Public raw Bunker odometry |
| `/wheel/odom` | Not used in this mode | N/A | EKF-only raw wheel odom topic |
| `/tf` | Bunker driver / robot state publisher / RTAB-Map later | RTAB-Map, Nav2, RViz | Dynamic transforms |
| `/tf_static` | Robot state publisher | RTAB-Map, Nav2, RViz | Static robot/camera/arm transforms |
| `/robot_description` | Robot state publisher | RViz RobotModel | URDF XML |
| `/joint_states` | Default joint-state publisher in Terminal 1 hardware startup | Robot state publisher, RViz | Arm/base joint visualization |
| `/yesense/imu_data_ros` | H30 launch | Diagnostics | External vehicle IMU stream, not fused in this mode |
| `/odometry/filtered` | Not used in this Terminal 1 setup | N/A | EKF disabled in this mode |
| `/cmd_vel` | Bunker driver subscribes | Nav2 drive mode output | Final chassis velocity command |

Checks:

```bash
ros2 topic info /odom
ros2 topic echo /odom --once
ros2 topic hz /odom
ros2 topic echo /wheel/odom --once
ros2 topic hz /yesense/imu_data_ros

# Optional passive one-terminal metric TUI for raw odom vs IMU vs fused odom.
# See ODOM_IMU_FUSION_METRICS.md for the trial procedure and scoring.
ros2 run bunker_slam_bringup odom_imu_fusion_metrics_tui.py

ros2 topic info /cmd_vel
ros2 topic echo /joint_states --once

ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link front_camera_color_optical_frame
```

`/odom` must have a publisher before mapping, localization, or Nav2 can work correctly.

EKF ownership rule:

- Run exactly one publisher for `odom -> base_link`.
- Current documented mode: `start_ekf:=false`, so the Bunker driver owns `odom -> base_link`.
- If `start_ekf:=true`, EKF owns `odom -> base_link` and the Bunker driver must not publish that same public TF edge.
- If both publish that TF edge, Nav2 and RViz can jump or reject goals.

## Manual RTAB-Map Mapping

Keep hardware startup running. Use this for manual map creation or map
extension. Drive manually with the physical Bunker controller. Keyboard teleop
and Nav2 are not required for this mode.

Start a new map:

```bash
mkdir -p /ros2_ws/maps

ros2 launch bunker_dual_piper_nav2 rtabmap_front_camera.launch.py \
  mode:=mapping \
  database_path:=/ros2_ws/maps/bunker_dual_rgbd_v2.db \
  reset_database:=true \
  mapping_camera:=both

rviz2 -d /ros2_ws/install/bunker_slam_bringup/share/bunker_slam_bringup/rviz/camera_rgbd_mapping.rviz
```

Continue an existing map:

```bash
ros2 launch bunker_dual_piper_nav2 rtabmap_front_camera.launch.py \
  mode:=mapping \
  database_path:=/ros2_ws/maps/bunker_dual_rgbd_v2.db \
  reset_database:=false \
  mapping_camera:=both

rviz2 -d /ros2_ws/install/bunker_slam_bringup/share/bunker_slam_bringup/rviz/camera_rgbd_mapping.rviz
```

Continue an existing map using only the rear camera:

```bash
ros2 launch bunker_dual_piper_nav2 rtabmap_front_camera.launch.py \
  mode:=mapping \
  database_path:=/ros2_ws/maps/bunker_dual_rgbd_v2.db \
  reset_database:=false \
  mapping_camera:=rear

rviz2 -d /ros2_ws/install/bunker_slam_bringup/share/bunker_slam_bringup/rviz/camera_rgbd_mapping.rviz
```

What it does:

- Starts RTAB-Map in mapping mode.
- Starts mapping RViz.
- With `mapping_camera:=both`, `/front_rgbd_sync` and `/rear_rgbd_sync` publish
  `/front_rgbd_image` and `/rear_rgbd_image`; `/rgbdx_sync` combines them into
  `/rgbd_images`; `/rtabmap` subscribes to `/rgbd_images`, plus `/odom`.
- With `mapping_camera:=front` or `mapping_camera:=rear`, only that camera's
  `rgbd_sync` node starts; `/rtabmap` subscribes directly to that single
  `/front_rgbd_image` or `/rear_rgbd_image`, plus `/odom`.
- `/rtabmap` publishes `/map`.
- Writes the selected `.db` database.
- Uses the robot TF and Bunker `/odom` already provided by Terminal 1.

Expected important topics:

| Topic | Publisher | Used by | Meaning |
|---|---|---|---|
| `/front_rgbd_image` | `front_rgbd_sync` | `rgbdx_sync` | Front RGB + aligned depth packet |
| `/rear_rgbd_image` | `rear_rgbd_sync` | `rgbdx_sync` | Rear RGB + aligned depth packet |
| `/rgbd_images` | `rgbdx_sync` | RTAB-Map | Combined front + rear RGB-D packet |
| `/map` | RTAB-Map | RViz, Nav2 later | 2D occupancy grid |
| `/map_updates` | RTAB-Map | RViz map display | Incremental occupancy updates |
| `/mapData` | RTAB-Map | Diagnostics/database inspection | RTAB-Map graph/map data |
| `/mapGraph` | RTAB-Map | Diagnostics | Pose graph |
| `/mapPath` | RTAB-Map | RViz/diagnostic path display | Estimated map path |
| `/cloud_map` | RTAB-Map | RViz 3D map display | Global RGB-D point cloud |
| `/tf` | RTAB-Map | RViz/Nav2 | Usually includes `map -> odom` |

Checks:

```bash
ros2 topic hz /front_rgbd_image
ros2 topic hz /rear_rgbd_image
ros2 node info /rtabmap | grep -E 'rgbd_image0|rgbd_image1|/odom' || true
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_link
```

Stop lower-level mapping correctly:

1. Stop the robot with the physical controller.
2. Press `Ctrl-C` once in the mapping terminal.
3. Wait for RTAB-Map and RViz to exit and for the shell prompt to return.
4. Do not stop the camera before RTAB-Map finishes closing.
5. Back up the closed database:

```bash
cp -a /ros2_ws/maps/bunker_dual_rgbd_v2.db \
  /ros2_ws/maps/bunker_dual_rgbd_v2_backup.db

ls -lh /ros2_ws/maps/bunker_dual_rgbd_v2*.db
```

Never copy the SQLite database while RTAB-Map is writing it.

## RTAB-Map Localization

Keep hardware startup running. Stop manual mapping before localization. Do not run two
RTAB-Map instances against the same camera/odom stream.

```bash
ros2 launch bunker_dual_piper_nav2 rtabmap_front_camera.launch.py \
  mode:=localization \
  database_path:=/ros2_ws/maps/bunker_dual_rgbd_v2.db \
  reset_database:=false \
  mapping_camera:=both
```

What it does:

- Loads the saved RTAB-Map database.
- Publishes `/map`.
- Estimates the robot pose in the database.
- Publishes `map -> odom`.

Expected important topics:

| Topic | Publisher | Used by | Meaning |
|---|---|---|---|
| `/front_rgbd_image` | `front_rgbd_sync` | `rgbdx_sync` | Front RGB-D input |
| `/rear_rgbd_image` | `rear_rgbd_sync` | `rgbdx_sync` | Rear RGB-D input |
| `/rgbd_images` | `rgbdx_sync` | RTAB-Map localization | Combined front + rear RGB-D input |
| `/map` | RTAB-Map | Nav2/RViz | Existing occupancy map |
| `/tf` | RTAB-Map | Nav2/RViz | `map -> odom` transform |
| `/mapGraph`, `/mapPath` | RTAB-Map | Diagnostics | Localization graph/path |

Checks:

```bash
ros2 topic info /map
ros2 run tf2_ros tf2_echo map base_link
```

If localization does not appear immediately, slowly move or rotate the robot in a previously mapped area.

## AMCL Localization Alternative

AMCL is available as an alternative to RTAB-Map localization when you have a
saved 2D map YAML and a real `/scan` topic.

Do not run AMCL and RTAB-Map localization together unless one of them is
configured not to publish `map -> odom`. They normally publish the same TF edge.

```bash
ros2 launch bunker_slam_bringup amcl_localization.launch.py \
  map:=/ros2_ws/maps/map.yaml \
  scan_topic:=/scan
```

You can also run AMCL from the same Terminal 3 wrapper used for mapping and
RTAB-Map localization:

```bash
ros2 launch bunker_slam_bringup terminal3_mapping.launch.py \
  mode:=localization \
  localization_backend:=amcl \
  map:=/ros2_ws/src/bunker_slam_bringup/maps/bunker_map.yaml \
  scan_topic:=/scan \
  use_rviz:=true
```

This Terminal 3 AMCL mode does not start RTAB-Map. It starts `map_server`,
`amcl`, the localization lifecycle manager, and RViz when `use_rviz:=true`.
`mode:=mapping localization_backend:=amcl` is rejected because AMCL cannot build
an RTAB-Map database.

AMCL requirements:

- `/odom` from the Terminal 1 Bunker driver in the documented `start_ekf:=false` mode.
- `odom -> base_link` from exactly one owner.
- `/map` from `map_server`.
- `/scan` as `sensor_msgs/LaserScan`.

Current limitation: this workspace does not have `pointcloud_to_laserscan`
installed in Docker, so RealSense point cloud cannot automatically become
`/scan` yet. If there is no `/scan`, AMCL can launch but cannot localize.

## Nav2 Controller

Start dry-run first:

```bash
ros2 launch bunker_slam_bringup nav2_bringup.launch.py \
  mode:=dry_run \
  allow_motion:=false \
  use_rviz:=false \
  start_depth_safety:=true \
  start_clicked_goal:=true
```

Dry-run starts planner/controller and safety monitoring, but the final Nav2
velocity output is `/cmd_vel_debug`. The chassis should not move.

Change to physical drive:

1. Keep RTAB-Map localization running.
2. Stop only the dry-run Nav2 launch with `Ctrl-C`.
3. Run:

```bash
ros2 launch bunker_slam_bringup nav2_bringup.launch.py \
  mode:=drive \
  allow_motion:=true \
  use_rviz:=false \
  start_depth_safety:=true \
  start_clicked_goal:=true
```

`start_clicked_goal:=true` starts `/clicked_point_nav_goal`, which subscribes to
RViz `/clicked_point` and sends a Nav2 `/navigate_to_pose` action goal. This is
the dot-click replacement for the old dragged arrow goal workflow.

Current default command routing is direct:

```text
dry-run: controller_server -> /cmd_vel_debug
drive:   controller_server -> /cmd_vel -> bunker_base_node
```

The legacy safety-mux route is optional. Use it only when specifically testing
that older chain:

```bash
use_cmd_vel_mux:=true
```

Legacy route:

```text
controller_server -> /nav2/cmd_vel_raw
velocity_smoother -> /cmd_vel_autonomy
nav2_cmd_vel_safety_mux -> /cmd_vel
```

Expected important topics/actions:

| Topic/action | Publisher/subscriber | Used by | Meaning |
|---|---|---|---|
| `/cmd_vel_debug` | Nav2 controller publishes in dry-run | Operator check | Safe test output; robot should not move |
| `/cmd_vel` | Nav2 controller publishes in drive mode; Bunker subscribes | Bunker chassis | Physical velocity command |
| `/safety_stop` | Depth safety/fusion publishes | Operator/optional mux | Current default does not gate `/cmd_vel`; legacy mux uses this as a hard stop |
| `/nav2/cmd_vel_raw` | Nav2 controller publishes only with `use_cmd_vel_mux:=true` | Velocity smoother | Legacy raw controller command |
| `/cmd_vel_autonomy` | Velocity smoother publishes only with `use_cmd_vel_mux:=true` | Safety mux | Legacy smoothed Nav2 command |
| `/cmd_vel_mux/reason` | Safety mux publishes only with `use_cmd_vel_mux:=true` | Operator check | Legacy mux hold reason such as `watchdog_timeout` |
| `/global_costmap/costmap` | Nav2 global costmap | RViz/Nav2 | Global obstacle/cost layer |
| `/local_costmap/costmap` | Nav2 local costmap | RViz/Nav2 | Local obstacle/cost layer |
| `/plan` | Planner server | RViz/controller | Global path |
| `/local_plan` | Controller server if configured | RViz | Local path |
| `/clicked_point` | RViz Publish Point tool publishes | `/clicked_point_nav_goal` | Dot-click goal input |
| `/navigate_to_pose` action | BT Navigator | `/clicked_point_nav_goal`, RViz/Nav2 tools | Goal interface |

Checks:

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 topic echo --once /safety_stop
ros2 topic info /cmd_vel
ros2 topic echo /cmd_vel_debug --once
```

Required before physical movement:

- `/map` exists.
- `/odom` exists and updates.
- `map -> base_link` prints continuously.
- `/front_camera/depth/color/points` publishes.
- Controller and planner report `active [3]`.
- `/safety_stop` is `false` when the route is clear.
- Bunker driver subscribes to `/cmd_vel`.

## Navigation RViz

```bash
rviz2 -d /ros2_ws/install/bunker_slam_bringup/share/bunker_slam_bringup/rviz/bunker_nav2.rviz
```

RViz controls:

1. Set `Fixed Frame` to `map`.
2. Confirm the robot model appears at the correct real-world position.
3. Confirm `/map`, global costmap, local costmap, and robot footprint are visible.
4. RTAB-Map localizes visually, so `2D Pose Estimate` is normally not needed. Use it only if the robot pose is clearly wrong.
5. Select `Publish Point`.
6. Click once on free white/open map space.
7. `/clicked_point_nav_goal` converts that dot into a Nav2 goal.
8. Use `Cancel` in the Navigation 2 panel to stop an active goal.

Dot-click note:

- The old arrow goal tool was removed from `bunker_nav2.rviz`.
- The clicked goal keeps the robot's current yaw as the final goal yaw.
- If Nav2 rejects a clicked white cell, check the costmap/inflation and whether a continuous free path exists. A white occupancy-grid cell can still be invalid in the Nav2 costmap.

First goal procedure:

1. Test the first goal in `dry_run`.
2. Use a short 0.5-1.0 m goal in an open area.
3. Confirm `/cmd_vel_debug` publishes.
4. Switch Nav2 from dry-run to drive.
5. Send the same kind of short goal.
6. Keep the physical controller/emergency stop ready.

## Optional Time-Limit Self-Exploration

Start this only after a manual clicked-dot Nav2 goal works in `drive` mode.
For exploration while building a map, keep Terminal 1 running, start RTAB-Map in
`mode:=mapping`, start Nav2 in `mode:=drive allow_motion:=true`, then start this
explorer pane. For exploration over an existing map only, use RTAB-Map
`mode:=localization` instead of `mode:=mapping`.

Launching the explorer does not immediately move the robot. The explicit start
service captures home and starts the timer/frontier completion logic.

Complete time-limit mapping startup:

```bash
# Terminal 1: hardware
ros2 launch bunker_slam_bringup terminal1_sensors.launch.py arm_can:=can2 bunker_can:=can4 rear_piper_can:=can3 front_piper_can:=can2 launch_front_camera:=true launch_rear_camera:=true front_camera_serial:=243322074578 rear_camera_serial:=261222077434 configure_can:=true start_piper_drivers:=true piper_control_enabled:=true run_piper_initial_pose:=true allow_piper_motion:=true start_ekf:=false
```

```bash
# RTAB-Map mapping
ros2 launch bunker_dual_piper_nav2 rtabmap_front_camera.launch.py \
  mode:=mapping \
  database_path:=/ros2_ws/maps/bunker_dual_rgbd_v2.db \
  reset_database:=true \
  mapping_camera:=both
```

```bash
# Nav2 physical drive stack
ros2 launch bunker_slam_bringup nav2_bringup.launch.py \
  mode:=drive \
  allow_motion:=true \
  use_rviz:=false \
  start_depth_safety:=true \
  start_clicked_goal:=true
```

```bash
# RViz
rviz2 -d /ros2_ws/install/bunker_slam_bringup/share/bunker_slam_bringup/rviz/bunker_nav2.rviz
```

Recommended first moving test:

```bash
ros2 launch bunker_slam_bringup frontier_exploration_ros2_bunker.launch.py \
  autostart:=false \
  control_service_enabled:=true
```

Then start and schedule a stop:

```bash
ros2 run frontier_exploration_ros2 frontier_exploration_ctl start
ros2 run frontier_exploration_ros2 frontier_exploration_ctl stop -t 120
```

ArUco stop-on-found mode:

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

This launches OpenCV ArUco landmark detection plus frontier exploration. It
stops exploration after any ArUco ID is confirmed `required_seen_count` times,
then returns to the captured home pose.

If you want to record multiple door markers and choose the furthest later, use
time-limit mode instead:

```bash
ros2 launch bunker_slam_bringup aruco_landmark_exploration.launch.py \
  completion_mode:=time_limit \
  time_limit_sec:=120.0 \
  target_marker_ids:=6 \
  marker_size_m:=0.05 \
  required_seen_count:=3 \
  dry_run:=false \
  return_to_start:=true
```

Landmarks are saved to:

```bash
/ros2_ws/maps/aruco_landmarks.json
```

Optional landmark navigator:

The ArUco exploration launch starts this by default with
`start_landmark_navigator:=true`. Use this separate launch only if you did not
start it through the ArUco exploration launch.

```bash
ros2 launch bunker_slam_bringup landmark_navigator.launch.py \
  landmark_path:=/ros2_ws/maps/aruco_landmarks.json
```

Send Nav2 to saved home:

```bash
ros2 service call /landmark_navigator/go_home std_srvs/srv/Trigger "{}"
```

Send Nav2 to Iliyas' marker ID `6`:

```bash
ros2 topic pub --once /landmark_navigator/go_marker std_msgs/msg/String "{data: 'aruco_6'}"
```

Send Nav2 to the furthest valid saved ArUco marker:

```bash
ros2 service call /landmark_navigator/go_furthest std_srvs/srv/Trigger "{}"
```

Start exploration, if not already started above:

```bash
ros2 service call /frontier_explorer/start std_srvs/srv/Trigger "{}"
```

Stop manually:

```bash
ros2 service call /frontier_explorer/stop std_srvs/srv/Trigger "{}"
```

Later frontier-ratio mode:

```bash
ros2 launch bunker_slam_bringup frontier_exploration_drive.launch.py \
  completion_mode:=frontier_ratio \
  frontier_stop_ratio:=0.20 \
  frontier_cluster_min_cells:=3 \
  no_frontier_return_cycles:=2 \
  return_to_start:=true
```

Explorer behavior notes:

- The explorer scans `/map` every 2 seconds by default.
- `frontier_cluster_min_cells:=3` accepts small frontier groups. This is useful
  for early maps where frontiers are fragmented. Increase it later if the robot
  chases noisy tiny frontiers.
- `no_frontier_return_cycles:=2` means it must see no usable frontier twice in
  a row before it returns home.
- Frontier goals are sent to known-free cells beside unknown frontier cells, not
  to the unknown cells themselves. This makes Nav2 more likely to accept and
  plan the goal.
- The terminal logs show `frontier_cells`, `clusters`, `usable`, `free_cells`
  and `ratio` when no usable frontier is found.

Current frontier-exploration limitation:

- The explorer currently chooses the nearest valid frontier goal and sends it to
  Nav2.
- If Nav2 returns status `6`, the frontier is blacklisted and the explorer picks
  another frontier.
- This works only when Nav2 can plan cleanly from the current robot pose. If the
  local costmap is blocked or Nav2 cannot route to the selected frontier,
  `/cmd_vel` will remain empty or zero. If the legacy command mux is enabled
  with `use_cmd_vel_mux:=true`, it may additionally hold `/cmd_vel` at zero with
  `watchdog_timeout`.

Recommended algorithm change before making exploration more aggressive:

1. Keep the current frontier detector.
2. Before sending a frontier goal, score candidate frontiers by Nav2 plan
   success, not only by distance.
3. If the selected frontier fails with Nav2 status `6`, send a recovery goal to
   a known-clear pose first.
4. Only after the robot reaches that clear pose, select the next frontier.
5. If safety stop is active, do not keep sending new frontier goals. Wait for
   clear safety or return to the last known-clear pose.

Suggested recovery behavior:

```text
frontier selected
  -> ask Nav2 to go to frontier-adjacent free cell
  -> if planning succeeds: continue
  -> if planning fails/status=6:
       blacklist frontier
       go to last_known_clear_pose or home
       wait until /safety_stop is false
       pick a new frontier from the updated map
```

This is better than continuously trying new frontier goals from a bad position.
It makes the robot first move to a stable, known-free area, then explore again.

Possible implementation modes:

| Option | Behavior | Use case |
|---|---|---|
| Current nearest-frontier | Pick nearest valid frontier, blacklist failed ones | Simple tests in open space |
| Plan-verified frontier | Try candidates by calling Nav2 planner before sending movement goal | Best next upgrade; avoids many status `6` failures |
| Recovery-to-clear-pose | On repeated failure/watchdog/safety stop, go to home or last known clear pose, then resume | Useful for your current issue |
| Corridor/clearance-biased frontier | Prefer frontiers with more known-free cells around the target | Better for tight indoor maps |
| Viewpoint frontier | Pick a goal facing the unknown area from a safe distance | Better RGB-D mapping coverage |

For full details, see:

```text
/ros2_ws/src/bunker_slam_bringup/SELF_EXPLORATION_STARTUP.md
```

## Optional colored Nav2 status monitor

Use this VS Code terminal-editor pane while manual Nav2 or self-exploration is running:

```bash
ros2 launch bunker_slam_bringup nav2_status_monitor.launch.py
```

It displays the current Nav2 goal state, safety stop, Bunker control/error
state, map/odom/cmd freshness and velocity direction. The final command line
shows whether the robot is stopped, moving forward/reverse, or turning
left/right.

## Readiness review of the current terminal configuration

Static inspection result:

| Startup part | Ready for Nav2? | Reason |
|---|---|---|
| Terminal 1 hardware | Ready if RGB/depth/point cloud, `/odom`, and TF are live | Provides camera, hardware, Bunker odom and robot TF |
| Manual mapping VS Code terminal-editor pane | Complete for map creation | RTAB-Map mapping, `/map`, front/rear RGB RViz panels and point clouds; no Nav2 chassis command |
| Localization + Nav2 VS Code terminal-editor panes | Complete for manual Nav2 over existing DB | RTAB-Map localization, `/map`, `map -> odom`, Nav2 direct `/cmd_vel`, clicked goals and RViz |
| Time-limit exploration VS Code terminal-editor pane | Complete for self-exploration after manual Nav2 works | Frontier explorer sends Nav2 goals after `/frontier_explorer/start` |
| Status monitor | Optional diagnostic | Colored dashboard for Nav2/safety/Bunker/cmd_vel status |

Current important caveat:

- Terminal 1 starts both PiPER drivers by default, with front feedback from
  `can3` and rear feedback from `can3`.
- The joint-state prefixer converts `/front_piper/feedback/joint_states` and
  `/rear_piper/feedback/joint_states` into prefixed `/joint_states` names for
  the URDF in RViz.
- For hardware-independent RViz review, use the lower-level description/system
  launch with hardware drivers disabled and `publish_default_joint_states:=true`.

Dry validation on 2026-08-12 inside Docker container
`trystan-bunker-navigation`:

- `--show-args` passed for hardware startup, mapping,
  RTAB-Map front-camera localization/mapping, Nav2, frontier exploration,
  ArUco landmark exploration, landmark navigator, EKF, AMCL and PiPER-X preset
  launches.
- Short `timeout --signal=INT` runtime checks passed for Nav2 dry-run and
  ArUco/frontier dry-run. The `exit code -2`, `context is invalid` and
  `KeyboardInterrupt` style messages seen in those checks are caused by the
  intentional timeout stopping the nodes.
- Nav2 costmap inflation was corrected from `0.10` m to `0.35` m for both
  local and global costmaps, because the configured Bunker footprint has an
  inscribed radius of about `0.285` m.
## Minimum health checklist before sending a Nav2 goal

Run from a temporary VS Code integrated terminal diagnostic pane:

```bash
ros2 topic list | grep -E '^/odom$|^/tf$|^/tf_static$|^/map$|^/front_rgbd_image$|^/rear_rgbd_image$|^/rgbd_images$|^/cmd_vel_debug$|^/safety_stop$'
ros2 topic echo /odom --once
ros2 topic echo /map --once
ros2 topic hz /front_rgbd_image
ros2 topic hz /rear_rgbd_image
ros2 topic hz /rgbd_images
ros2 topic info /front_camera/depth/color/points
ros2 topic info /rear_camera/depth/color/points
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link front_camera_color_optical_frame
ros2 run tf2_ros tf2_echo base_link rear_camera_color_optical_frame
```

Do not send a physical drive goal until all of these are true:

- `/odom` publishes.
- `/map` publishes.
- `/front_rgbd_image` and `/rear_rgbd_image` publish.
- `/front_camera/depth/color/points` publishes.
- `/rear_camera/depth/color/points` publishes.
- `map -> base_link` works.
- `base_link -> front_camera_color_optical_frame` works.
- `base_link -> rear_camera_color_optical_frame` works.
- Nav2 lifecycle nodes are active.
- `/safety_stop` is false in clear space.
- Bunker driver has a `/cmd_vel` subscriber.

## Historical Docker Status On 2026-08-10

These checks were run inside Docker container `trystan-bunker-navigation`.

Working:

- Terminal 1 camera/core:
  - `/front_camera/color/image_raw` publishes.
  - `/front_camera/aligned_depth_to_color/image_raw` publishes.
  - `/front_camera/depth/image_rect_raw` publishes.
  - `/front_camera/depth/color/points` publishes after the launch-level `pointcloud__neon_.*` runtime parameter fix.
- Core robot stack:
  - `/robot_description` publishes.
  - `/joint_states` publishes from the offline/default joint-state publisher.
  - `/tf` and `/tf_static` publish the robot/camera model.
  - `base_link -> front_camera_color_optical_frame` resolves.
  - `/yesense/imu_data_ros` publishes at about 200 Hz.
- Mode preset RGB-D sync:
  - `/front_rgbd_image` and `/rear_rgbd_image` publish at about 29 Hz when mapping/localization is started.
  - RTAB-Map detection is configured at 5 Hz; it intentionally consumes fewer
    frames than the RGB-D sync topics publish.

Blocking Nav2:

- `/odom` does not publish.
- `/cmd_vel` does not have the Bunker driver subscriber.
- The Bunker driver starts on `can4`, reports `Detected protocol: UNKNOWN`, then exits.
- `candump -L can2` shows active PiPER frames.
- `candump -L can4` showed no Bunker feedback frames during the sample, even though `can4` is configured at 500 kbit/s.
- On 2026-08-10, `can3` was also observed in `ERROR-PASSIVE`. The CAN setup
  script was updated to cycle a link in this stale/error state, but if it
  immediately returns to `ERROR-PASSIVE`, treat that as a Bunker bus-side issue.

Conclusion:

- The ROS camera and RGB-D path are now usable.
- Nav2 is still blocked before controller testing because Bunker odometry is unavailable.
- Fix the physical/CAN side of Bunker `can4` first: power, cable, adapter assignment, bus connection, bitrate, and whether the Bunker base is actually transmitting status frames.

## Common problems

| Symptom | Most likely cause | Check/fix |
|---|---|---|
| `/odom` missing | Bunker driver not initialized | Check CAN role/bitrate; Bunker should be `can4` at 500 kbit/s for the canonical setup |
| `Detected protocol: UNKNOWN` | Bunker driver is not receiving valid Bunker feedback frames | Identify adapters by serial, rerun `configure_can.sh can2 can4 can3`, then confirm `candump -L can4` shows Bunker frames |
| `can4` is `ERROR-PASSIVE` | CAN controller is seeing a bus error/no valid ACK/invalid physical bus state | Rerun `configure_can.sh can2 can4 can3`; if it returns to `ERROR-PASSIVE`, check Bunker power, e-stop, CAN H/L wiring, common ground, termination, and that the cable is connected to the serial-verified Bunker adapter |
| `configure_can.sh can2 can4 can4` exits with distinct-interface error | Rear PiPER was still passed as `can4`, same as Bunker | Use `front_piper_can:=can2`; the correct manual setup is `configure_can.sh can2 can4 can3` |
| RealSense `compressed_pub`, `compressedDepth_pub`, or `No plugins found!` during abort | Launch shutdown after an earlier error invalidated the RealSense node context | Fix the earlier launch error first. The container can list the image transports; these lines are usually secondary |
| RealSense `Device or resource busy` / `VIDIOC_S_FMT` | D435i USB/V4L device wedged or still recovering after a failed launch | Stop old camera launches, then recover only the camera with `ros2 run bunker_autonomy realsense_usb_recover`; restart Terminal 1 only if the camera node does not respawn. Terminal 1 defaults `reset_front_camera_usb:=true`, which resets the D435i before RealSense starts |
| Watchdog says `No RealSense USB device is currently enumerated` | Linux cannot currently see the D435i, so there is no camera device to reset | Do not reboot the Jetson or restart Docker first. Check/replug camera cable or hub power, confirm `rs-enumerate-devices -s`, then restart only the camera/Terminal 1 launch if needed |
| Running an old PiPER launch breaks CAN | Old command had reversed CAN arguments | Use `configure_can.sh can2 can4 can3`; front PiPER is `can2` at 1 Mbit/s, rear PiPER is `can3` at 1 Mbit/s, and Bunker is `can4` at 500 kbit/s |
| `sequence size exceeds remaining buffer` | DDS/domain pollution or stale participant | Kill stale nodes, use one domain, consider `ROS_LOCALHOST_ONLY=1` for local-only testing |
| RGB works but no `/front_rgbd_image` or `/rear_rgbd_image` | Missing aligned depth or camera info for that camera | Check Terminal 1 topics and rates |
| RViz `Frame [map] does not exist` | Localization/mapping not publishing map TF yet | Check RTAB-Map mapping/localization VS Code terminal-editor pane logs |
| RViz robot model red/no TF | Terminal 1 robot description/TF missing | Check `/robot_description`, `/tf_static`, and `tf2_echo base_link front_camera_color_optical_frame` |
| Goal accepted but robot does not move | Dry-run mode or safety stop | Check Nav2 mode, `/safety_stop`, and `/cmd_vel` subscriber |
| Map appears attached to robot | Fixed frame or TF chain wrong | Use fixed frame `map`; verify `map -> odom -> base_link` |

## Shutdown order

1. Cancel any active RViz goal.
2. Stop the exploration, Nav2, localization, or mapping VS Code terminal-editor panes.
3. Stop Terminal 1 hardware bringup.
4. On the host, revoke X11 access if desired:

```bash
xhost -SI:localuser:root
```

## Good-map requirements

A good map has:

- Coverage of every area where Nav2 will drive.
- Overlapping camera views and recognizable visual features.
- Loop closures from revisiting earlier areas.
- Straight walls without shifted duplicate copies.
- No false occupied patches in open floor space.
- Enough free space around doors for the full robot footprint.
- A stable robot pose after returning near the starting location.
- Many map nodes rather than one node recorded while stationary.

Avoid rapid turns. Revisit the starting area before finishing.
