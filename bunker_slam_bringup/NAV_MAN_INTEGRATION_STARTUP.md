# Nav-Man integration startup

This is the single maintained runbook for running the Trystan Bunker/Nav2 stack
and the Illiyas PiPER-X manipulation stack inside one Docker container:

```text
bunker-nav-man
```

This file is intentionally written like `NAV2_FULL_SYSTEM_STARTUP.md`: each mode
has copy-paste commands, and every manual terminal block includes the needed
environment setup.

Do not use these old integration docs for the current startup:

```text
MANIPULATION_TASK_COMMUNICATION.md
ILIYAS_ABOT_TRYSTAN_INTEGRATION.md
```

They were removed. This file replaces them.

## Current design

```text
Docker owner: bunker-nav-man only
tmux session: 0
ROS_DOMAIN_ID: 173
ROS_LOCALHOST_ONLY: 1
hardware owner: Terminal 1 only
automatic handoff service: disabled
manual manipulation/API calls: available but execution-gated
PiPER control gate: disabled in automatic startup by default
physical PiPER motion gate: explicit --allow-piper-motion launcher argument
RViz windows: one Nav/mapping RViz, one front PiPER MoveIt RViz
```

Hardware map:

```text
front PiPER: discover first, then pass resolved canX, 1 Mbit/s
rear PiPER:  discover first, then pass resolved canX, 1 Mbit/s
Bunker:      discover first, then pass resolved canX, 500 kbit/s
front D435i: discover first with rs-enumerate-devices, then pass serial
rear D435i:  disabled in the current simplified startup
Odometry:    raw Bunker `/odom`; no EKF in Terminal 1
H30/YESENSE: disabled in the current simplified startup
EKF:         not launched from Terminal 1
```

Current camera policy:

```text
front camera optical frame: front-facing calibrated D435i pose
front RealSense streams:    RGB + aligned depth + registered point cloud
front RealSense IMU:        disabled
rear RealSense:             disabled
```

RGB is still required because ArUco uses `/front_camera/color/image_raw`.
Aligned depth and point cloud are required because RTAB-Map and the
wall/touch/API stack use depth and `/front_camera/depth/color/points`.

The current integration passes CAN arguments in this shape:

```bash
arm_can:=FRONT_PIPER_CANX \
bunker_can:=BUNKER_CANX \
rear_piper_can:=REAR_PIPER_CANX \
front_piper_can:=FRONT_PIPER_CANX
```

Resolve the `canX` names before launch. The current lab robot has used
`FRONT_PIPER_CAN` for the front PiPER, `REAR_PIPER_CAN` for the rear PiPER, and `BUNKER_CAN` for the
Bunker, but public users should treat those as examples only because Linux
`canN` assignment can change.

## Integration with Trystan's work

Main issue this integration avoids:

```text
Do not start two owners for the same PiPER driver, RealSense camera, robot TF,
robot description, odometry, RTAB-Map, Nav2, or API port.
```

Earlier ABot/Iliyas startup commands were standalone. They could start their own
PiPER driver, RealSense node, TF publishers, MoveIt model, marker nodes, Agent
Server, and OpenClaw layer. In the integrated Nav-Man workflow, Trystan's
Terminal 1-3 startup owns the robot hardware and navigation stack. The Iliyas
terminals only add manipulation behavior on top of those existing topics.

Current ownership split:

| Component | Trystan/Nav-Man starts | Iliyas/ABot starts | Integration rule |
|---|---|---|---|
| PiPER-X hardware drivers | Yes, Terminal 1 starts front/rear PiPER on discovered `canX` links | No duplicate PiPER driver | Iliyas subscribes to `/joint_states` and commands the front trajectory bridge |
| Bunker driver | Yes, Terminal 1 starts Bunker on discovered `canX` link | No | Bunker owns raw `/odom` in this mode |
| RealSense cameras | Yes, Terminal 1 starts only the front D435i node in the current simplified startup | No duplicate camera node | Iliyas uses `/front_camera/color/image_raw`, `/front_camera/color/camera_info`, and `/front_camera/depth/color/points` |
| Robot URDF/TF | Yes, Terminal 1 publishes `/robot_description`, `/tf`, and `/tf_static` | No duplicate robot-state publisher or generic camera alias TF | MoveIt and Iliyas use Trystan's real front PiPER/camera frames |
| Joint states | Yes, Terminal 1 publishes integrated `/joint_states` with prefixed front/rear PiPER joints | No separate joint-state adapter in this integrated mode | API normalizes `front_piper_joint1..6` to raw `joint1..6` internally |
| RTAB-Map mapping/localization | Yes, Terminal 2 starts RTAB-Map and RGB-D sync | No | Iliyas marker/touch nodes only consume camera/point-cloud topics |
| Nav2 and landmarks | Yes, Terminal 3 starts Nav2 dry-run/drive and `/landmark_navigator` | No | Manipulation is called manually after navigation reaches the target |
| MoveIt for front PiPER | No in the original Nav2-only workflow; yes in Nav-Man Terminal 4-7 | Uses the integrated front PiPER MoveIt stack | MoveIt semantic model attaches to Trystan's `/robot_description` |
| ArUco detector | Optional debug in Trystan Terminal 1; normal Nav-Man detector is Terminal 8 | Yes, Terminal 8 starts the manipulation ArUco detector | It subscribes to Trystan's front camera topics |
| Search/approach/touch nodes | No | Yes, Terminal 9 and Terminal 10 | They use Trystan's camera, point cloud, TF, joint states, and MoveIt |
| HTTP API | No | Yes, Terminal 11 exposes lower-level API on `127.0.0.1:8892` | API calls ROS services/actions inside the same Docker |
| ABot Agent/OpenClaw layer | No hardware ownership | Optional Agent Server on `127.0.0.1:8893` | Agent calls `8892`; it must not own ROS hardware or TF |

Slide version:

```text
Iliyas starts:
- front PiPER MoveIt layer connected to Trystan's URDF/SRDF
- ArUco detector on Trystan's front camera topics
- marker search service
- wall approach / touch services
- lower-level HTTP API on 127.0.0.1:8892
- optional Iliyas ABot Agent Server on 127.0.0.1:8893, calling 8892 only

Trystan starts:
- bunker-nav-man Docker
- Terminal 1 hardware owner
- front PiPER driver on discovered `canX`
- rear PiPER driver on discovered `canX`
- Bunker driver on discovered `canX`
- front RealSense camera node only
- combined robot URDF, /robot_description, /tf, /tf_static
- integrated /joint_states
- raw Bunker /odom directly from Terminal 1; EKF is not launched
- Terminal 2 RTAB-Map mapping/localization
- Terminal 3 Nav2 dry-run/drive, depth safety, clicked goals, landmarks
- Nav/mapping RViz
```

Therefore, if an Iliyas command tries to start a second PiPER driver, second
RealSense camera, second robot-state publisher, second TF branch, second
RTAB-Map, second Nav2, or OpenClaw/Agent Server by default, do not use that
standalone command in this integration. Use Terminal 8-11 below instead.

## Mode 0: Host display and Docker shell

Run this on the robot host before starting RViz or any GUI tool:

```bash
export DISPLAY=:1
xhost +SI:localuser:root
```

Enter the Trystan Docker:

```bash
docker exec -it \
  -e DISPLAY=:1 \
  -e QT_X11_NO_MITSHM=1 \
  -e QT_QPA_PLATFORM=xcb \
  bunker-nav-man bash
```

Inside Docker, use this environment block in every manual terminal:

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

If one terminal uses a different `ROS_DOMAIN_ID` or `ROS_LOCALHOST_ONLY`, the
ROS graph can look empty even while nodes are running.

## Mode 1: Clean before startup

Use this before a new full run when you want to keep tmux session `0` and its
panes open. This kills the ROS/Nav-Man processes, but it does not kill tmux:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=173
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 run bunker_slam_bringup pkill_bunker_nav2_stack.sh
ros2 daemon stop
ros2 daemon start
ros2 node list
ros2 topic list
```

If you want one plain copy-paste `pkill` block that kills the whole Nav-Man
stack without killing tmux, use this:

```bash
pkill -TERM -f 'terminal1_sensors.launch.py|terminal2_robot.launch.py|terminal3_mapping.launch.py|nav2_bringup.launch.py|touch_marker_full_stack.launch.py|front_piper_description_bridge.py|front_piper_move_group_only.py|front_piper_moveit_tf_publisher.py|front_piper_moveit_rviz.py|front_piper_trajectory_bridge.py|start_nav_man_workflow.py|start_iliyas_abot_in_trystan.py' || true
pkill -TERM -f 'realsense2_camera_node|rgbd_sync|rgbdx_sync|rtabmap|move_group|rviz2|agx_arm_ctrl_single|bunker_base_node|robot_state_publisher|joint_state_prefixer|joint_state_publisher|ekf_node|yesense_node_publisher|piper_navigation_pose|piper_x_joint_preset|search_marker_node|wall_approach_node|piper_touch_marker_api|aruco_ros|aruco_single|static_transform_publisher|depth_route_monitor_node|sensor_fusion_node|safety_monitor_node|cmd_vel_mux_node|nav2_cmd_vel_safety_mux|controller_server|planner_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|smoother_server|lifecycle_manager' || true

sleep 2

pkill -KILL -f 'terminal1_sensors.launch.py|terminal2_robot.launch.py|terminal3_mapping.launch.py|nav2_bringup.launch.py|touch_marker_full_stack.launch.py|front_piper_description_bridge.py|front_piper_move_group_only.py|front_piper_moveit_tf_publisher.py|front_piper_moveit_rviz.py|front_piper_trajectory_bridge.py|start_nav_man_workflow.py|start_iliyas_abot_in_trystan.py' || true
pkill -KILL -f 'realsense2_camera_node|rgbd_sync|rgbdx_sync|rtabmap|move_group|rviz2|agx_arm_ctrl_single|bunker_base_node|robot_state_publisher|joint_state_prefixer|joint_state_publisher|ekf_node|yesense_node_publisher|piper_navigation_pose|piper_x_joint_preset|search_marker_node|wall_approach_node|piper_touch_marker_api|aruco_ros|aruco_single|static_transform_publisher|depth_route_monitor_node|sensor_fusion_node|safety_monitor_node|cmd_vel_mux_node|nav2_cmd_vel_safety_mux|controller_server|planner_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|smoother_server|lifecycle_manager' || true

ros2 daemon stop
ros2 daemon start
ros2 node list
ros2 topic list
```

Expected after cleanup:

```text
/parameter_events
/rosout
```

It is also acceptable if `ros2 node list` is empty. Do not continue if old
Bunker, RealSense, PiPER, MoveIt, RTAB-Map, Nav2, ArUco, or API nodes are still
running.

Camera-only cleanup, when you only want to release the front/rear RealSense and
RGB-D mapping pipeline without stopping PiPER, MoveIt, Nav2, CAN, or tmux:

```bash
CAMERA_PATTERN='dual_realsense\.launch\.py|realsense\.launch\.py|terminal3_mapping\.launch\.py|realsense2_camera_node|component_container.*realsense|rgbd_sync|rgbdx_sync|rgbd_odometry|point_cloud_xyzrgb|rtabmap_front_camera\.launch\.py|rtabmap\.launch\.py|rtabmap|rtabmapviz|camera_only_rviz\.launch\.py|camera_mapping_views\.launch\.py|minimal_rgbd_mapping\.launch\.py|standalone_camera_pipeline\.launch\.py|standalone_rgbd_yolo_mapping\.launch\.py|realsense_stream_watchdog|realsense_usb_recover'
PIDS="$(ps -eo pid=,cmd= | awk "/${CAMERA_PATTERN}/ && !/awk/ && !/bash -lc/ {print \$1}")"
[ -z "$PIDS" ] || kill -TERM $PIDS
sleep 2
PIDS="$(ps -eo pid=,cmd= | awk "/${CAMERA_PATTERN}/ && !/awk/ && !/bash -lc/ {print \$1}")"
[ -z "$PIDS" ] || kill -KILL $PIDS
ros2 daemon stop
ros2 daemon start
ros2 node list | grep -E 'camera|realsense|rgbd|rtabmap' || true
ros2 topic list | grep -E '^/(front_camera|front_rgbd_image)' || true
```

If those processes are owned by Docker root from the host, run the same block
inside `bunker-nav-man`:

```bash
docker exec -it bunker-nav-man bash
```

Manual fallback cleanup:

```bash
pkill -TERM -f 'realsense2_camera_node|rgbd_sync|rgbdx_sync|rtabmap|nav2|move_group|front_piper|rear_piper|agx_arm_ctrl|touch_marker|search_marker|wall_approach|piper_touch_marker_api|robot_state_publisher|static_transform_publisher|bunker_base_node|ekf_node|yesense_node_publisher' || true
sleep 2
pkill -KILL -f 'realsense2_camera_node|rgbd_sync|rgbdx_sync|rtabmap|nav2|move_group|front_piper|rear_piper|agx_arm_ctrl|touch_marker|search_marker|wall_approach|piper_touch_marker_api|robot_state_publisher|static_transform_publisher|bunker_base_node|ekf_node|yesense_node_publisher' || true
ros2 daemon stop
ros2 daemon start
ros2 node list
```

## Mode 2: Rebuild or refresh after edits

For normal launch/config/RViz/URDF/script/documentation edits, first try the
fast refresh if it exists:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash

if [ -x /ros2_ws/src/bunker_slam_bringup/scripts/refresh_without_colcon.sh ]; then
  /ros2_ws/src/bunker_slam_bringup/scripts/refresh_without_colcon.sh
fi

source /ros2_ws/install/setup.bash
```

If you changed the combined robot xacro colors, geometry, camera TF, or arm
mounts, also regenerate the expanded URDF that MoveIt reads:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

xacro src/bunker_dual_piper_nav2/urdf/bunker_dual_piper_d435i.urdf.xacro \
  > src/bunker_slam_bringup/descriptions/nav_man_full_robot.urdf
```

This is required because Terminal 4/5 read:

```text
/ros2_ws/src/bunker_slam_bringup/descriptions/nav_man_full_robot.urdf
```

If the Bunker color in MoveIt RViz still looks stale after changing the xacro,
restart Terminal 4, Terminal 5, and Terminal 7 after regenerating this file.

Use `colcon build` when you edit `CMakeLists.txt`, `package.xml`, Python entry
points, installed scripts, dependencies, Nav2 configs, RViz configs, or when a
new installed file does not show up through `ros2 run`.

For the Nav-Man startup, rebuild both Bunker packages because the active Nav2
config is in `bunker_slam_bringup`, and a duplicate Nav2 params file is kept in
`bunker_dual_piper_nav2`:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select bunker_slam_bringup bunker_dual_piper_nav2
source /ros2_ws/install/setup.bash
```

If the PiPER driver package changed, include `agx_arm_ctrl` as well:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select agx_arm_ctrl bunker_slam_bringup bunker_dual_piper_nav2
source /ros2_ws/install/setup.bash
```

If Terminal 8-11 or any Iliyas manipulation command prints this:

```text
Package not found
Package 'piper_x_aruco_wall_approach' not found
```

the Iliyas ROS package exists in source, but it is not registered in the current
Docker install index. Rebuild only the active Trystan-side copy:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src/piper_x_aruco_wall_approach
source /ros2_ws/install/setup.bash

ros2 pkg prefix piper_x_aruco_wall_approach
ros2 pkg executables piper_x_aruco_wall_approach
```

Expected:

```text
/ros2_ws/install/piper_x_aruco_wall_approach
piper_x_aruco_wall_approach search_marker_node
piper_x_aruco_wall_approach wall_approach_node
piper_x_aruco_wall_approach piper_touch_marker_api.py
```

Do not use a plain `--packages-select piper_x_aruco_wall_approach` if `colcon
list` shows two copies of the same package. This workspace can contain both the
active copy:

```text
/ros2_ws/src/piper_x_aruco_wall_approach
```

and the ABot mirror:

```text
/ros2_ws/src/ABot-Claw-piperX/external_ros2/piper_x_aruco_wall_approach
```

The Nav-Man startup uses the active Trystan-side copy.

If Docker dependencies changed, rebuild the image from the host:

```bash
cd /home/dase-orin/ros2_ws
docker compose -f docker/compose.yaml build bunker-nav-man
docker compose -f docker/compose.yaml up -d bunker-nav-man
```

Then re-enter Docker and source the environment again. Running ROS nodes do not
reload changed files automatically.

## Mode 3: Verify CAN and cameras before ROS launch

Check CAN interfaces:

```bash
for n in /sys/class/net/can*; do
  [ -e "$n" ] || continue
  n="$(basename "$n")"
  echo "=== $n ==="
  cat /sys/class/net/$n/device/../serial 2>/dev/null || true
  ip -details -statistics link show "$n" | sed -n '1,14p'
done
```

Expected shape:

```text
front PiPER  canX  1000000 bit/s
rear PiPER   canY  1000000 bit/s
Bunker       canZ  500000 bit/s
```

Configure and validate all Nav-Man CAN links:

```bash
ros2 run bunker_slam_bringup configure_can.sh FRONT_PIPER_CANX BUNKER_CANX REAR_PIPER_CANX
ip -details -statistics link show FRONT_PIPER_CANX
ip -details -statistics link show REAR_PIPER_CANX
ip -details -statistics link show BUNKER_CANX
```

Check passive frames:

```bash
timeout 3 candump -L FRONT_PIPER_CANX
timeout 3 candump -L REAR_PIPER_CANX
timeout 3 candump -L BUNKER_CANX
```

Expected:

```text
FRONT_PIPER_CANX is UP at 1000000 bit/s
REAR_PIPER_CANX is UP at 1000000 bit/s
BUNKER_CANX is UP at 500000 bit/s
FRONT_PIPER_CANX has front PiPER frames
REAR_PIPER_CANX has rear PiPER frames
BUNKER_CANX has Bunker frames
```

Check cameras:

```bash
rs-enumerate-devices -s
lsusb | grep -i -E 'intel|realsense|8086'
ls -l /dev/video*
```

Expected shape:

```text
FRONT_CAMERA_SERIAL
REAR_CAMERA_SERIAL
```

If the rear PiPER `canX` does not exist or rear PiPER firmware is not
publishing yet, use the front-only fallback mode until the rear arm is fixed.

## Mode 4: Full automatic tmux startup

This is the normal startup path. Run it inside Docker:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=173
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 run bunker_slam_bringup start_nav_man_workflow.py --replace \
  --database-path /ros2_ws/maps/site.db
tmux attach -t 0
```

The database must already exist for the default `localization` mode. To create
a new site database, use `--mapping-mode mapping --reset-database`. Automatic
startup denies PiPER motion unless `--allow-piper-motion` is supplied after
dry-run validation.

Expected tmux windows:

```text
t1_hardware          front camera, Bunker raw odom, front/rear PiPER, robot TF
t2_mapping           RTAB-Map localization with front RGB-D only
t3_nav2              Nav2 dry-run, no manipulation trigger
t4_srdf_description  integrated front PiPER URDF/SRDF publisher for MoveIt
t5_moveit            front PiPER MoveIt move_group using real front_piper_* links
t6_trajectory_bridge FollowJointTrajectory bridge to /front_piper/control/joint_states
t7_moveit_rviz       dedicated front PiPER MoveIt RViz
t8_aruco             integrated SRDF bridge plus ArUco ID 6 detector
t9_marker_search     /search_marker service
t10_wall_approach    /run_wall_approach and /run_marker_task services
t11_api_8892         HTTP API on 127.0.0.1:8892
t12_agent_8893       ABot Agent Server or bundled compatibility gateway on 8893
t13_watchdogs        live watchdog/source monitor for Nav2 mux, safety, camera
t14_frontier_mrtsp   upstream frontier_exploration_ros2 cold-idle explorer
```

The MoveIt RViz planning workspace box is intentionally small:

```text
center: 0.0, 0.1, 0.85 m
size:   1.150 x 0.600 x 0.600 m
```

`1.150 m` is the requested 115 cm longitudinal dimension. `0.600 m` is the
requested 60 cm cross dimension. The Nav2 local and global costmap footprints
use the same centered 1.15 m x 0.60 m rectangle:

```text
[[-0.575, -0.30], [0.575, -0.30], [0.575, 0.30], [-0.575, 0.30]]
footprint_padding: 0.0
```

If you want an extra safety margin later, increase `footprint_padding`; keep it
at `0.0` when you want the effective Nav2 footprint to stay exactly 115 x 60 cm.

Useful tmux keys:

```text
Ctrl-b n       next window
Ctrl-b p       previous window
Ctrl-b 0       t1_hardware
Ctrl-b 5       t6_trajectory_bridge
Ctrl-b 7       t8_aruco
Ctrl-b 8       t9_marker_search
Ctrl-b 9       t10_wall_approach
Ctrl-b n       continue to t11_api_8892 through t14_frontier_mrtsp
Ctrl-b d       detach but keep running
```

`t14_frontier_mrtsp` does not move the robot on launch. It starts
`frontier_exploration_ros2_bunker.launch.py` with `autostart:=false` and waits
for:

```bash
ros2 run frontier_exploration_ros2 frontier_exploration_ctl start
```

Use it only after `t3_nav2` has been restarted in drive mode and a short manual
Nav2 goal works. Stop it with:

```bash
ros2 run frontier_exploration_ros2 frontier_exploration_ctl stop
```

## Mode 5: Front-only fallback automatic startup

Use this when rear PiPER `REAR_PIPER_CAN` is unavailable but you still want Bunker, front
camera, front PiPER, MoveIt, RTAB-Map, Nav2, and the API to start:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=173
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 run bunker_slam_bringup start_nav_man_workflow.py --replace \
  --disable-rear-piper --database-path /ros2_ws/maps/site.db
tmux attach -t 0
```

This disables only the rear PiPER driver. The rear camera is already disabled in
the current simplified startup.

## Mode 6: Manual terminal-by-terminal startup

Use this mode when you do not want the tmux launcher to start everything for
you. Start each terminal in order. Keep Terminal 1 as the only hardware owner.

### Terminal 1: hardware

This starts the front camera, Bunker raw odometry, front/rear PiPER drivers,
robot TF, and the initial PiPER navigation pose. The rear camera, H30/YESENSE
IMU publisher are disabled in the default simplified startup. EKF is not
launched from Terminal 1. The
initial-pose move uses the
documented parked joints:

```text
front_piper_parked_joint1=-1.6
rear_piper_parked_joint1=1.6
other arm joints=0.0
```

Keep the workspace clear before Terminal 1 starts. The PiPER external control
gate is open by default for the integrated API path, but physical arm motion is
still controlled by `allow_piper_motion`. Set `allow_piper_motion:=false` when
you want drivers and feedback only.

No generic camera alias TF is required. Illiyas nodes subscribe directly to the
Trystan camera topics and frames:

```text
/front_camera/color/image_raw
/front_camera/color/camera_info
/front_camera/depth/color/points
front_camera_color_optical_frame
```

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=173
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 launch bunker_slam_bringup terminal1_sensors.launch.py \
  arm_can:=FRONT_PIPER_CAN \
  bunker_can:=BUNKER_CAN \
  rear_piper_can:=REAR_PIPER_CAN \
  front_piper_can:=FRONT_PIPER_CAN \
  launch_front_camera:=true \
  launch_rear_camera:=false \
  reset_front_camera_usb:=false \
  front_camera_serial:=FRONT_CAMERA_SERIAL \
  rear_camera_serial:=REAR_CAMERA_SERIAL \
  configure_can:=true \
  start_piper_drivers:=true \
  start_front_piper_driver:=true \
  start_rear_piper_driver:=true \
  piper_control_enabled:=true \
  piper_effector_type:=agx_gripper \
  front_piper_fw_version:=v189 \
  rear_piper_fw_version:=v189 \
  front_piper_tcp_offset:='[0.0, 0.0, 0.1425, 0.0, 0.0, 0.0]' \
  run_piper_initial_pose:=true \
  allow_piper_motion:=true \
  start_h30_imu:=false
```

### Terminal 2: mapping/localization

This runs RTAB-Map with the front RGB-D camera only.

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=173
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 launch bunker_slam_bringup terminal3_mapping.launch.py \
  mode:=localization \
  localization_backend:=rtabmap \
  mapping_camera:=front \
  database_path:=/ros2_ws/maps/bunker_dual_rgbd_v2.db \
  reset_database:=false \
  use_rviz:=false
```

For a new map:

```bash
ros2 launch bunker_slam_bringup terminal3_mapping.launch.py \
  mode:=mapping \
  localization_backend:=rtabmap \
  mapping_camera:=front \
  database_path:=/ros2_ws/maps/bunker_dual_rgbd_v2.db \
  reset_database:=true \
  use_rviz:=false
```

For front-camera-only mapping:

```bash
ros2 launch bunker_slam_bringup terminal3_mapping.launch.py \
  mode:=mapping \
  localization_backend:=rtabmap \
  mapping_camera:=front \
  database_path:=/ros2_ws/maps/bunker_dual_rgbd_v2.db \
  reset_database:=false \
  use_rviz:=false
```

For rear-camera-only mapping:

```bash
ros2 launch bunker_slam_bringup terminal3_mapping.launch.py \
  mode:=mapping \
  localization_backend:=rtabmap \
  mapping_camera:=rear \
  database_path:=/ros2_ws/maps/bunker_dual_rgbd_v2.db \
  reset_database:=false \
  use_rviz:=false
```

### Terminal 3: Nav2 dry-run

This starts Nav2 without automatic manipulation trigger and without chassis
motion.

Current Nav2 goal tolerances are intentionally strict:

```text
xy_goal_tolerance:  0.05 m
yaw_goal_tolerance: 0.05 rad
```

`0.05 rad` is about 2.9 degrees. If the robot oscillates at the goal or has
trouble declaring success, relax these values in
`bunker_slam_bringup/config/nav2_bunker.yaml`.

Current Nav2 recovery/replanning behavior:

```text
reverse local motion: enabled on known/free mapped space
min_speed_xy:         0.10 m/s
min_speed_theta:      0.20 rad/s
BackUp recovery:      0.15 m at 0.10 m/s
BT replanning rate:   5 Hz
```

This lets Nav2 refresh the global path after the robot is manually moved and
lets the local planner command reverse motion when that is the valid route in
already mapped/free space. `min_vel_y` remains `0.0` because the Bunker base is
not omnidirectional and cannot strafe sideways.

Current camera/map update tuning:

```text
RealSense RGB/depth profile: 640x480 at 15 Hz
RTAB-Map DetectionRate:      3 Hz
PointCloud2 mode:            unordered, allow no-texture points
```

`/map` is intentionally limited to about 3 Hz to reduce CPU pressure from
RTAB-Map. Depth and aligned-depth streams still target 15 Hz. The point cloud is
kept available for Nav2 safety and marker touch, but unordered/no-texture mode
reduces pointcloud overhead so it is less likely to starve RGB-D mapping.

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=173
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 launch bunker_slam_bringup nav2_bringup.launch.py \
  mode:=dry_run \
  allow_motion:=false \
  use_rviz:=true \
  start_depth_safety:=true \
  start_clicked_goal:=true \
  start_landmark_navigator:=true \
  start_manipulation_trigger:=false \
  landmark_path:=/ros2_ws/maps/manual_nav_landmarks.json
```

Dry-run does not command the chassis. In the current default routing, Nav2
publishes to `/cmd_vel_debug` in dry-run mode.

Only after dry-run behavior is correct, restart Terminal 3 in drive mode:

```bash
ros2 launch bunker_slam_bringup nav2_bringup.launch.py \
  mode:=drive \
  allow_motion:=true \
  use_rviz:=true \
  start_depth_safety:=true \
  start_clicked_goal:=true \
  start_landmark_navigator:=true \
  start_manipulation_trigger:=false \
  landmark_path:=/ros2_ws/maps/manual_nav_landmarks.json
```

Drive mode now uses direct command routing by default:

```text
controller_server -> /cmd_vel -> bunker_base_node
behavior_server   -> /cmd_vel -> bunker_base_node
```

The older velocity-smoother/safety-mux chain is still available, but only if
you explicitly add:

```bash
use_cmd_vel_mux:=true
```

That legacy route is:

```text
controller_server -> /nav2/cmd_vel_raw
velocity_smoother -> /cmd_vel_autonomy
nav2_cmd_vel_safety_mux -> /cmd_vel
```

### Terminal 4: front PiPER URDF/SRDF description

This publishes the integrated front PiPER robot description and semantic
description that MoveIt and Illiyas use. It is not a standalone AgileX arm.
The URDF is the full Nav-Man robot model, and the SRDF arm group uses the real
front PiPER TF branch:

```text
front_piper_base_link -> front_piper_flange_link
front_piper_joint1 ... front_piper_joint6
```

MoveIt subscribes to `/joint_states`, which is the prefixed joint-state output
from Terminal 1. Do not run a second robot_state_publisher for unprefixed
`link1..link6`.

Terminal 4 is the authoritative SRDF publisher for the manipulation stack:

```text
/front_piper/robot_description_semantic
```

Terminal 9 and Terminal 10 must remap `robot_description_semantic` to that live
topic. Do not point them at `/front_piper/integrated_robot_description_semantic`
unless you also start a persistent publisher for that topic.

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=173
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 run bunker_slam_bringup front_piper_description_bridge.py
```

### Terminal 5: front PiPER MoveIt

This starts `/front_piper/move_group` using the same integrated full-robot
URDF/SRDF model as Terminal 4. MoveIt plans with `front_piper_*` links and
joint names, so it stays coupled to the real front PiPER TF.

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=173
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 run bunker_slam_bringup front_piper_move_group_only.py
```

### Terminal 6: trajectory bridge

This exposes the front PiPER FollowJointTrajectory action and bridges MoveIt
trajectories to `/front_piper/control/joint_states`. MoveIt sends prefixed
joint names such as `front_piper_joint1`; this bridge translates them to the
physical PiPER driver command names `joint1..joint6`.

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=173
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 run bunker_slam_bringup front_piper_trajectory_bridge.py
```

### Terminal 7: front PiPER MoveIt RViz

This is the second RViz window. It is only for the integrated front PiPER
MoveIt model, planning scene, and interactive marker.

This RViz config intentionally does not include a separate `RobotModel`
display. The Nav/mapping RViz owns the normal robot visualization. In this
MoveIt RViz, the `MotionPlanning` display remains enabled but its scene robot
mesh is hidden so the robot is not drawn twice:

```text
Scene Robot / Robot Alpha: 0
Scene Robot / Show Robot Visual: false
Scene Robot / Show Robot Collision: false
```

If the Nav/mapping RViz shows the Bunker correctly but MoveIt RViz paints the
robot red, do not treat that as a URDF color issue first. MoveIt can color the
current robot state red when the semantic collision matrix says the mounted
robot is in self-collision. The integrated SRDF therefore disables collisions
for the fixed Bunker/support/mount links and the adjacent PiPER base links:

```text
base_link <-> upper_frame_link
upper_frame_link <-> front_mounting_plate_link
upper_frame_link <-> rear_mounting_plate_link
front_mounting_plate_link <-> front_piper_base_link
rear_mounting_plate_link <-> rear_piper_base_link
```

Those entries are generated by:

```text
/ros2_ws/src/bunker_slam_bringup/scripts/front_piper_integrated_moveit_model.py
```

After changing this SRDF generator, rebuild and restart Terminal 4, Terminal 5,
and Terminal 7. `move_group` reads `robot_description_semantic` only at startup;
an already-running MoveIt/RViz process will not reload the SRDF from disk.

The green box is the MoveIt planning workspace. It is not a physical obstacle.
The current config keeps it fully opaque but small:

```text
Planning Request / Workspace / Size X/Y/Z: 0.5 m
Scene Geometry / Scene Alpha: 1.0
```

If RViz still shows workspace size `2.00`, close and restart Terminal 7 so it
reloads `front_piper_moveit.rviz`.

The same RViz config also enables TF names:

```text
Front PiPER TF / Show Names: true
Front PiPER TF / Marker Scale: 0.08
```

Use those labels to identify unknown TF frames visually. For a text list, run:

```bash
ros2 run tf2_tools view_frames
```

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=173
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export DISPLAY=:1
export QT_QPA_PLATFORM=xcb
export QT_X11_NO_MITSHM=1

ros2 run bunker_slam_bringup front_piper_moveit_rviz.py
```

### Terminal 8: Illiyas ArUco detector

This starts only the ArUco detector. It does not start another camera, another
PiPER driver, another SRDF publisher, or a hand-eye TF publisher.

The ArUco target is ID `6` with physical size `0.06 m` / `60 mm x 60 mm`.

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=173
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 run aruco_ros single --ros-args \
  -p marker_id:=6 \
  -p marker_size:=0.06 \
  -p image_is_rectified:=true \
  -p reference_frame:=base_link \
  -p camera_frame:=front_camera_color_optical_frame \
  -p marker_frame:=aruco_marker_frame \
  -r /image:=/front_camera/color/image_raw \
  -r /camera_info:=/front_camera/color/camera_info
```

### Terminal 9: Illiyas marker search service

This starts only `/search_marker`.

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=173
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

PIPER_ARUCO_PREFIX="$(ros2 pkg prefix piper_x_aruco_wall_approach 2>/dev/null)" || {
  echo "Package piper_x_aruco_wall_approach not found."
  echo "Run Mode 2: rebuild the active Iliyas package with --base-paths src/piper_x_aruco_wall_approach."
  exit 1
}
PIPER_ARUCO_SHARE="$PIPER_ARUCO_PREFIX/share/piper_x_aruco_wall_approach"

ros2 run piper_x_aruco_wall_approach search_marker_node --ros-args \
  --params-file "$PIPER_ARUCO_SHARE/config/piper_x_search_poses.yaml" \
  -p aruco_pose_topic:=/aruco_single/pose \
  -p marker_id:=6 \
  -p joint_state_topic:=/joint_states \
  -p move_group_namespace:=front_piper \
  -p controller_manager_service:=/front_piper/controller_manager/list_hardware_components \
  -r joint_states:=/joint_states \
  -r robot_description:=/robot_description \
  -r robot_description_semantic:=/front_piper/robot_description_semantic
```

Manual search service call:

```bash
ros2 service call /search_marker \
  piper_x_aruco_wall_approach/srv/SearchMarker \
  "{execute: true, direction: 'auto', max_steps: 100}"
```

### Terminal 10: Illiyas wall approach/touch services

This starts `/run_wall_approach` and `/run_marker_task`.

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=173
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

PIPER_ARUCO_PREFIX="$(ros2 pkg prefix piper_x_aruco_wall_approach 2>/dev/null)" || {
  echo "Package piper_x_aruco_wall_approach not found."
  echo "Run Mode 2: rebuild the active Iliyas package with --base-paths src/piper_x_aruco_wall_approach."
  exit 1
}
PIPER_ARUCO_SHARE="$PIPER_ARUCO_PREFIX/share/piper_x_aruco_wall_approach"

ros2 run piper_x_aruco_wall_approach wall_approach_node --ros-args \
  --params-file "$PIPER_ARUCO_SHARE/config/wall_approach.yaml" \
  -p execute:=false \
  -p clearance:=0.05 \
  -p final_clearance:=0.005 \
  -p retract_after:=true \
  -p prefer_elbow_motion:=true \
  -p goal_orientation_tolerance:=0.35 \
  -p move_group_namespace:=front_piper \
  -p point_cloud_topic:=/front_camera/depth/color/points \
  -r joint_states:=/joint_states \
  -r robot_description:=/robot_description \
  -r robot_description_semantic:=/front_piper/robot_description_semantic
```

Manual touch service call:

```bash
ros2 service call /run_marker_task \
  piper_x_aruco_wall_approach/srv/RunMarkerTask \
  "{mode: 'touch', execute: true, pre_clearance_m: 0.05, final_clearance_m: 0.005, retract_distance_m: 0.05, final_velocity_scaling: 0.05, retract_after: true}"
```

Manual approach service call:

```bash
ros2 service call /run_marker_task \
  piper_x_aruco_wall_approach/srv/RunMarkerTask \
  "{mode: 'approach', execute: true, pre_clearance_m: 0.05, final_clearance_m: 0.005, retract_distance_m: 0.05, final_velocity_scaling: 0.05, retract_after: false}"
```

### Terminal 11: Illiyas HTTP API on 8892

This starts only the HTTP API. The API accepts integrated joint names such as
`front_piper_joint1` from `/joint_states` and normalizes them internally to the
raw PiPER command names `joint1..joint6` for `go-home`, `go-previous`, and
saved-pose commands.

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=173
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PIPER_TOUCH_ALLOW_EXECUTION=1

PIPER_ARUCO_PREFIX="$(ros2 pkg prefix piper_x_aruco_wall_approach 2>/dev/null)" || {
  echo "Package piper_x_aruco_wall_approach not found."
  echo "Run Mode 2: rebuild the active Iliyas package with --base-paths src/piper_x_aruco_wall_approach."
  exit 1
}
PIPER_ARUCO_SHARE="$PIPER_ARUCO_PREFIX/share/piper_x_aruco_wall_approach"

ros2 run piper_x_aruco_wall_approach piper_touch_marker_api.py \
  --host 127.0.0.1 \
  --port 8892 \
  --marker-id 6 \
  --marker-size-m 0.06 \
  --point-cloud-topic /front_camera/depth/color/points \
  --marker-timeout-s 1.0 \
  --point-cloud-timeout-s 2.0 \
  --home-pose-file "$PIPER_ARUCO_SHARE/config/piper_x_home_pose.yaml" \
  --previous-pose-file "$PIPER_ARUCO_SHARE/config/piper_x_previous_pose.yaml" \
  --found-marker-pose-file "$PIPER_ARUCO_SHARE/config/piper_x_found_marker_pose.yaml" \
  --joint-state-topic /joint_states \
  --joint-state-timeout-s 2.5 \
  --trajectory-action /front_piper/arm_controller/follow_joint_trajectory \
  --enable-service /front_piper/enable_agx_arm \
  --controller-manager-service /front_piper/controller_manager/list_hardware_components \
  --command-joint-prefix front_piper_
```

Important execution-gate note:

```text
Current Nav-Man 8892 API gate: PIPER_TOUCH_ALLOW_EXECUTION=1
Older PiPER-X agent message:   PIPER_X_AGENT_ALLOW_EXECUTION=1
```

If you see `PiPER-X Agent Server execution is disabled`, that message is from
the older agent path. For this integrated startup, run Terminal 11 exactly as
shown above so the active `piper_touch_marker_api.py` process receives
`PIPER_TOUCH_ALLOW_EXECUTION=1`.

Check:

```bash
curl -s http://127.0.0.1:8892/health | python3 -m json.tool
```

### Terminal 12: Iliyas ABot Agent Server on 8893

Use the external ABot Agent Server for the supplied Iliyas/OpenClaw workflow.
It forwards validated tool requests to 8892 and must not start another PiPER
driver, camera, robot-state-publisher, MoveIt stack, RTAB-Map, Nav2, or TF
branch. Install it outside `/ros2_ws/src` as described in
`docs/iliyas_openclaw_integration.md`.

```bash
cd /opt/nav-man-agent/ABot-Claw-piperX
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=173
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PIPER_X_AGENT_ALLOW_EXECUTION=0
export PIPER_TOUCH_ALLOW_EXECUTION=0
export PIPER_X_MARKER_API_URL=http://127.0.0.1:8892
export PIPER_X_MARKER_ID=6
export PIPER_X_MARKER_SIZE_M=0.06
export PIPER_X_JOINT_STATE_TOPIC=/front_piper/feedback/joint_states
export PIPER_X_GRIPPER_CONTROL_TOPIC=/front_piper/control/joint_states
export PIPER_X_TRAJECTORY_ACTION=/front_piper/arm_controller/follow_joint_trajectory

./robot_layer/arm_piper_x/agent_server/start_piper_x_agent_server.sh
```

Check:

```bash
curl -s http://127.0.0.1:8893/health | python3 -m json.tool
```

For automatic startup, pass:

```text
--abot-root /opt/nav-man-agent/ABot-Claw-piperX
```

When the external checkout is not installed, the launcher uses the bundled
8893 compatibility gateway instead. Do not run both implementations on the
same port. Start the OpenClaw gateway/TUI only after 8892 and 8893 are healthy.
The supplied reference uses the external checkout's deployment script; confirm
it exists in the selected revision:

```bash
./deployment/scripts/start_openclaw_trystan.sh gateway
./deployment/scripts/start_openclaw_trystan.sh tui
```

### Manual controls

For the current integrated startup, `piper_control_enabled:=true` opens the
front PiPER control gate at bringup. Use this manual call only to confirm or
reopen the gate after you have intentionally closed it:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=173
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 service call /front_piper/control_enable std_srvs/srv/SetBool "{data: true}"
```

Close it again when you want to block front PiPER control commands:

```bash
ros2 service call /front_piper/control_enable std_srvs/srv/SetBool "{data: false}"
```

Rear gate, only if you are explicitly testing the rear PiPER:

```bash
ros2 service call /rear_piper/control_enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /rear_piper/control_enable std_srvs/srv/SetBool "{data: false}"
```

## Mode 7: Validation checks

Run these after the automatic tmux startup or after manual Terminal 1-7 startup.

### CAN and odometry

```bash
ip -details link show FRONT_PIPER_CAN
ip -details link show REAR_PIPER_CAN
ip -details link show BUNKER_CAN

ros2 topic echo --once /odom
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
```

Expected:

```text
/odom publishes directly from the Bunker base driver
odom -> base_link is owned by the Bunker base driver
no /yesense/imu_data_ros publisher is expected in the simplified startup
odom -> base_link exists
```

### Cameras

```bash
ros2 topic echo --once /front_camera/color/camera_info
ros2 topic echo --once /front_camera/color/image_raw --field header.frame_id
ros2 topic echo --once /front_rgbd_image --field header.frame_id
ros2 topic echo --once /front_camera/depth/color/points

ros2 topic hz /front_camera/color/image_raw
ros2 topic hz /front_camera/depth/color/points
```

Expected:

```text
front RGB image publishes
front aligned depth point cloud publishes
front image/RGB-D frame_id is front_camera_color_optical_frame
rear camera topics are not required in the current simplified startup
```

### PiPER feedback

```bash
ros2 topic echo --once /front_piper/feedback/joint_states
ros2 topic echo --once /rear_piper/feedback/joint_states
```

Expected:

```text
front PiPER joint feedback publishes from FRONT_PIPER_CAN
rear PiPER joint feedback publishes from REAR_PIPER_CAN
```

If rear PiPER is disabled or firmware is missing, `/rear_piper/feedback/joint_states`
will not be valid. Use the front-only fallback mode.

### MoveIt and API

```bash
ros2 action info /front_piper/arm_controller/follow_joint_trajectory
ros2 service list | grep -E '^/search_marker$|^/run_marker_task$'
curl -fsS http://127.0.0.1:8892/health; echo
ros2 run tf2_ros tf2_echo base_link front_piper_link1
ros2 run tf2_ros tf2_echo base_link front_piper_flange_link
ros2 run tf2_ros tf2_echo base_link front_piper_gripper_base
```

Expected:

```text
/front_piper/arm_controller/follow_joint_trajectory exists
/search_marker exists
/run_marker_task exists
API health returns ready when camera, MoveIt, and joint feedback are fresh
MoveIt TF resolves base_link -> front_piper_link1/front_piper_flange_link/front_piper_gripper_base
```

`marker_visible:false` is not a startup failure. It only means the ArUco marker
is not visible right now.

### Mapping

```bash
ros2 topic hz /front_rgbd_image
ros2 topic echo --once /map
ros2 run tf2_ros tf2_echo map base_link
```

Expected with `mapping_camera:=front`:

```text
/front_rgbd_image publishes
/map publishes from RTAB-Map
map -> odom -> base_link exists
```

### No automatic handoff

This startup intentionally does not start automatic handoff request topics or
services. Terminal 11 does publish the passive 8892 API completion status topic
`/manipulation_task/finished`; that topic is not an automatic trigger.

```bash
ros2 node list | grep -E 'handoff|abotclaw' || true
ros2 topic list | grep -E '^/abotclaw|^/front_piper/task/start|^/manipulation_task/start' || true
```

Expected: no output.

## Mode 8: Home, door, landmark, and manipulation workflow

This is the normal manual communication flow between Nav2 and manipulation:

```text
landmark_navigator -> Nav2 /navigate_to_pose
door arrival       -> /door_navigation/arrived
home arrival       -> /home_navigation/arrived
operator decision  -> Illiyas API on 127.0.0.1:8892
Illiyas API        -> /search_marker, /run_marker_task, MoveIt trajectory action
Illiyas API        -> /manipulation_task/finished status Bool
```

There is still no automatic handoff service in this integration. You decide when
to call the manipulation API.

The 8892 API publishes:

```text
/manipulation_task/finished  std_msgs/msg/Bool
```

The topic uses the same pulse style as `/door_navigation/arrived` and
`/home_navigation/arrived`: it publishes `false` while idle/running, then 10
`true` messages at 2 Hz when one outer 8892 manipulation command is complete,
then returns to `false`.

For `/tools/piper/search-marker`, “marker not found” still counts as finished:
the search command completed and found nothing. For `/tools/piper/touch-marker`
with `execute:true`, the API returns the arm to the nav pose first, then pulses
`/manipulation_task/finished=true`.

### Save home and door landmarks

The landmark file used by this startup is:

```bash
/ros2_ws/maps/manual_nav_landmarks.json
```

After mapping/localization works and `map -> base_link` is valid, place the
robot at the real home position and save:

```bash
ros2 topic pub --once /landmark_navigator/save_current_pose std_msgs/msg/String "{data: home}"
```

Move or drive to the real door position and save:

```bash
ros2 topic pub --once /landmark_navigator/save_current_pose std_msgs/msg/String "{data: door}"
```

Check the saved RViz markers:

```bash
ros2 topic echo --once /landmark_navigator/markers
python3 -m json.tool /ros2_ws/maps/manual_nav_landmarks.json
```

### Go to home or door

Go to `door`:

```bash
ros2 topic pub --once /landmark_navigator/go_marker std_msgs/msg/String "{data: door}"
```

Go to `home`:

```bash
ros2 topic pub --once /landmark_navigator/go_marker std_msgs/msg/String "{data: home}"
```

Or call the home service:

```bash
ros2 service call /landmark_navigator/go_home std_srvs/srv/Trigger "{}"
```

Check Nav2/landmark communication:

```bash
ros2 topic echo /door_navigation/arrived --field data | tr '[:upper:]' '[:lower:]'
ros2 topic echo /home_navigation/arrived --field data | tr '[:upper:]' '[:lower:]'
ros2 topic hz /door_navigation/arrived
ros2 topic hz /home_navigation/arrived
ros2 topic info -v /door_navigation/arrived
ros2 topic info -v /home_navigation/arrived
ros2 topic echo /landmark_navigator/navigation_direction
ros2 topic echo /landmark_navigator/active_camera
```

Expected while no matching arrival has happened yet, or after a new goal starts:

```text
false
```

Expected after arriving at `door`: `/door_navigation/arrived` publishes
10 `true` messages at 2 Hz, then returns to `false`:

```text
true
true
...
false
```

Expected after arriving at `home`: `/home_navigation/arrived` publishes
10 `true` messages at 2 Hz, then returns to `false`:

```text
true
true
...
false
```

`/door_navigation/arrived` and `/home_navigation/arrived` are 2 Hz
`std_msgs/msg/Bool` topics from `/landmark_navigator`. They are not one-shot
strings. After a matching Nav2 arrival, the matching topic pulses `true` 10
times, then goes back to `false`.
Using `--field data` hides the ROS field name and prints only the bool value.
The `tr` pipe forces lowercase `false`/`true` output.

If `ros2 topic info -v /door_navigation/arrived` still shows
`std_msgs/msg/String`, Terminal 3/Nav2 is still running the old process. Restart
Terminal 3 or restart the full tmux session after rebuilding.

Expected when going from `door` back to `home`:

```text
/landmark_navigator/navigation_direction: reverse
/landmark_navigator/active_camera: rear_camera
```

### Add another named landmark

Example for a table or manipulation station:

```bash
ros2 topic pub --once /landmark_navigator/save_current_pose std_msgs/msg/String "{data: table}"
ros2 topic pub --once /landmark_navigator/go_marker std_msgs/msg/String "{data: table}"
```

To draw RViz connection lines, edit:

```bash
/ros2_ws/maps/manual_nav_landmarks.json
```

Example:

```json
"connections": [
  ["home", "door"],
  ["door", "table"]
]
```

Restart Nav2 or `/landmark_navigator` after editing the file.

### Manipulation after door arrival

When the robot has reached `door`, check the manipulation stack:

```bash
curl -fsS http://127.0.0.1:8892/health; echo
ros2 service list | grep -E '^/search_marker$|^/run_marker_task$'
ros2 action info /front_piper/arm_controller/follow_joint_trajectory
```

Dry search for marker 6. This should not move the arm:

```bash
curl -fsS -X POST http://127.0.0.1:8892/tools/piper/search-marker \
  -H "Content-Type: application/json" \
  -d '{"execute":false,"arm":"front","direction":"auto","max_steps":100}'; echo
```

Before a physical search, confirm the front PiPER control gate is open. This is
normally already true after Terminal 1 starts with `piper_control_enabled:=true`,
but this call is safe to repeat when the workspace is clear:

```bash
ros2 service call /front_piper/control_enable std_srvs/srv/SetBool "{data: true}"
```

Physical search for marker 6:

```bash
curl -fsS -X POST http://127.0.0.1:8892/tools/piper/search-marker \
  -H "Content-Type: application/json" \
  -d '{"execute":true,"arm":"front","direction":"auto","max_steps":100}'; echo
```

After a successful search, the API saves the found marker arm pose. Return to it
later with:

```bash
curl -fsS -X POST http://127.0.0.1:8892/tools/piper/go-found-marker \
  -H "Content-Type: application/json" \
  -d '{"execute":false,"arm":"front","duration_s":6.0}'; echo
```

Approach marker without touching:

```bash
curl -fsS -X POST http://127.0.0.1:8892/tools/piper/approach-marker \
  -H "Content-Type: application/json" \
  -d '{"execute":false,"arm":"front","pre_clearance_m":0.05,"final_clearance_m":0.005,"retract_after":true}'; echo
```

Touch marker. This can move the arm when `execute:true` and the front control
gate is open. After a successful physical touch, the 8892 API automatically
returns the front PiPER to nav pose before publishing
`/manipulation_task/finished=true`:

```bash
curl -fsS -X POST http://127.0.0.1:8892/tools/piper/touch-marker \
  -H "Content-Type: application/json" \
  -d '{"execute":true,"arm":"front","pre_clearance_m":0.05,"final_clearance_m":0.005,"retract_after":true,"return_home_after":false}'; echo
```

Watch manipulation completion continuously:

```bash
ros2 topic echo /manipulation_task/finished --field data | tr '[:upper:]' '[:lower:]'
```

Expected during idle/running:

```text
false
false
false
```

Expected after one completed 8892 command:

```text
true
true
true
...
```

The true pulse lasts 10 messages at 2 Hz, then the topic returns to `false`.

Return the arm to the saved home pose:

```bash
curl -fsS -X POST http://127.0.0.1:8892/tools/piper/go-home \
  -H "Content-Type: application/json" \
  -d '{"execute":true,"arm":"front","duration_s":6.0}'; echo
```

If this used to fail with:

```text
joint state missing required joints: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
```

that means the API was reading integrated `/joint_states` names such as
`front_piper_joint1` but validating only raw `joint1`. The current API converts
`front_piper_joint1..front_piper_joint6` to `joint1..joint6` internally before
saving or commanding poses.

Return the arm to the previous saved pose:

```bash
curl -fsS -X POST http://127.0.0.1:8892/tools/piper/go-previous \
  -H "Content-Type: application/json" \
  -d '{"execute":true,"arm":"front","duration_s":6.0}'; echo
```

Return the arm to the saved found-marker pose:

```bash
curl -fsS -X POST http://127.0.0.1:8892/tools/piper/go-found-marker \
  -H "Content-Type: application/json" \
  -d '{"execute":true,"arm":"front","duration_s":6.0}'; echo
```

Move one bounded search step instead of full auto search:

```bash
curl -fsS -X POST http://127.0.0.1:8892/tools/piper/search-step \
  -H "Content-Type: application/json" \
  -d '{"execute":true,"arm":"front","direction":"left","max_steps":1}'; echo
```

Useful directions:

```text
left, right, up, down, up_left, up_right, down_left, down_right, center, current
```

If curl returns only `409`, `422`, `502`, or `503`, rerun with `-i` and without
`-f` so the response body explains the reason:

```bash
curl -sS -i -X POST http://127.0.0.1:8892/tools/piper/search-marker \
  -H "Content-Type: application/json" \
  -d '{"execute":true,"arm":"front","direction":"auto","max_steps":100}'
```

Use the same debug form for saved-pose commands:

```bash
curl -sS -i -X POST http://127.0.0.1:8892/tools/piper/go-found-marker \
  -H "Content-Type: application/json" \
  -d '{"execute":true,"arm":"front","duration_s":6.0}'

curl -sS -i -X POST http://127.0.0.1:8892/tools/piper/go-nav-pose \
  -H "Content-Type: application/json" \
  -d '{"execute":true,"arm":"front","duration_s":6.0}'
```

For a safe no-motion validation of saved poses:

```bash
curl -sS -i -X POST http://127.0.0.1:8892/tools/piper/go-found-marker \
  -H "Content-Type: application/json" \
  -d '{"execute":false,"arm":"front","duration_s":6.0}'

curl -sS -i -X POST http://127.0.0.1:8892/tools/piper/go-nav-pose \
  -H "Content-Type: application/json" \
  -d '{"execute":false,"arm":"front","duration_s":6.0}'
```

Interpret common API status codes:

```text
409 another PiPER marker task is active
  Wait for the previous search/touch/home command to finish, or restart Terminal 11.

422 Unprocessable Entity
  The API reached the ROS action/service, but the requested command failed.
  Use curl -sS -i without -f to read the JSON body. Common causes are:
  saved pose unavailable, trajectory rejected, or MoveIt execution failed.
  If execute:false returns 200 but execute:true returns 422, the command format
  is valid and the failure is in motion execution. Check the control gate,
  /front_piper/feedback/joint_states, and the trajectory bridge.

502 Bad Gateway
  The API called ROS, but the ROS layer raised an exception. Check Terminal 9,
  Terminal 10, and Terminal 11.

503 Service Unavailable
  A required input is stale/missing, for example marker pose, point cloud,
  joint state, MoveIt action, /search_marker, or /run_marker_task.

504 Gateway Timeout
  The API waited for a ROS service/action and timed out. Check whether
  /search_marker or /run_marker_task is still alive and whether Terminal 9 or
  Terminal 10 printed a MoveIt/robot_description error.
```

### ArUco/search validation

Check the full marker path before physical search:

```bash
curl -sS http://127.0.0.1:8892/health; echo
ros2 node list | grep -E '^/aruco_single$|^/search_marker_node$|^/piper_touch_marker_api$'
ros2 topic info -v /aruco_single/pose
ros2 topic hz /front_camera/color/image_raw
ros2 topic hz /front_camera/color/camera_info
ros2 topic hz /front_camera/depth/color/points
```

`/aruco_single/pose --once` only prints when marker 6 is visible. If marker 6 is
not in view, this command will wait forever until you press `Ctrl-C`:

```bash
ros2 topic echo /aruco_single/pose --once
```

For search, `marker_visible:false` is acceptable because the search node can move
through search poses. For approach/touch, marker pose must be fresh.

If there are two `/aruco_single` nodes, one old marker stack is still alive.
Kill only the marker/search/API stack, then restart Terminal 8, Terminal 9,
Terminal 10, and Terminal 11. This does not kill tmux or Terminal 1 hardware:

```bash
PIDS="$(ps -eo pid=,cmd= | awk '/touch_marker_full_stack|aruco_ros\\/single|piper_touch_marker_api.py|search_marker_node|wall_approach_node/ && !/awk/ {print $1}')"
[ -n "$PIDS" ] && kill -TERM $PIDS || true
sleep 2
PIDS="$(ps -eo pid=,cmd= | awk '/touch_marker_full_stack|aruco_ros\\/single|piper_touch_marker_api.py|search_marker_node|wall_approach_node/ && !/awk/ {print $1}')"
[ -n "$PIDS" ] && kill -KILL $PIDS || true
ros2 daemon stop
ros2 daemon start
```

Then restart Terminal 8 through Terminal 11 with their separate commands above.

If you only want to stop the HTTP API server on port `8892` and leave ArUco,
search, wall approach, MoveIt, cameras, and hardware running, use:

```bash
API_PIDS="$(ss -ltnp 2>/dev/null | awk '/:8892 / {print $NF}' | sed -n 's/.*pid=\([0-9][0-9]*\).*/\1/p' | sort -u)"
if [ -z "$API_PIDS" ]; then
  API_PIDS="$(ps -eo pid=,cmd= | awk '/piper_touch_marker_api.py/ && !/awk/ {print $1}')"
fi
[ -n "$API_PIDS" ] && kill -TERM $API_PIDS || true
sleep 2
[ -n "$API_PIDS" ] && kill -KILL $API_PIDS 2>/dev/null || true
```

Confirm port `8892` is closed:

```bash
curl -sS http://127.0.0.1:8892/health || echo "8892 stopped"
```

If Terminal 9 or Terminal 10 prints this error:

```text
Could not find parameter robot_description
Could not find parameter robot_description_semantic
Could not parse the SRDF XML File. Error=XML_ERROR_EMPTY_DOCUMENT
Unable to construct robot model
```

first verify that Terminal 4 is running and publishing a non-empty SRDF:

```bash
ros2 node list | grep front_piper_description_bridge
ros2 topic info /front_piper/robot_description_semantic -v
ros2 topic echo --once /front_piper/robot_description_semantic std_msgs/msg/String
```

Expected:

```text
Publisher count: 1
data: "<?xml version=\"1.0\"?>..."
```

If that topic is missing, restart Terminal 4. If the commands still point at
`/front_piper/integrated_robot_description_semantic`, rebuild and restart
Terminal 8 through Terminal 11:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src/bunker_slam_bringup src/piper_x_aruco_wall_approach
source /ros2_ws/install/setup.bash
```

If `/health` says `fresh joint state unavailable`:

```bash
ros2 topic info /front_piper/feedback/joint_states
ros2 topic hz /front_piper/feedback/joint_states
ip -details -statistics link show FRONT_PIPER_CAN
timeout 3 candump -L FRONT_PIPER_CAN
```

Expected:

```text
/front_piper/feedback/joint_states has Publisher count: 1
FRONT_PIPER_CAN is UP at 1000000 bit/s
candump FRONT_PIPER_CAN shows PiPER frames
```

If `FRONT_PIPER_CAN` is DOWN, bring it back up:

```bash
ip link set FRONT_PIPER_CAN down 2>/dev/null || true
ip link set FRONT_PIPER_CAN type can bitrate 1000000 restart-ms 100
ip link set FRONT_PIPER_CAN up
```

If `FRONT_PIPER_CAN` is UP but `candump -L FRONT_PIPER_CAN` shows no frames, the front PiPER hardware,
power, firmware, or CAN cable is not publishing on FRONT_PIPER_CAN. The API cannot search
or move until this is fixed.

If `/health` says `point_cloud_available:false`:

```bash
ros2 topic info /front_camera/depth/color/points
ros2 topic hz /front_camera/depth/color/points
tmux capture-pane -t =0:t1_hardware -p -S -200 | grep -Ei 'realsense|front_camera|stale|disconnect|busy|error|failed'
```

Expected:

```text
/front_camera/depth/color/points has Publisher count: 1
```

If Publisher count is 0, Terminal 1 RealSense is not producing the point cloud.
Restart Terminal 1 or fix the camera ownership/USB issue first.

```bash
ros2 node list | grep '^/aruco_single$'
```

then Terminal 8 was started twice. Restart only the ArUco pane or use the
marker/search/API cleanup block before starting again. Exactly one
`/aruco_single` should publish `/aruco_single/pose`.

MoveIt now uses the real Terminal 1 front PiPER TF branch:

```text
base_link -> ... -> front_piper_base_link -> front_piper_link1 -> ... -> front_piper_flange_link
```

Check it:

```bash
ros2 run tf2_ros tf2_echo base_link front_piper_flange_link
ros2 run tf2_ros tf2_echo base_link aruco_marker_frame
```

There should be no unprefixed standalone AgileX arm TF:

```bash
ros2 run tf2_ros tf2_echo base_link link6
```

Expected: `link6` does not exist. If it exists, an old
`front_piper_moveit_tf_publisher.py` process is still running and must be
stopped.

Optionally close the front PiPER control gate after manipulation if you want to
block further front-arm control commands:

```bash
ros2 service call /front_piper/control_enable std_srvs/srv/SetBool "{data: false}"
```

Then navigate back to home:

```bash
ros2 topic pub --once /landmark_navigator/go_marker std_msgs/msg/String "{data: home}"
```

## Mode 9: Manual Illiyas API calls

Dry calls that should not move the arm:

```bash
curl -fsS -X POST http://127.0.0.1:8892/tools/piper/search-marker \
  -H "Content-Type: application/json" \
  -d '{"execute":false}'; echo

curl -fsS -X POST http://127.0.0.1:8892/tools/piper/touch-marker \
  -H "Content-Type: application/json" \
  -d '{"execute":false}'; echo
```

Execution calls can move the front PiPER if the front control gate is open:

```bash
curl -fsS -X POST http://127.0.0.1:8892/tools/piper/search-marker \
  -H "Content-Type: application/json" \
  -d '{"execute":true}'; echo
```

Keep the robot workspace clear before using `execute:true`.

## Mode 10: RTAB-Map front-camera behavior

The integrated startup uses:

```text
mapping_camera:=front
```

That means:

- one RGB-D sync node consumes the front camera
- RTAB-Map consumes the front RGB-D stream
- the rear camera is disabled to reduce USB/CPU load and avoid stale streams
- the same RTAB-Map database is used

The RTAB-Map config keeps:

```text
Odom/ResetCountdown=0
```

This prevents a temporary bad or blank camera frame from resetting the RTAB-Map
session.

Use `mapping_camera:=both` only after the front-only startup is stable and the
USB/camera rate problem has been solved.

## Mode 11: TF compatibility checks

Terminal 1 does not add generic camera alias TF publishers. Illiyas uses
Trystan's real front camera frame directly:

```text
front_camera_color_optical_frame
```

Check the main transforms:

```bash
ros2 run tf2_ros tf2_echo base_link front_camera_color_optical_frame
ros2 run tf2_ros tf2_echo base_link rear_camera_color_optical_frame
ros2 run tf2_ros tf2_echo base_link front_piper_base_link
ros2 run tf2_ros tf2_echo base_link rear_piper_base_link
ros2 run tf2_ros tf2_echo base_link link1
ros2 run tf2_ros tf2_echo base_link tcp_link
ros2 run tf2_ros tf2_echo base_link gripper_base
```

If a TF is missing, inspect the hardware and description panes first:

```bash
tmux capture-pane -t =0:t1_hardware -p -S -200
tmux capture-pane -t =0:t4_srdf_description -p -S -200
```

## Mode 12: Watchdog timeout debugging

Current default Nav2 routing bypasses the velocity smoother and safety mux.
With default launch arguments, `/nav2_cmd_vel_safety_mux`,
`/nav2/cmd_vel_raw`, `/cmd_vel_autonomy`, and `/cmd_vel_mux/reason` should not
be part of the active Nav2 command path. Nav2 publishes directly to:

```text
dry-run: /cmd_vel_debug
drive:   /cmd_vel
```

Use the watchdog checks below only when Nav2 was launched with:

```bash
use_cmd_vel_mux:=true
```

The integrated startup includes:

```text
t13_watchdogs
```

That terminal runs:

```bash
ros2 run bunker_slam_bringup watchdog_sources_monitor.sh
```

Use it when you see a timeout message and need to know which node is causing it.

For a legacy mux timeout:

```text
[nav2_cmd_vel_safety_mux]: Cmd vel mux holding zero due to watchdog_timeout
```

The source is:

```text
node:  /nav2_cmd_vel_safety_mux
topic: /cmd_vel_mux/reason
value: watchdog_timeout
```

Meaning:

```text
No fresh /cmd_vel_autonomy command reached the mux within command_timeout_sec.
The mux publishes zero velocity on /cmd_vel as a safety behavior.
```

This is normal when Nav2 is idle, paused, not actively controlling, or still
waiting for a valid goal/path. It is a problem only when you expected the Bunker
to be moving.

Manual checks:

```bash
ros2 topic echo /cmd_vel_mux/reason --field data
ros2 topic echo /safety_stop --field data
ros2 topic echo /safety_stop_reason --field data
ros2 topic echo /depth_route_status --field data
ros2 topic echo /route_status --field data

ros2 topic info -v /cmd_vel_mux/reason
ros2 topic info -v /cmd_vel_autonomy
ros2 topic info -v /nav2/cmd_vel_raw
ros2 topic info -v /cmd_vel
```

Expected ownership when `use_cmd_vel_mux:=true`:

```text
/cmd_vel_mux/reason publisher:  /nav2_cmd_vel_safety_mux
/safety_stop publisher:        /sensor_fusion_node
/depth_route_status publisher: /depth_route_monitor_node
/nav2/cmd_vel_raw publisher:   /controller_server
/cmd_vel_autonomy publisher:   /velocity_smoother
/cmd_vel publisher:            /nav2_cmd_vel_safety_mux
```

If `/cmd_vel_mux/reason` says:

```text
watchdog_timeout
```

then debug why `/cmd_vel_autonomy` is not fresh. Common causes:

- no active Nav2 goal;
- Nav2 controller is inactive or waiting;
- `mode:=dry_run` is running and you expected physical drive;
- planner/controller cannot produce a valid path;
- safety or route inputs are stale and Nav2 is being held.

If `/safety_stop` is `true`, debug `/safety_stop_reason`,
`/depth_route_status`, and `/route_status` before blaming Nav2.

Expected ownership with the current default direct route:

```text
/cmd_vel publisher while following a goal: /controller_server
/cmd_vel subscriber:                       /bunker
/safety_stop publisher:                    /sensor_fusion_node
/nav2/cmd_vel_raw:                         not used by Nav2
/cmd_vel_autonomy:                         not used by Nav2
/cmd_vel_mux/reason:                       not used by Nav2
```

If `/cmd_vel` is empty in the direct route, check for an active goal and a
valid plan first:

```bash
ros2 topic hz /plan
ros2 topic hz /local_plan
ros2 topic hz /cmd_vel
ros2 topic echo --once /navigate_to_pose/_action/status
```

## Mode 13: Logs and debugging

Capture tmux logs:

```bash
tmux capture-pane -t =0:t1_hardware -p -S -200
tmux capture-pane -t =0:t2_mapping -p -S -200
tmux capture-pane -t =0:t5_moveit -p -S -200
tmux capture-pane -t =0:t6_trajectory_bridge -p -S -200
tmux capture-pane -t =0:t7_moveit_rviz -p -S -200
tmux capture-pane -t =0:t8_aruco -p -S -200
tmux capture-pane -t =0:t9_marker_search -p -S -200
tmux capture-pane -t =0:t10_wall_approach -p -S -200
tmux capture-pane -t =0:t11_api_8892 -p -S -200
```

Useful filters:

```bash
tmux capture-pane -t =0:t1_hardware -p -S -400 | grep -Ei 'can|firmware|front_piper|rear_piper|realsense|camera|error|failed|disconnect|busy'
tmux capture-pane -t =0:t2_mapping -p -S -400 | grep -Ei 'rtabmap|rgbd|tf|odom|error|failed|reset'
tmux capture-pane -t =0:t5_moveit -p -S -400 | grep -Ei 'move_group|semantic|robot_description|joint|trajectory|error|failed'
tmux capture-pane -t =0:t6_trajectory_bridge -p -S -400 | grep -Ei 'trajectory|joint|feedback|goal|error|failed'
tmux capture-pane -t =0:t7_moveit_rviz -p -S -400 | grep -Ei 'rviz|motionplanning|semantic|robot_description|moveit|error|failed'
tmux capture-pane -t =0:t8_aruco -p -S -400 | grep -Ei 'aruco|semantic|marker|image|camera|error|failed'
tmux capture-pane -t =0:t9_marker_search -p -S -400 | grep -Ei 'search|marker|moveit|robot_description|joint|error|failed'
tmux capture-pane -t =0:t10_wall_approach -p -S -400 | grep -Ei 'wall|approach|touch|marker|point|moveit|error|failed'
tmux capture-pane -t =0:t11_api_8892 -p -S -400 | grep -Ei 'health|api|8892|search|marker|home|error|failed'
```

Common failures:

```text
RTNETLINK answers: No such device
```

The CAN interface name does not exist inside Docker. Recheck `/sys/class/net`
and the USB-CAN adapter mapping before launching.

```text
wait for device timeout of 10 secs expired
The device has been disconnected
```

Docker cannot see the RealSense device or another process owns it. Check
`rs-enumerate-devices -s`, `/dev/video*`, and kill stale RealSense processes.

```text
Device or resource busy
```

Another RealSense process still owns the camera. Run Mode 1 cleanup, then start
again.

```text
process has died ... cmd 'ros2 run bunker_autonomy realsense_usb_recover'
KeyboardInterrupt
exit code -2
```

This is the optional RealSense USB reset helper being interrupted. The current
front-only Nav-Man startup passes:

```text
reset_front_camera_usb:=false
```

so the helper is skipped during normal startup. If the D435i is physically
wedged and you intentionally want a USB reset, stop Terminal 1 first, then run
the helper manually and restart Terminal 1:

```bash
ros2 run bunker_autonomy realsense_usb_recover
```

If the Terminal 1 watchdog says:

```text
No RealSense USB device is currently enumerated
```

do not reboot the Jetson and do not restart Docker as the first fix. That means
Linux currently has no D435i USB device for the watchdog to reset. Recover only
the camera USB path: check the wrist camera cable/hub power, replug the D435i if
needed, confirm it with `rs-enumerate-devices -s`, then restart only the
camera/Terminal 1 launch if the RealSense node does not respawn. Nav-Man should
continue to use Trystan's Terminal 1 camera owner; do not start a second
RealSense driver from the Illiyas/Nav-Man side.

```text
No plugins found ... image_transport
```

This is an image transport dependency/environment problem. Re-source
`/opt/ros/humble/setup.bash` and `/ros2_ws/install/setup.bash`; if it persists,
the Docker image dependencies need to be rebuilt.

```text
Could not convert rgb/depth msgs
TF ... does not exist
```

RTAB-Map is receiving camera messages but cannot transform them into the robot
frame. Check Terminal 1 TF and camera frame names.

If the error mentions this old generic frame:

```text
camera_color_optical_frame
```

then the RealSense nodes were started before the frame-ID fix was loaded. Stop
Terminal 1 and Terminal 2, rebuild/refresh if needed, then restart them. The
camera/RGB-D headers must use:

```text
front_camera_color_optical_frame
rear_camera_color_optical_frame
```

## Mode 13: Stop everything

Stop all ROS/Nav-Man processes while keeping tmux session `0` open:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=173
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 run bunker_slam_bringup pkill_bunker_nav2_stack.sh
ros2 daemon stop
ros2 daemon start
ros2 node list
```

If the installed script is unavailable, use the plain no-tmux fallback:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=173
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

pkill -TERM -f 'terminal1_sensors.launch.py|terminal2_robot.launch.py|terminal3_mapping.launch.py|nav2_bringup.launch.py|touch_marker_full_stack.launch.py|front_piper_description_bridge.py|front_piper_move_group_only.py|front_piper_moveit_tf_publisher.py|front_piper_moveit_rviz.py|front_piper_trajectory_bridge.py|start_nav_man_workflow.py|start_iliyas_abot_in_trystan.py' || true
pkill -TERM -f 'realsense2_camera_node|rgbd_sync|rgbdx_sync|rtabmap|move_group|rviz2|agx_arm_ctrl_single|bunker_base_node|robot_state_publisher|joint_state_prefixer|joint_state_publisher|ekf_node|yesense_node_publisher|piper_navigation_pose|piper_x_joint_preset|search_marker_node|wall_approach_node|piper_touch_marker_api|aruco_ros|aruco_single|static_transform_publisher|depth_route_monitor_node|sensor_fusion_node|safety_monitor_node|cmd_vel_mux_node|nav2_cmd_vel_safety_mux|controller_server|planner_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|smoother_server|lifecycle_manager' || true

sleep 2

pkill -KILL -f 'terminal1_sensors.launch.py|terminal2_robot.launch.py|terminal3_mapping.launch.py|nav2_bringup.launch.py|touch_marker_full_stack.launch.py|front_piper_description_bridge.py|front_piper_move_group_only.py|front_piper_moveit_tf_publisher.py|front_piper_moveit_rviz.py|front_piper_trajectory_bridge.py|start_nav_man_workflow.py|start_iliyas_abot_in_trystan.py' || true
pkill -KILL -f 'realsense2_camera_node|rgbd_sync|rgbdx_sync|rtabmap|move_group|rviz2|agx_arm_ctrl_single|bunker_base_node|robot_state_publisher|joint_state_prefixer|joint_state_publisher|ekf_node|yesense_node_publisher|piper_navigation_pose|piper_x_joint_preset|search_marker_node|wall_approach_node|piper_touch_marker_api|aruco_ros|aruco_single|static_transform_publisher|depth_route_monitor_node|sensor_fusion_node|safety_monitor_node|cmd_vel_mux_node|nav2_cmd_vel_safety_mux|controller_server|planner_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|smoother_server|lifecycle_manager' || true

ros2 daemon stop
ros2 daemon start
ros2 node list
```

## Quick path

For a normal run, use only these blocks in order:

```text
Mode 0  enter Docker
Mode 1  clean old processes
Mode 3  verify CAN/cameras
Mode 4  start full tmux
Mode 7  validate topics
Mode 8  home/door navigation and manipulation calls
Mode 13 stop
```
