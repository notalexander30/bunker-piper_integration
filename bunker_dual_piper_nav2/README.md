# Bunker Mini + dual PiPER + dual D435i + RTAB-Map/Nav2

> **Current startup:** follow `/ros2_ws/RTABMAP_NAV2_DOCKER_TMUX_RUNBOOK.md`.
> It contains the maintained camera topics, RealSense profile, footprint,
> mapping, localization and Nav2 drive procedure.

This ROS 2 Humble package provides one robot description and one launch entry
point for the modified Bunker Mini. It is intended for the real Jetson system,
RViz and Nav2; it does not contain a Gazebo simulation.

## Included model

- Original Bunker Mini base visual and conservative box collision.
- User upper-frame STL, converted from millimetres with ROS axes aligned.
- Two 260 x 100 x 17 mm PiPER mounting plates at the protected frame ends.
- Two official PiPER arms, each with six revolute joints and two gripper-finger
  joints. PiPER's "7 DoF" description means six arm joints plus gripper motion.
- A D435i tree fixed to each gripper base. The rear camera can stay disabled
  until it is physically installed.
- RViz overlays the full visual URDF on the 2D occupancy map in a top-down
  view. Nav2 costmap windows remain available as optional, disabled displays.
- `rviz/bunker_description_2d.rviz` is the dedicated TF-accurate 2D view. It
  projects the same live 3D URDF from above and draws the Nav2 collision
  footprint as a separate gold outline; it does not duplicate the robot TFs.

The measured plate top is 0.631 m above the ground. The protected frame length
is 0.812 m and the bare frame length is 0.780 m. Both mount and camera origins
remain xacro arguments so they can be corrected without rebuilding meshes.

## Coordinate convention and current assumptions

- `base_link`: Bunker body frame, +X forward, +Y left, +Z up.
- `base_footprint`: ground projection used by Nav2.
- PiPER base XYZ: front mounts at X = +0.356 m and rear mounts at X = -0.356 m
  on the upper frame.
- PiPER base yaw: front 180 degrees, rear -90 degrees so the rear arm/camera
  keeps facing back.
- Default left-facing parked pose: front joint 1 = -1.6 rad and rear joint 1 =
  +1.6 rad; joints 2-6 and both gripper joints default to zero. Live arm
  feedback replaces these display defaults.
- Camera screw-frame estimate relative to each `gripper_base`:
  XYZ = `-0.030 0.000 0.070` m, RPY = `0 -1.5708 0` rad.

The arm and camera orientations are **engineering estimates from photographs and
tape measurements**, not a surveyed calibration. Validate them in RViz before
using wrist point clouds for close obstacle avoidance.

## What this package does not invent

The live diagnostic was collected while the robot drivers were off, so the
following remain site configuration, not confirmed facts:

- physical CAN assignment for both PiPER arms and the Bunker;
- whether the selected `front_piper_can` and `rear_piper_can` match the
  physical front/rear arm labels. Current operator-provided PiPER CAN
  assignment is front `FRONT_PIPER_CAN` and rear `REAR_PIPER_CAN`;
- the exact existing RTAB-Map launch filename;
- the Bunker driver's live odometry/frame names;
- both D435i serial numbers;
- PiPER firmware generation.

The supplied defaults launch two namespaced AGX PiPER drivers. Their feedback
topics feed the combined URDF state path:

```text
/front_piper/feedback/joint_states  \
                                      -> /dual_piper_joint_state_prefixer -> /joint_states
/rear_piper/feedback/joint_states   /

/robot_state_publisher:
  robot_description parameter/topic -> combined dual-PiPER URDF
  /joint_states subscription        -> front_piper_* and rear_piper_* joint values
  /tf and /tf_static publishers     -> live link transforms
```

There is no service between URDF publishing and joint-state feedback. Confirm
the front/rear CAN mapping by moving one arm slightly and checking that the
matching prefixed joints change in `/joint_states`.

## Two-PiPER URDF/RViz bringup

Use the dedicated bringup when you want both front and rear Piper arms connected
to the combined URDF and visible in RViz:

```bash
ros2 launch bunker_dual_piper_nav2 dual_piper_bringup.launch.py \
  rear_piper_can:=REAR_PIPER_CAN \
  front_piper_can:=FRONT_PIPER_CAN \
  run_initial_pose:=false \
  allow_piper_motion:=false \
  piper_control_enabled:=true \
  start_rviz:=true
```

This starts:

```text
/front_piper/agx_arm_ctrl_single_node -> /front_piper/feedback/joint_states
/rear_piper/agx_arm_ctrl_single_node  -> /rear_piper/feedback/joint_states
/dual_piper_joint_state_prefixer      -> /joint_states
/robot_state_publisher                -> /robot_description, /tf, /tf_static
/dual_piper_rviz2                     -> RViz model view
```

The prefixer publishes the parked initial pose for missing/stale feedback, so
RViz can show the full front/rear URDF before either live arm stream is healthy.

Physical arm motion is not automatic. To command both arms to the documented
initial pose, opt in explicitly:

```bash
ros2 launch bunker_dual_piper_nav2 dual_piper_bringup.launch.py \
  rear_piper_can:=REAR_PIPER_CAN \
  front_piper_can:=FRONT_PIPER_CAN \
  piper_control_enabled:=true \
  run_initial_pose:=true \
  allow_piper_motion:=true \
  start_rviz:=true
```

Current robot mapping is front PiPER on `FRONT_PIPER_CAN` and rear PiPER on `REAR_PIPER_CAN`. If
the physical adapters are moved, update `front_piper_can` and `rear_piper_can`
from the serial-verified SocketCAN names.

## Install on the Jetson

Copy the entire `bunker_dual_piper_nav2` directory to the Jetson, then run:

```bash
cd /path/to/bunker_dual_piper_nav2
bash scripts/install_into_robot_ws.sh /home/dase-orin/ros2_ws
source /home/dase-orin/ros2_ws/install/setup.bash
cp /home/dase-orin/ros2_ws/src/bunker_dual_piper_nav2/scripts/robot_system.env.example \
   /home/dase-orin/ros2_ws/src/bunker_dual_piper_nav2/scripts/robot_system.env
```

Edit `robot_system.env`. Do not enable hardware startup until the CAN names are
confirmed. When two D435i cameras are connected, set both serial numbers so the
front/rear assignment cannot change across boots.

```text
launch_front_camera:=true
launch_rear_camera:=true
front_camera_serial:=FRONT_CAMERA_SERIAL_HERE
rear_camera_serial:=REAR_CAMERA_SERIAL_HERE
```

## First test: URDF and TF only

This publishes safe zero joint values, starts no hardware and starts no camera:

```bash
ros2 launch bunker_dual_piper_nav2 system_bringup.launch.py \
  start_hardware_drivers:=false start_cameras:=false start_nav2:=false \
  publish_default_joint_states:=true prefix_joint_states:=false start_rviz:=true
```

In RViz, check:

1. Bunker front is +X.
2. Both arm bases are centred on their plates.
3. Cable side and zero-pose direction match the physical robot.
4. Camera body and optical axes match each wrist camera.

## Full system, motion blocked

Use the integrated script first in debug mode:

```bash
cd /home/dase-orin/ros2_ws/src/bunker_dual_piper_nav2
bash scripts/run_complete_system.sh --debug
```

The script can start the Bunker driver, both PiPER drivers on `FRONT_PIPER_CAN`/`REAR_PIPER_CAN`
(with automatic arm enable disabled), the selected D435i cameras, your existing
RTAB-Map launch, your existing YOLO launch, robot state publisher, Nav2 and
RViz. Nav2 velocity is clamped and routed to `/cmd_vel_debug`, so an RViz goal
does not move the base.

Your RTAB-Map system must provide this transform/topic contract:

```text
map -> odom -> base_link
/map       nav_msgs/OccupancyGrid
/odom      nav_msgs/Odometry
```

This integrated camera launch replaces the standalone RealSense launch and
uses these front-camera inputs:

```text
/front_camera/color/image_raw
/front_camera/color/camera_info
/front_camera/aligned_depth_to_color/image_raw
/front_camera/depth/color/points
/front_camera/imu
```

The rear equivalents begin with `/rear_camera/`. Update the input remappings in
your existing RTAB-Map and YOLO launches if they currently use `/camera/camera/*`
or `/camera/*`. Do not run the old standalone RealSense node at the same time;
two processes cannot reliably own one USB camera.

Nav2 consumes both D435i point clouds in
`config/nav2_bunker_params.yaml`:

```text
/front_camera/depth/color/points -> front_cloud
/rear_camera/depth/color/points  -> rear_cloud
```

The rear point cloud is part of both local and global obstacle costmaps. That is
what makes reverse recovery meaningful: the planner and behavior server can
check the same robot footprint against rear-camera obstacles before backing up.

Both PiPER feedback streams are consumed by the joint-state prefixer:

```text
/front_piper/feedback/joint_states -> front_piper_* names on /joint_states
/rear_piper/feedback/joint_states  -> rear_piper_* names on /joint_states
```

`robot_state_publisher` uses `/joint_states`, so RViz shows both live arm poses
when both PiPER drivers are running.

If the D435i reports `xioctl(VIDIOC_S_FMT) failed, errno=16` or
`Device or resource busy`, stop old camera launches and restart the maintained
Terminal 1 launch. `bunker_slam_bringup terminal1_sensors.launch.py` now defaults
`reset_front_camera_usb:=true`, which resets the connected D435i before starting
RealSense. If `image_transport/..._pub` or `No plugins found!` appears during a
launch abort, check the earlier error first; those lines are commonly shutdown
noise after the RealSense node context has already been invalidated.

Set `RTABMAP_LAUNCH_FILE` to a launch that does **not** start another RealSense
node. Set `YOLO_LAUNCH_FILE` similarly, then enable each component in
`robot_system.env`.

Only one node may publish each TF edge. This package publishes the fixed robot,
arm and camera geometry. RealSense TF publishing is disabled. RTAB-Map/local
odometry must own `map -> odom`, and the Bunker odometry source must own
`odom -> base_link`.

## RViz goal control

Once Nav2 is active, choose **2D Goal Pose** in RViz and click/drag on the map.
Nav2 plans on `/map`, tracks the robot through TF, checks the local/global
costmaps and generates velocity commands. In debug mode inspect them with:

```bash
ros2 topic echo /cmd_vel_debug
ros2 run tf2_ros tf2_echo map base_link
```

## Updating URDF RPY

The editable source is:

```text
urdf/bunker_dual_piper_d435i.urdf.xacro
```

The generated file is only a snapshot:

```text
urdf/bunker_dual_piper_d435i.generated.urdf
```

When changing arm or camera RPY values, update the matching launch defaults if
they override xacro args:

```text
launch/description.launch.py
launch/system_bringup.launch.py
launch/urdf_camera_preview.launch.py
launch/bunker_piper_3d_showcase.launch.py
scripts/run_complete_system.sh
scripts/robot_system.env.example
```

Then regenerate the snapshot from the xacro:

```bash
cd /home/dase-orin/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run xacro xacro \
  src/bunker_dual_piper_nav2/urdf/bunker_dual_piper_d435i.urdf.xacro \
  -o src/bunker_dual_piper_nav2/urdf/bunker_dual_piper_d435i.generated.urdf
```

Build and check the transforms:

```bash
colcon build --symlink-install --packages-select bunker_dual_piper_nav2 bunker_slam_bringup
source install/setup.bash
ros2 launch bunker_dual_piper_nav2 urdf_camera_preview.launch.py
ros2 run tf2_ros tf2_echo base_link front_camera_color_optical_frame
ros2 run tf2_ros tf2_echo base_link rear_camera_color_optical_frame
```

The live dual-camera topic contract is:

```text
/front_camera/color/image_raw
/front_camera/color/camera_info
/front_camera/aligned_depth_to_color/image_raw
/front_camera/depth/color/points
/front_camera/imu
/rear_camera/color/image_raw
/rear_camera/color/camera_info
/rear_camera/aligned_depth_to_color/image_raw
/rear_camera/depth/color/points
/rear_camera/imu
/front_rgbd_image
/rear_rgbd_image
```

This package overrides Nav2's default recovery tree with:

```text
behavior_trees/navigate_to_pose_back_up_recovery.xml
```

That tree clears costmaps and uses `BackUp backup_dist="0.35"
backup_speed="0.15"` instead of the default spin recovery. The robot should use
the rear camera as the active obstacle source while moving backward, instead of
rotating 360 degrees just to look around.

For landmark navigation, `bunker_slam_bringup nav2_bringup.launch.py` also
selects the rear RealSense during a `door -> home` reverse trip:

```text
/landmark_navigator/navigation_direction  reverse
/landmark_navigator/active_camera         rear_camera
/rear_camera/aligned_depth_to_color/image_raw
```

The rear camera is the D435i with serial `REAR_CAMERA_SERIAL`; the front camera is
`FRONT_CAMERA_SERIAL`.

The current Nav2 speed envelope is symmetric for forward and reverse:

```text
DWB planner:        -0.225 to +0.225 m/s, +/-0.375 rad/s
velocity smoother: -0.225 to +0.225 m/s, +/-0.375 rad/s
safe_cmd_vel_gate: clamps absolute linear X to 0.225 m/s and angular Z to 0.375 rad/s
```

Acceleration/deceleration are set faster than the previous commissioning values:

```text
linear accel/decel: 0.40 / 0.50 m/s^2
angular accel/decel: 0.60 / 0.75 rad/s^2
```

## Nav2 Planner And Costmap Tuning

The active Nav2 file is:

```text
config/nav2_bunker_params.yaml
```

Important knobs:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      width: 3
      height: 3
      footprint: "[[-0.75, -0.30], [0.75, -0.30], [0.75, 0.30], [-0.75, 0.30]]"
      inflation_layer:
        inflation_radius: 0.05

global_costmap:
  global_costmap:
    ros__parameters:
      footprint: "[[-0.75, -0.30], [0.75, -0.30], [0.75, 0.30], [-0.75, 0.30]]"
      inflation_layer:
        inflation_radius: 0.05

planner_server:
  ros__parameters:
    GridBased:
      plugin: nav2_navfn_planner/NavfnPlanner
      tolerance: 0.3
      use_astar: true
```

`footprint` is robot collision size in metres, centered on `base_link`. The
current footprint is 1.50 m long by 0.60 m wide.

`inflation_radius` is extra obstacle clearance added by the costmap. It is now
0.05 m.

`width` and `height` are the rolling local costmap window dimensions in metres.
Smaller windows update less map area but give the controller less local context.

`use_astar: true` makes the installed NavFn planner use A* instead of the
default Dijkstra-style expansion. This usually gives more deliberate paths
toward the goal while still using the same installed planner plugin. The Smac
planner is not installed in the current Humble environment, so this package
does not reference it.

## Safety Stop

There is no separate safety-stop YAML in `bunker_dual_piper_nav2`. The Nav2
motion stop layer here is the `safe_cmd_vel_gate` node configured in
`launch/nav2_rtabmap.launch.py`.

Algorithm:

```text
/cmd_vel_nav2_raw -> safe_cmd_vel_gate -> /cmd_vel_debug or /cmd_vel_autonomy
```

For every Nav2 command, the gate copies only `linear.x` and `angular.z`, clamps
them to the configured absolute limits, and publishes the filtered command. If
no command arrives for `timeout=0.50` seconds, it publishes a zero `Twist` once
and stays stopped until a fresh command arrives.

The broader `/safety_stop` fusion YAML lives outside this package at:

```text
/home/dase-orin/ros2_ws/src/bunker_autonomy/config/autonomy.yaml
```

That stack fuses LiDAR/depth stop decisions and feeds a separate cmd_vel mux.
Use it when launching the full autonomy safety pipeline; use
`safe_cmd_vel_gate` as the Nav2-local final clamp/watchdog.

## Hardware commissioning

Before allowing base motion:

1. Keep both PiPER arms folded inside the configured footprint and disable arm
   motion during navigation.
2. Put the Bunker on blocks for the first wheel-direction test.
3. Confirm positive linear X drives forward and positive angular Z turns left.
4. Confirm releasing commands produces a zero velocity within 0.5 seconds.
5. Test the physical emergency stop.
6. Start at 0.10 m/s, not the configured 0.30 m/s maximum.

After those tests, run:

```bash
bash scripts/run_complete_system.sh --drive
```

The script requires typing `DRIVE`. In drive mode the safety gate publishes to
`/cmd_vel_autonomy`; the hardware wrapper remaps only the Bunker driver's
`/cmd_vel` input to that isolated topic.

## PiPER firmware option

Firmware is not required to create the URDF. It changes only the official
joint-2/joint-3 frame offsets:

- firmware S-V1.6-3 or later: keep `FRONT_PIPER_LEGACY=0`;
- firmware older than S-V1.6-3: set the corresponding value to `1`.

Do not enable the arms merely to query firmware. Use the included read-only
diagnostic/query helpers when the correct CAN interface is known.

## Important Nav2 limitation

The costmap footprint covers the protected Bunker and fixed upper frame, not
arbitrary extended-arm poses. A moving or extended manipulator changes the true
collision envelope. For initial operation, enforce a folded navigation pose.
A later production system should add arm-state interlocks or dynamic footprint/
3D collision monitoring.

## Main files

- `urdf/bunker_dual_piper_d435i.urdf.xacro`: complete robot model.
- `launch/system_bringup.launch.py`: combined entry point.
- `launch/hardware_drivers.launch.py`: isolated Bunker/PiPER driver wrapper.
- `config/nav2_bunker_params.yaml`: conservative Nav2 settings.
- `scripts/run_complete_system.sh`: one-command runtime wrapper.
- `scripts/preflight_check.sh`: read-only readiness check.
- `scripts/robot_system.env.example`: site-specific values.
- `CODEX_IMPLEMENTATION_PROMPT.md`: prompt for Codex to inspect the live Jetson,
  finish TF/topic integration and add guarded navigation/exploration modes.
- `CODEX_UPDATED_URDF_PROMPT.md`: handoff prompt for the corrected left-facing
  parked arm pose and the later camera-extrinsic calibration.
