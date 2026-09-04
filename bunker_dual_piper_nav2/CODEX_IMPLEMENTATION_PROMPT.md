# Codex implementation prompt: Bunker Mini + dual PiPER + D435i + RTAB-Map + Nav2

You are working directly on my NVIDIA Jetson running Ubuntu and ROS 2 Humble.
Your task is to integrate the supplied `bunker_dual_piper_nav2` package into my
existing Bunker Mini system. The robot has two PiPER arms, wrist D435i cameras,
RTAB-Map and YOLO.

Work collaboratively until the integration is genuinely usable. Inspect the
live system and existing source files before changing anything. Discover values
from the computer whenever possible. If important information cannot be found,
ask me a concise question before making an unsafe or architecture-changing
assumption.

## Supplied package

The transfer archive is named:

```text
bunker_dual_piper_nav2_ws.zip
```

If the operator provides a SHA-256 value separately, compare it with the
transferred archive before extraction. Do not use a checksum embedded inside
the archive itself as proof of that same archive.

The archive contains:

```text
src/bunker_dual_piper_nav2
```

The target workspace is expected to be:

```text
/home/dase-orin/ros2_ws
```

Do not assume these paths are correct if the live system shows otherwise.

## Safety rules

1. Begin with read-only inspection. Do not publish velocity, arm, gripper,
   enable, reset, homing or pose commands during discovery.
2. Do not enable either PiPER automatically. Keep `auto_enable:=false`.
3. Keep Nav2 output isolated on `/cmd_vel_debug` during initial testing.
4. Do not connect Nav2 to the Bunker hardware until I explicitly approve the
   drive test after reviewing TF, odometry, costmaps and command signs.
5. Do not start two RealSense drivers for the same USB camera.
6. Preserve my existing RTAB-Map, YOLO and robot files. Back up or make focused
   changes; do not replace unrelated work.
7. Never invent CAN assignments, camera serial numbers, topic names, frame IDs,
   launch filenames or robot dimensions.
8. Both arms must be folded into a verified navigation pose before base motion.
   The current Nav2 footprint does not represent arbitrary extended-arm poses.

## Phase 1: inspect and report

Before implementation, inspect and summarize:

- ROS distribution, workspace and whether containers are involved;
- relevant packages and existing launch files for Bunker, PiPER, RealSense,
  RTAB-Map, YOLO, robot localization and Nav2;
- active nodes, topics, services and actions;
- publishers/subscribers for `/cmd_vel`, `/odom`, `/map`, `/tf`, `/tf_static`
  and `/joint_states`;
- actual Bunker odometry topic, message `header.frame_id` and `child_frame_id`;
- the current TF tree and which node owns each important transform;
- both PiPER CAN interfaces, namespaces and feedback JointState topics;
- whether PiPER `left` is physically the front arm or rear arm;
- D435i serial numbers, camera topics and message frame IDs;
- RTAB-Map mapping/localization launch files and parameters;
- YOLO input image topic and launch file;
- the Bunker base launch filename and its velocity subscription;
- whether an EKF or another node already publishes `odom -> base_link`;
- installed exploration packages compatible with ROS 2 Humble.

Use the included read-only diagnostic script if useful:

```bash
bash scripts/collect_robot_description_info.sh
bash scripts/preflight_check.sh
```

After inspection, give me a short table containing confirmed values, unresolved
values and proposed changes. Ask only for unresolved information that materially
affects the result. In particular, ask me if you cannot determine:

- front/rear PiPER CAN mapping;
- camera serial-to-arm mapping;
- which RTAB-Map and YOLO launches should become the integrated launches;
- the desired map storage directory;
- the safe folded-arm pose/interlock;
- exploration boundaries, prohibited areas and required stopping conditions;
- whether "controller" means RViz mouse goals, joystick/gamepad control, or both.

## Phase 2: install and validate the robot description

After the inspection and any necessary answers:

1. Verify the ZIP checksum if the archive is available.
2. Install `src/bunker_dual_piper_nav2` in the correct ROS 2 workspace.
3. Resolve dependencies and build with `colcon`.
4. Expand the xacro and run `check_urdf`.
5. Launch the description without hardware motion.
6. Confirm the model visually in RViz and generate a new TF graph.
7. Compare the model against live joint feedback and camera frame IDs.
8. Correct configurable mount/camera origins if evidence shows they are wrong.

The intended TF ownership is:

```text
RTAB-Map/localization:       map -> odom
Bunker odometry or EKF:      odom -> base_link
robot_state_publisher:       base_link -> upper frame -> plates -> PiPER links
PiPER joint feedback:        values used by robot_state_publisher
robot_state_publisher:       wrist/gripper -> D435i fixed/internal frames
RealSense driver:            sensor messages only; publish_tf=false
```

Detect and remove duplicate TF publishers. Do not hide a broken transform with
an arbitrary static transform.

The expected complete chain is:

```text
map -> odom -> base_link -> upper_frame_link
                         -> front PiPER -> front D435i optical frames
                         -> rear PiPER  -> rear D435i optical frames
```

## Phase 3: integrate cameras, RTAB-Map and YOLO

Replace the standalone RealSense startup with the integrated camera launch.
The intended topics are:

```text
/front_camera/color/image_raw
/front_camera/color/camera_info
/front_camera/aligned_depth_to_color/image_raw
/front_camera/depth/color/points
/front_camera/imu
```

Rear-camera topics use `/rear_camera/`.

Update RTAB-Map and YOLO remappings to use these topics. If the rear D435i is
not installed, leave its URDF model present but keep its driver and costmap
observation source disabled or harmless. Confirm that RTAB-Map receives valid
synchronized data and that YOLO receives the intended color image.

Do not make two components publish the same `map -> odom`, `odom -> base_link`
or camera transforms.

## Phase 4: create explicit operating modes

Create or update one operator script, preferably `scripts/robot_mode.sh`, with
clear help text and at least these modes:

```text
description       URDF/TF/RViz only, no hardware commands
mapping-manual    RTAB-Map incremental mapping with manual teleoperation
localization      Load an existing RTAB-Map database/map and localize only
navigate-debug    Localization + Nav2 + RViz goals, output /cmd_vel_debug
navigate-drive    Localization + Nav2 + RViz goals, isolated hardware output
explore-debug     Autonomous frontier goals but no base hardware connection
explore-drive     Autonomous mapping/exploration with guarded base output
save-map          Save the RTAB-Map database and 2D occupancy map
stop              Stop only processes started by this integration
status            Show mode, lifecycle state, important topics and TF health
```

The script should source ROS and the workspace, reject missing configuration,
avoid duplicate processes and stop cleanly. Put site-specific values in a
separate environment/configuration file, not hard-coded throughout scripts.

## Phase 5: Nav2 goal-controller mode

Implement Nav2 for an existing RTAB-Map map and localization. The minimum
runtime contract is:

```text
/map                     nav_msgs/msg/OccupancyGrid
/odom                    nav_msgs/msg/Odometry
map -> odom -> base_link valid continuously
sensor obstacles         valid PointCloud2 and/or LaserScan topics
```

The RViz **2D Goal Pose** tool must send a goal that Nav2 can plan and follow.
Validate global and local costmaps, footprint orientation, path planning,
controller output and recovery behavior in debug mode first.

Keep these protections:

- maximum linear velocity initially no more than 0.10 m/s for the first drive;
- maximum angular velocity initially no more than 0.30 rad/s;
- command watchdog publishing zero after at most 0.5 seconds;
- isolated `/cmd_vel_autonomy` input for the Bunker driver;
- no generic `/cmd_vel` connection that could bypass the safety gate;
- explicit confirmation before entering `navigate-drive`;
- emergency-stop instructions displayed before motion.

Verify positive X drives the physical Bunker forward and positive Z turns it
left while the robot is safely supported. Do not assume the signs.

If I request joystick/gamepad control as well as RViz goals, add it as a
separate manual mode with a dead-man button and velocity limits. Do not mix
uncoordinated teleop and Nav2 publishers; use a velocity multiplexer with clear
priority and lockout behavior.

## Phase 6: autonomous area-mapping mode

Nav2 does not automatically explore an unknown building by itself. Add a ROS 2
Humble-compatible frontier-exploration component only after checking what is
already installed. Prefer a maintained package compatible with the existing
Nav2/RTAB-Map versions. Explain the chosen component and any new dependency.

The exploration mode must:

1. Start RTAB-Map in incremental mapping mode.
2. Wait until sensor topics, odometry and TF are healthy.
3. Start Nav2 using the continuously updated occupancy grid.
4. Generate reachable frontier goals inside an operator-defined boundary.
5. Reject goals in prohibited or unsafe regions.
6. Handle unreachable frontiers without looping forever.
7. Pause or stop on TF loss, odometry loss, stale sensors or Nav2 failure.
8. Stop when no valid frontiers remain for a configurable period, the maximum
   mission time is reached, battery policy requires stopping, or I press stop.
9. Save the RTAB-Map database and export the final 2D map with timestamped names.
10. Print the saved paths and mission result.

Before enabling `explore-drive`, ask me for or confirm:

- physical exploration boundary and excluded areas;
- doors, stairs, glass, drop-offs or zones D435i may not detect reliably;
- maximum travel speed and mission duration;
- battery threshold and how battery status is obtained;
- whether the robot should return to its starting pose;
- the safe arm-folded condition and how it will be verified.

Do not claim that D435i point clouds alone provide reliable cliff/drop-off
protection. Report if additional lidar, cliff sensors or a safety observer is
needed for the actual environment.

## Required verification and handoff

Complete as many non-motion checks as possible automatically:

- build succeeds without hidden errors;
- xacro expansion and `check_urdf` pass;
- exactly one connected TF tree exists;
- every sensor message frame resolves to `base_link` and `map`;
- joint feedback animates the correct physical arm in RViz;
- RTAB-Map receives synchronized camera/odometry input;
- YOLO receives images;
- `/map` updates in mapping mode and remains stable in localization mode;
- Nav2 lifecycle nodes become active;
- global and local costmaps update;
- a debug goal produces a valid plan and only `/cmd_vel_debug` output;
- an exploration debug run generates reasonable frontier goals but cannot move
  the hardware.

At each motion-enabling boundary, stop and ask for my approval. When finished,
provide:

1. a list of changed files with absolute paths;
2. the exact commands for every operating mode;
3. confirmed topic and TF diagrams;
4. configuration values I may need to change later;
5. unresolved limitations and safety risks;
6. rollback instructions;
7. results of every validation performed.

Do not describe the system as hardware-ready unless the live motion tests were
actually completed and observed successfully.
