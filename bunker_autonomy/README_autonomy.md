# Bunker Autonomy: Find Trash Can Demo

## D435i-only arm-camera pipeline

The current camera-only profile uses one Intel RealSense D435i mounted at the
top of the arm:

```text
D435i RGB -> simple_vlm -> /vlm_result -> behavior
D435i aligned depth -> depth route/stop -> depth-only safety -> behavior/mux
```

No LiDAR node or D555 camera is started or required by this profile. Start the
complete pipeline in dry-run mode with:

```bash
cd ~/ros2_ws
colcon build --packages-select simple_vlm bunker_autonomy
source install/setup.bash
ros2 launch bunker_autonomy find_trash_can_d435i.launch.py mode:=dry_run
```

The older generic command now selects the same D435i-only pipeline and starts
the camera by default:

```bash
ros2 launch bunker_autonomy find_trash_can_realsense.launch.py mode:=dry_run
```

The camera publishes RGB on `/camera/camera/color/image_raw`. Navigation and
target ranging use `/camera/camera/aligned_depth_to_color/image_raw`, so the
VLM's normalized RGB bounding box indexes the corresponding depth pixels.

Camera and view parameters are separated so the mounting configuration can be
changed later:

- Driver/profile parameters: `config/realsense_d435i.yaml`
- Arm-view ROI, sector boundaries, distances, and speed limits:
  `config/autonomy_d435i.yaml`
- Optional mount TF launch arguments: `camera_x`, `camera_y`, `camera_z`,
  `camera_roll`, `camera_pitch`, and `camera_yaw`

For a fixed arm pose without a URDF-published camera transform, add
`publish_camera_static_tf:=true` and measured mount values. If the robot/arm
already publishes `base_link -> ... -> camera_link`, leave it false to avoid a
conflicting TF parent. The present navigation logic is image-sector based; the
TF does not correct an incorrectly aimed image.

Important limitations for this temporary sensor configuration:

- Keep the arm locked in the calibrated navigation pose. Moving the arm changes
  which base direction each image sector represents.
- Tune `roi_*_fraction`, `left_sector_end_fraction`, and
  `right_sector_start_fraction` from live aligned-depth images before drive mode.
- A single forward camera has no rear coverage, so reverse remains disabled.
- Camera depth does not provide the D555+LiDAR profile's independent 360-degree
  obstacle check or metric robot-width corridor fit. Use dry runs, low speed, a
  physical emergency stop, and do not treat this as production collision
  avoidance.

Verify the camera-only inputs and decisions before considering motion:

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
ros2 topic echo /depth_route_status
ros2 topic echo /route_status
ros2 topic echo /cmd_vel_debug
```

### X11 calibration workflow

From the Jetson desktop user account, the helper prepares Docker-safe X11
authorization and opens the live D435i calibration overlay:

```bash
cd ~/ros2_ws
./tools/trystan-bunker up
./tools/trystan-bunker build
./tools/trystan-bunker x11-test
./tools/trystan-bunker calibrate
```

The white overlay shows the ROI and left/front/right boundaries loaded from
`autonomy_d435i.yaml`. A report containing the factory-published color
intrinsics, field of view, aligned-depth size, sector sample, and active view
parameters is written to `~/vlm_results/d435i_calibration`.

This records camera intrinsics and helps tune the navigation view. It does not
guess the arm-to-camera transform: lock the arm, measure that pose, and provide
the `camera_*` launch arguments or publish it through the robot URDF.

For the complete two-stage tmux workflow, use:

```bash
./tools/start-d435i-tmux-pipeline calibrate
# Wait for the fresh summary in window 1 and inspect RViz in window 4.
# Detach with Ctrl-b then d.
./tools/start-d435i-tmux-pipeline dry_run
```

The first command creates separate calibration-info, camera, calibration-node,
and RViz windows. The second command adds separate VLM, chassis, autonomy, and
status windows without opening the D435i a second time.

### Continuous model/control timing

`simple_vlm` now performs one background request at a time and immediately
starts the next request with the newest image. ROS camera and route callbacks
remain responsive during network inference. `/vlm_status` reports the measured
request latency and whether each result was published, rejected as stale, or
failed.

The model does not publish at the 10 Hz motor-control rate. Instead,
`search_behavior_node` republishes the most recent fresh, locally validated
proposal at 10 Hz, and `cmd_vel_mux_node` publishes at 20 Hz. Live D435i depth
continues to stop or reroute the command independently. Model observations older
than three seconds are discarded and the behavior stops after eight seconds with
no accepted replacement.

## Legacy D555 + LiDAR pipeline

This package adds a bounded VLA-style autonomy layer for finding a trash can and
stopping 0.30 m from it. The remote vision-language model proposes a discrete
mobile-base action and fresh speed on every observation. RealSense depth and
LiDAR remain authoritative: they validate the requested path, reroute blocked
actions, and stop the robot when no route is safe. Every accepted command still
passes through a watchdog, acceleration limits, and final velocity caps.

Current behavior:

- There is no fixed rotate/forward timer loop. RGB plus the latest fused route
  envelope is sent to the model on every inference.
- The model returns `forward`, `curve_left`, `curve_right`, `turn_left`,
  `turn_right`, or `stop`, together with `[linear_x, angular_z]`, confidence,
  target position, and a normalized target bounding box.
- Target-visible mode does not force direct motion. The model can plan around a
  blocked straight path, and ROS validates front/left/right requirements live.
- A blocked proposal is rerouted through the best currently clear camera+LiDAR
  route. If no route is clear, the state is `vla_stuck_stop` and velocity is zero.
- A high-confidence target bbox associates raw depth with the trash can. Two
  centered depth confirmations at or below 0.30 m complete the mission.
- LiDAR and depth publish independent stop and route decisions. `sensor_fusion_node` requires both sensors to be fresh and never lets one sensor clear the other sensor's stop.
- Rerouting ranks camera sectors by longest robust depth clearance, requires the
  LiDAR robot-width box to fit, then uses LiDAR clutter count for near-ties.
- Normal commands use acceleration/deceleration ramps; emergency stops and watchdog failures still publish zero immediately.
- LiDAR evaluates only front/side candidate corridors. If none fits, autonomy
  publishes zero velocity; negative commands are rejected by the final mux.
- LiDAR evaluates forward candidate headings from -60 to +60 degrees plus the
  straight rear heading against a
  configurable metric corridor. The current requested corridor is 0.60 m wide,
  must remain free for at least 0.60 m, and is inspected out to 1.20 m. When a
  VLA action is blocked, the behavior turns toward the selected fitting angle
  before translating into the corridor.

## D435i door-search profile

`find_door_d435i.launch.py` uses a local YOLOv8 Open Images Door detector, confirmed object
tracking, and aligned D435i depth; it does not start a VLM node or make network
inference requests. It uses `planner_mode: forward_exploration`. With no door
detected, the robot drives forward only while the live depth route is clear. If
forward space is blocked, it uses the existing verified-route relocation state
machine; it never reverses or drives through a blocked route. Two consecutive
tracked YOLO door boxes are range-checked with aligned depth; two centered
confirmations at or below 0.60 m stop the robot and publish
`/mission_complete: true`, and end the launch. Start in `mode:=dry_run` before
using `mode:=drive`.

## Safety

- Do not run this with the chassis connected until `/safety_stop` has been tested with the actual LiDAR frame setup.
- Missing or stale raw-depth images also cause fail-safe `/safety_stop: true`.
- Camera depth is supplementary. Do not remove the LiDAR or the physical emergency stop.
- Model actions never bypass `/safety_stop` or route validation. The only
  directional exception is the bounded local reverse escape when a front/depth
  stop is active but fresh LiDAR independently proves the rear corridor clear.
  The model's
  requested speed is capped at 0.12 m/s and 0.30 rad/s again in the final mux.
- The D555 raw depth and RGB images are not perfectly registered. The central
  60% of a tight target bbox is used to reduce edge/parallax contamination;
  calibrate the camera-to-front-bumper offset before treating 0.30 m as a precise
  chassis clearance.
- The LiDAR emergency-stop box is unchanged. Only the non-emergency route look-ahead boxes are slightly shorter so depth can rank the longer corridor while LiDAR verifies near-term chassis fit.
- Non-emergency LiDAR route look-ahead spans are approximately 50% of their previous size. This does not weaken the emergency stop zone.
- The safety monitor evaluates the configured safety box in `base_frame` and defaults to `base_link`.
- If the point cloud frame is not `base_link`, publish a correct TF transform from `base_link` to the LiDAR frame. With the default config, missing TF causes fail-safe `/safety_stop: true`.
- Keep the output topic set to `/cmd_vel_debug` for dry runs.
- The measured 0.57 m vehicle in a 0.60 m corridor has only 0.015 m nominal
  clearance per side. Treat this as experimental: verify the true maximum width,
  LiDAR alignment, self-return filtering, and candidate-angle output in dry-run
  before considering chassis motion. A correct angle does not compensate for
  calibration error or track slip.
- For chassis tests, use blocks or an open area, low speed, and a physical emergency stop.

## Fix Missing TF Safety Stops

If you see a log like this, the safety stop is working as designed:

```text
Cannot transform point cloud from frame 'laser_link' to 'base_link'
Safety stop changed to True. reason=transform_error
```

The LiDAR cloud is in `laser_link`, but the safety box is configured in `base_link`.
Publish the real measured static transform from the robot base to the LiDAR before running
autonomy. For a dry run only, an identity transform can verify that the rest of the stack works:

```bash
source install/setup.bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.0 --y 0.0 --z 0.0 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id base_link \
  --child-frame-id laser_link
```

Or let the autonomy launch publish that static transform:

```bash
source install/setup.bash
ros2 launch bunker_autonomy find_trash_can_demo.launch.py \
  mode:=dry_run \
  publish_lidar_static_tf:=true \
  lidar_x:=0.0 lidar_y:=0.0 lidar_z:=0.0 \
  lidar_roll:=0.0 lidar_pitch:=0.0 lidar_yaw:=0.0
```

Replace the zero offsets with the LiDAR's actual position and orientation before using `/cmd_vel`
on the chassis. Verify TF with:

```bash
ros2 run tf2_ros tf2_echo base_link laser_link
```

### Recovering D555 frame timeouts on Jetson

If the camera is detected but repeatedly reports `Frames didn't arrived within 5 seconds`, stop
the camera launch with `Ctrl-C`, reset only the D555 USB device, and relaunch:

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 run bunker_autonomy realsense_usb_recover
ros2 launch bunker_autonomy realsense_d555.launch.py launch_rviz:=true
```

Run the recovery command inside the privileged `trystan-bunker` container. It refuses to reset the
USB device while a RealSense ROS node is active. The normal D555 configuration keeps
`initial_reset: false` because the librealsense startup reset can leave this Jetson/D555 pairing in
a UVC timeout or broken-pipe state.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select bunker_autonomy
source install/setup.bash
```

## Run

### Recommended: separately managed RealSense workflow

Start the LiDAR and RealSense separately (and the chassis driver only when using
`mode:=drive`), then launch the VLM and autonomy nodes:

```bash
cd ~/ros2_ws
colcon build --packages-select simple_vlm bunker_autonomy
source install/setup.bash
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY all_proxy ALL_PROXY
ros2 launch bunker_autonomy find_trash_can_realsense.launch.py mode:=dry_run
```

By default this launch reuses the existing RealSense process. Start it first with:

```bash
source install/setup.bash
ros2 launch bunker_autonomy realsense_d555.launch.py
```

The camera publishes:

- RGB on `/camera/camera/color/image_raw` for `simple_vlm`.
- Raw depth on `/camera/camera/depth/image_rect_raw`.
- Depth cloud on `/camera/camera/depth/color/points` for RViz.
- Depth safety and distance summaries on `/depth_safety_stop` and
  `/depth_route_status`.

Do not also run `ros2 launch realsense2_camera rs_launch.py`; two processes cannot
own the same USB camera. To have the integrated launch start the camera instead,
use `launch_realsense:=true` and do not start it separately.

Verify live frames and the published distance summary before enabling motion:

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic echo /depth_route_status
ros2 topic echo /target_status
ros2 topic echo /cmd_vel_debug
```

The multi-terminal workflow below remains useful for component debugging.

Terminal 1, LiDAR:

```bash
source install/setup.bash
ros2 launch lslidar_driver lslidar_cx_launch.py
```

Expected point cloud topic from the existing config is `/cx/lslidar_point_cloud`.

Terminal 2, camera:

```bash
source install/setup.bash
# Stable D555 color, raw depth, and point-cloud configuration.
ros2 launch bunker_autonomy realsense_d555.launch.py
```

Terminal 3, simple VLM:

```bash
source install/setup.bash
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY all_proxy ALL_PROXY
ros2 run simple_vlm simple_vlm_node --ros-args -p config_file:=src/simple_vlm/config.yaml
```

Terminal 4, autonomy dry run with no robot movement:

```bash
source install/setup.bash
ros2 launch bunker_autonomy find_trash_can_demo.launch.py mode:=dry_run
```

For one simple status message, use:

```bash
ros2 topic echo /autonomy_notice --once
```

When ready for a controlled chassis test, launch the chassis driver separately and use the real output topic:

```bash
source install/setup.bash
ros2 launch bunker_base bunker_base.launch.py port_name:=can3 is_bunker_mini:=true
ros2 launch bunker_autonomy find_trash_can_realsense.launch.py mode:=drive
```

## Expected Topics

- `/cx/lslidar_point_cloud` (`sensor_msgs/PointCloud2`)
- `/camera/camera/color/image_raw` (`sensor_msgs/Image`)
- `/camera/camera/depth/image_rect_raw` (`sensor_msgs/Image`)
- `/camera/camera/depth/color/points` (`sensor_msgs/PointCloud2`)
- `/vlm_result` (`std_msgs/String`)
- `/lidar_safety_stop` and `/depth_safety_stop` (`std_msgs/Bool`, raw sensor decisions)
- `/lidar_route_status` and `/depth_route_status` (`std_msgs/String`)
- `/safety_stop` (`std_msgs/Bool`)
- `/route_status` (`std_msgs/String`)
- `/safety_stop_reason` (`std_msgs/String`, attributed stop trigger JSON)
- `/target_detected` (`std_msgs/Bool`)
- `/target_status` (`std_msgs/String`)
- `/mission_complete` (`std_msgs/Bool`)
- `/mission_status` (`std_msgs/String`)
- `/autonomy_notice` (`std_msgs/String`)
- `/cmd_vel_autonomy` (`geometry_msgs/Twist`)
- `/cmd_vel` or `/cmd_vel_debug` (`geometry_msgs/Twist`)

## Debugging Commands

```bash
ros2 topic list
ros2 topic echo /vlm_result
ros2 topic echo /safety_stop
ros2 topic echo /route_status
ros2 topic echo /safety_stop_reason
ros2 topic echo /depth_safety_stop
ros2 topic echo /depth_route_status
ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic echo /target_detected
ros2 topic echo /target_status
ros2 topic echo /mission_status
ros2 topic echo /autonomy_notice
ros2 topic echo /cmd_vel_autonomy
ros2 topic echo /cmd_vel
ros2 topic echo /cmd_vel_debug
ros2 run tf2_ros tf2_echo base_link laser_link
```

### RealSense USB recovery

`VIDIOC_S_FMT ... Device or resource busy` means another process already owns a
camera video device. Stop every old RealSense launch before starting the integrated
workflow and confirm that only one driver remains:

```bash
ps -ef | grep -E 'realsense2_camera|rs_launch' | grep -v grep
fuser /dev/video* 2>/dev/null
```

`UVCIOC_CTRL_QUERY ... Protocol error` followed by a missing `/dev/video0` is a USB
device failure, not a VLM error. Stop the launch, reconnect or power-cycle the camera,
confirm it with `rs-enumerate-devices`, then restart the D555 launcher. Use a direct
USB 3 port and a short known-good cable; avoid starting a second camera launch while
recovering.

## Data Logging

The autonomy launch writes JSONL logs by default:

```bash
ls -lt ~/vlm_results/autonomy_logs
tail -f ~/vlm_results/autonomy_logs/trash_mission_*.jsonl
```

For the short transition-only stop history, use:

```bash
ros2 run bunker_autonomy show_safety_events
```

This reads the newest `safety_events_*.jsonl` file and prints entries such as:

```text
2026-07-14T10:00:00.000 | STOP | primary=depth_stale | reasons=depth_stale
2026-07-14T10:00:03.000 | CLEAR | primary=clear | reasons=none
```

Each regular mission row contains the latest VLM result, parsed target status, fused safety and route state,
individual LiDAR/depth decisions, autonomy command, final output command, mission status, and operator notice. The
separate safety-event file writes only reason transitions, including the primary trigger,
all contributing triggers, and the latest raw sensor topics. The
`simple_vlm` node also saves camera frames and VLM result JSON files in
`~/vlm_results`, which can be paired with the mission JSONL logs for later
training or failure analysis.

Disable autonomy JSONL logging with:

```bash
ros2 launch bunker_autonomy find_trash_can_demo.launch.py mode:=dry_run enable_data_logging:=false
```

## Test Plan

Test 1: No robot movement

```bash
ros2 launch bunker_autonomy find_trash_can_demo.launch.py mode:=dry_run
ros2 topic echo /cmd_vel_debug
ros2 topic echo /autonomy_notice
```

Confirm the commands look correct before connecting the chassis. When `/safety_stop` is true, all values should be zero. When safety is clear and no target is detected, angular `z` should be the configured low search speed.

Test 2: Fused safety stop

Run the LiDAR, put an object in front of the robot, and confirm:

```bash
ros2 topic echo /safety_stop
```

The topic should become `data: true` when enough LiDAR points enter the configured safety box or enough center depth pixels are within `hard_stop_distance_m`. Stop the camera or LiDAR and confirm stale input also produces `true`.

Inspect the three camera sectors while moving an object around, with no chassis motion:

```bash
ros2 topic echo /depth_route_status
ros2 topic echo /cmd_vel_debug
```

Tune the ROI and thresholds in `config/autonomy.yaml` for the measured camera mounting height and floor view before drive mode.

Test 3: Continuous VLA behavior without chassis

Run `simple_vlm` and the autonomy launch with `/cmd_vel_debug`. Inspect
`/vlm_result`, `/target_status`, and `/cmd_vel_debug`. Confirm every requested
action is fresh, within the final caps, and only uses a fused-clear route. Block
the requested route and verify either `REROUTE` selects another safe route or all
velocities become zero.

Test 4: Low-speed chassis test

Put the robot on blocks or in an open safe area. Keep the VLA safety ceilings low
in `config/autonomy.yaml`, launch the chassis driver, use `/cmd_vel`, and test the
emergency stop before any autonomous search.
