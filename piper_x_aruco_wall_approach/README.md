# PiPER-X ArUco Wall Approach

This ROS 2 Jazzy package makes the front PiPER-X
`front_piper_flange_link` approach a wall-mounted ArUco marker using the
integrated MoveIt model and wrist RealSense D435i depth cloud.

It supports two commands:

- `approach`: stop with `front_piper_flange_link` at a pre-touch clearance,
  default `0.05 m`.
- `touch`: calculate the final marker target from the current camera/depth state and send one MoveIt plan directly to that target, default final clearance `0.005 m`.
- `go-home`: move to the saved six-joint home pose.
- `go-previous`: move back to the saved six-joint previous pose.

There is no force or tactile sensor. A successful `touch` reports:

```json
{
  "contact_confirmed": false,
  "completion_type": "single_moveit_marker_touch"
}
```

## Architecture

```text
RealSense D435i color/depth
  -> aruco_ros /aruco_single/pose in base_link
  -> PointCloud2 /front_camera/depth/color/points
  -> wall_approach_node
       -> fits wall plane near marker
       -> publishes target poses and normal
       -> updates MoveIt detected_wall collision object
       -> plans/executes front_piper_flange_link targets through MoveIt
       -> prefers elbow/wrist motion by keeping joint1 near its current angle
  -> /run_marker_task ROS 2 service
  -> piper_touch_marker_api HTTP bridge on 127.0.0.1:8892
       -> saves previous pose before physical marker/home motion
  -> ABot-Claw piper-touch-marker skill
```

The calibrated TF chain must be:

```text
base_link
  -> PiPER-X links
  -> front_piper_flange_link
  -> front_camera_link
  -> front_camera_color_optical_frame
```

## Build

Install the Python API dependencies if they are not already present:

```bash
sudo apt update
sudo apt install -y python3-fastapi python3-uvicorn
```

```bash
cd ~/agx_arm_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select piper_x_aruco_wall_approach
source install/setup.bash
```

## One-Time Service Install

From the ABot-Claw-piper repository:

```bash
cd ~/ABot-Claw-piper-publish
./deployment/scripts/install_piper_touch_marker_service.sh
```

The installer enables the user service but does not start it. It does not configure CAN.

`can2` must already be configured and UP before starting the stack. The PiPER-X
arm is expected on `can2`; the Bunker base is expected on `can4`.

## Managed Startup

Plan-only default:

```bash
systemctl --user start piper-touch-marker-stack
systemctl --user status piper-touch-marker-stack
```

Logs:

```bash
journalctl --user -u piper-touch-marker-stack -f
```

Health:

```bash
curl -s http://127.0.0.1:8892/health | python3 -m json.tool
```

Health separates system readiness from marker visibility. When marker `6` is
outside the camera view, a healthy stack should report `system_ready=true`,
`ready_for_search=true`, `marker_visible=false`, and `ready_for_approach=false`.
`marker_pose_available` and `point_cloud_available` mean fresh data, not just
"seen once." By default the API requires a marker pose received within `1.0 s`
and a point cloud received within `2.0 s`. The response includes
`marker_pose_age_s` and `point_cloud_age_s` so stale data is visible.

Enable physical execution for an operator-approved session:

```bash
systemctl --user set-environment PIPER_TOUCH_ALLOW_EXECUTION=1
systemctl --user restart piper-touch-marker-stack
```

Disable physical execution again:

```bash
systemctl --user unset-environment PIPER_TOUCH_ALLOW_EXECUTION
systemctl --user restart piper-touch-marker-stack
```

Stop the stack:

```bash
systemctl --user stop piper-touch-marker-stack
```

## Direct Full-Stack Launch

Use this when not running systemd:

```bash
source /opt/ros/jazzy/setup.bash
source ~/agx_arm_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch piper_x_aruco_wall_approach touch_marker_full_stack.launch.py \
  execute_allowed:=false \
  calibration_file:=/home/dase-hw101/handeye/config/piper_x_d435i_eye_in_hand.json \
  piper_namespace:=front_piper \
  use_piper_motion_stack:=false \
  use_handeye_tf_publisher:=false \
  can_port:=can2 \
  use_realsense:=false \
  camera_image_topic:=/front_camera/color/image_raw \
  camera_info_topic:=/front_camera/color/camera_info \
  point_cloud_topic:=/front_camera/depth/color/points \
  camera_root_frame:=front_camera_link \
  camera_optical_frame:=front_camera_color_optical_frame \
  use_front_piper_joint_state_adapter:=false \
  integrated_joint_state_topic:=/joint_states \
  front_piper_joint_prefix:=front_piper_ \
  joint_state_topic:=/front_piper/feedback/joint_states \
  control_topic:=/front_piper/control/joint_states \
  trajectory_action:=/front_piper/arm_controller/follow_joint_trajectory \
  marker_id:=6 \
  marker_size:=0.06 \
  prefer_elbow_motion:=true \
  goal_orientation_tolerance:=0.35 \
  marker_timeout:=1.0 \
  point_cloud_timeout:=2.0
```

In integrated Bunker mode, the launch subscribes to Trystan's front RealSense,
driver feedback, MoveIt, and TF topics and does not start duplicate hardware or
MoveIt nodes. The launch starts only:

- packaged hand-eye TF publisher
- `aruco_ros` marker detector
- `search_marker_node`
- `wall_approach_node`
- HTTP API bridge

It must not be run together with another PiPER-X driver on the selected CAN
interface. The PiPER-X arm uses `can2`. The Bunker base uses `can4`.

The combined Bunker URDF publishes prefixed names such as
`front_piper_joint1` on `/joint_states`, while Trystan's namespaced front
MoveIt model intentionally uses raw `joint1..joint6` and `tcp_link`. The
integrated front driver already publishes raw feedback on
`/front_piper/feedback/joint_states`, so this package does not start the
adapter by default. Enable `use_front_piper_joint_state_adapter:=true` only
when that raw topic is absent.

MoveIt clients use `/front_piper/robot_description` with
`/front_piper/robot_description_semantic`; do not pair the front-only SRDF
with the combined root `/robot_description`. Perception uses the combined TF
chain `base_link -> front_piper_flange_link ->
front_camera_color_optical_frame`.

Motion interfaces are namespaced under `/front_piper`:

- action: `/front_piper/arm_controller/follow_joint_trajectory`
- control topic: `/front_piper/control/joint_states`
- MoveIt namespace: `/front_piper`
- control gate service: `/front_piper/control_enable`
- controller-manager hardware check:
  `/front_piper/controller_manager/list_hardware_components`

If another stack already starts a compatible front-PiPER MoveIt/controller
bundle, set `use_piper_motion_stack:=false` and keep the same interface names.
If Trystan's combined URDF publishes the camera TF, keep
`use_handeye_tf_publisher:=false` to avoid duplicate static transforms.

AgileX's `start_single_agx_arm_moveit.launch.py` intentionally includes the
MoveIt demo ros2_control controller. `/health` may therefore report
`hardware_fake: true` with `FakeSystem` or `mock_components/GenericSystem`.
That is allowed when the fake controller is being used as a trajectory sampler
and its output is remapped to the AGX driver command topic,
`/front_piper/control/joint_states`. Search still verifies real motion by
watching `/front_piper/feedback/joint_states` after MoveIt execution.

## Marker Search

`approach-marker` and `touch-marker` check the current camera view first. If
marker `6` is visible, they run the existing wall-plane and MoveIt
approach/touch pipeline unchanged. If marker `6` is hidden, OpenClaw should use
reactive search through the Agent Server `/tools/search-step` endpoint.

Direct search debug:

```bash
ros2 run piper_x_aruco_wall_approach piper_touch_marker_client.py search --execute
```

Search parameters live in `config/piper_x_search_poses.yaml`:

- no hardcoded 3x3 joint-pose grid
- full `direction=auto` follows `current -> right -> left -> up -> up_right ->
  up_left -> center -> down -> down_right -> down_left`
- joint1 performs the wide horizontal coverage; joint4 only selects upper/lower
  camera levels
- `settle_time_s: 0.2`
- `detection_window_frames: 5`
- `required_detections: 3`
- allowed directions: `left`, `right`, `up`, `down`, `up_left`, `up_right`,
  `down_left`, `down_right`, `center`, `current`
- each direction maps to a small configured six-joint delta and is planned
  through MoveIt
- auto search uses wide bounded joint1 sectors and upper/center/lower camera
  levels instead of the old joint4-first joint-limit sweep

There is no wall-clock or max-step search limit in the full ROS search loop.
Search stops when marker 6 is confirmed or when the finite joint sweep is
exhausted.

## Operator Client

Health:

```bash
ros2 run piper_x_aruco_wall_approach piper_touch_marker_client.py health
```

Plan-only approach:

```bash
ros2 run piper_x_aruco_wall_approach piper_touch_marker_client.py approach
```

Plan-only geometric touch:

```bash
ros2 run piper_x_aruco_wall_approach piper_touch_marker_client.py touch --retract
```

Physical approach, only after `PIPER_TOUCH_ALLOW_EXECUTION=1`:

```bash
ros2 run piper_x_aruco_wall_approach piper_touch_marker_client.py approach --execute
```

Physical geometric touch and retract:

```bash
ros2 run piper_x_aruco_wall_approach piper_touch_marker_client.py touch \
  --execute \
  --final-clearance 0.005 \
  --retract
```

Save current pose as the previous pose without moving:

```bash
ros2 run piper_x_aruco_wall_approach piper_touch_marker_client.py save-previous
```

Plan-only return to the previous pose:

```bash
ros2 run piper_x_aruco_wall_approach piper_touch_marker_client.py previous
```

Physical return to the previous pose, only after `PIPER_TOUCH_ALLOW_EXECUTION=1`:

```bash
ros2 run piper_x_aruco_wall_approach piper_touch_marker_client.py previous --execute
```

The previous pose is saved automatically before physical `approach`, `touch`,
and `go-home` commands. If no previous pose file exists yet, call
`save-previous` from a known safe pose before using `go-previous`.

### Persistent home pose from MoveIt/RViz2

The home pose is a six-joint snapshot read from the live integrated
`/joint_states` feedback. To teach a new home pose, move the physical front
arm with MoveIt/RViz2, wait until the displayed joints stop changing, then run:

```bash
ros2 run piper_x_aruco_wall_approach piper_touch_marker_client.py save-home
```

The pose is saved at `/ros2_ws/config/piper_x_home_pose.yaml` by default, not
inside the installed ROS package. It therefore survives node restarts and
workspace rebuilds. The checked-in package pose is used only as a bootstrap if
the runtime file does not exist. Override the location with
`PIPER_X_HOME_POSE_FILE` when the workspace uses a different persistent mount.

Teaching mode is compatible with this workflow: `save-home` only records fresh
joint feedback and does not send a trajectory or require the arm to leave
teaching mode. Exit teaching mode before running `go-home`, since that command
must send a trajectory through the physical front-arm controller.

Equivalent HTTP:

```bash
curl -sS http://127.0.0.1:8892/tools/piper/touch-marker \
  -H 'Content-Type: application/json' \
  -d '{
    "execute": true,
    "pre_clearance_m": 0.05,
    "final_clearance_m": 0.005,
    "retract_after": false,
    "retract_distance_m": 0.05,
    "final_velocity_scaling": 0.05
}' | python3 -m json.tool
```

Previous-pose HTTP:

```bash
curl -sS -X POST http://127.0.0.1:8892/tools/piper/save-previous \
  -H 'Content-Type: application/json' \
  -d '{}' | python3 -m json.tool

curl -sS -X POST http://127.0.0.1:8892/tools/piper/go-previous \
  -H 'Content-Type: application/json' \
  -d '{"execute":true,"duration_s":6.0}' | python3 -m json.tool
```

## ROS Services and Topics

Legacy service preserved:

```bash
ros2 service call /run_wall_approach std_srvs/srv/Trigger "{}"
```

Structured service:

```bash
ros2 service call /run_marker_task piper_x_aruco_wall_approach/srv/RunMarkerTask \
  "{mode: approach, execute: false, pre_clearance_m: 0.05, final_clearance_m: 0.005, retract_distance_m: 0.05, final_velocity_scaling: 0.16, retract_after: false}"
```

Published targets:

```text
/wall_approach/target_pose
/wall_approach/final_target
/wall_approach/normal
```

## ABot-Claw Examples

Natural-language examples for the `piper-touch-marker` skill:

```text
Approach the marker.
Touch ArUco marker 6.
Press the marked location.
Move the Piper arm to the marker.
Go back to the previous pose.
Save current pose as previous pose.
```

ABot-Claw must call `/health` first. It must not call `execute=true` unless
health reports `execution_allowed: true` and the required camera, MoveIt, marker
or search, and joint-state readiness gates are satisfied.

## Diagnostics

```bash
systemctl --user status piper-touch-marker-stack
journalctl --user -u piper-touch-marker-stack -f
curl -s http://127.0.0.1:8892/health | python3 -m json.tool
ros2 service list | grep marker
ros2 topic echo /aruco_single/pose --once
ros2 topic hz /front_camera/depth/color/points
ros2 run tf2_ros tf2_echo base_link front_camera_color_optical_frame
ros2 topic list | grep points
```

Override the cloud topic if needed:

```bash
ros2 launch piper_x_aruco_wall_approach touch_marker_full_stack.launch.py \
  point_cloud_topic:=/actual/points/topic
```

## Troubleshooting

If `/health` says `marker_pose_available=false`, confirm ArUco ID `6`, marker size `0.03`, marker visibility, and:

```bash
ros2 topic echo /aruco_single/pose --once
```

If `/health` says `point_cloud_available=false`, find the actual RealSense cloud topic:

```bash
ros2 topic list | grep points
```

If the service reports a transform error, verify the calibrated chain:

```bash
ros2 run tf2_ros tf2_echo base_link front_camera_color_optical_frame
```

If the API rejects `execute=true`, enable execution deliberately:

```bash
systemctl --user set-environment PIPER_TOUCH_ALLOW_EXECUTION=1
systemctl --user restart piper-touch-marker-stack
```

If MoveIt planning fails, inspect `/wall_approach/final_target`, `/wall_approach/target_pose`, and the `detected_wall` collision object in RViz.

The marker planner defaults to `prefer_elbow_motion:=true`. It first tries to keep `joint1` within small tolerances around the current angle (`0.03`, `0.06`, `0.10`, then `0.15` rad), so the arm should prefer joints 2/3/4/5 instead of yawing the base. If the marker is physically reachable only with more base yaw, widen `joint1_planning_tolerances_rad` in `config/wall_approach.yaml` deliberately.
