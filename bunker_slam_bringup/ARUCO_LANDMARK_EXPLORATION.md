# ArUco landmark exploration

This replaces the YOLO-door idea for the first working version.

Behavior:

1. Save `home` from the robot pose when the ArUco node first receives camera data and valid TF.
2. Detect ArUco marker IDs from the RGB camera.
3. Save each repeatedly seen marker as a door landmark.
4. Save the robot pose at detection time as the door `approach_pose`.
5. Frontier exploration can finish by one of three modes:
   - `aruco_found`: stop when any marker has been seen enough times.
   - `time_limit`: explore until the timer finishes.
   - `frontier_ratio`: explore until the frontier ratio is below the threshold.

The landmark file is saved by default at:

```bash
/ros2_ws/maps/aruco_landmarks.json
```

## Marker setup

Recommended first marker dictionary:

```text
DICT_4X4_50
```

Default marker ID:

```text
6
```

This matches the default used in Iliyas' `piper_x_aruco_wall_approach` launch.
The detector filters to ID `6` by default. Use `target_marker_ids:=all` only if
you intentionally want to accept every visible ArUco marker.

Marker size:

```text
0.05 m
```

Use different marker IDs above different doors.

## Launch

Run this after camera, odometry, RTAB-Map mapping, TF, and Nav2 are already healthy.

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

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

Start the exploration:

```bash
ros2 service call /frontier_explorer/start std_srvs/srv/Trigger "{}"
```

## Other completion modes

Explore for two minutes, recording all ArUco landmarks:

```bash
ros2 launch bunker_slam_bringup aruco_landmark_exploration.launch.py \
  completion_mode:=time_limit \
  target_marker_ids:=6 \
  time_limit_sec:=120.0
```

Explore until frontier ratio is low:

```bash
ros2 launch bunker_slam_bringup aruco_landmark_exploration.launch.py \
  completion_mode:=frontier_ratio \
  target_marker_ids:=6 \
  frontier_stop_ratio:=0.20
```

## Monitor

```bash
ros2 topic echo /aruco_landmarks/summary
ros2 topic echo /aruco_landmarks/found
ros2 topic hz /front_camera/color/image_raw
ros2 run tf2_ros tf2_echo map base_link
```

RViz marker topic:

```text
/aruco_landmarks/markers
```

## Saved landmark example

```json
{
  "id": "aruco_10",
  "marker_id": 10,
  "type": "door",
  "valid": true,
  "seen_count": 5,
  "distance_from_home_m": 4.2,
  "approach_pose": {
    "frame_id": "map",
    "x": 3.5,
    "y": -1.2,
    "yaw": 1.57
  }
}
```

For the first version, Nav2 should later go to `approach_pose`, not to the physical marker position.

## Navigate to saved landmarks

Start this after Nav2 is already running and `/navigate_to_pose` is available:

If you launched `aruco_landmark_exploration.launch.py`, this navigator is already
started by default through `start_landmark_navigator:=true`. Use the separate
launch below only when you want to start the navigator by itself.

```bash
ros2 launch bunker_slam_bringup landmark_navigator.launch.py \
  landmark_path:=/ros2_ws/maps/aruco_landmarks.json
```

Go home:

```bash
ros2 service call /landmark_navigator/go_home std_srvs/srv/Trigger "{}"
```

Go to the furthest valid ArUco landmark:

```bash
ros2 service call /landmark_navigator/go_furthest std_srvs/srv/Trigger "{}"
```

Go to Iliyas' marker ID `6`:

```bash
ros2 topic pub --once /landmark_navigator/go_marker std_msgs/msg/String "{data: 'aruco_6'}"
```

The navigator sends Nav2 to the saved `approach_pose`, not directly to the
physical marker. This is intentional because the marker is mounted above the
door; the useful robot goal is the pose where the robot saw it clearly.
