# Odom, IMU, and Fused Odom Metrics

This diagnostic adds a passive one-terminal TUI for checking whether raw wheel
odometry, IMU yaw rate, and EKF fused odometry are healthy enough for the Nav2
full system.

It does not publish `/cmd_vel`, reset localization, change EKF parameters, or
modify Nav2. It only subscribes to the current Terminal 1 topics:

| Topic | Meaning |
|---|---|
| `/wheel/odom` | Raw Bunker wheel odometry from the base driver |
| `/yesense/imu_data_ros` | H30/YESENSE IMU stream |
| `/odom` | EKF filtered odometry used by RTAB-Map and Nav2 |

## Why These Metrics Work Without Ground Truth

Without motion capture, survey marks, or another external reference, the tool
cannot prove absolute accuracy. It can still measure the things that usually
break Nav2:

- stationary drift while the robot is not moving;
- return-to-start error after forward/back, rotate-return, or loop tests;
- yaw disagreement between odometry and integrated IMU yaw rate;
- velocity noise and spikes;
- topic rate and freshness.

For this robot, fused `/odom` should generally win if the IMU yaw-rate frame,
sign, timing, and bias are correct. If fused `/odom` scores worse than raw
`/wheel/odom`, fix the IMU/EKF path before tuning Nav2.

## Start The Existing System

Start the normal hardware path from `NAV2_FULL_SYSTEM_STARTUP.md` first. The
important part is Terminal 1 with EKF enabled:

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
  start_ekf:=true
```

Do not continue if `/wheel/odom`, `/odom`, or `/yesense/imu_data_ros` is
missing.

## Run The One-Terminal TUI

In one extra terminal:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=173
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 run bunker_slam_bringup odom_imu_fusion_metrics_tui.py
```

This default mode is observe-only. It does not drive the robot.

If your topics are different:

```bash
ros2 run bunker_slam_bringup odom_imu_fusion_metrics_tui.py \
  --wheel-topic /wheel/odom \
  --fused-topic /odom \
  --imu-topic /yesense/imu_data_ros
```

To test the metric code without hardware or ROS topics:

```bash
ros2 run bunker_slam_bringup odom_imu_fusion_metrics_tui.py --self-test
```

This only checks the scoring/reporting logic with synthetic samples. It is not a
robot measurement.

## Optional Slow Drive Mode

The same TUI can run slow automated trial commands, but only when explicitly
requested:

```bash
ros2 run bunker_slam_bringup odom_imu_fusion_metrics_tui.py --drive-mode
```

By default, drive mode publishes to `/cmd_vel_debug`, which is the established
dry-run topic in this repository. The chassis should not move from
`/cmd_vel_debug`; this is useful for confirming the command profile and report
flow without driving.

For real robot motion, pass the command topic that your running safety stack
actually forwards to the Bunker base. Choose this intentionally after the stop
test passes:

```bash
ros2 run bunker_slam_bringup odom_imu_fusion_metrics_tui.py \
  --drive-mode \
  --cmd-topic /cmd_vel
```

If your current bringup uses a mux/gate path, use that gated input instead of
direct `/cmd_vel`.

Automated command limits are deliberately low:

| Trial | Command profile |
|---|---|
| stationary | zero command for 20 s |
| forward/back | +0.08 m/s, stop, -0.08 m/s, stop |
| rotate return | +0.18 rad/s, stop, -0.18 rad/s, stop |
| loop | four slow forward segments with slow left turns |
| one-way rotate | +0.18 rad/s, then stop |

The TUI publishes zero velocity when a trial stops, when `SPACE` is pressed, and
when quitting.

## TUI Keys

| Key | Trial | What to do |
|---|---|---|
| `s` | stationary | Keep the robot still for 60-120 seconds |
| `f` | forward/back | Drive forward, then reverse back to the same floor mark |
| `r` | rotate return | Rotate left/right, then return to the original heading |
| `l` | loop | Drive a rectangle and stop where you started |
| `o` | one-way rotate | Rotate one direction, for example 90 or 360 degrees |
| `SPACE` | stop | Stop and save the active trial |
| `q` | quit | Print the final report |

In observe-only mode, use a physical mark on the floor for the start pose and
manually drive each trial. In `--drive-mode`, pressing a trial key starts the
matching slow command profile and the tool auto-saves when the profile ends.

## How The Score Is Computed

The TUI computes per-trial values for raw wheel odom and fused odom:

```text
return_error = sqrt((x_end - x_start)^2 + (y_end - y_start)^2)
yaw_error = abs(wrap(yaw_end - yaw_start))
imu_yaw = integral(imu.angular_velocity.z dt)
yaw_vs_imu_error = abs(wrap(odom_yaw_delta - imu_yaw))
velocity_noise = stddev(linear_x) + stddev(angular_z)
```

For stationary, forward/back, rotate-return, and loop trials, lower
return/yaw error is better. For one-way rotation, yaw agreement with integrated
IMU is useful, but it is still not absolute truth.

The aggregate score is intentionally simple and conservative:

```text
stationary:
  score = 4 * return_error + 4 * yaw_error + velocity_noise

return-style tests:
  score = 3 * return_error + 4 * yaw_error
        + 0.5 * yaw_vs_imu_error + velocity_noise

one-way rotation:
  score = yaw_vs_imu_error + 0.5 * velocity_noise
```

Lower aggregate score means better no-ground-truth consistency. If fused
`/odom` has the lower score, keep using it for RTAB-Map and Nav2. If raw
`/wheel/odom` is consistently lower, the EKF is probably being hurt by IMU
frame/sign/bias/timing or covariance settings.

## Localization Jump When Loading A Saved Map

If the robot appears in the wrong place when loading a saved map, the odometry
metric tool will not directly fix that. It helps separate two different
problems:

| Symptom | Likely layer |
|---|---|
| `/odom` drifts or yaw is wrong before localization starts | Wheel/IMU/EKF problem |
| `/odom` is stable, but `map -> odom` jumps after loading a map | Localization initialization or map matching problem |

Common causes in the current Nav2 full system:

- The saved RTAB-Map database or occupancy map does not match the current
  environment layout.
- Localization started with no correct initial pose, so RTAB-Map or AMCL matched
  the current camera/scan data to the wrong similar-looking place.
- The robot was moved while the stack was off, then localization reused stale
  odom-relative assumptions.
- There are duplicate TF owners for `map -> odom` or `odom -> base_link`.
- The wrong map/database file was loaded.
- Camera TF or odom timing is stale, so visual localization receives inconsistent
  transforms.

Before blaming Nav2, verify:

```bash
ros2 run bunker_slam_bringup check_tf_ownership.py
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
```

Practical fixes:

- Always set a correct initial pose in RViz after loading a saved map.
- Start localization only after `/odom`, camera TF, and RGB-D topics are stable.
- Keep exactly one `map -> odom` owner and exactly one `odom -> base_link`
  owner.
- Use the same saved database/map pair that was produced from the current space.
- If localization repeatedly snaps to a wrong but similar corridor/room, add a
  distinctive initial pose workflow or use landmarks/AprilTags near startup
  areas.

Use the metric TUI first. If fused `/odom` is healthy, then the saved-map jump
is most likely a localization initialization or map-data association issue, not
an EKF issue.
