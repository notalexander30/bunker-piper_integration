# Codex handoff prompt: updated dual-PiPER URDF display pose

You are working on the real Bunker Mini Jetson over SSH.

## System locations

- SSH host alias: `jetson`
- SSH target: `dase-orin@192.168.1.148`
- ROS 2 distribution: Humble
- Robot workspace: `/home/dase-orin/ros2_ws`
- Package: `/home/dase-orin/ros2_ws/src/bunker_dual_piper_nav2`

Before changing anything, confirm these paths on the Jetson. Do not assume that
the similarly named `bunker_slam_bringup` package owns the robot description
just because its RViz configuration is open.

## Robot-frame convention

- `base_link`: Bunker body frame
- +X: Bunker forward
- +Y: Bunker left
- +Z: upward
- TF structure:
  `base_footprint -> base_link -> upper frame -> mounting plates -> PiPER arms -> cameras`

The two PiPER bases remain fixed to their mounting plates with the existing
mount transforms. Do not rotate a fixed mount merely to reproduce a parked arm
pose.

## Corrected parked arm pose

The desired left-facing display/default pose is:

```text
front PiPER: [+1.6, 0, 0, 0, 0, 0]
rear PiPER:  [-1.6, 0, 0, 0, 0, 0]
```

These values correspond to joints 1-6 in radians. Both gripper joints default
to zero.

Important distinction:

- `front_piper_mount_rpy` and `rear_piper_mount_rpy` describe the fixed physical
  installation of each complete arm.
- `front_piper_joint1 = +1.6` and `rear_piper_joint1 = -1.6` describe the parked
  moving-joint pose.
- A URDF describes joint geometry and limits; live/default `JointState` values
  determine the displayed pose of revolute joints.
- Never bake the +/-1.6 values into a fixed mount transform.

## Updated implementation

Inspect these source files:

1. `bunker_dual_piper_nav2/joint_state_prefixer.py`
   - Parameters `front_parked_joint1` and `rear_parked_joint1` default to -1.6
     and +1.6.
   - After startup it publishes a one-time parked pose only for an arm that has
     not supplied live feedback.
   - Live PiPER feedback remains authoritative and replaces the display default.
   - This keeps the not-yet-installed rear arm visible in RViz.

2. `launch/description.launch.py`
   - Exposes `front_piper_parked_joint1` and `rear_piper_parked_joint1` launch
     arguments.
   - Passes those values to the prefixer.
   - Uses the same values as the offline `joint_state_publisher` zero positions.

3. `launch/system_bringup.launch.py`
   - Declares and forwards the same parked-joint launch arguments.

4. `urdf/bunker_dual_piper_d435i.urdf.xacro`
   - Contains the fixed Bunker, upper-frame, mounting-plate, PiPER-base and D435i
     transforms.
   - The current camera transforms are estimates and have not been centered or
     calibrated yet.

5. `rviz/bunker_nav2.rviz`
   - Displays the visual 3D URDF from `/robot_description` over the 2D `/map`.
   - Uses a top-down orthographic view by default.
   - Hides URDF collision geometry.
   - Keeps the global and local costmap displays optional and disabled by
     default so the local rolling window is not mistaken for the robot body.

6. `rviz/bunker_description_2d.rviz`
   - Dedicated top-down projection of the same live `/robot_description` and TF
     tree; no second or flattened URDF is used.
   - Follows `base_link`, draws the Nav2 footprint as a separate gold outline,
     and keeps costmap overlays optional.

## Safety constraints

- Do not enable either PiPER arm.
- Do not publish trajectories, commands or target joint positions.
- Do not change `auto_enable` to true.
- Treat this task as robot-description and visualization work only.
- Do not change camera extrinsics without measurements or clear photographic
  evidence from the user.
- Preserve any Jetson-only configuration and unrelated local changes.

## Deployment task

1. Inspect the Jetson source package and confirm whether it contains the updated
   parked-pose implementation.
2. Back up only files that will be overwritten.
3. Copy or apply the updated source files to the Jetson package.
4. Build only the affected package:

   ```bash
   cd /home/dase-orin/ros2_ws
   source /opt/ros/humble/setup.bash
   colcon build --packages-select bunker_dual_piper_nav2 --symlink-install
   source install/setup.bash
   ```

5. Do not restart an active robot launch without first telling the user. A
   restart may interrupt Nav2, RTAB-Map and visualization even though this
   update does not command arm motion.

## Verification

With the PiPER arms disabled, verify:

1. The package builds without errors.
2. `robot_state_publisher` uses the combined description.
3. `/joint_states` contains prefixed joint names such as
   `front_piper_joint1` and `rear_piper_joint1`.
4. If the front driver is publishing, its live value overrides the +1.6 display
   default.
5. If the rear driver is absent, the rear model appears using -1.6 for joint 1
   and zero for its remaining joints.
6. Both arms remain connected to their mounting plates in the TF tree.
7. RViz shows both arms facing toward Bunker +Y (left) in the parked pose.

Useful read-only checks include:

```bash
ros2 topic echo --once /joint_states
ros2 node list
ros2 param get /robot_state_publisher robot_description
ros2 run tf2_ros tf2_echo base_link front_piper_base_link
ros2 run tf2_ros tf2_echo base_link rear_piper_base_link
```

If the visual result is incorrect, determine whether the problem comes from a
fixed mount transform, joint-state sign/name mapping, front/rear topic mapping,
or RViz fixed-frame selection before editing the URDF.

## Camera follow-up

The front D435i is installed at the wrist; the rear D435i is not installed yet.
Camera centering is a separate calibration task. Request measured XYZ offsets
and the intended optical direction before changing `front_camera_xyz` or
`front_camera_rpy`. Validate the camera link and optical-frame axes in RViz and
against a real depth/point-cloud observation after any correction.
