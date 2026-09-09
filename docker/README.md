# Docker Setup

This folder provides the ROS 2 Humble development container for the Bunker
Nav-Man workflow. It does not replace robot-specific CAN/camera discovery or
physical calibration.

## Build

From the repo root:

```bash
docker compose -f docker/compose.yaml build
```

## Start A Shell

Allow local X11 access for RViz:

```bash
export DISPLAY=${DISPLAY:-:1}
xhost +SI:localuser:root
```

Start the container:

```bash
docker compose -f docker/compose.yaml run --rm bunker-nav-man
```

Inside the container:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Build After Editing

```bash
cd /ros2_ws
colcon build --symlink-install \
  --packages-select bunker_slam_bringup bunker_dual_piper_nav2 bunker_autonomy piper_x_aruco_wall_approach
source install/setup.bash
```

## Preview URDF Without Hardware

```bash
ros2 launch bunker_dual_piper_nav2 system_bringup.launch.py \
  start_hardware_drivers:=false \
  start_cameras:=false \
  start_nav2:=false \
  publish_default_joint_states:=true \
  prefix_joint_states:=false \
  start_rviz:=true
```

## Real Robot Setup

Before starting hardware:

```bash
ip -br link show type can
rs-enumerate-devices
```

Then follow:

- `docs/can_discovery.md`
- `bunker_slam_bringup/NAV_MAN_INTEGRATION_STARTUP.md`
- `docs/openclaw_gateway.md`

Use discovered `canX` names in launch arguments. Do not assume a particular
`canN` assignment on a new machine.
