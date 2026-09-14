# Getting started with Bunker Nav-Man

This guide takes a new operator from a downloaded repository to a safe,
complete Nav-Man startup. The first launch keeps chassis navigation and PiPER
task execution in dry-run mode. Do not enable physical motion until every
validation check passes and an operator has the emergency stop in reach.

## What the complete workflow starts

The automatic launcher creates one tmux session containing:

1. Bunker, front/rear PiPER, front D435i, robot description, and TF.
2. RTAB-Map mapping or localization.
3. Nav2 in dry-run mode.
4. Front-arm MoveIt and its trajectory bridge.
5. ArUco detection, marker search, wall approach, and touch services.
6. The local API on port `8892` and agent-compatible gateway on `8893`.
7. Watchdogs and optional frontier exploration in an inactive state.

The launcher deliberately does not make the robot drive or execute the marker
task automatically.

## Before you begin

You need:

- An Ubuntu 22.04 host with Docker Engine and Docker Compose.
- An AgileX Bunker and at least the front PiPER arm.
- A front Intel RealSense D435i.
- Supported CAN adapters, correct cables, and powered hardware.
- A working physical emergency stop and a clear test area.
- Internet access for the first image build.
- An X11 display only if RViz will be used.

Confirm Docker is available:

```bash
docker --version
docker compose version
```

## Step 1: Download and unpack the repository

Choose one method.

### Method A: clone with Git

This is recommended because later updates are easier:

```bash
git clone https://github.com/notalexander30/bunker-piper_integration.git
cd bunker-piper_integration
```

### Method B: download the ZIP from GitHub

On the GitHub repository page, select **Code → Download ZIP**. Then run:

```bash
cd ~/Downloads
unzip bunker-piper_integration-main.zip
mv bunker-piper_integration-main bunker-piper_integration
cd bunker-piper_integration
```

If the downloaded filename differs, substitute the real filename. Do not place
the project inside another ROS package; this repository already contains four
ROS packages.

## Step 2: Build the Docker image

Run this from the repository root—the directory containing `README.md`:

```bash
docker compose -f docker/compose.yaml build bunker-nav-man
```

The first build can take a while. It downloads ROS 2 Humble, imports the exact
source revisions in `dependencies.repos`, installs rosdep packages, and builds
the maintained workspace. A warning from an upstream package is not necessarily
a failure; the command must finish with a successful exit status.

## Step 3: Start and enter the container

If RViz is needed, allow the container to reach the host display:

```bash
export DISPLAY=${DISPLAY:-:1}
xhost +SI:localuser:root
```

Start the named container and open a shell:

```bash
docker compose -f docker/compose.yaml up -d bunker-nav-man
docker exec -it bunker-nav-man bash
```

Commands from this point through Step 9 run **inside the container**, unless a
step explicitly says otherwise.

## Step 4: Load the ROS environment

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=173
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Use the same three ROS environment values in every additional shell. Different
values can make a healthy ROS graph appear empty.

Optional VLA navigation also requires a runtime credential. Leave it unset if
that feature is not being used:

```bash
export DASHSCOPE_API_KEY=
```

Never commit an actual credential to a YAML file.

## Step 5: Discover the hardware identifiers

Do not guess the `canN` assignments. Linux can change them after a reboot or
when adapters are reconnected.

```bash
ip -br link show type can
for interface in /sys/class/net/can*; do
  [ -e "$interface" ] && udevadm info -q property -p "$interface" | \
    grep -E 'ID_SERIAL=|ID_PATH='
done
rs-enumerate-devices -s
```

Record the actual values in this shell:

```bash
export BUNKER_CAN=
export FRONT_PIPER_CAN=
export REAR_PIPER_CAN=
export FRONT_CAMERA_SERIAL=
```

Fill in the blank values. The expected bitrates are:

| Device | Expected interface | Bitrate |
|---|---|---:|
| Bunker | value stored in `BUNKER_CAN` | 500000 |
| Front PiPER | value stored in `FRONT_PIPER_CAN` | 1000000 |
| Rear PiPER | value stored in `REAR_PIPER_CAN` | 1000000 |

For stable adapter-to-role mapping, follow [CAN discovery](can_discovery.md).

## Step 6: Choose mapping or localization

RTAB-Map localization requires an existing database. Use one of these paths.

### First visit to a site: create a map

Keep arm task execution disabled and start mapping:

```bash
ros2 run bunker_slam_bringup start_nav_man_workflow.py --replace \
  --mapping-mode mapping --reset-database \
  --database-path /ros2_ws/maps/site.db \
  --bunker-can "$BUNKER_CAN" \
  --front-piper-can "$FRONT_PIPER_CAN" \
  --rear-piper-can "$REAR_PIPER_CAN" \
  --front-camera-serial "$FRONT_CAMERA_SERIAL"
tmux attach -t 0
```

Drive slowly while mapping only after verifying the command route and safety
stop. Detach from tmux with `Ctrl-b`, then `d`. When mapping is complete, stop
the stack as shown in Step 9. The database remains in the Docker `nav_man_maps`
volume at `/ros2_ws/maps/site.db`.

### Returning to a mapped site: localize

Confirm the database exists:

```bash
test -f /ros2_ws/maps/site.db && echo "map database found"
```

Then start the complete workflow in localization mode:

```bash
ros2 run bunker_slam_bringup start_nav_man_workflow.py --replace \
  --mapping-mode localization \
  --database-path /ros2_ws/maps/site.db \
  --bunker-can "$BUNKER_CAN" \
  --front-piper-can "$FRONT_PIPER_CAN" \
  --rear-piper-can "$REAR_PIPER_CAN" \
  --front-camera-serial "$FRONT_CAMERA_SERIAL"
tmux attach -t 0
```

If the rear PiPER is not installed or not ready, omit its argument and add:

```text
--disable-rear-piper
```

## Step 7: Understand the tmux screen

Each tmux window owns one part of the system. Useful keys are:

| Keys | Action |
|---|---|
| `Ctrl-b`, then `n` | Next window |
| `Ctrl-b`, then `p` | Previous window |
| `Ctrl-b`, then a number | Select a numbered window |
| `Ctrl-b`, then `d` | Detach without stopping anything |

The most important windows are `t1_hardware`, `t2_mapping`, `t3_nav2`,
`t5_moveit`, `t8_aruco`, `t11_api_8892`, and `t13_watchdogs`. A process that
exits immediately in one of these windows should be diagnosed before motion is
enabled.

## Step 8: Validate the safe startup

Open a second container shell from the host:

```bash
docker exec -it bunker-nav-man bash
```

Load the environment from Step 4, then check:

```bash
ros2 topic hz /odom
ros2 topic hz /front_camera/color/image_raw
ros2 topic hz /front_camera/depth/color/points
ros2 topic echo --once /joint_states
ros2 run tf2_ros tf2_echo map base_link
ros2 action list | grep follow_joint_trajectory
curl -fsS http://127.0.0.1:8892/health
curl -fsS http://127.0.0.1:8893/health
```

Before allowing movement, verify all of the following:

- The Bunker odometry changes correctly when the base is moved safely.
- The front image is live and the optical frame faces forward.
- The aligned depth point cloud has plausible distances.
- `map → odom → base_link` is continuous and does not jump.
- Front and rear joint names correspond to the physical arms.
- The MoveIt model agrees with the real arm pose.
- Obstacles stop or reject unsafe commands.
- API health is ready, or clearly reports only an intentionally disabled part.

The full acceptance procedure is in the [Nav-Man integration runbook](../bunker_slam_bringup/NAV_MAN_INTEGRATION_STARTUP.md#mode-7-validation-checks).

## Step 9: Stop cleanly

Inside the container:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 run bunker_slam_bringup pkill_bunker_nav2_stack.sh
ros2 daemon stop
ros2 daemon start
```

Then, from the host if the container is no longer needed:

```bash
docker compose -f docker/compose.yaml down
```

`docker compose down` removes the container but preserves the named map volume.
Do not add `--volumes` unless deleting saved maps is intentional.

## Enabling real movement

The initial automatic startup is for validation. Physical PiPER task execution
requires restarting the launcher with `--allow-piper-motion`. Nav2 chassis drive
mode is a separate explicit change. Follow the staged instructions in the
[full runbook](../bunker_slam_bringup/NAV_MAN_INTEGRATION_STARTUP.md#mode-6-manual-terminal-by-terminal-startup),
and test one subsystem at a time with the emergency stop in reach.

Do not treat a green CI badge as physical safety certification. CI verifies that
the software dependencies build and the automated tests pass; it cannot verify
wiring, transforms, calibration, collision clearance, or the emergency stop.

## If a step fails

Open the [troubleshooting guide](troubleshooting.md). When asking for help,
include the failing command, its complete error text, the affected tmux window,
and the diagnostic output requested at the end of that guide.
