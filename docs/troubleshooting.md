# Nav-Man troubleshooting

Start with the symptom below. Apply one change at a time, then repeat the exact
check that failed. Keep physical motion disabled while diagnosing the system.

## Fast checks

Inside `bunker-nav-man`, load the same environment in every shell:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=173
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Then collect a quick snapshot:

```bash
ros2 node list
ros2 topic list
ip -details -statistics link show type can
rs-enumerate-devices -s
tmux list-windows -t 0
```

## Common problems

| Symptom | Likely cause | What to do |
|---|---|---|
| `docker: command not found` | Docker is not installed on the host. | Install Docker Engine and the Compose plugin, then confirm `docker --version` and `docker compose version`. |
| Docker socket `permission denied` | The current host user cannot use Docker. | Use the host’s approved Docker group setup, sign out/in, and retry. Avoid changing device permissions blindly. |
| Compose cannot find its file | Command was run from the wrong directory. | Change to the repository root and use `docker compose -f docker/compose.yaml …`. |
| Image build fails during download | Network, DNS, package mirror, or GitHub rate issue. | Confirm internet/DNS, retry the failed build, and preserve the first real error rather than the final summary line. |
| `install/setup.bash` is missing | The image/workspace was not built, or a build failed. | Rebuild the Docker image. After source edits, run `colcon build --symlink-install` inside `/ros2_ws`. |
| RViz says it cannot connect to display | X11 was not authorized or `DISPLAY` differs. | On the host run `export DISPLAY=${DISPLAY:-:1}` and `xhost +SI:localuser:root`, then recreate or re-enter the container with the same display. |
| `tmux session 0 already exists` | A previous workflow is still running. | Inspect it with `tmux attach -t 0`; use `--replace` only when replacing it is intended. |
| ROS node/topic lists are empty | ROS environment or domain settings differ between shells. | Source both setup files and use `ROS_DOMAIN_ID=173`, `ROS_LOCALHOST_ONLY=1`, and the same RMW implementation everywhere. Restart the ROS daemon afterward. |
| No `canN` devices appear | Adapter driver, USB connection, or host setup is missing. | Check the adapter with `lsusb`, reconnect it, load its supported driver, then repeat `ip -br link show type can`. |
| CAN interface is `DOWN` | SocketCAN was not configured. | Let launcher preflight configure it, or follow `docs/can_discovery.md` for the correct bitrate and interface. |
| CAN interface reports `BUS-OFF` | Wrong bitrate, wiring/termination fault, unpowered device, or wrong role mapping. | Stop commands, check power, CAN-H/CAN-L, termination, and bitrate. Bring the interface down/up only after correcting the cause. |
| Launcher reports no Bunker frames | Wrong CAN interface, 500 kbit/s mismatch, wiring, or Bunker power. | Confirm adapter identity and inspect `candump "$BUNKER_CAN"`; do not bypass this check for a real drive. |
| PiPER starts but has no feedback | Wrong interface, 1 Mbit/s mismatch, firmware/driver mismatch, or arm not powered. | Confirm frames with `candump`, verify firmware selection, and use `--require-front-piper-frames` when strict startup is desired. |
| Camera is not listed | USB/power issue or unsupported connection. | Use a direct USB 3 connection, verify with `rs-enumerate-devices -s`, and check host kernel logs. |
| RealSense `resource busy`, timeout, or broken pipe | A second camera node owns the same device, bandwidth is exhausted, or USB reset occurred. | Stop duplicate RealSense processes, keep only the Nav-Man camera owner, reconnect the camera, and restart the camera/workflow. |
| Wrong camera opens | Serial was left blank with multiple cameras attached. | Set `FRONT_CAMERA_SERIAL` to the discovered D435i serial and restart. |
| Localization says database is missing | `/ros2_ws/maps/site.db` does not exist or the wrong volume/path is used. | Create it in mapping mode or copy the correct database into `/ros2_ws/maps`; confirm with `test -f`. |
| `/map` is missing | Camera RGB-D input, RTAB-Map, or mapping window failed. | Check `t2_mapping`, `/front_rgbd_image`, camera info, and depth topics before restarting RTAB-Map. |
| `map → base_link` fails | Missing map/odom publisher, duplicate TF owner, or timestamp problem. | Check `map → odom` and `odom → base_link` separately; stop duplicate robot-state, odometry, or RTAB-Map processes. |
| Nav2 is inactive | Map/TF/odom/sensor inputs are unavailable or lifecycle startup failed. | Inspect `t3_nav2`, verify required topics and transforms, then query Nav2 lifecycle nodes. |
| Robot does not move in the initial launch | This is expected: Nav2 starts in dry-run mode. | Complete validation first, then follow the full runbook’s explicit drive-mode procedure. |
| Robot moves unexpectedly | Duplicate command publisher or drive mode was enabled prematurely. | Use the emergency stop, stop the stack, inspect `/cmd_vel` publishers, and restore dry-run before debugging. |
| MoveIt model does not match the arm | Missing/stale joint feedback, wrong joint prefix, or incorrect mount transform. | Verify `/joint_states`, front/rear feedback topics, and URDF frames before planning. |
| Trajectory action is missing | Controller or trajectory bridge did not start. | Inspect `t5_moveit` and `t6_trajectory_bridge`; check `ros2 action list`. |
| ArUco marker is never visible | Wrong marker ID/size, camera topic/frame, lighting, distance, or detector failure. | Use marker ID `6`, size `0.06 m`, confirm the RGB stream, and inspect `t8_aruco`. |
| API `8892` or `8893` refuses connection | Corresponding tmux process exited or another process owns the port. | Inspect `t11_api_8892`/`t12_agent_8893`, then check `ss -ltnp | grep -E ':8892|:8893'`. |
| API says execution is denied | Safe default is active. | Validate hardware first; only then restart with `--allow-piper-motion`. Do not bypass the gate in code. |
| VLA reports no API key | Runtime credential is blank. | Set `DASHSCOPE_API_KEY` in the container environment without committing it. |
| A GitHub commit still has a red cross | Checks on that historical commit failed and are immutable. | Open the newest commit/run. Current stability is represented by the latest `main` check, not old history. |

## Rebuild after changing source files

Because the repository is mounted into the development container, rebuild the
affected packages before retrying:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to \
  bunker_slam_bringup bunker_dual_piper_nav2 bunker_autonomy \
  piper_x_aruco_wall_approach
source /ros2_ws/install/setup.bash
```

If generated state from an older build causes confusing import or package
errors, stop the container and rebuild the Docker image. Do not delete map
volumes as a generic troubleshooting step.

## Detect duplicate owners

Only one process should own each camera, hardware driver, robot description,
TF branch, RTAB-Map instance, Nav2 stack, or API port.

```bash
ros2 node list | sort
ros2 topic info /robot_description --verbose
ros2 topic info /odom --verbose
ros2 topic info /tf --verbose
ps -ef | grep -E 'realsense|bunker_base|agx_arm|robot_state_publisher|rtabmap|nav2|move_group' | grep -v grep
ss -ltnp | grep -E ':8892|:8893'
```

If duplicates exist, stop the old workflow cleanly and start one owner. Do not
kill unrelated ROS containers or host processes.

## Reset the ROS discovery cache

Use this only after confirming every shell has matching ROS environment values:

```bash
ros2 daemon stop
ros2 daemon start
ros2 node list
```

This refreshes command-line discovery; it does not repair missing hardware,
topics, or transforms.

## Information to include when requesting help

Provide:

1. Whether you used mapping or localization mode.
2. Whether the rear PiPER and optional ABot layer were enabled.
3. The exact command that failed and its complete output.
4. The name of the failing tmux window and its last 50 lines.
5. From the **host**, output from:

```bash
git rev-parse --short HEAD
docker compose -f docker/compose.yaml ps
docker logs --tail 100 bunker-nav-man
```

6. From a sourced shell **inside the container**, output from:

```bash
ip -details -statistics link show type can
rs-enumerate-devices -s
ros2 node list
ros2 topic list
ros2 action list
ros2 service list
```

Do not post API keys, access tokens, device credentials, or private network
addresses. Redact them before sharing logs.
