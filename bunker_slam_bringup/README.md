# Bunker Mini Nav2 Bringup

Use `NAV2_FULL_SYSTEM_STARTUP.md` as the only maintained physical Nav2 startup
runbook for this package.

Use `NAV_MAN_INTEGRATION_STARTUP.md` for the current one-container Nav-Man
startup. It runs the front PiPER MoveIt/ArUco/API stack manually inside
`bunker-nav-man` and does not use the old automatic handoff service.

The old split-runbook helper scripts were removed so operators do not follow
stale procedures. The current workflow
uses:

- hardware startup with `terminal1_sensors.launch.py`
- RTAB-Map mapping/localization with `rtabmap_front_camera.launch.py`
- Nav2 with `nav2_bringup.launch.py`
- RViz configs from `rviz/`

Build after changes:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select bunker_slam_bringup bunker_dual_piper_nav2 bunker_autonomy --symlink-install
source /ros2_ws/install/setup.bash
```
