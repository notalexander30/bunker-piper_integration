# PiPER-X joint preset runner

This runner is for the rear PiPER-X arm on `can3`. It is separate from the front PiPER arm on `can2`.

## CAN setup

Run inside the Docker container:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

ip link set can3 down 2>/dev/null || true
ip link set can3 type can bitrate 1000000 restart-ms 100
ip link set can3 up
ip -details -statistics link show can3
```

Check frames:

```bash
candump can3
```

## Launch driver only

Use this first. It should publish PiPER-X feedback but should not send motion commands.

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

ros2 launch bunker_autonomy piper_x_joint_preset_bringup.launch.py \
  can_port:=can3 \
  run_preset:=false \
  allow_motion:=false
```

Check feedback:

```bash
ros2 topic hz /piper_x/feedback/joint_states
ros2 service list | grep piper_x
```

Expected important interfaces:

| Interface | Type | Meaning |
| --- | --- | --- |
| `/piper_x/feedback/joint_states` | topic | Live PiPER-X joint feedback |
| `/piper_x/control/joint_states` | topic | Full six-joint command input |
| `/piper_x/enable_agx_arm` | service | Enable/disable the arm |
| `/piper_x/control_enable` | service | Gate that allows/rejects control commands |
| `/piper_x/move_home` | service | Driver-provided home command |
| `/piper_x/emergency_stop` | service | Driver emergency stop |

## Edit the preset

Edit:

```bash
nano /ros2_ws/src/bunker_autonomy/config/piper_x_joint_preset.yaml
```

Use this compact six-joint pose format:

```yaml
rear: [1.6, 0.0, 0.0, 0.0, 0.0, 0.0]
hold_sec: 1.0
```

Rules:

- Values are radians in `[joint1, joint2, joint3, joint4, joint5, joint6]` order.
- The node publishes all six joints together.
- For first real-arm testing, keep `speed_percent:=5` and verify the target is physically safe before setting `allow_motion:=true`.

After editing the source workspace config, rebuild or use `--symlink-install` so
the installed launch file sees the updated YAML:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select bunker_autonomy
source /ros2_ws/install/setup.bash
```

## Run the preset

Only run this when the arm area is clear and E-stop is reachable:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

ros2 launch bunker_autonomy piper_x_joint_preset_bringup.launch.py \
  can_port:=can3 \
  speed_percent:=5 \
  preset_file:=/ros2_ws/src/bunker_autonomy/config/piper_x_joint_preset.yaml \
  run_preset:=true \
  allow_motion:=true
```

The runner waits for complete feedback and services before sending anything. It opens `/piper_x/control_enable`, runs the preset, then closes the gate again.

## Stop stale PiPER-X nodes

```bash
pkill -f piper_x_joint_preset
pkill -f agx_arm_ctrl_single
```
