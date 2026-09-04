# Nav2 to Manipulation Docker Handoff

The navigation Docker publishes one continuous ROS 2 topic. The manipulation
Docker should only listen to this topic and start its own script when a new
door-manipulation request appears.

## Shared topic

Topic:

```bash
/navigation_manipulation/progress
```

Type:

```bash
std_msgs/msg/String
```

Payload is JSON. Example while navigation is still active:

```json
{"progress":"navigation","event":"navigation_running","task":"door_manipulation","request_id":"","goal_id":"","arrived_at_door":false,"manipulation_status":"idle"}
```

Example when Nav2 reaches the door and manipulation should start:

```json
{"progress":"manipulation_requested","event":"door_landmark_arrived","task":"door_manipulation","request_id":"door_manipulation:door","goal_id":"door","arrived_at_door":true,"manipulation_status":"idle"}
```

The listener should trigger once per new `request_id` when:

```text
progress == "manipulation_requested"
```

## Start in the navigation Docker

In `trystan-bunker-navigation`, source ROS and start the handoff publisher:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

ros2 launch bunker_autonomy nav2_manipulation_handoff.launch.py
```

This listens for the existing continuous door-arrival bool:

```bash
/door_navigation/arrived
```

When that topic changes to `data: true`, the handoff topic changes to
`manipulation_requested` and keeps publishing that state continuously.

## Listener skeleton for the manipulation Docker

Use the same ROS domain and network as the navigation Docker. Then run a
listener like this:

```python
#!/usr/bin/env python3
import json
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


COMMANDS = {
    "door_manipulation": [
        "bash",
        "-lc",
        "cd /workspace/ABot-Claw-piperX && ./start_door_manipulation.sh",
    ],
}


class ManipulationListener(Node):
    def __init__(self):
        super().__init__("manipulation_handoff_listener")
        self.last_request_id = ""
        self.status_pub = self.create_publisher(
            String, "/manipulation_task/progress", 10)
        self.create_subscription(
            String,
            "/navigation_manipulation/progress",
            self.callback,
            10,
        )

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def callback(self, msg):
        data = json.loads(msg.data)
        if data.get("progress") != "manipulation_requested":
            return

        request_id = data.get("request_id", "")
        if not request_id or request_id == self.last_request_id:
            return

        self.last_request_id = request_id
        task = data.get("task", "door_manipulation")
        command = COMMANDS.get(task)
        if command is None:
            self.publish_status("failed")
            self.get_logger().error(f"No command configured for task={task}")
            return

        self.publish_status("running")
        result = subprocess.run(command, check=False)
        self.publish_status("succeeded" if result.returncode == 0 else "failed")


def main():
    rclpy.init()
    node = ManipulationListener()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
```

Replace `./start_door_manipulation.sh` with the real command in the
manipulation Docker.

## Manual test

In the navigation Docker, fake a door arrival:

```bash
ros2 topic pub --once /door_navigation/arrived std_msgs/msg/Bool \
  "{data: true}"
```

Watch the shared topic:

```bash
ros2 topic echo /navigation_manipulation/progress
```

The manipulation Docker can report progress back:

```bash
ros2 topic pub --once /manipulation_task/progress std_msgs/msg/String \
  "{data: running}"

ros2 topic pub --once /manipulation_task/progress std_msgs/msg/String \
  "{data: succeeded}"
```

The navigation Docker will keep publishing the updated progress on the same
shared `/navigation_manipulation/progress` topic.

## Stop Stale ABotClaw Nodes

If the manipulation Docker leaves old ABotClaw lifecycle, robot-description, or
MoveIt processes running, clean them before starting a new test.

Run this inside the ABotClaw Docker:

```bash
bash /workspace/ABot-Claw-piperX/robot_layer/arm_piper_x/agent_server/stop_abotclaw_stale_nodes.sh
```

If that script is not mounted there yet, use this direct pkill block:

```bash
pkill -TERM -f 'abotclaw_handoff_service.py' || true
pkill -TERM -f 'manipulation_task_lifecycle_listener.py' || true
pkill -TERM -f 'abotclaw_manipulation_lifecycle.sh' || true
pkill -TERM -f 'start_abotclaw_handoff_service.sh' || true
pkill -TERM -f 'start_manipulation_task_listener.sh' || true
pkill -TERM -f 'front_piper_description_bridge.py' || true
pkill -TERM -f 'front_piper_move_group_only.py' || true
pkill -TERM -f 'touch_marker_full_stack.launch.py' || true
pkill -TERM -f 'piper_touch_marker_api.py' || true
pkill -TERM -f 'piper_x_control_gate.py' || true
pkill -TERM -f 'aruco_ros/single' || true
pkill -TERM -f 'wall_approach_node' || true
pkill -TERM -f 'search_marker_node' || true
pkill -TERM -f 'moveit_ros_move_group/move_group.*__ns:=/front_piper' || true
pkill -TERM -f 'move_group.*front_piper' || true
pkill -TERM -f 'robot_state_publisher.*front_piper' || true

sleep 1

pkill -KILL -f 'abotclaw_handoff_service.py' || true
pkill -KILL -f 'manipulation_task_lifecycle_listener.py' || true
pkill -KILL -f 'front_piper_description_bridge.py' || true
pkill -KILL -f 'front_piper_move_group_only.py' || true
pkill -KILL -f 'move_group.*front_piper' || true
pkill -KILL -f 'robot_state_publisher.*front_piper' || true
```

Do not use a broad `pkill -f robot_state_publisher` during Nav2 testing. The
main navigation container also uses `robot_state_publisher` for the Bunker
URDF/TF tree.
