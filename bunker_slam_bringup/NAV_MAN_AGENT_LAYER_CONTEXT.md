# Navigation-Manipulation Agent Layer Context

This file is context for a future agent layer that coordinates Bunker Nav2 landmark navigation with the front PiPER/ABotClaw manipulation layer.

## Goal

The agent layer should manage a repeatable task loop:

```text
home -> door
door reached -> request manipulation
manipulation complete -> door -> home
home reached -> ready for the next cycle
```

Nav2 itself should stay running. The agent should not kill and restart the Nav2 stack for each trip. It should start and stop navigation intent by sending goals, waiting for results, and canceling goals only when needed.

## Existing Navigation Layer

The landmark navigator is the named-goal layer above Nav2.

Inputs:

```text
/landmark_navigator/go_marker          std_msgs/msg/String
/landmark_navigator/go_home            std_srvs/srv/Trigger
/landmark_navigator/save_current_pose  std_msgs/msg/String
```

Known landmark names:

```text
home
door
```

Send a landmark goal:

```bash
ros2 topic pub --once /landmark_navigator/go_marker std_msgs/msg/String "{data: door}"
ros2 topic pub --once /landmark_navigator/go_marker std_msgs/msg/String "{data: home}"
```

The landmark file is:

```text
/ros2_ws/maps/manual_nav_landmarks.json
```

The agent should assume the `home` and `door` poses must already be saved and verified before autonomous cycling.

## Existing Door Arrival Handoff

When `/landmark_navigator` sends the robot to `door` and the Nav2 goal succeeds, it publishes:

```text
/door_navigation/arrived  std_msgs/msg/Bool
/home_navigation/arrived  std_msgs/msg/Bool
```

Expected data while no matching arrival has happened yet, or after a new goal starts:

```yaml
data: false
```

Expected data after the `door` Nav2 goal succeeds: `/door_navigation/arrived`
publishes 10 `true` messages at 2 Hz, then returns to `false`.

```yaml
data: true
```

Expected data after the `home` Nav2 goal succeeds: `/home_navigation/arrived`
publishes 10 `true` messages at 2 Hz, then returns to `false`.

```yaml
data: true
```

If `start_manipulation_trigger:=true` is enabled in the Nav2 launch, `/nav2_arrival_manipulation_trigger` watches the rising edge from `false` to `true` and publishes the manipulation start request once.

Manipulation start topics:

```text
/front_piper/task/start       std_msgs/msg/String JSON
/front_piper/task/start_bool  std_msgs/msg/Bool
```

Continuous progress topic:

```text
/navigation_manipulation/progress  std_msgs/msg/String JSON
```

Manipulation status input:

```text
/manipulation_task/progress  std_msgs/msg/String
```

Expected manipulation status values:

```text
running
succeeded
failed
```

The agent should treat `succeeded`, `done`, `finished`, `success`, or `manipulation_succeeded` as successful completion if it mirrors the existing progress node behavior.

## Intended Agent State Machine

The future agent layer should own the high-level task state:

```text
IDLE_AT_HOME
NAVIGATING_TO_DOOR
WAITING_FOR_DOOR_ARRIVAL
MANIPULATION_REQUESTED
MANIPULATION_RUNNING
NAVIGATING_TO_HOME
IDLE_AT_HOME
FAILED
```

Recommended behavior:

1. Start in `IDLE_AT_HOME`.
2. On task start, publish `door` to `/landmark_navigator/go_marker`.
3. Enter `NAVIGATING_TO_DOOR`.
4. Wait for `/door_navigation/arrived == true`.
5. Enter `MANIPULATION_REQUESTED`.
6. Let the existing handoff trigger or the agent publish `/front_piper/task/start`, but do not publish duplicate manipulation requests for the same door arrival.
7. Enter `MANIPULATION_RUNNING` when `/manipulation_task/progress` reports `running`.
8. When manipulation reports success, publish `home` to `/landmark_navigator/go_marker`.
9. Enter `NAVIGATING_TO_HOME`.
10. Wait for `/home_navigation/arrived == true`, then enter `IDLE_AT_HOME`.
11. On navigation failure, timeout, manipulation failure, or safety stop, enter `FAILED` and require an explicit recovery decision.

## Navigation Direction Context

The existing landmark navigator publishes navigation mode hints:

```text
/landmark_navigator/navigation_direction  std_msgs/msg/String
/landmark_navigator/active_camera         std_msgs/msg/String
```

Expected forward trip:

```text
navigation_direction: forward
active_camera: front_camera
```

Expected door-to-home reverse trip:

```text
navigation_direction: reverse
active_camera: rear_camera
```

The agent should treat these as navigation context, not as commands to the robot.

## Safety And Ownership Rules

The agent layer should not directly command the base or arm hardware.

Navigation control path:

```text
agent -> /landmark_navigator/go_marker -> /navigate_to_pose -> Nav2 controller -> /cmd_vel -> chassis
```

Current default: `nav2_bringup.launch.py` sends Nav2 velocity commands directly
to `/cmd_vel`. The older safety-mux route
`/nav2/cmd_vel_raw -> /cmd_vel_autonomy -> /nav2_cmd_vel_safety_mux -> /cmd_vel`
is optional and is enabled only when launching Nav2 with
`use_cmd_vel_mux:=true`.

Manipulation control path:

```text
agent or handoff trigger -> /front_piper/task/start -> iliyas-abot listener -> manipulation lifecycle
```

Safety rules:

```text
Keep Nav2 lifecycle nodes alive.
Do not restart Nav2 for every home/door cycle.
Do not send arm motion directly from the navigation agent.
Do not send another manipulation request while one request_id is active.
On manipulation failure, do not automatically drive away unless recovery policy is explicit.
On Nav2 failure or timeout, stop the task loop and report FAILED.
```

## Startup Assumptions

The full system startup should already have:

```text
ROS_DOMAIN_ID=173
ROS_LOCALHOST_ONLY=1
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
RTAB-Map localization or another valid map->odom owner
Bunker base driver owns odom->base_link with `start_ekf:=false`
Nav2 active
/landmark_navigator running
front and rear RealSense streams running
front PiPER manipulation listener running in iliyas-abot if manipulation is enabled
```

The agent layer should verify these before starting the loop:

```bash
ros2 node list | grep -E 'landmark_navigator|bt_navigator|controller_server'
ros2 topic echo /landmark_navigator/markers --once
ros2 topic info /front_piper/task/start
ros2 topic info /manipulation_task/progress
```

## Minimal First Version

A minimal first agent can implement:

```text
command: start_cycle
publish door goal
wait for door arrival
wait for manipulation succeeded
publish home goal
wait for generic home arrival, if available
otherwise stop after sending home and require operator confirmation
```

The minimal version should not attempt complex recovery. It should make failures visible and stop.
