#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: kill_bunker_nav2_stack.sh [--force] [--dry-run]

Stops known ROS 2 processes used by the Bunker RTAB-Map/Nav2/self-exploration
stack inside this container. It does not stop Docker and it does not bring CAN
interfaces down.

Options:
  --dry-run   Print matching processes without killing them.
  --force     Send SIGKILL after SIGTERM for processes that are still alive.

Recommended before a clean restart:
  ros2 run bunker_slam_bringup kill_bunker_nav2_stack.sh
EOF
}

force=false
dry_run=false

for arg in "$@"; do
  case "$arg" in
    --force) force=true ;;
    --dry-run) dry_run=true ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $arg" >&2; usage >&2; exit 2 ;;
  esac
done

patterns=(
  # Standalone visual-only showcase. Must not run with real Nav2.
  'bunker_piper_3d_showcase\.launch\.py'
  'showcase_static_map_to_base_link'

  # Bunker/Nav2/RTAB-Map launch files.
  'terminal1_sensors\.launch\.py'
  'rtabmap_front_camera\.launch\.py'
  'nav2_bringup\.launch\.py'
  'frontier_exploration(_drive)?\.launch\.py'
  'frontier_exploration_ros2_bunker\.launch\.py'
  'aruco_landmark_exploration\.launch\.py'
  'landmark_navigator\.launch\.py'
  'nav2_status_monitor\.launch\.py'
  'nav2_manipulation_handoff\.launch\.py'
  'amcl_localization\.launch\.py'
  'amcl_bringup\.launch\.py'
  'ekf_h30\.launch\.py'
  'terminal2_robot\.launch\.py'
  'terminal3_mapping\.launch\.py'
  'system_bringup\.launch\.py'
  'hardware_drivers\.launch\.py'
  'description\.launch\.py'
  'dual_realsense\.launch\.py'
  'h30_imu\.launch\.py'

  # Core nodes commonly left behind after Ctrl-C or launch errors.
  'realsense2_camera_node'
  'rgbd_sync'
  'rgbdx_sync'
  'rgbd_odometry'
  'point_cloud_xyzrgb'
  'rtabmap_slam/rtabmap'
  'rtabmap_sync/rgbd_sync'
  'rtabmap'
  'map_server'
  'amcl'
  'controller_server'
  'planner_server'
  'behavior_server'
  'bt_navigator'
  'waypoint_follower'
  'velocity_smoother'
  'smoother_server'
  'lifecycle_manager_navigation'
  'cmd_vel_mux_node'
  'nav2_cmd_vel_safety_mux'
  'clicked_point_nav_goal'
  'nav2_arrival_manipulation_trigger'
  'nav2_manipulation_handoff'
  'nav2_status_monitor'
  'abotclaw_handoff_service\.py'
  'abotclaw_handoff_service'
  'abotclaw_manipulation_lifecycle\.sh'
  'abotclaw_manipulation_lifecycle_listener'
  'start_abotclaw_handoff_service\.sh'
  'start_iliyas_abotclaw_service\.sh'
  'front_piper_description_bridge\.py'
  'front_piper_description_bridge'
  'agent_server/server\.py'
  'front_piper_move_group_only'
  'front_piper_moveit_tf_publisher\.py'
  'front_piper_moveit_tf_publisher'
  'front_piper_moveit_rviz\.py'
  'front_piper_moveit_rviz'
  'front_piper_trajectory_bridge'
  'touch_marker_full_stack'
  'search_marker_node'
  'piper_touch_marker_api'
  'depth_route_monitor_node'
  'sensor_fusion_node'
  'safety_monitor_node'
  'operator_status_node'
  'data_logger_node'
  'frontier_explorer_node'
  'frontier_explorer'
  'frontier_debug_observer'
  'frontier_exploration_ctl'
  'aruco_landmark_node'
  'landmark_navigator_node'
  'bunker_base_node'
  'robot_state_publisher'
  'joint_state_prefixer'
  'joint_state_publisher'
  'offline_initial_pose_publisher'
  'static_transform_publisher'
  'ekf_node'
  'imu_bias_corrector\.py'
  'odom_imu_fusion_metrics_tui\.py'
  'yesense_node_publisher'
  'rviz2'

  # PiPER helper/preset processes used in this workspace.
  'piper_x_joint_preset'
  'piper_navigation_pose'
  'agx_arm_ctrl_single'
)

self_pid="$$"

collect_pids() {
  local pattern pid command
  for pattern in "${patterns[@]}"; do
    while IFS= read -r line; do
      [[ -n "$line" ]] || continue
      pid="${line%% *}"
      command="${line#* }"
      [[ "$pid" == "$self_pid" ]] && continue
      [[ "$command" == *"kill_bunker_nav2_stack.sh"* ]] && continue
      printf '%s\t%s\n' "$pid" "$command"
    done < <(pgrep -af "$pattern" || true)
  done | sort -n -u
}

matches="$(collect_pids)"

if [[ -z "$matches" ]]; then
  echo "No known Bunker/Nav2 stack processes are running."
  exit 0
fi

echo "Matched processes:"
echo "$matches"

if [[ "$dry_run" == true ]]; then
  echo "Dry-run only; no processes killed."
  exit 0
fi

echo "$matches" | awk '{print $1}' | xargs -r kill -TERM
sleep 2

remaining="$(collect_pids)"
if [[ -n "$remaining" && "$force" == true ]]; then
  echo "Force-killing remaining processes:"
  echo "$remaining"
  echo "$remaining" | awk '{print $1}' | xargs -r kill -KILL
  sleep 1
  remaining="$(collect_pids)"
fi

if [[ -n "$remaining" ]]; then
  echo "Some processes are still running. Re-run with --force if needed:"
  echo "$remaining"
  exit 1
fi

echo "Bunker/Nav2 stack processes stopped."
