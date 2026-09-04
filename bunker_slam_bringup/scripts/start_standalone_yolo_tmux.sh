#!/usr/bin/env bash
set -Eeuo pipefail

# One debuggable ROS component per tmux window. Inside tmux, add windows to the
# current session; outside tmux, create and attach to a dedicated session.
setup_command="source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash"
session="${STANDALONE_YOLO_TMUX_SESSION:-standalone-yolo}"
database="${STANDALONE_YOLO_DATABASE:-/ros2_ws/vlm_results/maps/handheld_yolo26_debug.db}"
memory_file="${STANDALONE_YOLO_MEMORY:-/ros2_ws/vlm_results/semantic/handheld_yolo26_debug.json}"
weights="${STANDALONE_YOLO_WEIGHTS:-/ros2_ws/models/yolo26n.pt}"
print_only=false
detector_config="/ros2_ws/install/bunker_object_follower/share/bunker_object_follower/config/standalone_rgbd_yolo26.yaml"
rviz_config="/ros2_ws/install/bunker_slam_bringup/share/bunker_slam_bringup/rviz/standalone_rgbd_yolo_mapping.rviz"

usage() {
  cat <<'EOF'
Usage: start_standalone_yolo_tmux.sh [--session NAME] [--database PATH]
                                      [--memory PATH] [--weights PATH] [--print]

The RealSense driver must already be running. Existing named windows are kept,
so an individual component can be restarted without replacing the others.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --session) session="${2:?missing session name}"; shift 2 ;;
    --database) database="${2:?missing database path}"; shift 2 ;;
    --memory) memory_file="${2:?missing memory path}"; shift 2 ;;
    --weights) weights="${2:?missing weights path}"; shift 2 ;;
    --print) print_only=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

for value in "$session" "$database" "$memory_file" "$weights"; do
  if [[ "$value" == *"'"* || "$value" == *$'\n'* ]]; then
    echo "Session names and paths cannot contain quotes or newlines." >&2
    exit 2
  fi
done

inside_tmux=false
if [[ "$print_only" == true ]]; then
  :
elif [[ -n "${TMUX:-}" ]]; then
  inside_tmux=true
  session="$(tmux display-message -p '#S')"
elif ! tmux has-session -t "$session" 2>/dev/null; then
  tmux new-session -d -s "$session" -n shell
  tmux set-option -t "$session" remain-on-exit on
fi

window_exists() {
  tmux list-windows -t "$session" -F '#{window_name}' | grep -Fqx "$1"
}

add_window() {
  local name="$1"
  local command="$2"
  local complete
  if [[ "$print_only" == true ]]; then
    printf '\n# %s\n%s\n' "$name" "$command"
    return
  fi
  if window_exists "$name"; then
    printf 'Keeping existing window: %s:%s\n' "$session" "$name"
    return
  fi
  complete="${setup_command} && ${command}; exec bash"
  printf -v complete '%q' "$complete"
  tmux new-window -d -t "$session" -n "$name" "bash -lc ${complete}"
  printf 'Created window: %s:%s\n' "$session" "$name"
}

mapping_base="ros2 launch bunker_slam_bringup minimal_rgbd_mapping.launch.py database_path:=${database} reset_database:=false"
add_window rgbd-sync \
  "${mapping_base} start_rgbd_odometry:=false start_rgbd_sync:=true start_live_cloud:=false start_rtabmap:=false"
add_window rgbd-odom \
  "${mapping_base} start_rgbd_odometry:=true start_rgbd_sync:=false start_live_cloud:=false start_rtabmap:=false"
add_window rtabmap \
  "${mapping_base} start_rgbd_odometry:=false start_rgbd_sync:=false start_live_cloud:=false start_rtabmap:=true"
add_window live-cloud \
  "${mapping_base} start_rgbd_odometry:=false start_rgbd_sync:=false start_live_cloud:=true start_rtabmap:=false"
add_window yolo26 \
  "ros2 run bunker_object_follower detector_tracker_node --ros-args --params-file ${detector_config} -p weights_path:=${weights}"
add_window semantic \
  "ros2 launch semantic_memory semantic_memory.launch.py params_file:=${detector_config} memory_file:=${memory_file} rtabmap_database_path:=${database} yolo_detection_topic:=/standalone_yolo/local_target_status stationary_test_map:=false"
add_window rviz \
  "rviz2 -d ${rviz_config}"
add_window yolo-status \
  "ros2 topic echo /standalone_yolo/detector_status"
add_window odom-status \
  "ros2 topic echo /odom_info"

if [[ "$print_only" == false ]]; then
cat <<EOF

Standalone mapping and YOLO windows are ready in tmux session: ${session}
Database: ${database}
Semantic memory: ${memory_file}

Switch with Ctrl-b n / Ctrl-b p, or use Ctrl-b w for the window list.
Existing windows were preserved. No camera driver or motion node was started.
EOF
fi

if [[ "$print_only" == true ]]; then
  exit 0
elif [[ "$inside_tmux" == true ]]; then
  tmux select-window -t "${session}:rgbd-sync"
else
  exec tmux attach-session -t "$session"
fi
