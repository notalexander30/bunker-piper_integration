#!/usr/bin/env bash
# Start the physical Bunker RGB-D mapping stack in the foreground.

set -Eeo pipefail

usage() {
  cat <<'EOF'
Usage: start_mapping.sh [launch_argument:=value ...]

Starts bunker_slam_bringup/slam_bringup.launch.py in mapping mode. The launch
process remains in the foreground so Ctrl-C reaches every managed ROS process.

Examples:
  start_mapping.sh
  start_mapping.sh use_rviz:=false configure_can:=false
  start_mapping.sh reset_database:=true database_path:=/absolute/path/map.db

reset_database defaults to false. Setting it true irreversibly clears the
selected RTAB-Map database before mapping.

Environment:
  ROS_UNDERLAY_SETUP   ROS underlay setup (default /opt/ros/humble/setup.bash)
  BUNKER_SLAM_ROS_WS   workspace root (then ROS_WS, then /ros2_ws)
  BUNKER_SLAM_SETUP    overlay setup (default WORKSPACE/install/setup.bash)
EOF
}

reset_argument_present=false
for argument in "$@"; do
  case "$argument" in
    -h|--help)
      usage
      exit 0
      ;;
    mode:=*)
      echo "error: start_mapping.sh fixes mode:=mapping" >&2
      exit 2
      ;;
    reset_database:=*)
      reset_argument_present=true
      ;;
  esac
done

underlay_setup="${ROS_UNDERLAY_SETUP:-/opt/ros/humble/setup.bash}"
slam_workspace="${BUNKER_SLAM_ROS_WS:-${ROS_WS:-/ros2_ws}}"
overlay_setup="${BUNKER_SLAM_SETUP:-${slam_workspace}/install/setup.bash}"

if [[ ! -r "$underlay_setup" ]]; then
  echo "error: ROS underlay setup not readable: $underlay_setup" >&2
  exit 1
fi
if [[ ! -r "$overlay_setup" ]]; then
  echo "error: workspace overlay not built/readable: $overlay_setup" >&2
  exit 1
fi

# shellcheck disable=SC1090
source "$underlay_setup"
# shellcheck disable=SC1090
source "$overlay_setup"
set -u

if ! ros2 pkg prefix bunker_slam_bringup >/dev/null 2>&1; then
  echo "error: bunker_slam_bringup is not discoverable in the overlay" >&2
  exit 1
fi

echo "Starting RGB-D mapping. Stop safely with Ctrl-C."
launch_arguments=(mode:=mapping)
if [[ "$reset_argument_present" == false ]]; then
  launch_arguments+=(reset_database:=false)
fi
exec ros2 launch bunker_slam_bringup slam_bringup.launch.py \
  "${launch_arguments[@]}" "$@"
