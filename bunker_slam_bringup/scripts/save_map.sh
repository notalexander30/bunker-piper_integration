#!/usr/bin/env bash
# Save the latest /map OccupancyGrid using the package's atomic writer.

set -Eeo pipefail

usage() {
  cat <<'EOF'
Usage: save_map.sh [OUTPUT_PREFIX] [save_occupancy_grid.py options]

Writes OUTPUT_PREFIX.pgm and OUTPUT_PREFIX.yaml. If OUTPUT_PREFIX is omitted,
the default is WORKSPACE/src/bunker_slam_bringup/maps/bunker_map.

Examples:
  save_map.sh
  save_map.sh /data/maps/warehouse --timeout 60
  save_map.sh ./maps/test --map-topic /map --occupied-thresh 0.65

Environment:
  ROS_UNDERLAY_SETUP   ROS underlay setup (default /opt/ros/humble/setup.bash)
  BUNKER_SLAM_ROS_WS   workspace root (then ROS_WS, then /ros2_ws)
  BUNKER_SLAM_SETUP    overlay setup (default WORKSPACE/install/setup.bash)
  BUNKER_MAP_PREFIX    default output prefix
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

script_directory="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
underlay_setup="${ROS_UNDERLAY_SETUP:-/opt/ros/humble/setup.bash}"
slam_workspace="${BUNKER_SLAM_ROS_WS:-${ROS_WS:-/ros2_ws}}"
overlay_setup="${BUNKER_SLAM_SETUP:-${slam_workspace}/install/setup.bash}"
default_prefix="${BUNKER_MAP_PREFIX:-${slam_workspace}/src/bunker_slam_bringup/maps/bunker_map}"

if [[ ! -r "$underlay_setup" ]]; then
  echo "error: ROS underlay setup not readable: $underlay_setup" >&2
  exit 1
fi
if [[ ! -r "$overlay_setup" ]]; then
  echo "error: workspace overlay not built/readable: $overlay_setup" >&2
  exit 1
fi
if [[ ! -r "${script_directory}/save_occupancy_grid.py" ]]; then
  echo "error: save_occupancy_grid.py is missing beside this script" >&2
  exit 1
fi

# shellcheck disable=SC1090
source "$underlay_setup"
# shellcheck disable=SC1090
source "$overlay_setup"
set -u

output_prefix="$default_prefix"
if [[ $# -gt 0 && "$1" != --* ]]; then
  output_prefix="$1"
  shift
fi

exec python3 "${script_directory}/save_occupancy_grid.py" \
  "$output_prefix" "$@"
