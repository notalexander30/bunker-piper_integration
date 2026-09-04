#!/usr/bin/env bash
# Rebuild and open the safe dual-PiPER 3-D orientation preview.
# Extra arguments are passed directly to the ROS launch command.
# ROS Humble setup scripts intentionally read optional unset trace variables,
# so do not enable `nounset` until after sourcing them.
set -Eeo pipefail

WORKSPACE="${ROBOT_WS:-/ros2_ws}"
PACKAGE="bunker_dual_piper_nav2"

# Keep this visualization-only session away from other domain-0 ROS traffic.
# Set URDF_PREVIEW_DOMAIN_ID if another private domain is preferred.
export ROS_DOMAIN_ID="${URDF_PREVIEW_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY=1

source /opt/ros/humble/setup.bash
cd "${WORKSPACE}"

# --base-paths avoids duplicate-name errors if this workspace contains backups.
colcon build --symlink-install --base-paths "src/${PACKAGE}" --packages-select "${PACKAGE}"
source "${WORKSPACE}/install/setup.bash"

exec ros2 launch "${PACKAGE}" urdf_camera_preview.launch.py "$@"
