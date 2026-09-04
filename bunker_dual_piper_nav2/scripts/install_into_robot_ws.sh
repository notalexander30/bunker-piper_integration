#!/usr/bin/env bash
set -Eeuo pipefail

SOURCE_PACKAGE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET_WS="${1:-/home/dase-orin/ros2_ws}"
TARGET_PACKAGE="${TARGET_WS}/src/bunker_dual_piper_nav2"

if [[ ! -f "${SOURCE_PACKAGE}/package.xml" ]]; then
  echo "ERROR: package.xml was not found beside this installer." >&2
  exit 2
fi
if [[ ! -d "${TARGET_WS}/src" ]]; then
  echo "ERROR: ROS workspace src directory does not exist: ${TARGET_WS}/src" >&2
  exit 2
fi

SOURCE_REAL="$(realpath "${SOURCE_PACKAGE}")"
TARGET_REAL="$(realpath -m "${TARGET_PACKAGE}")"
if [[ "${SOURCE_REAL}" != "${TARGET_REAL}" ]]; then
  mkdir -p "${TARGET_PACKAGE}"
  cp -a "${SOURCE_PACKAGE}/." "${TARGET_PACKAGE}/"
  echo "Installed package source into ${TARGET_PACKAGE}"
else
  echo "Package is already inside ${TARGET_WS}; no copy needed."
fi
chmod +x "${TARGET_PACKAGE}/scripts/"*.sh

source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
cd "${TARGET_WS}"
rosdep install --from-paths src --ignore-src --rosdistro "${ROS_DISTRO:-humble}" -r -y
colcon build --symlink-install --packages-select bunker_dual_piper_nav2

echo
echo "Build complete. Source it with:"
echo "  source ${TARGET_WS}/install/setup.bash"
echo "Then create your runtime configuration:"
echo "  cp ${TARGET_PACKAGE}/scripts/robot_system.env.example ${TARGET_PACKAGE}/scripts/robot_system.env"
