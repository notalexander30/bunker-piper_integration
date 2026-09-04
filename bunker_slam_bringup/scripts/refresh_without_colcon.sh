#!/usr/bin/env bash
# Refresh edited launch/config/URDF/RViz/script files into the active install
# tree without running colcon. Use this after normal file edits, then restart
# the affected ROS launch.

set -Eeuo pipefail

workspace="${ROS2_WS:-/ros2_ws}"
if [[ ! -d "$workspace/src" && -d "$(pwd)/src" ]]; then
  workspace="$(pwd)"
fi

dual_src="$workspace/src/bunker_dual_piper_nav2"
slam_src="$workspace/src/bunker_slam_bringup"
dual_build="$workspace/build/bunker_dual_piper_nav2"
dual_install="$workspace/install/bunker_dual_piper_nav2/share/bunker_dual_piper_nav2"
slam_install_share="$workspace/install/bunker_slam_bringup/share/bunker_slam_bringup"
slam_install_lib="$workspace/install/bunker_slam_bringup/lib/bunker_slam_bringup"

require_dir() {
  local path="$1"
  if [[ ! -d "$path" ]]; then
    echo "error: missing directory: $path" >&2
    echo "Run one normal colcon build first, then use this refresh script." >&2
    exit 1
  fi
}

link_file() {
  local src="$1" dst="$2"
  if [[ -f "$src" ]]; then
    mkdir -p "$(dirname "$dst")"
    if [[ -e "$dst" || -L "$dst" ]]; then
      rm -f "$dst"
    fi
    ln -s "$src" "$dst"
  fi
}

link_dir() {
  local src="$1" dst="$2"
  if [[ -d "$src" ]]; then
    if [[ -e "$dst" || -L "$dst" ]]; then
      rm -rf "$dst"
    fi
    mkdir -p "$dst"
    shopt -s dotglob nullglob
    for entry in "$src"/*; do
      case "$entry" in
        *__pycache__|*.pyc) continue ;;
      esac
      ln -s "$entry" "$dst/$(basename "$entry")"
    done
    shopt -u dotglob nullglob
  fi
}

require_dir "$dual_src"
require_dir "$slam_src"
require_dir "$dual_build"
require_dir "$dual_install"
require_dir "$slam_install_share"
require_dir "$slam_install_lib"

echo "Linking bunker_dual_piper_nav2 share files directly to source..."
for dir in launch config rviz urdf scripts behavior_trees; do
  link_dir "$dual_src/$dir" "$dual_build/$dir"
  link_dir "$dual_src/$dir" "$dual_install/$dir"
done
for file in \
  package.xml README.md CODEX_IMPLEMENTATION_PROMPT.md CODEX_UPDATED_URDF_PROMPT.md \
  THIRD_PARTY_NOTICES.md LICENSE PIPER_LICENSE.txt; do
  link_file "$dual_src/$file" "$dual_build/$file"
  link_file "$dual_src/$file" "$dual_install/$file"
done

echo "Linking bunker_slam_bringup share files directly to source..."
for dir in config launch maps rviz urdf; do
  link_dir "$slam_src/$dir" "$slam_install_share/$dir"
done
for file in \
  LICENSE README.md NAV2_FULL_SYSTEM_STARTUP.md NAV_MAN_INTEGRATION_STARTUP.md \
  SELF_EXPLORATION_STARTUP.md ARUCO_LANDMARK_EXPLORATION.md UPDATED_NAVIGATION_PIPELINE.md \
  ODOM_IMU_FUSION_METRICS.md; do
  link_file "$slam_src/$file" "$slam_install_share/$file"
done

echo "Linking bunker_slam_bringup executable scripts directly to source..."
mkdir -p "$slam_install_lib"
shopt -s nullglob
for script in "$slam_src"/scripts/*; do
  [[ -f "$script" ]] || continue
  case "$script" in
    *__pycache__*|*.pyc) continue ;;
  esac
  link_file "$script" "$slam_install_lib/$(basename "$script")"
done
shopt -u nullglob
chmod +x "$slam_install_lib"/* 2>/dev/null || true

echo "Regenerating generated dual-PiPER URDF..."
set +u
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"
set -u
if command -v xacro >/dev/null 2>&1; then
  xacro "$dual_src/urdf/bunker_dual_piper_d435i.urdf.xacro" \
    > "$dual_src/urdf/bunker_dual_piper_d435i.generated.urdf"
else
  echo "warning: xacro not found; skipped generated URDF refresh" >&2
fi

echo
echo "Refresh complete."
echo "Restart the affected ROS launch. Running nodes do not live-reload files."
echo "Still run colcon build after setup.py/package.xml/CMakeLists.txt changes or new entry points."
