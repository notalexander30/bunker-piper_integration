#!/usr/bin/env bash
set -u

# Targeted cleanup for PiPER/URDF/TF leftovers.
# Use this when ros2 topic list still shows:
#   /front_piper/feedback/joint_states
#   /rear_piper/feedback/joint_states
#   /front_piper/robot_description
#   /rear_piper/robot_description
#   /robot_description
#   /joint_states
#   /tf
#   /tf_static

echo "[pkill-piper] stopping PiPER drivers, robot description, joint-state and TF publishers..."

launch_patterns=(
  'terminal1_sensors.launch.py'
  'terminal2_robot.launch.py'
  'system_bringup.launch.py'
  'hardware_drivers.launch.py'
  'dual_piper_bringup.launch.py'
  'description.launch.py'
  'robot_tf.launch.py'
  'bunker_piper_3d_showcase.launch.py'
)

node_patterns=(
  'agx_arm_ctrl_single'
  'agx_arm_ctrl_single_node'
  'piper_navigation_pose'
  'piper_x_joint_preset'
  'robot_state_publisher'
  'joint_state_prefixer'
  'joint_state_publisher'
  'offline_initial_pose_publisher'
  'static_transform_publisher'
  'front_piper_description_bridge.py'
  'front_piper_description_bridge'
)

pkill_patterns() {
  local signal="$1"
  shift
  local pattern
  for pattern in "$@"; do
    pkill "-${signal}" -f "${pattern}" 2>/dev/null || true
  done
}

echo "[pkill-piper] current ROS graph before cleanup:"
ros2 node list 2>/dev/null || true
ros2 topic list 2>/dev/null | grep -E '^/(front_piper|rear_piper|robot_description|joint_states|tf|tf_static)' || true

pkill_patterns TERM "${launch_patterns[@]}"
pkill_patterns TERM "${node_patterns[@]}"

sleep 2

echo "[pkill-piper] force-killing leftovers if any..."
pkill_patterns KILL "${launch_patterns[@]}"
pkill_patterns KILL "${node_patterns[@]}"

echo "[pkill-piper] restarting ROS 2 daemon to clear stale topic cache..."
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

echo "[pkill-piper] remaining Piper/description/TF topics:"
ros2 topic list 2>/dev/null | grep -E '^/(front_piper|rear_piper|robot_description|joint_states|tf|tf_static)' || true

echo "[pkill-piper] full remaining ROS graph:"
ros2 node list 2>/dev/null || true
ros2 topic list 2>/dev/null || true

echo "[pkill-piper] done. /parameter_events and /rosout are normal ROS 2 topics."
