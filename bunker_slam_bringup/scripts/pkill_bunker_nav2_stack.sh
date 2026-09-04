#!/usr/bin/env bash
set -u

# Plain pkill cleanup for the Bunker RTAB-Map/Nav2/full-system stack.
# This intentionally does not kill tmux, does not bring CAN links down, and
# does not stop Docker.

echo "[pkill] stopping Bunker/Nav2/RTAB-Map/camera/RViz/PiPER processes..."

launch_patterns=(
  'terminal1_sensors.launch.py'
  'terminal2_robot.launch.py'
  'terminal3_mapping.launch.py'
  'system_bringup.launch.py'
  'hardware_drivers.launch.py'
  'description.launch.py'
  'dual_realsense.launch.py'
  'bunker_piper_3d_showcase.launch.py'
  'showcase_static_map_to_base_link'
  'rtabmap_front_camera.launch.py'
  'rtabmap.launch.py'
  'nav2_bringup.launch.py'
  'nav2_rtabmap.launch.py'
  'frontier_exploration.launch.py'
  'frontier_exploration_drive.launch.py'
  'frontier_exploration_ros2_bunker.launch.py'
  'aruco_landmark_exploration.launch.py'
  'landmark_navigator.launch.py'
  'nav2_status_monitor.launch.py'
  'nav2_manipulation_handoff.launch.py'
  'amcl_localization.launch.py'
  'amcl_bringup.launch.py'
  'ekf_h30.launch.py'
  'ekf.launch.py'
  'h30_imu.launch.py'
  'robot_tf.launch.py'
  'realsense.launch.py'
  'camera_only_rviz.launch.py'
  'camera_mapping_views.launch.py'
  'minimal_rgbd_mapping.launch.py'
  'standalone_camera_pipeline.launch.py'
  'standalone_rgbd_yolo_mapping.launch.py'
  'standalone_yolo26.launch.py'
  'standalone_door_yolov8.launch.py'
  'front_door_yolov8.launch.py'
  'door_segmentation_demo.launch.py'
  'navigation_memory_bringup.launch.py'
  'robot_yolo_memory.launch.py'
  'semantic_yolo_demo.launch.py'
  'door_semantic_memory_demo.launch.py'
  'find_trash_can_demo.launch.py'
  'find_trash_can_d435i.launch.py'
  'find_door_d435i.launch.py'
)

node_patterns=(
  # Cameras and RGBD/RTAB-Map mapping.
  'realsense2_camera_node'
  'rgbd_sync'
  'rgbdx_sync'
  'rgbd_odometry'
  'point_cloud_xyzrgb'
  'rtabmap'
  'rtabmapviz'
  'wait_for_navigation_inputs.sh'
  'wait_for_corrected_imu.py'
  'save_occupancy_grid.py'
  'save_map.sh'

  # Nav2 localization and navigation.
  'map_server'
  'amcl'
  'controller_server'
  'planner_server'
  'behavior_server'
  'bt_navigator'
  'waypoint_follower'
  'velocity_smoother'
  'smoother_server'
  'lifecycle_manager'
  'lifecycle_manager_navigation'
  'lifecycle_manager_localization'

  # Bunker autonomy, safety, goals, handoff, and exploration helpers.
  'cmd_vel_mux_node'
  'nav2_cmd_vel_safety_mux'
  'clicked_point_nav_goal'
  'nav2_arrival_manipulation_trigger'
  'nav2_manipulation_handoff'
  'nav2_status_monitor'
  'abotclaw_handoff_service.py'
  'abotclaw_handoff_service'
  'abotclaw_manipulation_lifecycle.sh'
  'abotclaw_manipulation_lifecycle_listener'
  'start_abotclaw_handoff_service.sh'
  'start_iliyas_abotclaw_service.sh'
  'front_piper_description_bridge.py'
  'front_piper_description_bridge'
  'agent_server/server.py'
  'front_piper_move_group_only'
  'front_piper_moveit_tf_publisher.py'
  'front_piper_moveit_tf_publisher'
  'front_piper_moveit_rviz.py'
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
  'landmark_navigator'
  'search_behavior_node'
  'vlm_goal_monitor_node'
  'simple_vlm_node'
  'd435i_calibration_node'
  'realsense_usb_recover'

  # Robot hardware, description, TF, IMU, and EKF.
  'bunker_base_node'
  'robot_state_publisher'
  'joint_state_prefixer'
  'joint_state_publisher'
  'offline_initial_pose_publisher'
  'static_transform_publisher'
  'ekf_node'
  'imu_bias_corrector.py'
  'yesense_node_publisher'
  'odom_imu_fusion_metrics_tui.py'

  # PiPER drivers and initialization helpers.
  'piper_x_joint_preset'
  'piper_navigation_pose'
  'agx_arm_ctrl_single'
  'agx_arm_ctrl_single_node'

  # YOLO/object/memory side nodes that create topics in the full workflow.
  'detector_tracker_node'
  'target_mux_node'
  'depth_geometry_memory_node'
  'semantic_memory_node'

  # Visualization.
  'rviz2'
)

pkill_patterns() {
  local signal="$1"
  shift
  local pattern
  for pattern in "$@"; do
    pkill "-${signal}" -f "${pattern}" 2>/dev/null || true
  done
}

pkill_patterns TERM "${launch_patterns[@]}"
pkill_patterns TERM "${node_patterns[@]}"

sleep 2

echo "[pkill] force-killing leftovers if any..."
pkill_patterns KILL "${launch_patterns[@]}"
pkill_patterns KILL "${node_patterns[@]}"

echo "[pkill] restarting ROS 2 daemon to clear stale topic cache..."
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

echo "[pkill] remaining ROS graph:"
ros2 node list 2>/dev/null || true
ros2 topic list 2>/dev/null || true

echo "[pkill] done. Only /parameter_events and /rosout may remain; those are normal ROS 2 topics."
