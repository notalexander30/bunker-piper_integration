cmake_minimum_required(VERSION 3.8)
project(bunker_slam_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config descriptions launch maps rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/check_rgbd_sync.py
    scripts/check_tf_ownership.py
    scripts/configure_can.sh
    scripts/resolve_can_interfaces.sh
    scripts/imu_bias_corrector.py
    scripts/odom_imu_fusion_metrics_tui.py
    scripts/kill_bunker_nav2_stack.sh
    scripts/pkill_bunker_nav2_stack.sh
    scripts/pkill_piper_description_topics.sh
    scripts/publish_locked_arm_state.py
    scripts/refresh_without_colcon.sh
    scripts/save_occupancy_grid.py
    scripts/save_map.sh
    scripts/start_mapping.sh
    scripts/front_piper_description_bridge.py
    scripts/front_piper_integrated_moveit_model.py
    scripts/front_piper_move_group_only.py
    scripts/front_piper_moveit_rviz.py
    scripts/front_piper_moveit_tf_publisher.py
    scripts/front_piper_trajectory_bridge.py
    scripts/start_iliyas_abot_in_trystan.py
    scripts/start_standalone_yolo_tmux.sh
    scripts/verify_slam.sh
    scripts/watchdog_sources_monitor.sh
    scripts/wait_for_navigation_inputs.sh
    scripts/wait_for_corrected_imu.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    scripts/front_piper_moveit.srdf
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    LICENSE
    README.md
    NAV2_FULL_SYSTEM_STARTUP.md
    NAV_MAN_INTEGRATION_STARTUP.md
    ODOM_IMU_FUSION_METRICS.md
    SELF_EXPLORATION_STARTUP.md
    ARUCO_LANDMARK_EXPLORATION.md
    UPDATED_NAVIGATION_PIPELINE.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
