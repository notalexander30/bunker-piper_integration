cmake_minimum_required(VERSION 3.8)
project(piper_x_aruco_wall_approach)

find_package(ament_cmake REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RunMarkerTask.srv"
  "srv/SearchMarker.srv"
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(wall_approach_node src/wall_approach_node.cpp)
target_compile_features(wall_approach_node PUBLIC cxx_std_17)
target_include_directories(wall_approach_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(wall_approach_node Eigen3::Eigen ${PCL_LIBRARIES})
ament_target_dependencies(wall_approach_node
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  pcl_conversions
  rclcpp
  sensor_msgs
  shape_msgs
  std_srvs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(wall_approach_node "${cpp_typesupport_target}")

add_executable(search_marker_node src/search_marker_node.cpp)
target_compile_features(search_marker_node PUBLIC cxx_std_17)
target_link_libraries(search_marker_node Eigen3::Eigen)
ament_target_dependencies(search_marker_node
  controller_manager_msgs
  geometry_msgs
  moveit_ros_planning_interface
  rclcpp
)
target_link_libraries(search_marker_node "${cpp_typesupport_target}")

install(TARGETS wall_approach_node search_marker_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/front_piper_joint_state_adapter.py
  scripts/piper_touch_marker_api.py
  scripts/piper_touch_marker_client.py
  scripts/piper_x_control_gate.py
  scripts/publish_integrated_moveit_semantic.py
  scripts/publish_handeye_tf.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_piper_touch_marker_api test/test_piper_touch_marker_api.py)
  ament_add_pytest_test(test_launch_files test/test_launch_files.py)
  ament_add_pytest_test(test_front_piper_joint_state_adapter test/test_front_piper_joint_state_adapter.py)
  ament_add_pytest_test(test_skill_contract test/test_skill_contract.py)
endif()

ament_package()
