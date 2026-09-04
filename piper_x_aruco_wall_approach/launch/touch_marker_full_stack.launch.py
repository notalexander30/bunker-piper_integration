from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    execute_allowed = LaunchConfiguration("execute_allowed")
    calibration_file = LaunchConfiguration("calibration_file")
    point_cloud_topic = LaunchConfiguration("point_cloud_topic")
    marker_id = LaunchConfiguration("marker_id")
    marker_size = LaunchConfiguration("marker_size")
    api_port = LaunchConfiguration("api_port")
    api_host = LaunchConfiguration("api_host")
    pre_clearance = LaunchConfiguration("pre_clearance")
    final_clearance = LaunchConfiguration("final_clearance")
    retract_after = LaunchConfiguration("retract_after")
    prefer_elbow_motion = LaunchConfiguration("prefer_elbow_motion")
    goal_orientation_tolerance = LaunchConfiguration("goal_orientation_tolerance")
    piper_namespace = LaunchConfiguration("piper_namespace")
    use_piper_motion_stack = LaunchConfiguration("use_piper_motion_stack")
    use_aruco_detector = LaunchConfiguration("use_aruco_detector")
    use_wall_approach_node = LaunchConfiguration("use_wall_approach_node")
    use_search_marker_node = LaunchConfiguration("use_search_marker_node")
    use_marker_api = LaunchConfiguration("use_marker_api")
    can_port = LaunchConfiguration("can_port")
    auto_enable = LaunchConfiguration("auto_enable")
    follow = LaunchConfiguration("follow")
    auto_control_gate = LaunchConfiguration("auto_control_gate")
    use_piper_control_gate = LaunchConfiguration("use_piper_control_gate")
    pub_rate = LaunchConfiguration("pub_rate")
    enable_color = LaunchConfiguration("enable_color")
    enable_depth = LaunchConfiguration("enable_depth")
    color_profile = LaunchConfiguration("color_profile")
    align_depth = LaunchConfiguration("align_depth")
    pointcloud = LaunchConfiguration("pointcloud")
    use_realsense = LaunchConfiguration("use_realsense")
    realsense_config_file = LaunchConfiguration("realsense_config_file")
    use_handeye_tf_publisher = LaunchConfiguration("use_handeye_tf_publisher")
    use_front_piper_joint_state_adapter = LaunchConfiguration("use_front_piper_joint_state_adapter")
    integrated_joint_state_topic = LaunchConfiguration("integrated_joint_state_topic")
    front_piper_joint_prefix = LaunchConfiguration("front_piper_joint_prefix")
    camera_image_topic = LaunchConfiguration("camera_image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    camera_root_frame = LaunchConfiguration("camera_root_frame")
    camera_optical_frame = LaunchConfiguration("camera_optical_frame")
    handeye_parent_frame = LaunchConfiguration("handeye_parent_frame")
    marker_timeout = LaunchConfiguration("marker_timeout")
    point_cloud_timeout = LaunchConfiguration("point_cloud_timeout")
    home_pose_file = LaunchConfiguration("home_pose_file")
    previous_pose_file = LaunchConfiguration("previous_pose_file")
    found_marker_pose_file = LaunchConfiguration("found_marker_pose_file")
    joint_state_topic = LaunchConfiguration("joint_state_topic")
    marker_api_joint_state_topic = LaunchConfiguration("marker_api_joint_state_topic")
    joint_state_timeout = LaunchConfiguration("joint_state_timeout")
    trajectory_action = LaunchConfiguration("trajectory_action")
    enable_service = LaunchConfiguration("enable_service")
    controller_manager_service = LaunchConfiguration("controller_manager_service")
    require_physical_hardware = LaunchConfiguration("require_physical_hardware")
    use_integrated_moveit_semantic_bridge = LaunchConfiguration("use_integrated_moveit_semantic_bridge")
    integrated_semantic_topic = LaunchConfiguration("integrated_semantic_topic")
    control_topic = LaunchConfiguration("control_topic")
    robot_description_topic = LaunchConfiguration("robot_description_topic")
    robot_description_semantic_topic = LaunchConfiguration("robot_description_semantic_topic")

    package_share = FindPackageShare("piper_x_aruco_wall_approach")

    return LaunchDescription([
        DeclareLaunchArgument("execute_allowed", default_value="false"),
        DeclareLaunchArgument(
            "calibration_file",
            default_value="/home/dase-hw101/handeye/config/piper_x_d435i_eye_in_hand.json",
        ),
        DeclareLaunchArgument(
            "point_cloud_topic",
            default_value="/front_camera/depth/color/points",
        ),
        DeclareLaunchArgument("marker_id", default_value="6"),
        DeclareLaunchArgument("marker_size", default_value="0.06"),
        DeclareLaunchArgument("api_port", default_value="8892"),
        DeclareLaunchArgument("api_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("pre_clearance", default_value="0.05"),
        DeclareLaunchArgument("final_clearance", default_value="0.005"),
        DeclareLaunchArgument("retract_after", default_value="true"),
        DeclareLaunchArgument("prefer_elbow_motion", default_value="true"),
        DeclareLaunchArgument("goal_orientation_tolerance", default_value="0.35"),
        DeclareLaunchArgument("piper_namespace", default_value="front_piper"),
        # Trystan's integrated stack owns the drivers and the namespaced
        # front MoveIt instance.  Opt into the standalone stack only when
        # running this package outside the Bunker system.
        DeclareLaunchArgument("use_piper_motion_stack", default_value="false"),
        DeclareLaunchArgument("use_aruco_detector", default_value="true"),
        DeclareLaunchArgument("use_wall_approach_node", default_value="true"),
        DeclareLaunchArgument("use_search_marker_node", default_value="true"),
        DeclareLaunchArgument("use_marker_api", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("can_port", default_value="can2"),
        DeclareLaunchArgument("auto_enable", default_value="true"),
        DeclareLaunchArgument("follow", default_value="true"),
        DeclareLaunchArgument("auto_control_gate", default_value="false"),
        DeclareLaunchArgument("use_piper_control_gate", default_value="false"),
        DeclareLaunchArgument("pub_rate", default_value="80"),
        DeclareLaunchArgument("enable_color", default_value="true"),
        DeclareLaunchArgument("enable_depth", default_value="true"),
        DeclareLaunchArgument("color_width", default_value="1280"),
        DeclareLaunchArgument("color_height", default_value="720"),
        DeclareLaunchArgument("color_profile", default_value="1280x720x30"),
        DeclareLaunchArgument("align_depth", default_value="true"),
        DeclareLaunchArgument("pointcloud", default_value="true"),
        DeclareLaunchArgument("use_realsense", default_value="false"),
        DeclareLaunchArgument("use_handeye_tf_publisher", default_value="false"),
        # The integrated front driver already publishes raw feedback on
        # /front_piper/feedback/joint_states.  Enable this fallback only when
        # that topic is absent and only /joint_states is available.
        DeclareLaunchArgument("use_front_piper_joint_state_adapter", default_value="false"),
        DeclareLaunchArgument("use_integrated_moveit_semantic_bridge", default_value="false"),
        DeclareLaunchArgument(
            "integrated_semantic_topic",
            default_value="/front_piper/integrated_robot_description_semantic",
        ),
        DeclareLaunchArgument("integrated_joint_state_topic", default_value="/joint_states"),
        DeclareLaunchArgument("front_piper_joint_prefix", default_value="front_piper_"),
        DeclareLaunchArgument(
            "realsense_config_file",
            default_value=PathJoinSubstitution([
                package_share,
                "config",
                "realsense_d435i_piper_x.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "camera_image_topic",
            default_value="/front_camera/color/image_raw",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/front_camera/color/camera_info",
        ),
        DeclareLaunchArgument("camera_root_frame", default_value="front_camera_link"),
        DeclareLaunchArgument(
            "camera_optical_frame",
            default_value="front_camera_color_optical_frame",
        ),
        DeclareLaunchArgument("handeye_parent_frame", default_value="front_piper_flange_link"),
        DeclareLaunchArgument("marker_timeout", default_value="1.0"),
        DeclareLaunchArgument("point_cloud_timeout", default_value="2.0"),
        # MoveIt/search consume the integrated prefixed state on /joint_states,
        # while the physical API gate must read the raw front-arm feedback.
        DeclareLaunchArgument("joint_state_topic", default_value="/joint_states"),
        DeclareLaunchArgument(
            "marker_api_joint_state_topic",
            default_value="/front_piper/feedback/joint_states",
        ),
        DeclareLaunchArgument("control_topic", default_value="/front_piper/control/joint_states"),
        # Trystan's active MoveIt model is the prefixed full-system URDF.  The
        # semantic bridge below supplies its matching front-arm SRDF.
        DeclareLaunchArgument("robot_description_topic", default_value="/robot_description"),
        DeclareLaunchArgument(
            "robot_description_semantic_topic",
            default_value="/front_piper/robot_description_semantic",
        ),
        DeclareLaunchArgument(
            "trajectory_action",
            default_value="/front_piper/arm_controller/follow_joint_trajectory",
        ),
        DeclareLaunchArgument("enable_service", default_value="/front_piper/enable_agx_arm"),
        DeclareLaunchArgument(
            "controller_manager_service",
            default_value="/front_piper/controller_manager/list_hardware_components",
        ),
        DeclareLaunchArgument(
            "require_physical_hardware",
            # The integrated AgileX driver exposes feedback and an action
            # bridge but does not run a ros2_control controller manager.
            default_value="false",
        ),
        DeclareLaunchArgument("joint_state_timeout", default_value="2.5"),
        DeclareLaunchArgument(
            "home_pose_file",
            default_value=EnvironmentVariable(
                "PIPER_X_HOME_POSE_FILE",
                default_value="/ros2_ws/config/piper_x_manipulation_pose.yaml",
            ),
        ),
        DeclareLaunchArgument(
            "previous_pose_file",
            default_value=EnvironmentVariable(
                "PIPER_X_PREVIOUS_POSE_FILE",
                default_value="/ros2_ws/config/piper_x_previous_pose.yaml",
            ),
        ),
        DeclareLaunchArgument(
            "found_marker_pose_file",
            default_value=EnvironmentVariable(
                "PIPER_X_FOUND_MARKER_POSE_FILE",
                default_value="/ros2_ws/config/piper_x_found_marker_pose.yaml",
            ),
        ),
        SetEnvironmentVariable("PIPER_TOUCH_ALLOW_EXECUTION", execute_allowed),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("agx_arm_ctrl"),
                    "launch",
                    "start_single_agx_arm_moveit.launch.py",
                ])
            ]),
            launch_arguments={
                "can_port": can_port,
                "namespace": piper_namespace,
                "arm_type": "piper_x",
                "effector_type": "agx_gripper",
                "fw_version": "v189",
                "auto_enable": auto_enable,
                "follow": follow,
                "feedback_topic": joint_state_topic,
                "control_topic": control_topic,
                "auto_control_gate": auto_control_gate,
                "pub_rate": pub_rate,
                "use_rviz": LaunchConfiguration("use_rviz"),
                "tcp_offset": "[0.0, 0.0, 0.1425, 0.0, 0.0, 0.0]",
            }.items(),
            condition=IfCondition(use_piper_motion_stack),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("realsense2_camera"),
                    "launch",
                    "rs_launch.py",
                ])
            ]),
            launch_arguments={
                "camera_name": "front_camera",
                "enable_color": enable_color,
                "enable_depth": enable_depth,
                "rgb_camera.color_profile": color_profile,
                "align_depth.enable": align_depth,
                "pointcloud.enable": pointcloud,
                "pointcloud__neon_.enable": pointcloud,
                "config_file": realsense_config_file,
            }.items(),
            condition=IfCondition(use_realsense),
        ),
        Node(
            package="piper_x_aruco_wall_approach",
            executable="publish_integrated_moveit_semantic.py",
            name="front_piper_integrated_moveit_semantic",
            output="screen",
            parameters=[{"topic": integrated_semantic_topic}],
            condition=IfCondition(use_integrated_moveit_semantic_bridge),
        ),
        Node(
            package="piper_x_aruco_wall_approach",
            executable="front_piper_joint_state_adapter.py",
            name="front_piper_joint_state_adapter",
            output="screen",
            parameters=[{
                "input_topic": integrated_joint_state_topic,
                "output_topic": joint_state_topic,
                "source_prefix": front_piper_joint_prefix,
                "output_joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            }],
            condition=IfCondition(use_front_piper_joint_state_adapter),
        ),
        Node(
            package="piper_x_aruco_wall_approach",
            executable="piper_x_control_gate.py",
            name="piper_x_control_gate",
            output="screen",
            parameters=[{
                "status_topics": [
                    "/front_piper/arm_controller/follow_joint_trajectory/_action/status",
                    "/front_piper/execute_trajectory/_action/status",
                    "/front_piper/move_action/_action/status",
                ],
                "gate_service_name": "/front_piper/control_enable",
                "hold_open_after_active_s": 4.0,
                "open_on_startup": True,
                "close_on_startup": False,
                "close_when_idle": False,
            }],
            condition=IfCondition(use_piper_control_gate),
        ),
        Node(
            package="piper_x_aruco_wall_approach",
            executable="publish_handeye_tf.py",
            name="piper_x_handeye_tf_publisher",
            output="screen",
            arguments=[
                "--calibration",
                calibration_file,
                "--parent-frame",
                handeye_parent_frame,
                "--camera-root",
                camera_root_frame,
                "--optical-frame",
                camera_optical_frame,
            ],
            condition=IfCondition(use_handeye_tf_publisher),
        ),
        Node(
            package="aruco_ros",
            executable="single",
            name="aruco_single",
            output="screen",
            parameters=[{
                "marker_id": ParameterValue(marker_id, value_type=int),
                "marker_size": ParameterValue(marker_size, value_type=float),
                "image_is_rectified": True,
                "reference_frame": "base_link",
                "camera_frame": camera_optical_frame,
                "marker_frame": "aruco_marker_frame",
            }],
            remappings=[
                ("/image", camera_image_topic),
                ("/camera_info", camera_info_topic),
            ],
            condition=IfCondition(use_aruco_detector),
        ),
        Node(
            package="piper_x_aruco_wall_approach",
            executable="wall_approach_node",
            name="wall_approach_node",
            output="screen",
            parameters=[
                PathJoinSubstitution([package_share, "config", "wall_approach.yaml"]),
                {
                    "execute": False,
                    "clearance": ParameterValue(pre_clearance, value_type=float),
                    "final_clearance": ParameterValue(final_clearance, value_type=float),
                    "retract_after": ParameterValue(retract_after, value_type=bool),
                    "prefer_elbow_motion": ParameterValue(prefer_elbow_motion, value_type=bool),
                    "goal_orientation_tolerance": ParameterValue(
                        goal_orientation_tolerance,
                        value_type=float,
                    ),
                    "move_group_namespace": piper_namespace,
                    "point_cloud_topic": point_cloud_topic,
                },
            ],
            remappings=[
                ("joint_states", joint_state_topic),
                ("robot_description", robot_description_topic),
                ("robot_description_semantic", robot_description_semantic_topic),
            ],
            condition=IfCondition(use_wall_approach_node),
        ),
        Node(
            package="piper_x_aruco_wall_approach",
            executable="search_marker_node",
            name="search_marker_node",
            output="screen",
            parameters=[
                PathJoinSubstitution([package_share, "config", "piper_x_search_poses.yaml"]),
                {
                    "aruco_pose_topic": "/aruco_single/pose",
                    "marker_id": ParameterValue(marker_id, value_type=int),
                    "joint_state_topic": joint_state_topic,
                    "move_group_namespace": piper_namespace,
                    "controller_manager_service": controller_manager_service,
                    "require_physical_hardware": ParameterValue(
                        require_physical_hardware, value_type=bool
                    ),
                },
            ],
            remappings=[
                ("joint_states", joint_state_topic),
                ("robot_description", robot_description_topic),
                ("robot_description_semantic", robot_description_semantic_topic),
            ],
            condition=IfCondition(use_search_marker_node),
        ),
        Node(
            package="piper_x_aruco_wall_approach",
            executable="piper_touch_marker_api.py",
            name="piper_touch_marker_api",
            output="screen",
            arguments=[
                "--host",
                api_host,
                "--port",
                api_port,
                "--marker-id",
                marker_id,
                "--marker-size-m",
                marker_size,
                "--point-cloud-topic",
                point_cloud_topic,
                "--marker-timeout-s",
                marker_timeout,
                "--point-cloud-timeout-s",
                point_cloud_timeout,
                "--home-pose-file",
                home_pose_file,
                "--previous-pose-file",
                previous_pose_file,
                "--found-marker-pose-file",
                found_marker_pose_file,
                "--joint-state-topic",
                marker_api_joint_state_topic,
                "--joint-state-timeout-s",
                joint_state_timeout,
                "--trajectory-action",
                trajectory_action,
                "--enable-service",
                enable_service,
                "--controller-manager-service",
                controller_manager_service,
                "--command-joint-prefix",
                "front_piper_",
            ],
            condition=IfCondition(use_marker_api),
        ),
    ])
