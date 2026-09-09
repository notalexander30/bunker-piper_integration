"""Terminal 1 combined hardware bringup.

Starts the front RealSense, optional rear RealSense, optional ArUco detector,
Bunker hardware/TF, optional H30 IMU, and PiPER drivers. EKF and YOLO are
intentionally not part of this launch.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera = PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare('bunker_dual_piper_nav2'),
        'launch',
        'dual_realsense.launch.py',
    ]))
    robot = PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare('bunker_slam_bringup'),
        'launch',
        'terminal2_robot.launch.py',
    ]))

    def initial_pose_node(
            name, namespace, joint1, close_control_gate_on_finish, driver_enabled_arg):
        return Node(
            package='bunker_autonomy',
            executable='piper_navigation_pose',
            name=name,
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('run_piper_initial_pose'), "' == 'true' and '",
                LaunchConfiguration(driver_enabled_arg), "' == 'true'",
            ])),
            parameters=[{
                'joint_names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
                'joint_positions': [joint1, 0.0, 0.0, 0.0, 0.0, 0.0],
                'feedback_topic': f'/{namespace}/feedback/joint_states',
                'command_topic': f'/{namespace}/control/joint_states',
                'arm_enable_service': f'/{namespace}/enable_agx_arm',
                'control_gate_service': f'/{namespace}/control_enable',
                'trigger_on_start': True,
                'allow_motion': ParameterValue(
                    LaunchConfiguration('allow_piper_motion'),
                    value_type=bool,
                ),
                'close_control_gate_on_finish': close_control_gate_on_finish,
            }],
        )

    def camera_launch_action(condition):
        return IncludeLaunchDescription(
            camera,
            condition=condition,
            launch_arguments={
                'launch_front_camera': LaunchConfiguration('launch_front_camera'),
                'launch_rear_camera': LaunchConfiguration('launch_rear_camera'),
                'front_camera_serial': LaunchConfiguration('front_camera_serial'),
                'rear_camera_serial': LaunchConfiguration('rear_camera_serial'),
            }.items(),
        )

    reset_front_camera = ExecuteProcess(
        cmd=['ros2', 'run', 'bunker_autonomy', 'realsense_usb_recover'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('reset_front_camera_usb')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('arm_can', default_value=''),
        DeclareLaunchArgument('bunker_can', default_value=''),
        DeclareLaunchArgument('front_piper_can', default_value=''),
        DeclareLaunchArgument('rear_piper_can', default_value=''),
        DeclareLaunchArgument('bunker_control_rate', default_value='20'),
        DeclareLaunchArgument(
            'start_piper_drivers',
            default_value='true',
            choices=['true', 'false'],
            description='Start front/rear PiPER drivers so feedback updates the URDF.',
        ),
        DeclareLaunchArgument(
            'start_front_piper_driver',
            default_value='true',
            choices=['true', 'false'],
            description='Start the front PiPER driver when start_piper_drivers is true.',
        ),
        DeclareLaunchArgument(
            'start_rear_piper_driver',
            default_value='true',
            choices=['true', 'false'],
            description='Start the rear PiPER driver when start_piper_drivers is true.',
        ),
        DeclareLaunchArgument(
            'piper_control_enabled',
            default_value='true',
            choices=['true', 'false'],
            description='Keep the PiPER external control gate open by default.',
        ),
        DeclareLaunchArgument('piper_speed_percent', default_value='25'),
        DeclareLaunchArgument(
            'piper_effector_type',
            default_value='agx_gripper',
            description=(
                'PiPER end effector. agx_gripper matches the front-only '
                'Illiyas MoveIt URDF/SRDF and controller model.'
            ),
        ),
        DeclareLaunchArgument('front_piper_fw_version', default_value='v189'),
        DeclareLaunchArgument('rear_piper_fw_version', default_value='v189'),
        DeclareLaunchArgument(
            'enable_front_piper_control_gate',
            default_value='false',
            choices=['true', 'false'],
            description=(
                'Deprecated manual gate helper; kept as a no-op launch argument.'
            ),
        ),
        DeclareLaunchArgument(
            'front_piper_tcp_offset',
            default_value='[0.0, 0.0, 0.1425, 0.0, 0.0, 0.0]',
            description='TCP offset shared with the front-only Illiyas MoveIt model.',
        ),
        DeclareLaunchArgument(
            'rear_piper_tcp_offset',
            default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
        ),
        DeclareLaunchArgument('front_piper_parked_joint1', default_value='-1.6'),
        DeclareLaunchArgument('rear_piper_parked_joint1', default_value='1.6'),
        DeclareLaunchArgument(
            'run_piper_initial_pose',
            default_value='true',
            choices=['true', 'false'],
            description='Optionally command both PiPER arms to the documented initial pose.',
        ),
        DeclareLaunchArgument(
            'allow_piper_motion',
            default_value='true',
            choices=['true', 'false'],
            description='Second explicit gate required before physical arm motion.',
        ),
        DeclareLaunchArgument(
            'configure_can', default_value='true',
            choices=['true', 'false']),
        DeclareLaunchArgument(
            'h30_serial_port',
            default_value=''),
        DeclareLaunchArgument('h30_baud_rate', default_value='460800'),
        DeclareLaunchArgument('h30_frame_id', default_value='imu_link'),
        DeclareLaunchArgument(
            'start_h30_imu', default_value='false',
            choices=['true', 'false']),
        DeclareLaunchArgument(
            'start_ekf', default_value='false',
            choices=['true', 'false']),
        DeclareLaunchArgument(
            'start_aruco_detector', default_value='false',
            choices=['true', 'false']),
        DeclareLaunchArgument(
            'launch_front_camera',
            default_value='true',
            choices=['true', 'false']),
        DeclareLaunchArgument(
            'launch_rear_camera',
            default_value='false',
            choices=['true', 'false']),
        DeclareLaunchArgument(
            'front_camera_serial',
            default_value='',
            description='Front D435i serial; old known camera, used for mapping.'),
        DeclareLaunchArgument(
            'rear_camera_serial',
            default_value='',
            description='Rear D435i serial; newer second camera.'),
        DeclareLaunchArgument(
            'reset_front_camera_usb',
            default_value='true',
            choices=['true', 'false'],
            description='Reset the D435i USB device before launching RealSense.',
        ),
        DeclareLaunchArgument('aruco_image_topic', default_value='/front_camera/color/image_raw'),
        DeclareLaunchArgument('aruco_camera_info_topic', default_value='/front_camera/color/camera_info'),
        DeclareLaunchArgument('aruco_dictionary', default_value='DICT_4X4_50'),
        DeclareLaunchArgument('aruco_target_marker_ids', default_value='6'),
        DeclareLaunchArgument('aruco_marker_size_m', default_value='0.05'),
        DeclareLaunchArgument('aruco_required_seen_count', default_value='3'),
        DeclareLaunchArgument('aruco_save_path', default_value='/ros2_ws/maps/aruco_landmarks.json'),
        DeclareLaunchArgument(
            'aruco_publish_debug_image', default_value='true',
            choices=['true', 'false']),
        LogInfo(msg=(
            'Terminal 1 combined hardware startup: front D435i by default, '
            'optional rear D435i/ArUco/H30, Bunker/CAN and robot TF. '
            'YOLO is not started.')),
        reset_front_camera,
        RegisterEventHandler(
            OnProcessExit(
                target_action=reset_front_camera,
                on_exit=[camera_launch_action(None)],
            ),
            condition=IfCondition(LaunchConfiguration('reset_front_camera_usb')),
        ),
        camera_launch_action(UnlessCondition(LaunchConfiguration('reset_front_camera_usb'))),
        IncludeLaunchDescription(
            robot,
            launch_arguments={
                'arm_can': LaunchConfiguration('arm_can'),
                'bunker_can': LaunchConfiguration('bunker_can'),
                'front_piper_can': LaunchConfiguration('front_piper_can'),
                'rear_piper_can': LaunchConfiguration('rear_piper_can'),
                'bunker_control_rate': LaunchConfiguration('bunker_control_rate'),
                'start_piper_drivers': LaunchConfiguration('start_piper_drivers'),
                'start_front_piper_driver': LaunchConfiguration(
                    'start_front_piper_driver'),
                'start_rear_piper_driver': LaunchConfiguration(
                    'start_rear_piper_driver'),
                'piper_control_enabled': LaunchConfiguration('piper_control_enabled'),
                'piper_speed_percent': LaunchConfiguration('piper_speed_percent'),
                'piper_effector_type': LaunchConfiguration('piper_effector_type'),
                'front_piper_fw_version': LaunchConfiguration(
                    'front_piper_fw_version'),
                'rear_piper_fw_version': LaunchConfiguration(
                    'rear_piper_fw_version'),
                'front_piper_tcp_offset': LaunchConfiguration(
                    'front_piper_tcp_offset'),
                'rear_piper_tcp_offset': LaunchConfiguration(
                    'rear_piper_tcp_offset'),
                'prefix_joint_states': 'true',
                'publish_default_joint_states': 'false',
                'front_joint_states_topic': '/front_piper/feedback/joint_states',
                'rear_joint_states_topic': '/rear_piper/feedback/joint_states',
                'use_front_joint_feedback': 'true',
                'use_rear_joint_feedback': 'true',
                'configure_can': LaunchConfiguration('configure_can'),
                'h30_serial_port': LaunchConfiguration('h30_serial_port'),
                'h30_baud_rate': LaunchConfiguration('h30_baud_rate'),
                'h30_frame_id': LaunchConfiguration('h30_frame_id'),
                'start_h30_imu': LaunchConfiguration('start_h30_imu'),
                'publish_h30_static_tf': LaunchConfiguration('start_h30_imu'),
                'start_ekf': LaunchConfiguration('start_ekf'),
            }.items(),
        ),
        TimerAction(
            period=4.0,
            actions=[
                initial_pose_node(
                    'front_piper_initial_pose',
                    'front_piper',
                    LaunchConfiguration('front_piper_parked_joint1'),
                    False,
                    'start_front_piper_driver',
                ),
                initial_pose_node(
                    'rear_piper_initial_pose',
                    'rear_piper',
                    LaunchConfiguration('rear_piper_parked_joint1'),
                    True,
                    'start_rear_piper_driver',
                ),
            ],
        ),
        Node(
            package='bunker_autonomy',
            executable='aruco_landmark_node',
            name='aruco_landmarks',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_aruco_detector')),
            parameters=[{
                'image_topic': LaunchConfiguration('aruco_image_topic'),
                'camera_info_topic': LaunchConfiguration('aruco_camera_info_topic'),
                'aruco_dictionary': LaunchConfiguration('aruco_dictionary'),
                'target_marker_ids': ParameterValue(
                    LaunchConfiguration('aruco_target_marker_ids'), value_type=str),
                'marker_size_m': ParameterValue(
                    LaunchConfiguration('aruco_marker_size_m'), value_type=float),
                'required_seen_count': ParameterValue(
                    LaunchConfiguration('aruco_required_seen_count'), value_type=int),
                'save_path': LaunchConfiguration('aruco_save_path'),
                'publish_debug_image': ParameterValue(
                    LaunchConfiguration('aruco_publish_debug_image'), value_type=bool),
            }],
        ),
    ])
