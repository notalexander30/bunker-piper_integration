"""One-pane Bunker/URDF/TF and optional H30 IMU bringup."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description():
    system = PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare('bunker_dual_piper_nav2'),
        'launch',
        'system_bringup.launch.py',
    ]))
    h30 = PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare('bunker_slam_bringup'),
        'launch',
        'h30_imu.launch.py',
    ]))

    configure_can = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([
                FindPackagePrefix('bunker_slam_bringup'),
                'lib',
                'bunker_slam_bringup',
                'configure_can.sh',
            ]),
            LaunchConfiguration('front_piper_can'),
            LaunchConfiguration('bunker_can'),
            PythonExpression([
                "'' if '", LaunchConfiguration('start_rear_piper_driver'),
                "' != 'true' else '", LaunchConfiguration('rear_piper_can'), "'",
            ]),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('configure_can')),
    )

    def robot_actions():
        return [
            IncludeLaunchDescription(
                system,
                launch_arguments={
                    'start_hardware_drivers': 'true',
                    'start_bunker_driver': 'true',
                    'bunker_can': LaunchConfiguration('bunker_can'),
                    'is_bunker_mini': 'true',
                    'bunker_control_rate': LaunchConfiguration('bunker_control_rate'),
                    'bunker_publish_tf': 'true',
                    'bunker_odom_topic_name': 'odom',
                    'start_piper_drivers': LaunchConfiguration('start_piper_drivers'),
                    'start_front_piper_driver': LaunchConfiguration(
                        'start_front_piper_driver'),
                    'start_rear_piper_driver': LaunchConfiguration(
                        'start_rear_piper_driver'),
                    'front_piper_can': LaunchConfiguration('front_piper_can'),
                    'rear_piper_can': LaunchConfiguration('rear_piper_can'),
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
                    'start_cameras': 'false',
                    'start_nav2': 'false',
                    'start_rviz': 'false',
                    'launch_rtabmap': 'false',
                    'launch_yolo': 'false',
                    'prefix_joint_states': LaunchConfiguration('prefix_joint_states'),
                    'publish_default_joint_states': LaunchConfiguration(
                        'publish_default_joint_states'),
                    'front_joint_states_topic': LaunchConfiguration(
                        'front_joint_states_topic'),
                    'rear_joint_states_topic': LaunchConfiguration(
                        'rear_joint_states_topic'),
                    'use_front_joint_feedback': LaunchConfiguration(
                        'use_front_joint_feedback'),
                    'use_rear_joint_feedback': LaunchConfiguration(
                        'use_rear_joint_feedback'),
                }.items(),
            ),
            IncludeLaunchDescription(
                h30,
                condition=IfCondition(LaunchConfiguration('start_h30_imu')),
                launch_arguments={
                    'serial_port': LaunchConfiguration('h30_serial_port'),
                    'baud_rate': LaunchConfiguration('h30_baud_rate'),
                    'frame_id': LaunchConfiguration('h30_frame_id'),
                }.items(),
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='h30_imu_static_tf',
                output='screen',
                arguments=[
                    '--x', LaunchConfiguration('h30_x'),
                    '--y', LaunchConfiguration('h30_y'),
                    '--z', LaunchConfiguration('h30_z'),
                    '--roll', LaunchConfiguration('h30_roll'),
                    '--pitch', LaunchConfiguration('h30_pitch'),
                    '--yaw', LaunchConfiguration('h30_yaw'),
                    '--frame-id', 'base_link',
                    '--child-frame-id', LaunchConfiguration('h30_frame_id'),
                ],
                condition=IfCondition(PythonExpression([
                    "'", LaunchConfiguration('start_h30_imu'), "' == 'true' and '",
                    LaunchConfiguration('publish_h30_static_tf'), "' == 'true'",
                ])),
            ),
        ]

    def after_can_setup(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason=(
                f'CAN configuration failed with exit code {event.returncode}; '
                'Bunker/robot startup cancelled.')))]
        return [
            LogInfo(msg=(
                'PiPER and Bunker CAN links configured; starting Bunker/TF '
                'and optional H30 IMU.')),
            *robot_actions(),
        ]

    can_exit_handler = RegisterEventHandler(OnProcessExit(
        target_action=configure_can,
        on_exit=after_can_setup,
    ))

    return LaunchDescription([
        DeclareLaunchArgument('arm_can', default_value='can2'),
        DeclareLaunchArgument('bunker_can', default_value='can4'),
        DeclareLaunchArgument('front_piper_can', default_value='can2'),
        DeclareLaunchArgument('rear_piper_can', default_value='can3'),
        DeclareLaunchArgument('bunker_control_rate', default_value='20'),
        DeclareLaunchArgument(
            'start_piper_drivers',
            default_value='false',
            choices=['true', 'false'],
            description='Start front/rear PiPER drivers so live feedback drives the URDF.',
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
        DeclareLaunchArgument('piper_effector_type', default_value='none'),
        DeclareLaunchArgument('front_piper_fw_version', default_value='v189'),
        DeclareLaunchArgument('rear_piper_fw_version', default_value='v189'),
        DeclareLaunchArgument(
            'front_piper_tcp_offset',
            default_value='[0.0, 0.0, 0.1425, 0.0, 0.0, 0.0]'),
        DeclareLaunchArgument(
            'rear_piper_tcp_offset',
            default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('prefix_joint_states', default_value='true'),
        DeclareLaunchArgument('publish_default_joint_states', default_value='true'),
        DeclareLaunchArgument(
            'front_joint_states_topic',
            default_value='/front_piper/feedback/joint_states'),
        DeclareLaunchArgument(
            'rear_joint_states_topic',
            default_value='/rear_piper/feedback/joint_states'),
        DeclareLaunchArgument('use_front_joint_feedback', default_value='true'),
        DeclareLaunchArgument('use_rear_joint_feedback', default_value='true'),
        DeclareLaunchArgument(
            'configure_can', default_value='true',
            choices=['true', 'false']),
        DeclareLaunchArgument(
            'h30_serial_port',
            default_value=(
                '/dev/serial/by-id/'
                'usb-WCH.CN_USB_Single_Serial_0003-if00')),
        DeclareLaunchArgument('h30_baud_rate', default_value='460800'),
        DeclareLaunchArgument('h30_frame_id', default_value='imu_link'),
        DeclareLaunchArgument('h30_x', default_value='0.0'),
        DeclareLaunchArgument('h30_y', default_value='0.0'),
        DeclareLaunchArgument('h30_z', default_value='0.0'),
        DeclareLaunchArgument('h30_roll', default_value='0.0'),
        DeclareLaunchArgument('h30_pitch', default_value='0.0'),
        DeclareLaunchArgument('h30_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'start_h30_imu',
            default_value='false',
            choices=['true', 'false'],
            description='Start the external H30/YESENSE IMU publisher.'),
        DeclareLaunchArgument(
            'publish_h30_static_tf',
            default_value='false',
            choices=['true', 'false']),
        DeclareLaunchArgument(
            'start_ekf',
            default_value='false',
            choices=['true', 'false'],
            description=(
                'Deprecated no-op in Terminal 1. EKF is not started from this '
                'launch; Bunker publishes raw /odom directly.'
            )),
        configure_can,
        can_exit_handler,
        GroupAction(
            actions=robot_actions(),
            condition=UnlessCondition(LaunchConfiguration('configure_can')),
        ),
    ])
