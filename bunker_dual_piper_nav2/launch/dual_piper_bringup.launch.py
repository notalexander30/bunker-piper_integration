"""Bring up both PiPER arms, the combined URDF, joint-state bridge and RViz."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def initial_pose_node(name, namespace, joint1):
    return Node(
        package='bunker_autonomy',
        executable='piper_navigation_pose',
        name=name,
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_initial_pose')),
        parameters=[{
            'joint_names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
            'joint_positions': [
                joint1, 0.0, 0.0, 0.0, 0.0, 0.0,
            ],
            'feedback_topic': f'/{namespace}/feedback/joint_states',
            'command_topic': f'/{namespace}/control/joint_states',
            'arm_enable_service': f'/{namespace}/enable_agx_arm',
            'control_gate_service': f'/{namespace}/control_enable',
            'trigger_on_start': True,
            'allow_motion': ParameterValue(
                LaunchConfiguration('allow_piper_motion'),
                value_type=bool,
            ),
        }],
    )


def generate_launch_description():
    package_share = FindPackageShare('bunker_dual_piper_nav2')
    hardware_source = PythonLaunchDescriptionSource(
        PathJoinSubstitution([package_share, 'launch', 'hardware_drivers.launch.py'])
    )
    description_source = PythonLaunchDescriptionSource(
        PathJoinSubstitution([package_share, 'launch', 'description.launch.py'])
    )
    default_rviz_config = PathJoinSubstitution(
        [package_share, 'rviz', 'bunker_description_2d.rviz']
    )

    configure_can = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([
                FindPackagePrefix('bunker_dual_piper_nav2'),
                'share',
                'bunker_dual_piper_nav2',
                'scripts',
                'configure_dual_piper_can.sh',
            ]),
            LaunchConfiguration('front_piper_can'),
            LaunchConfiguration('rear_piper_can'),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('configure_piper_can')),
    )

    def live_piper_actions():
        return [
            IncludeLaunchDescription(
                hardware_source,
                launch_arguments={
                    'start_bunker_driver': 'false',
                    'start_piper_drivers': 'true',
                    'front_piper_can': LaunchConfiguration('front_piper_can'),
                    'rear_piper_can': LaunchConfiguration('rear_piper_can'),
                    'piper_auto_enable': 'false',
                    'piper_control_enabled': LaunchConfiguration('piper_control_enabled'),
                    'piper_speed_percent': LaunchConfiguration('piper_speed_percent'),
                    'piper_effector_type': LaunchConfiguration('piper_effector_type'),
                }.items(),
            ),
            TimerAction(
                period=4.0,
                actions=[
                    initial_pose_node(
                        'front_piper_initial_pose',
                        'front_piper',
                        LaunchConfiguration('front_piper_parked_joint1'),
                    ),
                    initial_pose_node(
                        'rear_piper_initial_pose',
                        'rear_piper',
                        LaunchConfiguration('rear_piper_parked_joint1'),
                    ),
                ],
            ),
        ]

    def after_can_setup(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason=(
                f'Dual PiPER CAN configuration failed with exit code '
                f'{event.returncode}; front/rear PiPER drivers cancelled.')))]
        return [
            LogInfo(msg='Dual PiPER CAN configured; starting front/rear drivers.'),
            *live_piper_actions(),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('front_piper_can', default_value='can2'),
        DeclareLaunchArgument('rear_piper_can', default_value='can3'),
        DeclareLaunchArgument(
            'configure_piper_can',
            default_value='true',
            choices=['true', 'false'],
            description='Configure front/rear PiPER CAN links at 1 Mbit/s before starting drivers.',
        ),
        DeclareLaunchArgument('piper_speed_percent', default_value='25'),
        DeclareLaunchArgument('piper_effector_type', default_value='none'),
        DeclareLaunchArgument(
            'piper_control_enabled',
            default_value='true',
            description=(
                'Keep the PiPER external control gate open by default.'
            ),
        ),
        DeclareLaunchArgument('front_piper_parked_joint1', default_value='-1.6'),
        DeclareLaunchArgument('rear_piper_parked_joint1', default_value='1.6'),
        DeclareLaunchArgument(
            'run_initial_pose',
            default_value='true',
            description='Optionally command both arms to the documented initial pose.',
        ),
        DeclareLaunchArgument(
            'allow_piper_motion',
            default_value='true',
            description='Second explicit gate for physical arm motion.',
        ),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config_file', default_value=default_rviz_config),
        LogInfo(msg=(
            'Dual PiPER bringup uses front_piper_can:=can2 and '
            'rear_piper_can:=can3 by default. It commands the low-speed '
            'front/rear initial pose unless launch arguments override it.')),
        configure_can,
        RegisterEventHandler(
            OnProcessExit(target_action=configure_can, on_exit=after_can_setup)
        ),
        GroupAction(
            actions=live_piper_actions(),
            condition=UnlessCondition(LaunchConfiguration('configure_piper_can')),
        ),
        IncludeLaunchDescription(
            description_source,
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'prefix_joint_states': 'true',
                'publish_default_joint_states': 'false',
                'front_joint_states_topic': '/front_piper/feedback/joint_states',
                'rear_joint_states_topic': '/rear_piper/feedback/joint_states',
                'use_front_joint_feedback': 'true',
                'use_rear_joint_feedback': 'true',
                'front_piper_parked_joint1': LaunchConfiguration(
                    'front_piper_parked_joint1'),
                'rear_piper_parked_joint1': LaunchConfiguration(
                    'rear_piper_parked_joint1'),
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='dual_piper_rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_rviz')),
            arguments=['-d', LaunchConfiguration('rviz_config_file')],
        ),
    ])
