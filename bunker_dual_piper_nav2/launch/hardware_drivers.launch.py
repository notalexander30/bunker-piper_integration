"""Optional hardware wrappers with an isolated autonomy velocity input."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def validate_bunker_can(context):
    if LaunchConfiguration('start_bunker_driver').perform(context).lower() != 'true':
        return []

    interface = LaunchConfiguration('bunker_can').perform(context).strip()
    if not interface or interface == 'UNSET_BUNKER_CAN':
        raise RuntimeError(
            'start_bunker_driver:=true requires bunker_can:=canX. '
            'Use a real SocketCAN interface name without angle brackets.'
        )
    return []


def validate_piper_can(context):
    if LaunchConfiguration('start_piper_drivers').perform(context).lower() != 'true':
        return []

    front_interface = LaunchConfiguration('front_piper_can').perform(context).strip()
    rear_interface = LaunchConfiguration('rear_piper_can').perform(context).strip()
    start_front = LaunchConfiguration('start_front_piper_driver').perform(
        context).lower() == 'true'
    start_rear = LaunchConfiguration('start_rear_piper_driver').perform(
        context).lower() == 'true'
    missing = []
    if start_front and (
            not front_interface or front_interface == 'UNSET_FRONT_CAN'):
        missing.append('front_piper_can')
    if start_rear and (
            not rear_interface or rear_interface == 'UNSET_REAR_CAN'):
        missing.append('rear_piper_can')
    if missing:
        raise RuntimeError(
            'start_piper_drivers:=true requires real PiPER CAN interfaces. '
            'Pass the discovered front_piper_can and rear_piper_can when each '
            'corresponding driver is enabled. Missing: '
            + ', '.join(missing)
        )
    return []


def generate_launch_description():
    bunker_source = PythonLaunchDescriptionSource(
        PathJoinSubstitution(
            [FindPackageShare('bunker_base'), 'launch', 'bunker_base.launch.py']
        )
    )

    def piper_source():
        return PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('agx_arm_ctrl'),
                'launch',
                'start_single_agx_arm.launch.py',
            ])
        )

    hidden_piper_topic_arguments = {
        'feedback_arm_status_topic': '_unused/feedback/arm_status',
        'feedback_leader_joint_states_topic': '_unused/feedback/leader_joint_states',
        'feedback_gripper_status_topic': '_unused/feedback/gripper_status',
        'feedback_hand_status_topic': '_unused/feedback/hand_status',
        'control_move_j_topic': '_unused/control/move_j',
        'control_move_p_topic': '_unused/control/move_p',
        'control_move_l_topic': '_unused/control/move_l',
        'control_move_c_topic': '_unused/control/move_c',
        'control_move_js_topic': '_unused/control/move_js',
        'control_move_mit_topic': '_unused/control/move_mit',
        'control_hand_topic': '_unused/control/hand',
        'control_hand_position_time_topic': '_unused/control/hand_position_time',
    }

    def piper_launch_arguments(can_port, tcp_offset, fw_version):
        arguments = {
            'can_port': can_port,
            'arm_type': 'piper_x',
            'fw_version': fw_version,
            'effector_type': LaunchConfiguration('piper_effector_type'),
            'auto_enable': LaunchConfiguration('piper_auto_enable'),
            'control_enabled': LaunchConfiguration('piper_control_enabled'),
            'speed_percent': LaunchConfiguration('piper_speed_percent'),
            'tcp_offset': tcp_offset,
        }
        arguments.update(hidden_piper_topic_arguments)
        return arguments


    return LaunchDescription([
        DeclareLaunchArgument('start_bunker_driver', default_value='true'),
        DeclareLaunchArgument('start_piper_drivers', default_value='true'),
        DeclareLaunchArgument(
            'start_front_piper_driver',
            default_value='true',
            description='Start the front PiPER hardware driver when PiPER drivers are enabled.',
        ),
        DeclareLaunchArgument(
            'start_rear_piper_driver',
            default_value='true',
            description='Start the rear PiPER hardware driver when PiPER drivers are enabled.',
        ),
        DeclareLaunchArgument(
            'bunker_can',
            default_value='UNSET_BUNKER_CAN',
            description='Required CAN interface when start_bunker_driver:=true.',
        ),
        DeclareLaunchArgument(
            'front_piper_can',
            default_value='',
            description='Front PiPER SocketCAN interface.',
        ),
        DeclareLaunchArgument(
            'rear_piper_can',
            default_value='',
            description='Rear PiPER SocketCAN interface.',
        ),
        DeclareLaunchArgument('is_bunker_mini', default_value='true'),
        DeclareLaunchArgument('bunker_control_rate', default_value='20'),
        DeclareLaunchArgument(
            'bunker_publish_tf',
            default_value='true',
            description=(
                'Let the Bunker driver publish odom -> base_link. Set false '
                'only when an EKF owns that same TF edge.'
            ),
        ),
        DeclareLaunchArgument(
            'bunker_odom_topic_name',
            default_value='odom',
            description=(
                'Bunker driver odometry topic. Use wheel/odom when EKF '
                'publishes the public /odom topic.'
            ),
        ),
        DeclareLaunchArgument(
            'piper_auto_enable', default_value='false',
            description='Keep false during Nav2/TF commissioning.',
        ),
        DeclareLaunchArgument(
            'piper_control_enabled',
            default_value='true',
            description='Keep the PiPER external control gate open by default.',
        ),
        DeclareLaunchArgument(
            'piper_speed_percent',
            default_value='5',
            description='Low PiPER speed limit for commissioning and initial-pose moves.',
        ),
        DeclareLaunchArgument(
            'piper_effector_type',
            default_value='none',
            description='PiPER end effector passed to agx_arm_ctrl.',
        ),
        DeclareLaunchArgument(
            'front_piper_fw_version',
            default_value='v189',
            description='Front PiPER firmware fallback when probing is unavailable.',
        ),
        DeclareLaunchArgument(
            'rear_piper_fw_version',
            default_value='v189',
            description='Rear PiPER firmware fallback when probing is unavailable.',
        ),
        DeclareLaunchArgument(
            'front_piper_tcp_offset',
            default_value='[0.0, 0.0, 0.1425, 0.0, 0.0, 0.0]',
            description='Front PiPER TCP offset used by the Illiyas model.',
        ),
        DeclareLaunchArgument(
            'rear_piper_tcp_offset',
            default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
        ),
        OpaqueFunction(function=validate_bunker_can),
        OpaqueFunction(function=validate_piper_can),
        GroupAction(
            condition=IfCondition(LaunchConfiguration('start_bunker_driver')),
            actions=[
                IncludeLaunchDescription(
                    bunker_source,
                    launch_arguments={
                        'port_name': LaunchConfiguration('bunker_can'),
                        'is_bunker_mini': LaunchConfiguration('is_bunker_mini'),
                        'control_rate': LaunchConfiguration('bunker_control_rate'),
                        'publish_tf': LaunchConfiguration('bunker_publish_tf'),
                        'odom_topic_name': LaunchConfiguration('bunker_odom_topic_name'),
                    }.items(),
                ),
            ],
        ),
        GroupAction(
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('start_piper_drivers'), "' == 'true' and '",
                LaunchConfiguration('start_front_piper_driver'), "' == 'true'",
            ])),
            actions=[
                PushRosNamespace('front_piper'),
                IncludeLaunchDescription(
                    piper_source(),
                    launch_arguments=piper_launch_arguments(
                        LaunchConfiguration('front_piper_can'),
                        LaunchConfiguration('front_piper_tcp_offset'),
                        LaunchConfiguration('front_piper_fw_version')).items(),
                ),
            ],
        ),
        GroupAction(
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('start_piper_drivers'), "' == 'true' and '",
                LaunchConfiguration('start_rear_piper_driver'), "' == 'true'",
            ])),
            actions=[
                PushRosNamespace('rear_piper'),
                IncludeLaunchDescription(
                    piper_source(),
                    launch_arguments=piper_launch_arguments(
                        LaunchConfiguration('rear_piper_can'),
                        LaunchConfiguration('rear_piper_tcp_offset'),
                        LaunchConfiguration('rear_piper_fw_version')).items(),
                ),
            ],
        ),
    ])
