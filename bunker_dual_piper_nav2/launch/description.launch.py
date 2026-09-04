"""Publish the combined Bunker Mini, two PiPER arms and two D435i TF tree."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model = PathJoinSubstitution(
        [FindPackageShare('bunker_dual_piper_nav2'), 'urdf',
         'bunker_dual_piper_d435i.urdf.xacro']
    )
    robot_description = ParameterValue(
        Command([
            'xacro ', model,
            ' front_piper_legacy:=', LaunchConfiguration('front_piper_legacy'),
            ' rear_piper_legacy:=', LaunchConfiguration('rear_piper_legacy'),
            # Xacro values contain spaces. Quote each one so a negative pitch
            # is not parsed as an xacro command-line option.
            ' front_piper_mount_rpy:="', LaunchConfiguration('front_piper_mount_rpy'), '"',
            ' rear_piper_mount_rpy:="', LaunchConfiguration('rear_piper_mount_rpy'), '"',
            ' front_camera_xyz:="', LaunchConfiguration('front_camera_xyz'), '"',
            ' front_camera_rpy:="', LaunchConfiguration('front_camera_rpy'), '"',
            ' front_camera_name:=', LaunchConfiguration('front_camera_name'),
            ' front_camera_nominal_extrinsics:=',
            LaunchConfiguration('front_camera_nominal_extrinsics'),
            ' rear_camera_xyz:="', LaunchConfiguration('rear_camera_xyz'), '"',
            ' rear_camera_rpy:="', LaunchConfiguration('rear_camera_rpy'), '"',
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('front_piper_legacy', default_value='false'),
        DeclareLaunchArgument('rear_piper_legacy', default_value='false'),
        DeclareLaunchArgument(
            'front_piper_mount_rpy', default_value='0 0 1.57079632679'),
        DeclareLaunchArgument('rear_piper_mount_rpy', default_value='0 0 1.57079632679'),
        DeclareLaunchArgument('front_camera_xyz', default_value='-0.03325362536889407 -0.04784194427852743 0.08630445414919316'),
        DeclareLaunchArgument('front_camera_rpy', default_value='-0.08017405150425999 -0.0031709794581423934 0.023976595541682157'),
        DeclareLaunchArgument('front_camera_name', default_value='front_camera'),
        DeclareLaunchArgument(
            'front_camera_nominal_extrinsics', default_value='true'),
        DeclareLaunchArgument('rear_camera_xyz', default_value='-0.0016016943280790555 -0.07988793520475261 0.0420998734823866'),
        DeclareLaunchArgument('rear_camera_rpy', default_value='-0.0041241555313950035 -0.003886773443390706 0.0554203109827187'),
        DeclareLaunchArgument(
            'prefix_joint_states', default_value='true',
            description='Prefix two raw PiPER JointState streams for this URDF.',
        ),
        DeclareLaunchArgument(
            'publish_default_joint_states', default_value='false',
            description='Offline RViz only; do not use with live PiPER drivers.',
        ),
        DeclareLaunchArgument(
            'front_joint_states_topic',
            default_value='/front_piper/feedback/joint_states',
        ),
        DeclareLaunchArgument(
            'rear_joint_states_topic',
            default_value='/rear_piper/feedback/joint_states',
        ),
        DeclareLaunchArgument('front_piper_parked_joint1', default_value='-1.6'),
        DeclareLaunchArgument('rear_piper_parked_joint1', default_value='1.6'),
        DeclareLaunchArgument('use_front_joint_feedback', default_value='true'),
        DeclareLaunchArgument('use_rear_joint_feedback', default_value='true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
        Node(
            package='bunker_dual_piper_nav2',
            executable='joint_state_prefixer',
            name='dual_piper_joint_state_prefixer',
            output='screen',
            condition=IfCondition(LaunchConfiguration('prefix_joint_states')),
            parameters=[{
                'front_topic': LaunchConfiguration('front_joint_states_topic'),
                'rear_topic': LaunchConfiguration('rear_joint_states_topic'),
                'front_output_topic': '/front_joint_states',
                'rear_output_topic': '/rear_joint_states',
                'use_front_feedback': ParameterValue(
                    LaunchConfiguration('use_front_joint_feedback'),
                    value_type=bool,
                ),
                'use_rear_feedback': ParameterValue(
                    LaunchConfiguration('use_rear_joint_feedback'),
                    value_type=bool,
                ),
                'front_parked_joint1': ParameterValue(
                    LaunchConfiguration('front_piper_parked_joint1'),
                    value_type=float,
                ),
                'rear_parked_joint1': ParameterValue(
                    LaunchConfiguration('rear_piper_parked_joint1'),
                    value_type=float,
                ),
            }],
        ),
        # Static preview mode does not consume a physical JointState stream.
        # Publish one complete, fixed PiPER pose directly instead of relying on
        # joint_state_publisher's optional /robot_description topic.
        Node(
            package='bunker_dual_piper_nav2',
            executable='joint_state_prefixer',
            name='offline_initial_pose_publisher',
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('publish_default_joint_states')
            ),
            parameters=[{
                'front_topic': '/__unused_urdf_preview_front_joint_states',
                'rear_topic': '/__unused_urdf_preview_rear_joint_states',
                'output_topic': '/joint_states',
                'front_output_topic': '/front_joint_states',
                'rear_output_topic': '/rear_joint_states',
                'front_parked_joint1': ParameterValue(
                    LaunchConfiguration('front_piper_parked_joint1'),
                    value_type=float,
                ),
                'rear_parked_joint1': ParameterValue(
                    LaunchConfiguration('rear_piper_parked_joint1'),
                    value_type=float,
                ),
            }],
        ),
    ])
