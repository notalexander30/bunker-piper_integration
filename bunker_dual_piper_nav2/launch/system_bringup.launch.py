"""One entry point for description, wrist cameras, Nav2 and optional RTAB-Map."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def optional_external_launch(context, enabled_arg, path_arg, label):
    if LaunchConfiguration(enabled_arg).perform(context).lower() != 'true':
        return []
    path = LaunchConfiguration(path_arg).perform(context).strip()
    if not path:
        raise RuntimeError(
            f'{enabled_arg}:=true requires {path_arg}:=/absolute/path/to/file.launch.py'
        )
    return [IncludeLaunchDescription(AnyLaunchDescriptionSource(path))]


def generate_launch_description():
    package_share = FindPackageShare('bunker_dual_piper_nav2')
    description_source = PythonLaunchDescriptionSource(
        PathJoinSubstitution([package_share, 'launch', 'description.launch.py'])
    )
    camera_source = PythonLaunchDescriptionSource(
        PathJoinSubstitution([package_share, 'launch', 'dual_realsense.launch.py'])
    )
    nav2_source = PythonLaunchDescriptionSource(
        PathJoinSubstitution([package_share, 'launch', 'nav2_rtabmap.launch.py'])
    )
    hardware_source = PythonLaunchDescriptionSource(
        PathJoinSubstitution([package_share, 'launch', 'hardware_drivers.launch.py'])
    )
    default_rviz_config = PathJoinSubstitution(
        [package_share, 'rviz', 'bunker_description_2d.rviz']
    )

    common_description_args = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'prefix_joint_states': LaunchConfiguration('prefix_joint_states'),
        'publish_default_joint_states': LaunchConfiguration('publish_default_joint_states'),
        'front_joint_states_topic': LaunchConfiguration('front_joint_states_topic'),
        'rear_joint_states_topic': LaunchConfiguration('rear_joint_states_topic'),
        'use_front_joint_feedback': LaunchConfiguration('use_front_joint_feedback'),
        'use_rear_joint_feedback': LaunchConfiguration('use_rear_joint_feedback'),
        'front_piper_parked_joint1': LaunchConfiguration('front_piper_parked_joint1'),
        'rear_piper_parked_joint1': LaunchConfiguration('rear_piper_parked_joint1'),
        'front_piper_legacy': LaunchConfiguration('front_piper_legacy'),
        'rear_piper_legacy': LaunchConfiguration('rear_piper_legacy'),
        'front_piper_mount_rpy': LaunchConfiguration('front_piper_mount_rpy'),
        'rear_piper_mount_rpy': LaunchConfiguration('rear_piper_mount_rpy'),
        'front_camera_xyz': LaunchConfiguration('front_camera_xyz'),
        'front_camera_rpy': LaunchConfiguration('front_camera_rpy'),
        'front_camera_name': LaunchConfiguration('front_camera_name'),
        'front_camera_nominal_extrinsics': LaunchConfiguration(
            'front_camera_nominal_extrinsics'),
        'rear_camera_xyz': LaunchConfiguration('rear_camera_xyz'),
        'rear_camera_rpy': LaunchConfiguration('rear_camera_rpy'),
    }

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_hardware_drivers', default_value='false'),
        DeclareLaunchArgument('start_bunker_driver', default_value='true'),
        DeclareLaunchArgument('start_piper_drivers', default_value='true'),
        DeclareLaunchArgument('start_front_piper_driver', default_value='true'),
        DeclareLaunchArgument('start_rear_piper_driver', default_value='true'),
        DeclareLaunchArgument(
            'bunker_can',
            default_value='UNSET_BUNKER_CAN',
            description='Required CAN interface when start_bunker_driver:=true.',
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
            description='Public/raw Bunker odometry topic name passed to bunker_base.',
        ),
        DeclareLaunchArgument('front_piper_can', default_value='UNSET_FRONT_CAN'),
        DeclareLaunchArgument('rear_piper_can', default_value='UNSET_REAR_CAN'),
        DeclareLaunchArgument('piper_control_enabled', default_value='true'),
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
        DeclareLaunchArgument('start_cameras', default_value='true'),
        DeclareLaunchArgument('launch_front_camera', default_value='true'),
        DeclareLaunchArgument('launch_rear_camera', default_value='true'),
        DeclareLaunchArgument('front_camera_serial', default_value=''),
        DeclareLaunchArgument('rear_camera_serial', default_value=''),
        DeclareLaunchArgument('start_nav2', default_value='true'),
        DeclareLaunchArgument('safe_cmd_vel_output', default_value='/cmd_vel_debug'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=default_rviz_config,
            description='RViz configuration; defaults to the TF-accurate 2D projection.',
        ),
        DeclareLaunchArgument('launch_rtabmap', default_value='false'),
        DeclareLaunchArgument('rtabmap_launch_file', default_value=''),
        DeclareLaunchArgument('launch_yolo', default_value='false'),
        DeclareLaunchArgument('yolo_launch_file', default_value=''),
        DeclareLaunchArgument('prefix_joint_states', default_value='true'),
        DeclareLaunchArgument('publish_default_joint_states', default_value='false'),
        DeclareLaunchArgument('front_joint_states_topic', default_value='/front_piper/feedback/joint_states'),
        DeclareLaunchArgument('rear_joint_states_topic', default_value='/rear_piper/feedback/joint_states'),
        DeclareLaunchArgument('use_front_joint_feedback', default_value='true'),
        DeclareLaunchArgument('use_rear_joint_feedback', default_value='true'),
        DeclareLaunchArgument('front_piper_parked_joint1', default_value='-1.6'),
        DeclareLaunchArgument('rear_piper_parked_joint1', default_value='1.6'),
        DeclareLaunchArgument('front_piper_legacy', default_value='false'),
        DeclareLaunchArgument('rear_piper_legacy', default_value='false'),
        DeclareLaunchArgument(
            'front_piper_mount_rpy', default_value='0 0 1.57079632679'),
        DeclareLaunchArgument('rear_piper_mount_rpy', default_value='0 0 1.57079632679'),
        DeclareLaunchArgument('front_camera_xyz', default_value='-0.03325362536889407 -0.04784194427852743 0.08630445414919316'),
        DeclareLaunchArgument('front_camera_rpy', default_value='-0.08017405150425999 -0.0031709794581423934 0.023976595541682157'),
        DeclareLaunchArgument('front_camera_name', default_value='front_camera'),
        DeclareLaunchArgument(
            'front_camera_nominal_extrinsics', default_value='false'),
        DeclareLaunchArgument('rear_camera_xyz', default_value='-0.0016016943280790555 -0.07988793520475261 0.0420998734823866'),
        DeclareLaunchArgument('rear_camera_rpy', default_value='-0.0041241555313950035 -0.003886773443390706 0.0554203109827187'),
        IncludeLaunchDescription(
            hardware_source,
            condition=IfCondition(LaunchConfiguration('start_hardware_drivers')),
            launch_arguments={
                'start_bunker_driver': LaunchConfiguration('start_bunker_driver'),
                'start_piper_drivers': LaunchConfiguration('start_piper_drivers'),
                'start_front_piper_driver': LaunchConfiguration(
                    'start_front_piper_driver'),
                'start_rear_piper_driver': LaunchConfiguration(
                    'start_rear_piper_driver'),
                'bunker_can': LaunchConfiguration('bunker_can'),
                'is_bunker_mini': LaunchConfiguration('is_bunker_mini'),
                'bunker_control_rate': LaunchConfiguration('bunker_control_rate'),
                'bunker_publish_tf': LaunchConfiguration('bunker_publish_tf'),
                'bunker_odom_topic_name': LaunchConfiguration('bunker_odom_topic_name'),
                'front_piper_can': LaunchConfiguration('front_piper_can'),
                'rear_piper_can': LaunchConfiguration('rear_piper_can'),
                'piper_auto_enable': 'false',
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
            }.items(),
        ),
        IncludeLaunchDescription(
            description_source,
            launch_arguments=common_description_args.items(),
        ),
        IncludeLaunchDescription(
            camera_source,
            condition=IfCondition(LaunchConfiguration('start_cameras')),
            launch_arguments={
                'launch_front_camera': LaunchConfiguration('launch_front_camera'),
                'launch_rear_camera': LaunchConfiguration('launch_rear_camera'),
                'front_camera_serial': LaunchConfiguration('front_camera_serial'),
                'rear_camera_serial': LaunchConfiguration('rear_camera_serial'),
            }.items(),
        ),
        OpaqueFunction(
            function=optional_external_launch,
            kwargs={
                'enabled_arg': 'launch_rtabmap',
                'path_arg': 'rtabmap_launch_file',
                'label': 'RTAB-Map',
            },
        ),
        OpaqueFunction(
            function=optional_external_launch,
            kwargs={
                'enabled_arg': 'launch_yolo',
                'path_arg': 'yolo_launch_file',
                'label': 'YOLO',
            },
        ),
        IncludeLaunchDescription(
            nav2_source,
            condition=IfCondition(LaunchConfiguration('start_nav2')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'safe_cmd_vel_output': LaunchConfiguration('safe_cmd_vel_output'),
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_rviz')),
            arguments=['-d', LaunchConfiguration('rviz_config_file')],
        ),
    ])
