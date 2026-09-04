"""One-pane mapping/localization, optional robot TF and X11 RViz bringup."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def validate_backend(context):
    backend = LaunchConfiguration('localization_backend').perform(context)
    mode = LaunchConfiguration('mode').perform(context)
    if backend == 'amcl' and mode != 'localization':
        raise RuntimeError(
            'localization_backend:=amcl requires mode:=localization. '
            'AMCL localizes against an existing 2D map and cannot create an '
            'RTAB-Map database.'
        )
    return []


def generate_launch_description():
    rtabmap = PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare('bunker_dual_piper_nav2'),
        'launch',
        'rtabmap_front_camera.launch.py',
    ]))
    amcl = PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare('bunker_slam_bringup'),
        'launch',
        'amcl_localization.launch.py',
    ]))
    description = PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare('bunker_dual_piper_nav2'),
        'launch',
        'description.launch.py',
    ]))
    rviz_config = PathJoinSubstitution([
        FindPackageShare('bunker_slam_bringup'),
        'rviz',
        'camera_rgbd_mapping.rviz',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='mapping',
            choices=['mapping', 'localization'],
            description='Mapping/localization mode for this terminal preset.'),
        DeclareLaunchArgument(
            'localization_backend',
            default_value='rtabmap',
            choices=['rtabmap', 'amcl'],
            description=(
                'Use rtabmap for RGB-D mapping/localization, or amcl for '
                'saved 2D map localization with a real /scan topic.')),
        DeclareLaunchArgument(
            'database_path', default_value='/ros2_ws/maps/bunker_dual_rgbd_v2.db'),
        DeclareLaunchArgument(
            'reset_database', default_value='false',
            choices=['true', 'false']),
        DeclareLaunchArgument(
            'mapping_camera',
            default_value='both',
            choices=['front', 'rear', 'both'],
            description=(
                'RTAB-Map RGB-D input. front/rear use identical single-camera '
                'mapping; both requires synchronized front and rear RGB-D.')),
        DeclareLaunchArgument(
            'map',
            default_value='/ros2_ws/src/bunker_slam_bringup/maps/bunker_map.yaml',
            description='2D map YAML used only with localization_backend:=amcl.'),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='LaserScan topic used only with localization_backend:=amcl.'),
        DeclareLaunchArgument(
            'amcl_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('bunker_slam_bringup'),
                'config',
                'amcl_nav2_params.yaml',
            ]),
            description='AMCL/map_server params used only by the AMCL backend.'),
        DeclareLaunchArgument(
            'start_description', default_value='false',
            choices=['true', 'false']),
        DeclareLaunchArgument(
            'use_rviz', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=validate_backend),
        IncludeLaunchDescription(
            description,
            condition=IfCondition(LaunchConfiguration('start_description')),
            launch_arguments={
                # Mapping pane must publish the robot TF tree even when the
                # robot/IMU pane is not running.
                'prefix_joint_states': 'true',
                'publish_default_joint_states': 'true',
                'front_camera_name': 'front_camera',
                'front_camera_nominal_extrinsics': 'true',
                'use_front_joint_feedback': 'false',
                'use_rear_joint_feedback': 'false',
            }.items(),
        ),
        IncludeLaunchDescription(
            rtabmap,
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('localization_backend'), "' == 'rtabmap'",
            ])),
            launch_arguments={
                'mode': LaunchConfiguration('mode'),
                'database_path': LaunchConfiguration('database_path'),
                'reset_database': LaunchConfiguration('reset_database'),
                'mapping_camera': LaunchConfiguration('mapping_camera'),
            }.items(),
        ),
        IncludeLaunchDescription(
            amcl,
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('localization_backend'), "' == 'amcl'",
            ])),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'params_file': LaunchConfiguration('amcl_params_file'),
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='mapping_rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            additional_env={
                'QT_QPA_PLATFORM': 'xcb',
                'QT_X11_NO_MITSHM': '1',
            },
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
