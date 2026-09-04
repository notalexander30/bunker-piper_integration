"""Run ArUco door-landmark detection plus frontier exploration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    frontier_drive_launch = PathJoinSubstitution([
        FindPackageShare('bunker_slam_bringup'),
        'launch',
        'frontier_exploration_drive.launch.py',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/front_camera/color/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/front_camera/color/camera_info'),
        DeclareLaunchArgument('aruco_dictionary', default_value='DICT_4X4_50'),
        DeclareLaunchArgument('target_marker_ids', default_value='6'),
        DeclareLaunchArgument('marker_size_m', default_value='0.05'),
        DeclareLaunchArgument('required_seen_count', default_value='3'),
        DeclareLaunchArgument('save_path', default_value='/ros2_ws/maps/aruco_landmarks.json'),
        DeclareLaunchArgument(
            'start_landmark_navigator',
            default_value='true',
            choices=['true', 'false']),
        DeclareLaunchArgument(
            'completion_mode',
            default_value='aruco_found',
            choices=['aruco_found', 'time_limit', 'frontier_ratio']),
        DeclareLaunchArgument('time_limit_sec', default_value='120.0'),
        DeclareLaunchArgument('frontier_stop_ratio', default_value='0.20'),
        DeclareLaunchArgument('frontier_max_distance_m', default_value='4.0'),
        DeclareLaunchArgument('frontier_min_distance_m', default_value='0.75'),
        DeclareLaunchArgument('frontier_cluster_min_cells', default_value='3'),
        DeclareLaunchArgument('no_frontier_return_cycles', default_value='2'),
        DeclareLaunchArgument('return_to_start', default_value='true'),
        DeclareLaunchArgument(
            'dry_run',
            default_value='false',
            choices=['true', 'false'],
            description='false sends real Nav2 goals; true only logs selected frontiers.'),
        LogInfo(msg=(
            'ArUco landmark exploration: detector saves door landmarks; '
            'frontier explorer starts only after /frontier_explorer/start.')),
        Node(
            package='bunker_autonomy',
            executable='aruco_landmark_node',
            name='aruco_landmarks',
            output='screen',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'aruco_dictionary': LaunchConfiguration('aruco_dictionary'),
                'target_marker_ids': ParameterValue(
                    LaunchConfiguration('target_marker_ids'), value_type=str),
                'marker_size_m': ParameterValue(
                    LaunchConfiguration('marker_size_m'), value_type=float),
                'required_seen_count': ParameterValue(
                    LaunchConfiguration('required_seen_count'), value_type=int),
                'save_path': LaunchConfiguration('save_path'),
            }],
        ),
        Node(
            package='bunker_autonomy',
            executable='landmark_navigator_node',
            name='landmark_navigator',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_landmark_navigator')),
            parameters=[{
                'landmark_path': LaunchConfiguration('save_path'),
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(frontier_drive_launch),
            launch_arguments={
                'completion_mode': LaunchConfiguration('completion_mode'),
                'time_limit_sec': LaunchConfiguration('time_limit_sec'),
                'frontier_stop_ratio': LaunchConfiguration('frontier_stop_ratio'),
                'frontier_max_distance_m': LaunchConfiguration('frontier_max_distance_m'),
                'frontier_min_distance_m': LaunchConfiguration('frontier_min_distance_m'),
                'frontier_cluster_min_cells': LaunchConfiguration(
                    'frontier_cluster_min_cells'),
                'no_frontier_return_cycles': LaunchConfiguration(
                    'no_frontier_return_cycles'),
                'return_to_start': LaunchConfiguration('return_to_start'),
                'dry_run': LaunchConfiguration('dry_run'),
                'aruco_found_topic': '/aruco_landmarks/found',
            }.items(),
        ),
    ])
