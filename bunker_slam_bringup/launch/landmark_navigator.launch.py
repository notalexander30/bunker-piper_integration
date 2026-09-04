"""Nav2 goal sender and RViz marker publisher for saved landmarks."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'landmark_path',
            default_value='/ros2_ws/maps/manual_nav_landmarks.json'),
        DeclareLaunchArgument('action_name', default_value='/navigate_to_pose'),
        DeclareLaunchArgument('marker_topic', default_value='/landmark_navigator/markers'),
        DeclareLaunchArgument(
            'door_arrival_topic',
            default_value='/door_navigation/arrived'),
        DeclareLaunchArgument(
            'home_arrival_topic',
            default_value='/home_navigation/arrived'),
        DeclareLaunchArgument(
            'arrival_publish_hz',
            default_value='2.0'),
        DeclareLaunchArgument(
            'arrival_true_publish_count',
            default_value='10'),
        LogInfo(msg=(
            'Landmark navigator: publishes RViz home/door markers and sends '
            'Nav2 goals to home, door, aruco_N, or furthest valid marker.')),
        Node(
            package='bunker_autonomy',
            executable='landmark_navigator_node',
            name='landmark_navigator',
            output='screen',
            parameters=[{
                'landmark_path': LaunchConfiguration('landmark_path'),
                'action_name': LaunchConfiguration('action_name'),
                'marker_topic': LaunchConfiguration('marker_topic'),
                'door_arrival_topic': LaunchConfiguration('door_arrival_topic'),
                'home_arrival_topic': LaunchConfiguration('home_arrival_topic'),
                'door_arrival_publish_hz': LaunchConfiguration(
                    'arrival_publish_hz'),
                'home_arrival_publish_hz': LaunchConfiguration(
                    'arrival_publish_hz'),
                'arrival_true_publish_count': LaunchConfiguration(
                    'arrival_true_publish_count'),
            }],
        ),
    ])
