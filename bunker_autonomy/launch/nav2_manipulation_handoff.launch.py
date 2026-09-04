"""Publish a continuous Nav2-to-manipulation handoff state topic."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'progress_topic',
            default_value='/navigation_manipulation/progress',
            description='Continuous JSON String topic for the other Docker to listen to.',
        ),
        DeclareLaunchArgument(
            'manipulation_status_topic',
            default_value='/manipulation_task/progress',
            description='Optional String topic where manipulation reports running/succeeded/failed.',
        ),
        DeclareLaunchArgument(
            'task_name',
            default_value='door_manipulation',
            description='Task label included in the progress JSON.',
        ),
        DeclareLaunchArgument(
            'trigger_on_nav2_success',
            default_value='false',
            choices=['true', 'false'],
            description='Set true to request manipulation after any Nav2 NavigateToPose success.',
        ),
        DeclareLaunchArgument(
            'trigger_on_door_arrival',
            default_value='true',
            choices=['true', 'false'],
            description='Set true to request manipulation when /door_navigation/arrived is received.',
        ),
        Node(
            package='bunker_autonomy',
            executable='nav2_arrival_manipulation_trigger',
            name='nav2_manipulation_handoff',
            output='screen',
            parameters=[{
                'progress_topic': LaunchConfiguration('progress_topic'),
                'manipulation_status_topic': LaunchConfiguration('manipulation_status_topic'),
                'task_name': LaunchConfiguration('task_name'),
                'trigger_on_nav2_success': ParameterValue(
                    LaunchConfiguration('trigger_on_nav2_success'),
                    value_type=bool,
                ),
                'trigger_on_door_arrival': ParameterValue(
                    LaunchConfiguration('trigger_on_door_arrival'),
                    value_type=bool,
                ),
                'publish_string': False,
                'publish_bool': False,
                'publish_receiver_message': False,
            }],
        ),
    ])
