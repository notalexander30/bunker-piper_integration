"""Safety-gated Nav2 planner/controller bringup for RTAB-Map localization.

This launch expects the full ``map -> odom -> base_link -> camera`` TF chain,
``/map``, the Bunker driver's ``/odom`` and the registered RealSense point cloud to
already exist. The default dry-run mode cannot publish to the chassis topic.
"""

import os

from ament_index_python.packages import get_package_share_directory
from bunker_autonomy.launch_safety import resolve_output_cmd_vel_topic
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _nodes(context):
    mode = LaunchConfiguration('mode').perform(context).strip().lower()
    allow_motion = (
        LaunchConfiguration('allow_motion').perform(context).strip().lower()
        == 'true'
    )
    if mode == 'drive' and not allow_motion:
        raise RuntimeError(
            'drive mode requires the explicit allow_motion:=true opt-in')
    output_topic = resolve_output_cmd_vel_topic(mode)
    use_cmd_vel_mux = (
        LaunchConfiguration('use_cmd_vel_mux').perform(context).strip().lower()
        == 'true'
    )
    config = LaunchConfiguration('params_file')
    default_nav_to_pose_bt_xml = PathJoinSubstitution([
        FindPackageShare('bunker_dual_piper_nav2'),
        'behavior_trees',
        'navigate_to_pose_back_up_recovery.xml',
    ])
    default_nav_through_poses_bt_xml = PathJoinSubstitution([
        FindPackageShare('bunker_dual_piper_nav2'),
        'behavior_trees',
        'navigate_through_poses_back_up_recovery.xml',
    ])
    bt_xml_override = {
        'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
        'default_nav_through_poses_bt_xml': default_nav_through_poses_bt_xml,
    }

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]
    if use_cmd_vel_mux:
        lifecycle_nodes.append('velocity_smoother')
    common = {'parameters': [config], 'output': 'screen'}
    nav_nodes = [
        Node(
            package='nav2_controller', executable='controller_server',
            name='controller_server',
            remappings=[(
                'cmd_vel',
                '/nav2/cmd_vel_raw' if use_cmd_vel_mux else output_topic,
            )], **common),
        Node(
            package='nav2_smoother', executable='smoother_server',
            name='smoother_server', **common),
        Node(
            package='nav2_planner', executable='planner_server',
            name='planner_server', **common),
        Node(
            package='nav2_behaviors', executable='behavior_server',
            name='behavior_server',
            remappings=[(
                'cmd_vel',
                '/cmd_vel_autonomy' if use_cmd_vel_mux else output_topic,
            )], **common),
        Node(
            package='nav2_bt_navigator', executable='bt_navigator',
            name='bt_navigator',
            parameters=[config, bt_xml_override],
            output='screen'),
        Node(
            package='nav2_waypoint_follower', executable='waypoint_follower',
            name='waypoint_follower', **common),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': lifecycle_nodes,
            }]),
    ]
    if use_cmd_vel_mux:
        nav_nodes.append(Node(
            package='nav2_velocity_smoother', executable='velocity_smoother',
            name='velocity_smoother',
            remappings=[
                ('cmd_vel', '/nav2/cmd_vel_raw'),
                ('cmd_vel_smoothed', '/cmd_vel_autonomy'),
            ], **common))
    mux = Node(
        package='bunker_autonomy', executable='cmd_vel_mux_node',
        name='nav2_cmd_vel_safety_mux', output='screen',
        parameters=[{
            'cmd_vel_autonomy_topic': '/cmd_vel_autonomy',
            'safety_stop_topic': '/safety_stop',
            'output_cmd_vel_topic': output_topic,
            'mux_reason_topic': '/cmd_vel_mux/reason',
            'initial_safety_stop': False,
            'use_safety_stop': True,
            'allow_reverse': False,
            'max_linear_speed_mps': 0.225,
            'max_angular_speed_radps': 0.375,
            'command_timeout_sec': 0.5,
            'publish_rate_hz': 20.0,
            'linear_acceleration_mps2': 0.40,
            'linear_deceleration_mps2': 0.50,
            'angular_acceleration_radps2': 0.60,
            'angular_deceleration_radps2': 0.75,
        }],
    )
    clicked_goal = Node(
        package='bunker_autonomy',
        executable='clicked_point_nav_goal',
        name='clicked_point_nav_goal',
        output='screen',
        parameters=[{
            'clicked_point_topic': '/clicked_point',
            'action_name': '/navigate_to_pose',
            'global_frame': 'map',
            'base_frame': 'base_link',
            'use_current_robot_yaw': True,
        }],
        condition=IfCondition(LaunchConfiguration('start_clicked_goal')),
    )
    manipulation_trigger = Node(
        package='bunker_autonomy',
        executable='nav2_arrival_manipulation_trigger',
        name='nav2_arrival_manipulation_trigger',
        output='screen',
        parameters=[{
            'status_topic': '/navigate_to_pose/_action/status',
            'trigger_topic': LaunchConfiguration('manipulation_trigger_topic'),
            'trigger_bool_topic': LaunchConfiguration(
                'manipulation_trigger_bool_topic'),
            'receiver_topic': LaunchConfiguration(
                'manipulation_receiver_topic'),
            'task_name': LaunchConfiguration('manipulation_task_name'),
            'receiver_message': LaunchConfiguration(
                'manipulation_receiver_message'),
            'trigger_on_nav2_success': False,
            'trigger_on_door_arrival': True,
            'door_arrival_topic': LaunchConfiguration('door_arrival_topic'),
        }],
        condition=IfCondition(LaunchConfiguration('start_manipulation_trigger')),
    )
    landmark_navigator = Node(
        package='bunker_autonomy',
        executable='landmark_navigator_node',
        name='landmark_navigator',
        output='screen',
        parameters=[{
            'landmark_path': LaunchConfiguration('landmark_path'),
            'action_name': '/navigate_to_pose',
            'marker_topic': '/landmark_navigator/markers',
            'door_arrival_topic': LaunchConfiguration('door_arrival_topic'),
            'home_arrival_topic': LaunchConfiguration('home_arrival_topic'),
            'door_arrival_publish_hz': LaunchConfiguration(
                'arrival_publish_hz'),
            'home_arrival_publish_hz': LaunchConfiguration(
                'arrival_publish_hz'),
            'arrival_true_publish_count': LaunchConfiguration(
                'arrival_true_publish_count'),
            'active_camera_topic': '/landmark_navigator/active_camera',
            'navigation_direction_topic': (
                '/landmark_navigator/navigation_direction'),
            'forward_camera_name': 'front_camera',
            'reverse_camera_name': 'rear_camera',
            'reverse_home_from_door': True,
        }],
        condition=IfCondition(LaunchConfiguration('start_landmark_navigator')),
    )
    return [
        LogInfo(msg=(
            f'Nav2 mode={mode}; cmd_vel output={output_topic}; '
            f'use_cmd_vel_mux={use_cmd_vel_mux}.')),
        *nav_nodes,
        *([mux] if use_cmd_vel_mux else []),
        clicked_goal,
        manipulation_trigger,
        landmark_navigator,
    ]


def generate_launch_description():
    share = get_package_share_directory('bunker_slam_bringup')
    default_config = os.path.join(share, 'config', 'nav2_bunker.yaml')
    depth_safety_config = os.path.join(
        get_package_share_directory('bunker_autonomy'),
        'config', 'autonomy_d435i.yaml')
    rviz_config = os.path.join(share, 'rviz', 'bunker_nav2.rviz')
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='dry_run', choices=['dry_run', 'drive']),
        DeclareLaunchArgument(
            'allow_motion', default_value='false', choices=['true', 'false'],
            description='Required explicit opt-in before drive mode can start.'),
        DeclareLaunchArgument('params_file', default_value=default_config),
        DeclareLaunchArgument(
            'use_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'start_depth_safety', default_value='false',
            choices=['true', 'false'],
            description=(
                'Publish fail-safe /safety_stop from the aligned D435i depth '
                'stream. Keep enabled for both dry-run and drive modes.')),
        DeclareLaunchArgument(
            'use_cmd_vel_mux', default_value='false',
            choices=['true', 'false'],
            description=(
                'When false, controller_server and behavior_server publish '
                'directly to the resolved cmd_vel output. When true, use the '
                'legacy /nav2/cmd_vel_raw -> /cmd_vel_autonomy -> safety mux '
                'pipeline.')),
        DeclareLaunchArgument(
            'start_clicked_goal', default_value='true',
            choices=['true', 'false'],
            description=(
                'Translate RViz Publish Point clicks on /clicked_point into '
                'Nav2 NavigateToPose goals.')),
        DeclareLaunchArgument(
            'start_manipulation_trigger', default_value='true',
            choices=['true', 'false'],
            description=(
                'Publish a manipulation-task start event after the door '
                'landmark arrival signal.')),
        DeclareLaunchArgument(
            'manipulation_trigger_topic',
            default_value='/front_piper/task/start'),
        DeclareLaunchArgument(
            'manipulation_trigger_bool_topic',
            default_value='/front_piper/task/start_bool'),
        DeclareLaunchArgument(
            'manipulation_receiver_topic',
            default_value='/navigation_task/finished'),
        DeclareLaunchArgument(
            'manipulation_receiver_message',
            default_value='navigation_finished'),
        DeclareLaunchArgument(
            'manipulation_task_name',
            default_value='front_piper_pick'),
        DeclareLaunchArgument(
            'start_landmark_navigator', default_value='true',
            choices=['true', 'false'],
            description=(
                'Publish manual landmark markers and accept named Nav2 goals '
                'on /landmark_navigator/go_marker.')),
        DeclareLaunchArgument(
            'landmark_path',
            default_value='/ros2_ws/maps/manual_nav_landmarks.json'),
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
            'Nav2 requires RTAB-Map localization, /map, /odom, '
            'the complete TF tree and the RealSense point cloud.')),
        OpaqueFunction(function=_nodes),
        Node(
            package='bunker_autonomy',
            executable='depth_route_monitor_node',
            name='depth_route_monitor_node',
            output='screen',
            parameters=[depth_safety_config],
            condition=IfCondition(LaunchConfiguration('start_depth_safety')),
        ),
        Node(
            package='bunker_autonomy',
            executable='sensor_fusion_node',
            name='sensor_fusion_node',
            output='screen',
            parameters=[depth_safety_config],
            condition=IfCondition(LaunchConfiguration('start_depth_safety')),
        ),
        Node(
            package='rviz2', executable='rviz2', name='bunker_nav2_rviz2',
            arguments=['-d', rviz_config], output='screen',
            additional_env={'QT_X11_NO_MITSHM': '1'},
            condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ])
