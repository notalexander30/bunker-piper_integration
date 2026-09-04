"""Run Nav2 against an external RTAB-Map map and isolated velocity channel."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare('bunker_dual_piper_nav2'), 'config',
         'nav2_bunker_params.yaml']
    )
    default_nav_to_pose_bt_xml = PathJoinSubstitution(
        [FindPackageShare('bunker_dual_piper_nav2'), 'behavior_trees',
         'navigate_to_pose_back_up_recovery.xml']
    )
    default_nav_through_poses_bt_xml = PathJoinSubstitution(
        [FindPackageShare('bunker_dual_piper_nav2'), 'behavior_trees',
         'navigate_through_poses_back_up_recovery.xml']
    )
    params = LaunchConfiguration('params_file')
    clock_override = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    bt_xml_override = {
        'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
        'default_nav_through_poses_bt_xml': default_nav_through_poses_bt_xml,
    }
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    nodes = [
        Node(
            package='nav2_controller', executable='controller_server',
            name='controller_server', output='screen',
            parameters=[params, clock_override],
            remappings=tf_remaps + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_smoother', executable='smoother_server',
            name='smoother_server', output='screen',
            parameters=[params, clock_override], remappings=tf_remaps,
        ),
        Node(
            package='nav2_planner', executable='planner_server',
            name='planner_server', output='screen',
            parameters=[params, clock_override], remappings=tf_remaps,
        ),
        Node(
            package='nav2_behaviors', executable='behavior_server',
            name='behavior_server', output='screen',
            parameters=[params, clock_override], remappings=tf_remaps,
        ),
        Node(
            package='nav2_bt_navigator', executable='bt_navigator',
            name='bt_navigator', output='screen',
            parameters=[params, bt_xml_override, clock_override], remappings=tf_remaps,
        ),
        Node(
            package='nav2_waypoint_follower', executable='waypoint_follower',
            name='waypoint_follower', output='screen',
            parameters=[params, clock_override], remappings=tf_remaps,
        ),
        Node(
            package='nav2_velocity_smoother', executable='velocity_smoother',
            name='velocity_smoother', output='screen',
            parameters=[params, clock_override],
            # Unlike the stock bringup, the final output never uses /cmd_vel.
            remappings=tf_remaps + [
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', '/cmd_vel_nav2_raw'),
            ],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True,
                'node_names': lifecycle_nodes,
            }],
        ),
        Node(
            package='bunker_dual_piper_nav2',
            executable='safe_cmd_vel_gate',
            name='safe_cmd_vel_gate', output='screen',
            parameters=[{
                'input_topic': '/cmd_vel_nav2_raw',
                'output_topic': LaunchConfiguration('safe_cmd_vel_output'),
                'max_linear_x': 0.225,
                'max_angular_z': 0.375,
                'timeout': 0.50,
            }],
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument(
            'safe_cmd_vel_output', default_value='/cmd_vel_debug',
            description=(
                'Use /cmd_vel_debug for validation. Only use '
                '/cmd_vel_autonomy after the stop test passes.'
            ),
        ),
        *nodes,
    ])
