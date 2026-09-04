"""Simulation-only dual-PiPER smoke test layered on the standalone workflow.

No Bunker or PiPER hardware package is included.  Gazebo consumes only the
dedicated /cmd_vel_sim topic, while the optional standalone YOLO launch keeps
its existing /camera/camera/color/image_raw input contract.
"""

from launch import LaunchDescription
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('bunker_dual_piper_nav2')
    model = PathJoinSubstitution(
        [package_share, 'urdf', 'bunker_dual_piper_d435i.urdf.xacro']
    )
    world = PathJoinSubstitution([package_share, 'worlds', 'standalone_test.sdf'])
    rviz_config = PathJoinSubstitution(
        [package_share, 'rviz', 'bunker_nav2.rviz']
    )
    standalone_yolo = PathJoinSubstitution([
        FindPackageShare('bunker_slam_bringup'),
        'launch',
        'standalone_yolo26.launch.py',
    ])

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ign sdf rewrites package:// URIs as model:// URIs.  Gazebo therefore
    # needs the parents of the ROS package share directories in its resource
    # path in order to find this package's and RealSense's meshes.
    resource_roots = {
        os.path.dirname(get_package_share_directory('bunker_dual_piper_nav2')),
        os.path.dirname(get_package_share_directory('realsense2_description')),
    }
    existing_resource_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    if existing_resource_path:
        resource_roots.add(existing_resource_path)
    gazebo_resource_path = os.pathsep.join(sorted(resource_roots))
    robot_description = ParameterValue(
        Command([
            'xacro ', model,
            ' simulation:=true',
            ' sim_cmd_vel_topic:=/cmd_vel_sim',
            ' sim_odom_topic:=/odom',
            ' sim_tf_topic:=/tf',
        ]),
        value_type=str,
    )

    gazebo = ExecuteProcess(
        # Server-only mode makes headless testing independent of DISPLAY.
        cmd=['ign', 'gazebo', '-v', '3', '-s', '-r', world],
        output='screen',
    )
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='dual_piper_sim_robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
    )
    default_joint_states = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='dual_piper_sim_joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
    )
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_dual_piper_sim',
        output='screen',
        arguments=[
            '-name', 'bunker_dual_piper',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.02',
        ],
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='dual_piper_sim_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel_sim@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/camera/camera/color/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/camera/color/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/camera/camera/aligned_depth_to_color/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
        ],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='dual_piper_sim_rviz',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_rviz')),
        arguments=['-d', rviz_config, '-f', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(standalone_yolo),
        condition=IfCondition(LaunchConfiguration('start_standalone_yolo')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument(
            'start_standalone_yolo',
            default_value='false',
            description=(
                'Start the existing observation-only YOLO26 launch. It sends '
                'no velocity or hardware commands.'
            ),
        ),
        SetEnvironmentVariable(
            'IGN_GAZEBO_RESOURCE_PATH', gazebo_resource_path
        ),
        gazebo,
        state_publisher,
        default_joint_states,
        bridge,
        TimerAction(period=3.0, actions=[spawn]),
        rviz,
        yolo,
    ])
