"""Start the wrist D435i while the URDF remains the only TF publisher."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def camera_group(name: str, enabled_arg: str, serial_arg: str):
    pointcloud_param_cmd = (
        f'while true; do '
        f'  if ros2 node list 2>/dev/null | grep -qx "/{name}"; then '
        f'    if ros2 param list /{name} 2>/dev/null | '
        f'grep -q "pointcloud__neon_.enable"; then '
        f'      timeout 3 ros2 param set /{name} '
        f'pointcloud__neon_.stream_filter 2 >/dev/null 2>&1 || true; '
        f'      timeout 3 ros2 param set /{name} '
        f'pointcloud__neon_.stream_index_filter 0 '
        f'>/dev/null 2>&1 || true; '
        f'      timeout 3 ros2 param set /{name} '
        f'pointcloud__neon_.allow_no_texture_points true '
        f'>/dev/null 2>&1 || true; '
        f'      timeout 3 ros2 param set /{name} '
        f'pointcloud__neon_.ordered_pc false '
        f'>/dev/null 2>&1 || true; '
        f'      timeout 3 ros2 param set /{name} '
        f'pointcloud__neon_.enable true >/dev/null 2>&1 || true; '
        f'    fi; '
        f'  fi; '
        f'  sleep 5; '
        f'done'
    )

    def launch_camera(context):
        serial_no = LaunchConfiguration(serial_arg).perform(context)
        serial_filter = (
            f'_{serial_no}'
            if serial_no and not serial_no.startswith('_')
            else serial_no
        )
        return [
            GroupAction(
                scoped=True,
                forwarding=False,
                actions=[
                    Node(
                        package='realsense2_camera',
                        executable='realsense2_camera_node',
                        namespace='',
                        name=name,
                        output='log',
                        arguments=['--ros-args', '--log-level', 'error'],
                        respawn=True,
                        respawn_delay=20.0,
                        parameters=[{
                            # The RealSense wrapper uses camera_name/base_frame_id
                            # to derive message header frame_ids. Without these
                            # explicit values both physical cameras publish
                            # generic camera_color_optical_frame headers, which
                            # RTAB-Map cannot transform after the compatibility
                            # alias TF publishers were removed.
                            'camera_name': name,
                            'camera_namespace': '',
                            'base_frame_id': f'{name}_link',
                            'tf_prefix': '',
                            # The RealSense launch accepts a leading
                            # underscore to force digit-only serials to remain
                            # strings after ROS launch/YAML parameter parsing.
                            'serial_no': serial_filter,
                            # The vendor 4.58.2 node emits a non-actionable
                            # warning for some internal defaults. Keep terminal
                            # output quiet; detailed camera logs still go to
                            # ~/.ros/log.
                            'enable_color': True,
                            'enable_depth': True,
                            'enable_infra': False,
                            'enable_infra1': False,
                            'enable_infra2': False,
                            'enable_sync': True,
                            # Keep only the streams needed by Nav-Man:
                            # RGB for ArUco/RTAB-Map, depth for RGB-D sync,
                            # and registered point cloud for wall/touch/depth
                            # safety. The D435i IMU streams are disabled here;
                            # the external H30 is the vehicle IMU diagnostic.
                            'depth_module.depth_profile': '640,480,15',
                            'rgb_camera.color_profile': '640,480,15',
                            'wait_for_device_timeout': 10.0,
                            'reconnect_timeout': 3.0,
                            'align_depth.enable': True,
                            'pointcloud.enable': True,
                            'pointcloud.stream_filter': 2,
                            'pointcloud.stream_index_filter': 0,
                            'pointcloud.allow_no_texture_points': True,
                            'pointcloud.ordered_pc': False,
                            # RealSense ROS 4.58.x on this Jetson exposes the
                            # active pointcloud filter as pointcloud__neon_.*.
                            # Set both names so the point cloud comes up on
                            # first launch instead of relying only on recovery.
                            'pointcloud__neon_.enable': True,
                            'pointcloud__neon_.stream_filter': 2,
                            'pointcloud__neon_.stream_index_filter': 0,
                            'pointcloud__neon_.allow_no_texture_points': True,
                            'pointcloud__neon_.ordered_pc': False,
                            'enable_gyro': False,
                            'enable_accel': False,
                            'enable_motion': False,
                            'unite_imu_method': 0,
                            'diagnostics_period': 0.0,
                            # robot_state_publisher owns camera transforms.
                            'publish_tf': False,
                        }],
                    ),
                    # RealSense ROS 4.58.x on this Jetson exposes the
                    # pointcloud filter parameters under the generated
                    # pointcloud__neon_ prefix instead of the documented
                    # pointcloud.* launch names. Set the actual runtime
                    # parameters after the camera node exists and keep
                    # reapplying them after an automatic process respawn.
                    TimerAction(period=3.0, actions=[
                        ExecuteProcess(
                            cmd=['bash', '-lc', pointcloud_param_cmd],
                            output='log',
                        ),
                    ]),
                ],
            ),
        ]

    return OpaqueFunction(
        function=launch_camera,
        condition=IfCondition(LaunchConfiguration(enabled_arg)),
    )


def generate_launch_description():
    def watchdog_node(context):
        image_topics = []
        pointcloud_topics = []
        if LaunchConfiguration('launch_front_camera').perform(context).lower() == 'true':
            image_topics.append('/front_camera/color/image_raw')
            pointcloud_topics.append('/front_camera/depth/color/points')
        if LaunchConfiguration('launch_rear_camera').perform(context).lower() == 'true':
            image_topics.append('/rear_camera/color/image_raw')
            pointcloud_topics.append('/rear_camera/depth/color/points')
        return [
            Node(
                package='bunker_autonomy',
                executable='realsense_stream_watchdog',
                name='realsense_stream_watchdog',
                output='screen',
                parameters=[{
                    'startup_grace_sec': 45.0,
                    'stale_timeout_sec': 8.0,
                    'restart_cooldown_sec': 30.0,
                    'terminate_grace_sec': 3.0,
                    'reset_usb_on_restart': True,
                    'image_topics': image_topics,
                    'pointcloud_topics': pointcloud_topics,
                }],
                condition=IfCondition(LaunchConfiguration('launch_front_camera')),
            ),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('launch_front_camera', default_value='true'),
        DeclareLaunchArgument('launch_rear_camera', default_value='false'),
        DeclareLaunchArgument(
            'front_camera_serial',
            default_value='243322074578',
            description=(
                'Front D435i serial; old known camera, used for mapping.'),
        ),
        DeclareLaunchArgument(
            'rear_camera_serial',
            default_value='261222077434',
            description='Rear D435i serial; newer second camera.',
        ),
        camera_group(
            'front_camera', 'launch_front_camera', 'front_camera_serial'),
        camera_group(
            'rear_camera', 'launch_rear_camera', 'rear_camera_serial'),
        OpaqueFunction(function=watchdog_node),
    ])
