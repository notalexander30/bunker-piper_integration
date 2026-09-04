"""Standalone RGB-D mapping for a handheld D435i.

This launch intentionally starts no CAN interface, Bunker driver, PiPER driver,
robot_state_publisher or camera.  It consumes an already-running RealSense
camera and produces full six-degree-of-freedom visual odometry in the
conventional camera body frame. The optical frame has Z forward and is not a
suitable planar reference for a hand-carried occupancy map.

The external H30 is launched separately.  It is not fused here until its rigid
transform to the camera has been measured; fusing an uncalibrated IMU frame
would make the map worse rather than better.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_database_path():
    prefix = Path(get_package_prefix('bunker_slam_bringup'))
    workspace = prefix.parent.parent if prefix.parent.name == 'install' else Path('/ros2_ws')
    return str(workspace / 'src' / 'bunker_slam_bringup' / 'maps' / 'd435i_visual_mapping.db')


def _rtabmap(context):
    database_path = str(Path(LaunchConfiguration('database_path').perform(context)).expanduser())
    reset = LaunchConfiguration('reset_database').perform(context).lower() == 'true'
    Path(database_path).parent.mkdir(parents=True, exist_ok=True)
    config = os.path.join(get_package_share_directory('bunker_slam_bringup'), 'config', 'rtabmap.yaml')
    return [
        LogInfo(msg=f'Minimal D435i visual mapping database: {database_path}'),
        Node(
            package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
            condition=IfCondition(LaunchConfiguration('start_rtabmap')),
            parameters=[config, {
                'frame_id': 'camera_link',
                # RTAB-Map owns map -> odom. RViz, semantic memory and saved
                # door positions all use this stable map frame.
                'map_frame_id': 'map',
                'odom_frame_id': '',
                'publish_tf': True,
                'database_path': database_path,
                'Mem/IncrementalMemory': 'true',
                # rgbd_sync supplies registered color plus aligned depth.
                'subscribe_rgb': False,
                'subscribe_depth': False,
                'subscribe_rgbd': True,
                'Grid/Sensor': '1',
                'Grid/3D': 'false',
                # The full robot config is planar because base_link stays
                # level. A handheld camera changes height, roll and pitch.
                'Reg/Force3DoF': 'false',
            }],
            arguments=['--delete_db_on_start'] if reset else [],
            remappings=[('rgbd_image', '/rgbd_image'), ('odom', '/visual_odom')],
        ),
    ]


def generate_launch_description():
    config = os.path.join(get_package_share_directory('bunker_slam_bringup'), 'config', 'rtabmap.yaml')
    rgbd_odom = Node(
        package='rtabmap_odom', executable='rgbd_odometry', name='rgbd_odometry', output='screen',
        condition=IfCondition(LaunchConfiguration('start_rgbd_odometry')),
        parameters=[{
            'frame_id': 'camera_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'subscribe_imu': False,
            'approx_sync': True,
            'topic_queue_size': 30,
            'sync_queue_size': 30,
            'qos': 1,
            'qos_camera_info': 1,
            'wait_for_transform': 0.25,
            # Handheld scanning must not create a new RTAB-Map session after a
            # single blurred or textureless frame.  With auto-reset disabled,
            # a tracking loss pauses registration until the operator returns
            # to the last recognizable view.
            'Odom/Strategy': '0',
            'Odom/ResetCountdown': '0',
            # GFTT corners with ORB descriptors are substantially more useful
            # than the default detector on the low-texture 640x480/15 Hz feed
            # available through this D435i's USB connection.  Eight inliers is
            # still enough to reject a blank wall, while allowing the observed
            # 8--13-match handheld frames to initialize odometry.
            'Vis/FeatureType': '8',
            'Vis/MaxFeatures': '1500',
            'Vis/MinInliers': '8',
            'Vis/CorNNDR': '0.85',
            'OdomF2M/MaxSize': '2000',
            'OdomF2M/MaxNewFeatures': '800',
            # Ignore the camera mount, operator hand and nearby surfaces. The
            # current failed local map was dominated by 0.15 m features, which
            # are too close for stable handheld RGB-D visual odometry.
            'Vis/MinDepth': '0.20',
            'Vis/MaxDepth': '5.0',
            'Reg/Force3DoF': 'false',
        }],
        remappings=[
            ('rgb/image', '/camera/camera/color/image_raw'),
            ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('odom', '/visual_odom'),
        ],
    )
    rgbd_sync = Node(
        package='rtabmap_sync', executable='rgbd_sync', name='rgbd_sync', output='screen',
        condition=IfCondition(LaunchConfiguration('start_rgbd_sync')),
        parameters=[config],
        remappings=[
            ('rgb/image', '/camera/camera/color/image_raw'),
            ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('rgbd_image', '/rgbd_image'),
        ],
    )
    live_cloud = Node(
        package='rtabmap_util',
        executable='point_cloud_xyzrgb',
        name='camera_live_cloud',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_live_cloud')),
        parameters=[{
            'approx_sync': True,
            # Keep the diagnostic cloud light enough to run beside SLAM on
            # the Jetson. RTAB-Map's /cloud_map remains the accumulated map.
            'decimation': 4,
            'voxel_size': 0.02,
        }],
        remappings=[
            ('rgb/image', '/camera/camera/color/image_raw'),
            ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('cloud', '/camera/live_rgbd_cloud'),
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument('database_path', default_value=_default_database_path()),
        DeclareLaunchArgument('reset_database', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('start_rgbd_odometry', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_rgbd_sync', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_live_cloud', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_rtabmap', default_value='true', choices=['true', 'false']),
        LogInfo(msg=(
            'Standalone mapping: D435i full 6-DoF RGB-D visual odometry in '
            'camera_link; CAN and PiPER are not started.')),
        rgbd_odom,
        rgbd_sync,
        live_cloud,
        OpaqueFunction(function=_rtabmap),
    ])
