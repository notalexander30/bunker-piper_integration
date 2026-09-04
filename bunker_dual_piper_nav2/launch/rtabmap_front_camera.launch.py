"""RTAB-Map using front, rear or dual D435i RGB-D streams and Bunker odometry."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CAMERA_TOPICS = {
    'front': {
        'rgb': '/front_camera/color/image_raw',
        'depth': '/front_camera/aligned_depth_to_color/image_raw',
        'info': '/front_camera/color/camera_info',
        'rgbd': '/front_rgbd_image',
        'sync_name': 'front_rgbd_sync',
    },
    'rear': {
        'rgb': '/rear_camera/color/image_raw',
        'depth': '/rear_camera/aligned_depth_to_color/image_raw',
        'info': '/rear_camera/color/camera_info',
        'rgbd': '/rear_rgbd_image',
        'sync_name': 'rear_rgbd_sync',
    },
}


def rgbd_sync_node(camera_name, config):
    camera = CAMERA_TOPICS[camera_name]
    return Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name=camera['sync_name'],
        output='screen',
        parameters=[config],
        remappings=[
            ('rgb/image', camera['rgb']),
            ('depth/image', camera['depth']),
            ('rgb/camera_info', camera['info']),
            ('rgbd_image', camera['rgbd']),
        ],
    )


def launch_rtabmap(context):
    mode = LaunchConfiguration('mode').perform(context)
    reset = LaunchConfiguration('reset_database').perform(context).lower() == 'true'
    mapping_camera = LaunchConfiguration('mapping_camera').perform(context).strip().lower()
    if mapping_camera not in ('front', 'rear', 'both'):
        raise RuntimeError("mapping_camera must be one of: front, rear, both")

    database_path = str(Path(LaunchConfiguration('database_path').perform(context)).expanduser())
    if mode == 'localization' and reset:
        raise RuntimeError('reset_database is forbidden in localization mode')
    if mode == 'localization' and not Path(database_path).is_file():
        raise RuntimeError(f'Localization database does not exist: {database_path}')
    if mode == 'mapping':
        Path(database_path).parent.mkdir(parents=True, exist_ok=True)

    config = str(Path(get_package_share_directory('bunker_dual_piper_nav2')) /
                 'config' / 'rtabmap_front_camera.yaml')
    mode_parameters = {
        'database_path': database_path,
        'Mem/IncrementalMemory': 'true' if mode == 'mapping' else 'false',
        'Mem/InitWMWithAllNodes': 'false' if mode == 'mapping' else 'true',
        # Keep the same no-auto-reset behavior used by the front-camera
        # standalone mapping path. Bad/blank rear frames should not force a new
        # session or make RTAB-Map forget the current database.
        'Odom/ResetCountdown': '0',
    }

    actions = [
        LogInfo(msg=(
            f'RTAB-Map {mode}; database: {database_path}; '
            f'mapping_camera={mapping_camera}; reset_database={reset}.')),
    ]

    if mapping_camera in ('front', 'rear'):
        camera = CAMERA_TOPICS[mapping_camera]
        mode_parameters.update({
            'rgbd_cameras': 1,
            'subscribe_rgbd': True,
        })
        actions.extend([
            rgbd_sync_node(mapping_camera, config),
            Node(
                package='rtabmap_slam',
                executable='rtabmap',
                name='rtabmap',
                output='screen',
                parameters=[config, mode_parameters],
                arguments=['--delete_db_on_start'] if reset else [],
                remappings=[
                    ('rgbd_image', camera['rgbd']),
                    ('odom', '/odom'),
                ],
            ),
        ])
        return actions

    mode_parameters.update({
        'rgbd_cameras': 0,
        'subscribe_rgbd': True,
    })
    actions.extend([
        rgbd_sync_node('front', config),
        rgbd_sync_node('rear', config),
        Node(
            package='rtabmap_sync',
            executable='rgbdx_sync',
            name='rgbdx_sync',
            output='screen',
            parameters=[config],
            remappings=[
                ('rgbd_image0', '/front_rgbd_image'),
                ('rgbd_image1', '/rear_rgbd_image'),
                ('rgbd_images', '/rgbd_images'),
            ],
        ),
        Node(
            package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
            parameters=[config, mode_parameters],
            arguments=['--delete_db_on_start'] if reset else [],
            remappings=[
                ('rgbd_images', '/rgbd_images'),
                ('odom', '/odom'),
            ],
        ),
    ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping', choices=['mapping', 'localization']),
        DeclareLaunchArgument('database_path', default_value='/ros2_ws/maps/bunker_dual_rgbd.db'),
        DeclareLaunchArgument('reset_database', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument(
            'mapping_camera',
            default_value='both',
            choices=['front', 'rear', 'both'],
            description=(
                'front/rear use the same single-camera RTAB-Map path; both '
                'uses synchronized front+rear RGBDImages. Use rear when the '
                'rear camera should extend the existing DB without depending '
                'on the front view.')),
        OpaqueFunction(function=launch_rtabmap),
    ])
