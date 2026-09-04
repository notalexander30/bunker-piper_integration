import os
from glob import glob

from setuptools import setup


package_name = 'bunker_autonomy'
include_remote_vlm = os.environ.get(
    'BUNKER_INCLUDE_REMOTE_VLM', 'true'
).strip().lower() == 'true'


def regular_files(pattern):
    return [path for path in glob(pattern) if os.path.isfile(path)]

console_scripts = [
    'safety_monitor_node = bunker_autonomy.safety_monitor_node:main',
    'depth_route_monitor_node = bunker_autonomy.depth_route_monitor_node:main',
    'sensor_fusion_node = bunker_autonomy.sensor_fusion_node:main',
    'cmd_vel_mux_node = bunker_autonomy.cmd_vel_mux_node:main',
    'operator_status_node = bunker_autonomy.operator_status_node:main',
    'data_logger_node = bunker_autonomy.data_logger_node:main',
    'label_autonomy_run = bunker_autonomy.label_autonomy_run:main',
    'show_safety_events = bunker_autonomy.show_safety_events:main',
    'realsense_usb_recover = bunker_autonomy.realsense_usb_recover:main',
    'realsense_stream_watchdog = bunker_autonomy.realsense_stream_watchdog:main',
    'd435i_calibration_node = bunker_autonomy.d435i_calibration_node:main',
    'd435i_calibration_info = bunker_autonomy.d435i_calibration_info:main',
    'piper_navigation_pose = bunker_autonomy.piper_navigation_pose:main',
    'piper_x_joint_preset = bunker_autonomy.piper_x_joint_preset:main',
    'clicked_point_nav_goal = bunker_autonomy.clicked_point_nav_goal:main',
    'frontier_explorer_node = bunker_autonomy.frontier_explorer_node:main',
    'aruco_landmark_node = bunker_autonomy.aruco_landmark_node:main',
    'landmark_navigator_node = bunker_autonomy.landmark_navigator_node:main',
    'nav2_status_monitor = bunker_autonomy.nav2_status_monitor:main',
    'nav2_arrival_manipulation_trigger = bunker_autonomy.nav2_arrival_manipulation_trigger:main',
]
if include_remote_vlm:
    console_scripts.extend([
        'vlm_goal_monitor_node = bunker_autonomy.vlm_goal_monitor_node:main',
        'search_behavior_node = bunker_autonomy.search_behavior_node:main',
    ])

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, [
            'package.xml',
            'README_autonomy.md',
            'PIPER_X_JOINT_PRESET.md',
            'NAV2_MANIPULATION_HANDOFF.md',
        ]),
        (
            os.path.join('share', package_name, 'config'),
            regular_files('config/*.yaml') + regular_files('config/*.rviz'),
        ),
        (
            os.path.join('share', package_name, 'launch'),
            regular_files('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Autonomy Maintainer',
    maintainer_email='user@todo.todo',
    description='First-stage safe autonomy nodes for AgileX Bunker Mini search behavior.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': console_scripts,
    },
)
