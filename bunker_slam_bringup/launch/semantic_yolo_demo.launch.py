"""Stationary RGB-D YOLO and RTAB-Map-frame semantic-memory demonstration.

This launch starts localization, a chair-only YOLO detector, and semantic
memory. It intentionally does not include a follower controller, command mux,
or any node that publishes ``/cmd_vel``.
"""

import os
from pathlib import Path

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def default_database_path():
    prefix = Path(get_package_prefix("bunker_slam_bringup"))
    workspace = prefix.parent.parent if prefix.parent.name == "install" else Path("/ros2_ws")
    return str(workspace / "src" / "bunker_slam_bringup" / "maps" / "bunker_acceptance.db")


def generate_launch_description():
    bringup_share = get_package_share_directory("bunker_slam_bringup")
    follower_share = get_package_share_directory("bunker_object_follower")
    semantic_share = get_package_share_directory("semantic_memory")
    localization_launch = os.path.join(
        bringup_share, "launch", "localization_bringup.launch.py"
    )
    semantic_launch = os.path.join(
        semantic_share, "launch", "semantic_memory.launch.py"
    )
    chair_config = os.path.join(
        follower_share, "config", "semantic_memory_chair_d435i.yaml"
    )
    rviz_config = os.path.join(bringup_share, "rviz", "semantic_yolo_demo.rviz")

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        launch_arguments={
            "database_path": LaunchConfiguration("database_path"),
            "bunker_port": LaunchConfiguration("bunker_port"),
            "arm_can_port": LaunchConfiguration("arm_can_port"),
            "serial_no": LaunchConfiguration("serial_no"),
            "configure_can": LaunchConfiguration("configure_can"),
            "start_base": "true",
            "start_arm": "true",
            "start_camera": "true",
            "use_rviz": "false",
            "imu_gate_timeout": LaunchConfiguration("imu_gate_timeout"),
        }.items(),
    )
    semantic_memory = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(semantic_launch),
        launch_arguments={
            "memory_file": LaunchConfiguration("memory_file"),
            "rtabmap_database_path": LaunchConfiguration("database_path"),
            "yolo_detection_topic": "/object_follower/local_target_status",
        }.items(),
    )
    detector = Node(
        package="bunker_object_follower",
        executable="detector_tracker_node",
        name="detector_tracker_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("detector_config"),
            {
                "device": ParameterValue(
                    LaunchConfiguration("detector_device"), value_type=str
                )
            },
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="semantic_yolo_demo_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        additional_env={"QT_X11_NO_MITSHM": "1"},
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("database_path", default_value=default_database_path()),
            DeclareLaunchArgument(
                "memory_file", default_value="/root/vlm_results/semantic_demo.json"
            ),
            DeclareLaunchArgument("bunker_port", default_value="can3"),
            DeclareLaunchArgument("arm_can_port", default_value="can2"),
            DeclareLaunchArgument("serial_no", default_value="261222075829"),
            DeclareLaunchArgument("configure_can", default_value="false"),
            DeclareLaunchArgument("imu_gate_timeout", default_value="30.0"),
            DeclareLaunchArgument("detector_config", default_value=chair_config),
            DeclareLaunchArgument("detector_device", default_value="0"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            LogInfo(
                msg=(
                    "Semantic YOLO demo: feedback-only localization + chair YOLO "
                    "+ map-frame semantic memory. No controller and no cmd_vel "
                    "publisher are started."
                )
            ),
            localization,
            semantic_memory,
            detector,
            rviz,
        ]
    )
