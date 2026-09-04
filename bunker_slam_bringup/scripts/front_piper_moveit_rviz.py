#!/usr/bin/env python3
"""Start a dedicated RViz window for the front PiPER MoveIt instance."""

import os
import sys
from pathlib import Path

from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).parent))
from front_piper_integrated_moveit_model import moveit_parameters


def main() -> int:
    rviz_config = (
        Path("/ros2_ws/src/bunker_slam_bringup/rviz/front_piper_moveit.rviz")
        if Path("/ros2_ws/src/bunker_slam_bringup/rviz/front_piper_moveit.rviz").is_file()
        else Path("/ros2_ws/install/bunker_slam_bringup/share/bunker_slam_bringup/rviz/front_piper_moveit.rviz")
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="front_piper_moveit_rviz2",
        output="screen",
        arguments=["-d", str(rviz_config)],
        parameters=moveit_parameters(),
        remappings=[("joint_states", "/joint_states")],
        additional_env={
            "DISPLAY": os.environ.get("DISPLAY", ":1"),
            "QT_QPA_PLATFORM": "xcb",
            "QT_X11_NO_MITSHM": "1",
        },
    )
    launch_service = LaunchService()
    launch_service.include_launch_description(LaunchDescription([rviz]))
    return launch_service.run()


if __name__ == "__main__":
    raise SystemExit(main())
