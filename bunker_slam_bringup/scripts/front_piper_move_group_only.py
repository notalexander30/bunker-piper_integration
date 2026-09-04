#!/usr/bin/env python3
"""Start front_piper MoveIt using the real prefixed Nav-Man URDF branch."""

import os
import sys
from pathlib import Path

from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).parent))
from front_piper_integrated_moveit_model import moveit_parameters  # noqa: E402


def main() -> int:
    params = [
        *moveit_parameters(),
        {
            "publish_robot_description": False,
            "publish_robot_description_semantic": True,
            "allow_trajectory_execution": True,
            "publish_planning_scene": True,
            # Collision objects still work through the planning-scene API.
            # Disable Octomap geometry monitoring because this image has no
            # occupancy-map updater plugin and the marker task does not use it.
            "publish_geometry_updates": False,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
            "monitor_dynamics": False,
        },
    ]
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace="front_piper",
        output="screen",
        parameters=params,
        remappings=[("joint_states", "/joint_states")],
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
    )
    launch_service = LaunchService()
    launch_service.include_launch_description(LaunchDescription([move_group]))
    return launch_service.run()


if __name__ == "__main__":
    raise SystemExit(main())
