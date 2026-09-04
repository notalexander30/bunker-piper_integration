#!/usr/bin/env python3
"""Publish integrated front_piper MoveIt descriptions for Illiyas clients."""

import sys
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).parent))
from front_piper_integrated_moveit_model import (  # noqa: E402
    robot_description,
    robot_description_semantic,
)


def main() -> int:
    description = robot_description()
    semantic = robot_description_semantic()
    rclpy.init()
    node = rclpy.create_node("front_piper_description_bridge")
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    description_pub = node.create_publisher(
        String,
        "/front_piper/robot_description",
        qos,
    )
    semantic_pub = node.create_publisher(
        String,
        "/front_piper/robot_description_semantic",
        qos,
    )
    description_msg = String(data=description)
    semantic_msg = String(data=semantic)
    try:
        while rclpy.ok():
            description_pub.publish(description_msg)
            semantic_pub.publish(semantic_msg)
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(1.0)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
