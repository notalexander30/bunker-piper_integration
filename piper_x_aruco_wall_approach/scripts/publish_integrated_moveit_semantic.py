#!/usr/bin/env python3
"""Publish the SRDF matching Trystan's prefixed full-system MoveIt model."""

from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class IntegratedSemanticPublisher(Node):
    def __init__(self) -> None:
        super().__init__("front_piper_integrated_moveit_semantic")
        self.declare_parameter("topic", "/front_piper/integrated_robot_description_semantic")
        package_root = Path(get_package_share_directory("piper_x_aruco_wall_approach"))
        semantic_path = package_root / "config" / "front_piper_integrated_moveit.srdf"
        self.semantic = semantic_path.read_text(encoding="utf-8")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            String, str(self.get_parameter("topic").value), qos
        )
        self.message = String(data=self.semantic)
        self.timer = self.create_timer(1.0, self.publish_description)
        self.publish_description()

    def publish_description(self) -> None:
        self.publisher.publish(self.message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IntegratedSemanticPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
