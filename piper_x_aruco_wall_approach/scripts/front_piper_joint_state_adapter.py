#!/usr/bin/env python3
"""Republish integrated front-PiPER joint states with single-arm joint names."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class FrontPiperJointStateAdapter(Node):
    def __init__(self) -> None:
        super().__init__("front_piper_joint_state_adapter")

        self.declare_parameter("input_topic", "/joint_states")
        self.declare_parameter("output_topic", "/front_piper/feedback/joint_states")
        self.declare_parameter("source_prefix", "front_piper_")
        self.declare_parameter("output_joint_names", ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.source_prefix = str(self.get_parameter("source_prefix").value)
        self.output_joint_names = [str(name) for name in self.get_parameter("output_joint_names").value]

        if not self.output_joint_names:
            raise ValueError("output_joint_names must not be empty")

        self.publisher = self.create_publisher(JointState, self.output_topic, 10)
        self.subscription = self.create_subscription(
            JointState,
            self.input_topic,
            self._callback,
            10,
        )
        self._missing_log_time = self.get_clock().now()
        self._published_once = False

        self.get_logger().info(
            "Adapting %s %s%s -> %s %s"
            % (
                self.input_topic,
                self.source_prefix,
                self.output_joint_names,
                self.output_topic,
                self.output_joint_names,
            )
        )

    def _callback(self, msg: JointState) -> None:
        positions_by_name = dict(zip(msg.name, msg.position))
        velocities_by_name = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
        efforts_by_name = dict(zip(msg.name, msg.effort)) if msg.effort else {}

        source_names = [self.source_prefix + name for name in self.output_joint_names]
        missing = [source for source in source_names if source not in positions_by_name]
        if missing:
            now = self.get_clock().now()
            if (now - self._missing_log_time).nanoseconds > 2_000_000_000:
                self.get_logger().warn(
                    "Cannot adapt joint state; missing integrated joints: %s" % missing
                )
                self._missing_log_time = now
            return

        out = JointState()
        out.header = msg.header
        out.name = list(self.output_joint_names)
        out.position = [float(positions_by_name[source]) for source in source_names]

        if velocities_by_name:
            out.velocity = [float(velocities_by_name.get(source, 0.0)) for source in source_names]
        if efforts_by_name:
            out.effort = [float(efforts_by_name.get(source, 0.0)) for source in source_names]

        self.publisher.publish(out)
        if not self._published_once:
            self.get_logger().info(
                "Publishing adapted front PiPER joint states on %s" % self.output_topic
            )
            self._published_once = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontPiperJointStateAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
