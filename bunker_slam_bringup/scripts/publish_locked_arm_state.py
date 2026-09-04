#!/usr/bin/env python3
"""Publish the documented stationary PiPER pose; never sends arm commands."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class LockedArmStatePublisher(Node):
    def __init__(self):
        super().__init__('locked_arm_state_publisher')
        self.declare_parameter(
            'joint_names',
            ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])
        self.declare_parameter(
            'joint_positions', [1.6, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('topic', '/joint_states')

        self.names = list(self.get_parameter('joint_names').value)
        self.positions = list(self.get_parameter('joint_positions').value)
        if len(self.names) != len(self.positions):
            raise ValueError('joint_names and joint_positions lengths differ')
        topic = str(self.get_parameter('topic').value)
        rate = float(self.get_parameter('publish_rate').value)
        self.publisher = self.create_publisher(JointState, topic, 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_state)
        self.get_logger().warning(
            'Publishing the documented LOCKED PiPER pose to %s. This is '
            'read-only TF fallback, not live arm feedback.' % topic)

    def publish_state(self):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = self.names
        message.position = self.positions
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = LockedArmStatePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
