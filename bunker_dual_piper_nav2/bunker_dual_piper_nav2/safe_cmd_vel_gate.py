"""Clamp Nav2 velocity and route it to debug or the configured autonomy topic."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class SafeCmdVelGate(Node):
    def __init__(self) -> None:
        super().__init__('safe_cmd_vel_gate')
        self.declare_parameter('input_topic', '/cmd_vel_nav2_raw')
        self.declare_parameter('output_topic', '/cmd_vel_debug')
        self.declare_parameter('max_linear_x', 0.225)
        self.declare_parameter('max_angular_z', 0.375)
        self.declare_parameter('timeout', 0.5)

        self.last_command_time = self.get_clock().now()
        self.output = self.create_publisher(
            Twist, self.get_parameter('output_topic').value, 10
        )
        self.create_subscription(
            Twist,
            self.get_parameter('input_topic').value,
            self.command_callback,
            10,
        )
        self.timer = self.create_timer(0.1, self.watchdog)
        self.stopped = True

    @staticmethod
    def clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def command_callback(self, source: Twist) -> None:
        command = Twist()
        command.linear.x = self.clamp(
            source.linear.x, float(self.get_parameter('max_linear_x').value)
        )
        command.angular.z = self.clamp(
            source.angular.z, float(self.get_parameter('max_angular_z').value)
        )
        self.output.publish(command)
        self.last_command_time = self.get_clock().now()
        self.stopped = command.linear.x == 0.0 and command.angular.z == 0.0

    def watchdog(self) -> None:
        age = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
        if age > float(self.get_parameter('timeout').value) and not self.stopped:
            self.output.publish(Twist())
            self.stopped = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafeCmdVelGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
