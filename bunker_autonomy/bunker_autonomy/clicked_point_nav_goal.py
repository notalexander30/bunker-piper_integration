#!/usr/bin/env python3
"""Convert RViz Publish Point clicks into Nav2 NavigateToPose goals.

RViz's normal Nav2 goal tool requires dragging an arrow to set yaw. For this
robot that is often slower than useful. This node lets the operator use the
Publish Point tool instead: one click sends a goal at that map position, with
the goal yaw copied from the robot's current yaw.
"""

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class ClickedPointNavGoal(Node):
    def __init__(self) -> None:
        super().__init__('clicked_point_nav_goal')
        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('action_name', '/navigate_to_pose')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('use_current_robot_yaw', True)
        self.declare_parameter('fallback_yaw_rad', 0.0)
        self.declare_parameter('wait_for_server_sec', 5.0)

        self.global_frame = self.get_parameter(
            'global_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter(
            'base_frame').get_parameter_value().string_value
        self.use_current_robot_yaw = self.get_parameter(
            'use_current_robot_yaw').get_parameter_value().bool_value
        self.fallback_yaw_rad = self.get_parameter(
            'fallback_yaw_rad').get_parameter_value().double_value

        action_name = self.get_parameter(
            'action_name').get_parameter_value().string_value
        topic = self.get_parameter(
            'clicked_point_topic').get_parameter_value().string_value
        wait_sec = self.get_parameter(
            'wait_for_server_sec').get_parameter_value().double_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = ActionClient(self, NavigateToPose, action_name)
        self.subscription = self.create_subscription(
            PointStamped, topic, self.on_clicked_point, 10)
        self.server_ready = self.client.wait_for_server(timeout_sec=wait_sec)
        if not self.server_ready:
            self.get_logger().warn(
                f'Nav2 action server {action_name} not ready yet; clicks will '
                'retry when received.')
        self.get_logger().info(
            f'Click a point in RViz on {topic} to send a Nav2 goal in '
            f'{self.global_frame}.')

    def current_yaw(self) -> float:
        if not self.use_current_robot_yaw:
            return self.fallback_yaw_rad
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, rclpy.time.Time())
            return yaw_from_quaternion(transform.transform.rotation)
        except TransformException as exc:
            self.get_logger().warn(
                f'Cannot read {self.global_frame}->{self.base_frame} yaw: '
                f'{exc}; using fallback yaw {self.fallback_yaw_rad:.3f} rad.')
            return self.fallback_yaw_rad

    def on_clicked_point(self, msg: PointStamped) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.global_frame:
            self.get_logger().warn(
                f'Clicked point frame is {msg.header.frame_id}, expected '
                f'{self.global_frame}. Set RViz Fixed Frame to '
                f'{self.global_frame}.')

        if not self.client.server_is_ready():
            self.get_logger().warn('Nav2 action server is not ready; goal ignored.')
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.global_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = msg.point.x
        goal.pose.pose.position.y = msg.point.y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = quaternion_from_yaw(self.current_yaw())

        self.get_logger().info(
            f'Sending clicked Nav2 goal: x={msg.point.x:.2f}, '
            f'y={msg.point.y:.2f}.')
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Clicked Nav2 goal was rejected.')
            return
        self.get_logger().info('Clicked Nav2 goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result)

    def _goal_result(self, future) -> None:
        result = future.result()
        self.get_logger().info(f'Clicked Nav2 goal finished with status={result.status}.')


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = ClickedPointNavGoal()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
