#!/usr/bin/env python3
"""Feedback-aware FollowJointTrajectory bridge for the existing PiPER driver.

Unlike the stock MoveIt GenericSystem, this node publishes no idle commands.
It forwards interpolated JointState commands only while an accepted action is
executing and uses the physical arm feedback for start and goal validation.
"""

import math
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


def duration_seconds(duration):
    return float(duration.sec) + float(duration.nanosec) / 1e9


class FrontPiperTrajectoryBridge(Node):
    ARM_JOINTS = tuple(f'joint{index}' for index in range(1, 7))
    PREFIXED_ARM_JOINTS = tuple(f'front_piper_joint{index}' for index in range(1, 7))
    GRIPPER_JOINTS = ('gripper',)
    PREFIXED_TO_DRIVER = {
        f'front_piper_joint{index}': f'joint{index}' for index in range(1, 7)
    }
    PREFIXED_TO_DRIVER.update({
        'front_piper_joint7': 'gripper',
        'front_piper_joint8': 'gripper',
    })

    def __init__(self):
        super().__init__('front_piper_trajectory_bridge')
        self.declare_parameter(
            'feedback_topic', '/front_piper/feedback/joint_states')
        self.declare_parameter(
            'command_topic', '/front_piper/control/joint_states')
        self.declare_parameter('command_rate_hz', 50.0)
        self.declare_parameter('feedback_timeout_sec', 1.0)
        self.declare_parameter('start_tolerance_rad', 0.20)
        self.declare_parameter('goal_tolerance_rad', 0.06)
        self.declare_parameter('goal_timeout_sec', 5.0)

        self.command_rate_hz = float(
            self.get_parameter('command_rate_hz').value)
        self.feedback_timeout_sec = float(
            self.get_parameter('feedback_timeout_sec').value)
        self.start_tolerance_rad = float(
            self.get_parameter('start_tolerance_rad').value)
        self.goal_tolerance_rad = float(
            self.get_parameter('goal_tolerance_rad').value)
        self.goal_timeout_sec = float(
            self.get_parameter('goal_timeout_sec').value)
        self._feedback = {}
        self._feedback_time = 0.0
        self._lock = threading.Lock()
        self._active_lock = threading.Lock()
        callback_group = ReentrantCallbackGroup()
        self.create_subscription(
            JointState,
            str(self.get_parameter('feedback_topic').value),
            self._feedback_callback,
            20,
            callback_group=callback_group,
        )
        self.command_pub = self.create_publisher(
            JointState,
            str(self.get_parameter('command_topic').value),
            10,
        )
        self.arm_server = self._make_server(
            '/front_piper/arm_controller/follow_joint_trajectory',
            self.ARM_JOINTS + self.PREFIXED_ARM_JOINTS,
            callback_group,
        )
        self.gripper_server = self._make_server(
            '/front_piper/gripper_controller/follow_joint_trajectory',
            self.GRIPPER_JOINTS,
            callback_group,
        )
        self.get_logger().info(
            'Front PiPER trajectory bridge ready; idle command publication is disabled')

    def _make_server(self, action_name, allowed_joints, callback_group):
        return ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            execute_callback=lambda handle: self._execute(handle, allowed_joints),
            goal_callback=lambda request: self._goal(request, allowed_joints),
            cancel_callback=self._cancel,
            callback_group=callback_group,
        )

    def _feedback_callback(self, message):
        with self._lock:
            for name, position in zip(message.name, message.position):
                if math.isfinite(position):
                    self._feedback[name] = float(position)
                    self._feedback_time = time.monotonic()

    def _driver_names(self, names):
        return tuple(self.PREFIXED_TO_DRIVER.get(name, name) for name in names)

    def _snapshot(self, names):
        with self._lock:
            age = time.monotonic() - self._feedback_time
            if age > self.feedback_timeout_sec:
                return None
            driver_names = self._driver_names(names)
            if any(name not in self._feedback for name in driver_names):
                return None
            return [self._feedback[name] for name in driver_names]

    def _goal(self, request, allowed_joints):
        trajectory = request.trajectory
        names = tuple(trajectory.joint_names)
        if not names or not set(names).issubset(set(allowed_joints)):
            self.get_logger().error(
                f'Rejecting trajectory with unsupported joints: {list(names)}')
            return GoalResponse.REJECT
        if not trajectory.points:
            self.get_logger().error('Rejecting empty trajectory')
            return GoalResponse.REJECT
        previous_time = -1.0
        for point in trajectory.points:
            point_time = duration_seconds(point.time_from_start)
            if (len(point.positions) != len(names) or point_time < previous_time or
                    not all(math.isfinite(value) for value in point.positions)):
                self.get_logger().error('Rejecting malformed trajectory')
                return GoalResponse.REJECT
            previous_time = point_time
        if self._snapshot(names) is None:
            self.get_logger().error('Rejecting trajectory: physical feedback is stale')
            return GoalResponse.REJECT
        if not self._active_lock.acquire(blocking=False):
            self.get_logger().error('Rejecting trajectory: another command is active')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel(_goal_handle):
        return CancelResponse.ACCEPT

    @staticmethod
    def _interpolate(start, points, times, elapsed):
        if elapsed <= times[0]:
            fraction = 1.0 if times[0] <= 0.0 else elapsed / times[0]
            return [a + (b - a) * fraction
                    for a, b in zip(start, points[0].positions)]
        for index in range(1, len(points)):
            if elapsed <= times[index]:
                span = max(times[index] - times[index - 1], 1e-9)
                fraction = (elapsed - times[index - 1]) / span
                return [a + (b - a) * fraction for a, b in zip(
                    points[index - 1].positions, points[index].positions)]
        return list(points[-1].positions)

    def _result(self, code, text):
        result = FollowJointTrajectory.Result()
        result.error_code = code
        result.error_string = text
        return result

    def _publish(self, names, positions):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self._driver_names(names))
        message.position = list(positions)
        self.command_pub.publish(message)

    def _execute(self, goal_handle, _allowed_joints):
        try:
            trajectory = goal_handle.request.trajectory
            names = tuple(trajectory.joint_names)
            start = self._snapshot(names)
            if start is None:
                goal_handle.abort()
                return self._result(
                    FollowJointTrajectory.Result.INVALID_GOAL,
                    'Physical feedback became stale before execution',
                )
            first = trajectory.points[0].positions
            if max(abs(actual - planned) for actual, planned in zip(start, first)) \
                    > self.start_tolerance_rad:
                goal_handle.abort()
                return self._result(
                    FollowJointTrajectory.Result.INVALID_GOAL,
                    'Trajectory start differs from physical feedback',
                )

            points = trajectory.points
            times = [duration_seconds(point.time_from_start) for point in points]
            final_time = times[-1]
            begin = time.monotonic()
            period = 1.0 / max(self.command_rate_hz, 1.0)
            while True:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self._result(
                        FollowJointTrajectory.Result.SUCCESSFUL,
                        'Trajectory canceled; no further commands published',
                    )
                elapsed = time.monotonic() - begin
                desired = self._interpolate(start, points, times, elapsed)
                self._publish(names, desired)
                actual = self._snapshot(names)
                feedback = FollowJointTrajectory.Feedback()
                feedback.header.stamp = self.get_clock().now().to_msg()
                feedback.joint_names = list(names)
                feedback.desired = JointTrajectoryPoint(positions=list(desired))
                if actual is not None:
                    feedback.actual = JointTrajectoryPoint(positions=list(actual))
                    feedback.error = JointTrajectoryPoint(
                        positions=[d - a for d, a in zip(desired, actual)])
                goal_handle.publish_feedback(feedback)
                if elapsed >= final_time:
                    break
                time.sleep(period)

            deadline = time.monotonic() + self.goal_timeout_sec
            final = list(points[-1].positions)
            while time.monotonic() < deadline:
                actual = self._snapshot(names)
                if actual is not None and max(
                        abs(a - b) for a, b in zip(actual, final)) \
                        <= self.goal_tolerance_rad:
                    goal_handle.succeed()
                    return self._result(
                        FollowJointTrajectory.Result.SUCCESSFUL,
                        'Physical feedback reached the trajectory goal',
                    )
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self._result(
                        FollowJointTrajectory.Result.SUCCESSFUL,
                        'Trajectory canceled while waiting for goal feedback',
                    )
                self._publish(names, final)
                time.sleep(period)
            goal_handle.abort()
            return self._result(
                FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
                'Physical feedback did not reach the trajectory goal in time',
            )
        finally:
            self._active_lock.release()

    def destroy_node(self):
        self.arm_server.destroy()
        self.gripper_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FrontPiperTrajectoryBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
