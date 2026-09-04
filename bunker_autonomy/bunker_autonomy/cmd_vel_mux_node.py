#!/usr/bin/env python3

import math

from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


def zero_twist() -> Twist:
    return Twist()


def clone_twist(command: Twist) -> Twist:
    cloned = Twist()
    cloned.linear.x = command.linear.x
    cloned.linear.y = command.linear.y
    cloned.linear.z = command.linear.z
    cloned.angular.x = command.angular.x
    cloned.angular.y = command.angular.y
    cloned.angular.z = command.angular.z
    return cloned


def select_mux_command(
    latest_command: Optional[Twist],
    safety_stop: bool,
    command_age_sec: Optional[float],
    timeout_sec: float,
    allow_reverse: bool = False,
) -> Tuple[Twist, str]:
    negative_requested = bool(
        latest_command is not None
        and latest_command.linear.x < 0.0
    )
    if negative_requested and not allow_reverse:
        return zero_twist(), 'invalid_reverse'
    if safety_stop:
        return zero_twist(), 'safety_stop'
    if latest_command is None:
        return zero_twist(), 'no_command'
    if command_age_sec is None or command_age_sec > timeout_sec:
        return zero_twist(), 'watchdog_timeout'
    return clone_twist(latest_command), 'forward'


def apply_motion_policy(
    command: Twist,
    allow_reverse: bool,
    max_linear_speed: Optional[float] = None,
    max_angular_speed: Optional[float] = None,
) -> Twist:
    """Enforce the final chassis-direction policy after all upstream commands."""
    filtered = clone_twist(command)
    if not math.isfinite(filtered.linear.x) or not math.isfinite(filtered.angular.z):
        return zero_twist()
    if not allow_reverse and filtered.linear.x < 0.0:
        filtered.linear.x = 0.0
    if max_linear_speed is not None:
        limit = max(0.0, float(max_linear_speed))
        filtered.linear.x = max(-limit, min(limit, filtered.linear.x))
    if max_angular_speed is not None:
        limit = max(0.0, float(max_angular_speed))
        filtered.angular.z = max(-limit, min(limit, filtered.angular.z))
    # BUNKER MINI is differential drive; reject unsupported action dimensions.
    filtered.linear.y = 0.0
    filtered.linear.z = 0.0
    filtered.angular.x = 0.0
    filtered.angular.y = 0.0
    return filtered


def _limit_axis_rate(
    current: float,
    target: float,
    dt_sec: float,
    acceleration: float,
    deceleration: float,
) -> float:
    """Move one velocity axis toward its target without crossing zero on reversal."""
    dt_sec = max(0.0, min(float(dt_sec), 0.25))
    acceleration = max(0.0, float(acceleration))
    deceleration = max(0.0, float(deceleration))

    if abs(target - current) <= 1e-9:
        return float(target)

    # A direction reversal first brakes to zero. Acceleration in the opposite
    # direction begins on a later publish cycle.
    if current * target < 0.0:
        step = deceleration * dt_sec
        if abs(current) <= step:
            return 0.0
        return current - step if current > 0.0 else current + step

    speeding_up = abs(target) > abs(current)
    rate = acceleration if speeding_up else deceleration
    max_delta = rate * dt_sec
    delta = target - current
    if abs(delta) <= max_delta:
        return float(target)
    return current + max_delta if delta > 0.0 else current - max_delta


def limit_twist_rate(
    current: Twist,
    target: Twist,
    dt_sec: float,
    linear_acceleration: float,
    linear_deceleration: float,
    angular_acceleration: float,
    angular_deceleration: float,
    immediate_stop: bool = False,
) -> Twist:
    """Rate-limit differential-drive velocity while preserving emergency stops."""
    if immediate_stop:
        return zero_twist()

    limited = clone_twist(target)
    limited.linear.x = _limit_axis_rate(
        current.linear.x,
        target.linear.x,
        dt_sec,
        linear_acceleration,
        linear_deceleration,
    )
    limited.angular.z = _limit_axis_rate(
        current.angular.z,
        target.angular.z,
        dt_sec,
        angular_acceleration,
        angular_deceleration,
    )
    return limited


class CmdVelMuxNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux_node')

        self.declare_parameter('cmd_vel_autonomy_topic', '/cmd_vel_autonomy')
        self.declare_parameter('safety_stop_topic', '/safety_stop')
        self.declare_parameter('output_cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('mux_reason_topic', '/cmd_vel_mux/reason')
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('initial_safety_stop', True)
        self.declare_parameter('use_safety_stop', True)
        self.declare_parameter('allow_reverse', False)
        self.declare_parameter('max_linear_speed_mps', 0.06)
        self.declare_parameter('max_angular_speed_radps', 0.15)
        self.declare_parameter('linear_acceleration_mps2', 0.06)
        self.declare_parameter('linear_deceleration_mps2', 0.125)
        self.declare_parameter('angular_acceleration_radps2', 0.175)
        self.declare_parameter('angular_deceleration_radps2', 0.35)

        cmd_vel_autonomy_topic = self.get_parameter('cmd_vel_autonomy_topic').value
        safety_stop_topic = self.get_parameter('safety_stop_topic').value
        output_cmd_vel_topic = self.get_parameter('output_cmd_vel_topic').value
        mux_reason_topic = self.get_parameter('mux_reason_topic').value
        self._command_timeout_sec = float(self.get_parameter('command_timeout_sec').value)
        publish_rate_hz = max(0.1, float(self.get_parameter('publish_rate_hz').value))
        self._linear_acceleration = max(
            0.01, float(self.get_parameter('linear_acceleration_mps2').value)
        )
        self._linear_deceleration = max(
            self._linear_acceleration,
            float(self.get_parameter('linear_deceleration_mps2').value),
        )
        self._angular_acceleration = max(
            0.01, float(self.get_parameter('angular_acceleration_radps2').value)
        )
        self._angular_deceleration = max(
            self._angular_acceleration,
            float(self.get_parameter('angular_deceleration_radps2').value),
        )
        self._safety_stop = bool(self.get_parameter('initial_safety_stop').value)
        self._use_safety_stop = bool(self.get_parameter('use_safety_stop').value)
        self._allow_reverse = bool(self.get_parameter('allow_reverse').value)
        self._max_linear_speed = max(
            0.0, float(self.get_parameter('max_linear_speed_mps').value)
        )
        self._max_angular_speed = max(
            0.0, float(self.get_parameter('max_angular_speed_radps').value)
        )
        self._latest_command: Optional[Twist] = None
        self._latest_command_time = None
        self._last_reason = None
        self._last_output = zero_twist()
        self._last_publish_time = self.get_clock().now()

        self._cmd_pub = self.create_publisher(Twist, output_cmd_vel_topic, 10)
        self._reason_pub = self.create_publisher(String, mux_reason_topic, 10)
        self._cmd_sub = self.create_subscription(
            Twist,
            cmd_vel_autonomy_topic,
            self.cmd_vel_autonomy_callback,
            10,
        )
        if self._use_safety_stop:
            self._safety_sub = self.create_subscription(
                Bool,
                safety_stop_topic,
                self.safety_stop_callback,
                10,
            )
        else:
            self._safety_sub = None
        self._timer = self.create_timer(1.0 / publish_rate_hz, self.publish_selected_command)

        self.get_logger().info(
            'Cmd vel mux started: cmd_vel_autonomy_topic=%s, safety_stop_topic=%s, '
            'output_cmd_vel_topic=%s, mux_reason_topic=%s, command_timeout_sec=%.3f, '
            'use_safety_stop=%s, linear_accel/decel=%.2f/%.2f m/s^2, '
            'angular_accel/decel=%.2f/%.2f rad/s^2'
            % (
                cmd_vel_autonomy_topic,
                safety_stop_topic,
                output_cmd_vel_topic,
                mux_reason_topic,
                self._command_timeout_sec,
                self._use_safety_stop,
                self._linear_acceleration,
                self._linear_deceleration,
                self._angular_acceleration,
                self._angular_deceleration,
            )
        )

    def cmd_vel_autonomy_callback(self, msg: Twist) -> None:
        self._latest_command = clone_twist(msg)
        self._latest_command_time = self.get_clock().now()
        self.publish_selected_command()

    def safety_stop_callback(self, msg: Bool) -> None:
        previous = self._safety_stop
        self._safety_stop = bool(msg.data)
        if previous != self._safety_stop:
            self.get_logger().info('safety_stop changed to %s' % self._safety_stop)
        if self._safety_stop:
            self._last_output = zero_twist()
            self._last_publish_time = self.get_clock().now()
            self._cmd_pub.publish(self._last_output)

    def _command_age_sec(self) -> Optional[float]:
        if self._latest_command_time is None:
            return None
        return (self.get_clock().now() - self._latest_command_time).nanoseconds / 1e9

    def publish_selected_command(self) -> None:
        target, reason = select_mux_command(
            self._latest_command,
            self._safety_stop if self._use_safety_stop else False,
            self._command_age_sec(),
            self._command_timeout_sec,
            self._allow_reverse,
        )
        target = apply_motion_policy(
            target,
            self._allow_reverse,
            self._max_linear_speed,
            self._max_angular_speed,
        )
        now = self.get_clock().now()
        dt_sec = (now - self._last_publish_time).nanoseconds / 1e9
        command = limit_twist_rate(
            self._last_output,
            target,
            dt_sec,
            self._linear_acceleration,
            self._linear_deceleration,
            self._angular_acceleration,
            self._angular_deceleration,
            immediate_stop=reason in (
                'safety_stop',
                'watchdog_timeout',
                'no_command',
                'invalid_reverse',
            ),
        )
        self._cmd_pub.publish(command)
        self._last_output = clone_twist(command)
        self._last_publish_time = now

        if reason != self._last_reason:
            self.get_logger().info('Cmd vel mux output reason: %s' % reason)
            reason_msg = String()
            reason_msg.data = reason
            self._reason_pub.publish(reason_msg)
        elif reason in ('safety_stop', 'watchdog_timeout'):
            self.get_logger().warn(
                'Cmd vel mux holding zero due to %s' % reason,
                throttle_duration_sec=2.0,
            )
        self._last_reason = reason


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
