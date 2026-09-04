#!/usr/bin/env python3

import json
import re
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Bool, String


@dataclass(frozen=True)
class OperatorState:
    has_safety_status: bool
    safety_status_stale: bool
    safety_stop: bool
    safety_detail: str
    has_target_status: bool
    target_status_stale: bool
    target_detected: bool
    command_active: bool
    mission_complete: bool
    mission_status: str
    mode: str
    output_cmd_vel_topic: str
    output_subscription_count: int
    navigation_mode: str = 'lidar_depth'


def twist_is_active(msg: Optional[Twist]) -> bool:
    if msg is None:
        return False

    values = (
        msg.linear.x,
        msg.linear.y,
        msg.linear.z,
        msg.angular.x,
        msg.angular.y,
        msg.angular.z,
    )
    return any(abs(value) > 1e-6 for value in values)


def notice_dedup_key(notice: str) -> str:
    """Ignore changing point counts when deciding whether a stop cause changed."""
    if 'LiDAR trigger=' not in notice:
        return notice
    key = re.sub(r'points=\d+', 'points=*', notice)
    return re.sub(r'\b(front|back|left|right)=\d+', r'\1=*', key)


def summarize_safety_detail(text: str) -> str:
    """Reduce fused route JSON to one actionable operator-facing reason."""
    try:
        status = json.loads(text or '{}')
    except json.JSONDecodeError:
        return 'safety blocked'
    if not isinstance(status, dict):
        return 'safety blocked'

    primary_reason = str(status.get('primary_stop_reason', '')).strip()
    reason_labels = {
        'lidar_stale': 'LiDAR data stale',
        'depth_stale': 'depth data stale',
        'lidar_obstacle': 'LiDAR obstacle',
        'depth_obstacle': 'depth obstacle',
        'depth_close_obstacle': 'depth close obstacle',
        'depth_insufficient_valid_data': 'depth image invalid/insufficient',
        'depth_clear_confirmation_pending': 'depth confirming clear path',
        'depth_input_stale': 'depth input stale',
        'depth_decode_error': 'depth decode error',
        'lidar_transform_invalid': 'LiDAR transform invalid',
        'lidar_empty_frame': 'LiDAR frame missing',
    }
    if primary_reason == 'lidar_obstacle':
        diagnostics = status.get('lidar_diagnostics', {})
        if isinstance(diagnostics, dict) and diagnostics:
            count = int(diagnostics.get('hard_stop_count', 0) or 0)
            threshold = int(
                diagnostics.get('hard_stop_point_threshold', 0) or 0
            )
            region = diagnostics.get('hard_stop_region', {})
            if not isinstance(region, dict):
                region = {}
            name = str(region.get('name') or 'hard_stop_box(front)')
            frame = str(region.get('frame_id') or 'base_link')
            z_text = 'ignored'
            if region.get('z_filter_enabled') is True:
                z_text = '[%.2f,%.2f]m' % (
                    float(region.get('z_min', 0.0) or 0.0),
                    float(region.get('z_max', 0.0) or 0.0),
                )
            route_counts = diagnostics.get('route_counts', {})
            if not isinstance(route_counts, dict):
                route_counts = {}
            routes = ' '.join(
                '%s=%d' % (route, int(route_counts.get(route, 0) or 0))
                for route in ('front', 'back', 'left', 'right')
            )
            return (
                'LiDAR trigger=%s | points=%d > threshold=%d | '
                '%s x[%.2f,%.2f]m y[%.2f,%.2f]m z=%s | route_points %s'
                % (
                    name,
                    count,
                    threshold,
                    frame,
                    float(region.get('x_min', 0.0) or 0.0),
                    float(region.get('x_max', 0.0) or 0.0),
                    float(region.get('y_min', 0.0) or 0.0),
                    float(region.get('y_max', 0.0) or 0.0),
                    z_text,
                    routes,
                )
            )

    if primary_reason and primary_reason != 'clear':
        return reason_labels.get(primary_reason, primary_reason.replace('_', ' '))

    sources = status.get('sources', {})
    if not isinstance(sources, dict):
        sources = {}

    stale = []
    if sources.get('lidar_fresh') is False:
        stale.append('LiDAR')
    if sources.get('depth_fresh') is False:
        stale.append('depth')
    if stale:
        return '%s data stale' % '+'.join(stale)

    blocked = []
    if sources.get('lidar_stop') is True:
        blocked.append('LiDAR')
    if sources.get('depth_stop') is True:
        blocked.append('depth')
    if blocked:
        return '%s obstacle' % '+'.join(blocked)

    if status.get('transform_ok') is False:
        return 'sensor transform/input invalid'
    return 'safety blocked'


def build_operator_notice(state: OperatorState) -> str:
    mode = (state.mode or 'dry_run').lower()
    dry_run = mode != 'drive' or state.output_cmd_vel_topic != '/cmd_vel'
    safety_nodes = (
        'D435i depth node'
        if state.navigation_mode == 'depth_only'
        else 'LiDAR+depth nodes'
    )

    if state.mission_complete:
        return state.mission_status or 'MISSION ACCOMPLISHED: robot stopped.'

    if not state.has_safety_status:
        return 'WAIT | safety data missing | check %s' % safety_nodes

    if state.safety_status_stale:
        return 'WAIT | safety data stale | check %s' % safety_nodes

    if state.safety_stop:
        return 'STOP | %s' % (state.safety_detail or 'safety blocked')

    if not state.has_target_status:
        return 'WAIT | camera/VLM data missing'

    if state.target_status_stale:
        return 'WAIT | camera/VLM data stale'

    if (
        not dry_run
        and state.command_active
        and state.output_subscription_count == 0
    ):
        return 'ERROR | chassis not listening on /cmd_vel | start bunker_base'

    if state.mission_status:
        prefix = 'DRY RUN' if dry_run else 'DRIVE'
        return '%s | %s' % (prefix, state.mission_status)

    if not state.command_active:
        return 'READY | safety clear | waiting for search command'

    if dry_run:
        return 'DRY RUN | command active | robot will not move'

    return 'DRIVE | command active'


class OperatorStatusNode(Node):
    def __init__(self):
        super().__init__('operator_status_node')

        self.declare_parameter('mode', 'dry_run')
        self.declare_parameter('navigation_mode', 'lidar_depth')
        self.declare_parameter('output_cmd_vel_topic', '/cmd_vel_debug')
        self.declare_parameter('safety_stop_topic', '/safety_stop')
        self.declare_parameter('route_status_topic', '/route_status')
        self.declare_parameter('target_detected_topic', '/target_detected')
        self.declare_parameter('target_status_topic', '/target_status')
        self.declare_parameter('mission_complete_topic', '/mission_complete')
        self.declare_parameter('mission_status_topic', '/mission_status')
        self.declare_parameter('cmd_vel_autonomy_topic', '/cmd_vel_autonomy')
        self.declare_parameter('status_topic', '/autonomy_notice')
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('repeat_notice_sec', 5.0)
        self.declare_parameter('stale_timeout_sec', 20.0)

        self._mode = str(self.get_parameter('mode').value)
        self._navigation_mode = str(
            self.get_parameter('navigation_mode').value
        ).strip().lower()
        self._output_cmd_vel_topic = str(self.get_parameter('output_cmd_vel_topic').value)
        safety_stop_topic = str(self.get_parameter('safety_stop_topic').value)
        route_status_topic = str(self.get_parameter('route_status_topic').value)
        target_detected_topic = str(self.get_parameter('target_detected_topic').value)
        target_status_topic = str(self.get_parameter('target_status_topic').value)
        mission_complete_topic = str(self.get_parameter('mission_complete_topic').value)
        mission_status_topic = str(self.get_parameter('mission_status_topic').value)
        cmd_vel_autonomy_topic = str(self.get_parameter('cmd_vel_autonomy_topic').value)
        status_topic = str(self.get_parameter('status_topic').value)
        publish_rate_hz = max(0.1, float(self.get_parameter('publish_rate_hz').value))
        self._repeat_notice_sec = max(1.0, float(self.get_parameter('repeat_notice_sec').value))
        self._stale_timeout_sec = max(1.0, float(self.get_parameter('stale_timeout_sec').value))

        status_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._status_pub = self.create_publisher(String, status_topic, status_qos)
        self.create_subscription(Bool, safety_stop_topic, self.safety_stop_callback, 10)
        self.create_subscription(String, route_status_topic, self.route_status_callback, 10)
        self.create_subscription(Bool, target_detected_topic, self.target_detected_callback, 10)
        self.create_subscription(String, target_status_topic, self.target_status_callback, 10)
        self.create_subscription(Bool, mission_complete_topic, self.mission_complete_callback, 10)
        self.create_subscription(String, mission_status_topic, self.mission_status_callback, 10)
        self.create_subscription(Twist, cmd_vel_autonomy_topic, self.cmd_vel_callback, 10)

        self._has_safety_status = False
        self._safety_stop = True
        self._safety_detail = 'safety data starting'
        self._has_target_status = False
        self._target_detected = False
        self._mission_complete = False
        self._mission_status = ''
        self._latest_command: Optional[Twist] = None
        self._last_safety_time = None
        self._last_target_time = None
        self._last_command_time = None
        self._last_notice = ''
        self._last_notice_key = ''
        self._last_notice_time = None

        self.create_timer(1.0 / publish_rate_hz, self.publish_notice)
        self.get_logger().info(
            'Operator status started: mode=%s, output_cmd_vel_topic=%s, status_topic=%s'
            % (self._mode, self._output_cmd_vel_topic, status_topic)
        )

    def safety_stop_callback(self, msg: Bool) -> None:
        self._has_safety_status = True
        self._safety_stop = bool(msg.data)
        self._last_safety_time = self.get_clock().now()

    def target_detected_callback(self, msg: Bool) -> None:
        self._has_target_status = True
        self._target_detected = bool(msg.data)
        self._last_target_time = self.get_clock().now()

    def route_status_callback(self, msg: String) -> None:
        self._safety_detail = summarize_safety_detail(msg.data)

    def target_status_callback(self, msg: String) -> None:
        self._has_target_status = True
        self._last_target_time = self.get_clock().now()

    def mission_complete_callback(self, msg: Bool) -> None:
        self._mission_complete = bool(msg.data)

    def mission_status_callback(self, msg: String) -> None:
        self._mission_status = msg.data

    def cmd_vel_callback(self, msg: Twist) -> None:
        self._latest_command = msg
        self._last_command_time = self.get_clock().now()

    def _is_stale(self, last_time) -> bool:
        if last_time is None:
            return False
        age_sec = (self.get_clock().now() - last_time).nanoseconds / 1e9
        return age_sec > self._stale_timeout_sec

    def publish_notice(self) -> None:
        subscription_count = len(
            self.get_subscriptions_info_by_topic(self._output_cmd_vel_topic)
        )
        command_active = twist_is_active(self._latest_command) and not self._is_stale(
            self._last_command_time
        )
        notice = build_operator_notice(
            OperatorState(
                has_safety_status=self._has_safety_status,
                safety_status_stale=self._is_stale(self._last_safety_time),
                safety_stop=self._safety_stop,
                safety_detail=self._safety_detail,
                has_target_status=self._has_target_status,
                target_status_stale=self._is_stale(self._last_target_time),
                target_detected=self._target_detected,
                command_active=command_active,
                mission_complete=self._mission_complete,
                mission_status=self._mission_status,
                mode=self._mode,
                output_cmd_vel_topic=self._output_cmd_vel_topic,
                output_subscription_count=subscription_count,
                navigation_mode=self._navigation_mode,
            )
        )

        now = self.get_clock().now()
        notice_key = notice_dedup_key(notice)
        should_repeat = (
            self._last_notice_time is None
            or (now - self._last_notice_time).nanoseconds / 1e9 >= self._repeat_notice_sec
        )
        if notice_key != self._last_notice_key or should_repeat:
            msg = String()
            msg.data = notice
            self._status_pub.publish(msg)
            if notice.startswith(('STOP', 'ERROR', 'WAIT')):
                self.get_logger().warn(notice)
            else:
                self.get_logger().info(notice)
            self._last_notice = notice
            self._last_notice_key = notice_key
            self._last_notice_time = now


def main(args=None):
    rclpy.init(args=args)
    node = OperatorStatusNode()
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
