#!/usr/bin/env python3

import json
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


ROUTES = ('front', 'back', 'left', 'right')
DEPTH_ROUTE_MAP = {'front': 'front', 'left': 'left', 'right': 'right'}
FUSION_LIDAR_DEPTH = 'lidar_depth'
FUSION_DEPTH_ONLY = 'depth_only'
FUSION_MODES = (FUSION_LIDAR_DEPTH, FUSION_DEPTH_ONLY)


def parse_json_status(text: str) -> Dict[str, Any]:
    try:
        value = json.loads(text or '{}')
    except json.JSONDecodeError:
        return {}
    return value if isinstance(value, dict) else {}


def _status_dict(status: Dict[str, Any], key: str) -> Dict[str, Any]:
    value = status.get(key, {})
    return value if isinstance(value, dict) else {}


def build_stop_reasons(
    lidar_status: Dict[str, Any],
    depth_status: Dict[str, Any],
    lidar_stop: bool,
    depth_stop: bool,
    lidar_fresh: bool,
    depth_fresh: bool,
    fusion_mode: str = FUSION_LIDAR_DEPTH,
) -> list:
    """Return ordered, machine-readable reasons for a fused stop."""
    reasons = []

    def active_reason(status: Dict[str, Any], fallback: str) -> str:
        reason = str(status.get('reason') or '').strip()
        return fallback if reason in ('', 'clear') else reason

    if fusion_mode == FUSION_LIDAR_DEPTH:
        if not lidar_fresh:
            reasons.append('lidar_stale')
        elif not bool(lidar_status.get('transform_ok', False)):
            reasons.append(active_reason(lidar_status, 'lidar_transform_invalid'))
        elif bool(lidar_stop) or bool(lidar_status.get('hard_stop', True)):
            reasons.append(active_reason(lidar_status, 'lidar_obstacle'))

    if not depth_fresh:
        reasons.append('depth_stale')
    elif not bool(depth_status.get('transform_ok', False)):
        reasons.append(active_reason(depth_status, 'depth_input_invalid'))
    elif bool(depth_stop) or bool(depth_status.get('hard_stop', True)):
        reasons.append(active_reason(depth_status, 'depth_obstacle'))

    return reasons


def fuse_route_status(
    lidar_status: Dict[str, Any],
    depth_status: Dict[str, Any],
    lidar_stop: bool,
    depth_stop: bool,
    lidar_fresh: bool = True,
    depth_fresh: bool = True,
    fusion_mode: str = FUSION_LIDAR_DEPTH,
) -> Dict[str, Any]:
    if fusion_mode not in FUSION_MODES:
        raise ValueError(
            'fusion_mode must be one of %s, got %r' % (FUSION_MODES, fusion_mode)
        )
    lidar_required = fusion_mode == FUSION_LIDAR_DEPTH
    lidar_clear = _status_dict(lidar_status, 'clear')
    depth_clear = _status_dict(depth_status, 'clear')
    lidar_counts = _status_dict(lidar_status, 'counts')
    lidar_corridor = _status_dict(lidar_status, 'corridor')
    depth_clearance = _status_dict(depth_status, 'clearance_m')
    raw_target_distance = depth_status.get('target_distance_m')
    try:
        target_distance_m = float(raw_target_distance)
    except (TypeError, ValueError):
        target_distance_m = None
    if not depth_fresh or not bool(depth_status.get('transform_ok', False)):
        target_distance_m = None
    stop_reasons = build_stop_reasons(
        lidar_status,
        depth_status,
        lidar_stop,
        depth_stop,
        lidar_fresh,
        depth_fresh,
        fusion_mode,
    )

    inputs_ok = (
        depth_fresh
        and bool(depth_status.get('transform_ok', False))
        and (
            not lidar_required
            or (
                lidar_fresh
                and bool(lidar_status.get('transform_ok', False))
            )
        )
    )
    hard_stop = (
        not inputs_ok
        or bool(depth_stop)
        or bool(depth_status.get('hard_stop', True))
        or (
            lidar_required
            and (
                bool(lidar_stop)
                or bool(lidar_status.get('hard_stop', True))
            )
        )
    )

    clear: Dict[str, bool] = {}
    clearance_m: Dict[str, Optional[float]] = {}
    for route in ROUTES:
        lidar_route_clear = (
            bool(lidar_clear.get(route, False)) if lidar_required else True
        )
        depth_route = DEPTH_ROUTE_MAP.get(route)
        if depth_route is None:
            # A forward-facing depth camera cannot validate a reverse route.
            depth_route_clear = lidar_required
            clearance_m[route] = None
        else:
            depth_route_clear = bool(depth_clear.get(depth_route, False))
            raw_clearance = depth_clearance.get(depth_route)
            try:
                clearance_m[route] = float(raw_clearance)
            except (TypeError, ValueError):
                clearance_m[route] = None
        clear[route] = inputs_ok and not hard_stop and lidar_route_clear and depth_route_clear

    return {
        'sensor': (
            'lidar_depth_fusion' if lidar_required else 'camera_depth_only'
        ),
        'navigation_mode': fusion_mode,
        'transform_ok': inputs_ok,
        'hard_stop': hard_stop,
        'primary_stop_reason': stop_reasons[0] if stop_reasons else 'clear',
        'stop_reasons': stop_reasons,
        'clear': clear,
        'lidar_clear': (
            {
                route: inputs_ok and bool(lidar_clear.get(route, False))
                for route in ROUTES
            }
            if lidar_required
            else {}
        ),
        'depth_clear': {
            route: bool(depth_clear.get(route, False))
            for route in DEPTH_ROUTE_MAP.values()
        },
        'blocked': {route: not is_clear for route, is_clear in clear.items()},
        'counts': (
            {
                route: int(lidar_counts.get(route, 0) or 0)
                for route in ROUTES
            }
            if lidar_required
            else {}
        ),
        'lidar_diagnostics': (
            {
                'hard_stop_count': int(
                    lidar_status.get('hard_stop_count', 0) or 0
                ),
                'hard_stop_point_threshold': int(
                    lidar_status.get('hard_stop_point_threshold', 0) or 0
                ),
                'hard_stop_region': _status_dict(lidar_status, 'hard_stop_region'),
                'route_counts': {
                    route: int(lidar_counts.get(route, 0) or 0)
                    for route in ROUTES
                },
                'route_blocked': _status_dict(lidar_status, 'blocked'),
                'route_block_point_thresholds': _status_dict(
                    lidar_status,
                    'route_block_point_thresholds',
                ),
            }
            if lidar_required
            else {}
        ),
        'clearance_m': clearance_m,
        'target_distance_m': target_distance_m,
        'corridor': lidar_corridor if lidar_required else {},
        'sources': {
            'navigation_mode': fusion_mode,
            'lidar_required': lidar_required,
            'lidar_fresh': bool(lidar_fresh) if lidar_required else None,
            'depth_fresh': bool(depth_fresh),
            'lidar_stop': bool(lidar_stop) if lidar_required else None,
            'depth_stop': bool(depth_stop),
        },
    }


class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        self.declare_parameter('lidar_safety_stop_topic', '/lidar_safety_stop')
        self.declare_parameter('depth_safety_stop_topic', '/depth_safety_stop')
        self.declare_parameter('lidar_route_status_topic', '/lidar_route_status')
        self.declare_parameter('depth_route_status_topic', '/depth_route_status')
        self.declare_parameter('safety_stop_topic', '/safety_stop')
        self.declare_parameter('route_status_topic', '/route_status')
        self.declare_parameter('safety_stop_reason_topic', '/safety_stop_reason')
        self.declare_parameter('fusion_mode', FUSION_LIDAR_DEPTH)
        self.declare_parameter('input_timeout_sec', 1.0)
        self.declare_parameter('publish_rate_hz', 10.0)

        self._fusion_mode = str(self.get_parameter('fusion_mode').value).strip().lower()
        if self._fusion_mode not in FUSION_MODES:
            raise ValueError(
                'fusion_mode must be one of %s, got %r'
                % (FUSION_MODES, self._fusion_mode)
            )
        self._lidar_required = self._fusion_mode == FUSION_LIDAR_DEPTH
        self._timeout_sec = max(0.1, float(self.get_parameter('input_timeout_sec').value))
        publish_rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self._lidar_stop = True
        self._depth_stop = True
        self._lidar_status: Dict[str, Any] = {}
        self._depth_status: Dict[str, Any] = {}
        self._last_lidar_stop_time = None
        self._last_depth_stop_time = None
        self._last_lidar_status_time = None
        self._last_depth_status_time = None
        self._last_published_stop: Optional[bool] = None

        self._stop_pub = self.create_publisher(
            Bool,
            str(self.get_parameter('safety_stop_topic').value),
            10,
        )
        self._route_pub = self.create_publisher(
            String,
            str(self.get_parameter('route_status_topic').value),
            10,
        )
        self._reason_pub = self.create_publisher(
            String,
            str(self.get_parameter('safety_stop_reason_topic').value),
            10,
        )
        if self._lidar_required:
            self.create_subscription(
                Bool,
                str(self.get_parameter('lidar_safety_stop_topic').value),
                self._lidar_stop_callback,
                10,
            )
        self.create_subscription(
            Bool,
            str(self.get_parameter('depth_safety_stop_topic').value),
            self._depth_stop_callback,
            10,
        )
        if self._lidar_required:
            self.create_subscription(
                String,
                str(self.get_parameter('lidar_route_status_topic').value),
                self._lidar_status_callback,
                10,
            )
        self.create_subscription(
            String,
            str(self.get_parameter('depth_route_status_topic').value),
            self._depth_status_callback,
            10,
        )
        self.create_timer(1.0 / publish_rate, self.publish_fused_status)
        required_inputs = 'LiDAR and depth' if self._lidar_required else 'camera depth'
        self.get_logger().info(
            'Navigation safety started: mode=%s; %s must be fresh within %.2f s.'
            % (self._fusion_mode, required_inputs, self._timeout_sec)
        )

    def _lidar_stop_callback(self, msg: Bool) -> None:
        self._lidar_stop = bool(msg.data)
        self._last_lidar_stop_time = self.get_clock().now()

    def _depth_stop_callback(self, msg: Bool) -> None:
        self._depth_stop = bool(msg.data)
        self._last_depth_stop_time = self.get_clock().now()

    def _lidar_status_callback(self, msg: String) -> None:
        self._lidar_status = parse_json_status(msg.data)
        self._last_lidar_status_time = self.get_clock().now()

    def _depth_status_callback(self, msg: String) -> None:
        self._depth_status = parse_json_status(msg.data)
        self._last_depth_status_time = self.get_clock().now()

    def _fresh(self, *times) -> bool:
        now = self.get_clock().now()
        for stamp in times:
            if stamp is None:
                return False
            if (now - stamp).nanoseconds / 1e9 > self._timeout_sec:
                return False
        return True

    def publish_fused_status(self) -> None:
        lidar_fresh = (
            self._fresh(
                self._last_lidar_stop_time,
                self._last_lidar_status_time,
            )
            if self._lidar_required
            else True
        )
        depth_fresh = self._fresh(
            self._last_depth_stop_time,
            self._last_depth_status_time,
        )
        status = fuse_route_status(
            self._lidar_status,
            self._depth_status,
            self._lidar_stop,
            self._depth_stop,
            lidar_fresh,
            depth_fresh,
            self._fusion_mode,
        )

        stop_msg = Bool()
        stop_msg.data = bool(status['hard_stop'])
        self._stop_pub.publish(stop_msg)
        route_msg = String()
        route_msg.data = json.dumps(status, sort_keys=True)
        self._route_pub.publish(route_msg)
        reason_msg = String()
        reason_msg.data = json.dumps(
            {
                'stopped': bool(status['hard_stop']),
                'primary_reason': status['primary_stop_reason'],
                'reasons': status['stop_reasons'],
                'sources': status['sources'],
                'lidar_diagnostics': status['lidar_diagnostics'],
            },
            sort_keys=True,
        )
        self._reason_pub.publish(reason_msg)

        if self._last_published_stop is None or self._last_published_stop != stop_msg.data:
            self.get_logger().warn(
                'Fused safety stop changed to %s; primary_reason=%s; reasons=%s'
                % (
                    stop_msg.data,
                    status['primary_stop_reason'],
                    status['stop_reasons'],
                )
            )
        self._last_published_stop = stop_msg.data


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
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
