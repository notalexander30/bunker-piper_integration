#!/usr/bin/env python3

import json
import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
import tf2_ros


@dataclass(frozen=True)
class SafetyBox:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float
    use_z_filter: bool = True

    def contains(self, x: float, y: float, z: float) -> bool:
        return (
            self.x_min <= x <= self.x_max
            and self.y_min <= y <= self.y_max
            and (not self.use_z_filter or self.z_min <= z <= self.z_max)
        )


@dataclass(frozen=True)
class CorridorCandidate:
    angle_deg: float
    free_length_m: float
    obstacle_count: int
    fits: bool


def normalize_frame_id(frame_id: str) -> str:
    return frame_id.lstrip('/') if frame_id else ''


def point_as_xyz(point) -> Tuple[float, float, float]:
    try:
        return float(point[0]), float(point[1]), float(point[2])
    except (IndexError, KeyError, TypeError, ValueError):
        return float(point['x']), float(point['y']), float(point['z'])


def rotate_vector_by_quaternion(
    x: float,
    y: float,
    z: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> Tuple[float, float, float]:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm == 0.0:
        return x, y, z

    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm

    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    return (
        (1.0 - 2.0 * (yy + zz)) * x + 2.0 * (xy - wz) * y + 2.0 * (xz + wy) * z,
        2.0 * (xy + wz) * x + (1.0 - 2.0 * (xx + zz)) * y + 2.0 * (yz - wx) * z,
        2.0 * (xz - wy) * x + 2.0 * (yz + wx) * y + (1.0 - 2.0 * (xx + yy)) * z,
    )


def transform_xyz(transform, x: float, y: float, z: float) -> Tuple[float, float, float]:
    rotation = transform.transform.rotation
    translation = transform.transform.translation
    rx, ry, rz = rotate_vector_by_quaternion(
        x,
        y,
        z,
        rotation.x,
        rotation.y,
        rotation.z,
        rotation.w,
    )
    return rx + translation.x, ry + translation.y, rz + translation.z


def count_points_in_safety_box(
    points: Iterable,
    safety_box: SafetyBox,
    transform=None,
) -> int:
    danger_count = 0
    for point in points:
        x, y, z = point_as_xyz(point)
        if transform is not None:
            x, y, z = transform_xyz(transform, x, y, z)
        if safety_box.contains(x, y, z):
            danger_count += 1
    return danger_count


def count_points_in_boxes(
    points: Iterable,
    boxes: Dict[str, SafetyBox],
    transform=None,
) -> Dict[str, int]:
    counts = {name: 0 for name in boxes}
    for point in points:
        x, y, z = point_as_xyz(point)
        if transform is not None:
            x, y, z = transform_xyz(transform, x, y, z)
        for name, box in boxes.items():
            if box.contains(x, y, z):
                counts[name] += 1
    return counts


def evaluate_corridor_candidates(
    points: Iterable,
    angles_deg: Sequence[float],
    corridor_width_m: float,
    start_distance_m: float,
    lookahead_distance_m: float,
    minimum_free_length_m: float,
    z_min: float,
    z_max: float,
) -> List[CorridorCandidate]:
    """Measure a robot-width clear strip along each candidate heading."""
    half_width = max(0.0, float(corridor_width_m)) / 2.0
    start = max(0.0, float(start_distance_m))
    lookahead = max(start, float(lookahead_distance_m))
    required = max(start, min(lookahead, float(minimum_free_length_m)))
    xyz_points = []
    for point in points:
        x, y, z = point_as_xyz(point)
        if all(math.isfinite(value) for value in (x, y, z)) and z_min <= z <= z_max:
            xyz_points.append((x, y, z))

    candidates = []
    for angle_deg in angles_deg:
        angle_rad = math.radians(float(angle_deg))
        cosine = math.cos(angle_rad)
        sine = math.sin(angle_rad)
        obstacle_distances = []
        for x, y, _ in xyz_points:
            forward = x * cosine + y * sine
            lateral = -x * sine + y * cosine
            if start <= forward <= lookahead and abs(lateral) <= half_width:
                obstacle_distances.append(forward)

        nearest = min(obstacle_distances) if obstacle_distances else lookahead
        free_length = max(start, min(lookahead, nearest))
        candidates.append(
            CorridorCandidate(
                angle_deg=float(angle_deg),
                free_length_m=free_length,
                obstacle_count=len(obstacle_distances),
                fits=free_length >= required,
            )
        )
    return candidates


def select_best_corridor_candidate(
    candidates: Sequence[CorridorCandidate],
) -> Optional[CorridorCandidate]:
    fitting = [candidate for candidate in candidates if candidate.fits]
    if not fitting:
        return None
    # Prefer the longest path, then the smallest steering change. The angle is
    # deterministic, so equal left/right openings do not alternate arbitrarily.
    return min(
        fitting,
        key=lambda candidate: (
            -candidate.free_length_m,
            abs(candidate.angle_deg),
            -candidate.angle_deg,
        ),
    )


def corridor_route_clear(candidates: Sequence[CorridorCandidate]) -> Dict[str, bool]:
    return {
        'front': any(candidate.fits and candidate.angle_deg == 0.0 for candidate in candidates),
        'left': any(
            candidate.fits and 0.0 < candidate.angle_deg < 90.0
            for candidate in candidates
        ),
        'right': any(
            candidate.fits and -90.0 < candidate.angle_deg < 0.0
            for candidate in candidates
        ),
        'back': any(
            candidate.fits and abs(candidate.angle_deg) >= 175.0
            for candidate in candidates
        ),
    }


def build_route_blocked_status(
    route_counts: Dict[str, int],
    route_thresholds: Dict[str, int],
    default_threshold: int,
) -> Tuple[Dict[str, bool], Dict[str, int]]:
    thresholds = {}
    blocked = {}
    for name, count in route_counts.items():
        threshold = int(route_thresholds.get(name, default_threshold))
        thresholds[name] = threshold
        blocked[name] = int(count) > threshold
    return blocked, thresholds


def front_obstacle_stop_policy(
    front_obstacle: bool,
    route_blocked: Dict[str, bool],
) -> Tuple[bool, bool]:
    """Return (stop, side_route_available) for the binary reroute policy."""
    side_route_available = (
        not bool(route_blocked.get('left', True))
        or not bool(route_blocked.get('right', True))
    )
    return bool(front_obstacle and not side_route_available), side_route_available


class SafetyMonitorNode(Node):
    def __init__(self):
        super().__init__('safety_monitor_node')

        self.declare_parameter('pointcloud_topic', '/cx/lslidar_point_cloud')
        self.declare_parameter('safety_stop_topic', '/safety_stop')
        self.declare_parameter('route_status_topic', '/route_status')
        self.declare_parameter('base_frame', 'laser_link')
        self.declare_parameter('enable_tf_transform', True)
        self.declare_parameter('use_latest_transform', True)
        self.declare_parameter('transform_timeout_sec', 0.05)
        self.declare_parameter('fail_safe_on_transform_error', True)
        # Live bumper self-returns measured 11-24 points; 31+ points stops.
        self.declare_parameter('hard_stop_point_threshold', 30)
        self.declare_parameter('route_block_point_threshold', 40)
        self.declare_parameter('route_block_point_thresholds.front', 80)
        self.declare_parameter('route_block_point_thresholds.back', 40)
        self.declare_parameter('route_block_point_thresholds.left', 150)
        self.declare_parameter('route_block_point_thresholds.right', 150)
        self.declare_parameter('hard_stop_box.x_min', 0.2)
        # LiDAR is 20 cm behind the bumper; monitor the next 20 cm.
        self.declare_parameter('hard_stop_box.x_max', 0.4)
        self.declare_parameter('hard_stop_box.y_min', -0.5)
        self.declare_parameter('hard_stop_box.y_max', 0.5)
        self.declare_parameter('hard_stop_box.z_min', 0.15)
        self.declare_parameter('hard_stop_box.z_max', 1.5)
        self.declare_parameter('hard_stop_box.use_z_filter', False)
        self.declare_parameter('route_box.front.x_min', 0.2)
        self.declare_parameter('route_box.front.x_max', 0.4)
        self.declare_parameter('route_box.front.y_min', -0.5)
        self.declare_parameter('route_box.front.y_max', 0.5)
        self.declare_parameter('route_box.back.x_min', -0.4)
        self.declare_parameter('route_box.back.x_max', -0.2)
        self.declare_parameter('route_box.back.y_min', -0.5)
        self.declare_parameter('route_box.back.y_max', 0.5)
        self.declare_parameter('route_box.left.x_min', -0.5)
        self.declare_parameter('route_box.left.x_max', 0.5)
        self.declare_parameter('route_box.left.y_min', 0.2)
        self.declare_parameter('route_box.left.y_max', 0.4)
        self.declare_parameter('route_box.right.x_min', -0.5)
        self.declare_parameter('route_box.right.x_max', 0.5)
        self.declare_parameter('route_box.right.y_min', -0.4)
        self.declare_parameter('route_box.right.y_max', -0.2)
        self.declare_parameter('route_box.z_min', 0.15)
        self.declare_parameter('route_box.z_max', 1.5)
        self.declare_parameter('corridor.width_m', 0.60)
        self.declare_parameter('corridor.start_distance_m', 0.20)
        self.declare_parameter('corridor.lookahead_distance_m', 1.20)
        self.declare_parameter('corridor.minimum_free_length_m', 0.60)
        self.declare_parameter(
            'corridor.candidate_angles_deg',
            [
                -180.0,
                -60.0, -45.0, -30.0, -15.0,
                0.0,
                15.0, 30.0, 45.0, 60.0,
                180.0,
            ],
        )

        self._pointcloud_topic = self.get_parameter('pointcloud_topic').value
        safety_stop_topic = self.get_parameter('safety_stop_topic').value
        route_status_topic = self.get_parameter('route_status_topic').value
        self._base_frame = normalize_frame_id(self.get_parameter('base_frame').value)
        self._enable_tf_transform = self.get_parameter('enable_tf_transform').value
        self._use_latest_transform = self.get_parameter('use_latest_transform').value
        self._transform_timeout_sec = float(self.get_parameter('transform_timeout_sec').value)
        self._fail_safe_on_transform_error = self.get_parameter(
            'fail_safe_on_transform_error'
        ).value
        self._hard_stop_point_threshold = int(
            self.get_parameter('hard_stop_point_threshold').value
        )
        self._route_block_point_threshold = int(
            self.get_parameter('route_block_point_threshold').value
        )
        self._route_block_point_thresholds = {
            route: self._route_block_threshold_for_route(route)
            for route in ('front', 'back', 'left', 'right')
        }
        self._hard_stop_box = SafetyBox(
            x_min=float(self.get_parameter('hard_stop_box.x_min').value),
            x_max=float(self.get_parameter('hard_stop_box.x_max').value),
            y_min=float(self.get_parameter('hard_stop_box.y_min').value),
            y_max=float(self.get_parameter('hard_stop_box.y_max').value),
            z_min=float(self.get_parameter('hard_stop_box.z_min').value),
            z_max=float(self.get_parameter('hard_stop_box.z_max').value),
            use_z_filter=bool(
                self.get_parameter('hard_stop_box.use_z_filter').value
            ),
        )
        route_z_min = float(self.get_parameter('route_box.z_min').value)
        route_z_max = float(self.get_parameter('route_box.z_max').value)
        self._route_z_min = route_z_min
        self._route_z_max = route_z_max
        self._corridor_width_m = max(
            0.01, float(self.get_parameter('corridor.width_m').value)
        )
        self._corridor_start_distance_m = max(
            0.0, float(self.get_parameter('corridor.start_distance_m').value)
        )
        self._corridor_lookahead_distance_m = max(
            self._corridor_start_distance_m,
            float(self.get_parameter('corridor.lookahead_distance_m').value),
        )
        self._corridor_minimum_free_length_m = max(
            self._corridor_start_distance_m,
            min(
                self._corridor_lookahead_distance_m,
                float(self.get_parameter('corridor.minimum_free_length_m').value),
            ),
        )
        self._corridor_candidate_angles_deg = [
            float(value)
            for value in self.get_parameter('corridor.candidate_angles_deg').value
        ]
        self._route_boxes = {
            'front': SafetyBox(
                x_min=float(self.get_parameter('route_box.front.x_min').value),
                x_max=float(self.get_parameter('route_box.front.x_max').value),
                y_min=float(self.get_parameter('route_box.front.y_min').value),
                y_max=float(self.get_parameter('route_box.front.y_max').value),
                z_min=route_z_min,
                z_max=route_z_max,
            ),
            'back': SafetyBox(
                x_min=float(self.get_parameter('route_box.back.x_min').value),
                x_max=float(self.get_parameter('route_box.back.x_max').value),
                y_min=float(self.get_parameter('route_box.back.y_min').value),
                y_max=float(self.get_parameter('route_box.back.y_max').value),
                z_min=route_z_min,
                z_max=route_z_max,
            ),
            'left': SafetyBox(
                x_min=float(self.get_parameter('route_box.left.x_min').value),
                x_max=float(self.get_parameter('route_box.left.x_max').value),
                y_min=float(self.get_parameter('route_box.left.y_min').value),
                y_max=float(self.get_parameter('route_box.left.y_max').value),
                z_min=route_z_min,
                z_max=route_z_max,
            ),
            'right': SafetyBox(
                x_min=float(self.get_parameter('route_box.right.x_min').value),
                x_max=float(self.get_parameter('route_box.right.x_max').value),
                y_min=float(self.get_parameter('route_box.right.y_min').value),
                y_max=float(self.get_parameter('route_box.right.y_max').value),
                z_min=route_z_min,
                z_max=route_z_max,
            ),
        }

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._safety_stop_pub = self.create_publisher(Bool, safety_stop_topic, 10)
        self._route_status_pub = self.create_publisher(String, route_status_topic, 10)
        self._pointcloud_sub = self.create_subscription(
            PointCloud2,
            self._pointcloud_topic,
            self.pointcloud_callback,
            sensor_qos,
        )
        self._last_safety_stop: Optional[bool] = None

        self.get_logger().info(
            'Safety monitor started: pointcloud_topic=%s, safety_stop_topic=%s, '
            'route_status_topic=%s, base_frame=%s, hard_stop_threshold=%d, '
            'route_block_threshold=%d'
            % (
                self._pointcloud_topic,
                safety_stop_topic,
                route_status_topic,
                self._base_frame,
                self._hard_stop_point_threshold,
                self._route_block_point_threshold,
            )
        )

    def _route_block_threshold_for_route(self, route: str) -> int:
        value = int(self.get_parameter('route_block_point_thresholds.%s' % route).value)
        if value < 0:
            return self._route_block_point_threshold
        return value

    def _lookup_transform(self, cloud_msg: PointCloud2):
        source_frame = normalize_frame_id(cloud_msg.header.frame_id)
        if not self._enable_tf_transform:
            return None

        if not source_frame:
            self.get_logger().warn(
                'Point cloud frame_id is empty; cannot verify safety box in base frame.',
                throttle_duration_sec=2.0,
            )
            if self._fail_safe_on_transform_error:
                self._publish_safety_stop(True, reason='empty_pointcloud_frame')
                self._publish_route_status(
                    transform_ok=False,
                    reason='lidar_empty_frame',
                )
                return False
            return None

        if source_frame == self._base_frame:
            return None

        stamp = Time() if self._use_latest_transform else Time.from_msg(cloud_msg.header.stamp)
        try:
            return self._tf_buffer.lookup_transform(
                self._base_frame,
                source_frame,
                stamp,
                Duration(seconds=self._transform_timeout_sec),
            )
        except Exception as exc:  # tf2 exception types vary slightly across ROS 2 patch levels.
            self.get_logger().warn(
                "Cannot transform point cloud from frame '%s' to '%s': %s"
                % (source_frame, self._base_frame, exc),
                throttle_duration_sec=2.0,
            )
            if self._fail_safe_on_transform_error:
                self._publish_safety_stop(True, reason='transform_error')
                self._publish_route_status(
                    transform_ok=False,
                    reason='lidar_transform_invalid',
                )
                return False
            return None

    def pointcloud_callback(self, msg: PointCloud2) -> None:
        transform = self._lookup_transform(msg)
        if transform is False:
            return

        raw_points = point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True
        )
        points = []
        for point in raw_points:
            x, y, z = point_as_xyz(point)
            if transform is not None:
                x, y, z = transform_xyz(transform, x, y, z)
            points.append((x, y, z))
        boxes = {'hard_stop': self._hard_stop_box, **self._route_boxes}
        counts = count_points_in_boxes(points, boxes)
        hard_stop_count = counts['hard_stop']
        front_obstacle = hard_stop_count > self._hard_stop_point_threshold
        route_counts = {
            name: counts[name]
            for name in ('front', 'back', 'left', 'right')
        }
        route_blocked, _ = build_route_blocked_status(
            route_counts,
            self._route_block_point_thresholds,
            self._route_block_point_threshold,
        )
        corridor_candidates = evaluate_corridor_candidates(
            points=points,
            angles_deg=self._corridor_candidate_angles_deg,
            corridor_width_m=self._corridor_width_m,
            start_distance_m=self._corridor_start_distance_m,
            lookahead_distance_m=self._corridor_lookahead_distance_m,
            minimum_free_length_m=self._corridor_minimum_free_length_m,
            z_min=self._route_z_min,
            z_max=self._route_z_max,
        )
        corridor_clear = corridor_route_clear(corridor_candidates)
        for route in ('front', 'left', 'right', 'back'):
            route_blocked[route] = route_blocked[route] or not corridor_clear[route]
        selected_corridor = select_best_corridor_candidate(corridor_candidates)
        # A close front object blocks forward travel. Rotation remains available
        # when at least one LiDAR side box is clear; otherwise fail-safe stop.
        safety_stop, side_route_available = front_obstacle_stop_policy(
            front_obstacle,
            route_blocked,
        )

        self._publish_safety_stop(safety_stop, danger_count=hard_stop_count)
        self._publish_route_status(
            transform_ok=True,
            hard_stop=safety_stop,
            hard_stop_count=hard_stop_count,
            route_counts=route_counts,
            reason=(
                'lidar_obstacle_no_side_route'
                if safety_stop
                else (
                    'lidar_front_obstacle_reroute'
                    if front_obstacle
                    else 'clear'
                )
            ),
            source_frame=normalize_frame_id(msg.header.frame_id),
            front_obstacle=front_obstacle,
            side_route_available=side_route_available,
            corridor_candidates=corridor_candidates,
            selected_corridor=selected_corridor,
        )
        self.get_logger().info(
            'LiDAR trigger=hard_stop_box(front): points=%d > threshold=%d; '
            'box=%s x[%.2f,%.2f] y[%.2f,%.2f] z=%s; route_counts=%s; stop=%s'
            % (
                hard_stop_count,
                self._hard_stop_point_threshold,
                self._base_frame,
                self._hard_stop_box.x_min,
                self._hard_stop_box.x_max,
                self._hard_stop_box.y_min,
                self._hard_stop_box.y_max,
                (
                    '[%.2f,%.2f]'
                    % (self._hard_stop_box.z_min, self._hard_stop_box.z_max)
                    if self._hard_stop_box.use_z_filter
                    else 'ignored'
                ),
                route_counts,
                safety_stop,
            ),
            throttle_duration_sec=1.0,
        )

    def _publish_safety_stop(
        self,
        active: bool,
        danger_count: Optional[int] = None,
        reason: Optional[str] = None,
    ) -> None:
        msg = Bool()
        msg.data = bool(active)
        self._safety_stop_pub.publish(msg)

        if self._last_safety_stop is None or self._last_safety_stop != msg.data:
            detail = ''
            if danger_count is not None:
                detail = ' danger_count=%d' % danger_count
            elif reason:
                detail = ' reason=%s' % reason
            self.get_logger().info('Safety stop changed to %s.%s' % (msg.data, detail))
        self._last_safety_stop = msg.data

    def _publish_route_status(
        self,
        transform_ok: bool,
        hard_stop: bool = True,
        hard_stop_count: int = 0,
        route_counts: Optional[Dict[str, int]] = None,
        reason: str = '',
        source_frame: str = '',
        front_obstacle: bool = False,
        side_route_available: bool = False,
        corridor_candidates: Optional[Sequence[CorridorCandidate]] = None,
        selected_corridor: Optional[CorridorCandidate] = None,
    ) -> None:
        route_counts = route_counts or {
            'front': 0,
            'back': 0,
            'left': 0,
            'right': 0,
        }
        blocked, route_block_thresholds = build_route_blocked_status(
            route_counts,
            self._route_block_point_thresholds,
            self._route_block_point_threshold,
        )
        # The near-distance rule is authoritative for the front route even when
        # the larger route-count box itself is below its threshold.
        blocked['front'] = blocked['front'] or bool(front_obstacle)
        if corridor_candidates is not None:
            corridor_clear = corridor_route_clear(corridor_candidates)
            for route in ('front', 'left', 'right', 'back'):
                blocked[route] = blocked[route] or not corridor_clear[route]
        status = {
            'transform_ok': bool(transform_ok),
            'hard_stop': bool(hard_stop),
            'reason': str(reason or ('lidar_obstacle' if hard_stop else 'clear')),
            'hard_stop_count': int(hard_stop_count),
            'hard_stop_point_threshold': int(self._hard_stop_point_threshold),
            'front_obstacle': bool(front_obstacle),
            'side_route_available': bool(side_route_available),
            'hard_stop_region': {
                'name': 'hard_stop_box(front)',
                'frame_id': self._base_frame,
                'source_frame': str(source_frame),
                'x_min': self._hard_stop_box.x_min,
                'x_max': self._hard_stop_box.x_max,
                'y_min': self._hard_stop_box.y_min,
                'y_max': self._hard_stop_box.y_max,
                'z_filter_enabled': self._hard_stop_box.use_z_filter,
                'z_min': (
                    self._hard_stop_box.z_min
                    if self._hard_stop_box.use_z_filter
                    else None
                ),
                'z_max': (
                    self._hard_stop_box.z_max
                    if self._hard_stop_box.use_z_filter
                    else None
                ),
            },
            'route_block_point_threshold': int(self._route_block_point_threshold),
            'route_block_point_thresholds': route_block_thresholds,
            'blocked': blocked,
            'clear': {
                name: transform_ok and not hard_stop and not is_blocked
                for name, is_blocked in blocked.items()
            },
            'counts': route_counts,
            'corridor': {
                'width_m': self._corridor_width_m,
                'start_distance_m': self._corridor_start_distance_m,
                'lookahead_distance_m': self._corridor_lookahead_distance_m,
                'minimum_free_length_m': self._corridor_minimum_free_length_m,
                'selected_angle_deg': (
                    selected_corridor.angle_deg if selected_corridor is not None else None
                ),
                'selected_free_length_m': (
                    selected_corridor.free_length_m
                    if selected_corridor is not None
                    else 0.0
                ),
                'fits': selected_corridor is not None,
                'candidates': [
                    {
                        'angle_deg': candidate.angle_deg,
                        'free_length_m': round(candidate.free_length_m, 3),
                        'obstacle_count': candidate.obstacle_count,
                        'fits': candidate.fits,
                    }
                    for candidate in (corridor_candidates or [])
                ],
            },
        }
        msg = String()
        msg.data = json.dumps(status, sort_keys=True)
        self._route_status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
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
