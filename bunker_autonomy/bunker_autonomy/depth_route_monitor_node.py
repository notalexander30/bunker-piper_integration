#!/usr/bin/env python3

import json
from dataclasses import dataclass
from typing import Dict, Optional, Sequence

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


ROUTES = ('left', 'front', 'right')


@dataclass(frozen=True)
class DepthSector:
    valid_fraction: float
    finite_positive_fraction: float
    clearance_m: Optional[float]
    hard_stop_fraction: float
    route_block_fraction: float
    clear: bool


def depth_image_to_meters(msg: Image) -> np.ndarray:
    """Decode common RealSense depth encodings without requiring cv_bridge."""
    encoding = str(msg.encoding).upper()
    if encoding in ('16UC1', 'MONO16'):
        dtype = np.dtype('>u2' if msg.is_bigendian else '<u2')
        scale = 0.001
    elif encoding == '32FC1':
        dtype = np.dtype('>f4' if msg.is_bigendian else '<f4')
        scale = 1.0
    else:
        raise ValueError('unsupported depth encoding %r' % msg.encoding)

    if int(msg.step) % dtype.itemsize:
        raise ValueError('depth image step is not aligned to its pixel size')
    row_values = int(msg.step) // dtype.itemsize
    if row_values < int(msg.width):
        raise ValueError('depth image step is smaller than its width')
    expected_values = int(msg.height) * row_values
    raw = np.frombuffer(msg.data, dtype=dtype, count=expected_values)
    if raw.size != expected_values:
        raise ValueError('depth image data is shorter than height * step')
    return raw.reshape(int(msg.height), row_values)[:, : int(msg.width)].astype(
        np.float32
    ) * scale


def _sector_summary(
    sector: np.ndarray,
    min_depth_m: float,
    max_depth_m: float,
    hard_stop_distance_m: float,
    route_block_distance_m: float,
    obstacle_pixel_fraction: float,
    minimum_valid_fraction: float,
    clearance_percentile: float,
) -> DepthSector:
    finite_positive_mask = np.isfinite(sector) & (sector > 0.0)
    valid_mask = (
        finite_positive_mask
        & (sector >= min_depth_m)
        & (sector <= max_depth_m)
    )
    pixel_count = max(1, int(sector.size))
    finite_positive_fraction = int(np.count_nonzero(finite_positive_mask)) / pixel_count
    valid_count = int(np.count_nonzero(valid_mask))
    valid_fraction = valid_count / pixel_count
    hard_stop_fraction = float(
        np.count_nonzero(finite_positive_mask & (sector <= hard_stop_distance_m))
    ) / pixel_count
    route_block_fraction = float(
        np.count_nonzero(finite_positive_mask & (sector <= route_block_distance_m))
    ) / pixel_count
    if valid_count == 0:
        return DepthSector(
            valid_fraction,
            finite_positive_fraction,
            None,
            hard_stop_fraction,
            route_block_fraction,
            False,
        )

    values = sector[valid_mask]
    clearance_m = float(np.percentile(values, clearance_percentile))
    clear = (
        valid_fraction >= minimum_valid_fraction
        and route_block_fraction < obstacle_pixel_fraction
    )
    return DepthSector(
        valid_fraction=valid_fraction,
        finite_positive_fraction=finite_positive_fraction,
        clearance_m=clearance_m,
        hard_stop_fraction=hard_stop_fraction,
        route_block_fraction=route_block_fraction,
        clear=clear,
    )


def validate_depth_settings(settings: Dict[str, float]) -> None:
    top = settings['roi_top_fraction']
    bottom = settings['roi_bottom_fraction']
    if not 0.0 <= top < bottom <= 1.0:
        raise ValueError('depth ROI fractions must satisfy 0 <= top < bottom <= 1')
    left = settings['roi_left_fraction']
    left_sector_end = settings['left_sector_end_fraction']
    right_sector_start = settings['right_sector_start_fraction']
    right = settings['roi_right_fraction']
    if not 0.0 <= left < left_sector_end < right_sector_start < right <= 1.0:
        raise ValueError(
            'horizontal depth fractions must satisfy 0 <= roi_left < '
            'left_sector_end < right_sector_start < roi_right <= 1'
        )
    if not 0.0 < settings['min_depth_m'] < settings['max_depth_m']:
        raise ValueError('depth range must satisfy 0 < min_depth_m < max_depth_m')
    if not (
        0.0
        < settings['hard_stop_distance_m']
        <= settings['route_block_distance_m']
        <= settings['max_depth_m']
    ):
        raise ValueError(
            'depth thresholds must satisfy 0 < hard_stop <= route_block <= max_depth'
        )
    if not 0.0 < settings['obstacle_pixel_fraction'] <= 1.0:
        raise ValueError('obstacle_pixel_fraction must be in (0, 1]')
    if not 0.0 < settings['minimum_valid_fraction'] <= 1.0:
        raise ValueError('minimum_valid_fraction must be in (0, 1]')
    if not 0.0 <= settings['clearance_percentile'] <= 100.0:
        raise ValueError('clearance_percentile must be in [0, 100]')


def summarize_depth_image(
    depth_m: np.ndarray,
    roi_top_fraction: float = 0.2,
    roi_bottom_fraction: float = 0.82,
    roi_left_fraction: float = 0.0,
    roi_right_fraction: float = 1.0,
    left_sector_end_fraction: float = 1.0 / 3.0,
    right_sector_start_fraction: float = 2.0 / 3.0,
    min_depth_m: float = 0.075,
    max_depth_m: float = 3.0,
    hard_stop_distance_m: float = 0.6,
    route_block_distance_m: float = 0.6,
    obstacle_pixel_fraction: float = 0.02,
    minimum_valid_fraction: float = 0.25,
    clearance_percentile: float = 10.0,
) -> Dict[str, DepthSector]:
    if depth_m.ndim != 2 or depth_m.size == 0:
        raise ValueError('depth image must be a non-empty 2-D array')

    settings = {
        'roi_top_fraction': float(roi_top_fraction),
        'roi_bottom_fraction': float(roi_bottom_fraction),
        'roi_left_fraction': float(roi_left_fraction),
        'roi_right_fraction': float(roi_right_fraction),
        'left_sector_end_fraction': float(left_sector_end_fraction),
        'right_sector_start_fraction': float(right_sector_start_fraction),
        'min_depth_m': float(min_depth_m),
        'max_depth_m': float(max_depth_m),
        'hard_stop_distance_m': float(hard_stop_distance_m),
        'route_block_distance_m': float(route_block_distance_m),
        'obstacle_pixel_fraction': float(obstacle_pixel_fraction),
        'minimum_valid_fraction': float(minimum_valid_fraction),
        'clearance_percentile': float(clearance_percentile),
    }
    validate_depth_settings(settings)

    height, width = depth_m.shape
    top = min(height - 1, max(0, int(height * roi_top_fraction)))
    bottom = min(height, max(top + 1, int(height * roi_bottom_fraction)))
    roi_left = min(width - 1, max(0, int(width * roi_left_fraction)))
    roi_right = min(width, max(roi_left + 3, int(width * roi_right_fraction)))
    left_edge = min(
        roi_right - 2,
        max(roi_left + 1, int(width * left_sector_end_fraction)),
    )
    right_edge = min(
        roi_right - 1,
        max(left_edge + 1, int(width * right_sector_start_fraction)),
    )
    slices = {
        'left': depth_m[top:bottom, roi_left:left_edge],
        'front': depth_m[top:bottom, left_edge:right_edge],
        'right': depth_m[top:bottom, right_edge:roi_right],
    }
    return {
        route: _sector_summary(
            sector,
            min_depth_m,
            max_depth_m,
            hard_stop_distance_m,
            route_block_distance_m,
            obstacle_pixel_fraction,
            minimum_valid_fraction,
            clearance_percentile,
        )
        for route, sector in slices.items()
    }


def target_depth_from_bbox(
    depth_m: np.ndarray,
    bbox_norm: Optional[Sequence[float]],
    min_depth_m: float,
    max_depth_m: float,
    percentile: float = 30.0,
) -> Optional[float]:
    """Estimate target range from the central area of a normalized image bbox."""
    if bbox_norm is None or len(bbox_norm) != 4 or depth_m.ndim != 2:
        return None
    try:
        x_min, y_min, x_max, y_max = [float(value) for value in bbox_norm]
    except (TypeError, ValueError):
        return None
    if not all(np.isfinite(value) for value in (x_min, y_min, x_max, y_max)):
        return None
    if not (0.0 <= x_min < x_max <= 1.0 and 0.0 <= y_min < y_max <= 1.0):
        return None

    # Ignore bbox edges, where background depth and RGB/depth parallax are most
    # likely to contaminate the target measurement.
    x_margin = (x_max - x_min) * 0.2
    y_margin = (y_max - y_min) * 0.2
    x_min += x_margin
    x_max -= x_margin
    y_min += y_margin
    y_max -= y_margin
    height, width = depth_m.shape
    left = max(0, min(width - 1, int(x_min * width)))
    right = max(left + 1, min(width, int(np.ceil(x_max * width))))
    top = max(0, min(height - 1, int(y_min * height)))
    bottom = max(top + 1, min(height, int(np.ceil(y_max * height))))
    region = depth_m[top:bottom, left:right]
    valid = region[
        np.isfinite(region)
        & (region >= float(min_depth_m))
        & (region <= float(max_depth_m))
    ]
    if valid.size < max(8, int(region.size * 0.1)):
        return None
    return round(float(np.percentile(valid, max(0.0, min(100.0, percentile)))), 3)


def mask_bbox(depth_m: np.ndarray, bbox_norm: Sequence[float]) -> np.ndarray:
    """Return a copy with a normalized bbox masked out of generic obstacle checks."""
    x_min, y_min, x_max, y_max = [float(value) for value in bbox_norm]
    height, width = depth_m.shape
    left = max(0, min(width - 1, int(x_min * width)))
    right = max(left + 1, min(width, int(np.ceil(x_max * width))))
    top = max(0, min(height - 1, int(y_min * height)))
    bottom = max(top + 1, min(height, int(np.ceil(y_max * height))))
    masked = depth_m.copy()
    masked[top:bottom, left:right] = np.nan
    return masked


class DepthRouteMonitorNode(Node):
    def __init__(self):
        super().__init__('depth_route_monitor_node')

        self.declare_parameter(
            'depth_image_topic',
            '/camera/camera/depth/image_rect_raw',
        )
        self.declare_parameter('front_depth_image_topic', '')
        self.declare_parameter('rear_depth_image_topic', '')
        self.declare_parameter(
            'active_camera_topic', '/landmark_navigator/active_camera')
        self.declare_parameter('active_depth_camera', 'front')
        self.declare_parameter('depth_safety_stop_topic', '/depth_safety_stop')
        self.declare_parameter('depth_route_status_topic', '/depth_route_status')
        self.declare_parameter('target_status_topic', '/target_status')
        self.declare_parameter('roi_top_fraction', 0.2)
        self.declare_parameter('roi_bottom_fraction', 0.82)
        self.declare_parameter('roi_left_fraction', 0.0)
        self.declare_parameter('roi_right_fraction', 1.0)
        self.declare_parameter('left_sector_end_fraction', 1.0 / 3.0)
        self.declare_parameter('right_sector_start_fraction', 2.0 / 3.0)
        self.declare_parameter('camera_view_name', 'forward')
        self.declare_parameter('min_depth_m', 0.075)
        self.declare_parameter('max_depth_m', 3.0)
        self.declare_parameter('hard_stop_distance_m', 0.6)
        self.declare_parameter('route_block_distance_m', 0.6)
        self.declare_parameter('obstacle_pixel_fraction', 0.02)
        self.declare_parameter('minimum_valid_fraction', 0.25)
        self.declare_parameter('clearance_percentile', 10.0)
        self.declare_parameter('required_clear_frames', 3)
        self.declare_parameter('allow_front_obstacle_reroute', True)
        self.declare_parameter('input_timeout_sec', 1.0)
        self.declare_parameter('target_status_timeout_sec', 4.0)
        self.declare_parameter('target_depth_percentile', 30.0)
        self.declare_parameter('target_ignore_until_distance_m', 0.30)
        self.declare_parameter('minimum_target_confidence', 0.70)

        self._depth_topic = str(self.get_parameter('depth_image_topic').value)
        self._front_depth_topic = str(
            self.get_parameter('front_depth_image_topic').value).strip()
        self._rear_depth_topic = str(
            self.get_parameter('rear_depth_image_topic').value).strip()
        self._active_camera_topic = str(
            self.get_parameter('active_camera_topic').value).strip()
        self._active_depth_camera = self._normalize_camera_name(
            str(self.get_parameter('active_depth_camera').value))
        self._settings = {
            name: float(self.get_parameter(name).value)
            for name in (
                'roi_top_fraction',
                'roi_bottom_fraction',
                'roi_left_fraction',
                'roi_right_fraction',
                'left_sector_end_fraction',
                'right_sector_start_fraction',
                'min_depth_m',
                'max_depth_m',
                'hard_stop_distance_m',
                'route_block_distance_m',
                'obstacle_pixel_fraction',
                'minimum_valid_fraction',
                'clearance_percentile',
            )
        }
        self._required_clear_frames = max(
            1, int(self.get_parameter('required_clear_frames').value)
        )
        self._allow_front_obstacle_reroute = bool(
            self.get_parameter('allow_front_obstacle_reroute').value
        )
        validate_depth_settings(self._settings)
        self._input_timeout_sec = max(
            0.1, float(self.get_parameter('input_timeout_sec').value)
        )
        self._target_status_timeout_sec = max(
            0.1, float(self.get_parameter('target_status_timeout_sec').value)
        )
        self._target_depth_percentile = float(
            self.get_parameter('target_depth_percentile').value
        )
        self._target_ignore_until_distance_m = float(
            self.get_parameter('target_ignore_until_distance_m').value
        )
        self._minimum_target_confidence = float(
            self.get_parameter('minimum_target_confidence').value
        )
        self._camera_view_name = str(
            self.get_parameter('camera_view_name').value
        ).strip()
        self._target_bbox_norm = None
        self._target_confidence = 0.0
        self._last_target_status_time = None
        self._clear_frame_count = 0
        self._safety_stop_active = True
        self._last_depth_time = None
        self._last_depth_times = {}

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._stop_pub = self.create_publisher(
            Bool,
            str(self.get_parameter('depth_safety_stop_topic').value),
            10,
        )
        self._route_pub = self.create_publisher(
            String,
            str(self.get_parameter('depth_route_status_topic').value),
            10,
        )
        self._depth_subscriptions = []
        if self._front_depth_topic or self._rear_depth_topic:
            if self._front_depth_topic:
                self._depth_subscriptions.append(
                    self.create_subscription(
                        Image,
                        self._front_depth_topic,
                        lambda msg: self.depth_callback(msg, 'front'),
                        sensor_qos,
                    )
                )
            if self._rear_depth_topic:
                self._depth_subscriptions.append(
                    self.create_subscription(
                        Image,
                        self._rear_depth_topic,
                        lambda msg: self.depth_callback(msg, 'rear'),
                        sensor_qos,
                    )
                )
        else:
            self._depth_subscriptions.append(
                self.create_subscription(
                    Image,
                    self._depth_topic,
                    lambda msg: self.depth_callback(
                        msg, self._active_depth_camera),
                    sensor_qos,
                )
            )
        self._target_sub = self.create_subscription(
            String,
            str(self.get_parameter('target_status_topic').value),
            self.target_status_callback,
            10,
        )
        self._active_camera_sub = None
        if self._active_camera_topic:
            self._active_camera_sub = self.create_subscription(
                String,
                self._active_camera_topic,
                self.active_camera_callback,
                10,
            )
        self._watchdog_timer = self.create_timer(
            min(0.2, self._input_timeout_sec / 2.0),
            self._depth_watchdog_callback,
        )
        self.get_logger().info(
            'Depth route monitor started: depth_topic=%s, front_depth=%s, '
            'rear_depth=%s, active_camera=%s, stop_distance=%.2f m, '
            'route_distance=%.2f m, camera_view=%s'
            % (
                self._depth_topic,
                self._front_depth_topic or 'disabled',
                self._rear_depth_topic or 'disabled',
                self._active_depth_camera,
                self._settings['hard_stop_distance_m'],
                self._settings['route_block_distance_m'],
                self._camera_view_name,
            )
        )

    @staticmethod
    def _normalize_camera_name(value: str) -> str:
        text = str(value or '').strip().lower()
        if text in ('rear', 'rear_camera', 'back', 'reverse') or text.startswith('26'):
            return 'rear'
        return 'front'

    def _active_depth_topic(self) -> str:
        if self._active_depth_camera == 'rear' and self._rear_depth_topic:
            return self._rear_depth_topic
        if self._active_depth_camera == 'front' and self._front_depth_topic:
            return self._front_depth_topic
        return self._depth_topic

    def active_camera_callback(self, msg: String) -> None:
        selected = self._normalize_camera_name(msg.data)
        if selected == self._active_depth_camera:
            return
        self._active_depth_camera = selected
        self._clear_frame_count = 0
        self._safety_stop_active = True
        self._publish_stop(True)
        self.get_logger().info(
            'Depth safety switched to %s camera depth topic %s'
            % (self._active_depth_camera, self._active_depth_topic())
        )

    def target_status_callback(self, msg: String) -> None:
        try:
            status = json.loads(msg.data or '{}')
        except json.JSONDecodeError:
            status = {}
        bbox = status.get('target_bbox_norm') if isinstance(status, dict) else None
        detected = bool(status.get('target_detected', False)) if isinstance(status, dict) else False
        try:
            confidence = float(status.get('confidence', 0.0))
        except (TypeError, ValueError):
            confidence = 0.0
        self._target_bbox_norm = (
            bbox
            if detected
            and isinstance(bbox, list)
            and confidence >= self._minimum_target_confidence
            else None
        )
        self._target_confidence = confidence
        self._last_target_status_time = self.get_clock().now()

    def _fresh_target_bbox(self):
        if self._target_bbox_norm is None or self._last_target_status_time is None:
            return None
        age_sec = (
            self.get_clock().now() - self._last_target_status_time
        ).nanoseconds / 1e9
        return (
            self._target_bbox_norm
            if age_sec <= self._target_status_timeout_sec
            else None
        )

    def depth_callback(self, msg: Image, camera_name: str = 'front') -> None:
        camera_name = self._normalize_camera_name(camera_name)
        now = self.get_clock().now()
        self._last_depth_times[camera_name] = now
        if camera_name != self._active_depth_camera:
            return
        self._last_depth_time = now
        try:
            depth_m = depth_image_to_meters(msg)
            target_bbox = self._fresh_target_bbox()
            target_distance_m = target_depth_from_bbox(
                depth_m,
                target_bbox,
                self._settings['min_depth_m'],
                self._settings['max_depth_m'],
                self._target_depth_percentile,
            )
            route_depth_m = depth_m
            if (
                target_bbox is not None
                and target_distance_m is not None
                and target_distance_m > self._target_ignore_until_distance_m
            ):
                # A high-confidence trash-can bbox is the mission goal, not a
                # generic obstacle. Other pixels in the same route remain gated.
                route_depth_m = mask_bbox(depth_m, target_bbox)
            sectors = summarize_depth_image(route_depth_m, **self._settings)
        except (TypeError, ValueError) as exc:
            self.get_logger().error(
                'Cannot process depth image: %s' % exc,
                throttle_duration_sec=2.0,
            )
            self._clear_frame_count = 0
            self._safety_stop_active = True
            self._publish_stop(True)
            self._publish_unavailable_status('depth_decode_error')
            return

        front = sectors['front']
        invalid_front = front.valid_fraction < self._settings['minimum_valid_fraction']
        front_obstacle = (
            front.hard_stop_fraction >= self._settings['obstacle_pixel_fraction']
        )
        side_route_available = sectors['left'].clear or sectors['right'].clear
        immediate_stop = invalid_front or (
            front_obstacle
            and (
                not self._allow_front_obstacle_reroute
                or not side_route_available
            )
        )
        depth_reason = 'clear'
        if immediate_stop:
            self._clear_frame_count = 0
            self._safety_stop_active = True
            if invalid_front:
                depth_reason = 'depth_insufficient_valid_data'
            elif side_route_available and not self._allow_front_obstacle_reroute:
                depth_reason = 'depth_front_obstacle_stop'
            else:
                depth_reason = 'depth_obstacle_no_side_route'
        else:
            self._clear_frame_count += 1
            if self._clear_frame_count >= self._required_clear_frames:
                self._safety_stop_active = False
            if front_obstacle:
                depth_reason = 'depth_front_obstacle_reroute'
            elif self._safety_stop_active:
                depth_reason = 'depth_clear_confirmation_pending'

        self._publish_stop(self._safety_stop_active)
        status = {
            'sensor': 'camera_depth',
            'camera_view': self._camera_view_name,
            'active_depth_camera': self._active_depth_camera,
            'depth_topic': self._active_depth_topic(),
            'frame_id': msg.header.frame_id,
            'stamp': {
                'sec': int(msg.header.stamp.sec),
                'nanosec': int(msg.header.stamp.nanosec),
            },
            'transform_ok': True,
            'hard_stop': bool(self._safety_stop_active),
            'reason': depth_reason,
            'front_obstacle': bool(front_obstacle),
            'side_route_available': bool(side_route_available),
            'clear': {route: sector.clear for route, sector in sectors.items()},
            'blocked': {route: not sector.clear for route, sector in sectors.items()},
            'clearance_m': {
                route: sector.clearance_m for route, sector in sectors.items()
            },
            'valid_fraction': {
                route: sector.valid_fraction for route, sector in sectors.items()
            },
            # This separates a genuinely blank/invalid depth image from a
            # sparse RealSense frame whose valid samples are all beyond the
            # near-obstacle thresholds.  It is diagnostic only; no missing
            # depth is ever considered clear.
            'finite_positive_fraction': {
                route: sector.finite_positive_fraction
                for route, sector in sectors.items()
            },
            'hard_stop_fraction': {
                route: sector.hard_stop_fraction for route, sector in sectors.items()
            },
            'route_block_fraction': {
                route: sector.route_block_fraction for route, sector in sectors.items()
            },
            'target_distance_m': target_distance_m,
        }
        route_msg = String()
        route_msg.data = json.dumps(status, sort_keys=True)
        self._route_pub.publish(route_msg)
        self.get_logger().info(
            'Depth clearance_m=%s, valid=%s, observed=%s, clear=%s, '
            'safety_stop=%s, reason=%s'
            % (
                status['clearance_m'], status['valid_fraction'],
                status['finite_positive_fraction'], status['clear'],
                self._safety_stop_active, depth_reason,
            ),
            throttle_duration_sec=1.0,
        )

    def _depth_watchdog_callback(self) -> None:
        active_last_depth_time = self._last_depth_times.get(
            self._active_depth_camera,
            self._last_depth_time,
        )
        stale = active_last_depth_time is None
        if active_last_depth_time is not None:
            age_sec = (
                self.get_clock().now() - active_last_depth_time
            ).nanoseconds / 1e9
            stale = age_sec > self._input_timeout_sec
        if not stale:
            return

        self._clear_frame_count = 0
        self._safety_stop_active = True
        self._publish_stop(True)
        self._publish_unavailable_status('depth_input_stale')
        self.get_logger().warn(
            'Depth input is missing or stale; holding depth safety stop.',
            throttle_duration_sec=2.0,
        )

    def _publish_unavailable_status(self, reason: str) -> None:
        status = {
            'sensor': 'camera_depth',
            'camera_view': self._camera_view_name,
            'active_depth_camera': self._active_depth_camera,
            'depth_topic': self._active_depth_topic(),
            'frame_id': '',
            'transform_ok': False,
            'hard_stop': True,
            'reason': str(reason),
            'clear': {route: False for route in ROUTES},
            'blocked': {route: True for route in ROUTES},
            'clearance_m': {route: None for route in ROUTES},
            'valid_fraction': {route: 0.0 for route in ROUTES},
            'finite_positive_fraction': {route: 0.0 for route in ROUTES},
            'hard_stop_fraction': {route: 0.0 for route in ROUTES},
            'route_block_fraction': {route: 0.0 for route in ROUTES},
        }
        msg = String()
        msg.data = json.dumps(status, sort_keys=True)
        self._route_pub.publish(msg)

    def _publish_stop(self, active: bool) -> None:
        msg = Bool()
        msg.data = bool(active)
        self._stop_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthRouteMonitorNode()
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
