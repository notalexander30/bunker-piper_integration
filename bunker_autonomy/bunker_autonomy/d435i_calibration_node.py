#!/usr/bin/env python3

import json
import math
import os
from datetime import datetime
from pathlib import Path
from typing import Dict

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from bunker_autonomy.depth_route_monitor_node import (
    depth_image_to_meters,
    summarize_depth_image,
)


VIEW_PARAMETER_NAMES = (
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

DEFAULT_VIEW_SETTINGS = {
    'roi_top_fraction': 0.20,
    'roi_bottom_fraction': 0.82,
    'roi_left_fraction': 0.0,
    'roi_right_fraction': 1.0,
    'left_sector_end_fraction': 1.0 / 3.0,
    'right_sector_start_fraction': 2.0 / 3.0,
    'min_depth_m': 0.175,
    'max_depth_m': 3.0,
    'hard_stop_distance_m': 0.05,
    'route_block_distance_m': 0.05,
    'obstacle_pixel_fraction': 0.02,
    'minimum_valid_fraction': 0.25,
    'clearance_percentile': 10.0,
}


def load_view_settings(config_file: str) -> Dict[str, float]:
    settings = dict(DEFAULT_VIEW_SETTINGS)
    if not config_file:
        return settings
    with open(os.path.expanduser(config_file), encoding='utf-8') as stream:
        document = yaml.safe_load(stream) or {}
    parameters = (
        document.get('depth_route_monitor_node', {}).get('ros__parameters', {})
        if isinstance(document, dict)
        else {}
    )
    for name in VIEW_PARAMETER_NAMES:
        if name in parameters:
            settings[name] = float(parameters[name])
    # Reuse the navigation implementation as the single source of validation.
    summarize_depth_image(np.ones((6, 9), dtype=np.float32), **settings)
    return settings


def camera_info_summary(msg: CameraInfo) -> Dict[str, object]:
    fx = float(msg.k[0])
    fy = float(msg.k[4])
    horizontal_fov_deg = (
        math.degrees(2.0 * math.atan(float(msg.width) / (2.0 * fx)))
        if fx > 0.0
        else None
    )
    vertical_fov_deg = (
        math.degrees(2.0 * math.atan(float(msg.height) / (2.0 * fy)))
        if fy > 0.0
        else None
    )
    return {
        'frame_id': msg.header.frame_id,
        'width': int(msg.width),
        'height': int(msg.height),
        'distortion_model': msg.distortion_model,
        'd': [float(value) for value in msg.d],
        'k': [float(value) for value in msg.k],
        'r': [float(value) for value in msg.r],
        'p': [float(value) for value in msg.p],
        'horizontal_fov_deg': (
            None if horizontal_fov_deg is None else round(horizontal_fov_deg, 3)
        ),
        'vertical_fov_deg': (
            None if vertical_fov_deg is None else round(vertical_fov_deg, 3)
        ),
    }


def make_depth_overlay(
    depth_m: np.ndarray,
    settings: Dict[str, float],
) -> np.ndarray:
    if depth_m.ndim != 2 or depth_m.size == 0:
        raise ValueError('depth image must be a non-empty 2-D array')
    minimum = float(settings['min_depth_m'])
    maximum = float(settings['max_depth_m'])
    valid = np.isfinite(depth_m) & (depth_m >= minimum) & (depth_m <= maximum)
    clipped = np.clip(depth_m, minimum, maximum)
    scaled = np.zeros(depth_m.shape, dtype=np.uint8)
    scaled[valid] = np.asarray(
        255.0 * (maximum - clipped[valid]) / max(1e-6, maximum - minimum),
        dtype=np.uint8,
    )
    overlay = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
    overlay[~valid] = (0, 0, 0)

    height, width = depth_m.shape

    def pixel_x(fraction: float) -> int:
        return max(0, min(width - 1, int(width * fraction)))

    x_values = {
        'roi_left': pixel_x(settings['roi_left_fraction']),
        'left/front': pixel_x(settings['left_sector_end_fraction']),
        'front/right': pixel_x(settings['right_sector_start_fraction']),
        'roi_right': pixel_x(settings['roi_right_fraction']),
    }
    top = int(height * settings['roi_top_fraction'])
    bottom = int(height * settings['roi_bottom_fraction'])
    for x in x_values.values():
        cv2.line(overlay, (x, top), (x, bottom), (255, 255, 255), 2)
    cv2.line(
        overlay,
        (x_values['roi_left'], top),
        (x_values['roi_right'], top),
        (255, 255, 255),
        2,
    )
    cv2.line(
        overlay,
        (x_values['roi_left'], bottom),
        (x_values['roi_right'], bottom),
        (255, 255, 255),
        2,
    )

    label_y = max(18, top + 20)
    centers = (
        ('LEFT', (x_values['roi_left'] + x_values['left/front']) // 2),
        ('FRONT', (x_values['left/front'] + x_values['front/right']) // 2),
        ('RIGHT', (x_values['front/right'] + x_values['roi_right']) // 2),
    )
    for label, center_x in centers:
        cv2.putText(
            overlay,
            label,
            (max(0, center_x - 30), label_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return overlay


class D435iCalibrationNode(Node):
    def __init__(self):
        super().__init__('d435i_calibration_node')
        self.declare_parameter('config_file', '')
        config_file = str(self.get_parameter('config_file').value)
        defaults = load_view_settings(config_file)

        self.declare_parameter(
            'depth_image_topic',
            '/camera/camera/aligned_depth_to_color/image_raw',
        )
        self.declare_parameter(
            'camera_info_topic',
            '/camera/camera/color/camera_info',
        )
        self.declare_parameter('overlay_topic', '/d435i/calibration_overlay')
        self.declare_parameter('status_topic', '/d435i/calibration_status')
        self.declare_parameter(
            'report_directory',
            '~/vlm_results/d435i_calibration',
        )
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        self._settings = {
            name: float(self.get_parameter(name).value)
            for name in VIEW_PARAMETER_NAMES
        }
        summarize_depth_image(
            np.ones((6, 9), dtype=np.float32),
            **self._settings,
        )
        self._config_file = os.path.expanduser(config_file)
        self._report_directory = os.path.expanduser(
            str(self.get_parameter('report_directory').value)
        )
        self._camera_info = None
        self._latest_depth_shape = None
        self._latest_sector_status = None
        self._report_path = None
        self._bridge = CvBridge()

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        # RViz image displays can request reliable delivery even when their
        # saved config says best-effort.  A reliable publisher interoperates
        # with both, unlike the camera's best-effort sensor subscription.
        overlay_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._overlay_pub = self.create_publisher(
            Image,
            str(self.get_parameter('overlay_topic').value),
            overlay_qos,
        )
        self._status_pub = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            10,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('depth_image_topic').value),
            self.depth_callback,
            sensor_qos,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self.camera_info_callback,
            sensor_qos,
        )
        self.get_logger().info(
            'D435i calibration overlay started. Adjust ROI fractions in %s.'
            % (self._config_file or 'the ROS parameters')
        )

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self._camera_info = camera_info_summary(msg)
        self._write_report_if_ready()

    def depth_callback(self, msg: Image) -> None:
        try:
            depth_m = depth_image_to_meters(msg)
            sectors = summarize_depth_image(depth_m, **self._settings)
            overlay = make_depth_overlay(depth_m, self._settings)
        except (TypeError, ValueError) as exc:
            self.get_logger().error(
                'Cannot create calibration overlay: %s' % exc,
                throttle_duration_sec=2.0,
            )
            return

        overlay_msg = self._bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        overlay_msg.header = msg.header
        self._overlay_pub.publish(overlay_msg)
        self._latest_depth_shape = [int(depth_m.shape[1]), int(depth_m.shape[0])]
        self._latest_sector_status = {
            route: {
                'clear': bool(sector.clear),
                'valid_fraction': round(float(sector.valid_fraction), 4),
                'clearance_m': sector.clearance_m,
                'obstacle_fraction': round(float(sector.route_block_fraction), 4),
            }
            for route, sector in sectors.items()
        }
        self._write_report_if_ready()
        status = String()
        status.data = json.dumps(
            {
                'ready': self._camera_info is not None,
                'camera_info': self._camera_info,
                'depth_size': self._latest_depth_shape,
                'view_settings': self._settings,
                'sectors': self._latest_sector_status,
                'report_path': self._report_path,
                'mount_extrinsics_status': 'measure_arm_pose_and_set_launch_arguments',
            },
            sort_keys=True,
        )
        self._status_pub.publish(status)

    def _write_report_if_ready(self) -> None:
        if (
            self._report_path is not None
            or self._camera_info is None
            or self._latest_depth_shape is None
        ):
            return
        Path(self._report_directory).mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        report_path = os.path.join(
            self._report_directory,
            'd435i_calibration_%s.yaml' % timestamp,
        )
        report = {
            'created_at': datetime.now().isoformat(timespec='seconds'),
            'camera_intrinsics': self._camera_info,
            'aligned_depth_size': self._latest_depth_shape,
            'view_settings': self._settings,
            'sector_sample': self._latest_sector_status,
            'mount_extrinsics': {
                'calibrated': False,
                'note': (
                    'Measure the locked arm pose from the chosen TF parent to '
                    'camera_link; an arbitrary scene cannot determine this transform.'
                ),
            },
        }
        try:
            with open(report_path, 'x', encoding='utf-8') as stream:
                yaml.safe_dump(report, stream, sort_keys=False)
        except OSError as exc:
            self.get_logger().error('Cannot write calibration report: %s' % exc)
            return
        self._report_path = report_path
        self.get_logger().info('Saved D435i calibration report: %s' % report_path)


def main(args=None):
    rclpy.init(args=args)
    node = D435iCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Launch sends SIGINT to the process group. On some ROS 2/Python
        # combinations that interrupt can arrive while destroy_node() is
        # releasing publishers, so keep normal shutdown quiet and idempotent.
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
