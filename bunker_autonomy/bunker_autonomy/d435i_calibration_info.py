#!/usr/bin/env python3

import json
import os
import time
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from bunker_autonomy.d435i_calibration_node import load_view_settings


ROUTE_ORDER = ('left', 'front', 'right')


def find_latest_report(report_directory: str) -> Optional[Path]:
    directory = Path(os.path.expanduser(report_directory))
    if not directory.is_dir():
        return None
    reports = sorted(directory.glob('d435i_calibration_*.yaml'))
    return reports[-1] if reports else None


def format_view_settings(settings: Dict[str, float]) -> List[str]:
    return [
        'Active navigation view:',
        '  ROI: left={:.3f}, right={:.3f}, top={:.3f}, bottom={:.3f}'.format(
            settings['roi_left_fraction'],
            settings['roi_right_fraction'],
            settings['roi_top_fraction'],
            settings['roi_bottom_fraction'],
        ),
        '  Sectors: LEFT < {:.3f}, FRONT {:.3f}..{:.3f}, RIGHT > {:.3f}'.format(
            settings['left_sector_end_fraction'],
            settings['left_sector_end_fraction'],
            settings['right_sector_start_fraction'],
            settings['right_sector_start_fraction'],
        ),
        '  Depth: valid {:.3f}..{:.3f} m, stop/block at {:.3f}/{:.3f} m'.format(
            settings['min_depth_m'],
            settings['max_depth_m'],
            settings['hard_stop_distance_m'],
            settings['route_block_distance_m'],
        ),
    ]


def format_live_status(status_text: str) -> List[str]:
    try:
        status = json.loads(status_text)
    except json.JSONDecodeError:
        return ['Live calibration status is not valid JSON.']
    if not isinstance(status, dict):
        return ['Live calibration status has an unexpected format.']

    camera = status.get('camera_info') or {}
    lines = [
        '',
        'LIVE D435i CALIBRATION READY',
        '  Frame: {}'.format(camera.get('frame_id', 'unknown')),
        '  RGB intrinsics size: {}x{}'.format(
            camera.get('width', 'unknown'),
            camera.get('height', 'unknown'),
        ),
        '  Field of view: horizontal={} deg, vertical={} deg'.format(
            camera.get('horizontal_fov_deg', 'unknown'),
            camera.get('vertical_fov_deg', 'unknown'),
        ),
        '  Aligned depth size: {}'.format(status.get('depth_size', 'unknown')),
    ]
    sectors = status.get('sectors') or {}
    for route in ROUTE_ORDER:
        sector = sectors.get(route) or {}
        lines.append(
            '  {:>5}: clear={}, clearance_m={}, valid_fraction={}'.format(
                route.upper(),
                sector.get('clear', 'unknown'),
                sector.get('clearance_m', 'unknown'),
                sector.get('valid_fraction', 'unknown'),
            )
        )
    lines.extend([
        '  Fresh report: {}'.format(status.get('report_path') or 'not written yet'),
        '  Mount extrinsics: {}'.format(
            status.get('mount_extrinsics_status', 'unknown')
        ),
        '',
        'Keep the arm locked in this pose. Inspect the RViz overlay in window 4.',
        'Detach with Ctrl-b then d, then add runtime windows with:',
        '  ./tools/start-five-terminal-pipeline dry_run',
    ])
    return lines


class D435iCalibrationInfo(Node):
    def __init__(self):
        super().__init__('d435i_calibration_info')
        self.declare_parameter(
            'config_file',
            '/ros2_ws/src/bunker_autonomy/config/autonomy_d435i.yaml',
        )
        self.declare_parameter(
            'report_directory',
            '~/vlm_results/d435i_calibration',
        )
        self.declare_parameter('wait_timeout_sec', 45.0)

        config_file = str(self.get_parameter('config_file').value)
        report_directory = str(self.get_parameter('report_directory').value)
        self._deadline = time.monotonic() + max(
            1.0,
            float(self.get_parameter('wait_timeout_sec').value),
        )
        self.done = False

        self.get_logger().info('D435i calibration preflight')
        self.get_logger().info('Configuration: %s' % config_file)
        for line in format_view_settings(load_view_settings(config_file)):
            self.get_logger().info(line)
        previous_report = find_latest_report(report_directory)
        if previous_report is None:
            self.get_logger().info('Previous report: none')
        else:
            self.get_logger().info('Previous report: %s' % previous_report)
        self.get_logger().info(
            'Waiting for fresh camera intrinsics and aligned-depth calibration data...'
        )

        self.create_subscription(
            String,
            '/d435i/calibration_status',
            self._status_callback,
            10,
        )
        self.create_timer(0.25, self._check_timeout)

    def _status_callback(self, msg: String) -> None:
        if self.done:
            return
        for line in format_live_status(msg.data):
            self.get_logger().info(line)
        self.done = True

    def _check_timeout(self) -> None:
        if self.done or time.monotonic() < self._deadline:
            return
        self.get_logger().error(
            'No live calibration data arrived. Check windows 2-camera and '
            '3-calib-node, the USB connection, and the D435i topics.'
        )
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = D435iCalibrationInfo()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
