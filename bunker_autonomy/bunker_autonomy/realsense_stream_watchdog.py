#!/usr/bin/env python3
"""Restart RealSense processes when their shared USB hub drops streams."""

import os
from pathlib import Path
import signal
import time

from bunker_autonomy.realsense_usb_recover import find_devices, reset_device

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2


class RealSenseStreamWatchdog(Node):
    def __init__(self):
        super().__init__('realsense_stream_watchdog')
        self.declare_parameter('startup_grace_sec', 45.0)
        self.declare_parameter('stale_timeout_sec', 8.0)
        self.declare_parameter('restart_cooldown_sec', 30.0)
        self.declare_parameter('terminate_grace_sec', 3.0)
        self.declare_parameter('reset_usb_on_restart', True)
        self.declare_parameter('image_topics', [
            '/front_camera/color/image_raw',
            '/rear_camera/color/image_raw',
        ])
        self.declare_parameter('pointcloud_topics', [
            '/front_camera/depth/color/points',
            '/rear_camera/depth/color/points',
        ])
        self.startup_grace = float(
            self.get_parameter('startup_grace_sec').value)
        self.stale_timeout = float(
            self.get_parameter('stale_timeout_sec').value)
        self.restart_cooldown = float(
            self.get_parameter('restart_cooldown_sec').value)
        self.terminate_grace = max(
            0.1, float(self.get_parameter('terminate_grace_sec').value))
        self.reset_usb_on_restart = bool(
            self.get_parameter('reset_usb_on_restart').value)

        image_topics = [
            str(topic)
            for topic in self.get_parameter('image_topics').value
            if str(topic)
        ]
        pointcloud_topics = [
            str(topic)
            for topic in self.get_parameter('pointcloud_topics').value
            if str(topic)
        ]
        self.topics = {topic: Image for topic in image_topics}
        self.topics.update({
            topic: PointCloud2 for topic in pointcloud_topics
        })
        self.last_seen = {topic: None for topic in self.topics}
        self._stream_subscriptions = []
        for topic, message_type in self.topics.items():
            self._stream_subscriptions.append(self.create_subscription(
                message_type,
                topic,
                lambda _msg, name=topic: self._mark_seen(name),
                qos_profile_sensor_data,
            ))

        now = time.monotonic()
        self.grace_until = now + self.startup_grace
        self.last_restart = now - self.restart_cooldown
        self.last_power_pin = 0.0
        self.timer = self.create_timer(1.0, self._check)
        self.get_logger().info(
            f'Watching RealSense streams: {list(self.topics)}; stale camera '
            'processes will be removed so the launch can respawn only the '
            'RealSense node. This watchdog never restarts Docker or the Jetson.')

    def _mark_seen(self, topic):
        self.last_seen[topic] = time.monotonic()

    @staticmethod
    def _camera_pids():
        pids = []
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():
                continue
            try:
                cmdline = (entry / 'cmdline').read_bytes().replace(b'\0', b' ')
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                continue
            if b'realsense2_camera_node' not in cmdline:
                continue
            if (
                b'__node:=front_camera' in cmdline
                or b'__node:=rear_camera' in cmdline
            ):
                pids.append(int(entry.name))
        return pids

    @staticmethod
    def _pin_camera_usb_power():
        devices = Path('/sys/bus/usb/devices')
        for vendor_file in devices.glob('*/idVendor'):
            try:
                vendor = vendor_file.read_text(
                    encoding='utf-8').strip().lower()
                if vendor != '8086':
                    continue
                current = vendor_file.parent.resolve()
                while current != devices and current.name != 'usb2':
                    control = current / 'power/control'
                    if control.exists():
                        try:
                            control.write_text('on', encoding='utf-8')
                        except OSError:
                            pass
                    current = current.parent
            except (FileNotFoundError, OSError):
                continue

    @staticmethod
    def _pid_exists(pid):
        return (Path('/proc') / str(pid)).exists()

    def _terminate_camera_pids(self, pids):
        if not pids:
            return
        for pid in pids:
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

        deadline = time.monotonic() + self.terminate_grace
        while time.monotonic() < deadline:
            if not any(self._pid_exists(pid) for pid in pids):
                return
            time.sleep(0.1)

        lingering = [pid for pid in pids if self._pid_exists(pid)]
        if not lingering:
            return
        self.get_logger().warning(
            f'RealSense processes ignored SIGTERM; forcing SIGKILL: {lingering}')
        for pid in lingering:
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    def _reset_usb_devices(self):
        if not self.reset_usb_on_restart:
            return
        try:
            devices = find_devices()
        except OSError as exc:
            self.get_logger().warning(f'Unable to enumerate RealSense USB devices: {exc}')
            return
        if not devices:
            self.get_logger().error(
                'No RealSense USB device is currently enumerated. There is no '
                'camera device for the watchdog to reset; keep the Jetson and '
                'container running, then recover only the camera USB path '
                '(check cable/power, replug the D435i, or run '
                'realsense_usb_recover after the camera reappears).')
            return
        for device in devices:
            try:
                reset_device(device)
            except RuntimeError as exc:
                self.get_logger().warning(f'RealSense USB reset skipped/failed: {exc}')

    def _check(self):
        now = time.monotonic()
        if now - self.last_power_pin >= 30.0:
            self._pin_camera_usb_power()
            self.last_power_pin = now
        if (
            now < self.grace_until
            or now - self.last_restart < self.restart_cooldown
        ):
            return

        stale = [
            topic for topic, stamp in self.last_seen.items()
            if stamp is None or now - stamp > self.stale_timeout
        ]
        if not stale:
            return

        pids = self._camera_pids()
        self.get_logger().error(
            f'RealSense streams stale: {stale}; removing camera processes '
            f'{pids} so launch respawns only the RealSense node')
        self._pin_camera_usb_power()
        self._terminate_camera_pids(pids)
        self._reset_usb_devices()
        self.last_seen = {topic: None for topic in self.topics}
        self.last_restart = now
        self.grace_until = now + self.startup_grace


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseStreamWatchdog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
