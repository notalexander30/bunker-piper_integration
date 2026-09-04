#!/usr/bin/env python3
# Copyright 2026 Dase Orin
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd
"""Exit successfully after receiving one structurally valid corrected IMU."""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Optional, Sequence

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Imu


class CorrectedImuGate(Node):
    """Wait for a corrected IMU sample that is safe to feed to the EKF."""

    def __init__(self, topic: str) -> None:
        super().__init__("corrected_imu_startup_gate")
        self.valid_message_received = False
        self.invalid_message_count = 0
        self.subscription = self.create_subscription(
            Imu, topic, self._imu_callback, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"Waiting for a valid corrected IMU message on {topic}"
        )

    def _imu_callback(self, message: Imu) -> None:
        reason = self._invalid_reason(message)
        if reason is not None:
            self.invalid_message_count += 1
            if self.invalid_message_count == 1:
                self.get_logger().warning(
                    f"Ignoring invalid corrected IMU message: {reason}"
                )
            return

        self.valid_message_received = True
        self.get_logger().info(
            "Corrected IMU is ready; releasing RTAB-Map startup"
        )

    @staticmethod
    def _invalid_reason(message: Imu) -> Optional[str]:
        stamp = message.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return "header stamp is zero"
        if stamp.nanosec < 0 or stamp.nanosec >= 1_000_000_000:
            return "header nanoseconds are outside [0, 1e9)"
        if not message.header.frame_id.strip():
            return "header frame_id is empty"

        angular_velocity = message.angular_velocity
        if not all(
            math.isfinite(value)
            for value in (
                angular_velocity.x,
                angular_velocity.y,
                angular_velocity.z,
            )
        ):
            return "angular velocity contains a non-finite value"
        return None


def _parse_arguments(arguments: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Wait for one valid corrected sensor_msgs/Imu message."
    )
    parser.add_argument(
        "--topic",
        default="/camera/camera/imu/bias_corrected",
        help="Corrected sensor_msgs/Imu topic to inspect.",
    )
    parser.add_argument(
        "--timeout-seconds",
        type=float,
        default=30.0,
        help="Monotonic-clock timeout; must be finite and positive.",
    )
    return parser.parse_args(remove_ros_args(args=list(arguments))[1:])


def main(arguments: Optional[Sequence[str]] = None) -> int:
    raw_arguments = list(arguments) if arguments is not None else sys.argv
    parsed = _parse_arguments(raw_arguments)
    topic = parsed.topic.strip()
    if not topic:
        print("Corrected IMU topic must not be empty", file=sys.stderr)
        return 2
    if (
        not math.isfinite(parsed.timeout_seconds)
        or parsed.timeout_seconds <= 0.0
    ):
        print(
            "Corrected IMU timeout must be finite and positive",
            file=sys.stderr,
        )
        return 2

    rclpy.init(args=raw_arguments)
    node: Optional[CorrectedImuGate] = None
    try:
        node = CorrectedImuGate(topic)
        deadline = time.monotonic() + parsed.timeout_seconds
        while rclpy.ok() and not node.valid_message_received:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                node.get_logger().error(
                    f"Timed out after {parsed.timeout_seconds:.3f} seconds "
                    f"waiting for valid corrected IMU data on {topic}; "
                    f"invalid messages observed: {node.invalid_message_count}"
                )
                return 1
            rclpy.spin_once(node, timeout_sec=min(remaining, 0.1))

        if node.valid_message_received:
            return 0
        node.get_logger().error(
            "ROS shut down before corrected IMU readiness was established"
        )
        return 1
    except (KeyboardInterrupt, ExternalShutdownException):
        return 130
    except RuntimeError:
        if rclpy.ok():
            raise
        return 130
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
