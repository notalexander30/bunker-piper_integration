#!/usr/bin/env python3
# Copyright 2026 Dase Orin
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd
"""Estimate stationary D435i gyro bias and publish corrected IMU data."""

from __future__ import annotations

import copy
import math
import time
from typing import List, Sequence, Tuple

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu


AXES = ("x", "y", "z")


class RunningVectorStatistics:
    """Numerically stable running mean and variance for a three-vector."""

    def __init__(self) -> None:
        """Initialize an empty accumulator."""
        self.reset()

    def reset(self) -> None:
        """Discard all accumulated samples."""
        self.count = 0
        self.mean = [0.0, 0.0, 0.0]
        self.m2 = [0.0, 0.0, 0.0]

    def add(self, values: Sequence[float]) -> None:
        """Add one x/y/z sample using Welford's online algorithm."""
        self.count += 1
        for index, value in enumerate(values):
            delta = value - self.mean[index]
            self.mean[index] += delta / self.count
            delta_after_mean = value - self.mean[index]
            self.m2[index] += delta * delta_after_mean

    def variance(self) -> List[float]:
        """Return unbiased sample variance for each axis."""
        if self.count < 2:
            return [math.inf, math.inf, math.inf]
        return [value / (self.count - 1) for value in self.m2]

    def standard_deviation(self) -> List[float]:
        """Return unbiased sample standard deviation for each axis."""
        return [math.sqrt(value) for value in self.variance()]


def _three_finite_values(node: Node, name: str) -> Tuple[float, float, float]:
    values = list(node.get_parameter(name).value)
    if len(values) != 3 or not all(
        math.isfinite(float(value)) for value in values
    ):
        raise ValueError(f"parameter {name} must contain three finite numbers")
    return (float(values[0]), float(values[1]), float(values[2]))


class ImuBiasCorrector(Node):
    """Gate startup calibration on stationary wheel feedback."""

    def __init__(self) -> None:
        """Create subscriptions, publisher, and calibration state."""
        super().__init__("imu_bias_corrector")

        self.declare_parameter("input_topic", "/camera/camera/imu")
        self.declare_parameter(
            "output_topic", "/camera/camera/imu/bias_corrected"
        )
        self.declare_parameter("wheel_odometry_topic", "/wheel/odometry")
        self.declare_parameter("calibration_samples", 1000)
        self.declare_parameter("stationary_hold_seconds", 1.0)
        self.declare_parameter("wheel_message_timeout_seconds", 0.25)
        self.declare_parameter("stationary_linear_speed_threshold", 0.01)
        self.declare_parameter("stationary_angular_speed_threshold", 0.01)
        self.declare_parameter(
            "max_standard_deviation", [0.02, 0.02, 0.02]
        )
        self.declare_parameter("max_absolute_bias", [0.05, 0.05, 0.05])
        self.declare_parameter("angular_covariance_floor", 0.0001)

        self.calibration_samples = int(
            self.get_parameter("calibration_samples").value
        )
        self.stationary_hold = float(
            self.get_parameter("stationary_hold_seconds").value
        )
        self.wheel_timeout = float(
            self.get_parameter("wheel_message_timeout_seconds").value
        )
        self.linear_speed_threshold = float(
            self.get_parameter("stationary_linear_speed_threshold").value
        )
        self.angular_speed_threshold = float(
            self.get_parameter("stationary_angular_speed_threshold").value
        )
        self.max_standard_deviation = _three_finite_values(
            self, "max_standard_deviation"
        )
        self.max_absolute_bias = _three_finite_values(
            self, "max_absolute_bias"
        )
        self.covariance_floor = float(
            self.get_parameter("angular_covariance_floor").value
        )
        self._validate_parameters()

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        wheel_topic = str(self.get_parameter("wheel_odometry_topic").value)

        wheel_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.corrected_publisher = self.create_publisher(
            Imu, output_topic, qos_profile_sensor_data
        )
        self.imu_subscription = self.create_subscription(
            Imu, input_topic, self._imu_callback, qos_profile_sensor_data
        )
        self.wheel_subscription = self.create_subscription(
            Odometry, wheel_topic, self._wheel_callback, wheel_qos
        )

        self.statistics = RunningVectorStatistics()
        self.bias = [0.0, 0.0, 0.0]
        self.bias_variance = [0.0, 0.0, 0.0]
        self.calibrated = False
        self.wheel_stationary = False
        self.wheel_stationary_since: float | None = None
        self.last_wheel_reception: float | None = None
        self.last_status_log = 0.0

        self.get_logger().info(
            f"Waiting for stationary wheel feedback, then collecting "
            f"{self.calibration_samples} gyroscope samples from "
            f"{input_topic}; "
            f"corrected output will be {output_topic}"
        )

    def _validate_parameters(self) -> None:
        positive_scalars = {
            "stationary_hold_seconds": self.stationary_hold,
            "wheel_message_timeout_seconds": self.wheel_timeout,
            "stationary_linear_speed_threshold": self.linear_speed_threshold,
            "stationary_angular_speed_threshold": self.angular_speed_threshold,
            "angular_covariance_floor": self.covariance_floor,
        }
        if self.calibration_samples < 2:
            raise ValueError("calibration_samples must be at least 2")
        for name, value in positive_scalars.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(
                    f"parameter {name} must be finite and positive"
                )
        for name, values in (
            ("max_standard_deviation", self.max_standard_deviation),
            ("max_absolute_bias", self.max_absolute_bias),
        ):
            if any(value <= 0.0 for value in values):
                raise ValueError(f"all entries in {name} must be positive")

    def _wheel_callback(self, message: Odometry) -> None:
        now = time.monotonic()
        self.last_wheel_reception = now
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        twist_values = (
            linear.x,
            linear.y,
            linear.z,
            angular.x,
            angular.y,
            angular.z,
        )
        finite = all(math.isfinite(value) for value in twist_values)
        linear_speed = math.sqrt(
            linear.x * linear.x + linear.y * linear.y + linear.z * linear.z
        ) if finite else math.inf
        angular_speed = math.sqrt(
            angular.x * angular.x
            + angular.y * angular.y
            + angular.z * angular.z
        ) if finite else math.inf
        stationary = (
            linear_speed <= self.linear_speed_threshold
            and angular_speed <= self.angular_speed_threshold
        )

        if stationary:
            if not self.wheel_stationary:
                self.wheel_stationary_since = now
            self.wheel_stationary = True
            return

        self.wheel_stationary = False
        self.wheel_stationary_since = None
        if not self.calibrated:
            reason = "non-finite wheel twist" if not finite else (
                f"wheel motion (linear={linear_speed:.4f} m/s, "
                f"angular={angular_speed:.4f} rad/s)"
            )
            self._reset_calibration(reason)

    def _imu_callback(self, message: Imu) -> None:
        angular_velocity = message.angular_velocity
        values = (
            float(angular_velocity.x),
            float(angular_velocity.y),
            float(angular_velocity.z),
        )
        if not all(math.isfinite(value) for value in values):
            if not self.calibrated:
                self._reset_calibration("non-finite gyroscope sample")
            return

        if self.calibrated:
            self._publish_corrected(message)
            return

        now = time.monotonic()
        if (
            self.last_wheel_reception is None
            or now - self.last_wheel_reception > self.wheel_timeout
        ):
            self._reset_calibration("wheel odometry missing or stale")
            # Motion during a feedback gap cannot be excluded. Require a new
            # wheel message and the full stationary hold before retrying.
            self.wheel_stationary = False
            self.wheel_stationary_since = None
            self._log_waiting("Waiting for fresh wheel odometry")
            return
        if not self.wheel_stationary or self.wheel_stationary_since is None:
            self._reset_calibration("wheel is not stationary")
            return
        if now - self.wheel_stationary_since < self.stationary_hold:
            self._reset_calibration("stationary hold interval not complete")
            self._log_waiting(
                "Waiting for uninterrupted stationary hold interval"
            )
            return

        self.statistics.add(values)
        if self.statistics.count % 200 == 0:
            self.get_logger().info(
                f"Gyroscope calibration: {self.statistics.count}/"
                f"{self.calibration_samples} stationary samples"
            )
        if self.statistics.count < self.calibration_samples:
            return

        means = list(self.statistics.mean)
        variances = self.statistics.variance()
        standard_deviations = self.statistics.standard_deviation()
        rejected_axes = []
        for index, axis in enumerate(AXES):
            if abs(means[index]) > self.max_absolute_bias[index]:
                rejected_axes.append(
                    f"{axis} bias {means[index]:.6f} > "
                    f"{self.max_absolute_bias[index]:.6f} rad/s"
                )
            if standard_deviations[index] > self.max_standard_deviation[index]:
                rejected_axes.append(
                    f"{axis} stddev {standard_deviations[index]:.6f} > "
                    f"{self.max_standard_deviation[index]:.6f} rad/s"
                )

        if rejected_axes:
            self.get_logger().warning(
                "Rejected gyroscope calibration batch: "
                + "; ".join(rejected_axes)
            )
            self.statistics.reset()
            self.wheel_stationary_since = now
            return

        self.bias = means
        self.bias_variance = variances
        self.calibrated = True
        self.get_logger().info(
            "Gyroscope bias calibration complete: "
            + ", ".join(
                f"{axis}={means[index]:.7f} rad/s "
                f"(stddev={standard_deviations[index]:.7f})"
                for index, axis in enumerate(AXES)
            )
        )
        self._publish_corrected(message)

    def _reset_calibration(self, reason: str) -> None:
        if self.statistics.count > 0:
            self.get_logger().warning(
                f"Reset gyroscope calibration after {self.statistics.count} "
                f"samples: {reason}"
            )
        self.statistics.reset()

    def _log_waiting(self, text: str) -> None:
        now = time.monotonic()
        if now - self.last_status_log >= 5.0:
            self.get_logger().info(text)
            self.last_status_log = now

    def _publish_corrected(self, source: Imu) -> None:
        corrected = copy.deepcopy(source)
        corrected.angular_velocity.x -= self.bias[0]
        corrected.angular_velocity.y -= self.bias[1]
        corrected.angular_velocity.z -= self.bias[2]

        covariance = list(source.angular_velocity_covariance)
        covariance_unknown = (
            len(covariance) != 9
            or covariance[0] < 0.0
            or not all(math.isfinite(value) for value in covariance)
        )
        if covariance_unknown:
            covariance = [0.0] * 9
        for axis_index, covariance_index in enumerate((0, 4, 8)):
            reported_variance = max(0.0, covariance[covariance_index])
            covariance[covariance_index] = (
                max(reported_variance, self.covariance_floor)
                + self.bias_variance[axis_index]
            )
        corrected.angular_velocity_covariance = covariance
        self.corrected_publisher.publish(corrected)


def main(args: Sequence[str] | None = None) -> None:
    """Run the IMU bias correction node until ROS shutdown."""
    rclpy.init(args=args)
    node: ImuBiasCorrector | None = None
    try:
        node = ImuBiasCorrector()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # Humble's rclpy/Fast DDS binding can throw while taking a message if
        # SIGINT invalidates the context at the same instant. Suppress only
        # that shutdown case; a RuntimeError while ROS is live remains fatal.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
