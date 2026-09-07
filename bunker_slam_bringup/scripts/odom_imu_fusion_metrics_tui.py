#!/usr/bin/env python3
# Copyright 2026 Dase Orin
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd
"""One-terminal odometry/IMU/fused odometry metric TUI.

This diagnostic is intentionally passive. It never publishes commands, resets
localization, or changes EKF/Nav2 parameters.
"""

from __future__ import annotations

import argparse
import curses
from dataclasses import dataclass, field
import math
import statistics
import sys
import threading
import time
from typing import Dict, List, Optional, Sequence, Tuple

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Imu


Pose2 = Tuple[float, float, float]


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def now_seconds() -> float:
    return time.monotonic()


def mean(values: Sequence[float]) -> float:
    return statistics.fmean(values) if values else 0.0


def stdev(values: Sequence[float]) -> float:
    return statistics.pstdev(values) if len(values) > 1 else 0.0


@dataclass
class OdomSample:
    stamp: float
    x: float
    y: float
    yaw: float
    vx: float
    wz: float


@dataclass
class ImuSample:
    stamp: float
    wz: float


@dataclass
class TopicStats:
    count: int = 0
    first_stamp: Optional[float] = None
    last_stamp: Optional[float] = None

    def record(self, stamp: float) -> None:
        self.count += 1
        if self.first_stamp is None:
            self.first_stamp = stamp
        self.last_stamp = stamp

    def age(self) -> Optional[float]:
        if self.last_stamp is None:
            return None
        return now_seconds() - self.last_stamp

    def hz(self) -> float:
        if self.count < 2 or self.first_stamp is None or self.last_stamp is None:
            return 0.0
        duration = max(self.last_stamp - self.first_stamp, 1e-6)
        return float(self.count - 1) / duration


@dataclass
class TrialMetrics:
    label: str
    trial_type: str
    started_at: float
    stopped_at: float
    wheel_start: Optional[Pose2]
    wheel_end: Optional[Pose2]
    fused_start: Optional[Pose2]
    fused_end: Optional[Pose2]
    wheel_vx_noise: float
    wheel_wz_noise: float
    fused_vx_noise: float
    fused_wz_noise: float
    imu_integrated_yaw: float
    imu_wz_bias: float
    imu_wz_noise: float
    wheel_samples: int
    fused_samples: int
    imu_samples: int

    @property
    def duration(self) -> float:
        return max(self.stopped_at - self.started_at, 0.0)

    def odom_delta(self, source: str) -> Optional[Pose2]:
        start = self.wheel_start if source == "wheel" else self.fused_start
        end = self.wheel_end if source == "wheel" else self.fused_end
        if start is None or end is None:
            return None
        return (
            end[0] - start[0],
            end[1] - start[1],
            wrap_angle(end[2] - start[2]),
        )

    def distance_error(self, source: str) -> Optional[float]:
        delta = self.odom_delta(source)
        if delta is None:
            return None
        return math.hypot(delta[0], delta[1])

    def yaw_error(self, source: str) -> Optional[float]:
        delta = self.odom_delta(source)
        if delta is None:
            return None
        return abs(delta[2])

    def yaw_vs_imu_error(self, source: str) -> Optional[float]:
        delta = self.odom_delta(source)
        if delta is None or self.imu_samples < 2:
            return None
        return abs(wrap_angle(delta[2] - self.imu_integrated_yaw))


@dataclass
class TrialBuffer:
    label: str
    trial_type: str
    started_at: float
    wheel: List[OdomSample] = field(default_factory=list)
    fused: List[OdomSample] = field(default_factory=list)
    imu: List[ImuSample] = field(default_factory=list)
    auto_drive: bool = False


class MetricCollector(Node):
    def __init__(
        self,
        wheel_topic: str,
        fused_topic: str,
        imu_topic: str,
        cmd_topic: str,
        drive_mode: bool,
    ) -> None:
        super().__init__("odom_imu_fusion_metrics_tui")
        self._lock = threading.Lock()
        self._active: Optional[TrialBuffer] = None
        self._trials: List[TrialMetrics] = []
        self._drive_mode = drive_mode
        self._cmd_pub = self.create_publisher(Twist, cmd_topic, 10) if drive_mode else None
        self._stats: Dict[str, TopicStats] = {
            "wheel": TopicStats(),
            "fused": TopicStats(),
            "imu": TopicStats(),
        }
        self._last_samples: Dict[str, object] = {}
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Odometry, wheel_topic, self._wheel_cb, qos)
        self.create_subscription(Odometry, fused_topic, self._fused_cb, qos)
        self.create_subscription(Imu, imu_topic, self._imu_cb, qos)

    def _sample_odom(self, msg: Odometry) -> OdomSample:
        pose = msg.pose.pose
        twist = msg.twist.twist
        return OdomSample(
            stamp=now_seconds(),
            x=float(pose.position.x),
            y=float(pose.position.y),
            yaw=yaw_from_quaternion(
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            ),
            vx=float(twist.linear.x),
            wz=float(twist.angular.z),
        )

    def _wheel_cb(self, msg: Odometry) -> None:
        sample = self._sample_odom(msg)
        with self._lock:
            self._stats["wheel"].record(sample.stamp)
            self._last_samples["wheel"] = sample
            if self._active is not None:
                self._active.wheel.append(sample)

    def _fused_cb(self, msg: Odometry) -> None:
        sample = self._sample_odom(msg)
        with self._lock:
            self._stats["fused"].record(sample.stamp)
            self._last_samples["fused"] = sample
            if self._active is not None:
                self._active.fused.append(sample)

    def _imu_cb(self, msg: Imu) -> None:
        sample = ImuSample(stamp=now_seconds(), wz=float(msg.angular_velocity.z))
        with self._lock:
            self._stats["imu"].record(sample.stamp)
            self._last_samples["imu"] = sample
            if self._active is not None:
                self._active.imu.append(sample)

    def start_trial(self, label: str, trial_type: str) -> bool:
        with self._lock:
            if self._active is not None:
                return False
            self._active = TrialBuffer(
                label=label,
                trial_type=trial_type,
                started_at=now_seconds(),
                auto_drive=self._drive_mode,
            )
            return True

    def stop_trial(self) -> Optional[TrialMetrics]:
        with self._lock:
            active = self._active
            self._active = None
        if active is None:
            return None
        metrics = build_metrics(active, now_seconds())
        self.publish_zero()
        with self._lock:
            self._trials.append(metrics)
        return metrics

    def publish_zero(self) -> None:
        if self._cmd_pub is not None:
            self._cmd_pub.publish(Twist())

    def publish_drive_command(self) -> Optional[str]:
        if self._cmd_pub is None:
            return None
        with self._lock:
            active = self._active
        if active is None or not active.auto_drive:
            self.publish_zero()
            return None
        elapsed = now_seconds() - active.started_at
        command, done = drive_profile(active.trial_type, elapsed)
        self._cmd_pub.publish(command)
        if done:
            self.stop_trial()
            return f"auto-saved {active.trial_type}"
        return None

    def snapshot(self) -> Tuple[Dict[str, TopicStats], Dict[str, object], Optional[TrialBuffer], List[TrialMetrics]]:
        with self._lock:
            stats = {
                name: TopicStats(
                    count=value.count,
                    first_stamp=value.first_stamp,
                    last_stamp=value.last_stamp,
                )
                for name, value in self._stats.items()
            }
            return stats, dict(self._last_samples), self._active, list(self._trials)


def first_pose(samples: Sequence[OdomSample]) -> Optional[Pose2]:
    if not samples:
        return None
    sample = samples[0]
    return sample.x, sample.y, sample.yaw


def last_pose(samples: Sequence[OdomSample]) -> Optional[Pose2]:
    if not samples:
        return None
    sample = samples[-1]
    return sample.x, sample.y, sample.yaw


def integrate_imu_yaw(samples: Sequence[ImuSample]) -> float:
    if len(samples) < 2:
        return 0.0
    total = 0.0
    for previous, current in zip(samples, samples[1:]):
        dt = max(current.stamp - previous.stamp, 0.0)
        total += 0.5 * (previous.wz + current.wz) * dt
    return wrap_angle(total)


def build_metrics(buffer: TrialBuffer, stopped_at: float) -> TrialMetrics:
    return TrialMetrics(
        label=buffer.label,
        trial_type=buffer.trial_type,
        started_at=buffer.started_at,
        stopped_at=stopped_at,
        wheel_start=first_pose(buffer.wheel),
        wheel_end=last_pose(buffer.wheel),
        fused_start=first_pose(buffer.fused),
        fused_end=last_pose(buffer.fused),
        wheel_vx_noise=stdev([sample.vx for sample in buffer.wheel]),
        wheel_wz_noise=stdev([sample.wz for sample in buffer.wheel]),
        fused_vx_noise=stdev([sample.vx for sample in buffer.fused]),
        fused_wz_noise=stdev([sample.wz for sample in buffer.fused]),
        imu_integrated_yaw=integrate_imu_yaw(buffer.imu),
        imu_wz_bias=mean([sample.wz for sample in buffer.imu]),
        imu_wz_noise=stdev([sample.wz for sample in buffer.imu]),
        wheel_samples=len(buffer.wheel),
        fused_samples=len(buffer.fused),
        imu_samples=len(buffer.imu),
    )


def make_twist(vx: float = 0.0, wz: float = 0.0) -> Twist:
    twist = Twist()
    twist.linear.x = float(vx)
    twist.angular.z = float(wz)
    return twist


def drive_profile(trial_type: str, elapsed: float) -> Tuple[Twist, bool]:
    """Return a deliberately slow command for the active automated trial."""
    if trial_type == "stationary":
        return make_twist(), elapsed >= 20.0
    if trial_type == "forward_back":
        if elapsed < 2.0:
            return make_twist(), False
        if elapsed < 10.0:
            return make_twist(vx=0.08), False
        if elapsed < 12.0:
            return make_twist(), False
        if elapsed < 20.0:
            return make_twist(vx=-0.08), False
        return make_twist(), elapsed >= 22.0
    if trial_type == "rotate_return":
        if elapsed < 2.0:
            return make_twist(), False
        if elapsed < 8.0:
            return make_twist(wz=0.18), False
        if elapsed < 10.0:
            return make_twist(), False
        if elapsed < 16.0:
            return make_twist(wz=-0.18), False
        return make_twist(), elapsed >= 18.0
    if trial_type == "loop":
        period = 13.65
        step = elapsed % period
        lap = int(elapsed // period)
        if lap >= 4:
            return make_twist(), elapsed >= 56.6
        if step < 5.0:
            return make_twist(vx=0.08), False
        if step < 5.8:
            return make_twist(), False
        if step < period:
            return make_twist(wz=0.20), False
        return make_twist(), False
    if trial_type == "rotate_one_way":
        if elapsed < 2.0:
            return make_twist(), False
        if elapsed < 10.0:
            return make_twist(wz=0.18), False
        return make_twist(), elapsed >= 12.0
    return make_twist(), True


def fmt(value: Optional[float], width: int = 7, precision: int = 3) -> str:
    if value is None:
        return " " * (width - 3) + "n/a"
    return f"{value:{width}.{precision}f}"


def score_trial(trial: TrialMetrics, source: str) -> Optional[float]:
    distance = trial.distance_error(source)
    yaw = trial.yaw_error(source)
    imu_yaw = trial.yaw_vs_imu_error(source)
    if distance is None or yaw is None:
        return None
    if trial.trial_type == "stationary":
        return 4.0 * distance + 4.0 * yaw + noise_for_source(trial, source)
    if trial.trial_type in {"forward_back", "rotate_return", "loop"}:
        return 3.0 * distance + 4.0 * yaw + 0.5 * (imu_yaw or 0.0) + noise_for_source(trial, source)
    if trial.trial_type == "rotate_one_way":
        if imu_yaw is None:
            return yaw + noise_for_source(trial, source)
        return imu_yaw + 0.5 * noise_for_source(trial, source)
    return distance + yaw + noise_for_source(trial, source)


def noise_for_source(trial: TrialMetrics, source: str) -> float:
    if source == "wheel":
        return trial.wheel_vx_noise + trial.wheel_wz_noise
    return trial.fused_vx_noise + trial.fused_wz_noise


def source_summary(trials: Sequence[TrialMetrics], source: str) -> Tuple[Optional[float], str]:
    scores = [score for trial in trials if (score := score_trial(trial, source)) is not None]
    if not scores:
        return None, "not enough samples"
    return statistics.fmean(scores), "lower is better"


def draw_line(stdscr: curses.window, row: int, text: str) -> int:
    height, width = stdscr.getmaxyx()
    if row < height:
        stdscr.addnstr(row, 0, text.ljust(width), width - 1)
    return row + 1


def draw_dashboard(
    stdscr: curses.window,
    collector: MetricCollector,
    status: str,
    wheel_topic: str,
    fused_topic: str,
    imu_topic: str,
    drive_mode: bool,
    cmd_topic: str,
) -> None:
    stdscr.erase()
    stats, _last, active, trials = collector.snapshot()
    row = 0
    row = draw_line(stdscr, row, "Bunker odom / IMU / fused odom metrics TUI")
    row = draw_line(stdscr, row, "Passive diagnostic only: no cmd_vel, no EKF reset, no Nav2 changes.")
    row = draw_line(stdscr, row, "")
    row = draw_line(stdscr, row, f"Topics: wheel={wheel_topic}  fused={fused_topic}  imu={imu_topic}")
    row = draw_line(
        stdscr,
        row,
        f"Drive: {'ENABLED slow automated trials' if drive_mode else 'passive observe-only'}"
        + (f"  cmd={cmd_topic}" if drive_mode else ""),
    )
    row = draw_line(stdscr, row, "")
    row = draw_line(stdscr, row, "Health:")
    row = draw_line(stdscr, row, "  source      hz      age(s)     messages")
    for name in ("wheel", "fused", "imu"):
        stat = stats[name]
        age = stat.age()
        row = draw_line(
            stdscr,
            row,
            f"  {name:<7} {stat.hz():7.1f}   {fmt(age, 7, 2)}   {stat.count:9d}",
        )
    row = draw_line(stdscr, row, "")
    if active is None:
        row = draw_line(stdscr, row, "Trial: idle")
    else:
        row = draw_line(
            stdscr,
            row,
            (
                f"Trial: {active.label} running {now_seconds() - active.started_at:.1f}s  "
                f"samples wheel={len(active.wheel)} fused={len(active.fused)} imu={len(active.imu)}"
            ),
        )
    row = draw_line(stdscr, row, "")
    row = draw_line(
        stdscr,
        row,
        "Keys: s stationary  f forward/back  r rotate return  l loop  "
        "o one-way rotate  SPACE stop  q quit",
    )
    row = draw_line(stdscr, row, "")
    row = draw_line(stdscr, row, f"Status: {status}")
    row = draw_line(stdscr, row, "")
    row = draw_line(stdscr, row, "Completed trials:")
    row = draw_line(stdscr, row, "  # type           sec  wheel_d  wheel_yaw  fused_d  fused_yaw  imu_yaw")
    for index, trial in enumerate(trials[-10:], start=max(len(trials) - 9, 1)):
        row = draw_line(
            stdscr,
            row,
            (
                f"  {index:>1} {trial.trial_type:<13} {trial.duration:4.0f} "
                f"{fmt(trial.distance_error('wheel'))} {fmt(trial.yaw_error('wheel'))} "
                f"{fmt(trial.distance_error('fused'))} {fmt(trial.yaw_error('fused'))} "
                f"{fmt(abs(trial.imu_integrated_yaw))}"
            ),
        )
    row = draw_line(stdscr, row, "")
    wheel_score, wheel_note = source_summary(trials, "wheel")
    fused_score, fused_note = source_summary(trials, "fused")
    row = draw_line(stdscr, row, "Current ranking:")
    row = draw_line(stdscr, row, f"  wheel raw: {fmt(wheel_score, 8, 4)}  {wheel_note}")
    row = draw_line(stdscr, row, f"  fused odom: {fmt(fused_score, 8, 4)}  {fused_note}")
    if wheel_score is not None and fused_score is not None:
        winner = "fused /odom" if fused_score < wheel_score else "raw /wheel/odom"
        row = draw_line(stdscr, row, f"  lower aggregate score right now: {winner}")
    stdscr.refresh()


def trial_name_for_key(key: str) -> Optional[Tuple[str, str]]:
    names = {
        "s": ("stationary", "Keep robot still. Press SPACE when done."),
        "f": ("forward_back", "Drive forward, then back to the same mark. Press SPACE when done."),
        "r": ("rotate_return", "Rotate left/right and return to original heading. Press SPACE when done."),
        "l": ("loop", "Drive a rectangle and stop at the start mark. Press SPACE when done."),
        "o": ("rotate_one_way", "Rotate one direction, e.g. about 90 or 360 deg. Press SPACE when done."),
    }
    return names.get(key)


def run_tui(stdscr: curses.window, collector: MetricCollector, args: argparse.Namespace) -> None:
    curses.curs_set(0)
    stdscr.nodelay(True)
    status = "start Terminal 1 hardware first, then run trials"
    trial_counter = 0
    while rclpy.ok():
        auto_status = collector.publish_drive_command()
        if auto_status is not None:
            status = auto_status
        draw_dashboard(
            stdscr,
            collector,
            status,
            args.wheel_topic,
            args.fused_topic,
            args.imu_topic,
            args.drive_mode,
            args.cmd_topic,
        )
        try:
            key_code = stdscr.getch()
        except curses.error:
            key_code = -1
        if key_code == -1:
            time.sleep(0.2)
            continue
        key = chr(key_code).lower() if 0 <= key_code < 256 else ""
        if key == "q":
            collector.publish_zero()
            break
        if key == " ":
            metrics = collector.stop_trial()
            if metrics is None:
                status = "no active trial to stop"
            else:
                status = (
                    f"saved {metrics.trial_type}: "
                    f"wheel return={fmt(metrics.distance_error('wheel')).strip()}m/"
                    f"{fmt(metrics.yaw_error('wheel')).strip()}rad, "
                    f"fused return={fmt(metrics.distance_error('fused')).strip()}m/"
                    f"{fmt(metrics.yaw_error('fused')).strip()}rad"
                )
            continue
        selected = trial_name_for_key(key)
        if selected is None:
            continue
        trial_type, instruction = selected
        trial_counter += 1
        if collector.start_trial(f"{trial_counter:02d}-{trial_type}", trial_type):
            status = instruction
        else:
            status = "a trial is already running; press SPACE to stop it first"


def print_report(collector: MetricCollector) -> None:
    _stats, _last, _active, trials = collector.snapshot()
    print("\nOdometry / IMU / fused odometry metric report")
    print("Lower scores are better. These are no-ground-truth consistency metrics.")
    if not trials:
        print("No trials completed.")
        return
    print("\nTrials:")
    print(
        "type,duration_s,wheel_return_m,wheel_yaw_rad,fused_return_m,"
        "fused_yaw_rad,imu_yaw_rad,wheel_samples,fused_samples,imu_samples"
    )
    for trial in trials:
        print(
            ",".join(
                [
                    trial.trial_type,
                    f"{trial.duration:.3f}",
                    f"{trial.distance_error('wheel') or 0.0:.6f}",
                    f"{trial.yaw_error('wheel') or 0.0:.6f}",
                    f"{trial.distance_error('fused') or 0.0:.6f}",
                    f"{trial.yaw_error('fused') or 0.0:.6f}",
                    f"{trial.imu_integrated_yaw:.6f}",
                    str(trial.wheel_samples),
                    str(trial.fused_samples),
                    str(trial.imu_samples),
                ]
            )
        )
    wheel_score, _ = source_summary(trials, "wheel")
    fused_score, _ = source_summary(trials, "fused")
    print("\nAggregate:")
    print("rank,source,score,note")
    ranked = [
        ("raw /wheel/odom", wheel_score),
        ("fused /odom", fused_score),
    ]
    valid = sorted(
        [(source, score) for source, score in ranked if score is not None],
        key=lambda item: item[1],
    )
    for index, (source, score) in enumerate(valid, start=1):
        print(f"{index},{source},{score:.6f},lower is better")
    for source, score in ranked:
        if score is None:
            print(f"n/a,{source},n/a,not enough samples")
    if wheel_score is not None and fused_score is not None:
        print("Current better consistency source:", "fused /odom" if fused_score < wheel_score else "raw /wheel/odom")


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Interactive no-ground-truth odom/IMU/fused odom metric TUI."
    )
    parser.add_argument("--wheel-topic", default="/wheel/odom")
    parser.add_argument("--fused-topic", default="/odom")
    parser.add_argument("--imu-topic", default="/yesense/imu_data_ros")
    parser.add_argument(
        "--drive-mode",
        action="store_true",
        help="Publish slow automated trial commands. Default is passive observe-only.",
    )
    parser.add_argument(
        "--cmd-topic",
        default="/cmd_vel_debug",
        help="Command topic used only with --drive-mode. Use your final gated drive topic intentionally.",
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="Run synthetic trials without ROS topics, curses, hardware, or robot motion.",
    )
    raw_args = list(sys.argv[1:] if argv is None else argv)
    return parser.parse_args(remove_ros_args(args=raw_args))


def synthetic_odom(
    start: float,
    duration: float,
    end_x: float,
    end_y: float,
    end_yaw: float,
    samples: int = 60,
) -> List[OdomSample]:
    output = []
    previous_x = 0.0
    previous_y = 0.0
    previous_yaw = 0.0
    previous_stamp = start
    for index in range(samples):
        fraction = index / max(samples - 1, 1)
        stamp = start + duration * fraction
        x = end_x * fraction
        y = end_y * fraction
        yaw = end_yaw * fraction
        dt = max(stamp - previous_stamp, 1e-6)
        vx = math.hypot(x - previous_x, y - previous_y) / dt if index else 0.0
        wz = wrap_angle(yaw - previous_yaw) / dt if index else 0.0
        output.append(OdomSample(stamp, x, y, yaw, vx, wz))
        previous_x = x
        previous_y = y
        previous_yaw = yaw
        previous_stamp = stamp
    return output


def synthetic_imu(
    start: float,
    duration: float,
    yaw_delta: float,
    samples: int = 120,
) -> List[ImuSample]:
    if samples < 2:
        return []
    wz = yaw_delta / max(duration, 1e-6)
    return [
        ImuSample(start + duration * index / (samples - 1), wz)
        for index in range(samples)
    ]


def self_test_report() -> int:
    start = now_seconds()
    specs = [
        ("stationary", 60.0, (0.030, 0.000, 0.020), (0.010, 0.000, 0.006), 0.0),
        ("forward_back", 35.0, (0.180, 0.050, 0.070), (0.060, 0.015, 0.020), 0.0),
        ("rotate_return", 25.0, (0.040, 0.030, 0.160), (0.020, 0.010, 0.045), 0.0),
        ("loop", 55.0, (0.300, -0.120, 0.200), (0.090, -0.030, 0.055), 0.0),
        ("rotate_one_way", 20.0, (0.020, 0.000, 1.420), (0.010, 0.000, 1.555), 1.570),
    ]
    trials = []
    for index, (trial_type, duration, wheel_end, fused_end, imu_yaw) in enumerate(specs):
        trial_start = start + index * 100.0
        buffer = TrialBuffer(
            label=f"self-{index + 1:02d}-{trial_type}",
            trial_type=trial_type,
            started_at=trial_start,
            wheel=synthetic_odom(trial_start, duration, *wheel_end),
            fused=synthetic_odom(trial_start, duration, *fused_end),
            imu=synthetic_imu(trial_start, duration, imu_yaw),
        )
        trials.append(build_metrics(buffer, trial_start + duration))

    print("Synthetic self-test report")
    print("This validates scoring/reporting only. It is not a robot measurement.")
    print("type,wheel_return_m,wheel_yaw_rad,fused_return_m,fused_yaw_rad,imu_yaw_rad")
    for trial in trials:
        print(
            ",".join(
                [
                    trial.trial_type,
                    f"{trial.distance_error('wheel') or 0.0:.6f}",
                    f"{trial.yaw_error('wheel') or 0.0:.6f}",
                    f"{trial.distance_error('fused') or 0.0:.6f}",
                    f"{trial.yaw_error('fused') or 0.0:.6f}",
                    f"{trial.imu_integrated_yaw:.6f}",
                ]
            )
        )
    wheel_score, _ = source_summary(trials, "wheel")
    fused_score, _ = source_summary(trials, "fused")
    print(f"raw /wheel/odom score: {wheel_score:.6f}")
    print(f"fused /odom score: {fused_score:.6f}")
    print("expected lower aggregate score: fused /odom")
    if fused_score is None or wheel_score is None or fused_score >= wheel_score:
        print("self-test failed: expected fused /odom to score lower")
        return 1
    print("self-test passed")
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    if args.self_test:
        return self_test_report()
    rclpy.init(args=argv)
    collector = MetricCollector(
        args.wheel_topic,
        args.fused_topic,
        args.imu_topic,
        args.cmd_topic,
        args.drive_mode,
    )
    spin_thread = threading.Thread(target=rclpy.spin, args=(collector,), daemon=True)
    spin_thread.start()
    try:
        curses.wrapper(run_tui, collector, args)
    finally:
        collector.stop_trial()
        collector.publish_zero()
        collector.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
    print_report(collector)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
