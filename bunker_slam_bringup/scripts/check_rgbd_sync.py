#!/usr/bin/env python3
# Copyright 2026 Dase Orin
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd
"""Verify live RGB, aligned-depth and CameraInfo synchronization plus TF."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


StreamMessage = Tuple[int, int, object]


def _stamp_ns(message: object) -> int:
    stamp = message.header.stamp  # type: ignore[attr-defined]
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _frame_id(message: object) -> str:
    return str(message.header.frame_id).strip().lstrip("/")  # type: ignore[attr-defined]


@dataclass
class PendingMatch:
    messages: Dict[str, object]
    deadline: float


class RgbdSyncChecker(Node):
    def __init__(
        self,
        rgb_topic: str,
        depth_topic: str,
        camera_info_topic: str,
        base_frame: str,
        slop_ns: int,
        tf_timeout: float,
        queue_size: int,
    ) -> None:
        super().__init__("check_rgbd_sync")
        self.base_frame = base_frame.strip().lstrip("/")
        self.slop_ns = slop_ns
        self.tf_timeout = tf_timeout
        self.queue_size = queue_size
        self.sequence = 0
        self.queues: Dict[str, List[StreamMessage]] = {
            "rgb": [],
            "depth": [],
            "camera_info": [],
        }
        self.received: Counter[str] = Counter()
        self.dropped: Counter[str] = Counter()
        self.pending: List[PendingMatch] = []
        self.success_count = 0
        self.failures: List[str] = []
        self.rgb_depth_offsets_ns: List[int] = []
        self.rgb_info_offsets_ns: List[int] = []
        self.encodings: Counter[Tuple[str, str]] = Counter()
        self.frames: Counter[Tuple[str, str]] = Counter()

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=min(queue_size, 100),
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.rgb_subscription = self.create_subscription(
            Image,
            rgb_topic,
            lambda message: self._receive("rgb", message),
            sensor_qos,
        )
        self.depth_subscription = self.create_subscription(
            Image,
            depth_topic,
            lambda message: self._receive("depth", message),
            sensor_qos,
        )
        self.info_subscription = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            lambda message: self._receive("camera_info", message),
            sensor_qos,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0), node=self)
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=False
        )

    def _receive(self, stream: str, message: object) -> None:
        stamp_ns = _stamp_ns(message)
        self.sequence += 1
        queue = self.queues[stream]
        queue.append((stamp_ns, self.sequence, message))
        queue.sort(key=lambda item: (item[0], item[1]))
        self.received[stream] += 1
        self.frames[(stream, _frame_id(message))] += 1
        if stream in {"rgb", "depth"}:
            self.encodings[(stream, str(message.encoding))] += 1  # type: ignore[attr-defined]
        while len(queue) > self.queue_size:
            queue.pop(0)
            self.dropped[stream] += 1
        self._match_available()

    def _match_available(self) -> None:
        while all(self.queues.values()):
            heads = {
                stream: queue[0] for stream, queue in self.queues.items()
            }
            stamps = {stream: item[0] for stream, item in heads.items()}
            oldest_stream = min(stamps, key=stamps.get)  # type: ignore[arg-type]
            if max(stamps.values()) - min(stamps.values()) > self.slop_ns:
                self.queues[oldest_stream].pop(0)
                self.dropped[oldest_stream] += 1
                continue

            messages = {
                stream: self.queues[stream].pop(0)[2]
                for stream in self.queues
            }
            self.pending.append(
                PendingMatch(
                    messages=messages,
                    deadline=time.monotonic() + self.tf_timeout,
                )
            )

    def _validate_messages(self, messages: Dict[str, object]) -> Optional[str]:
        rgb: Image = messages["rgb"]  # type: ignore[assignment]
        depth: Image = messages["depth"]  # type: ignore[assignment]
        info: CameraInfo = messages["camera_info"]  # type: ignore[assignment]

        stamps = {name: _stamp_ns(message) for name, message in messages.items()}
        if any(stamp <= 0 for stamp in stamps.values()):
            return f"zero/invalid header stamp in matched set: {stamps}"
        skew = max(stamps.values()) - min(stamps.values())
        if skew > self.slop_ns:
            return f"matched timestamp skew {skew / 1e6:.3f} ms exceeds limit"
        if rgb.width <= 0 or rgb.height <= 0 or depth.width <= 0 or depth.height <= 0:
            return "RGB or depth image has zero dimensions"
        if (rgb.width, rgb.height) != (depth.width, depth.height):
            return (
                f"RGB dimensions {rgb.width}x{rgb.height} differ from aligned "
                f"depth {depth.width}x{depth.height}"
            )
        if (rgb.width, rgb.height) != (info.width, info.height):
            return (
                f"RGB dimensions {rgb.width}x{rgb.height} differ from "
                f"CameraInfo {info.width}x{info.height}"
            )
        frames = {name: _frame_id(message) for name, message in messages.items()}
        if any(not frame for frame in frames.values()):
            return f"empty frame_id in matched set: {frames}"
        if len(set(frames.values())) != 1:
            return (
                "RGB, aligned depth, and CameraInfo frame IDs differ: "
                f"{frames}"
            )
        return None

    def process_pending(self) -> None:
        now = time.monotonic()
        still_pending: List[PendingMatch] = []
        for pending in self.pending:
            validation_error = self._validate_messages(pending.messages)
            if validation_error is not None:
                self.failures.append(validation_error)
                continue

            missing_tf: List[str] = []
            for stream, message in pending.messages.items():
                frame = _frame_id(message)
                stamp = Time.from_msg(message.header.stamp)  # type: ignore[attr-defined]
                if not self.tf_buffer.can_transform(
                    self.base_frame, frame, stamp, timeout=Duration()
                ):
                    missing_tf.append(
                        f"{self.base_frame} <- {frame} at "
                        f"{_stamp_ns(message)} ({stream})"
                    )

            if missing_tf:
                if now < pending.deadline:
                    still_pending.append(pending)
                else:
                    self.failures.append(
                        "TF unavailable before timeout: " + "; ".join(missing_tf)
                    )
                continue

            rgb_stamp = _stamp_ns(pending.messages["rgb"])
            depth_stamp = _stamp_ns(pending.messages["depth"])
            info_stamp = _stamp_ns(pending.messages["camera_info"])
            self.rgb_depth_offsets_ns.append(depth_stamp - rgb_stamp)
            self.rgb_info_offsets_ns.append(info_stamp - rgb_stamp)
            self.success_count += 1

        self.pending = still_pending

    def print_summary(self) -> None:
        def _offset_range(values: List[int]) -> str:
            if not values:
                return "n/a"
            return f"[{min(values) / 1e6:.3f}, {max(values) / 1e6:.3f}] ms"

        print(
            "received: "
            + ", ".join(
                f"{name}={self.received[name]}" for name in self.queues
            )
        )
        print(
            "unmatched/dropped: "
            + ", ".join(f"{name}={self.dropped[name]}" for name in self.queues)
        )
        print(f"validated synchronized sets: {self.success_count}")
        print(
            "depth-minus-RGB stamp range: "
            f"{_offset_range(self.rgb_depth_offsets_ns)}"
        )
        print(
            "CameraInfo-minus-RGB stamp range: "
            f"{_offset_range(self.rgb_info_offsets_ns)}"
        )
        if self.frames:
            print(
                "observed frames: "
                + ", ".join(
                    f"{stream}={frame!r} ({count})"
                    for (stream, frame), count in sorted(self.frames.items())
                )
            )
        if self.encodings:
            print(
                "observed encodings: "
                + ", ".join(
                    f"{stream}={encoding!r} ({count})"
                    for (stream, encoding), count in sorted(self.encodings.items())
                )
            )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Check raw RGB, aligned depth, CameraInfo timestamp agreement and "
            "timestamped TF connectivity to base_link."
        )
    )
    parser.add_argument(
        "--rgb-topic", default="/front_camera/color/image_raw"
    )
    parser.add_argument(
        "--depth-topic",
        default="/front_camera/aligned_depth_to_color/image_raw",
    )
    parser.add_argument(
        "--camera-info-topic", default="/front_camera/color/camera_info"
    )
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--samples", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--slop-ms", type=float, default=50.0)
    # A new TF listener must first discover transient-local static transforms.
    # Two seconds avoids treating DDS discovery at checker startup as a missing
    # robot transform; each set is still validated at its exact image stamp.
    parser.add_argument("--tf-timeout", type=float, default=2.0)
    parser.add_argument("--queue-size", type=int, default=100)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    raw_argv = list(sys.argv if argv is None else argv)
    args = _parser().parse_args(remove_ros_args(args=raw_argv)[1:])
    if args.samples <= 0:
        _parser().error("--samples must be greater than zero")
    if args.timeout <= 0.0 or args.tf_timeout < 0.0:
        _parser().error("--timeout must be positive and --tf-timeout non-negative")
    if args.slop_ms < 0.0 or args.queue_size <= 0:
        _parser().error("--slop-ms must be non-negative and --queue-size positive")

    rclpy.init(args=raw_argv)
    node = RgbdSyncChecker(
        args.rgb_topic,
        args.depth_topic,
        args.camera_info_topic,
        args.base_frame,
        int(args.slop_ms * 1_000_000.0),
        args.tf_timeout,
        args.queue_size,
    )
    try:
        deadline = time.monotonic() + args.timeout
        while rclpy.ok() and not node.failures:
            if node.success_count >= args.samples:
                break
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            rclpy.spin_once(node, timeout_sec=min(0.05, remaining))
            node.process_pending()

        node.process_pending()
        node.print_summary()
        if node.failures:
            for failure in node.failures:
                print(f"FAIL: {failure}")
            return 1
        if node.success_count < args.samples:
            print(
                f"FAIL: timed out after {args.timeout:.3g}s with "
                f"{node.success_count}/{args.samples} valid synchronized sets"
            )
            return 1
        print(
            f"PASS: {node.success_count} RGB-D-CameraInfo sets were within "
            f"{args.slop_ms:.3g} ms and had timestamped TF to "
            f"{node.base_frame}"
        )
        return 0
    except KeyboardInterrupt:
        print("RGB-D synchronization check interrupted", file=sys.stderr)
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
