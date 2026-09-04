#!/usr/bin/env python3
# Copyright 2026 Dase Orin
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd
"""Save one nav_msgs/OccupancyGrid as an atomic PGM/YAML map pair."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
from typing import Optional, Sequence

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _normalise_prefix(value: str) -> Path:
    path = Path(value).expanduser()
    if path.suffix.lower() in {".pgm", ".yaml", ".yml"}:
        path = path.with_suffix("")
    return path.absolute()


def _pgm_bytes(
    grid: OccupancyGrid,
    occupied_thresh: float,
    free_thresh: float,
) -> bytes:
    width = int(grid.info.width)
    height = int(grid.info.height)
    expected_cells = width * height
    if width <= 0 or height <= 0:
        raise ValueError(f"invalid map dimensions {width}x{height}")
    if len(grid.data) != expected_cells:
        raise ValueError(
            f"map data has {len(grid.data)} cells; expected {expected_cells}"
        )

    pixels = bytearray(expected_cells)
    occupied_limit = occupied_thresh * 100.0
    free_limit = free_thresh * 100.0

    # OccupancyGrid row zero starts at the map origin (bottom-left), while PGM
    # row zero is displayed at the top. Reverse the row order on disk.
    output_index = 0
    for y in range(height - 1, -1, -1):
        row_start = y * width
        for x in range(width):
            occupancy = int(grid.data[row_start + x])
            if occupancy < 0:
                pixel = 205
            elif occupancy >= occupied_limit:
                pixel = 0
            elif occupancy <= free_limit:
                pixel = 254
            else:
                pixel = 205
            pixels[output_index] = pixel
            output_index += 1

    header = (
        "P5\n"
        "# CREATOR: bunker_slam_bringup save_occupancy_grid.py\n"
        f"{width} {height}\n"
        "255\n"
    ).encode("ascii")
    return header + bytes(pixels)


def _yaml_bytes(
    grid: OccupancyGrid,
    pgm_name: str,
    occupied_thresh: float,
    free_thresh: float,
) -> bytes:
    origin = grid.info.origin
    yaw = _yaw_from_quaternion(
        origin.orientation.x,
        origin.orientation.y,
        origin.orientation.z,
        origin.orientation.w,
    )
    # JSON string quoting is a valid subset of YAML and safely handles spaces.
    image_value = json.dumps(pgm_name)
    content = (
        f"image: {image_value}\n"
        "mode: trinary\n"
        f"resolution: {float(grid.info.resolution):.15g}\n"
        "origin: "
        f"[{float(origin.position.x):.15g}, "
        f"{float(origin.position.y):.15g}, {yaw:.15g}]\n"
        "negate: 0\n"
        f"occupied_thresh: {occupied_thresh:.15g}\n"
        f"free_thresh: {free_thresh:.15g}\n"
    )
    return content.encode("utf-8")


def _prepare_atomic_file(path: Path, payload: bytes) -> Path:
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary_path, 0o644)
    except BaseException:
        temporary_path.unlink(missing_ok=True)
        raise
    return temporary_path


def _fsync_directory(directory: Path) -> None:
    flags = os.O_RDONLY
    if hasattr(os, "O_DIRECTORY"):
        flags |= os.O_DIRECTORY
    descriptor = os.open(directory, flags)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def save_map_pair(
    grid: OccupancyGrid,
    prefix: Path,
    occupied_thresh: float,
    free_thresh: float,
) -> tuple[Path, Path]:
    """Write complete sibling files, then atomically replace both destinations."""
    prefix.parent.mkdir(parents=True, exist_ok=True)
    pgm_path = prefix.with_suffix(".pgm")
    yaml_path = prefix.with_suffix(".yaml")

    pgm_temporary: Optional[Path] = None
    yaml_temporary: Optional[Path] = None
    try:
        pgm_temporary = _prepare_atomic_file(
            pgm_path, _pgm_bytes(grid, occupied_thresh, free_thresh)
        )
        yaml_temporary = _prepare_atomic_file(
            yaml_path,
            _yaml_bytes(grid, pgm_path.name, occupied_thresh, free_thresh),
        )

        # Publish the image first and YAML last. A reader treating YAML as the
        # commit marker can never observe a YAML file that names a missing PGM.
        os.replace(pgm_temporary, pgm_path)
        pgm_temporary = None
        os.replace(yaml_temporary, yaml_path)
        yaml_temporary = None
        _fsync_directory(prefix.parent)
    finally:
        if pgm_temporary is not None:
            pgm_temporary.unlink(missing_ok=True)
        if yaml_temporary is not None:
            yaml_temporary.unlink(missing_ok=True)

    return pgm_path, yaml_path


class MapReceiver(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("save_occupancy_grid")
        self.grid: Optional[OccupancyGrid] = None
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.subscription = self.create_subscription(
            OccupancyGrid, topic, self._on_map, qos
        )

    def _on_map(self, message: OccupancyGrid) -> None:
        if self.grid is None:
            self.grid = message


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Save one /map OccupancyGrid as PREFIX.pgm and PREFIX.yaml."
    )
    parser.add_argument(
        "output_prefix",
        help="Destination prefix; a trailing .pgm/.yaml suffix is removed.",
    )
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--occupied-thresh", type=float, default=0.65)
    parser.add_argument("--free-thresh", type=float, default=0.196)
    return parser


def _parse_args(argv: Sequence[str]) -> argparse.Namespace:
    return _parser().parse_args(remove_ros_args(args=list(argv))[1:])


def main(argv: Optional[Sequence[str]] = None) -> int:
    raw_argv = list(sys.argv if argv is None else argv)
    args = _parse_args(raw_argv)
    if args.timeout <= 0.0:
        _parser().error("--timeout must be greater than zero")
    if not 0.0 <= args.free_thresh < args.occupied_thresh <= 1.0:
        _parser().error(
            "thresholds must satisfy 0 <= free < occupied <= 1"
        )

    rclpy.init(args=raw_argv)
    node = MapReceiver(args.map_topic)
    try:
        deadline = time.monotonic() + args.timeout
        while rclpy.ok() and node.grid is None:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                node.get_logger().error(
                    f"timed out after {args.timeout:.3g}s waiting for "
                    f"{node.resolve_topic_name(args.map_topic)}"
                )
                return 1
            rclpy.spin_once(node, timeout_sec=min(0.2, remaining))

        if node.grid is None:
            node.get_logger().error("ROS shut down before a map was received")
            return 1

        prefix = _normalise_prefix(args.output_prefix)
        pgm_path, yaml_path = save_map_pair(
            node.grid, prefix, args.occupied_thresh, args.free_thresh
        )
        node.get_logger().info(
            f"saved {node.grid.info.width}x{node.grid.info.height} map to "
            f"{pgm_path} and {yaml_path}"
        )
        return 0
    except (OSError, ValueError) as error:
        node.get_logger().error(f"map save failed: {error}")
        return 1
    except KeyboardInterrupt:
        node.get_logger().warning("map save interrupted")
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
