#!/usr/bin/env python3
# Copyright 2026 Dase Orin
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd
"""Assert exclusive ownership of the two SLAM localization TF edges."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import ctypes
from dataclasses import dataclass
import sys
import time
from typing import DefaultDict, Dict, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.utilities import remove_ros_args
from rclpy.type_support import check_for_type_support
from tf2_msgs.msg import TFMessage


GidKey = Tuple[str, bytes]
FramePair = Tuple[str, str]


class RmwGid(ctypes.Structure):
    """ctypes mirror of Humble's rmw_gid_t."""

    _fields_ = [
        ("implementation_identifier", ctypes.c_char_p),
        ("data", ctypes.c_uint8 * 24),
    ]


class RmwMessageInfo(ctypes.Structure):
    """ctypes mirror of Humble's rmw_message_info_t, including publisher_gid."""

    _fields_ = [
        ("source_timestamp", ctypes.c_int64),
        ("received_timestamp", ctypes.c_int64),
        ("publication_sequence_number", ctypes.c_uint64),
        ("reception_sequence_number", ctypes.c_uint64),
        ("publisher_gid", RmwGid),
        ("from_intra_process", ctypes.c_bool),
    ]


class MessageInfoTaker:
    """
    Take a Python ROS message together with the full rmw MessageInfo.

    ROS 2 Humble's rclpy 0.18 converts MessageInfo to a dict containing only
    timestamps and drops ``publisher_gid``. Calling the public Humble C API is
    therefore necessary to perform the requested per-message ownership check.
    """

    RCL_RET_OK = 0
    RCL_RET_SUBSCRIPTION_TAKE_FAILED = 401

    def __init__(self, message_type: object) -> None:
        check_for_type_support(message_type)
        message_metaclass = type(message_type)

        get_pointer = ctypes.pythonapi.PyCapsule_GetPointer
        get_pointer.argtypes = [ctypes.py_object, ctypes.c_char_p]
        get_pointer.restype = ctypes.c_void_p

        self._create = ctypes.PYFUNCTYPE(ctypes.c_void_p)(
            get_pointer(message_metaclass._CREATE_ROS_MESSAGE, None)
        )
        self._convert = ctypes.PYFUNCTYPE(ctypes.py_object, ctypes.c_void_p)(
            get_pointer(message_metaclass._CONVERT_TO_PY, None)
        )
        self._destroy = ctypes.PYFUNCTYPE(None, ctypes.c_void_p)(
            get_pointer(message_metaclass._DESTROY_ROS_MESSAGE, None)
        )

        rcl_library = ctypes.CDLL("librcl.so")
        self._rcl_take = rcl_library.rcl_take
        self._rcl_take.argtypes = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.POINTER(RmwMessageInfo),
            ctypes.c_void_p,
        ]
        self._rcl_take.restype = ctypes.c_int
        self._message_type = message_type

    def take(self, subscription: object) -> Optional[Tuple[object, RmwMessageInfo]]:
        ros_message = self._create()
        if not ros_message:
            raise RuntimeError("failed to allocate a ROS message")
        message_info = RmwMessageInfo()
        try:
            with subscription.handle:  # type: ignore[attr-defined]
                result = self._rcl_take(
                    ctypes.c_void_p(subscription.handle.pointer),  # type: ignore[attr-defined]
                    ros_message,
                    ctypes.byref(message_info),
                    None,
                )
            if result == self.RCL_RET_SUBSCRIPTION_TAKE_FAILED:
                return None
            if result != self.RCL_RET_OK:
                raise RuntimeError(f"rcl_take failed with return code {result}")
            return self._convert(ros_message), message_info
        finally:
            self._destroy(ros_message)


def _normalise_frame(frame: str) -> str:
    return frame.strip().lstrip("/")


def _gid_bytes(value: object) -> bytes:
    try:
        return bytes(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return b""


@dataclass(frozen=True)
class EndpointOwner:
    node_name: str
    node_namespace: str

    @property
    def fully_qualified_name(self) -> str:
        namespace = self.node_namespace.rstrip("/")
        if not namespace:
            return f"/{self.node_name}"
        return f"{namespace}/{self.node_name}"


class TfOwnershipObserver(Node):
    def __init__(self, map_frame: str, odom_frame: str, base_frame: str) -> None:
        super().__init__("check_tf_ownership")
        self.map_to_odom = (
            _normalise_frame(map_frame),
            _normalise_frame(odom_frame),
        )
        self.odom_to_base = (
            _normalise_frame(odom_frame),
            _normalise_frame(base_frame),
        )
        self.observations: DefaultDict[FramePair, Counter[GidKey]] = defaultdict(
            Counter
        )
        self.endpoint_owners: Dict[GidKey, EndpointOwner] = {}

        dynamic_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.dynamic_subscription = self.create_subscription(
            TFMessage, "/tf", lambda _message: None, dynamic_qos
        )
        self.static_subscription = self.create_subscription(
            TFMessage, "/tf_static", lambda _message: None, static_qos
        )

    def record(
        self, topic: str, message: TFMessage, message_info: RmwMessageInfo
    ) -> None:
        publisher_gid = bytes(message_info.publisher_gid.data)
        key = (topic, publisher_gid)
        for transform in message.transforms:
            pair = (
                _normalise_frame(transform.header.frame_id),
                _normalise_frame(transform.child_frame_id),
            )
            if pair in {self.map_to_odom, self.odom_to_base}:
                self.observations[pair][key] += 1

    def refresh_endpoint_owners(self, topic: Optional[str] = None) -> None:
        topics = (topic,) if topic is not None else ("/tf", "/tf_static")
        for topic_name in topics:
            for endpoint in self.get_publishers_info_by_topic(topic_name):
                gid = _gid_bytes(endpoint.endpoint_gid)
                self.endpoint_owners[(topic_name, gid)] = EndpointOwner(
                    node_name=endpoint.node_name,
                    node_namespace=endpoint.node_namespace,
                )


def _owner_matches(owner: EndpointOwner, expected: str) -> bool:
    if expected.startswith("/"):
        normalised = "/" + expected.strip("/")
        return owner.fully_qualified_name == normalised
    return owner.node_name == expected


def _format_gid(gid: bytes) -> str:
    return gid.hex() if gid else "<unavailable>"


def evaluate_pair(
    pair: FramePair,
    expected_owner: str,
    observations: Counter[GidKey],
    owners: Dict[GidKey, EndpointOwner],
) -> bool:
    label = f"{pair[0]} -> {pair[1]}"
    if not observations:
        print(f"FAIL: did not observe {label} on /tf or /tf_static")
        return False

    passed = True
    if len(observations) != 1:
        print(
            f"FAIL: {label} came from {len(observations)} publisher GIDs; "
            "exactly one is required"
        )
        passed = False

    for (topic, gid), message_count in sorted(
        observations.items(), key=lambda item: (item[0][0], item[0][1])
    ):
        owner = owners.get((topic, gid))
        if owner is None:
            print(
                f"FAIL: {label} publisher on {topic} has unresolved GID "
                f"{_format_gid(gid)} ({message_count} messages)"
            )
            passed = False
            continue
        owner_label = owner.fully_qualified_name
        print(
            f"OBSERVED: {label} on {topic}, owner={owner_label}, "
            f"gid={_format_gid(gid)}, messages={message_count}"
        )
        if not _owner_matches(owner, expected_owner):
            print(
                f"FAIL: {label} owner is {owner_label}; expected "
                f"{expected_owner!r}"
            )
            passed = False

    if passed:
        print(f"PASS: {label} is exclusively owned by {expected_owner!r}")
    return passed


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Observe /tf and /tf_static, map publisher GIDs to graph endpoint "
            "owners, and assert exclusive SLAM TF ownership."
        )
    )
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--map-odom-owner", default="rtabmap")
    parser.add_argument("--odom-base-owner", default="ekf_filter_node")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    raw_argv = list(sys.argv if argv is None else argv)
    args = _parser().parse_args(remove_ros_args(args=raw_argv)[1:])
    if args.duration <= 0.0:
        _parser().error("--duration must be greater than zero")

    rclpy.init(args=raw_argv)
    node = TfOwnershipObserver(
        args.map_frame, args.odom_frame, args.base_frame
    )
    taker = MessageInfoTaker(TFMessage)
    subscriptions = (
        ("/tf", node.dynamic_subscription),
        ("/tf_static", node.static_subscription),
    )
    try:
        deadline = time.monotonic() + args.duration
        next_graph_refresh = 0.0
        while rclpy.ok():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            took_message = False
            for topic, subscription in subscriptions:
                # Bound each drain pass so a saturated /tf cannot starve
                # /tf_static, graph refreshes, or the observation deadline.
                for _ in range(1000):
                    taken = taker.take(subscription)
                    if taken is None:
                        break
                    message, message_info = taken
                    node.record(topic, message, message_info)
                    took_message = True

            now = time.monotonic()
            if now >= next_graph_refresh:
                node.refresh_endpoint_owners()
                next_graph_refresh = now + 0.25
            if not took_message:
                time.sleep(min(0.005, max(0.0, remaining)))

        node.refresh_endpoint_owners()
        map_ok = evaluate_pair(
            node.map_to_odom,
            args.map_odom_owner,
            node.observations[node.map_to_odom],
            node.endpoint_owners,
        )
        odom_ok = evaluate_pair(
            node.odom_to_base,
            args.odom_base_owner,
            node.observations[node.odom_to_base],
            node.endpoint_owners,
        )
        return 0 if map_ok and odom_ok else 1
    except KeyboardInterrupt:
        print("TF ownership check interrupted", file=sys.stderr)
        return 130
    except (OSError, RuntimeError) as error:
        print(f"TF ownership check failed: {error}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
