#!/usr/bin/env python3

import argparse
import json
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener


def matrix_from_values(position, quaternion):
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    matrix[:3, 3] = np.asarray(position, dtype=float)
    return matrix


def matrix_from_transform(transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return matrix_from_values(
        [translation.x, translation.y, translation.z],
        [rotation.x, rotation.y, rotation.z, rotation.w],
    )


class HandEyeTFPublisher(Node):
    def __init__(self, calibration_file, parent_frame, camera_root, optical_frame):
        super().__init__("piper_x_handeye_tf_publisher")
        self.parent_frame = parent_frame
        self.camera_root = camera_root
        self.optical_frame = optical_frame

        with open(calibration_file, "r", encoding="utf-8") as file:
            calibration = json.load(file)

        self.t_parent_optical = matrix_from_values(
            calibration["position"],
            calibration["orientation"],
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.published = False
        self.wait_message_printed = False
        self.timer = self.create_timer(0.5, self.try_publish)
        self.get_logger().info(f"Loaded calibration: {calibration_file}")
        self.get_logger().info(f"Will publish {parent_frame} -> {camera_root}")

    def try_publish(self):
        if self.published:
            return
        try:
            camera_to_optical_msg = self.tf_buffer.lookup_transform(
                self.camera_root,
                self.optical_frame,
                Time(),
                timeout=Duration(seconds=0.25),
            )
        except TransformException:
            if not self.wait_message_printed:
                self.get_logger().info(
                    f"Waiting for RealSense TF {self.camera_root} -> {self.optical_frame}..."
                )
                self.wait_message_printed = True
            return

        t_camera_optical = matrix_from_transform(camera_to_optical_msg)
        t_parent_camera = self.t_parent_optical @ np.linalg.inv(t_camera_optical)
        translation = t_parent_camera[:3, 3]
        quaternion = Rotation.from_matrix(t_parent_camera[:3, :3]).as_quat()

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.camera_root
        transform.transform.translation.x = float(translation[0])
        transform.transform.translation.y = float(translation[1])
        transform.transform.translation.z = float(translation[2])
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])

        self.static_broadcaster.sendTransform(transform)
        self.published = True
        self.timer.cancel()
        self.get_logger().info(f"Published calibrated TF: {self.parent_frame} -> {self.camera_root}")
        self.get_logger().info(
            "Translation [m]: "
            f"x={translation[0]:+.6f}, y={translation[1]:+.6f}, z={translation[2]:+.6f}"
        )
        self.get_logger().info("Keep this node running during robot operation.")


def main():
    parser = argparse.ArgumentParser(description="Publish PiPER-X hand-eye calibration as ROS TF.")
    parser.add_argument("--calibration", required=True)
    parser.add_argument("--parent-frame", default="front_piper_flange_link")
    parser.add_argument("--camera-root", default="front_camera_link")
    parser.add_argument("--optical-frame", default="front_camera_color_optical_frame")
    args, _ros_args = parser.parse_known_args()

    calibration_file = Path(args.calibration).expanduser()
    if not calibration_file.is_file():
        raise FileNotFoundError(f"Calibration file does not exist: {calibration_file}")

    rclpy.init()
    node = HandEyeTFPublisher(
        str(calibration_file),
        args.parent_frame,
        args.camera_root,
        args.optical_frame,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
