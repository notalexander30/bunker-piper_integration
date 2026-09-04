#!/usr/bin/env python3
"""Detect ArUco markers and save them as door/home landmarks."""

import json
import math
import os
from dataclasses import dataclass, field
from typing import Dict, Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class Landmark:
    marker_id: int
    first_seen_sec: float
    last_seen_sec: float
    seen_count: int = 0
    approach_x: float = 0.0
    approach_y: float = 0.0
    approach_yaw: float = 0.0
    distance_from_home: float = 0.0
    image_topic: str = ''
    valid: bool = False
    last_pixel_center: list = field(default_factory=list)


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ArucoLandmarkNode(Node):
    def __init__(self) -> None:
        super().__init__('aruco_landmark_node')
        self.declare_parameter('image_topic', '/front_camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/front_camera/color/camera_info')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('target_marker_ids', '6')
        self.declare_parameter('marker_size_m', 0.05)
        self.declare_parameter('required_seen_count', 3)
        self.declare_parameter('save_path', '/ros2_ws/maps/aruco_landmarks.json')
        self.declare_parameter('save_period_sec', 2.0)
        self.declare_parameter('publish_debug_image', False)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self.global_frame = str(self.get_parameter('global_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.required_seen_count = int(self.get_parameter('required_seen_count').value)
        self.target_marker_ids = self._parse_target_marker_ids(
            str(self.get_parameter('target_marker_ids').value))
        self.save_path = str(self.get_parameter('save_path').value)
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)

        self.bridge = CvBridge()
        self.camera_info: Optional[CameraInfo] = None
        self.home_pose: Optional[PoseStamped] = None
        self.landmarks: Dict[int, Landmark] = {}
        self.last_saved_text = ''

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.detector = self._create_detector()

        self.found_pub = self.create_publisher(Bool, '~/found', 1)
        self.summary_pub = self.create_publisher(String, '~/summary', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '~/markers', 1)
        self.debug_pub = (
            self.create_publisher(Image, '~/debug_image', 1)
            if self.publish_debug_image else None
        )

        self.create_subscription(Image, self.image_topic, self.on_image, 1)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.on_camera_info, 1)
        self.create_timer(float(self.get_parameter('save_period_sec').value), self.save_landmarks)
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info(
            f'ArUco landmark node started: image={self.image_topic}, '
            f'camera_info={self.camera_info_topic}, dictionary='
            f'{self.get_parameter("aruco_dictionary").value}, '
            f'target_marker_ids={sorted(self.target_marker_ids) if self.target_marker_ids else "all"}, '
            f'required_seen_count={self.required_seen_count}.')

    def _parse_target_marker_ids(self, text: str):
        cleaned = text.strip()
        if not cleaned or cleaned.lower() in ('all', '*', 'any'):
            return set()
        ids = set()
        for item in cleaned.replace(';', ',').split(','):
            item = item.strip()
            if not item:
                continue
            ids.add(int(item))
        return ids

    def _create_detector(self):
        dictionary_name = str(self.get_parameter('aruco_dictionary').value)
        if not hasattr(cv2, 'aruco'):
            raise RuntimeError('OpenCV was built without cv2.aruco support')
        if not hasattr(cv2.aruco, dictionary_name):
            raise RuntimeError(f'Unknown ArUco dictionary: {dictionary_name}')
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
        parameters = cv2.aruco.DetectorParameters()
        if hasattr(cv2.aruco, 'ArucoDetector'):
            return cv2.aruco.ArucoDetector(dictionary, parameters)
        return (dictionary, parameters)

    def on_camera_info(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def robot_pose(self) -> Optional[PoseStamped]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().warn(
                f'Cannot read {self.global_frame}->{self.base_frame}: {exc}',
                throttle_duration_sec=2.0)
            return None
        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def on_image(self, msg: Image) -> None:
        pose = self.robot_pose()
        if pose is None:
            return
        if self.home_pose is None:
            self.home_pose = pose
            self.get_logger().info(
                f'Home landmark captured at x={pose.pose.position.x:.2f}, '
                f'y={pose.pose.position.y:.2f}.')

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'Cannot convert image for ArUco detection: {exc}')
            return

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if isinstance(self.detector, tuple):
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.detector[0], parameters=self.detector[1])
        else:
            corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is None or len(ids) == 0:
            self.publish_found_flag()
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        yaw = yaw_from_quaternion(pose.pose.orientation)
        distance = 0.0
        if self.home_pose is not None:
            distance = math.hypot(
                pose.pose.position.x - self.home_pose.pose.position.x,
                pose.pose.position.y - self.home_pose.pose.position.y,
            )

        for marker_id_raw, marker_corners in zip(ids.flatten(), corners):
            marker_id = int(marker_id_raw)
            if self.target_marker_ids and marker_id not in self.target_marker_ids:
                continue
            center = marker_corners.reshape(-1, 2).mean(axis=0)
            landmark = self.landmarks.get(marker_id)
            if landmark is None:
                landmark = Landmark(
                    marker_id=marker_id,
                    first_seen_sec=now_sec,
                    last_seen_sec=now_sec,
                    image_topic=self.image_topic,
                )
                self.landmarks[marker_id] = landmark
            landmark.seen_count += 1
            landmark.last_seen_sec = now_sec
            landmark.approach_x = float(pose.pose.position.x)
            landmark.approach_y = float(pose.pose.position.y)
            landmark.approach_yaw = float(yaw)
            landmark.distance_from_home = float(distance)
            landmark.last_pixel_center = [float(center[0]), float(center[1])]
            landmark.valid = landmark.seen_count >= self.required_seen_count

            state = 'valid' if landmark.valid else 'candidate'
            self.get_logger().info(
                f'ArUco marker {marker_id} seen '
                f'({landmark.seen_count}/{self.required_seen_count}, {state}); '
                f'approach=({landmark.approach_x:.2f}, {landmark.approach_y:.2f}), '
                f'distance_from_home={landmark.distance_from_home:.2f}m',
                throttle_duration_sec=1.0)

        if self.debug_pub is not None:
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(image, encoding='bgr8'))
        self.publish_found_flag()
        self.publish_markers()

    def valid_landmarks(self):
        return [item for item in self.landmarks.values() if item.valid]

    def furthest_landmark(self) -> Optional[Landmark]:
        valid = self.valid_landmarks()
        if not valid:
            return None
        return max(valid, key=lambda item: item.distance_from_home)

    def publish_found_flag(self) -> None:
        msg = Bool()
        msg.data = len(self.valid_landmarks()) > 0
        self.found_pub.publish(msg)

    def publish_status(self) -> None:
        best = self.furthest_landmark()
        payload = {
            'home_saved': self.home_pose is not None,
            'candidate_count': len(self.landmarks),
            'valid_count': len(self.valid_landmarks()),
            'furthest_marker_id': best.marker_id if best else None,
            'furthest_distance_from_home_m': best.distance_from_home if best else None,
            'save_path': self.save_path,
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.summary_pub.publish(msg)
        self.publish_found_flag()
        self.publish_markers()

    def publish_markers(self) -> None:
        array = MarkerArray()
        now = self.get_clock().now().to_msg()
        for landmark in self.landmarks.values():
            marker = Marker()
            marker.header.frame_id = self.global_frame
            marker.header.stamp = now
            marker.ns = 'aruco_door_landmarks'
            marker.id = landmark.marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = landmark.approach_x
            marker.pose.position.y = landmark.approach_y
            marker.pose.position.z = 0.15
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.25
            marker.color.a = 1.0
            marker.color.r = 0.1 if landmark.valid else 1.0
            marker.color.g = 0.9 if landmark.valid else 0.6
            marker.color.b = 0.1
            array.markers.append(marker)

            text = Marker()
            text.header.frame_id = self.global_frame
            text.header.stamp = now
            text.ns = 'aruco_door_landmark_labels'
            text.id = 10000 + landmark.marker_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = landmark.approach_x
            text.pose.position.y = landmark.approach_y
            text.pose.position.z = 0.55
            text.pose.orientation.w = 1.0
            text.scale.z = 0.25
            text.color.a = 1.0
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.text = (
                f'aruco_{landmark.marker_id}\n'
                f'seen={landmark.seen_count}\n'
                f'd={landmark.distance_from_home:.1f}m')
            array.markers.append(text)
        self.marker_pub.publish(array)

    def save_landmarks(self) -> None:
        payload = {
            'global_frame': self.global_frame,
            'base_frame': self.base_frame,
            'home': self._pose_to_dict(self.home_pose) if self.home_pose else None,
            'landmarks': [
                {
                    'id': f'aruco_{lm.marker_id}',
                    'marker_id': lm.marker_id,
                    'type': 'door',
                    'valid': lm.valid,
                    'seen_count': lm.seen_count,
                    'distance_from_home_m': lm.distance_from_home,
                    'approach_pose': {
                        'frame_id': self.global_frame,
                        'x': lm.approach_x,
                        'y': lm.approach_y,
                        'yaw': lm.approach_yaw,
                    },
                    'last_pixel_center': lm.last_pixel_center,
                    'first_seen_sec': lm.first_seen_sec,
                    'last_seen_sec': lm.last_seen_sec,
                    'image_topic': lm.image_topic,
                }
                for lm in sorted(self.landmarks.values(), key=lambda item: item.marker_id)
            ],
        }
        text = json.dumps(payload, indent=2, sort_keys=True)
        if text == self.last_saved_text:
            return
        directory = os.path.dirname(self.save_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        tmp_path = f'{self.save_path}.tmp'
        with open(tmp_path, 'w', encoding='utf-8') as handle:
            handle.write(text)
            handle.write('\n')
        os.replace(tmp_path, self.save_path)
        self.last_saved_text = text

    def _pose_to_dict(self, pose: Optional[PoseStamped]):
        if pose is None:
            return None
        return {
            'frame_id': pose.header.frame_id,
            'x': pose.pose.position.x,
            'y': pose.pose.position.y,
            'yaw': yaw_from_quaternion(pose.pose.orientation),
        }


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoLandmarkNode()
    try:
        rclpy.spin(node)
    finally:
        node.save_landmarks()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
