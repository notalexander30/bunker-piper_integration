#!/usr/bin/env python3
"""Send Nav2 goals to saved manual or ArUco landmarks."""

import json
import math
import os
from typing import Dict, Iterable, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def yaw_from_quaternion(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_name(name: str) -> str:
    return name.strip().lower().replace(' ', '_')


def yaw_between(source, target) -> float:
    return math.atan2(
        float(target['y']) - float(source['y']),
        float(target['x']) - float(source['x']),
    )


class LandmarkNavigator(Node):
    def __init__(self) -> None:
        super().__init__('landmark_navigator')
        self.declare_parameter(
            'landmark_path', '/ros2_ws/maps/manual_nav_landmarks.json')
        self.declare_parameter('action_name', '/navigate_to_pose')
        self.declare_parameter('default_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('goal_timeout_sec', 120.0)
        self.declare_parameter(
            'marker_topic', '/landmark_navigator/markers')
        self.declare_parameter('marker_publish_hz', 1.0)
        self.declare_parameter('auto_create_file', True)
        self.declare_parameter('door_arrival_topic', '/door_navigation/arrived')
        self.declare_parameter('home_arrival_topic', '/home_navigation/arrived')
        self.declare_parameter('door_arrival_publish_hz', 2.0)
        self.declare_parameter('home_arrival_publish_hz', 2.0)
        self.declare_parameter('arrival_true_publish_count', 10)
        self.declare_parameter('active_camera_topic', '~/active_camera')
        self.declare_parameter('navigation_direction_topic', '~/navigation_direction')
        self.declare_parameter('forward_camera_name', 'front_camera')
        self.declare_parameter('reverse_camera_name', 'rear_camera')
        self.declare_parameter('reverse_home_from_door', True)
        self.declare_parameter('reverse_home_from_door_radius_m', 1.0)

        self.landmark_path = str(self.get_parameter('landmark_path').value)
        action_name = str(self.get_parameter('action_name').value)
        self.default_frame = str(self.get_parameter('default_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.goal_timeout_sec = float(
            self.get_parameter('goal_timeout_sec').value)
        self.marker_topic = str(self.get_parameter('marker_topic').value)
        self.door_arrival_topic = str(
            self.get_parameter('door_arrival_topic').value)
        self.home_arrival_topic = str(
            self.get_parameter('home_arrival_topic').value)
        self.door_arrival_publish_hz = max(
            0.1, float(self.get_parameter('door_arrival_publish_hz').value))
        self.home_arrival_publish_hz = max(
            0.1, float(self.get_parameter('home_arrival_publish_hz').value))
        self.arrival_true_publish_count = max(
            1, int(self.get_parameter('arrival_true_publish_count').value))
        self.active_camera_topic = str(
            self.get_parameter('active_camera_topic').value)
        self.navigation_direction_topic = str(
            self.get_parameter('navigation_direction_topic').value)
        self.forward_camera_name = str(
            self.get_parameter('forward_camera_name').value)
        self.reverse_camera_name = str(
            self.get_parameter('reverse_camera_name').value)
        self.reverse_home_from_door = bool(
            self.get_parameter('reverse_home_from_door').value)
        self.reverse_home_from_door_radius_m = max(
            0.0,
            float(self.get_parameter('reverse_home_from_door_radius_m').value),
        )
        marker_publish_hz = max(
            0.2, float(self.get_parameter('marker_publish_hz').value))
        self.auto_create_file = bool(
            self.get_parameter('auto_create_file').value)

        self.client = ActionClient(self, NavigateToPose, action_name)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.marker_pub = self.create_publisher(
            MarkerArray, self.marker_topic, 10)
        self.door_arrival_pub = self.create_publisher(
            Bool, self.door_arrival_topic, 10)
        self.home_arrival_pub = self.create_publisher(
            Bool, self.home_arrival_topic, 10)
        self.active_camera_pub = self.create_publisher(
            String, self.active_camera_topic, 10)
        self.navigation_direction_pub = self.create_publisher(
            String, self.navigation_direction_topic, 10)
        self.active_goal_label = None
        self.active_goal_started = None
        self.active_goal_direction = 'forward'
        self.last_succeeded_label = None
        self.door_arrived = False
        self.home_arrived = False
        self.door_arrival_true_remaining = 0
        self.home_arrival_true_remaining = 0

        self.create_service(Trigger, '~/go_home', self.go_home)
        self.create_service(Trigger, '~/go_furthest', self.go_furthest)
        self.create_subscription(String, '~/go_marker', self.go_marker_msg, 10)
        self.create_subscription(
            String, '~/save_current_pose', self.save_current_pose_msg, 10)
        self.create_timer(1.0, self.tick)
        self.create_timer(1.0 / marker_publish_hz, self.publish_markers)
        self.create_timer(
            1.0 / self.door_arrival_publish_hz,
            self.publish_door_arrival)
        self.create_timer(
            1.0 / self.home_arrival_publish_hz,
            self.publish_home_arrival)

        if self.auto_create_file:
            self.ensure_landmark_file()

        self.get_logger().info(
            f'Landmark navigator ready. landmark_path={self.landmark_path}. '
            'Publish a name such as "home" or "door" to '
            '/landmark_navigator/go_marker to send Nav2 there. Publish a name '
            'to /landmark_navigator/save_current_pose to save the current '
            'map pose.')

    def set_door_arrived(self, arrived: bool, publish_now: bool = True) -> None:
        self.door_arrived = bool(arrived)
        self.door_arrival_true_remaining = (
            self.arrival_true_publish_count if self.door_arrived else 0)
        if publish_now:
            self.publish_door_arrival()

    def publish_door_arrival(self) -> None:
        msg = Bool()
        msg.data = self.door_arrival_true_remaining > 0
        self.door_arrival_pub.publish(msg)
        if self.door_arrival_true_remaining > 0:
            self.door_arrival_true_remaining -= 1
            if self.door_arrival_true_remaining <= 0:
                self.door_arrived = False

    def set_home_arrived(self, arrived: bool, publish_now: bool = True) -> None:
        self.home_arrived = bool(arrived)
        self.home_arrival_true_remaining = (
            self.arrival_true_publish_count if self.home_arrived else 0)
        if publish_now:
            self.publish_home_arrival()

    def publish_home_arrival(self) -> None:
        msg = Bool()
        msg.data = self.home_arrival_true_remaining > 0
        self.home_arrival_pub.publish(msg)
        if self.home_arrival_true_remaining > 0:
            self.home_arrival_true_remaining -= 1
            if self.home_arrival_true_remaining <= 0:
                self.home_arrived = False

    def publish_navigation_mode(self, direction: str) -> None:
        direction = 'reverse' if direction == 'reverse' else 'forward'
        camera_name = (
            self.reverse_camera_name
            if direction == 'reverse'
            else self.forward_camera_name
        )
        camera_msg = String()
        camera_msg.data = camera_name
        self.active_camera_pub.publish(camera_msg)
        direction_msg = String()
        direction_msg.data = direction
        self.navigation_direction_pub.publish(direction_msg)
        self.get_logger().info(
            f'Navigation mode: direction={direction}, active_camera={camera_name}')

    def tick(self) -> None:
        if self.active_goal_started is None:
            return
        elapsed = (
            self.get_clock().now() - self.active_goal_started
        ).nanoseconds / 1e9
        if elapsed > self.goal_timeout_sec:
            self.get_logger().warn(
                f'Landmark goal {self.active_goal_label} has been active for '
                f'{elapsed:.1f}s; Nav2 may be blocked or unreachable.')
            self.active_goal_started = self.get_clock().now()

    def ensure_landmark_file(self) -> None:
        if os.path.isfile(self.landmark_path):
            return
        os.makedirs(os.path.dirname(self.landmark_path), exist_ok=True)
        data = {
            'home': {
                'frame_id': self.default_frame,
                'x': 0.0,
                'y': 0.0,
                'yaw': 0.0,
            },
            'manual_landmarks': [
                {
                    'id': 'door',
                    'frame_id': self.default_frame,
                    'x': 1.0,
                    'y': 0.0,
                    'yaw': 0.0,
                },
            ],
            'connections': [
                ['home', 'door'],
            ],
            'landmarks': [],
        }
        self.save_data(data)
        self.get_logger().warn(
            f'Created default landmark file: {self.landmark_path}. '
            'Update home/door coordinates before using it for real driving.')

    def load_data(self):
        if not os.path.isfile(self.landmark_path):
            raise FileNotFoundError(
                f'landmark file does not exist: {self.landmark_path}')
        with open(self.landmark_path, 'r', encoding='utf-8') as handle:
            return json.load(handle)

    def save_data(self, data) -> None:
        os.makedirs(os.path.dirname(self.landmark_path), exist_ok=True)
        temp_path = f'{self.landmark_path}.tmp'
        with open(temp_path, 'w', encoding='utf-8') as handle:
            json.dump(data, handle, indent=2, sort_keys=True)
            handle.write('\n')
        os.replace(temp_path, self.landmark_path)

    def go_home(self, _request, response):
        try:
            data = self.load_data()
            pose_data = self.find_named_pose(data, 'home')
            if pose_data is None:
                raise RuntimeError('home landmark is missing in landmark file')
            pose = self.home_pose_for_request(data, pose_data)
            self.send_goal(pose, 'home', self.home_direction_for_request(data))
            response.success = True
            response.message = 'Sent Nav2 goal to home landmark.'
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(response.message)
        return response

    def go_furthest(self, _request, response):
        try:
            data = self.load_data()
            landmark = self.furthest_valid_landmark(data)
            if landmark is None:
                raise RuntimeError('no valid ArUco landmark found')
            pose = self.pose_from_landmark(landmark)
            self.send_goal(pose, landmark.get('id', 'furthest'))
            response.success = True
            response.message = (
                f"Sent Nav2 goal to furthest landmark {landmark.get('id')} "
                f"distance={landmark.get('distance_from_home_m', 0.0):.2f}m.")
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(response.message)
        return response

    def go_marker_msg(self, msg: String) -> None:
        marker_name = normalize_name(msg.data)
        if not marker_name:
            self.get_logger().warn('Empty marker request ignored')
            return
        try:
            data = self.load_data()
            pose_data = self.find_named_pose(data, marker_name)
            if pose_data is not None:
                if marker_name == 'home':
                    self.send_goal(
                        self.home_pose_for_request(data, pose_data),
                        marker_name,
                        self.home_direction_for_request(data),
                    )
                else:
                    self.send_goal(
                        self.pose_from_dict(pose_data), marker_name, 'forward')
                return

            landmark = self.find_aruco_landmark(data, marker_name)
            if landmark is None:
                raise RuntimeError(f'landmark not found: {marker_name}')
            pose = self.pose_from_landmark(landmark)
            self.send_goal(pose, landmark.get('id', marker_name), 'forward')
        except Exception as exc:
            self.get_logger().error(str(exc))

    def save_current_pose_msg(self, msg: String) -> None:
        name = normalize_name(msg.data)
        if not name:
            self.get_logger().warn('Empty save-current-pose request ignored')
            return
        try:
            pose_data = self.current_robot_pose_dict()
            data = self.load_data() if os.path.isfile(self.landmark_path) else {}
            self.upsert_named_pose(data, name, pose_data)
            self.save_data(data)
            self.publish_markers()
            self.get_logger().info(
                f'Saved current robot pose as {name}: '
                f"x={pose_data['x']:.3f}, y={pose_data['y']:.3f}, "
                f"yaw={pose_data['yaw']:.3f}.")
        except Exception as exc:
            self.get_logger().error(str(exc))

    def current_robot_pose_dict(self) -> Dict[str, float]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.default_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=1.0),
            )
        except TransformException as exc:
            raise RuntimeError(
                f'cannot read {self.default_frame} -> {self.base_frame}: {exc}'
            ) from exc

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return {
            'frame_id': self.default_frame,
            'x': float(translation.x),
            'y': float(translation.y),
            'yaw': float(yaw_from_quaternion(rotation)),
        }

    def upsert_named_pose(self, data, name: str, pose_data) -> None:
        if name == 'home':
            data['home'] = pose_data
            return

        manual = data.setdefault('manual_landmarks', [])
        for entry in manual:
            if normalize_name(str(entry.get('id', ''))) == name:
                entry.update(pose_data)
                entry['id'] = name
                return
        entry = {'id': name}
        entry.update(pose_data)
        manual.append(entry)

    def named_poses(self, data) -> Dict[str, Dict]:
        poses: Dict[str, Dict] = {}
        home = data.get('home')
        if home:
            poses['home'] = home

        for entry in data.get('manual_landmarks', []):
            name = normalize_name(str(entry.get('id', '')))
            if name:
                poses[name] = entry

        for entry in data.get('landmarks', []):
            approach = entry.get('approach_pose')
            if not approach:
                continue
            names = [
                str(entry.get('id', '')),
                f"aruco_{entry.get('marker_id')}",
                str(entry.get('marker_id', '')),
            ]
            for name in names:
                normalized = normalize_name(name)
                if normalized:
                    poses.setdefault(normalized, approach)
        return poses

    def find_named_pose(self, data, marker_name: str):
        return self.named_poses(data).get(normalize_name(marker_name))

    def find_aruco_landmark(self, data, marker_name: str):
        requested = normalize_name(marker_name)
        if requested.isdigit():
            requested_names = {requested, f'aruco_{requested}'}
            requested_id = int(requested)
        elif requested.startswith('aruco_') and requested[6:].isdigit():
            requested_names = {requested, requested[6:]}
            requested_id = int(requested[6:])
        else:
            requested_names = {requested}
            requested_id = None
        for landmark in data.get('landmarks', []):
            names = {
                normalize_name(str(landmark.get('id', ''))),
                normalize_name(f"aruco_{landmark.get('marker_id')}"),
                normalize_name(str(landmark.get('marker_id', ''))),
            }
            if requested_id is not None and landmark.get('marker_id') == requested_id:
                return landmark
            if names & requested_names:
                return landmark
        return None

    def furthest_valid_landmark(self, data):
        valid = [
            landmark for landmark in data.get('landmarks', [])
            if bool(landmark.get('valid', False))
        ]
        if not valid:
            return None
        return max(
            valid,
            key=lambda item: float(item.get('distance_from_home_m', 0.0)))

    def pose_from_landmark(self, landmark) -> PoseStamped:
        approach = landmark.get('approach_pose')
        if not approach:
            raise RuntimeError(
                f"landmark {landmark.get('id')} has no approach_pose")
        return self.pose_from_dict(approach)

    def pose_from_dict(self, data) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = str(data.get('frame_id') or self.default_frame)
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(data['x'])
        pose.pose.position.y = float(data['y'])
        pose.pose.orientation = quaternion_from_yaw(
            float(data.get('yaw', 0.0)))
        return pose

    def current_pose_or_none(self):
        try:
            return self.current_robot_pose_dict()
        except Exception as exc:
            self.get_logger().warn(
                f'Cannot read current robot pose for reverse-home check: {exc}',
                throttle_duration_sec=2.0,
            )
            return None

    def should_reverse_home_from_door(self, data) -> bool:
        if not self.reverse_home_from_door:
            return False
        if normalize_name(str(self.last_succeeded_label or '')) == 'door':
            return True
        door_pose = self.find_named_pose(data, 'door')
        current_pose = self.current_pose_or_none()
        if door_pose is None or current_pose is None:
            return False
        distance_to_door = math.hypot(
            float(current_pose['x']) - float(door_pose['x']),
            float(current_pose['y']) - float(door_pose['y']),
        )
        return distance_to_door <= self.reverse_home_from_door_radius_m

    def home_direction_for_request(self, data) -> str:
        return 'reverse' if self.should_reverse_home_from_door(data) else 'forward'

    def home_pose_for_request(self, data, home_pose_data) -> PoseStamped:
        pose_data = dict(home_pose_data)
        if self.home_direction_for_request(data) == 'reverse':
            current_pose = self.current_pose_or_none()
            reference_pose = current_pose or self.find_named_pose(data, 'door')
            if reference_pose is not None:
                pose_data['yaw'] = yaw_between(pose_data, reference_pose)
                self.get_logger().info(
                    'Reverse home goal yaw set so the robot backs from door '
                    'toward home: yaw=%.3f rad.' % pose_data['yaw'])
        return self.pose_from_dict(pose_data)

    def send_goal(
        self,
        pose: PoseStamped,
        label: str,
        direction: str = 'forward',
    ) -> None:
        if not self.client.wait_for_server(timeout_sec=2.0):
            raise RuntimeError(
                'Nav2 /navigate_to_pose action server is not ready')
        direction = 'reverse' if direction == 'reverse' else 'forward'
        self.publish_navigation_mode(direction)
        self.set_door_arrived(False)
        self.set_home_arrived(False)
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.active_goal_label = label
        self.active_goal_started = self.get_clock().now()
        self.active_goal_direction = direction
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.goal_response)
        self.get_logger().info(
            f'Sent landmark goal {label}: '
            f'x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}, '
            f'frame={pose.header.frame_id}, direction={direction}.')

    def goal_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn(
                f'Landmark goal {self.active_goal_label} rejected')
            self.active_goal_started = None
            self.active_goal_direction = 'forward'
            self.publish_navigation_mode('forward')
            return
        self.get_logger().info(
            f'Landmark goal {self.active_goal_label} accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result)

    def goal_result(self, future) -> None:
        result = future.result()
        label = self.active_goal_label
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'Landmark goal {label} succeeded')
            self.last_succeeded_label = label
            if normalize_name(str(label or '')) == 'door':
                self.set_door_arrived(True)
                self.get_logger().info(
                    f'Door arrival published on '
                    f'{self.door_arrival_topic}: true')
            if normalize_name(str(label or '')) == 'home':
                self.set_home_arrived(True)
                self.get_logger().info(
                    f'Home arrival published on '
                    f'{self.home_arrival_topic}: true')
        else:
            self.get_logger().warn(
                f'Landmark goal {label} finished '
                f'with status={result.status}')
        self.active_goal_started = None
        self.active_goal_direction = 'forward'
        self.publish_navigation_mode('forward')

    def publish_markers(self) -> None:
        try:
            data = self.load_data()
        except Exception:
            return

        poses = self.named_poses(data)
        markers = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        now = self.get_clock().now().to_msg()
        for index, (name, pose_data) in enumerate(sorted(poses.items())):
            pose = self.pose_from_dict(pose_data)
            markers.markers.append(
                self.sphere_marker(index, name, pose, now))
            markers.markers.append(
                self.text_marker(index + 1000, name, pose, now))

        for index, pair in enumerate(self.connection_pairs(data, poses)):
            markers.markers.append(self.line_marker(index + 2000, pair, now))

        self.marker_pub.publish(markers)

    def connection_pairs(self, data, poses) -> Iterable[Tuple[PoseStamped, PoseStamped]]:
        for connection in data.get('connections', []):
            if len(connection) != 2:
                continue
            first = poses.get(normalize_name(str(connection[0])))
            second = poses.get(normalize_name(str(connection[1])))
            if first is None or second is None:
                continue
            yield self.pose_from_dict(first), self.pose_from_dict(second)

    def sphere_marker(
            self, marker_id: int, name: str, pose: PoseStamped, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = pose.header.frame_id
        marker.header.stamp = stamp
        marker.ns = 'manual_landmark_points'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = self.marker_pose(pose, 0.08)
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.a = 1.0
        if name == 'home':
            marker.color.r = 0.0
            marker.color.g = 0.55
            marker.color.b = 1.0
        elif name == 'door':
            marker.color.r = 1.0
            marker.color.g = 0.35
            marker.color.b = 0.0
        else:
            marker.color.r = 0.25
            marker.color.g = 0.85
            marker.color.b = 0.25
        return marker

    def text_marker(
            self, marker_id: int, name: str, pose: PoseStamped, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = pose.header.frame_id
        marker.header.stamp = stamp
        marker.ns = 'manual_landmark_labels'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose = self.marker_pose(pose, 0.45)
        marker.scale.z = 0.22
        marker.color.a = 1.0
        marker.color.r = 0.05
        marker.color.g = 0.05
        marker.color.b = 0.05
        marker.text = name
        return marker

    @staticmethod
    def marker_pose(pose: PoseStamped, z: float) -> Pose:
        marker_pose = Pose()
        marker_pose.position.x = pose.pose.position.x
        marker_pose.position.y = pose.pose.position.y
        marker_pose.position.z = z
        marker_pose.orientation = pose.pose.orientation
        return marker_pose

    def line_marker(
            self,
            marker_id: int,
            pair: Tuple[PoseStamped, PoseStamped],
            stamp,
    ) -> Marker:
        first, second = pair
        marker = Marker()
        marker.header.frame_id = first.header.frame_id
        marker.header.stamp = stamp
        marker.ns = 'manual_landmark_connections'
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.a = 0.9
        marker.color.r = 0.15
        marker.color.g = 0.15
        marker.color.b = 0.15
        p1 = Point()
        p1.x = first.pose.position.x
        p1.y = first.pose.position.y
        p1.z = 0.04
        p2 = Point()
        p2.x = second.pose.position.x
        p2.y = second.pose.position.y
        p2.z = 0.04
        marker.points = [p1, p2]
        return marker


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LandmarkNavigator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
