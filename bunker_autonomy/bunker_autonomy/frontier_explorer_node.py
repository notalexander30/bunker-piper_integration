#!/usr/bin/env python3
"""Conservative frontier exploration for RTAB-Map/Nav2 mapping.

The node looks for frontier cells in /map: unknown cells touching known-free
cells. It sends one Nav2 goal at a time, then returns to the robot pose captured
when /frontier_explorer/start is called.
"""

import math
from collections import deque
from dataclasses import dataclass
from typing import List, Optional, Sequence, Set, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


Cell = Tuple[int, int]


@dataclass
class Frontier:
    cells: List[Cell]
    x: float
    y: float
    distance: float
    frontier_center_x: float
    frontier_center_y: float


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def yaw_from_quaternion(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class FrontierExplorer(Node):
    def __init__(self) -> None:
        super().__init__('frontier_explorer_node')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('action_name', '/navigate_to_pose')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('dry_run', True)
        self.declare_parameter('return_to_start', True)
        self.declare_parameter('completion_mode', 'time_limit')
        self.declare_parameter('time_limit_sec', 120.0)
        self.declare_parameter('frontier_stop_ratio', 0.20)
        self.declare_parameter('explore_period_sec', 2.0)
        self.declare_parameter('frontier_min_distance_m', 0.75)
        self.declare_parameter('frontier_max_distance_m', 4.0)
        self.declare_parameter('frontier_cluster_min_cells', 12)
        self.declare_parameter('goal_timeout_sec', 75.0)
        self.declare_parameter('failed_frontier_blacklist_radius_m', 0.7)
        self.declare_parameter('no_frontier_return_cycles', 2)
        self.declare_parameter('aruco_found_topic', '/aruco_landmarks/found')
        self.declare_parameter('safety_stop_topic', '/safety_stop')
        self.declare_parameter('mux_reason_topic', '/cmd_vel_mux/reason')
        self.declare_parameter('clear_pose_min_age_sec', 1.0)
        self.declare_parameter('recovery_goal_min_distance_m', 0.35)

        self.global_frame = self.get_parameter(
            'global_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter(
            'base_frame').get_parameter_value().string_value
        self.dry_run = self.get_parameter('dry_run').get_parameter_value().bool_value
        self.return_to_start = self.get_parameter(
            'return_to_start').get_parameter_value().bool_value
        self.completion_mode = self.get_parameter(
            'completion_mode').get_parameter_value().string_value
        self.time_limit_sec = self.get_parameter(
            'time_limit_sec').get_parameter_value().double_value
        self.frontier_stop_ratio = self.get_parameter(
            'frontier_stop_ratio').get_parameter_value().double_value
        self.min_distance = self.get_parameter(
            'frontier_min_distance_m').get_parameter_value().double_value
        self.max_distance = self.get_parameter(
            'frontier_max_distance_m').get_parameter_value().double_value
        self.min_cluster_cells = self.get_parameter(
            'frontier_cluster_min_cells').get_parameter_value().integer_value
        self.goal_timeout_sec = self.get_parameter(
            'goal_timeout_sec').get_parameter_value().double_value
        self.blacklist_radius = self.get_parameter(
            'failed_frontier_blacklist_radius_m').get_parameter_value().double_value
        self.no_frontier_return_cycles = max(
            1,
            self.get_parameter(
                'no_frontier_return_cycles').get_parameter_value().integer_value,
        )
        self.clear_pose_min_age_sec = max(
            0.0,
            self.get_parameter(
                'clear_pose_min_age_sec').get_parameter_value().double_value,
        )
        self.recovery_goal_min_distance_m = max(
            0.0,
            self.get_parameter(
                'recovery_goal_min_distance_m').get_parameter_value().double_value,
        )

        map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        action_name = self.get_parameter(
            'action_name').get_parameter_value().string_value
        aruco_found_topic = self.get_parameter(
            'aruco_found_topic').get_parameter_value().string_value
        safety_stop_topic = self.get_parameter(
            'safety_stop_topic').get_parameter_value().string_value
        mux_reason_topic = self.get_parameter(
            'mux_reason_topic').get_parameter_value().string_value
        period = self.get_parameter(
            'explore_period_sec').get_parameter_value().double_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = ActionClient(self, NavigateToPose, action_name)
        self.map_msg: Optional[OccupancyGrid] = None
        self.active_goal = None
        self.active_goal_handle = None
        self.active_goal_started = None
        self.active_goal_id = 0
        self.return_goal_sent = False
        self.failed_goals: List[Tuple[float, float]] = []
        self.start_pose: Optional[PoseStamped] = None
        self.started = False
        self.finished = False
        self.exploration_started_time = None
        self.last_frontier_ratio = 1.0
        self.no_frontier_cycles = 0
        self.last_frontier_cell_count = 0
        self.last_free_cell_count = 0
        self.last_cluster_count = 0
        self.last_usable_frontier_count = 0
        self.aruco_found = False
        self.safety_stop = True
        self.mux_reason = 'unknown'
        self.last_clear_pose: Optional[PoseStamped] = None
        self.last_clear_pose_time = None
        self.recovery_goal_active = False

        self.create_subscription(OccupancyGrid, map_topic, self.on_map, 1)
        self.create_subscription(Bool, aruco_found_topic, self.on_aruco_found, 1)
        self.create_subscription(Bool, safety_stop_topic, self.on_safety_stop, 10)
        self.create_subscription(String, mux_reason_topic, self.on_mux_reason, 10)
        self.create_service(Trigger, '~/start', self.start_exploration)
        self.create_service(Trigger, '~/stop', self.stop_exploration)
        self.create_timer(period, self.tick)
        self.get_logger().info(
            f'Frontier explorer started. dry_run={self.dry_run}; '
            f'return_to_start={self.return_to_start}; '
            f'completion_mode={self.completion_mode}; map_topic={map_topic}. '
            f'aruco_found_topic={aruco_found_topic}. '
            f'safety_stop_topic={safety_stop_topic}; '
            f'mux_reason_topic={mux_reason_topic}. '
            'Call /frontier_explorer/start to capture home and begin.')

    def on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def on_aruco_found(self, msg: Bool) -> None:
        self.aruco_found = bool(msg.data)

    def on_safety_stop(self, msg: Bool) -> None:
        self.safety_stop = bool(msg.data)

    def on_mux_reason(self, msg: String) -> None:
        self.mux_reason = msg.data

    def start_exploration(self, request, response):
        if self.map_msg is None:
            response.success = False
            response.message = 'Cannot start: /map has not been received.'
            return response
        robot = self.robot_pose()
        if robot is None:
            response.success = False
            response.message = (
                f'Cannot start: missing {self.global_frame}->{self.base_frame} TF.')
            return response
        self.start_pose = robot
        self.exploration_started_time = self.get_clock().now()
        self.started = True
        self.finished = False
        self.return_goal_sent = False
        self.active_goal = None
        self.active_goal_handle = None
        self.recovery_goal_active = False
        self.failed_goals.clear()
        self.no_frontier_cycles = 0
        response.success = True
        response.message = (
            f'Exploration started; home captured at '
            f'x={robot.pose.position.x:.2f}, y={robot.pose.position.y:.2f}.')
        self.get_logger().info(response.message)
        return response

    def stop_exploration(self, request, response):
        self.started = False
        self.finished = True
        self.active_goal = None
        self.cancel_active_goal()
        self.recovery_goal_active = False
        response.success = True
        response.message = 'Exploration stopped by operator.'
        self.get_logger().warn(response.message)
        return response

    def robot_pose(self) -> Optional[PoseStamped]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().warn(
                f'Cannot read {self.global_frame}->{self.base_frame}: {exc}')
            return None
        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.orientation = transform.transform.rotation
        return pose

    def tick(self) -> None:
        if not self.started or self.finished:
            return
        if self.map_msg is None:
            self.get_logger().info('Waiting for /map before exploring.')
            return
        robot = self.robot_pose()
        if robot is None:
            return
        self.update_last_clear_pose(robot)

        if self.active_goal is not None:
            if self.should_recover_from_stale_controller():
                self.recover_from_stale_controller(robot)
                return
            elapsed = (
                self.get_clock().now() - self.active_goal_started).nanoseconds / 1e9
            if elapsed > self.goal_timeout_sec:
                self.get_logger().warn(
                    f'Current exploration goal timed out after {elapsed:.1f}s.')
                if self.active_goal:
                    self.failed_goals.append((self.active_goal[0], self.active_goal[1]))
                self.cancel_active_goal()
                self.active_goal = None
            return

        if self.safety_stop:
            self.get_logger().warn(
                'Waiting for /safety_stop to clear before selecting another frontier.',
                throttle_duration_sec=2.0,
            )
            return

        if self.should_finish_by_time():
            self.get_logger().info(
                f'Time limit reached ({self.time_limit_sec:.1f}s); returning home.')
            self.finish_and_return()
            return
        if self.should_finish_by_aruco():
            self.get_logger().info(
                'ArUco landmark found; stopping exploration and returning home.')
            self.finish_and_return()
            return

        frontiers = self.find_frontiers(robot.pose.position.x, robot.pose.position.y)
        if self.should_finish_by_frontier_ratio():
            self.get_logger().info(
                f'Frontier ratio {self.last_frontier_ratio:.3f} <= '
                f'{self.frontier_stop_ratio:.3f}; returning home.')
            self.finish_and_return()
            return

        if not frontiers:
            self.no_frontier_cycles += 1
            self.get_logger().warn(
                'No usable frontier this cycle '
                f'({self.no_frontier_cycles}/{self.no_frontier_return_cycles}); '
                f'frontier_cells={self.last_frontier_cell_count}, '
                f'clusters={self.last_cluster_count}, '
                f'usable={self.last_usable_frontier_count}, '
                f'free_cells={self.last_free_cell_count}, '
                f'ratio={self.last_frontier_ratio:.3f}.')
            if self.no_frontier_cycles >= self.no_frontier_return_cycles:
                self.get_logger().info(
                    'No usable frontiers persisted; returning home.')
                self.finish_and_return()
            return
        self.no_frontier_cycles = 0

        target = frontiers[0]
        goal = PoseStamped()
        goal.header.frame_id = self.global_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = target.x
        goal.pose.position.y = target.y
        yaw = math.atan2(
            target.y - robot.pose.position.y,
            target.x - robot.pose.position.x)
        goal.pose.orientation = quaternion_from_yaw(yaw)
        self.get_logger().info(
            f'Selected frontier: x={target.x:.2f}, y={target.y:.2f}, '
            f'distance={target.distance:.2f}m, cells={len(target.cells)}, '
            f'frontier_center=({target.frontier_center_x:.2f}, '
            f'{target.frontier_center_y:.2f}).')
        self.send_goal(goal, label='frontier')

    def should_finish_by_time(self) -> bool:
        if self.completion_mode != 'time_limit':
            return False
        if self.exploration_started_time is None:
            return False
        elapsed = (
            self.get_clock().now() - self.exploration_started_time).nanoseconds / 1e9
        return elapsed >= self.time_limit_sec

    def should_finish_by_frontier_ratio(self) -> bool:
        return (
            self.completion_mode == 'frontier_ratio'
            and self.last_frontier_ratio <= self.frontier_stop_ratio
        )

    def should_finish_by_aruco(self) -> bool:
        return self.completion_mode == 'aruco_found' and self.aruco_found

    def finish_and_return(self) -> None:
        if self.return_to_start and not self.return_goal_sent and self.start_pose:
            self.return_goal_sent = True
            self.send_goal(self.start_pose, label='return_to_start')
            if self.dry_run:
                self.finished = True
                self.started = False
        else:
            self.finished = True
            self.started = False

    def update_last_clear_pose(self, robot: PoseStamped) -> None:
        if self.safety_stop or self.mux_reason in ('watchdog_timeout', 'invalid_reverse'):
            return
        if self.active_goal is not None and self.active_goal[2] in (
            'frontier',
            'recovery_to_clear_pose',
        ):
            return
        self.last_clear_pose = robot
        self.last_clear_pose_time = self.get_clock().now()

    def should_recover_from_stale_controller(self) -> bool:
        if self.recovery_goal_active:
            return False
        if self.active_goal is None or self.active_goal[2] != 'frontier':
            return False
        return self.mux_reason == 'watchdog_timeout'

    def recover_from_stale_controller(self, robot: PoseStamped) -> None:
        if self.active_goal:
            self.failed_goals.append((self.active_goal[0], self.active_goal[1]))
        reason = self.mux_reason
        recovery_pose = self.choose_recovery_pose(robot)
        self.cancel_active_goal()
        self.active_goal = None
        if recovery_pose is None:
            self.recovery_goal_active = False
            self.get_logger().warn(
                f'Controller became stale during frontier goal ({reason}); '
                'no safe recovery pose is available yet, waiting for map/safety update.')
            return
        self.get_logger().warn(
            f'Controller became stale during frontier goal ({reason}); '
            'rerouting to known-clear pose before continuing exploration.')
        self.recovery_goal_active = True
        self.send_goal(recovery_pose, label='recovery_to_clear_pose')

    def choose_recovery_pose(self, robot: PoseStamped) -> Optional[PoseStamped]:
        candidates = []
        if self.last_clear_pose is not None and self.last_clear_pose_time is not None:
            age = (
                self.get_clock().now() - self.last_clear_pose_time).nanoseconds / 1e9
            distance = math.hypot(
                self.last_clear_pose.pose.position.x - robot.pose.position.x,
                self.last_clear_pose.pose.position.y - robot.pose.position.y,
            )
            if age >= self.clear_pose_min_age_sec and distance >= self.recovery_goal_min_distance_m:
                candidates.append(self.last_clear_pose)
        if self.start_pose is not None:
            distance = math.hypot(
                self.start_pose.pose.position.x - robot.pose.position.x,
                self.start_pose.pose.position.y - robot.pose.position.y,
            )
            if distance >= self.recovery_goal_min_distance_m:
                candidates.append(self.start_pose)
        return candidates[0] if candidates else None

    def send_goal(self, pose: PoseStamped, label: str) -> None:
        if self.dry_run:
            self.get_logger().info(
                f'DRY RUN: would send {label} goal at '
                f'x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}.')
            return
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 action server not ready; goal not sent.')
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.active_goal = (pose.pose.position.x, pose.pose.position.y, label)
        self.active_goal_started = self.get_clock().now()
        self.active_goal_id += 1
        goal_id = self.active_goal_id
        future = self.client.send_goal_async(goal)
        future.add_done_callback(lambda done: self._goal_response(done, goal_id))

    def cancel_active_goal(self) -> None:
        if self.active_goal_handle is None:
            return
        try:
            self.active_goal_handle.cancel_goal_async()
        except Exception as exc:
            self.get_logger().warn(f'Failed to cancel active Nav2 goal: {exc}')
        self.active_goal_handle = None

    def _goal_response(self, future, goal_id: int) -> None:
        if goal_id != self.active_goal_id:
            self.get_logger().info('Ignoring stale Nav2 goal response.')
            return
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Exploration goal rejected.')
            if self.active_goal:
                self.failed_goals.append((self.active_goal[0], self.active_goal[1]))
            self.active_goal = None
            return
        self.active_goal_handle = goal_handle
        self.get_logger().info('Exploration goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda done: self._goal_result(done, goal_id))

    def _goal_result(self, future, goal_id: int) -> None:
        if goal_id != self.active_goal_id:
            self.get_logger().info('Ignoring stale Nav2 goal result.')
            return
        result = future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED and self.active_goal:
            self.get_logger().warn(
                f'Exploration goal failed with status={result.status}; '
                'blacklisting this frontier.')
            if self.active_goal[2] == 'frontier':
                self.failed_goals.append((self.active_goal[0], self.active_goal[1]))
        else:
            self.get_logger().info('Exploration goal succeeded.')
        if self.active_goal and self.active_goal[2] == 'return_to_start':
            self.get_logger().info('Returned to home; exploration complete.')
            self.finished = True
            self.started = False
        if self.active_goal and self.active_goal[2] == 'recovery_to_clear_pose':
            self.recovery_goal_active = False
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(
                    'Recovered to known-clear pose; resuming frontier selection.')
            else:
                self.get_logger().warn(
                    'Recovery-to-clear-pose goal failed; waiting for safety/map update.')
        self.active_goal = None
        self.active_goal_handle = None

    def find_frontiers(self, robot_x: float, robot_y: float) -> List[Frontier]:
        msg = self.map_msg
        width = msg.info.width
        height = msg.info.height
        data = msg.data
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        def index(cell: Cell) -> int:
            return cell[1] * width + cell[0]

        def in_bounds(cell: Cell) -> bool:
            return 0 <= cell[0] < width and 0 <= cell[1] < height

        def neighbors4(cell: Cell) -> Sequence[Cell]:
            x, y = cell
            return ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1))

        def world(cell: Cell) -> Tuple[float, float]:
            return (
                origin_x + (cell[0] + 0.5) * resolution,
                origin_y + (cell[1] + 0.5) * resolution,
            )

        def is_frontier(cell: Cell) -> bool:
            if data[index(cell)] != -1:
                return False
            for nb in neighbors4(cell):
                if in_bounds(nb) and data[index(nb)] == 0:
                    return True
            return False

        frontier_cells: Set[Cell] = set()
        free_cells = 0
        for y in range(height):
            row = y * width
            for x in range(width):
                cell = (x, y)
                if data[row + x] == 0:
                    free_cells += 1
                if data[row + x] == -1 and is_frontier(cell):
                    frontier_cells.add(cell)
        self.last_frontier_ratio = len(frontier_cells) / max(float(free_cells), 1.0)
        self.last_frontier_cell_count = len(frontier_cells)
        self.last_free_cell_count = free_cells

        clusters: List[List[Cell]] = []
        visited: Set[Cell] = set()
        for cell in frontier_cells:
            if cell in visited:
                continue
            cluster = []
            queue = deque([cell])
            visited.add(cell)
            while queue:
                cur = queue.popleft()
                cluster.append(cur)
                for nb in neighbors4(cur):
                    if nb in frontier_cells and nb not in visited:
                        visited.add(nb)
                        queue.append(nb)
            if len(cluster) >= self.min_cluster_cells:
                clusters.append(cluster)
        self.last_cluster_count = len(clusters)

        frontiers: List[Frontier] = []
        for cluster in clusters:
            xs, ys = zip(*(world(cell) for cell in cluster))
            cx = sum(xs) / len(xs)
            cy = sum(ys) / len(ys)

            free_neighbors: Set[Cell] = set()
            for cell in cluster:
                for nb in neighbors4(cell):
                    if in_bounds(nb) and data[index(nb)] == 0:
                        free_neighbors.add(nb)
            if not free_neighbors:
                continue

            # Nav2 goals must be in known-free space. Target the free cell next
            # to the unknown frontier that is closest to the frontier centroid.
            goal_cell = min(
                free_neighbors,
                key=lambda cell: math.hypot(world(cell)[0] - cx, world(cell)[1] - cy),
            )
            gx, gy = world(goal_cell)
            distance = math.hypot(gx - robot_x, gy - robot_y)
            if distance < self.min_distance or distance > self.max_distance:
                continue
            if any(
                math.hypot(gx - fx, gy - fy) < self.blacklist_radius
                for fx, fy in self.failed_goals
            ):
                continue
            frontiers.append(Frontier(cluster, gx, gy, distance, cx, cy))

        self.last_usable_frontier_count = len(frontiers)
        frontiers.sort(key=lambda item: (item.distance, -len(item.cells)))
        return frontiers


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
