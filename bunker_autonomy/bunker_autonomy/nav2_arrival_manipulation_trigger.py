#!/usr/bin/env python3
"""Publish navigation/manipulation handoff progress."""

import json
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from rclpy.node import Node
from std_msgs.msg import Bool, String


def status_name(status: int) -> str:
    return {
        GoalStatus.STATUS_UNKNOWN: 'unknown',
        GoalStatus.STATUS_ACCEPTED: 'accepted',
        GoalStatus.STATUS_EXECUTING: 'executing',
        GoalStatus.STATUS_CANCELING: 'canceling',
        GoalStatus.STATUS_SUCCEEDED: 'succeeded',
        GoalStatus.STATUS_CANCELED: 'canceled',
        GoalStatus.STATUS_ABORTED: 'aborted',
    }.get(status, f'unknown_{status}')


class Nav2ArrivalManipulationTrigger(Node):
    """Bridge navigation completion into a continuous handoff progress topic."""

    def __init__(self) -> None:
        super().__init__('nav2_arrival_manipulation_trigger')

        self.declare_parameter(
            'status_topic', '/navigate_to_pose/_action/status')
        self.declare_parameter('trigger_topic', '/front_piper/task/start')
        self.declare_parameter(
            'trigger_bool_topic', '/front_piper/task/start_bool')
        self.declare_parameter(
            'receiver_topic', '/navigation_task/finished')
        self.declare_parameter(
            'progress_topic', '/navigation_manipulation/progress')
        self.declare_parameter(
            'manipulation_status_topic', '/manipulation_task/progress')
        self.declare_parameter('task_name', 'front_piper_pick')
        self.declare_parameter('receiver_message', 'navigation_finished')
        self.declare_parameter(
            'succeeded_status', GoalStatus.STATUS_SUCCEEDED)
        self.declare_parameter('trigger_on_nav2_success', True)
        self.declare_parameter('trigger_on_door_arrival', False)
        self.declare_parameter('door_arrival_topic', '/door_navigation/arrived')
        self.declare_parameter('publish_string', True)
        self.declare_parameter('publish_bool', True)
        self.declare_parameter('publish_receiver_message', True)
        self.declare_parameter('progress_publish_rate_hz', 1.0)

        self.status_topic = str(self.get_parameter('status_topic').value)
        self.trigger_topic = str(self.get_parameter('trigger_topic').value)
        self.trigger_bool_topic = str(
            self.get_parameter('trigger_bool_topic').value)
        self.receiver_topic = str(self.get_parameter('receiver_topic').value)
        self.progress_topic = str(self.get_parameter('progress_topic').value)
        self.manipulation_status_topic = str(
            self.get_parameter('manipulation_status_topic').value)
        self.task_name = str(self.get_parameter('task_name').value)
        self.receiver_message = str(
            self.get_parameter('receiver_message').value)
        self.succeeded_status = int(
            self.get_parameter('succeeded_status').value)
        self.trigger_on_nav2_success = bool(
            self.get_parameter('trigger_on_nav2_success').value)
        self.trigger_on_door_arrival = bool(
            self.get_parameter('trigger_on_door_arrival').value)
        self.door_arrival_topic = str(
            self.get_parameter('door_arrival_topic').value)
        self.publish_string = bool(self.get_parameter('publish_string').value)
        self.publish_bool = bool(self.get_parameter('publish_bool').value)
        self.publish_receiver_message = bool(
            self.get_parameter('publish_receiver_message').value)
        self.progress_publish_rate_hz = max(
            float(self.get_parameter('progress_publish_rate_hz').value), 0.1)

        self.trigger_pub = None
        self.trigger_bool_pub = None
        self.receiver_pub = None
        self.progress = 'navigation'
        self.last_event = 'navigation_running'
        self.last_goal_id = ''
        self.request_id = ''
        self.request_counter = 0
        self.arrived_at_door = False
        self.last_door_arrival_state = False
        self.manipulation_status = 'idle'
        self.extra_progress: dict[str, Any] = {}

        self.progress_pub = self.create_publisher(
            String, self.progress_topic, 10)
        if self.publish_string:
            self.trigger_pub = self.create_publisher(
                String, self.trigger_topic, 10)
        if self.publish_bool:
            self.trigger_bool_pub = self.create_publisher(
                Bool, self.trigger_bool_topic, 10)
        if self.publish_receiver_message:
            self.receiver_pub = self.create_publisher(
                String, self.receiver_topic, 10)

        self.triggered_goal_ids = set()
        if self.trigger_on_nav2_success:
            self.create_subscription(
                GoalStatusArray, self.status_topic, self.status_callback, 10)
        if self.trigger_on_door_arrival:
            self.create_subscription(
                Bool, self.door_arrival_topic,
                self.door_arrival_callback, 10)
        self.create_subscription(
            String, self.manipulation_status_topic,
            self.manipulation_status_callback, 10)
        self.create_timer(
            1.0 / self.progress_publish_rate_hz,
            self.publish_progress)

        watch_modes = []
        if self.trigger_on_nav2_success:
            watch_modes.append(
                f'{self.status_topic} Nav2 {status_name(self.succeeded_status)}')
        if self.trigger_on_door_arrival:
            watch_modes.append(
                f'{self.door_arrival_topic} std_msgs/Bool rising edge true')
        if not watch_modes:
            watch_modes.append('nothing; all trigger inputs disabled')
        self.get_logger().info(
            'Watching ' + '; '.join(watch_modes)
            + f'; publishing continuous progress on {self.progress_topic}.')

    @staticmethod
    def goal_id_hex(status: GoalStatus) -> str:
        return ''.join(
            f'{int(value) & 0xff:02x}'
            for value in status.goal_info.goal_id.uuid
        )

    def status_callback(self, msg: GoalStatusArray) -> None:
        for goal_status in msg.status_list:
            goal_id = self.goal_id_hex(goal_status)
            if goal_status.status != self.succeeded_status:
                continue
            if goal_id in self.triggered_goal_ids:
                continue

            self.triggered_goal_ids.add(goal_id)
            self.publish_trigger(goal_id, goal_status.status, 'nav2_arrived')

    def door_arrival_callback(self, msg: Bool) -> None:
        arrived = bool(msg.data)
        if not arrived:
            self.last_door_arrival_state = False
            self.arrived_at_door = False
            return
        if self.last_door_arrival_state:
            return
        self.last_door_arrival_state = True
        self.publish_trigger(
            'door', self.succeeded_status, 'door_landmark_arrived',
            extra={
                'door_arrival_topic': self.door_arrival_topic,
                'door_arrived': True,
            })

    def manipulation_status_callback(self, msg: String) -> None:
        text = msg.data.strip()
        normalized = text.lower()
        if normalized in ('running', 'started', 'manipulation_running'):
            self.progress = 'manipulation_running'
            self.manipulation_status = 'running'
        elif normalized in ('done', 'finished', 'success', 'succeeded', 'manipulation_succeeded'):
            self.progress = 'manipulation_succeeded'
            self.manipulation_status = 'succeeded'
        elif normalized in ('failed', 'error', 'aborted', 'manipulation_failed'):
            self.progress = 'manipulation_failed'
            self.manipulation_status = 'failed'
        else:
            self.manipulation_status = text
            self.extra_progress['manipulation_message'] = text
        self.last_event = 'manipulation_status_update'
        self.publish_progress()

    def publish_trigger(
        self,
        goal_id: str,
        status: int,
        event: str,
        extra: dict | None = None,
    ) -> None:
        stamp = self.get_clock().now().to_msg()
        payload = {
            'event': event,
            'task': self.task_name,
            'goal_id': goal_id,
            'status': int(status),
            'status_name': status_name(status),
            'stamp': {
                'sec': int(stamp.sec),
                'nanosec': int(stamp.nanosec),
            },
            'source_node': self.get_name(),
        }
        if extra:
            payload.update(extra)

        self.last_event = event
        self.last_goal_id = goal_id
        self.request_counter += 1
        self.request_id = f'{self.task_name}:{goal_id}:{self.request_counter}'
        if event == 'door_landmark_arrived':
            self.arrived_at_door = True
            self.progress = 'manipulation_requested'
        else:
            self.progress = 'manipulation_requested'
        self.extra_progress = dict(extra or {})

        if self.trigger_pub is not None:
            out = String()
            out.data = json.dumps(payload, separators=(',', ':'))
            self.trigger_pub.publish(out)

        if self.trigger_bool_pub is not None:
            out_bool = Bool()
            out_bool.data = True
            self.trigger_bool_pub.publish(out_bool)

        if self.receiver_pub is not None:
            receiver_msg = String()
            receiver_msg.data = self.receiver_message
            self.receiver_pub.publish(receiver_msg)

        self.get_logger().info(
            f'{event} for {goal_id}; manipulation trigger published.')

        self.publish_progress()

    def publish_progress(self) -> None:
        stamp = self.get_clock().now().to_msg()
        payload = {
            'progress': self.progress,
            'event': self.last_event,
            'task': self.task_name,
            'request_id': self.request_id,
            'goal_id': self.last_goal_id,
            'arrived_at_door': self.arrived_at_door,
            'manipulation_status': self.manipulation_status,
            'stamp': {
                'sec': int(stamp.sec),
                'nanosec': int(stamp.nanosec),
            },
            'source_node': self.get_name(),
        }
        payload.update(self.extra_progress)
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'))
        self.progress_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2ArrivalManipulationTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
