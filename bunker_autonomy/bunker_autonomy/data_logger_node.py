#!/usr/bin/env python3

import json
import os
from datetime import datetime
from typing import Any, Dict, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

from bunker_autonomy.label_autonomy_run import parse_operator_label


def twist_to_dict(msg: Twist) -> Dict[str, Dict[str, float]]:
    return {
        'linear': {
            'x': msg.linear.x,
            'y': msg.linear.y,
            'z': msg.linear.z,
        },
        'angular': {
            'x': msg.angular.x,
            'y': msg.angular.y,
            'z': msg.angular.z,
        },
    }


def parse_safety_reason(text: str) -> Dict[str, Any]:
    try:
        value = json.loads(text or '{}')
    except json.JSONDecodeError:
        return {
            'stopped': True,
            'primary_reason': 'invalid_reason_message',
            'reasons': ['invalid_reason_message'],
            'raw': text,
        }
    return value if isinstance(value, dict) else {}


def parse_json_topic(text: str) -> Dict[str, Any]:
    """Decode a JSON String topic while preserving malformed payloads."""
    try:
        value = json.loads(text or '{}')
    except json.JSONDecodeError:
        return {
            'parse_ok': False,
            'raw': text,
        }
    if not isinstance(value, dict):
        return {
            'parse_ok': False,
            'raw': text,
        }
    return {
        'parse_ok': True,
        'data': value,
    }


def safety_reason_signature(reason: Dict[str, Any]):
    reasons = reason.get('reasons', [])
    if not isinstance(reasons, list):
        reasons = [str(reasons)]
    return (
        bool(reason.get('stopped', True)),
        str(reason.get('primary_reason', 'unknown')),
        tuple(str(item) for item in reasons),
    )


class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger_node')

        self.declare_parameter('log_directory', '~/vlm_results/autonomy_logs')
        self.declare_parameter('log_prefix', 'trash_mission')
        self.declare_parameter('safety_event_prefix', 'safety_events')
        self.declare_parameter('perception_trace_prefix', 'perception_trace')
        self.declare_parameter('sample_rate_hz', 1.0)
        self.declare_parameter('vlm_result_topic', '/vlm_result')
        self.declare_parameter('target_status_topic', '/target_status')
        self.declare_parameter('target_detected_topic', '/target_detected')
        self.declare_parameter('safety_stop_topic', '/safety_stop')
        self.declare_parameter('route_status_topic', '/route_status')
        self.declare_parameter('safety_stop_reason_topic', '/safety_stop_reason')
        self.declare_parameter('cmd_vel_autonomy_topic', '/cmd_vel_autonomy')
        self.declare_parameter('output_cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('mission_complete_topic', '/mission_complete')
        self.declare_parameter('mission_status_topic', '/mission_status')
        self.declare_parameter('autonomy_notice_topic', '/autonomy_notice')
        self.declare_parameter('operator_label_topic', '/operator_label')
        self.declare_parameter('lidar_safety_stop_topic', '/lidar_safety_stop')
        self.declare_parameter('depth_safety_stop_topic', '/depth_safety_stop')
        self.declare_parameter('lidar_route_status_topic', '/lidar_route_status')
        self.declare_parameter('depth_route_status_topic', '/depth_route_status')

        log_directory = os.path.expanduser(str(self.get_parameter('log_directory').value))
        log_prefix = str(self.get_parameter('log_prefix').value)
        safety_event_prefix = str(self.get_parameter('safety_event_prefix').value)
        perception_trace_prefix = str(
            self.get_parameter('perception_trace_prefix').value
        )
        sample_rate_hz = max(0.1, float(self.get_parameter('sample_rate_hz').value))

        os.makedirs(log_directory, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._log_path = os.path.join(log_directory, '%s_%s.jsonl' % (log_prefix, timestamp))
        self._safety_event_path = os.path.join(
            log_directory,
            '%s_%s.jsonl' % (safety_event_prefix, timestamp),
        )
        self._perception_trace_path = os.path.join(
            log_directory,
            '%s_%s.jsonl' % (perception_trace_prefix, timestamp),
        )
        self._log_file = open(self._log_path, 'a', encoding='utf-8')
        self._safety_event_file = open(
            self._safety_event_path,
            'a',
            encoding='utf-8',
        )
        self._perception_trace_file = open(
            self._perception_trace_path,
            'a',
            encoding='utf-8',
        )

        self._latest: Dict[str, Any] = {}
        self._last_safety_signature = None
        self._perception_cycle_id = 0
        self._active_perception_cycle: Optional[Dict[str, Any]] = None
        self._pending_depth_cycle_id: Optional[int] = None
        self._pending_fusion_cycle_id: Optional[int] = None
        self._pending_behavior_cycle_id: Optional[int] = None
        self._pending_output_cycle_id: Optional[int] = None
        self.create_subscription(
            String,
            str(self.get_parameter('vlm_result_topic').value),
            self.vlm_result_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('target_status_topic').value),
            self.target_status_callback,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('target_detected_topic').value),
            lambda msg: self._store_bool('target_detected', msg),
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('safety_stop_topic').value),
            lambda msg: self._store_bool('safety_stop', msg),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('route_status_topic').value),
            self.route_status_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('safety_stop_reason_topic').value),
            self.safety_reason_callback,
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('cmd_vel_autonomy_topic').value),
            self.cmd_vel_autonomy_callback,
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('output_cmd_vel_topic').value),
            self.output_cmd_vel_callback,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('mission_complete_topic').value),
            lambda msg: self._store_bool('mission_complete', msg),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('mission_status_topic').value),
            lambda msg: self._store_string('mission_status', msg),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('autonomy_notice_topic').value),
            lambda msg: self._store_string('autonomy_notice', msg),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('operator_label_topic').value),
            self.operator_label_callback,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('lidar_safety_stop_topic').value),
            lambda msg: self._store_bool('lidar_safety_stop', msg),
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('depth_safety_stop_topic').value),
            lambda msg: self._store_bool('depth_safety_stop', msg),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('lidar_route_status_topic').value),
            lambda msg: self._store_string('lidar_route_status', msg),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('depth_route_status_topic').value),
            self.depth_route_status_callback,
            10,
        )
        self.create_timer(1.0 / sample_rate_hz, self.write_sample)

        self.get_logger().info(
            'Data logger writing samples=%s safety_events=%s perception_trace=%s'
            % (
                self._log_path,
                self._safety_event_path,
                self._perception_trace_path,
            )
        )
        self._write_perception_trace(
            {
                **self._now(),
                'event': 'perception_trace_started',
                'description': (
                    'One row per VLM cycle: VLM JSON -> parsed target/bbox -> '
                    'depth distance/sectors -> lidar+depth fusion -> behavior output.'
                ),
                'topics': {
                    'vlm_output': str(self.get_parameter('vlm_result_topic').value),
                    'parsed_target': str(
                        self.get_parameter('target_status_topic').value
                    ),
                    'depth_measurement': str(
                        self.get_parameter('depth_route_status_topic').value
                    ),
                    'lidar_measurement': str(
                        self.get_parameter('lidar_route_status_topic').value
                    ),
                    'fused_routes': str(self.get_parameter('route_status_topic').value),
                    'behavior_command': str(
                        self.get_parameter('cmd_vel_autonomy_topic').value
                    ),
                    'final_command': str(
                        self.get_parameter('output_cmd_vel_topic').value
                    ),
                },
                'note': (
                    'Depth images are not copied into this JSONL. target_distance_m '
                    'is computed from depth pixels associated with target_bbox_norm.'
                ),
            }
        )

    def _now(self) -> Dict[str, Any]:
        now = self.get_clock().now()
        return {
            'ros_time_sec': now.nanoseconds / 1e9,
            'wall_time': datetime.now().isoformat(timespec='milliseconds'),
        }

    def _store(self, key: str, value: Any) -> None:
        self._latest[key] = {
            **self._now(),
            'value': value,
        }

    def _store_string(self, key: str, msg: String) -> None:
        self._store(key, msg.data)

    def _store_bool(self, key: str, msg: Bool) -> None:
        self._store(key, bool(msg.data))

    def _store_twist(self, key: str, msg: Twist) -> None:
        self._store(key, twist_to_dict(msg))

    def _write_row(self, row: Dict[str, Any]) -> None:
        self._log_file.write(json.dumps(row, sort_keys=True) + '\n')
        self._log_file.flush()

    def _write_safety_event(self, row: Dict[str, Any]) -> None:
        serialized = json.dumps(row, sort_keys=True) + '\n'
        self._log_file.write(serialized)
        self._log_file.flush()
        self._safety_event_file.write(serialized)
        self._safety_event_file.flush()

    def _write_perception_trace(self, row: Dict[str, Any]) -> None:
        serialized = json.dumps(row, sort_keys=True) + '\n'
        self._perception_trace_file.write(serialized)
        self._perception_trace_file.flush()

    def vlm_result_callback(self, msg: String) -> None:
        self._store_string('vlm_result', msg)
        self._perception_cycle_id += 1
        self._active_perception_cycle = {
            'cycle_id': self._perception_cycle_id,
            'vlm_received': {
                **self._now(),
                'raw': msg.data,
                'decoded': parse_json_topic(msg.data),
            },
        }
        self._pending_depth_cycle_id = None
        self._pending_fusion_cycle_id = None
        self._pending_behavior_cycle_id = None
        self._pending_output_cycle_id = None

    def target_status_callback(self, msg: String) -> None:
        self._store_string('target_status', msg)
        if self._active_perception_cycle is None:
            return
        self._active_perception_cycle['target_status'] = {
            **self._now(),
            'decoded': parse_json_topic(msg.data),
        }
        self._pending_depth_cycle_id = int(
            self._active_perception_cycle['cycle_id']
        )

    def depth_route_status_callback(self, msg: String) -> None:
        self._store_string('depth_route_status', msg)
        cycle = self._active_perception_cycle
        if (
            cycle is None
            or self._pending_depth_cycle_id != cycle.get('cycle_id')
        ):
            return
        cycle['depth_association'] = {
            **self._now(),
            'decoded': parse_json_topic(msg.data),
            'explanation': (
                'target_distance_m comes from valid depth pixels inside the '
                'VLM target_bbox_norm; clearance_m/clear describe image sectors.'
            ),
        }
        lidar = self._latest.get('lidar_route_status')
        if lidar is not None:
            cycle['latest_lidar_status'] = {
                **lidar,
                'decoded': parse_json_topic(str(lidar.get('value', ''))),
            }
        self._pending_depth_cycle_id = None
        self._pending_fusion_cycle_id = int(cycle['cycle_id'])

    def route_status_callback(self, msg: String) -> None:
        self._store_string('route_status', msg)
        cycle = self._active_perception_cycle
        if (
            cycle is None
            or self._pending_fusion_cycle_id != cycle.get('cycle_id')
        ):
            return
        cycle['fused_route_status'] = {
            **self._now(),
            'decoded': parse_json_topic(msg.data),
        }
        self._pending_fusion_cycle_id = None
        self._pending_behavior_cycle_id = int(cycle['cycle_id'])

    def cmd_vel_autonomy_callback(self, msg: Twist) -> None:
        self._store_twist('cmd_vel_autonomy', msg)
        cycle = self._active_perception_cycle
        if (
            cycle is None
            or self._pending_behavior_cycle_id != cycle.get('cycle_id')
        ):
            return
        cycle['behavior_command'] = {
            **self._now(),
            'value': twist_to_dict(msg),
            'mission_status': self._latest.get('mission_status'),
        }
        self._pending_behavior_cycle_id = None
        self._pending_output_cycle_id = int(cycle['cycle_id'])

    def output_cmd_vel_callback(self, msg: Twist) -> None:
        self._store_twist('cmd_vel_output', msg)
        cycle = self._active_perception_cycle
        if (
            cycle is None
            or self._pending_output_cycle_id != cycle.get('cycle_id')
        ):
            return
        cycle['final_command'] = {
            **self._now(),
            'value': twist_to_dict(msg),
            'explanation': (
                'Final command after the safety mux, speed limits, watchdog, and ramp.'
            ),
        }
        cycle['safety_snapshot'] = {
            key: self._latest[key]
            for key in ('safety_stop', 'safety_stop_reason')
            if key in self._latest
        }
        row = {
            **self._now(),
            'event': 'perception_cycle',
            **cycle,
        }
        self._write_row(row)
        self._write_perception_trace(row)
        self._pending_output_cycle_id = None

    def write_sample(self) -> None:
        row = {
            **self._now(),
            'topics': self._latest,
        }
        self._write_row(row)

    def operator_label_callback(self, msg: String) -> None:
        label = parse_operator_label(msg.data, source='live_topic')
        self._store('operator_label', label)
        self._write_row(
            {
                **self._now(),
                'event': 'operator_label',
                'operator_label': label,
                'topics': self._latest,
            }
        )

    def safety_reason_callback(self, msg: String) -> None:
        reason = parse_safety_reason(msg.data)
        self._store('safety_stop_reason', reason)
        signature = safety_reason_signature(reason)
        if signature == self._last_safety_signature:
            return

        row = {
            **self._now(),
            'event': 'safety_transition',
            'stopped': signature[0],
            'primary_reason': signature[1],
            'reasons': list(signature[2]),
            'reason_detail': reason,
            'topics': self._latest,
        }
        self._write_safety_event(row)
        self._last_safety_signature = signature

    def destroy_node(self):
        try:
            self._log_file.close()
            self._safety_event_file.close()
            self._perception_trace_file.close()
        finally:
            return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
