"""Preset runner for a PiPER-X arm.

Edit config/piper_x_joint_preset.yaml when you want to test a different
joint pose.

Safety behavior:
  * no command is sent unless trigger_on_start:=true and allow_motion:=true
  * feedback must be received first, so unchanged joints keep their live values
  * every command publishes all six arm joints because agx_arm_ctrl fills
    missing joints with zero
  * the /control_enable gate is closed again when the preset finishes/fails
"""

from math import isfinite
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
import yaml


JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

FALLBACK_PRESET_STEPS = [
    {'joint': 'joint1', 'position': 0.05, 'hold_sec': 1.0},
    {'joint': 'joint1', 'position': 0.00, 'hold_sec': 1.0},
]


class PiperXJointPreset(Node):
    def __init__(self):
        super().__init__('piper_x_joint_preset')
        defaults = {
            'joint_names': JOINT_NAMES,
            'feedback_topic': '/piper_x/feedback/joint_states',
            'command_topic': '/piper_x/control/joint_states',
            'arm_enable_service': '/piper_x/enable_agx_arm',
            'control_gate_service': '/piper_x/control_enable',
            'preset_file': '',
            'position_tolerance_rad': 0.035,
            'motion_timeout_sec': 20.0,
            'startup_timeout_sec': 20.0,
            'hold_default_sec': 1.0,
            'trigger_on_start': False,
            'allow_motion': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.names = list(self.get_parameter('joint_names').value)
        if len(self.names) != 6:
            raise ValueError('PiPER-X preset runner expects exactly six arm joints')
        self.name_to_index = {name: index for index, name in enumerate(self.names)}
        self.steps = self._load_preset_steps()
        self.tolerance = float(self.get_parameter('position_tolerance_rad').value)
        self.motion_timeout = float(self.get_parameter('motion_timeout_sec').value)
        self.startup_timeout = float(self.get_parameter('startup_timeout_sec').value)
        self.hold_default = float(self.get_parameter('hold_default_sec').value)

        self.feedback = None
        self.target = None
        self.step_index = -1
        self.phase = 'waiting'
        self.startup_time = self.get_clock().now()
        self.step_start_time = None
        self.hold_start_time = None
        self.finishing = False
        self.finish_time = None
        self.succeeded = False
        self.exit_code = 1

        self.command_pub = self.create_publisher(
            JointState, str(self.get_parameter('command_topic').value), 1)
        self.create_subscription(
            JointState, str(self.get_parameter('feedback_topic').value), self._feedback, 10)
        self.enable_client = self.create_client(
            SetBool, str(self.get_parameter('arm_enable_service').value))
        self.gate_client = self.create_client(
            SetBool, str(self.get_parameter('control_gate_service').value))
        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            'PiPER-X preset runner loaded. Edit config/piper_x_joint_preset.yaml '
            'to change the test sequence. '
            'No motion is sent unless trigger_on_start and allow_motion are both true.')

    def _load_preset_steps(self):
        preset_file = str(self.get_parameter('preset_file').value).strip()
        if preset_file:
            path = Path(preset_file).expanduser()
            with path.open('r', encoding='utf-8') as stream:
                document = yaml.safe_load(stream) or {}
            raw_steps = self._raw_steps_from_document(document, path)
            self.get_logger().info(f'Loading PiPER-X preset steps from {path}')
        else:
            raw_steps = FALLBACK_PRESET_STEPS
            self.get_logger().warning(
                'No preset_file parameter set; using built-in fallback preset')

        steps = []
        for raw in raw_steps:
            if not isinstance(raw, dict):
                raise ValueError(f'Invalid PiPER-X preset step: {raw}')
            hold_sec = float(raw.get('hold_sec', 1.0))
            if not isfinite(hold_sec) or hold_sec < 0.0:
                raise ValueError(f'Invalid PiPER-X preset step: {raw}')

            if 'positions' in raw:
                positions = [float(value) for value in raw.get('positions')]
                if len(positions) != len(self.names) or not all(isfinite(value) for value in positions):
                    raise ValueError(
                        f'Invalid PiPER-X preset positions. Expected {len(self.names)} '
                        f'finite values for {self.names}: {raw}')
                steps.append({
                    'label': str(raw.get('label', 'pose')),
                    'positions': positions,
                    'hold_sec': hold_sec,
                })
                continue

            joint = str(raw.get('joint', '')).strip()
            if joint not in self.name_to_index:
                raise ValueError(
                    f"Invalid PiPER-X preset joint '{joint}'. Expected one of {self.names}")
            position = float(raw.get('position'))
            if not isfinite(position):
                raise ValueError(f'Invalid PiPER-X preset step: {raw}')
            steps.append({'joint': joint, 'position': position, 'hold_sec': hold_sec})
        if not steps:
            raise ValueError('PiPER-X preset_steps cannot be empty')
        return steps

    def _raw_steps_from_document(self, document, path):
        if 'rear' in document:
            rear = document['rear']
            if not isinstance(rear, list):
                raise ValueError(f'{path}: rear must be a six-value list')
            return [{
                'label': 'rear',
                'positions': rear,
                'hold_sec': document.get('hold_sec', 1.0),
            }]

        raw_steps = document.get('preset_steps', [])
        if not isinstance(raw_steps, list):
            raise ValueError(f'{path}: preset_steps must be a list')
        return raw_steps

    def _feedback(self, msg):
        self.feedback = dict(zip(msg.name, msg.position))

    def _ready_for_motion(self):
        if self.feedback is None:
            return False
        return all(name in self.feedback for name in self.names)

    def _call_set_bool(self, client, value, label, callback):
        if not client.service_is_ready():
            self.get_logger().warning(
                f'{label} service is not ready; waiting without sending arm commands',
                throttle_duration_sec=2.0)
            return False
        request = SetBool.Request()
        request.data = bool(value)
        future = client.call_async(request)
        future.add_done_callback(callback)
        return True

    def _tick(self):
        if self.finishing:
            elapsed = (self.get_clock().now() - self.finish_time).nanoseconds / 1e9
            if elapsed >= 2.0 and rclpy.ok():
                self.get_logger().warning(
                    'Timed out while closing PiPER-X control gate; shutting down')
                rclpy.shutdown()
            return

        if self.phase == 'moving':
            self._monitor_step()
            return
        if self.phase == 'holding':
            self._monitor_hold()
            return
        if self.phase != 'waiting':
            return

        if not bool(self.get_parameter('trigger_on_start').value):
            return
        if not bool(self.get_parameter('allow_motion').value):
            self.get_logger().error(
                'trigger_on_start requested but allow_motion is false; no command sent')
            self.phase = 'blocked'
            rclpy.shutdown()
            return

        startup_elapsed = (self.get_clock().now() - self.startup_time).nanoseconds / 1e9
        if startup_elapsed >= self.startup_timeout:
            self.get_logger().error(
                f'PiPER-X initialization aborted after {startup_elapsed:.1f} s: '
                'missing complete feedback/services; no command sent')
            self.phase = 'blocked'
            rclpy.shutdown()
            return

        if not self._ready_for_motion():
            return
        if not self.enable_client.service_is_ready() or not self.gate_client.service_is_ready():
            self.get_logger().warning(
                'Waiting for PiPER-X enable and control-gate services; no command sent',
                throttle_duration_sec=2.0)
            return

        self.phase = 'enabling'
        self._call_set_bool(self.enable_client, True, 'arm enable', self._enabled)

    def _enabled(self, future):
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f'PiPER-X arm enable request failed: {error}')
            self._finish(False)
            return
        if not response.success:
            self.get_logger().error(f'PiPER-X arm enable rejected: {response.message}')
            self._finish(False)
            return
        self.phase = 'opening_gate'
        self._call_set_bool(self.gate_client, True, 'control gate', self._gate_opened)

    def _gate_opened(self, future):
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f'PiPER-X control-gate request failed: {error}')
            self._finish(False)
            return
        if not response.success:
            self.get_logger().error(f'PiPER-X control gate rejected: {response.message}')
            self._finish(False)
            return

        self.target = [float(self.feedback[name]) for name in self.names]
        self.get_logger().warning(
            'PiPER-X control gate opened. Starting preset sequence.')
        self._send_next_step()

    def _send_next_step(self):
        self.step_index += 1
        if self.step_index >= len(self.steps):
            self.get_logger().info('PiPER-X preset sequence finished')
            self._finish(True)
            return

        step = self.steps[self.step_index]
        if 'positions' in step:
            self.target = list(step['positions'])
        else:
            joint = step['joint']
            index = self.name_to_index[joint]
            self.target[index] = step['position']

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.names)
        msg.position = list(self.target)
        self.command_pub.publish(msg)

        self.step_start_time = self.get_clock().now()
        self.phase = 'moving'
        if 'positions' in step:
            target_text = ', '.join(f'{name}={value:.4f}' for name, value in zip(self.names, self.target))
            self.get_logger().warning(
                f"PiPER-X step {self.step_index + 1}/{len(self.steps)}: "
                f"{step['label']} -> [{target_text}]")
        else:
            self.get_logger().warning(
                f"PiPER-X step {self.step_index + 1}/{len(self.steps)}: "
                f"{step['joint']} -> {step['position']:.4f} rad")

    def _monitor_step(self):
        step = self.steps[self.step_index]
        if 'positions' in step:
            if self.feedback:
                errors = [
                    abs(float(self.feedback[name]) - target)
                    for name, target in zip(self.names, step['positions'])
                    if name in self.feedback
                ]
            else:
                errors = []
            if len(errors) == len(self.names) and max(errors) <= self.tolerance:
                self.hold_start_time = self.get_clock().now()
                self.phase = 'holding'
                self.get_logger().info(
                    f"{step['label']} pose reached within {self.tolerance:.3f} rad; holding")
                return
        else:
            joint = step['joint']
            current = self.feedback.get(joint) if self.feedback else None
            if current is not None and abs(current - step['position']) <= self.tolerance:
                self.hold_start_time = self.get_clock().now()
                self.phase = 'holding'
                self.get_logger().info(
                    f"{joint} reached target within {self.tolerance:.3f} rad; holding")
                return

        elapsed = (self.get_clock().now() - self.step_start_time).nanoseconds / 1e9
        if elapsed >= self.motion_timeout:
            if 'positions' in step:
                self.get_logger().error(
                    f"PiPER-X step timed out after {elapsed:.1f} s: "
                    f"{step['label']} target={step['positions']}, feedback={self.feedback}")
            else:
                joint = step['joint']
                current = self.feedback.get(joint) if self.feedback else None
                self.get_logger().error(
                    f"PiPER-X step timed out after {elapsed:.1f} s: "
                    f"{joint} target={step['position']:.4f}, current={current}")
            self._finish(False)

    def _monitor_hold(self):
        step = self.steps[self.step_index]
        hold_sec = step.get('hold_sec', self.hold_default)
        elapsed = (self.get_clock().now() - self.hold_start_time).nanoseconds / 1e9
        if elapsed >= hold_sec:
            self._send_next_step()

    def _finish(self, success):
        if self.finishing:
            return
        self.finishing = True
        self.succeeded = bool(success)
        self.exit_code = 0 if success else 1
        self.phase = 'finished' if success else 'failed'
        self.finish_time = self.get_clock().now()
        if self.gate_client.service_is_ready():
            self._call_set_bool(self.gate_client, False, 'control gate', self._gate_closed)
        elif rclpy.ok():
            self.get_logger().warning('PiPER-X control-gate service unavailable while finishing')
            rclpy.shutdown()

    def _gate_closed(self, future):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(
                    f'PiPER-X control gate did not close cleanly: {response.message}')
                self.exit_code = 1
        except Exception as error:
            self.get_logger().error(f'PiPER-X control-gate close request failed: {error}')
            self.exit_code = 1
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = PiperXJointPreset()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if not node.succeeded:
            node.exit_code = 1
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)
