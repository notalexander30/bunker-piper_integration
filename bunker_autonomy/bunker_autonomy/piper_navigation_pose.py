"""Move PiPER to one reviewed navigation pose, with a closed-by-default gate."""

from math import isfinite

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool


class PiperNavigationPose(Node):
    def __init__(self):
        super().__init__('piper_navigation_pose')
        defaults = {
            'joint_names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
            'joint_positions': [0.0] * 6,
            'feedback_topic': '/piper/feedback/joint_states',
            'command_topic': '/piper/control/joint_states',
            'arm_enable_service': '/piper/enable_agx_arm',
            'control_gate_service': '/piper/control_enable',
            'position_tolerance_rad': 0.035,
            'motion_timeout_sec': 30.0,
            'startup_timeout_sec': 20.0,
            'trigger_on_start': False,
            'allow_motion': False,
            'close_control_gate_on_finish': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.names = list(self.get_parameter('joint_names').value)
        self.targets = [float(v) for v in self.get_parameter('joint_positions').value]
        valid_targets = all(isfinite(value) for value in self.targets)
        if len(self.names) != 6 or len(self.targets) != 6 or not valid_targets:
            raise ValueError('PiPER navigation pose must contain six finite joint positions')
        self.tolerance = float(self.get_parameter('position_tolerance_rad').value)
        self.timeout = float(self.get_parameter('motion_timeout_sec').value)
        self.startup_timeout = float(
            self.get_parameter('startup_timeout_sec').value)
        self.startup_time = self.get_clock().now()
        self.feedback = None
        self.phase = 'waiting'
        self.command_sent = False
        self.finishing = False
        self.succeeded = False
        self.exit_code = 1
        self.finish_time = None
        self.start_time = None
        self.command_pub = self.create_publisher(
            JointState, str(self.get_parameter('command_topic').value), 1)
        self.create_subscription(
            JointState, str(self.get_parameter('feedback_topic').value), self._feedback, 10)
        self.enable_client = self.create_client(
            SetBool, str(self.get_parameter('arm_enable_service').value))
        self.gate_client = self.create_client(
            SetBool, str(self.get_parameter('control_gate_service').value))
        self.create_timer(0.2, self._tick)
        self.get_logger().info(
            'PiPER pose manager is closed by default. Set trigger_on_start:=true '
            'and allow_motion:=true only after visually reviewing this saved pose.')

    def _feedback(self, msg):
        self.feedback = dict(zip(msg.name, msg.position))

    def _call(self, client, value, label, callback=None):
        if not client.service_is_ready():
            self.get_logger().warning(
                f'{label} service is not ready yet; waiting without sending an arm command',
                throttle_duration_sec=2.0)
            return False
        request = SetBool.Request()
        request.data = value
        future = client.call_async(request)
        if callback is not None:
            future.add_done_callback(callback)
        return True

    def _tick(self):
        if self.finishing:
            elapsed = (
                self.get_clock().now() - self.finish_time).nanoseconds / 1e9
            if elapsed >= 2.0 and rclpy.ok():
                self.get_logger().warning(
                    'Timed out waiting for the control gate to close; shutting down')
                rclpy.shutdown()
            return
        if self.phase == 'moving':
            self._monitor()
            return
        if not bool(self.get_parameter('trigger_on_start').value):
            return
        if not bool(self.get_parameter('allow_motion').value):
            self.get_logger().error(
                'trigger_on_start requested but allow_motion is false; '
                'no command sent')
            self.phase = 'blocked'
            self.exit_code = 1
            rclpy.shutdown()
            return
        startup_elapsed = (
            self.get_clock().now() - self.startup_time).nanoseconds / 1e9
        if self.phase == 'waiting' and startup_elapsed >= self.startup_timeout:
            self.get_logger().error(
                'PiPER initialization aborted: no complete feedback/services '
                f'after {startup_elapsed:.1f} s; no motion command was sent')
            self.phase = 'blocked'
            self.exit_code = 1
            rclpy.shutdown()
            return
        if self.feedback is None or any(name not in self.feedback for name in self.names):
            return
        if self.phase != 'waiting':
            return
        if not self.enable_client.service_is_ready() or not self.gate_client.service_is_ready():
            self.get_logger().warning(
                'Waiting for PiPER enable and control-gate services; no motion command sent',
                throttle_duration_sec=2.0)
            return
        self.start_time = self.get_clock().now()
        self.phase = 'enabling'
        self._call(self.enable_client, True, 'arm enable', self._enabled)

    def _enabled(self, future):
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f'Arm enable request failed: {error}')
            self._finish(False)
            return
        if not response.success:
            self.get_logger().error(f'Arm enable rejected: {response.message}')
            self._finish(False)
            return
        self.phase = 'opening_gate'
        self._call(self.gate_client, True, 'control gate', self._gate_opened)

    def _gate_opened(self, future):
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f'Control-gate request failed: {error}')
            self._finish(False)
            return
        if not response.success:
            self.get_logger().error(f'Control gate rejected: {response.message}')
            self._finish(False)
            return
        message = JointState()
        message.name = self.names
        message.position = self.targets
        self.command_pub.publish(message)
        self.command_sent = True
        self.phase = 'moving'
        self.get_logger().warn(
            'PiPER navigation-pose command sent; monitoring position and '
            'closing gate afterward')

    def _monitor(self):
        if not self.command_sent:
            return
        if self.feedback and all(abs(self.feedback[name] - target) <= self.tolerance
                                 for name, target in zip(self.names, self.targets)):
            self.get_logger().info(
                f'PiPER reached navigation pose within {self.tolerance:.3f} rad')
            self._finish(True)
            return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed >= self.timeout:
            self.get_logger().error(f'PiPER navigation pose timed out after {elapsed:.1f} s')
            self._finish(False)

    def _finish(self, success):
        if self.finishing:
            return
        self.finishing = True
        self.succeeded = bool(success)
        self.exit_code = 0 if success else 1
        self.finish_time = self.get_clock().now()
        self.phase = 'finished' if success else 'failed'
        if not bool(self.get_parameter('close_control_gate_on_finish').value):
            self.get_logger().info(
                'Leaving the PiPER control gate enabled as requested by the '
                'hardware bringup')
            self.command_sent = False
            if rclpy.ok():
                rclpy.shutdown()
            return
        if self.gate_client.service_is_ready():
            request = SetBool.Request()
            request.data = False
            future = self.gate_client.call_async(request)
            future.add_done_callback(self._gate_closed)
        elif rclpy.ok():
            self.get_logger().warning(
                'Control-gate service unavailable while finishing')
            rclpy.shutdown()
        self.command_sent = False

    def _gate_closed(self, future):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(
                    f'Control gate did not close cleanly: {response.message}')
                self.exit_code = 1
        except Exception as error:
            self.get_logger().error(f'Control-gate close request failed: {error}')
            self.exit_code = 1
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = PiperNavigationPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # A normal Ctrl-C is not an application error.
        if not node.succeeded:
            node.exit_code = 1
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)
