"""Merge two unprefixed PiPER JointState streams into the combined URDF names."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStatePrefixer(Node):
    ARM_JOINTS = tuple(f'joint{index}' for index in range(1, 9))

    def __init__(self) -> None:
        super().__init__('dual_piper_joint_state_prefixer')
        self.declare_parameter('front_topic', '/front_piper/feedback/joint_states')
        self.declare_parameter('rear_topic', '/rear_piper/feedback/joint_states')
        self.declare_parameter('output_topic', '/joint_states')
        self.declare_parameter('front_output_topic', '/front_joint_states')
        self.declare_parameter('rear_output_topic', '/rear_joint_states')
        self.declare_parameter('front_prefix', 'front_piper_')
        self.declare_parameter('rear_prefix', 'rear_piper_')
        self.declare_parameter('use_front_feedback', True)
        self.declare_parameter('use_rear_feedback', False)
        self.declare_parameter('front_parked_joint1', -1.6)
        self.declare_parameter('rear_parked_joint1', 1.6)
        self.declare_parameter('fallback_publish_rate', 10.0)
        self.declare_parameter('feedback_timeout', 0.5)

        output = self.get_parameter('output_topic').value
        self.publisher = self.create_publisher(JointState, output, 20)
        self.front_publisher = self.create_publisher(
            JointState, self.get_parameter('front_output_topic').value, 20)
        self.rear_publisher = self.create_publisher(
            JointState, self.get_parameter('rear_output_topic').value, 20)
        self.last_feedback = {}
        self.fallback_active = set()
        self.state = {}
        self.velocity = {}
        self.effort = {}
        self._seed_arm_defaults(
            self.get_parameter('front_prefix').value,
            float(self.get_parameter('front_parked_joint1').value),
        )
        self._seed_arm_defaults(
            self.get_parameter('rear_prefix').value,
            float(self.get_parameter('rear_parked_joint1').value),
        )
        self._joint_state_subscriptions = []
        if self.get_parameter('use_front_feedback').value:
            self._joint_state_subscriptions.append(self.create_subscription(
                JointState,
                self.get_parameter('front_topic').value,
                lambda message: self.publish_prefixed(
                    message, self.get_parameter('front_prefix').value),
                20,
            ))
        if self.get_parameter('use_rear_feedback').value:
            self._joint_state_subscriptions.append(self.create_subscription(
                JointState,
                self.get_parameter('rear_topic').value,
                lambda message: self.publish_prefixed(
                    message, self.get_parameter('rear_prefix').value),
                20,
            ))
        self.get_logger().info(
            'Prefixing front and rear PiPER joint states onto %s' % output
        )
        # Continuously maintain TF for an arm whose driver is absent or stale.
        # Fresh live feedback always replaces this display-only parked pose.
        fallback_rate = float(
            self.get_parameter('fallback_publish_rate').value
        )
        self.default_pose_timer = self.create_timer(
            1.0 / fallback_rate, self.publish_missing_arm_defaults
        )

    def _seed_arm_defaults(self, prefix: str, joint1: float) -> None:
        positions = [joint1] + [0.0] * (len(self.ARM_JOINTS) - 1)
        for name, position in zip(self.ARM_JOINTS, positions):
            joint_name = prefix + name
            self.state[joint_name] = position
            self.velocity[joint_name] = 0.0
            self.effort[joint_name] = 0.0

    def _publish_combined_state(self, stamp=None) -> None:
        result = JointState()
        result.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()

        for prefix_parameter in ('front_prefix', 'rear_prefix'):
            prefix = self.get_parameter(prefix_parameter).value
            for name in self.ARM_JOINTS:
                joint_name = prefix + name
                result.name.append(joint_name)
                result.position.append(float(self.state.get(joint_name, 0.0)))
                result.velocity.append(float(self.velocity.get(joint_name, 0.0)))
                result.effort.append(float(self.effort.get(joint_name, 0.0)))

        self.publisher.publish(result)
        self.front_publisher.publish(self._arm_state_message(
            self.get_parameter('front_prefix').value, result.header.stamp))
        self.rear_publisher.publish(self._arm_state_message(
            self.get_parameter('rear_prefix').value, result.header.stamp))

    def _arm_state_message(self, prefix: str, stamp) -> JointState:
        message = JointState()
        message.header.stamp = stamp
        for name in self.ARM_JOINTS:
            joint_name = prefix + name
            message.name.append(joint_name)
            message.position.append(float(self.state.get(joint_name, 0.0)))
            message.velocity.append(float(self.velocity.get(joint_name, 0.0)))
            message.effort.append(float(self.effort.get(joint_name, 0.0)))
        return message

    def publish_missing_arm_defaults(self) -> None:
        arms = (
            ('front_prefix', 'front_parked_joint1'),
            ('rear_prefix', 'rear_parked_joint1'),
        )
        seeded = []
        now = self.get_clock().now()
        timeout = float(self.get_parameter('feedback_timeout').value)
        for prefix_parameter, joint1_parameter in arms:
            prefix = self.get_parameter(prefix_parameter).value
            feedback_parameter = (
                'use_front_feedback'
                if prefix_parameter == 'front_prefix'
                else 'use_rear_feedback'
            )
            last = self.last_feedback.get(prefix)
            if (
                self.get_parameter(feedback_parameter).value
                and last is not None
                and (now - last).nanoseconds / 1e9 <= timeout
            ):
                self.fallback_active.discard(prefix)
                continue
            self._seed_arm_defaults(
                prefix, float(self.get_parameter(joint1_parameter).value)
            )
            if prefix not in self.fallback_active:
                seeded.append(prefix.rstrip('_'))
                self.fallback_active.add(prefix)

        self._publish_combined_state()
        if seeded:
            offline = (
                str(self.get_parameter('front_topic').value).startswith(
                    '/__unused_')
                and str(self.get_parameter('rear_topic').value).startswith(
                    '/__unused_')
            )
            if offline:
                message = 'Published static initial RViz pose: %s'
            else:
                message = (
                    'Published parked RViz pose for missing feedback: %s'
                )
            self.get_logger().info(message % ', '.join(seeded))

    def publish_prefixed(self, source: JointState, prefix: str) -> None:
        self.last_feedback[prefix] = self.get_clock().now()

        def sample(values, index, default=0.0):
            return values[index] if index < len(values) else default

        for index, raw_name in enumerate(source.name):
            name = raw_name[len(prefix):] if raw_name.startswith(prefix) else raw_name
            if name == 'gripper':
                # Official PiPER feedback reports full jaw opening as `gripper`.
                # The public URDF represents the two fingers independently.
                opening = sample(source.position, index)
                velocity = sample(source.velocity, index)
                effort = sample(source.effort, index)
                left_finger = prefix + 'joint7'
                right_finger = prefix + 'joint8'
                self.state[left_finger] = opening / 2.0
                self.state[right_finger] = -opening / 2.0
                self.velocity[left_finger] = velocity / 2.0
                self.velocity[right_finger] = -velocity / 2.0
                self.effort[left_finger] = effort / 2.0
                self.effort[right_finger] = -effort / 2.0
                continue

            joint_name = prefix + name
            self.state[joint_name] = sample(source.position, index)
            self.velocity[joint_name] = sample(source.velocity, index)
            self.effort[joint_name] = sample(source.effort, index)
        self._publish_combined_state(source.header.stamp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointStatePrefixer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
