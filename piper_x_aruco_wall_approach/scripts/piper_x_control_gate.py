#!/usr/bin/env python3
import rclpy
from action_msgs.msg import GoalStatusArray
from rclpy.node import Node
from std_srvs.srv import SetBool


class PiperXControlGate(Node):
    def __init__(self):
        super().__init__("piper_x_control_gate")
        self.declare_parameter(
            "status_topics",
            [
                "arm_controller/follow_joint_trajectory/_action/status",
                "execute_trajectory/_action/status",
                "move_action/_action/status",
            ],
        )
        self.declare_parameter("gate_service_name", "control_enable")
        self.declare_parameter("idle_close_period_s", 0.5)
        self.declare_parameter("hold_open_after_active_s", 4.0)
        self.declare_parameter("open_on_startup", True)
        self.declare_parameter("close_on_startup", False)
        self.declare_parameter("close_when_idle", False)

        self.status_topics = list(self.get_parameter("status_topics").value)
        self.gate_service_name = str(self.get_parameter("gate_service_name").value)
        idle_close_period_s = float(self.get_parameter("idle_close_period_s").value)
        self.hold_open_after_active_s = max(
            float(self.get_parameter("hold_open_after_active_s").value),
            0.0,
        )
        self.open_on_startup = bool(self.get_parameter("open_on_startup").value)
        self.close_on_startup = bool(self.get_parameter("close_on_startup").value)
        self.close_when_idle = bool(self.get_parameter("close_when_idle").value)

        # ACCEPTED, EXECUTING, CANCELING.
        self._active_states = {1, 2, 3}
        self._topic_active = {topic: False for topic in self.status_topics}
        self._gate_open = None
        self._service_ready_logged = False
        self._hold_open_until = self.get_clock().now()

        self._gate_client = self.create_client(SetBool, self.gate_service_name)
        for topic in self.status_topics:
            self.create_subscription(
                GoalStatusArray,
                topic,
                lambda msg, topic_name=topic: self._status_callback(msg, topic_name),
                10,
            )

        if self.open_on_startup:
            self._startup_timer = self.create_timer(0.2, self._open_on_startup)
        elif self.close_on_startup:
            self._startup_timer = self.create_timer(0.2, self._close_on_startup)
        else:
            self._startup_timer = None
        if self.close_when_idle:
            self.create_timer(max(idle_close_period_s, 0.1), self._close_when_idle)
        self.get_logger().info(
            "PiPER-X control gate watching "
            f"{self.status_topics}; service={self.gate_service_name}; "
            f"open_on_startup={self.open_on_startup}; "
            f"close_on_startup={self.close_on_startup}; close_when_idle={self.close_when_idle}"
        )

    def _open_on_startup(self):
        if self._gate_client.wait_for_service(timeout_sec=0.0):
            if not self._service_ready_logged:
                self.get_logger().info("Control gate service ready; forcing gate open")
                self._service_ready_logged = True
            self._set_gate(True)
            if self._startup_timer is not None:
                self._startup_timer.cancel()

    def _close_on_startup(self):
        if self._gate_client.wait_for_service(timeout_sec=0.0):
            if not self._service_ready_logged:
                self.get_logger().info("Control gate service ready; forcing gate closed")
                self._service_ready_logged = True
            self._set_gate(False)
            if self._startup_timer is not None:
                self._startup_timer.cancel()

    def _close_when_idle(self):
        if not any(self._topic_active.values()) and self.get_clock().now() >= self._hold_open_until:
            self._set_gate(False)

    def _status_callback(self, msg, topic_name):
        self._topic_active[topic_name] = any(
            status.status in self._active_states for status in msg.status_list
        )
        if any(self._topic_active.values()):
            self._hold_open_until = self.get_clock().now() + rclpy.duration.Duration(
                seconds=self.hold_open_after_active_s
            )
            self._set_gate(True)
        else:
            if self.close_when_idle:
                self._close_when_idle()

    def _set_gate(self, open_gate):
        if self._gate_open is open_gate:
            return
        if not self._gate_client.wait_for_service(timeout_sec=0.0):
            return

        request = SetBool.Request()
        request.data = bool(open_gate)
        self._gate_client.call_async(request)
        self._gate_open = open_gate
        state = "opened" if open_gate else "closed"
        self.get_logger().info(f"PiPER-X external control gate {state}")


def main(args=None):
    rclpy.init(args=args)
    node = PiperXControlGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
