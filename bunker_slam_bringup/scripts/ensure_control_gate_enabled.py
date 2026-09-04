#!/usr/bin/env python3
"""Wait for one PiPER SetBool control gate and explicitly enable it."""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class ControlGateEnabler(Node):
    def __init__(self):
        super().__init__('control_gate_enabler')
        self.declare_parameter('service_name', '/front_piper/control_enable')
        self.declare_parameter('wait_timeout_sec', 45.0)
        self.service_name = str(self.get_parameter('service_name').value)
        self.wait_timeout_sec = float(
            self.get_parameter('wait_timeout_sec').value)
        self.started = self.get_clock().now()
        self.request_in_flight = False
        self.exit_code = 1
        self.client = self.create_client(SetBool, self.service_name)
        self.timer = self.create_timer(0.25, self._tick)
        self.get_logger().info(
            f'Waiting to enable PiPER control gate {self.service_name}')

    def _tick(self):
        elapsed = (self.get_clock().now() - self.started).nanoseconds / 1e9
        if elapsed >= self.wait_timeout_sec:
            self.get_logger().error(
                f'Timed out waiting for {self.service_name}; gate was not enabled')
            rclpy.shutdown()
            return
        if self.request_in_flight or not self.client.service_is_ready():
            return
        request = SetBool.Request()
        request.data = True
        self.request_in_flight = True
        future = self.client.call_async(request)
        future.add_done_callback(self._completed)

    def _completed(self, future):
        self.request_in_flight = False
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().warning(
                f'Control gate request failed; retrying: {error}')
            return
        if not response.success:
            self.get_logger().warning(
                f'Control gate rejected request; retrying: {response.message}')
            return
        self.exit_code = 0
        self.get_logger().info(
            f'{self.service_name} accepted SetBool data=true: {response.message}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ControlGateEnabler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
