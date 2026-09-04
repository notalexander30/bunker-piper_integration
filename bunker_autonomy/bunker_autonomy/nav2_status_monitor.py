#!/usr/bin/env python3
"""Colored terminal dashboard for Bunker Nav2/self-exploration status."""

from dataclasses import dataclass
from typing import Any, Optional

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String

try:
    from bunker_msgs.msg import BunkerStatus
except Exception:  # pragma: no cover - keeps monitor usable without bunker_msgs.
    BunkerStatus = None


RESET = '\033[0m'
BOLD = '\033[1m'
DIM = '\033[2m'
RED = '\033[31m'
GREEN = '\033[32m'
YELLOW = '\033[33m'
BLUE = '\033[34m'
MAGENTA = '\033[35m'
CYAN = '\033[36m'
WHITE = '\033[37m'
CLEAR = '\033[2J\033[H'


@dataclass
class TimedValue:
    value: Any = None
    stamp_sec: Optional[float] = None


def color(text: str, code: str) -> str:
    return f'{code}{text}{RESET}'


def status_name(status: int) -> str:
    return {
        GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
        GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
        GoalStatus.STATUS_EXECUTING: 'EXECUTING',
        GoalStatus.STATUS_CANCELING: 'CANCELING',
        GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
        GoalStatus.STATUS_CANCELED: 'CANCELED',
        GoalStatus.STATUS_ABORTED: 'ABORTED',
    }.get(status, f'UNKNOWN({status})')


def vehicle_state_name(value: int) -> str:
    return {0: 'NORMAL', 1: 'E-STOP', 2: 'EXCEPTION'}.get(value, f'UNKNOWN({value})')


def control_mode_name(value: int) -> str:
    return {0: 'STANDBY', 1: 'CAN', 2: 'UART', 3: 'RC'}.get(value, f'UNKNOWN({value})')


def error_summary(error_code: int) -> str:
    if error_code == 0:
        return 'none'
    flags = [
        (0x0001, 'battery_fault'),
        (0x0002, 'battery_warning'),
        (0x0004, 'rc_signal_loss'),
        (0x0008, 'motor1_comm'),
        (0x0010, 'motor2_comm'),
        (0x0020, 'motor3_comm'),
        (0x0040, 'motor4_comm'),
        (0x0080, 'steer_encoder'),
        (0x0100, 'motor_driver'),
        (0x0200, 'hl_comm'),
    ]
    names = [name for bit, name in flags if error_code & bit]
    return ','.join(names) if names else f'0x{error_code:04x}'


class Nav2StatusMonitor(Node):
    def __init__(self) -> None:
        super().__init__('nav2_status_monitor')
        self.declare_parameter('refresh_hz', 2.0)
        self.declare_parameter('stale_sec', 2.5)

        self.refresh_hz = max(0.2, float(self.get_parameter('refresh_hz').value))
        self.stale_sec = max(0.5, float(self.get_parameter('stale_sec').value))

        self.raw_cmd = TimedValue()
        self.auto_cmd = TimedValue()
        self.out_cmd = TimedValue()
        self.odom = TimedValue()
        self.safety_stop = TimedValue()
        self.safety_reason = TimedValue()
        self.route_status = TimedValue()
        self.depth_route_status = TimedValue()
        self.map = TimedValue()
        self.nav_status = TimedValue()
        self.bunker_status = TimedValue()

        self.create_subscription(Twist, '/nav2/cmd_vel_raw', self.cb_raw_cmd, 10)
        self.create_subscription(Twist, '/cmd_vel_autonomy', self.cb_auto_cmd, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cb_out_cmd, 10)
        self.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        self.create_subscription(Bool, '/safety_stop', self.cb_safety_stop, 10)
        self.create_subscription(String, '/safety_stop_reason', self.cb_safety_reason, 10)
        self.create_subscription(String, '/route_status', self.cb_route_status, 10)
        self.create_subscription(String, '/depth_route_status', self.cb_depth_route_status, 10)
        self.create_subscription(OccupancyGrid, '/map', self.cb_map, 1)
        self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.cb_nav_status,
            10,
        )
        if BunkerStatus is not None:
            self.create_subscription(BunkerStatus, '/bunker_status', self.cb_bunker_status, 10)

        self.create_timer(1.0 / self.refresh_hz, self.render)
        self.get_logger().info('Colored Nav2 status monitor started.')

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def store(self, slot: TimedValue, msg: Any) -> None:
        slot.value = msg
        slot.stamp_sec = self.now_sec()

    def cb_raw_cmd(self, msg): self.store(self.raw_cmd, msg)
    def cb_auto_cmd(self, msg): self.store(self.auto_cmd, msg)
    def cb_out_cmd(self, msg): self.store(self.out_cmd, msg)
    def cb_odom(self, msg): self.store(self.odom, msg)
    def cb_safety_stop(self, msg): self.store(self.safety_stop, msg)
    def cb_safety_reason(self, msg): self.store(self.safety_reason, msg)
    def cb_route_status(self, msg): self.store(self.route_status, msg)
    def cb_depth_route_status(self, msg): self.store(self.depth_route_status, msg)
    def cb_map(self, msg): self.store(self.map, msg)
    def cb_nav_status(self, msg): self.store(self.nav_status, msg)
    def cb_bunker_status(self, msg): self.store(self.bunker_status, msg)

    def age(self, slot: TimedValue) -> Optional[float]:
        if slot.stamp_sec is None:
            return None
        return self.now_sec() - slot.stamp_sec

    def fresh(self, slot: TimedValue) -> bool:
        age = self.age(slot)
        return age is not None and age <= self.stale_sec

    def fresh_label(self, label: str, slot: TimedValue) -> str:
        age = self.age(slot)
        if age is None:
            return color(f'{label}: MISSING', RED)
        if age > self.stale_sec:
            return color(f'{label}: STALE {age:.1f}s', YELLOW)
        return color(f'{label}: OK {age:.1f}s', GREEN)

    def twist_line(self, label: str, slot: TimedValue) -> str:
        if slot.value is None:
            return color(f'{label:<18} missing', RED)
        msg = slot.value
        lx = float(msg.linear.x)
        az = float(msg.angular.z)
        direction = self.motion_label(lx, az)
        return f'{label:<18} x={lx:+.3f} m/s  z={az:+.3f} rad/s  {direction}'

    def motion_label(self, linear_x: float, angular_z: float) -> str:
        moving = abs(linear_x) >= 0.015
        turning = abs(angular_z) >= 0.04
        parts = []
        if moving:
            parts.append('FORWARD' if linear_x > 0.0 else 'REVERSE')
        if turning:
            parts.append('LEFT' if angular_z > 0.0 else 'RIGHT')
        if not parts:
            return color('STOP', YELLOW)
        code = GREEN if linear_x >= 0.0 else MAGENTA
        return color('+'.join(parts), code)

    def nav_goal_line(self) -> str:
        if self.nav_status.value is None:
            return color('Nav2 goal          no action status yet', YELLOW)
        statuses = self.nav_status.value.status_list
        if not statuses:
            return color('Nav2 goal          idle', CYAN)
        last = statuses[-1].status
        name = status_name(last)
        code = {
            'EXECUTING': GREEN,
            'ACCEPTED': BLUE,
            'SUCCEEDED': GREEN,
            'ABORTED': RED,
            'CANCELED': YELLOW,
            'CANCELING': YELLOW,
        }.get(name, WHITE)
        return color(f'Nav2 goal          {name}', code)

    def safety_line(self) -> str:
        if self.safety_stop.value is None:
            return color('Safety             missing /safety_stop', RED)
        stopped = bool(self.safety_stop.value.data)
        reason = ''
        if self.safety_reason.value is not None:
            reason = str(self.safety_reason.value.data)
        if stopped:
            return color(f'Safety             STOP {reason}', RED)
        return color(f'Safety             CLEAR {reason}', GREEN)

    def bunker_line(self) -> str:
        if self.bunker_status.value is None:
            return color('Bunker             missing /bunker_status', RED)
        msg = self.bunker_status.value
        ok = msg.vehicle_state == 0 and msg.control_mode == 1 and msg.error_code == 0
        code = GREEN if ok else RED
        return color(
            'Bunker             '
            f'vehicle={msg.vehicle_state}({vehicle_state_name(msg.vehicle_state)})  '
            f'control={msg.control_mode}({control_mode_name(msg.control_mode)})  '
            f'error=0x{msg.error_code:04x}({error_summary(msg.error_code)})',
            code,
        )

    def odom_line(self) -> str:
        if self.odom.value is None:
            return color('Odom               missing', RED)
        msg = self.odom.value
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        return f'Odom               x={x:+.2f} y={y:+.2f}  vx={vx:+.3f} wz={wz:+.3f}'

    def map_line(self) -> str:
        if self.map.value is None:
            return color('Map                missing', RED)
        msg = self.map.value
        known = sum(1 for cell in msg.data if cell >= 0)
        total = max(1, len(msg.data))
        known_pct = 100.0 * known / total
        return color(
            f'Map                {msg.info.width}x{msg.info.height} '
            f'res={msg.info.resolution:.3f} known={known_pct:.1f}%',
            GREEN,
        )

    def render(self) -> None:
        lines = [
            CLEAR + color('BUNKER NAV2 / SELF-EXPLORATION STATUS', BOLD + CYAN),
            color('=' * 54, CYAN),
            self.nav_goal_line(),
            self.safety_line(),
            self.bunker_line(),
            '',
            self.fresh_label('/map', self.map),
            self.fresh_label('/odom', self.odom),
            self.fresh_label('/cmd_vel', self.out_cmd),
            self.map_line(),
            self.odom_line(),
            '',
            self.twist_line('Nav2 raw', self.raw_cmd),
            self.twist_line('Smoothed', self.auto_cmd),
            self.twist_line('To Bunker', self.out_cmd),
            '',
            f'Route status       {self.string_value(self.route_status)}',
            f'Depth route        {self.string_value(self.depth_route_status)}',
            '',
            color('Legend: GREEN=healthy/moving, YELLOW=idle/stale, RED=blocked/missing', DIM),
        ]
        print('\n'.join(lines), flush=True)

    def string_value(self, slot: TimedValue) -> str:
        if slot.value is None:
            return color('missing', YELLOW)
        text = str(slot.value.data)
        code = RED if 'BLOCK' in text.upper() or 'STOP' in text.upper() else GREEN
        return color(text, code)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2StatusMonitor()
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
