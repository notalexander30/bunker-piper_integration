from geometry_msgs.msg import Twist

from bunker_autonomy.cmd_vel_mux_node import (
    apply_motion_policy,
    limit_twist_rate,
    select_mux_command,
)


def make_command() -> Twist:
    msg = Twist()
    msg.linear.x = 0.4
    msg.angular.z = 0.2
    return msg


def test_safety_override_outputs_zero():
    command, reason = select_mux_command(
        latest_command=make_command(),
        safety_stop=True,
        command_age_sec=0.1,
        timeout_sec=0.5,
    )

    assert reason == 'safety_stop'
    assert command.linear.x == 0.0
    assert command.angular.z == 0.0


def test_fresh_command_is_forwarded_when_safe():
    command, reason = select_mux_command(
        latest_command=make_command(),
        safety_stop=False,
        command_age_sec=0.1,
        timeout_sec=0.5,
    )

    assert reason == 'forward'
    assert command.linear.x == 0.4
    assert command.angular.z == 0.2


def test_stale_command_times_out_to_zero():
    command, reason = select_mux_command(
        latest_command=make_command(),
        safety_stop=False,
        command_age_sec=0.6,
        timeout_sec=0.5,
    )

    assert reason == 'watchdog_timeout'
    assert command.linear.x == 0.0
    assert command.angular.z == 0.0


def test_velocity_ramps_up_at_configured_acceleration():
    limited = limit_twist_rate(
        current=Twist(),
        target=make_command(),
        dt_sec=0.1,
        linear_acceleration=0.12,
        linear_deceleration=0.25,
        angular_acceleration=0.35,
        angular_deceleration=0.7,
    )

    assert abs(limited.linear.x - 0.012) < 1e-6
    assert abs(limited.angular.z - 0.035) < 1e-6


def test_normal_zero_command_decelerates_smoothly():
    current = make_command()
    limited = limit_twist_rate(
        current=current,
        target=Twist(),
        dt_sec=0.1,
        linear_acceleration=0.12,
        linear_deceleration=0.25,
        angular_acceleration=0.35,
        angular_deceleration=0.7,
    )

    assert abs(limited.linear.x - 0.375) < 1e-6
    assert abs(limited.angular.z - 0.13) < 1e-6


def test_direction_reversal_brakes_to_zero_before_reversing():
    current = make_command()
    target = make_command()
    target.linear.x = -0.4
    limited = limit_twist_rate(
        current=current,
        target=target,
        dt_sec=0.25,
        linear_acceleration=1.0,
        linear_deceleration=2.0,
        angular_acceleration=1.0,
        angular_deceleration=2.0,
    )

    assert limited.linear.x == 0.0


def test_emergency_stop_bypasses_deceleration_ramp():
    limited = limit_twist_rate(
        current=make_command(),
        target=Twist(),
        dt_sec=0.01,
        linear_acceleration=0.12,
        linear_deceleration=0.25,
        angular_acceleration=0.35,
        angular_deceleration=0.7,
        immediate_stop=True,
    )

    assert limited.linear.x == 0.0
    assert limited.angular.z == 0.0


def test_final_motion_policy_clamps_reverse_command():
    command = make_command()
    command.linear.x = -0.08

    filtered = apply_motion_policy(command, allow_reverse=False)

    assert filtered.linear.x == 0.0
    assert filtered.angular.z == command.angular.z


def test_final_motion_policy_caps_model_velocity():
    command = make_command()
    command.linear.x = 1.2
    command.angular.z = -2.0

    filtered = apply_motion_policy(
        command,
        allow_reverse=False,
        max_linear_speed=0.12,
        max_angular_speed=0.30,
    )

    assert filtered.linear.x == 0.12
    assert filtered.angular.z == -0.30


def test_final_motion_policy_rejects_non_finite_action():
    command = make_command()
    command.linear.x = float('nan')

    filtered = apply_motion_policy(command, allow_reverse=False)

    assert filtered.linear.x == 0.0
    assert filtered.angular.z == 0.0


def test_reverse_is_rejected_when_reverse_not_allowed():
    reverse = Twist()
    reverse.linear.x = -0.04

    command, reason = select_mux_command(
        latest_command=reverse,
        safety_stop=False,
        command_age_sec=0.1,
        timeout_sec=0.5,
        allow_reverse=False,
    )

    assert reason == 'invalid_reverse'
    assert command.linear.x == 0.0


def test_reverse_is_forwarded_when_reverse_allowed():
    reverse = Twist()
    reverse.linear.x = -0.04
    reverse.angular.z = 0.1

    command, reason = select_mux_command(
        latest_command=reverse,
        safety_stop=False,
        command_age_sec=0.1,
        timeout_sec=0.5,
        allow_reverse=True,
    )

    assert reason == 'forward'
    assert command.linear.x == -0.04
    assert command.angular.z == 0.1
