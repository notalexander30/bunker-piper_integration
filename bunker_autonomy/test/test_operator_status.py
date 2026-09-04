from geometry_msgs.msg import Twist

from bunker_autonomy.operator_status_node import (
    OperatorState,
    build_operator_notice,
    notice_dedup_key,
    summarize_safety_detail,
    twist_is_active,
)


def make_state(**overrides):
    values = {
        'has_safety_status': True,
        'safety_status_stale': False,
        'safety_stop': False,
        'safety_detail': '',
        'has_target_status': True,
        'target_status_stale': False,
        'target_detected': False,
        'command_active': True,
        'mission_complete': False,
        'mission_status': '',
        'mode': 'dry_run',
        'output_cmd_vel_topic': '/cmd_vel_debug',
        'output_subscription_count': 0,
    }
    values.update(overrides)
    return OperatorState(**values)


def test_notice_waits_for_safety_status():
    notice = build_operator_notice(make_state(has_safety_status=False))

    assert notice == 'WAIT | safety data missing | check LiDAR+depth nodes'


def test_notice_reports_safety_stop_before_other_states():
    notice = build_operator_notice(
        make_state(
            safety_stop=True,
            safety_detail='depth obstacle',
            target_detected=True,
        )
    )

    assert notice == 'STOP | depth obstacle'


def test_notice_reports_stale_vlm_status():
    notice = build_operator_notice(make_state(target_status_stale=True))

    assert notice == 'WAIT | camera/VLM data stale'


def test_notice_reports_mission_status():
    notice = build_operator_notice(
        make_state(
            target_detected=True,
            mission_status='TARGET | approach | depth=1.25m',
        )
    )

    assert notice == 'DRY RUN | TARGET | approach | depth=1.25m'


def test_notice_reports_mission_complete():
    notice = build_operator_notice(
        make_state(
            mission_complete=True,
            mission_status='DONE | trash can centered | depth=0.45m',
        )
    )

    assert notice == 'DONE | trash can centered | depth=0.45m'


def test_notice_reports_dry_run_when_command_active():
    notice = build_operator_notice(make_state())

    assert notice.startswith('DRY RUN |')


def test_notice_reports_missing_chassis_in_drive_mode():
    notice = build_operator_notice(
        make_state(
            mode='drive',
            output_cmd_vel_topic='/cmd_vel',
            output_subscription_count=0,
        )
    )

    assert notice == 'ERROR | chassis not listening on /cmd_vel | start bunker_base'


def test_twist_is_active_detects_nonzero_command():
    msg = Twist()
    msg.angular.z = 0.2

    assert twist_is_active(msg) is True


def test_safety_detail_reports_stale_sensor_names():
    detail = summarize_safety_detail(
        '{"sources":{"lidar_fresh":true,"depth_fresh":false,'
        '"lidar_stop":false,"depth_stop":true}}'
    )

    assert detail == 'depth data stale'


def test_safety_detail_reports_obstacle_source():
    detail = summarize_safety_detail(
        '{"transform_ok":true,"sources":{"lidar_fresh":true,'
        '"depth_fresh":true,"lidar_stop":true,"depth_stop":false}}'
    )

    assert detail == 'LiDAR obstacle'


def test_safety_detail_prefers_explicit_primary_reason():
    detail = summarize_safety_detail(
        '{"hard_stop":true,"primary_stop_reason":'
        '"depth_insufficient_valid_data","stop_reasons":'
        '["depth_insufficient_valid_data"]}'
    )

    assert detail == 'depth image invalid/insufficient'


def test_safety_detail_explains_lidar_trigger_region_and_route_counts():
    detail = summarize_safety_detail(
        '{"primary_stop_reason":"lidar_obstacle","lidar_diagnostics":{'
        '"hard_stop_count":1775,"hard_stop_point_threshold":0,'
        '"hard_stop_region":{"name":"hard_stop_box(front)",'
        '"frame_id":"base_link","x_min":0.0,"x_max":0.4,'
        '"y_min":-0.35,"y_max":0.35,"z_filter_enabled":false},'
        '"route_counts":{"front":0,"back":652,"left":0,"right":0}}}'
    )

    assert 'trigger=hard_stop_box(front)' in detail
    assert 'points=1775 > threshold=0' in detail
    assert 'x[0.00,0.40]m y[-0.35,0.35]m z=ignored' in detail
    assert 'route_points front=0 back=652 left=0 right=0' in detail


def test_lidar_notice_dedup_ignores_live_count_changes():
    first = (
        'STOP | LiDAR trigger=hard_stop_box(front) | points=1745 > threshold=0 | '
        'route_points front=0 back=451 left=0 right=0'
    )
    second = (
        'STOP | LiDAR trigger=hard_stop_box(front) | points=1759 > threshold=0 | '
        'route_points front=0 back=466 left=0 right=0'
    )

    assert notice_dedup_key(first) == notice_dedup_key(second)
