from bunker_autonomy.data_logger_node import (
    parse_json_topic,
    parse_safety_reason,
    safety_reason_signature,
)


def test_parse_safety_reason_keeps_trigger_attribution():
    reason = parse_safety_reason(
        '{"stopped":true,"primary_reason":"depth_stale",'
        '"reasons":["depth_stale","lidar_obstacle"]}'
    )

    assert reason['stopped'] is True
    assert reason['primary_reason'] == 'depth_stale'
    assert reason['reasons'] == ['depth_stale', 'lidar_obstacle']


def test_safety_signature_changes_only_with_stop_state_or_reasons():
    first = safety_reason_signature(
        {
            'stopped': True,
            'primary_reason': 'lidar_obstacle',
            'reasons': ['lidar_obstacle'],
            'sources': {'lidar_stop': True},
        }
    )
    same_reason_new_detail = safety_reason_signature(
        {
            'stopped': True,
            'primary_reason': 'lidar_obstacle',
            'reasons': ['lidar_obstacle'],
            'sources': {'lidar_stop': True, 'danger_count': 50},
        }
    )
    cleared = safety_reason_signature(
        {'stopped': False, 'primary_reason': 'clear', 'reasons': []}
    )

    assert first == same_reason_new_detail
    assert first != cleared


def test_invalid_reason_message_becomes_visible_logger_error():
    reason = parse_safety_reason('not-json')

    assert reason['primary_reason'] == 'invalid_reason_message'
    assert reason['stopped'] is True


def test_json_topic_parser_exposes_structured_vlm_and_depth_data():
    parsed = parse_json_topic(
        '{"target_bbox_norm":[0.2,0.3,0.6,0.9],"target_distance_m":1.4}'
    )

    assert parsed['parse_ok'] is True
    assert parsed['data']['target_bbox_norm'] == [0.2, 0.3, 0.6, 0.9]
    assert parsed['data']['target_distance_m'] == 1.4


def test_json_topic_parser_preserves_invalid_payload_for_debugging():
    parsed = parse_json_topic('model returned non-json')

    assert parsed == {
        'parse_ok': False,
        'raw': 'model returned non-json',
    }
