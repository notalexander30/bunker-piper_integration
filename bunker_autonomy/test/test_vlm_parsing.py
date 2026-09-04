from bunker_autonomy.vlm_goal_monitor_node import (
    ACTION_CURVE_RIGHT,
    DISTANCE_FAR,
    MOVEMENT_FORWARD_LEFT,
    MOVEMENT_STOP,
    POSITION_LEFT,
    STATUS_TARGET_NOT_VISIBLE,
    STATUS_TARGET_VISIBLE,
    STATUS_UNCERTAIN,
    classify_vlm_text,
    detect_target_visible,
    parse_vlm_observation,
)


KEYWORDS = ['trash can visible']


def test_visible_target_keyword_detected():
    text = 'trash can visible'

    assert classify_vlm_text(text, KEYWORDS) == STATUS_TARGET_VISIBLE
    assert detect_target_visible(text, KEYWORDS) is True


def test_negated_target_keyword_not_detected():
    text = 'I do not see trash can visible in the image.'

    assert classify_vlm_text(text, KEYWORDS) == STATUS_TARGET_NOT_VISIBLE
    assert detect_target_visible(text, KEYWORDS) is False


def test_uncertain_target_status_is_not_detected():
    text = 'There might be a trash can visible at the far edge of the image.'

    assert classify_vlm_text(text, KEYWORDS) == STATUS_UNCERTAIN
    assert detect_target_visible(text, KEYWORDS) is False


def test_missing_keyword_not_detected():
    text = 'The scene shows a chair and a doorway.'

    assert classify_vlm_text(text, KEYWORDS) == STATUS_TARGET_NOT_VISIBLE


def test_broad_scene_description_does_not_trigger_detection():
    text = 'The image shows a workshop with a large trash bin in the background.'

    assert classify_vlm_text(text, KEYWORDS) == STATUS_TARGET_NOT_VISIBLE


def test_json_target_observation_parsed():
    text = (
        '{"visible": true, "horizontal_position": "left", "distance": "far", '
        '"estimated_distance_m": 2.4, "recommended_forward_m": 1.65, '
        '"recommended_turn_degrees": 18, "small_obstacle_visible": true, '
        '"small_obstacle_position": "center", "small_obstacle_distance": "near", '
        '"small_obstacle_note": "small box on floor"}'
    )

    observation = parse_vlm_observation(text, KEYWORDS)

    assert observation.status == STATUS_TARGET_VISIBLE
    assert observation.target_detected is True
    assert observation.horizontal_position == POSITION_LEFT
    assert observation.movement_proposal == MOVEMENT_FORWARD_LEFT
    assert observation.distance == DISTANCE_FAR
    assert observation.estimated_distance_m == 2.4
    assert observation.recommended_forward_m == 1.65
    assert observation.recommended_turn_degrees == 18
    assert observation.small_obstacle_visible is True
    assert observation.small_obstacle_position == 'center'
    assert observation.small_obstacle_distance == 'near'
    assert observation.small_obstacle_note == 'small box on floor'


def test_json_no_target_parsed():
    text = '{"visible": false, "horizontal_position": "unknown", "distance": "unknown"}'

    observation = parse_vlm_observation(text, KEYWORDS)

    assert observation.status == STATUS_TARGET_NOT_VISIBLE
    assert observation.target_detected is False
    assert observation.movement_proposal == MOVEMENT_STOP


def test_minimal_target_json_derives_safe_forward_left_proposal():
    observation = parse_vlm_observation(
        '{"visible": true, "horizontal_position": "left"}',
        KEYWORDS,
    )

    assert observation.status == STATUS_TARGET_VISIBLE
    assert observation.target_detected is True
    assert observation.horizontal_position == POSITION_LEFT
    assert observation.movement_proposal == MOVEMENT_FORWARD_LEFT
    assert observation.estimated_distance_m is None
    assert observation.recommended_forward_m is None
    assert observation.recommended_turn_degrees is None


def test_reverse_model_proposal_is_replaced_by_target_direction():
    observation = parse_vlm_observation(
        '{"visible": true, "horizontal_position": "left", '
        '"movement_proposal": "backward"}',
        KEYWORDS,
    )

    assert observation.movement_proposal == MOVEMENT_FORWARD_LEFT


def test_vla_search_action_is_kept_when_target_is_not_visible():
    observation = parse_vlm_observation(
        '{"target_visible": false, "target_position": "unknown", '
        '"action": "turn_right", "linear_velocity_mps": 0.0, '
        '"angular_velocity_radps": -0.17, "confidence": 0.82, '
        '"reason": "right route is clear"}',
        KEYWORDS,
    )

    assert observation.target_detected is False
    assert observation.action == 'turn_right'
    assert observation.angular_velocity_radps == -0.17
    assert observation.confidence == 0.82


def test_vla_bbox_and_curve_speed_are_normalized():
    observation = parse_vlm_observation(
        '{"target_visible": true, "target_position": "right", '
        '"target_bbox_norm": [0.62, 0.2, 0.91, 0.9], '
        '"action": "curve_right", "linear_velocity_mps": 0.08, '
        '"angular_velocity_radps": 0.12, "confidence": 0.9}',
        KEYWORDS,
    )

    assert observation.action == ACTION_CURVE_RIGHT
    assert observation.linear_velocity_mps == 0.08
    assert observation.angular_velocity_radps == -0.12
    assert observation.target_bbox_norm == [0.62, 0.2, 0.91, 0.9]
