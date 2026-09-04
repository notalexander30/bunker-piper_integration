#!/usr/bin/env python3

import json
import math
import re
from dataclasses import asdict, dataclass
from typing import Iterable, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


DEFAULT_TARGET_KEYWORDS = ['trash can', 'bin', 'garbage bin', 'waste bin']
STATUS_TARGET_VISIBLE = 'target_visible'
STATUS_TARGET_NOT_VISIBLE = 'target_not_visible'
STATUS_UNCERTAIN = 'uncertain'
POSITION_LEFT = 'left'
POSITION_CENTER = 'center'
POSITION_RIGHT = 'right'
POSITION_UNKNOWN = 'unknown'
DISTANCE_FAR = 'far'
DISTANCE_MEDIUM = 'medium'
DISTANCE_NEAR = 'near'
DISTANCE_UNKNOWN = 'unknown'
MOVEMENT_FORWARD_LEFT = 'forward_left'
MOVEMENT_FORWARD = 'forward'
MOVEMENT_FORWARD_RIGHT = 'forward_right'
MOVEMENT_STOP = 'stop'
ACTION_FORWARD = 'forward'
ACTION_CURVE_LEFT = 'curve_left'
ACTION_CURVE_RIGHT = 'curve_right'
ACTION_TURN_LEFT = 'turn_left'
ACTION_TURN_RIGHT = 'turn_right'
ACTION_STOP = 'stop'
VLA_ACTIONS = {
    ACTION_FORWARD,
    ACTION_CURVE_LEFT,
    ACTION_CURVE_RIGHT,
    ACTION_TURN_LEFT,
    ACTION_TURN_RIGHT,
    ACTION_STOP,
}


@dataclass(frozen=True)
class TargetObservation:
    status: str
    target_detected: bool
    horizontal_position: str = POSITION_UNKNOWN
    movement_proposal: str = MOVEMENT_STOP
    distance: str = DISTANCE_UNKNOWN
    estimated_distance_m: Optional[float] = None
    recommended_forward_m: Optional[float] = None
    recommended_turn_degrees: Optional[float] = None
    small_obstacle_visible: bool = False
    small_obstacle_position: str = POSITION_UNKNOWN
    small_obstacle_distance: str = DISTANCE_UNKNOWN
    small_obstacle_note: str = ''
    raw_result: str = ''
    action: str = ACTION_STOP
    linear_velocity_mps: float = 0.0
    angular_velocity_radps: float = 0.0
    confidence: float = 0.0
    reason: str = ''
    target_bbox_norm: Optional[List[float]] = None

NEGATION_TOKENS = {
    'no',
    'not',
    'none',
    'without',
    'absent',
    'missing',
    'cannot',
    'cant',
    'dont',
    'doesnt',
    'didnt',
}
UNCERTAIN_PHRASES = {
    'maybe',
    'possibly',
    'uncertain',
    'unclear',
    'not sure',
    'hard to tell',
    'cannot tell',
    'cant tell',
    'may be',
    'might be',
    'could be',
}
ABSENCE_PHRASES = {
    'no target',
    'nothing relevant',
    'nothing visible',
    'not visible',
    'cannot see it',
    'cant see it',
}


def normalize_position(value: object) -> str:
    text = str(value or '').strip().lower()
    aliases = {
        'left': POSITION_LEFT,
        'center': POSITION_CENTER,
        'centre': POSITION_CENTER,
        'middle': POSITION_CENTER,
        'right': POSITION_RIGHT,
    }
    return aliases.get(text, POSITION_UNKNOWN)


def normalize_movement_proposal(value: object) -> str:
    text = str(value or '').strip().lower().replace('-', '_').replace(' ', '_')
    aliases = {
        'forward_left': MOVEMENT_FORWARD_LEFT,
        'left': MOVEMENT_FORWARD_LEFT,
        'forward': MOVEMENT_FORWARD,
        'straight': MOVEMENT_FORWARD,
        'forward_right': MOVEMENT_FORWARD_RIGHT,
        'right': MOVEMENT_FORWARD_RIGHT,
        'stop': MOVEMENT_STOP,
        'hold': MOVEMENT_STOP,
        'none': MOVEMENT_STOP,
    }
    return aliases.get(text, MOVEMENT_STOP)


def normalize_vla_action(value: object) -> str:
    text = str(value or '').strip().lower().replace('-', '_').replace(' ', '_')
    aliases = {
        'straight': ACTION_FORWARD,
        'forward': ACTION_FORWARD,
        'forward_left': ACTION_CURVE_LEFT,
        'arc_left': ACTION_CURVE_LEFT,
        'curve_left': ACTION_CURVE_LEFT,
        'forward_right': ACTION_CURVE_RIGHT,
        'arc_right': ACTION_CURVE_RIGHT,
        'curve_right': ACTION_CURVE_RIGHT,
        'left': ACTION_TURN_LEFT,
        'rotate_left': ACTION_TURN_LEFT,
        'turn_left': ACTION_TURN_LEFT,
        'right': ACTION_TURN_RIGHT,
        'rotate_right': ACTION_TURN_RIGHT,
        'turn_right': ACTION_TURN_RIGHT,
        'hold': ACTION_STOP,
        'none': ACTION_STOP,
        'stop': ACTION_STOP,
    }
    return aliases.get(text, ACTION_STOP)


def normalize_bbox(value: object) -> Optional[List[float]]:
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        return None
    try:
        x_min, y_min, x_max, y_max = [float(item) for item in value]
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(item) for item in (x_min, y_min, x_max, y_max)):
        return None
    x_min = max(0.0, min(1.0, x_min))
    y_min = max(0.0, min(1.0, y_min))
    x_max = max(0.0, min(1.0, x_max))
    y_max = max(0.0, min(1.0, y_max))
    if x_max <= x_min or y_max <= y_min:
        return None
    return [round(x_min, 4), round(y_min, 4), round(x_max, 4), round(y_max, 4)]


def normalized_action_velocities(action: str, linear: float, angular: float) -> Tuple[float, float]:
    linear = max(0.0, float(linear))
    angular = float(angular)
    if action == ACTION_STOP:
        return 0.0, 0.0
    if action == ACTION_FORWARD:
        return linear, 0.0
    if action == ACTION_CURVE_LEFT:
        return linear, abs(angular)
    if action == ACTION_CURVE_RIGHT:
        return linear, -abs(angular)
    if action == ACTION_TURN_LEFT:
        return 0.0, abs(angular)
    if action == ACTION_TURN_RIGHT:
        return 0.0, -abs(angular)
    return 0.0, 0.0


def proposal_for_position(visible: bool, position: str, proposed: object = None) -> str:
    proposal = normalize_movement_proposal(proposed)
    if not visible:
        return MOVEMENT_STOP
    expected = {
        POSITION_LEFT: MOVEMENT_FORWARD_LEFT,
        POSITION_CENTER: MOVEMENT_FORWARD,
        POSITION_RIGHT: MOVEMENT_FORWARD_RIGHT,
    }.get(position, MOVEMENT_STOP)
    # Reject inconsistent or unsupported model advice. In particular, no
    # reverse proposal can pass normalization.
    return proposal if proposal == expected else expected


def normalize_distance(value: object) -> str:
    text = str(value or '').strip().lower()
    aliases = {
        'far': DISTANCE_FAR,
        'distant': DISTANCE_FAR,
        'medium': DISTANCE_MEDIUM,
        'mid': DISTANCE_MEDIUM,
        'middle': DISTANCE_MEDIUM,
        'near': DISTANCE_NEAR,
        'close': DISTANCE_NEAR,
    }
    return aliases.get(text, DISTANCE_UNKNOWN)


def coerce_bool(value: object) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    text = str(value or '').strip().lower()
    if text in ('true', 'yes', 'visible', 'detected', '1'):
        return True
    if text in ('false', 'no', 'not visible', 'none', '0'):
        return False
    return None


def coerce_optional_float(
    value: object,
    minimum: Optional[float] = None,
    maximum: Optional[float] = None,
) -> Optional[float]:
    if value is None:
        return None

    try:
        number = float(value)
    except (TypeError, ValueError):
        match = re.search(r'-?\d+(?:\.\d+)?', str(value))
        if not match:
            return None
        try:
            number = float(match.group(0))
        except ValueError:
            return None

    if not math.isfinite(number):
        return None
    if minimum is not None and number < minimum:
        number = minimum
    if maximum is not None and number > maximum:
        number = maximum
    return round(number, 3)


def first_present(data: dict, *keys: str):
    for key in keys:
        if key in data:
            return data[key]
    return None


def normalize_note(value: object) -> str:
    text = str(value or '').strip()
    return text[:160]


def extract_json_object(text: str) -> Optional[dict]:
    stripped = (text or '').strip()
    if stripped.startswith('```'):
        stripped = re.sub(r'^```(?:json)?\s*', '', stripped, flags=re.IGNORECASE)
        stripped = re.sub(r'\s*```$', '', stripped)

    candidates = [stripped]
    start = stripped.find('{')
    end = stripped.rfind('}')
    if start != -1 and end != -1 and end > start:
        candidates.append(stripped[start:end + 1])

    for candidate in candidates:
        try:
            parsed = json.loads(candidate)
        except json.JSONDecodeError:
            continue
        if isinstance(parsed, dict):
            return parsed
    return None


def normalize_text(text: str) -> str:
    lowered = text.lower()
    return re.sub(r'\s+', ' ', re.sub(r'[^a-z0-9]+', ' ', lowered)).strip()


def _keyword_positions(tokens: List[str], keyword_tokens: List[str]) -> Iterable[int]:
    if not keyword_tokens:
        return []

    last_start = len(tokens) - len(keyword_tokens)
    if last_start < 0:
        return []

    return (
        index
        for index in range(last_start + 1)
        if tokens[index:index + len(keyword_tokens)] == keyword_tokens
    )


def _has_uncertainty(normalized_text: str) -> bool:
    padded = ' %s ' % normalized_text
    return any((' %s ' % phrase) in padded for phrase in UNCERTAIN_PHRASES)


def _has_absence_phrase(normalized_text: str) -> bool:
    padded = ' %s ' % normalized_text
    return any((' %s ' % phrase) in padded for phrase in ABSENCE_PHRASES)


def _span_is_negated(tokens: List[str], start: int) -> bool:
    preceding_window = tokens[max(0, start - 6):start]
    return any(token in NEGATION_TOKENS for token in preceding_window)


def _span_is_contained(span: Tuple[int, int], containing_spans: Iterable[Tuple[int, int]]) -> bool:
    start, end = span
    return any(start >= parent_start and end <= parent_end for parent_start, parent_end in containing_spans)


def classify_vlm_text(text: str, target_keywords: Iterable[str]) -> str:
    normalized = normalize_text(text or '')
    if not normalized:
        return STATUS_UNCERTAIN

    tokens = normalized.split()
    normalized_keywords = [
        normalize_text(keyword).split()
        for keyword in target_keywords
        if normalize_text(keyword)
    ]
    normalized_keywords.sort(key=len, reverse=True)

    matched_any_keyword = False
    negated_spans: List[Tuple[int, int]] = []
    for keyword_tokens in normalized_keywords:
        positions = list(_keyword_positions(tokens, keyword_tokens))

        for start in positions:
            span = (start, start + len(keyword_tokens))
            if _span_is_contained(span, negated_spans):
                continue

            matched_any_keyword = True
            if _span_is_negated(tokens, start):
                negated_spans.append(span)
                continue

            if _has_uncertainty(normalized):
                return STATUS_UNCERTAIN
            return STATUS_TARGET_VISIBLE

    if matched_any_keyword:
        return STATUS_TARGET_NOT_VISIBLE

    if _has_absence_phrase(normalized):
        return STATUS_TARGET_NOT_VISIBLE

    return STATUS_TARGET_NOT_VISIBLE


def detect_target_visible(text: str, target_keywords: Iterable[str]) -> bool:
    return classify_vlm_text(text, target_keywords) == STATUS_TARGET_VISIBLE


def parse_vlm_observation(text: str, target_keywords: Iterable[str]) -> TargetObservation:
    raw_text = text or ''
    parsed_json = extract_json_object(raw_text)
    if parsed_json is not None:
        visible = coerce_bool(
            parsed_json.get(
                'visible',
                parsed_json.get(
                    'target_visible',
                    parsed_json.get('trash_can_visible', parsed_json.get('detected')),
                ),
            )
        )
        if visible is None:
            visible = detect_target_visible(raw_text, target_keywords)

        position = normalize_position(
            parsed_json.get(
                'horizontal_position',
                parsed_json.get('position', parsed_json.get('target_position')),
            )
        )
        legacy_proposal = proposal_for_position(
            bool(visible),
            position,
            parsed_json.get('movement_proposal'),
        )
        action = normalize_vla_action(
            parsed_json.get('action', parsed_json.get('movement_proposal', legacy_proposal))
        )
        linear_velocity = coerce_optional_float(
            first_present(parsed_json, 'linear_velocity_mps', 'linear_x_mps', 'linear_x'),
            minimum=0.0,
            maximum=2.0,
        )
        angular_velocity = coerce_optional_float(
            first_present(parsed_json, 'angular_velocity_radps', 'angular_z_radps', 'angular_z'),
            minimum=-2.0,
            maximum=2.0,
        )
        linear_velocity, angular_velocity = normalized_action_velocities(
            action,
            linear_velocity or 0.0,
            angular_velocity or 0.0,
        )

        small_obstacle_visible = coerce_bool(
            first_present(
                parsed_json,
                'small_obstacle_visible',
                'small_object_visible',
                'obstacle_visible',
            )
        )

        return TargetObservation(
            status=STATUS_TARGET_VISIBLE if visible else STATUS_TARGET_NOT_VISIBLE,
            target_detected=bool(visible),
            horizontal_position=position,
            movement_proposal=legacy_proposal,
            distance=normalize_distance(
                parsed_json.get('distance', parsed_json.get('target_distance'))
            ),
            estimated_distance_m=coerce_optional_float(
                first_present(
                    parsed_json,
                    'estimated_distance_m',
                    'target_distance_m',
                    'distance_m',
                ),
                minimum=0.0,
                maximum=20.0,
            ),
            recommended_forward_m=coerce_optional_float(
                first_present(
                    parsed_json,
                    'recommended_forward_m',
                    'move_forward_m',
                    'forward_m',
                ),
                minimum=0.0,
                maximum=5.0,
            ),
            recommended_turn_degrees=coerce_optional_float(
                first_present(
                    parsed_json,
                    'recommended_turn_degrees',
                    'turn_degrees',
                    'turn_angle_degrees',
                    'heading_error_degrees',
                ),
                minimum=-180.0,
                maximum=180.0,
            ),
            small_obstacle_visible=bool(small_obstacle_visible),
            small_obstacle_position=normalize_position(
                first_present(
                    parsed_json,
                    'small_obstacle_position',
                    'small_object_position',
                    'obstacle_position',
                )
            ),
            small_obstacle_distance=normalize_distance(
                first_present(
                    parsed_json,
                    'small_obstacle_distance',
                    'small_object_distance',
                    'obstacle_distance',
                )
            ),
            small_obstacle_note=normalize_note(
                first_present(
                    parsed_json,
                    'small_obstacle_note',
                    'small_object_note',
                    'obstacle_note',
                )
            ),
            raw_result=raw_text,
            action=action,
            linear_velocity_mps=linear_velocity,
            angular_velocity_radps=angular_velocity,
            confidence=coerce_optional_float(
                parsed_json.get('confidence', 1.0), minimum=0.0, maximum=1.0
            ) or 0.0,
            reason=normalize_note(parsed_json.get('reason', '')),
            target_bbox_norm=normalize_bbox(
                first_present(parsed_json, 'target_bbox_norm', 'bbox_norm', 'target_bbox')
            ),
        )

    status = classify_vlm_text(raw_text, target_keywords)
    normalized = normalize_text(raw_text)
    position = POSITION_UNKNOWN
    if ' left ' in (' %s ' % normalized):
        position = POSITION_LEFT
    elif ' right ' in (' %s ' % normalized):
        position = POSITION_RIGHT
    elif any(word in (' %s ' % normalized) for word in (' center ', ' centre ', ' middle ')):
        position = POSITION_CENTER

    distance = DISTANCE_UNKNOWN
    if ' far ' in (' %s ' % normalized):
        distance = DISTANCE_FAR
    elif any(word in (' %s ' % normalized) for word in (' medium ', ' mid ')):
        distance = DISTANCE_MEDIUM
    elif any(word in (' %s ' % normalized) for word in (' near ', ' close ')):
        distance = DISTANCE_NEAR

    estimated_distance_m = coerce_optional_float(
        re.search(r'\b\d+(?:\.\d+)?\s*(?:m|meter|meters)\b', raw_text, flags=re.I).group(0)
        if re.search(r'\b\d+(?:\.\d+)?\s*(?:m|meter|meters)\b', raw_text, flags=re.I)
        else None,
        minimum=0.0,
        maximum=20.0,
    )
    small_obstacle_visible = any(
        phrase in (' %s ' % normalized)
        for phrase in (' small obstacle ', ' small object ', ' object on floor ')
    )

    return TargetObservation(
        status=status,
        target_detected=status == STATUS_TARGET_VISIBLE,
        horizontal_position=position,
        movement_proposal=proposal_for_position(
            status == STATUS_TARGET_VISIBLE,
            position,
        ),
        distance=distance,
        estimated_distance_m=estimated_distance_m,
        small_obstacle_visible=small_obstacle_visible,
        raw_result=raw_text,
        action=normalize_vla_action(
            proposal_for_position(status == STATUS_TARGET_VISIBLE, position)
        ),
        confidence=0.0,
    )


class VlmGoalMonitorNode(Node):
    def __init__(self):
        super().__init__('vlm_goal_monitor_node')

        self.declare_parameter('vlm_result_topic', '/vlm_result')
        self.declare_parameter('target_detected_topic', '/target_detected')
        self.declare_parameter('target_status_topic', '/target_status')
        self.declare_parameter('target_keywords', DEFAULT_TARGET_KEYWORDS)

        vlm_result_topic = self.get_parameter('vlm_result_topic').value
        target_detected_topic = self.get_parameter('target_detected_topic').value
        target_status_topic = self.get_parameter('target_status_topic').value
        self._target_keywords = self._read_target_keywords()

        self._target_detected_pub = self.create_publisher(Bool, target_detected_topic, 10)
        self._target_status_pub = self.create_publisher(String, target_status_topic, 10)
        self._vlm_result_sub = self.create_subscription(
            String,
            vlm_result_topic,
            self.vlm_result_callback,
            10,
        )
        self._last_status = None

        self.get_logger().info(
            'VLM goal monitor started: vlm_result_topic=%s, target_detected_topic=%s, '
            'target_status_topic=%s, target_keywords=%s'
            % (
                vlm_result_topic,
                target_detected_topic,
                target_status_topic,
                self._target_keywords,
            )
        )

    def _read_target_keywords(self) -> List[str]:
        value = self.get_parameter('target_keywords').value
        if isinstance(value, str):
            return [value]
        return [str(item) for item in value]

    def vlm_result_callback(self, msg: String) -> None:
        observation = parse_vlm_observation(msg.data, self._target_keywords)

        detected_msg = Bool()
        detected_msg.data = observation.target_detected
        self._target_detected_pub.publish(detected_msg)

        status_msg = String()
        status_msg.data = json.dumps(asdict(observation), sort_keys=True)
        self._target_status_pub.publish(status_msg)

        if self._last_status != observation.status:
            self.get_logger().info(
                'Target status changed to %s, target_detected=%s, position=%s, '
                'action=%s, linear=%.3f, angular=%.3f'
                % (
                    observation.status,
                    detected_msg.data,
                    observation.horizontal_position,
                    observation.action,
                    observation.linear_velocity_mps,
                    observation.angular_velocity_radps,
                )
            )
        else:
            self.get_logger().info(
                'Target status=%s, target_detected=%s, position=%s, '
                'action=%s, linear=%.3f, angular=%.3f'
                % (
                    observation.status,
                    detected_msg.data,
                    observation.horizontal_position,
                    observation.action,
                    observation.linear_velocity_mps,
                    observation.angular_velocity_radps,
                ),
                throttle_duration_sec=2.0,
            )
        self._last_status = observation.status


def main(args=None):
    rclpy.init(args=args)
    node = VlmGoalMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
