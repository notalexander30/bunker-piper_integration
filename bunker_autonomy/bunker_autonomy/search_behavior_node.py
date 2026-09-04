#!/usr/bin/env python3

import json
import math
import re
from dataclasses import dataclass, field, replace
from typing import Dict, Iterable, List, Optional, Set, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Bool, String


POSITION_LEFT = 'left'
POSITION_CENTER = 'center'
POSITION_RIGHT = 'right'
POSITION_UNKNOWN = 'unknown'

DISTANCE_FAR = 'far'
DISTANCE_MEDIUM = 'medium'
DISTANCE_NEAR = 'near'
DISTANCE_UNKNOWN = 'unknown'
ROUTE_FRONT = 'front'
ROUTE_BACK = 'back'
ROUTE_LEFT = 'left'
ROUTE_RIGHT = 'right'
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


@dataclass(frozen=True)
class TargetObservation:
    detected: bool = False
    horizontal_position: str = POSITION_UNKNOWN
    movement_proposal: str = MOVEMENT_STOP
    distance: str = DISTANCE_UNKNOWN
    status: str = 'target_not_visible'
    estimated_distance_m: Optional[float] = None
    recommended_forward_m: Optional[float] = None
    recommended_turn_degrees: Optional[float] = None
    small_obstacle_visible: bool = False
    small_obstacle_position: str = POSITION_UNKNOWN
    small_obstacle_distance: str = DISTANCE_UNKNOWN
    small_obstacle_note: str = ''
    action: str = ACTION_STOP
    linear_velocity_mps: float = 0.0
    angular_velocity_radps: float = 0.0
    confidence: float = 0.0
    reason: str = ''
    target_bbox_norm: Optional[List[float]] = None


@dataclass(frozen=True)
class RouteStatus:
    navigation_mode: str = 'lidar_depth'
    transform_ok: bool = False
    hard_stop: bool = True
    clear: Dict[str, bool] = field(default_factory=dict)
    blocked: Dict[str, bool] = field(default_factory=dict)
    counts: Dict[str, int] = field(default_factory=dict)
    clearance_m: Dict[str, float] = field(default_factory=dict)
    lidar_clear: Dict[str, bool] = field(default_factory=dict)
    depth_clear: Dict[str, bool] = field(default_factory=dict)
    target_distance_m: Optional[float] = None
    corridor: Dict[str, object] = field(default_factory=dict)

    def is_clear(self, route: str) -> bool:
        return bool(self.clear.get(route, False)) and not self.hard_stop

    def is_lidar_clear(self, route: str) -> bool:
        # Fall back to the fused route for compatibility with older route messages.
        source = self.lidar_clear if self.lidar_clear else self.clear
        return bool(source.get(route, False)) and not self.hard_stop

    def is_depth_clear(self, route: str) -> bool:
        # The depth camera covers front/left/right. Older fused messages only
        # exposed the combined clear map, so retain that compatibility fallback.
        source = self.depth_clear if self.depth_clear else self.clear
        return bool(source.get(route, False)) and not self.hard_stop


@dataclass(frozen=True)
class BehaviorDecision:
    state: str
    command: Twist
    mission_complete: bool = False
    mission_status: str = ''


def make_twist(linear_x: float = 0.0, angular_z: float = 0.0) -> Twist:
    msg = Twist()
    msg.linear.x = float(linear_x)
    msg.angular.z = float(angular_z)
    return msg


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


def normalize_vla_action(value: object) -> str:
    text = str(value or '').strip().lower().replace('-', '_').replace(' ', '_')
    return {
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
    }.get(text, ACTION_STOP)


def movement_proposal_for_target(
    detected: bool,
    position: str,
    proposed: object = None,
) -> str:
    if not detected:
        return MOVEMENT_STOP
    expected = {
        POSITION_LEFT: MOVEMENT_FORWARD_LEFT,
        POSITION_CENTER: MOVEMENT_FORWARD,
        POSITION_RIGHT: MOVEMENT_FORWARD_RIGHT,
    }.get(position, MOVEMENT_STOP)
    text = str(proposed or '').strip().lower().replace('-', '_').replace(' ', '_')
    aliases = {
        'left': MOVEMENT_FORWARD_LEFT,
        'forward_left': MOVEMENT_FORWARD_LEFT,
        'straight': MOVEMENT_FORWARD,
        'forward': MOVEMENT_FORWARD,
        'right': MOVEMENT_FORWARD_RIGHT,
        'forward_right': MOVEMENT_FORWARD_RIGHT,
        'stop': MOVEMENT_STOP,
    }
    proposal = aliases.get(text, expected)
    return proposal if proposal == expected else expected


def coerce_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    text = str(value or '').strip().lower()
    return text in ('true', 'yes', 'visible', 'detected', '1')


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


def format_meters(value: Optional[float]) -> str:
    if value is None:
        return 'unknown distance'
    return '%.2f m' % value


def planned_forward_distance_m(
    target: TargetObservation,
    approach_stop_distance_m: float,
) -> Optional[float]:
    if target.recommended_forward_m is not None:
        return max(0.0, target.recommended_forward_m)
    if target.estimated_distance_m is None:
        return None
    return max(0.0, target.estimated_distance_m - approach_stop_distance_m)


def target_is_close_enough(
    target: TargetObservation,
    approach_stop_distance_m: float,
) -> bool:
    if target.recommended_forward_m is not None and target.recommended_forward_m <= 0.05:
        return True
    if target.estimated_distance_m is not None:
        return target.estimated_distance_m <= approach_stop_distance_m
    # Qualitative VLM distance is not trusted for movement or completion.
    return False


def approach_speed_for_target(
    target: TargetObservation,
    approach_linear_speed: float,
    min_approach_linear_speed: float,
    approach_slow_distance_m: float,
) -> float:
    if target.estimated_distance_m is None:
        return approach_linear_speed
    if target.estimated_distance_m > approach_slow_distance_m:
        return approach_linear_speed
    return max(min_approach_linear_speed, approach_linear_speed * 0.5)


def approach_status(
    target: TargetObservation,
    approach_stop_distance_m: float,
) -> str:
    forward_m = planned_forward_distance_m(target, approach_stop_distance_m)
    if target.estimated_distance_m is None and forward_m is None:
        return 'TARGET | approach | depth=unknown | speed=slow'
    if forward_m is None:
        return 'TARGET | approach | depth=%s | stop=%.2fm' % (
            format_meters(target.estimated_distance_m),
            approach_stop_distance_m,
        )
    return (
        'TARGET | approach | depth=%s | remaining=%.2fm | stop=%.2fm'
        % (
            format_meters(target.estimated_distance_m),
            forward_m,
            approach_stop_distance_m,
        )
    )


def parse_target_status(text: str) -> TargetObservation:
    try:
        data = json.loads(text or '{}')
    except json.JSONDecodeError:
        data = {}

    if isinstance(data, dict):
        detected = bool(data.get('target_detected', data.get('detected', False)))
        position = normalize_position(
            data.get('horizontal_position', data.get('position'))
        )
        return TargetObservation(
            detected=detected,
            horizontal_position=position,
            movement_proposal=movement_proposal_for_target(
                detected,
                position,
                data.get('movement_proposal'),
            ),
            distance=normalize_distance(data.get('distance')),
            status=str(data.get('status', 'target_visible' if detected else 'target_not_visible')),
            estimated_distance_m=coerce_optional_float(
                first_present(data, 'estimated_distance_m', 'target_distance_m', 'distance_m'),
                minimum=0.0,
                maximum=20.0,
            ),
            recommended_forward_m=coerce_optional_float(
                first_present(data, 'recommended_forward_m', 'move_forward_m', 'forward_m'),
                minimum=0.0,
                maximum=5.0,
            ),
            recommended_turn_degrees=coerce_optional_float(
                first_present(
                    data,
                    'recommended_turn_degrees',
                    'turn_degrees',
                    'turn_angle_degrees',
                    'heading_error_degrees',
                ),
                minimum=-180.0,
                maximum=180.0,
            ),
            small_obstacle_visible=coerce_bool(
                first_present(
                    data,
                    'small_obstacle_visible',
                    'small_object_visible',
                    'obstacle_visible',
                )
            ),
            small_obstacle_position=normalize_position(
                first_present(
                    data,
                    'small_obstacle_position',
                    'small_object_position',
                    'obstacle_position',
                )
            ),
            small_obstacle_distance=normalize_distance(
                first_present(
                    data,
                    'small_obstacle_distance',
                    'small_object_distance',
                    'obstacle_distance',
                )
            ),
            small_obstacle_note=str(
                first_present(
                    data,
                    'small_obstacle_note',
                    'small_object_note',
                    'obstacle_note',
                )
                or ''
            )[:160],
            action=normalize_vla_action(
                data.get('action', data.get('movement_proposal', MOVEMENT_STOP))
            ),
            linear_velocity_mps=coerce_optional_float(
                first_present(data, 'linear_velocity_mps', 'linear_x_mps', 'linear_x'),
                minimum=0.0,
                maximum=2.0,
            ) or 0.0,
            angular_velocity_radps=coerce_optional_float(
                first_present(data, 'angular_velocity_radps', 'angular_z_radps', 'angular_z'),
                minimum=-2.0,
                maximum=2.0,
            ) or 0.0,
            confidence=coerce_optional_float(
                data.get('confidence', 0.0), minimum=0.0, maximum=1.0
            ) or 0.0,
            reason=str(data.get('reason', ''))[:160],
            target_bbox_norm=(
                list(data.get('target_bbox_norm'))
                if isinstance(data.get('target_bbox_norm'), list)
                and len(data.get('target_bbox_norm')) == 4
                else None
            ),
        )

    return TargetObservation()


def parse_route_status(text: str) -> RouteStatus:
    try:
        data = json.loads(text or '{}')
    except json.JSONDecodeError:
        data = {}

    if not isinstance(data, dict):
        return RouteStatus()

    clear = data.get('clear', {})
    blocked = data.get('blocked', {})
    counts = data.get('counts', {})
    clearance_m = data.get('clearance_m', {})
    lidar_clear = data.get('lidar_clear', {})
    depth_clear = data.get('depth_clear', {})
    target_distance_m = coerce_optional_float(
        data.get('target_distance_m'), minimum=0.0, maximum=20.0
    )
    corridor = data.get('corridor', {})
    return RouteStatus(
        navigation_mode=str(data.get('navigation_mode', 'lidar_depth')),
        transform_ok=bool(data.get('transform_ok', False)),
        hard_stop=bool(data.get('hard_stop', True)),
        clear=clear if isinstance(clear, dict) else {},
        blocked=blocked if isinstance(blocked, dict) else {},
        counts=counts if isinstance(counts, dict) else {},
        clearance_m=clearance_m if isinstance(clearance_m, dict) else {},
        lidar_clear=lidar_clear if isinstance(lidar_clear, dict) else {},
        depth_clear=depth_clear if isinstance(depth_clear, dict) else {},
        target_distance_m=target_distance_m,
        corridor=corridor if isinstance(corridor, dict) else {},
    )


def selected_corridor_route(
    route_status: RouteStatus,
) -> Optional[Tuple[str, float, float]]:
    corridor = route_status.corridor
    if not bool(corridor.get('fits', False)):
        return None

    def route_for_angle(angle_value: float) -> str:
        if abs(angle_value) < 7.5:
            return ROUTE_FRONT
        return ROUTE_LEFT if angle_value > 0.0 else ROUTE_RIGHT

    parsed_candidates = []
    raw_candidates = corridor.get('candidates', [])
    if isinstance(raw_candidates, list):
        for candidate in raw_candidates:
            if not isinstance(candidate, dict) or not bool(candidate.get('fits', False)):
                continue
            candidate_angle = coerce_optional_float(
                candidate.get('angle_deg'), minimum=-180.0, maximum=180.0
            )
            candidate_free = coerce_optional_float(
                candidate.get('free_length_m'), minimum=0.0, maximum=20.0
            )
            if candidate_angle is None or candidate_free is None:
                continue
            candidate_route = route_for_angle(candidate_angle)
            if route_status.is_clear(candidate_route):
                parsed_candidates.append(
                    (candidate_route, candidate_angle, candidate_free)
                )

    if parsed_candidates:
        return min(
            parsed_candidates,
            key=lambda candidate: (-candidate[2], abs(candidate[1]), -candidate[1]),
        )

    angle = coerce_optional_float(
        corridor.get('selected_angle_deg'), minimum=-180.0, maximum=180.0
    )
    free_length = coerce_optional_float(
        corridor.get('selected_free_length_m'), minimum=0.0, maximum=20.0
    )
    if angle is None or free_length is None:
        return None
    route = route_for_angle(angle)
    if not route_status.is_clear(route):
        return None
    return route, angle, free_length


def choose_scan_angular_speed(route_status: RouteStatus, search_angular_speed: float) -> float:
    left_clearance = route_clearance_m(route_status, ROUTE_LEFT)
    right_clearance = route_clearance_m(route_status, ROUTE_RIGHT)
    left_clear = route_status.is_clear(ROUTE_LEFT) and left_clearance is not None
    right_clear = route_status.is_clear(ROUTE_RIGHT) and right_clearance is not None
    if left_clear and not right_clear:
        return abs(search_angular_speed)
    if right_clear and not left_clear:
        return -abs(search_angular_speed)
    if left_clear and right_clear:
        if right_clearance > left_clearance + 0.2:
            return -abs(search_angular_speed)
        if left_clearance > right_clearance + 0.2:
            return abs(search_angular_speed)
        left_count = route_point_count(route_status, ROUTE_LEFT)
        right_count = route_point_count(route_status, ROUTE_RIGHT)
        if right_count < left_count:
            return -abs(search_angular_speed)
        return abs(search_angular_speed)
    return 0.0


def route_point_count(route_status: RouteStatus, route: str) -> int:
    try:
        return int(route_status.counts.get(route, 0))
    except (TypeError, ValueError):
        return 0


def navigation_source_label(route_status: RouteStatus) -> str:
    return (
        'camera depth'
        if route_status.navigation_mode == 'depth_only'
        else 'LiDAR+depth'
    )


def route_clearance_m(route_status: RouteStatus, route: str) -> Optional[float]:
    try:
        value = float(route_status.clearance_m.get(route))
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) and value >= 0.0 else None


def target_with_depth_distance(
    target: TargetObservation,
    route_status: RouteStatus,
    approach_stop_distance_m: float,
) -> TargetObservation:
    if not target.detected:
        return target
    if target.target_bbox_norm is not None and route_status.target_distance_m is not None:
        measured_distance = route_status.target_distance_m
        return replace(
            target,
            distance=DISTANCE_UNKNOWN,
            estimated_distance_m=measured_distance,
            recommended_forward_m=max(
                0.0, measured_distance - approach_stop_distance_m
            ),
            recommended_turn_degrees=None,
        )
    route = {
        POSITION_LEFT: ROUTE_LEFT,
        POSITION_CENTER: ROUTE_FRONT,
        POSITION_RIGHT: ROUTE_RIGHT,
    }.get(target.horizontal_position)
    if route is None:
        return replace(
            target,
            distance=DISTANCE_UNKNOWN,
            estimated_distance_m=None,
            recommended_forward_m=None,
            recommended_turn_degrees=None,
        )
    measured_distance = route_clearance_m(route_status, route)
    return replace(
        target,
        distance=DISTANCE_UNKNOWN,
        estimated_distance_m=measured_distance,
        recommended_forward_m=(
            None
            if measured_distance is None
            else max(0.0, measured_distance - approach_stop_distance_m)
        ),
        recommended_turn_degrees=None,
    )


def choose_next_relocation_route(
    route_status: RouteStatus,
    relocation_routes: Iterable[str],
    attempted_routes: Set[str],
    count_tie_margin: int = 0,
    clearance_tie_margin_m: float = 0.25,
) -> Optional[str]:
    candidates = []
    for index, route in enumerate(relocation_routes):
        if route == ROUTE_BACK or route in attempted_routes:
            continue
        # LiDAR route boxes are the robot-fit gate for every motion direction.
        if not route_status.is_lidar_clear(route):
            continue
        # The forward-facing camera adds a second gate for routes it can see.
        if route in (ROUTE_FRONT, ROUTE_LEFT, ROUTE_RIGHT) and not (
            route_status.is_depth_clear(route)
        ):
            continue
        clearance = route_clearance_m(route_status, route)
        if route in (ROUTE_FRONT, ROUTE_LEFT, ROUTE_RIGHT) and clearance is None:
            continue
        candidates.append(
            (
                route_point_count(route_status, route),
                index,
                route,
                clearance,
            )
        )
    if not candidates:
        return None

    # Camera depth ranks measurable routes by longest usable clearance. LiDAR
    # point count only breaks near-ties; it never approves a route that failed
    # the robot-width fit gate above. Reverse is intentionally not a candidate.
    measured = [candidate for candidate in candidates if candidate[3] is not None]
    if measured:
        longest = max(candidate[3] for candidate in measured)
        clearance_margin = max(0.0, float(clearance_tie_margin_m))
        candidates = [
            candidate
            for candidate in measured
            if candidate[3] >= longest - clearance_margin
        ]

    min_count = min(count for count, _, _, _ in candidates)
    margin = max(0, int(count_tie_margin))
    preferred_candidates = [
        candidate for candidate in candidates if candidate[0] <= min_count + margin
    ]
    return min(preferred_candidates, key=lambda candidate: candidate[1])[2]


def route_choice_status(route_status: RouteStatus, route: str) -> str:
    clearance = route_clearance_m(route_status, route)
    count = route_point_count(route_status, route)
    if route_status.navigation_mode == 'depth_only':
        return 'ROUTE | %s selected | camera depth=%.2fm' % (
            route,
            clearance if clearance is not None else 0.0,
        )
    if clearance is None:
        return 'ROUTE | %s selected | LiDAR fit=yes | points=%d' % (route, count)
    return 'ROUTE | %s selected | depth=%.2fm | LiDAR fit=yes | points=%d' % (
        route,
        clearance,
        count,
    )


def make_relocation_command(
    route: str,
    search_linear_speed: float,
    relocation_angular_speed: float = 0.2,
) -> Twist:
    if route == ROUTE_BACK:
        # Reverse is disabled because the forward camera cannot verify that path.
        return make_twist()
    if route == ROUTE_LEFT:
        return make_twist(
            angular_z=abs(relocation_angular_speed),
        )
    if route == ROUTE_RIGHT:
        return make_twist(
            angular_z=-abs(relocation_angular_speed),
        )
    return make_twist(linear_x=abs(search_linear_speed))


def action_required_routes(action: str) -> Tuple[str, ...]:
    return {
        ACTION_FORWARD: (ROUTE_FRONT,),
        ACTION_CURVE_LEFT: (ROUTE_FRONT, ROUTE_LEFT),
        ACTION_CURVE_RIGHT: (ROUTE_FRONT, ROUTE_RIGHT),
        ACTION_TURN_LEFT: (ROUTE_LEFT,),
        ACTION_TURN_RIGHT: (ROUTE_RIGHT,),
        ACTION_STOP: (),
    }.get(action, ())


def action_is_clear(action: str, route_status: RouteStatus) -> bool:
    required = action_required_routes(action)
    return bool(required) and all(route_status.is_clear(route) for route in required)


def bounded_vla_command(
    target: TargetObservation,
    max_linear_speed: float,
    max_angular_speed: float,
) -> Optional[Twist]:
    linear = float(target.linear_velocity_mps)
    angular = float(target.angular_velocity_radps)
    if not math.isfinite(linear) or not math.isfinite(angular):
        return None
    linear = max(0.0, min(abs(max_linear_speed), linear))
    angular = max(-abs(max_angular_speed), min(abs(max_angular_speed), angular))
    action = target.action
    if action == ACTION_STOP:
        return make_twist()
    if action == ACTION_FORWARD and linear > 0.0:
        return make_twist(linear_x=linear)
    if action == ACTION_CURVE_LEFT and linear > 0.0 and angular != 0.0:
        return make_twist(linear_x=linear, angular_z=abs(angular))
    if action == ACTION_CURVE_RIGHT and linear > 0.0 and angular != 0.0:
        return make_twist(linear_x=linear, angular_z=-abs(angular))
    if action == ACTION_TURN_LEFT and angular != 0.0:
        return make_twist(angular_z=abs(angular))
    if action == ACTION_TURN_RIGHT and angular != 0.0:
        return make_twist(angular_z=-abs(angular))
    return None


def reroute_command_from_proposal(
    route: str,
    target: TargetObservation,
    max_linear_speed: float,
    max_angular_speed: float,
) -> Twist:
    """Map model-selected intensity onto the best currently safe alternate route."""
    ratios = [max(0.0, min(1.0, float(target.confidence)))]
    if max_linear_speed > 0.0:
        ratios.append(abs(float(target.linear_velocity_mps)) / max_linear_speed)
    if max_angular_speed > 0.0:
        ratios.append(abs(float(target.angular_velocity_radps)) / max_angular_speed)
    intensity = max(0.2, min(1.0, max(ratios)))
    if route == ROUTE_FRONT:
        return make_twist(linear_x=max_linear_speed * intensity)
    if route == ROUTE_LEFT:
        return make_twist(angular_z=max_angular_speed * intensity)
    if route == ROUTE_RIGHT:
        return make_twist(angular_z=-max_angular_speed * intensity)
    return make_twist()


def reroute_command_for_corridor(
    angle_deg: float,
    target: TargetObservation,
    max_linear_speed: float,
    max_angular_speed: float,
    alignment_tolerance_deg: float = 7.5,
) -> Twist:
    """Turn toward a fitting corridor before translating into it."""
    ratios = [max(0.0, min(1.0, float(target.confidence)))]
    if max_linear_speed > 0.0:
        ratios.append(abs(float(target.linear_velocity_mps)) / max_linear_speed)
    if max_angular_speed > 0.0:
        ratios.append(abs(float(target.angular_velocity_radps)) / max_angular_speed)
    intensity = max(0.2, min(1.0, max(ratios)))
    if abs(angle_deg) <= alignment_tolerance_deg:
        return make_twist(linear_x=max_linear_speed * intensity)
    angular = max_angular_speed * intensity
    return make_twist(angular_z=angular if angle_deg > 0.0 else -angular)


def choose_vla_command(
    safety_stop: bool,
    target: TargetObservation,
    proposal_stale: bool,
    route_status: RouteStatus,
    route_stale: bool,
    max_linear_speed: float,
    max_angular_speed: float,
    minimum_confidence: float,
    mission_complete_distance_m: float,
    close_confirmation_count: int,
    required_close_confirmations: int,
) -> BehaviorDecision:
    """Validate one fresh VLA proposal against the live deterministic route envelope."""
    target = target_with_depth_distance(
        target,
        route_status,
        mission_complete_distance_m,
    )

    # Completion is target-specific: a high-confidence bbox must have produced
    # a depth measurement for consecutive observations. Stopping remains safe
    # even when the close target also activates an obstacle stop.
    if (
        target.detected
        and target.horizontal_position == POSITION_CENTER
        and target.target_bbox_norm is not None
        and route_status.target_distance_m is not None
        and route_status.target_distance_m <= mission_complete_distance_m
        and close_confirmation_count >= required_close_confirmations
    ):
        return BehaviorDecision(
            state='mission_accomplished_target_depth',
            command=make_twist(),
            mission_complete=True,
            mission_status='DONE | trash can centered | target_depth=%.2fm | standoff=%.2fm'
            % (route_status.target_distance_m, mission_complete_distance_m),
        )

    if route_stale:
        return BehaviorDecision(
            state='waiting_for_fresh_route',
            command=make_twist(),
            mission_status='WAIT | %s route data stale'
            % navigation_source_label(route_status),
        )
    if proposal_stale:
        return BehaviorDecision(
            state='waiting_for_fresh_vla',
            command=make_twist(),
            mission_status='WAIT | VLA action stale',
        )
    if target.confidence < minimum_confidence:
        return BehaviorDecision(
            state='vla_low_confidence_stop',
            command=make_twist(),
            mission_status='STOP | VLA confidence %.2f below %.2f'
            % (target.confidence, minimum_confidence),
        )

    corridor_choice = selected_corridor_route(route_status)
    if safety_stop:
        return BehaviorDecision(
            state='safety_stop',
            command=make_twist(),
            mission_status='STOP | safety sensor blocked',
        )
    command = bounded_vla_command(target, max_linear_speed, max_angular_speed)
    if command is None:
        return BehaviorDecision(
            state='vla_invalid_action_stop',
            command=make_twist(),
            mission_status='STOP | invalid VLA action or speed',
        )

    available = [
        route
        for route in (ROUTE_FRONT, ROUTE_LEFT, ROUTE_RIGHT)
        if route_status.is_clear(route)
    ]
    if target.action == ACTION_STOP:
        return BehaviorDecision(
            state='vla_stuck_stop' if not available else 'vla_hold',
            command=make_twist(),
            mission_status=(
                'STOP | no route passes %s checks'
                % navigation_source_label(route_status)
                if not available
                else 'VLA | hold | %s' % (target.reason or 'model selected stop')
            ),
        )

    if action_is_clear(target.action, route_status):
        return BehaviorDecision(
            state='vla_%s' % target.action,
            command=command,
            mission_status='VLA | %s | linear=%.3f angular=%.3f | %s'
            % (
                target.action,
                command.linear.x,
                command.angular.z,
                target.reason or 'fresh sensor-approved action',
            ),
        )

    reroute = corridor_choice[0] if corridor_choice is not None else None
    if reroute is None:
        reroute = choose_next_relocation_route(
            route_status,
            (ROUTE_FRONT, ROUTE_LEFT, ROUTE_RIGHT),
            attempted_routes=set(),
            count_tie_margin=2,
            clearance_tie_margin_m=0.15,
        )
    if reroute is None:
        return BehaviorDecision(
            state='vla_stuck_stop',
            command=make_twist(),
            mission_status='STOP | requested path blocked and no safe reroute',
        )
    if corridor_choice is not None:
        _, corridor_angle_deg, corridor_free_length_m = corridor_choice
        rerouted_command = reroute_command_for_corridor(
            corridor_angle_deg,
            target,
            max_linear_speed,
            max_angular_speed,
        )
    else:
        corridor_angle_deg = None
        corridor_free_length_m = None
        rerouted_command = reroute_command_from_proposal(
            reroute,
            target,
            max_linear_speed,
            max_angular_speed,
        )
    return BehaviorDecision(
        state='vla_reroute_%s' % reroute,
        command=rerouted_command,
        mission_status='REROUTE | VLA %s blocked | selected %s | angle=%s free=%s | linear=%.3f angular=%.3f'
        % (
            target.action,
            reroute,
            ('%.1fdeg' % corridor_angle_deg if corridor_angle_deg is not None else 'legacy'),
            ('%.2fm' % corridor_free_length_m if corridor_free_length_m is not None else 'legacy'),
            rerouted_command.linear.x,
            rerouted_command.angular.z,
        ),
    )


def choose_search_command(
    safety_stop: bool,
    target: TargetObservation,
    target_stale: bool,
    route_status: RouteStatus,
    route_stale: bool,
    search_phase_sec: float,
    search_rotate_duration_sec: float,
    search_forward_duration_sec: float,
    search_angular_speed: float,
    search_linear_speed: float,
    align_angular_speed: float,
    align_forward_speed: float,
    approach_linear_speed: float,
    approach_stop_distance_m: float,
    approach_slow_distance_m: float,
    min_approach_linear_speed: float,
    close_confirmation_count: int,
    required_close_confirmations: int,
    forward_exploration: bool = False,
    complete_on_target_detection: bool = False,
    target_detection_confirmation_count: int = 0,
    required_target_detection_confirmations: int = 1,
) -> BehaviorDecision:
    target = target_with_depth_distance(
        target,
        route_status,
        approach_stop_distance_m,
    )
    movement_proposal = movement_proposal_for_target(
        target.detected,
        target.horizontal_position,
        target.movement_proposal,
    )

    if safety_stop:
        return BehaviorDecision(
            state='safety_stop',
            command=make_twist(),
            mission_status='STOP | safety sensor blocked',
        )

    # Search-only missions (for example, finding a doorway) should stop as
    # soon as the goal is reliably observed.  They do not need to approach the
    # object, unlike a manipulation/collection mission that needs a standoff.
    if (
        complete_on_target_detection
        and target.detected
        and target_detection_confirmation_count
        >= max(1, required_target_detection_confirmations)
    ):
        return BehaviorDecision(
            state='mission_accomplished_target_found',
            command=make_twist(),
            mission_complete=True,
            mission_status='DONE | target found | confirmations=%d'
            % target_detection_confirmation_count,
        )

    if target.detected and movement_proposal == MOVEMENT_FORWARD:
        if target_is_close_enough(
            target,
            approach_stop_distance_m,
        ) and close_confirmation_count >= required_close_confirmations:
            return BehaviorDecision(
                state='mission_accomplished_centered_standoff',
                command=make_twist(),
                mission_complete=True,
                mission_status=(
                    'DONE | trash can centered | depth=%s | standoff=%.2fm'
                    % (format_meters(target.estimated_distance_m), approach_stop_distance_m)
                ),
            )

    # A stale detector only blocks tracking an already-seen target. Searching
    # remains sensor-driven when no target is visible.
    if target_stale and target.detected:
        return BehaviorDecision(
            state='waiting_for_fresh_vlm',
            command=make_twist(),
            mission_status='WAIT | VLM data stale',
        )

    if route_stale:
        return BehaviorDecision(
            state='waiting_for_fresh_lidar_route',
            command=make_twist(),
            mission_status='WAIT | %s route data stale'
            % navigation_source_label(route_status),
        )

    if target.detected:
        if movement_proposal == MOVEMENT_FORWARD_LEFT:
            if not route_status.is_clear(ROUTE_LEFT) or target.estimated_distance_m is None:
                return BehaviorDecision(
                    state='align_left_blocked',
                    command=make_twist(),
                    mission_status='STOP | target left | %s blocked'
                    % navigation_source_label(route_status),
                )
            return BehaviorDecision(
                state='align_left',
                command=make_twist(
                    linear_x=abs(align_forward_speed),
                    angular_z=align_angular_speed,
                ),
                mission_status='TARGET | forward arc left | %s clear'
                % navigation_source_label(route_status),
            )

        if movement_proposal == MOVEMENT_FORWARD_RIGHT:
            if not route_status.is_clear(ROUTE_RIGHT) or target.estimated_distance_m is None:
                return BehaviorDecision(
                    state='align_right_blocked',
                    command=make_twist(),
                    mission_status='STOP | target right | %s blocked'
                    % navigation_source_label(route_status),
                )
            return BehaviorDecision(
                state='align_right',
                command=make_twist(
                    linear_x=abs(align_forward_speed),
                    angular_z=-align_angular_speed,
                ),
                mission_status='TARGET | forward arc right | %s clear'
                % navigation_source_label(route_status),
            )

        if movement_proposal == MOVEMENT_STOP:
            return BehaviorDecision(
                state='target_position_unknown',
                command=make_twist(),
                mission_status='WAIT | target direction unknown',
            )

        if not route_status.is_clear(ROUTE_FRONT) or target.estimated_distance_m is None:
            return BehaviorDecision(
                state='approach_front_blocked',
                command=make_twist(),
                mission_status='STOP | target ahead | %s blocked'
                % navigation_source_label(route_status),
            )

        linear_x = approach_speed_for_target(
            target,
            approach_linear_speed,
            min_approach_linear_speed,
            approach_slow_distance_m,
        )
        return BehaviorDecision(
            state='approach_center',
            command=make_twist(linear_x=linear_x),
            mission_status=approach_status(target, approach_stop_distance_m),
        )

    # Forward exploration covers ground instead of spending most of a search
    # cycle rotating in place.  A blocked front route is handled by the node's
    # safe relocation state machine, which only selects a verified clear route.
    if forward_exploration:
        if (
            not route_status.is_clear(ROUTE_FRONT)
            or route_clearance_m(route_status, ROUTE_FRONT) is None
        ):
            return BehaviorDecision(
                state='forward_exploration_front_blocked',
                command=make_twist(),
                mission_status='ROUTE | forward exploration blocked | select clear route',
            )
        return BehaviorDecision(
            state='forward_exploration_forward',
            command=make_twist(linear_x=search_linear_speed),
            mission_status='SEARCH | forward exploration | %s clear'
            % navigation_source_label(route_status),
        )

    cycle_sec = max(0.1, search_rotate_duration_sec + search_forward_duration_sec)
    phase = search_phase_sec % cycle_sec
    if phase < search_rotate_duration_sec:
        angular_z = choose_scan_angular_speed(route_status, search_angular_speed)
        if angular_z == 0.0:
            return BehaviorDecision(
                state='search_rotation_blocked',
                command=make_twist(),
                mission_status='STOP | scan space blocked',
            )
        return BehaviorDecision(
            state='search_rotate',
            command=make_twist(angular_z=angular_z),
            mission_status='SEARCH | rotate %s' % ('left' if angular_z > 0.0 else 'right'),
        )

    if (
        not route_status.is_clear(ROUTE_FRONT)
        or route_clearance_m(route_status, ROUTE_FRONT) is None
    ):
        return BehaviorDecision(
            state='search_forward_blocked',
            command=make_twist(),
            mission_status='ROUTE | front blocked | select longest depth path',
        )

    return BehaviorDecision(
        state='search_forward',
        command=make_twist(linear_x=search_linear_speed),
        mission_status='SEARCH | forward | %s clear'
        % navigation_source_label(route_status),
    )


class SearchBehaviorNode(Node):
    def __init__(self):
        super().__init__('search_behavior_node')

        self.declare_parameter('target_detected_topic', '/target_detected')
        self.declare_parameter('target_status_topic', '/target_status')
        self.declare_parameter('safety_stop_topic', '/safety_stop')
        self.declare_parameter('route_status_topic', '/route_status')
        self.declare_parameter('cmd_vel_autonomy_topic', '/cmd_vel_autonomy')
        self.declare_parameter('mission_complete_topic', '/mission_complete')
        self.declare_parameter('mission_status_topic', '/mission_status')
        self.declare_parameter('planner_mode', 'vla')
        self.declare_parameter('vla_max_linear_speed_mps', 0.06)
        self.declare_parameter('vla_max_angular_speed_radps', 0.15)
        self.declare_parameter('vla_minimum_confidence', 0.35)
        self.declare_parameter('route_timeout_sec', 1.5)
        self.declare_parameter('search_angular_speed', 0.1)
        self.declare_parameter('search_linear_speed', 0.04)
        self.declare_parameter('full_scan_duration_sec', 32.0)
        self.declare_parameter('relocation_drive_duration_sec', 2.5)
        self.declare_parameter(
            'relocation_routes',
            [ROUTE_FRONT, ROUTE_LEFT, ROUTE_RIGHT],
        )
        self.declare_parameter('relocation_angular_speed', 0.1)
        self.declare_parameter('front_block_confirmation_sec', 0.0)
        self.declare_parameter('relocation_count_tie_margin', 0)
        self.declare_parameter('relocation_clearance_tie_margin_m', 0.25)
        self.declare_parameter('search_rotate_duration_sec', 4.0)
        self.declare_parameter('search_forward_duration_sec', 2.0)
        self.declare_parameter('complete_on_target_detection', False)
        self.declare_parameter('required_target_detection_confirmations', 1)
        self.declare_parameter('align_angular_speed', 0.07)
        self.declare_parameter('align_forward_speed', 0.015)
        self.declare_parameter('approach_linear_speed', 0.04)
        self.declare_parameter('approach_stop_distance_m', 0.75)
        self.declare_parameter('approach_slow_distance_m', 1.2)
        self.declare_parameter('min_approach_linear_speed', 0.015)
        self.declare_parameter('target_timeout_sec', 8.0)
        self.declare_parameter('required_close_confirmations', 2)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('initial_safety_stop', True)
        self.declare_parameter('exit_on_mission_complete', True)

        target_detected_topic = str(self.get_parameter('target_detected_topic').value)
        target_status_topic = str(self.get_parameter('target_status_topic').value)
        safety_stop_topic = str(self.get_parameter('safety_stop_topic').value)
        route_status_topic = str(self.get_parameter('route_status_topic').value)
        cmd_vel_autonomy_topic = str(self.get_parameter('cmd_vel_autonomy_topic').value)
        mission_complete_topic = str(self.get_parameter('mission_complete_topic').value)
        mission_status_topic = str(self.get_parameter('mission_status_topic').value)
        self._planner_mode = str(self.get_parameter('planner_mode').value).strip().lower()
        self._vla_max_linear_speed = max(
            0.0, float(self.get_parameter('vla_max_linear_speed_mps').value)
        )
        self._vla_max_angular_speed = max(
            0.0, float(self.get_parameter('vla_max_angular_speed_radps').value)
        )
        self._vla_minimum_confidence = max(
            0.0, min(1.0, float(self.get_parameter('vla_minimum_confidence').value))
        )
        self._route_timeout_sec = max(
            0.1, float(self.get_parameter('route_timeout_sec').value)
        )
        self._search_angular_speed = float(self.get_parameter('search_angular_speed').value)
        self._search_linear_speed = float(self.get_parameter('search_linear_speed').value)
        self._full_scan_duration_sec = float(
            self.get_parameter('full_scan_duration_sec').value
        )
        self._relocation_drive_duration_sec = float(
            self.get_parameter('relocation_drive_duration_sec').value
        )
        relocation_routes_value = self.get_parameter('relocation_routes').value
        self._relocation_routes = [
            str(route)
            for route in relocation_routes_value
            if str(route) in (ROUTE_FRONT, ROUTE_LEFT, ROUTE_RIGHT)
        ]
        self._relocation_angular_speed = float(
            self.get_parameter('relocation_angular_speed').value
        )
        self._front_block_confirmation_sec = max(
            0.0,
            float(self.get_parameter('front_block_confirmation_sec').value),
        )
        self._relocation_count_tie_margin = int(
            self.get_parameter('relocation_count_tie_margin').value
        )
        self._relocation_clearance_tie_margin_m = float(
            self.get_parameter('relocation_clearance_tie_margin_m').value
        )
        self._search_rotate_duration_sec = float(
            self.get_parameter('search_rotate_duration_sec').value
        )
        self._search_forward_duration_sec = float(
            self.get_parameter('search_forward_duration_sec').value
        )
        self._complete_on_target_detection = bool(
            self.get_parameter('complete_on_target_detection').value
        )
        self._required_target_detection_confirmations = max(
            1,
            int(self.get_parameter('required_target_detection_confirmations').value),
        )
        self._align_angular_speed = float(self.get_parameter('align_angular_speed').value)
        self._align_forward_speed = max(
            0.0, float(self.get_parameter('align_forward_speed').value)
        )
        self._approach_linear_speed = float(self.get_parameter('approach_linear_speed').value)
        self._approach_stop_distance_m = float(
            self.get_parameter('approach_stop_distance_m').value
        )
        self._approach_slow_distance_m = float(
            self.get_parameter('approach_slow_distance_m').value
        )
        self._min_approach_linear_speed = float(
            self.get_parameter('min_approach_linear_speed').value
        )
        self._target_timeout_sec = float(self.get_parameter('target_timeout_sec').value)
        self._required_close_confirmations = int(
            self.get_parameter('required_close_confirmations').value
        )
        publish_rate_hz = max(0.1, float(self.get_parameter('publish_rate_hz').value))
        self._safety_stop = bool(self.get_parameter('initial_safety_stop').value)
        self._exit_on_mission_complete = bool(
            self.get_parameter('exit_on_mission_complete').value
        )

        status_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._cmd_pub = self.create_publisher(Twist, cmd_vel_autonomy_topic, 10)
        self._mission_complete_pub = self.create_publisher(
            Bool,
            mission_complete_topic,
            status_qos,
        )
        self._mission_status_pub = self.create_publisher(
            String,
            mission_status_topic,
            status_qos,
        )
        self.create_subscription(Bool, target_detected_topic, self.target_detected_callback, 10)
        self.create_subscription(String, target_status_topic, self.target_status_callback, 10)
        self.create_subscription(Bool, safety_stop_topic, self.safety_stop_callback, 10)
        self.create_subscription(String, route_status_topic, self.route_status_callback, 10)
        self._timer = self.create_timer(1.0 / publish_rate_hz, self.publish_behavior_command)

        self._target = TargetObservation()
        self._route_status = RouteStatus()
        self._last_target_status_time = None
        self._last_route_status_time = None
        self._mission_complete = False
        self._close_confirmation_count = 0
        self._target_detection_confirmation_count = 0
        self._last_state = None
        self._last_mission_status = ''
        self._search_started_time = self.get_clock().now()
        self._relocation_started_time = None
        self._relocation_route: Optional[str] = None
        self._front_blocked_started_time = None
        self._attempted_routes: Set[str] = set()
        self._shutdown_requested = False

        self.get_logger().info(
            'Mission behavior started: target_status_topic=%s, safety_stop_topic=%s, '
            'route_status_topic=%s, cmd_vel_autonomy_topic=%s, mission_complete_topic=%s'
            % (
                target_status_topic,
                safety_stop_topic,
                route_status_topic,
                cmd_vel_autonomy_topic,
                mission_complete_topic,
            )
        )

    def target_detected_callback(self, msg: Bool) -> None:
        if bool(msg.data) != self._target.detected:
            self._target = replace(
                self._target,
                detected=bool(msg.data),
                status='target_visible' if msg.data else 'target_not_visible',
            )

    def target_status_callback(self, msg: String) -> None:
        self._target = parse_target_status(msg.data)
        self._last_target_status_time = self.get_clock().now()
        if self._target.detected:
            self._target_detection_confirmation_count += 1
        else:
            self._target_detection_confirmation_count = 0
        depth_target = target_with_depth_distance(
            self._target,
            self._route_status,
            self._approach_stop_distance_m,
        )
        if (
            depth_target.detected
            and depth_target.horizontal_position == POSITION_CENTER
            and target_is_close_enough(
                depth_target,
                self._approach_stop_distance_m,
            )
        ):
            self._close_confirmation_count += 1
        else:
            self._close_confirmation_count = 0

    def safety_stop_callback(self, msg: Bool) -> None:
        if self._safety_stop != msg.data:
            self.get_logger().info('safety_stop changed to %s' % msg.data)
        self._safety_stop = bool(msg.data)

    def route_status_callback(self, msg: String) -> None:
        self._route_status = parse_route_status(msg.data)
        self._last_route_status_time = self.get_clock().now()

    def _target_stale(self) -> bool:
        if self._last_target_status_time is None:
            return True
        age_sec = (self.get_clock().now() - self._last_target_status_time).nanoseconds / 1e9
        return age_sec > self._target_timeout_sec

    def _route_stale(self) -> bool:
        if self._last_route_status_time is None:
            return True
        age_sec = (self.get_clock().now() - self._last_route_status_time).nanoseconds / 1e9
        return age_sec > self._route_timeout_sec

    def _search_phase_sec(self) -> float:
        return (self.get_clock().now() - self._search_started_time).nanoseconds / 1e9

    def _relocation_phase_sec(self) -> float:
        if self._relocation_started_time is None:
            return 0.0
        return (self.get_clock().now() - self._relocation_started_time).nanoseconds / 1e9

    def _reset_scan(self) -> None:
        self._search_started_time = self.get_clock().now()
        self._relocation_started_time = None
        self._relocation_route = None

    def _front_block_has_persisted(self) -> bool:
        """Debounce side-route selection without ever clearing a safety stop."""
        if self._front_block_confirmation_sec <= 0.0:
            return True
        now = self.get_clock().now()
        if self._front_blocked_started_time is None:
            self._front_blocked_started_time = now
            return False
        elapsed_sec = (now - self._front_blocked_started_time).nanoseconds / 1e9
        return elapsed_sec >= self._front_block_confirmation_sec

    def _reset_front_block_confirmation(self) -> None:
        self._front_blocked_started_time = None

    def _publish_mission_status(self, complete: bool, status: str) -> None:
        complete_msg = Bool()
        complete_msg.data = bool(complete)
        self._mission_complete_pub.publish(complete_msg)

        if status and status != self._last_mission_status:
            status_msg = String()
            status_msg.data = status
            self._mission_status_pub.publish(status_msg)
            if complete:
                self.get_logger().info(status)
            else:
                # operator_status_node is the single user-facing INFO stream.
                self.get_logger().debug(status)
            self._last_mission_status = status

    def _start_or_complete_relocation(self) -> BehaviorDecision:
        route = choose_next_relocation_route(
            self._route_status,
            self._relocation_routes,
            self._attempted_routes,
            self._relocation_count_tie_margin,
            self._relocation_clearance_tie_margin_m,
        )
        if route is None:
            self._attempted_routes.clear()
            route = choose_next_relocation_route(
                self._route_status,
                self._relocation_routes,
                self._attempted_routes,
                self._relocation_count_tie_margin,
                self._relocation_clearance_tie_margin_m,
            )

        if route is None:
            return BehaviorDecision(
                state='waiting_for_clear_relocation_route',
                command=make_twist(),
                mission_status='STOP | no route passes %s checks'
                % navigation_source_label(self._route_status),
            )

        self._relocation_route = route
        self._relocation_started_time = self.get_clock().now()
        return BehaviorDecision(
            state='relocate_%s' % route,
            command=make_relocation_command(
                route,
                self._search_linear_speed,
                self._relocation_angular_speed,
            ),
            mission_status=route_choice_status(self._route_status, route),
        )

    def _relocation_decision(self) -> Optional[BehaviorDecision]:
        if self._relocation_route is None:
            return None

        route = self._relocation_route
        if not self._route_status.is_clear(route):
            self._reset_scan()
            return BehaviorDecision(
                state='relocation_route_blocked',
                command=make_twist(),
                mission_status='STOP | %s route no longer fits' % route,
            )

        if self._relocation_phase_sec() < self._relocation_drive_duration_sec:
            return BehaviorDecision(
                state='relocate_%s' % route,
                command=make_relocation_command(
                    route,
                    self._search_linear_speed,
                    self._relocation_angular_speed,
                ),
                mission_status=route_choice_status(self._route_status, route),
            )

        self._attempted_routes.add(route)
        self._reset_scan()
        return BehaviorDecision(
            state='scan_after_relocation',
            command=make_twist(),
            mission_status='SEARCH | relocation complete | scan',
        )

    def publish_behavior_command(self) -> None:
        if self._mission_complete:
            self._cmd_pub.publish(make_twist())
            self._publish_mission_status(True, self._last_mission_status)
            if self._exit_on_mission_complete and not self._shutdown_requested:
                self._shutdown_requested = True
                self.get_logger().info('Mission complete; shutting down autonomy launch.')
                rclpy.shutdown()
            return

        route_stale = self._route_stale()
        target_stale = self._target_stale()

        if self._planner_mode == 'vla':
            decision = choose_vla_command(
                safety_stop=self._safety_stop,
                target=self._target,
                proposal_stale=target_stale,
                route_status=self._route_status,
                route_stale=route_stale,
                max_linear_speed=self._vla_max_linear_speed,
                max_angular_speed=self._vla_max_angular_speed,
                minimum_confidence=self._vla_minimum_confidence,
                mission_complete_distance_m=self._approach_stop_distance_m,
                close_confirmation_count=self._close_confirmation_count,
                required_close_confirmations=self._required_close_confirmations,
            )
            self._cmd_pub.publish(decision.command)
            self._publish_mission_status(decision.mission_complete, decision.mission_status)
            if decision.state != self._last_state:
                self.get_logger().debug('VLA behavior state: %s' % decision.state)
            self._last_state = decision.state
            if decision.mission_complete:
                self._mission_complete = True
            return

        decision: Optional[BehaviorDecision] = None
        front_blocked = not self._route_status.is_clear(ROUTE_FRONT)
        if (
            not self._safety_stop
            and not route_stale
            and not self._target.detected
        ):
            relocation_decision = self._relocation_decision()
            if relocation_decision is not None:
                decision = relocation_decision
            elif front_blocked:
                # Binary near-obstacle policy: immediately choose a fused-clear
                # side route. _start_or_complete_relocation returns zero if no
                # depth+LiDAR side route is available.
                if (
                    self._planner_mode == 'forward_exploration'
                    and not self._front_block_has_persisted()
                ):
                    decision = BehaviorDecision(
                        state='forward_exploration_block_confirmation',
                        command=make_twist(),
                        mission_status=(
                            'WAIT | forward route blocked | confirming before reroute'
                        ),
                    )
                else:
                    decision = self._start_or_complete_relocation()
            elif (
                self._planner_mode != 'forward_exploration'
                and (
                self._search_phase_sec() >= self._full_scan_duration_sec
                or choose_scan_angular_speed(
                    self._route_status,
                    self._search_angular_speed,
                )
                == 0.0
                )
            ):
                decision = self._start_or_complete_relocation()

        if not front_blocked or self._safety_stop or route_stale or self._target.detected:
            self._reset_front_block_confirmation()

        if decision is None:
            decision = choose_search_command(
                safety_stop=self._safety_stop,
                target=self._target,
                target_stale=target_stale,
                route_status=self._route_status,
                route_stale=route_stale,
                search_phase_sec=self._search_phase_sec(),
                search_rotate_duration_sec=self._search_rotate_duration_sec,
                search_forward_duration_sec=self._search_forward_duration_sec,
                search_angular_speed=self._search_angular_speed,
                search_linear_speed=self._search_linear_speed,
                align_angular_speed=self._align_angular_speed,
                align_forward_speed=self._align_forward_speed,
                approach_linear_speed=self._approach_linear_speed,
                approach_stop_distance_m=self._approach_stop_distance_m,
                approach_slow_distance_m=self._approach_slow_distance_m,
                min_approach_linear_speed=self._min_approach_linear_speed,
                close_confirmation_count=self._close_confirmation_count,
                required_close_confirmations=self._required_close_confirmations,
                forward_exploration=self._planner_mode == 'forward_exploration',
                complete_on_target_detection=self._complete_on_target_detection,
                target_detection_confirmation_count=(
                    self._target_detection_confirmation_count
                ),
                required_target_detection_confirmations=(
                    self._required_target_detection_confirmations
                ),
            )

        self._cmd_pub.publish(decision.command)
        self._publish_mission_status(decision.mission_complete, decision.mission_status)

        if decision.state != self._last_state:
            self.get_logger().debug('Mission behavior state: %s' % decision.state)
        self._last_state = decision.state

        if decision.mission_complete:
            self._mission_complete = True


def main(args=None):
    rclpy.init(args=args)
    node = SearchBehaviorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
