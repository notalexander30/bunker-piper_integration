from dataclasses import replace

from bunker_autonomy.search_behavior_node import (
    ACTION_CURVE_LEFT,
    ACTION_FORWARD,
    ACTION_STOP,
    ACTION_TURN_LEFT,
    DISTANCE_FAR,
    DISTANCE_MEDIUM,
    DISTANCE_NEAR,
    POSITION_CENTER,
    POSITION_LEFT,
    ROUTE_BACK,
    ROUTE_FRONT,
    ROUTE_LEFT,
    ROUTE_RIGHT,
    BehaviorDecision,
    RouteStatus,
    SearchBehaviorNode,
    TargetObservation,
    choose_scan_angular_speed,
    choose_search_command,
    choose_vla_command,
    choose_next_relocation_route,
    make_relocation_command,
    make_twist,
    parse_route_status,
    parse_target_status,
)


def clear_route_status():
    return RouteStatus(
        transform_ok=True,
        hard_stop=False,
        clear={
            ROUTE_FRONT: True,
            ROUTE_BACK: True,
            ROUTE_LEFT: True,
            ROUTE_RIGHT: True,
        },
        blocked={
            ROUTE_FRONT: False,
            ROUTE_BACK: False,
            ROUTE_LEFT: False,
            ROUTE_RIGHT: False,
        },
        counts={
            ROUTE_FRONT: 0,
            ROUTE_BACK: 0,
            ROUTE_LEFT: 0,
            ROUTE_RIGHT: 0,
        },
        clearance_m={
            ROUTE_FRONT: 3.0,
            ROUTE_BACK: None,
            ROUTE_LEFT: 3.0,
            ROUTE_RIGHT: 3.0,
        },
    )


def choose(**overrides):
    values = {
        'safety_stop': False,
        'target': TargetObservation(),
        'target_stale': False,
        'route_status': clear_route_status(),
        'route_stale': False,
        'search_phase_sec': 0.0,
        'search_rotate_duration_sec': 4.0,
        'search_forward_duration_sec': 2.0,
        'search_angular_speed': 0.2,
        'search_linear_speed': 0.08,
        'align_angular_speed': 0.14,
        'align_forward_speed': 0.03,
        'approach_linear_speed': 0.08,
        'approach_stop_distance_m': 0.75,
        'approach_slow_distance_m': 1.2,
        'min_approach_linear_speed': 0.03,
        'close_confirmation_count': 0,
        'required_close_confirmations': 2,
    }
    values.update(overrides)
    return choose_search_command(**values)


def choose_vla(**overrides):
    values = {
        'safety_stop': False,
        'target': TargetObservation(
            action=ACTION_FORWARD,
            linear_velocity_mps=0.08,
            angular_velocity_radps=0.0,
            confidence=0.9,
        ),
        'proposal_stale': False,
        'route_status': clear_route_status(),
        'route_stale': False,
        'max_linear_speed': 0.12,
        'max_angular_speed': 0.30,
        'minimum_confidence': 0.35,
        'mission_complete_distance_m': 0.30,
        'close_confirmation_count': 0,
        'required_close_confirmations': 2,
    }
    values.update(overrides)
    return choose_vla_command(**values)


def test_vla_search_uses_fresh_model_action_without_timer_phase():
    decision = choose_vla(
        target=TargetObservation(
            detected=False,
            action=ACTION_TURN_LEFT,
            angular_velocity_radps=0.17,
            confidence=0.85,
            reason='left reveals the largest clear area',
        )
    )

    assert decision.state == 'vla_turn_left'
    assert decision.command.linear.x == 0.0
    assert decision.command.angular.z == 0.17


def test_visible_target_can_take_non_direct_sensor_approved_path():
    decision = choose_vla(
        target=TargetObservation(
            detected=True,
            horizontal_position='right',
            action=ACTION_TURN_LEFT,
            angular_velocity_radps=0.14,
            confidence=0.91,
            reason='right side is blocked; route around on the left',
        )
    )

    assert decision.state == 'vla_turn_left'
    assert decision.command.angular.z > 0.0


def test_vla_curve_requires_front_and_side_then_reroutes():
    status = clear_route_status()
    status.clear[ROUTE_FRONT] = False
    status.clearance_m[ROUTE_LEFT] = 2.5
    status.clearance_m[ROUTE_RIGHT] = 1.0
    decision = choose_vla(
        route_status=status,
        target=TargetObservation(
            action=ACTION_CURVE_LEFT,
            linear_velocity_mps=0.08,
            angular_velocity_radps=0.12,
            confidence=0.8,
        ),
    )

    assert decision.state == 'vla_reroute_left'
    assert decision.command.linear.x == 0.0
    assert decision.command.angular.z > 0.0


def test_blocked_vla_action_turns_to_selected_fitting_corridor_angle_first():
    status = clear_route_status()
    status.clear[ROUTE_FRONT] = False
    status.corridor.update({
        'width_m': 0.60,
        'selected_angle_deg': 30.0,
        'selected_free_length_m': 1.20,
        'fits': True,
    })

    decision = choose_vla(route_status=status)

    assert decision.state == 'vla_reroute_left'
    assert decision.command.linear.x == 0.0
    assert decision.command.angular.z > 0.0
    assert 'angle=30.0deg' in decision.mission_status


def test_aligned_corridor_translates_only_after_angle_is_near_zero():
    status = clear_route_status()
    status.clear[ROUTE_LEFT] = False
    status.corridor.update({
        'width_m': 0.60,
        'selected_angle_deg': 0.0,
        'selected_free_length_m': 1.20,
        'fits': True,
    })
    decision = choose_vla(
        route_status=status,
        target=TargetObservation(
            action=ACTION_CURVE_LEFT,
            linear_velocity_mps=0.08,
            angular_velocity_radps=0.12,
            confidence=0.8,
        ),
    )

    assert decision.state == 'vla_reroute_front'
    assert decision.command.linear.x > 0.0
    assert decision.command.angular.z == 0.0


def test_vla_no_available_route_stops():
    status = clear_route_status()
    for route in (ROUTE_FRONT, ROUTE_LEFT, ROUTE_RIGHT):
        status.clear[route] = False
    decision = choose_vla(route_status=status)

    assert decision.state == 'vla_stuck_stop'
    assert decision.command.linear.x == 0.0
    assert decision.command.angular.z == 0.0


def test_vla_speed_is_dynamic_but_capped():
    decision = choose_vla(
        target=TargetObservation(
            action=ACTION_FORWARD,
            linear_velocity_mps=0.8,
            confidence=0.9,
        )
    )

    assert decision.command.linear.x == 0.12


def test_stale_vla_proposal_stops_even_without_visible_target():
    decision = choose_vla(proposal_stale=True)

    assert decision.state == 'waiting_for_fresh_vla'
    assert decision.command.linear.x == 0.0


def test_target_specific_depth_completes_at_thirty_centimeters():
    status = replace(clear_route_status(), target_distance_m=0.30)
    decision = choose_vla(
        safety_stop=True,
        route_status=status,
        target=TargetObservation(
            detected=True,
            horizontal_position=POSITION_CENTER,
            target_bbox_norm=[0.4, 0.2, 0.6, 0.9],
            action=ACTION_STOP,
            confidence=0.95,
        ),
        close_confirmation_count=2,
    )

    assert decision.state == 'mission_accomplished_target_depth'
    assert decision.mission_complete is True
    assert decision.command.linear.x == 0.0


def test_target_specific_depth_above_standoff_is_not_complete():
    status = replace(clear_route_status(), target_distance_m=0.31)
    decision = choose_vla(
        route_status=status,
        target=TargetObservation(
            detected=True,
            horizontal_position=POSITION_CENTER,
            target_bbox_norm=[0.4, 0.2, 0.6, 0.9],
            action=ACTION_STOP,
            confidence=0.95,
        ),
        close_confirmation_count=2,
    )

    assert decision.mission_complete is False


def test_search_rotates_then_creeps_forward_when_no_target():
    rotating = choose(search_phase_sec=1.0)
    forward = choose(search_phase_sec=5.0)

    assert rotating.state == 'search_rotate'
    assert rotating.command.angular.z == 0.2
    assert forward.state == 'search_forward'
    assert forward.command.linear.x == 0.08


def test_forward_exploration_moves_straight_without_a_timed_scan():
    decision = choose(
        search_phase_sec=0.0,
        forward_exploration=True,
    )

    assert decision.state == 'forward_exploration_forward'
    assert decision.command.linear.x == 0.08
    assert decision.command.angular.z == 0.0


def test_forward_exploration_stops_for_relocation_when_front_is_blocked():
    route_status = clear_route_status()
    route_status.clear[ROUTE_FRONT] = False
    decision = choose(
        route_status=route_status,
        forward_exploration=True,
    )

    assert decision.state == 'forward_exploration_front_blocked'
    assert decision.command.linear.x == 0.0


def test_target_search_completes_after_required_door_confirmations():
    decision = choose(
        target=TargetObservation(detected=True, horizontal_position=POSITION_LEFT),
        complete_on_target_detection=True,
        target_detection_confirmation_count=2,
        required_target_detection_confirmations=2,
    )

    assert decision.state == 'mission_accomplished_target_found'
    assert decision.mission_complete is True
    assert decision.command.linear.x == 0.0


def test_visible_left_target_aligns_left():
    decision = choose(
        target=TargetObservation(
            detected=True,
            horizontal_position=POSITION_LEFT,
            distance=DISTANCE_FAR,
        )
    )

    assert decision.state == 'align_left'
    assert decision.command.linear.x > 0.0
    assert decision.command.angular.z > 0.0


def test_visible_left_target_stops_when_left_route_blocked():
    route_status = clear_route_status()
    route_status.clear[ROUTE_LEFT] = False
    decision = choose(
        target=TargetObservation(
            detected=True,
            horizontal_position=POSITION_LEFT,
            distance=DISTANCE_FAR,
        ),
        route_status=route_status,
    )

    assert decision.state == 'align_left_blocked'
    assert decision.command.angular.z == 0.0


def test_center_target_ignores_vlm_motion_distance_without_depth():
    route_status = clear_route_status()
    route_status.clearance_m[ROUTE_FRONT] = None
    decision = choose(
        target=TargetObservation(
            detected=True,
            horizontal_position=POSITION_CENTER,
            distance=DISTANCE_FAR,
            estimated_distance_m=1.8,
            recommended_forward_m=1.05,
        ),
        route_status=route_status,
    )

    assert decision.state == 'approach_front_blocked'
    assert decision.command.linear.x == 0.0
    assert '1.05' not in decision.mission_status


def test_center_target_uses_camera_depth_instead_of_vlm_distance():
    route_status = clear_route_status()
    route_status.clearance_m[ROUTE_FRONT] = 1.25
    decision = choose(
        target=TargetObservation(
            detected=True,
            horizontal_position=POSITION_CENTER,
            distance=DISTANCE_FAR,
            estimated_distance_m=3.0,
        ),
        route_status=route_status,
    )

    assert decision.state == 'approach_center'
    assert 'depth=1.25 m' in decision.mission_status


def test_unrelated_safety_stop_does_not_claim_mission_complete():
    decision = choose(
        safety_stop=True,
        target=TargetObservation(
            detected=True,
            horizontal_position=POSITION_CENTER,
            distance=DISTANCE_FAR,
            estimated_distance_m=2.0,
        ),
        close_confirmation_count=2,
    )

    assert decision.state == 'safety_stop'
    assert decision.mission_complete is False


def test_depth_standoff_never_overrides_an_active_safety_stop():
    route_status = clear_route_status()
    route_status.clearance_m[ROUTE_FRONT] = 0.4
    decision = choose(
        safety_stop=True,
        target=TargetObservation(
            detected=True,
            horizontal_position=POSITION_CENTER,
            distance=DISTANCE_NEAR,
            estimated_distance_m=0.4,
        ),
        route_status=route_status,
        close_confirmation_count=2,
    )

    assert decision.state == 'safety_stop'
    assert decision.mission_complete is False
    assert decision.command.linear.x == 0.0


def test_center_target_does_not_approach_when_front_route_blocked():
    route_status = clear_route_status()
    route_status.clear[ROUTE_FRONT] = False
    decision = choose(
        target=TargetObservation(
            detected=True,
            horizontal_position=POSITION_CENTER,
            distance=DISTANCE_FAR,
            estimated_distance_m=1.8,
        ),
        route_status=route_status,
    )

    assert decision.state == 'approach_front_blocked'
    assert decision.command.linear.x == 0.0


def test_center_near_target_completes_after_confirmations():
    route_status = clear_route_status()
    route_status.clearance_m[ROUTE_FRONT] = 0.7
    decision = choose(
        target=TargetObservation(
            detected=True,
            horizontal_position=POSITION_CENTER,
            distance=DISTANCE_NEAR,
            estimated_distance_m=0.7,
        ),
        route_status=route_status,
        close_confirmation_count=2,
    )

    assert decision.mission_complete is True
    assert decision.command.linear.x == 0.0


def test_center_target_completes_at_estimated_standoff():
    route_status = clear_route_status()
    route_status.clearance_m[ROUTE_FRONT] = 0.72
    decision = choose(
        target=TargetObservation(
            detected=True,
            horizontal_position=POSITION_CENTER,
            distance=DISTANCE_MEDIUM,
            estimated_distance_m=0.72,
        ),
        route_status=route_status,
        close_confirmation_count=2,
    )

    assert decision.state == 'mission_accomplished_centered_standoff'
    assert decision.mission_complete is True


def test_vlm_small_obstacle_does_not_override_depth_and_lidar_motion():
    decision = choose(
        target=TargetObservation(
            detected=False,
            small_obstacle_visible=True,
            small_obstacle_position=POSITION_CENTER,
            small_obstacle_distance=DISTANCE_NEAR,
            small_obstacle_note='small item on floor',
        )
    )

    assert decision.state == 'search_rotate'
    assert decision.command.angular.z != 0.0


def test_stale_vlm_without_visible_target_does_not_stop_sensor_search():
    decision = choose(target_stale=True, target=TargetObservation(detected=False))

    assert decision.state == 'search_rotate'
    assert decision.command.angular.z != 0.0


def test_visible_right_target_uses_safe_forward_arc():
    decision = choose(
        target=TargetObservation(
            detected=True,
            horizontal_position='right',
        )
    )

    assert decision.state == 'align_right'
    assert decision.command.linear.x > 0.0
    assert decision.command.angular.z < 0.0


def test_parse_target_status_json():
    observation = parse_target_status(
        '{"target_detected": true, "horizontal_position": "center", "distance": "near"}'
    )

    assert observation.detected is True
    assert observation.horizontal_position == POSITION_CENTER
    assert observation.distance == DISTANCE_NEAR


def test_parse_target_status_json_with_vlm_motion_advice():
    observation = parse_target_status(
        '{"target_detected": true, "horizontal_position": "right", '
        '"distance": "medium", "estimated_distance_m": 1.4, '
        '"recommended_forward_m": 0.65, "recommended_turn_degrees": -14, '
        '"small_obstacle_visible": false}'
    )

    assert observation.detected is True
    assert observation.estimated_distance_m == 1.4
    assert observation.recommended_forward_m == 0.65
    assert observation.recommended_turn_degrees == -14
    assert observation.movement_proposal == 'forward_right'


def test_parse_route_status_json():
    status = parse_route_status(
        '{"transform_ok": true, "hard_stop": false, '
        '"clear": {"front": false, "back": true}, '
        '"blocked": {"front": true, "back": false}, '
        '"counts": {"front": 20, "back": 0}}'
    )

    assert status.transform_ok is True
    assert status.is_clear(ROUTE_FRONT) is False
    assert status.is_clear(ROUTE_BACK) is True


def test_choose_next_relocation_route_never_selects_reverse():
    route_status = clear_route_status()
    route_status.clear[ROUTE_FRONT] = False

    route = choose_next_relocation_route(
        route_status,
        [ROUTE_FRONT, ROUTE_BACK],
        attempted_routes=set(),
    )

    assert route is None


def test_choose_next_relocation_route_prefers_lowest_lidar_count():
    route_status = clear_route_status()
    route_status.counts[ROUTE_FRONT] = 8
    route_status.counts[ROUTE_LEFT] = 2
    route_status.counts[ROUTE_RIGHT] = 6
    route_status.counts[ROUTE_BACK] = 4

    route = choose_next_relocation_route(
        route_status,
        [ROUTE_FRONT, ROUTE_LEFT, ROUTE_RIGHT, ROUTE_BACK],
        attempted_routes=set(),
    )

    assert route == ROUTE_LEFT


def test_choose_next_relocation_route_keeps_long_path_within_count_margin():
    route_status = clear_route_status()
    route_status.counts[ROUTE_FRONT] = 2
    route_status.counts[ROUTE_LEFT] = 0
    route_status.counts[ROUTE_RIGHT] = 4
    route_status.counts[ROUTE_BACK] = 3

    route = choose_next_relocation_route(
        route_status,
        [ROUTE_FRONT, ROUTE_LEFT, ROUTE_RIGHT, ROUTE_BACK],
        attempted_routes=set(),
        count_tie_margin=2,
    )

    assert route == ROUTE_FRONT


def test_relocation_prefers_longest_measured_depth_clearance():
    route_status = clear_route_status()
    route_status.counts[ROUTE_LEFT] = 1
    route_status.counts[ROUTE_RIGHT] = 8
    route_status.clearance_m[ROUTE_LEFT] = 1.6
    route_status.clearance_m[ROUTE_RIGHT] = 3.4

    route = choose_next_relocation_route(
        route_status,
        [ROUTE_LEFT, ROUTE_RIGHT],
        attempted_routes=set(),
    )

    assert route == ROUTE_RIGHT


def test_longest_depth_route_is_rejected_when_lidar_fit_fails():
    route_status = clear_route_status()
    route_status.lidar_clear.update({
        ROUTE_FRONT: True,
        ROUTE_BACK: True,
        ROUTE_LEFT: True,
        ROUTE_RIGHT: False,
    })
    route_status.depth_clear.update({
        ROUTE_FRONT: True,
        ROUTE_LEFT: True,
        ROUTE_RIGHT: True,
    })
    route_status.clearance_m[ROUTE_LEFT] = 1.8
    route_status.clearance_m[ROUTE_RIGHT] = 4.0

    route = choose_next_relocation_route(
        route_status,
        [ROUTE_LEFT, ROUTE_RIGHT],
        attempted_routes=set(),
    )

    assert route == ROUTE_LEFT


def test_depth_blocked_route_is_rejected_before_lidar_count_tie_break():
    route_status = clear_route_status()
    route_status.lidar_clear.update({
        ROUTE_FRONT: True,
        ROUTE_BACK: True,
        ROUTE_LEFT: True,
        ROUTE_RIGHT: True,
    })
    route_status.depth_clear.update({
        ROUTE_FRONT: True,
        ROUTE_LEFT: False,
        ROUTE_RIGHT: True,
    })
    route_status.counts[ROUTE_LEFT] = 0
    route_status.counts[ROUTE_RIGHT] = 8
    route_status.clearance_m[ROUTE_LEFT] = 4.0
    route_status.clearance_m[ROUTE_RIGHT] = 2.0

    route = choose_next_relocation_route(
        route_status,
        [ROUTE_LEFT, ROUTE_RIGHT],
        attempted_routes=set(),
    )

    assert route == ROUTE_RIGHT


def test_no_reverse_fallback_when_camera_routes_are_blocked():
    route_status = clear_route_status()
    route_status.lidar_clear.update({
        ROUTE_FRONT: True,
        ROUTE_BACK: True,
        ROUTE_LEFT: True,
        ROUTE_RIGHT: True,
    })
    route_status.depth_clear.update({
        ROUTE_FRONT: False,
        ROUTE_LEFT: False,
        ROUTE_RIGHT: False,
    })

    route = choose_next_relocation_route(
        route_status,
        [ROUTE_FRONT, ROUTE_LEFT, ROUTE_RIGHT, ROUTE_BACK],
        attempted_routes=set(),
    )

    assert route is None


def test_scan_turns_toward_side_with_longer_depth_clearance():
    route_status = clear_route_status()
    route_status.counts[ROUTE_LEFT] = 1
    route_status.counts[ROUTE_RIGHT] = 8
    route_status.clearance_m[ROUTE_LEFT] = 1.5
    route_status.clearance_m[ROUTE_RIGHT] = 3.0

    assert choose_scan_angular_speed(route_status, 0.2) == -0.2


def test_scan_turns_toward_less_cluttered_side():
    route_status = clear_route_status()
    route_status.counts[ROUTE_LEFT] = 8
    route_status.counts[ROUTE_RIGHT] = 1

    angular_speed = choose_scan_angular_speed(route_status, 0.2)

    assert angular_speed == -0.2


def test_make_relocation_command_never_moves_backward():
    command = make_relocation_command(ROUTE_BACK, 0.08)

    assert command.linear.x == 0.0


def test_make_relocation_command_rotates_in_place_for_left_and_right_routes():
    left = make_relocation_command(ROUTE_LEFT, 0.08, 0.18)
    right = make_relocation_command(ROUTE_RIGHT, 0.08, 0.18)

    assert left.linear.x == 0.0
    assert left.angular.z > 0.0
    assert right.linear.x == 0.0
    assert right.angular.z < 0.0
