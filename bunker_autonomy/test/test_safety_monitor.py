from bunker_autonomy.safety_monitor_node import (
    CorridorCandidate,
    SafetyBox,
    build_route_blocked_status,
    corridor_route_clear,
    evaluate_corridor_candidates,
    front_obstacle_stop_policy,
    select_best_corridor_candidate,
)


def test_emergency_region_can_ignore_height():
    emergency_region = SafetyBox(
        x_min=0.0,
        x_max=0.4,
        y_min=-0.35,
        y_max=0.35,
        z_min=0.0,
        z_max=0.0,
        use_z_filter=False,
    )

    assert emergency_region.contains(0.4, 0.35, 100.0)
    assert emergency_region.contains(0.2, 0.0, -100.0)
    assert not emergency_region.contains(0.41, 0.0, 0.0)
    assert not emergency_region.contains(0.2, 0.36, 0.0)


def test_route_blocked_status_uses_per_route_thresholds():
    blocked, thresholds = build_route_blocked_status(
        route_counts={
            'front': 2,
            'back': 2,
            'left': 1,
            'right': 0,
        },
        route_thresholds={
            'left': 0,
            'right': 0,
        },
        default_threshold=3,
    )

    assert thresholds == {
        'front': 3,
        'back': 3,
        'left': 0,
        'right': 0,
    }
    assert blocked == {
        'front': False,
        'back': False,
        'left': True,
        'right': False,
    }


def test_front_obstacle_reroutes_when_one_side_is_available():
    stop, reroute = front_obstacle_stop_policy(
        True,
        {'left': True, 'right': False},
    )

    assert stop is False
    assert reroute is True


def test_front_obstacle_stops_when_both_sides_are_blocked():
    stop, reroute = front_obstacle_stop_policy(
        True,
        {'left': True, 'right': True},
    )

    assert stop is True
    assert reroute is False


def test_corridor_rejects_obstacle_inside_sixty_centimeter_strip():
    candidates = evaluate_corridor_candidates(
        points=[(0.50, 0.29, 0.5)],
        angles_deg=[0.0],
        corridor_width_m=0.60,
        start_distance_m=0.20,
        lookahead_distance_m=1.20,
        minimum_free_length_m=0.60,
        z_min=0.15,
        z_max=1.5,
    )

    assert candidates[0].fits is False
    assert candidates[0].free_length_m == 0.50
    assert corridor_route_clear(candidates)['front'] is False


def test_corridor_accepts_obstacle_outside_sixty_centimeter_strip():
    candidates = evaluate_corridor_candidates(
        points=[(0.50, 0.31, 0.5)],
        angles_deg=[0.0],
        corridor_width_m=0.60,
        start_distance_m=0.20,
        lookahead_distance_m=1.20,
        minimum_free_length_m=0.60,
        z_min=0.15,
        z_max=1.5,
    )

    assert candidates[0].fits is True
    assert candidates[0].free_length_m == 1.20
    assert corridor_route_clear(candidates)['front'] is True


def test_best_corridor_prefers_longest_then_smallest_angle():
    selected = select_best_corridor_candidate([
        CorridorCandidate(-45.0, 1.0, 0, True),
        CorridorCandidate(15.0, 1.2, 0, True),
        CorridorCandidate(0.0, 1.2, 0, True),
    ])

    assert selected is not None
    assert selected.angle_deg == 0.0


def test_rear_corridor_is_separate_from_left_and_right():
    candidates = [
        CorridorCandidate(-180.0, 1.2, 0, True),
        CorridorCandidate(30.0, 0.4, 1, False),
        CorridorCandidate(-30.0, 0.4, 1, False),
    ]

    clear = corridor_route_clear(candidates)

    assert clear['back'] is True
    assert clear['left'] is False
    assert clear['right'] is False
