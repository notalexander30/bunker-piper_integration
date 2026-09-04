import pytest

from bunker_autonomy.sensor_fusion_node import fuse_route_status


def lidar_status():
    return {
        'transform_ok': True,
        'hard_stop': False,
        'clear': {'front': True, 'back': True, 'left': True, 'right': True},
        'counts': {'front': 5, 'back': 2, 'left': 8, 'right': 1},
    }


def depth_status():
    return {
        'transform_ok': True,
        'hard_stop': False,
        'clear': {'front': True, 'left': True, 'right': True},
        'clearance_m': {'front': 3.2, 'left': 2.1, 'right': 3.7},
    }


def test_fusion_keeps_routes_clear_only_when_both_sensors_agree():
    depth = depth_status()
    depth['clear']['left'] = False

    fused = fuse_route_status(lidar_status(), depth, False, False)

    assert fused['hard_stop'] is False
    assert fused['clear']['front'] is True
    assert fused['clear']['left'] is False
    assert fused['clear']['right'] is True
    assert fused['clear']['back'] is True
    assert fused['lidar_clear']['left'] is True
    assert fused['depth_clear']['left'] is False


def test_either_sensor_stop_produces_fused_stop_and_no_clear_routes():
    fused = fuse_route_status(lidar_status(), depth_status(), False, True)

    assert fused['hard_stop'] is True
    assert not any(fused['clear'].values())
    assert fused['primary_stop_reason'] == 'depth_obstacle'
    assert fused['stop_reasons'] == ['depth_obstacle']


def test_stale_depth_is_fail_safe():
    fused = fuse_route_status(
        lidar_status(),
        depth_status(),
        False,
        False,
        lidar_fresh=True,
        depth_fresh=False,
    )

    assert fused['hard_stop'] is True
    assert fused['sources']['depth_fresh'] is False
    assert fused['primary_stop_reason'] == 'depth_stale'


def test_fusion_preserves_depth_clearance_and_lidar_counts():
    fused = fuse_route_status(lidar_status(), depth_status(), False, False)

    assert fused['clearance_m']['right'] == 3.7
    assert fused['clearance_m']['back'] is None
    assert fused['counts']['left'] == 8
    assert fused['primary_stop_reason'] == 'clear'
    assert fused['stop_reasons'] == []


def test_fusion_preserves_lidar_stop_diagnostics():
    lidar = lidar_status()
    lidar.update({
        'hard_stop_count': 12,
        'hard_stop_point_threshold': 0,
        'hard_stop_region': {
            'name': 'hard_stop_box(front)',
            'frame_id': 'base_link',
        },
        'blocked': {'front': False, 'back': True, 'left': False, 'right': False},
        'route_block_point_thresholds': {
            'front': 240,
            'back': 80,
            'left': 120,
            'right': 120,
        },
    })

    fused = fuse_route_status(lidar, depth_status(), True, False)

    diagnostics = fused['lidar_diagnostics']
    assert diagnostics['hard_stop_count'] == 12
    assert diagnostics['hard_stop_region']['name'] == 'hard_stop_box(front)'
    assert diagnostics['route_counts']['back'] == 2
    assert diagnostics['route_blocked']['back'] is True


def test_sensor_specific_reason_is_preserved():
    depth = depth_status()
    depth['hard_stop'] = True
    depth['reason'] = 'depth_insufficient_valid_data'

    fused = fuse_route_status(lidar_status(), depth, False, True)

    assert fused['primary_stop_reason'] == 'depth_insufficient_valid_data'
    assert fused['stop_reasons'] == ['depth_insufficient_valid_data']


def test_active_stop_never_reports_clear_during_topic_transition():
    depth = depth_status()
    depth['reason'] = 'clear'

    fused = fuse_route_status(lidar_status(), depth, False, True)

    assert fused['primary_stop_reason'] == 'depth_obstacle'


def test_fusion_preserves_target_specific_depth():
    depth = depth_status()
    depth['target_distance_m'] = 0.30

    fused = fuse_route_status(lidar_status(), depth, False, False)

    assert fused['target_distance_m'] == 0.30


def test_stale_depth_never_preserves_target_completion_distance():
    depth = depth_status()
    depth['target_distance_m'] = 0.30

    fused = fuse_route_status(
        lidar_status(), depth, False, False, lidar_fresh=True, depth_fresh=False
    )

    assert fused['target_distance_m'] is None


def test_fusion_preserves_lidar_corridor_angle():
    lidar = lidar_status()
    lidar['corridor'] = {
        'width_m': 0.60,
        'selected_angle_deg': 30.0,
        'selected_free_length_m': 1.20,
        'fits': True,
    }

    fused = fuse_route_status(lidar, depth_status(), False, False)

    assert fused['corridor']['width_m'] == 0.60
    assert fused['corridor']['selected_angle_deg'] == 30.0


def test_depth_only_mode_does_not_wait_for_lidar_and_disables_reverse():
    fused = fuse_route_status(
        {},
        depth_status(),
        lidar_stop=True,
        depth_stop=False,
        lidar_fresh=False,
        depth_fresh=True,
        fusion_mode='depth_only',
    )

    assert fused['sensor'] == 'camera_depth_only'
    assert fused['navigation_mode'] == 'depth_only'
    assert fused['hard_stop'] is False
    assert fused['clear']['front'] is True
    assert fused['clear']['left'] is True
    assert fused['clear']['right'] is True
    assert fused['clear']['back'] is False
    assert fused['lidar_clear'] == {}
    assert fused['counts'] == {}
    assert fused['stop_reasons'] == []
    assert fused['sources']['lidar_required'] is False
    assert fused['sources']['lidar_fresh'] is None


def test_depth_only_mode_remains_fail_safe_when_camera_depth_is_stale():
    fused = fuse_route_status(
        {},
        depth_status(),
        lidar_stop=False,
        depth_stop=False,
        lidar_fresh=False,
        depth_fresh=False,
        fusion_mode='depth_only',
    )

    assert fused['hard_stop'] is True
    assert fused['primary_stop_reason'] == 'depth_stale'
    assert not any(fused['clear'].values())


def test_unknown_fusion_mode_is_rejected():
    with pytest.raises(ValueError, match='fusion_mode'):
        fuse_route_status(
            lidar_status(),
            depth_status(),
            False,
            False,
            fusion_mode='camera_magic',
        )
