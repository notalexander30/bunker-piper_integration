import numpy as np
import pytest
from sensor_msgs.msg import Image

from bunker_autonomy.depth_route_monitor_node import (
    depth_image_to_meters,
    mask_bbox,
    summarize_depth_image,
    target_depth_from_bbox,
)


def test_open_depth_image_reports_three_clear_long_sectors():
    depth = np.full((90, 120), 3.0, dtype=np.float32)

    sectors = summarize_depth_image(depth)

    assert all(sector.clear for sector in sectors.values())
    assert all(abs(sector.clearance_m - 3.0) < 0.01 for sector in sectors.values())
    assert sectors['front'].hard_stop_fraction == 0.0


def test_center_obstacle_blocks_front_and_crosses_stop_fraction():
    depth = np.full((90, 120), 2.0, dtype=np.float32)
    depth[30:60, 50:70] = 0.175

    sectors = summarize_depth_image(depth)

    assert sectors['front'].clear is False
    assert sectors['front'].hard_stop_fraction >= 0.02
    assert sectors['left'].clear is True
    assert sectors['right'].clear is True


def test_missing_depth_is_not_marked_clear():
    depth = np.zeros((90, 120), dtype=np.float32)

    sectors = summarize_depth_image(depth)

    assert all(sector.clear is False for sector in sectors.values())
    assert all(sector.clearance_m is None for sector in sectors.values())
    assert all(sector.finite_positive_fraction == 0.0 for sector in sectors.values())


def test_sparse_real_depth_can_be_accepted_with_an_explicit_lower_threshold():
    depth = np.zeros((90, 120), dtype=np.float32)
    # A distributed 6% of real, far measurements is not a blank depth image.
    depth[20:80:4, 42:78:3] = 2.3

    sectors = summarize_depth_image(depth, minimum_valid_fraction=0.05)

    assert sectors['front'].finite_positive_fraction > 0.05
    assert sectors['front'].clear is True
    assert sectors['front'].hard_stop_fraction == 0.0


def test_nearest_robust_percentile_describes_sector_clearance():
    depth = np.full((90, 120), 2.5, dtype=np.float32)
    depth[20:82, :16] = 1.0

    sectors = summarize_depth_image(depth)

    assert 0.9 <= sectors['left'].clearance_m <= 1.1
    assert sectors['front'].clearance_m > sectors['left'].clearance_m


def test_arm_camera_horizontal_roi_and_sector_boundaries_are_configurable():
    depth = np.full((60, 100), 2.5, dtype=np.float32)
    depth[12:50, 42:58] = 0.25

    default_sectors = summarize_depth_image(depth)
    shifted_sectors = summarize_depth_image(
        depth,
        roi_left_fraction=0.10,
        roi_right_fraction=0.90,
        left_sector_end_fraction=0.60,
        right_sector_start_fraction=0.80,
    )

    assert default_sectors['front'].clear is False
    assert shifted_sectors['left'].clear is False
    assert shifted_sectors['front'].clear is True


def test_positive_depth_below_valid_range_still_counts_as_close_obstacle():
    depth = np.full((90, 120), 3.0, dtype=np.float32)
    depth[30:60, 50:70] = 0.10

    sectors = summarize_depth_image(depth)

    assert sectors['front'].hard_stop_fraction >= 0.02
    assert sectors['front'].route_block_fraction >= 0.02
    assert sectors['front'].clear is False


@pytest.mark.parametrize(
    'overrides',
    [
        {'roi_top_fraction': 0.9, 'roi_bottom_fraction': 0.2},
        {'min_depth_m': 2.0, 'max_depth_m': 1.0},
        {'hard_stop_distance_m': 1.5, 'route_block_distance_m': 1.0},
        {'obstacle_pixel_fraction': 0.0},
        {'minimum_valid_fraction': 1.1},
        {'clearance_percentile': 101.0},
        {
            'roi_left_fraction': 0.5,
            'left_sector_end_fraction': 0.4,
        },
    ],
)
def test_invalid_depth_settings_are_rejected(overrides):
    with pytest.raises(ValueError):
        summarize_depth_image(
            np.full((30, 30), 2.0, dtype=np.float32),
            **overrides,
        )


def test_16uc1_depth_decode_handles_row_padding():
    msg = Image()
    msg.height = 2
    msg.width = 2
    msg.encoding = '16UC1'
    msg.is_bigendian = False
    msg.step = 6
    msg.data = np.array([1000, 2000, 9999, 3000, 4000, 9999], dtype='<u2').tobytes()

    decoded = depth_image_to_meters(msg)

    np.testing.assert_allclose(decoded, [[1.0, 2.0], [3.0, 4.0]])


def test_depth_decode_rejects_misaligned_step():
    msg = Image()
    msg.height = 1
    msg.width = 1
    msg.encoding = '16UC1'
    msg.is_bigendian = False
    msg.step = 3
    msg.data = b'\x00\x00\x00'

    with pytest.raises(ValueError):
        depth_image_to_meters(msg)


def test_target_bbox_uses_depth_inside_target_instead_of_whole_sector():
    depth = np.full((100, 100), 2.5, dtype=np.float32)
    depth[20:90, 40:60] = 0.30

    target_depth = target_depth_from_bbox(
        depth,
        [0.40, 0.20, 0.60, 0.90],
        min_depth_m=0.075,
        max_depth_m=3.0,
    )

    assert target_depth == 0.30


def test_invalid_target_bbox_has_no_depth_measurement():
    depth = np.full((20, 20), 1.0, dtype=np.float32)

    assert target_depth_from_bbox(depth, [0.8, 0.2, 0.2, 0.9], 0.075, 3.0) is None


def test_masking_goal_bbox_does_not_hide_other_front_obstacles():
    depth = np.full((90, 120), 2.0, dtype=np.float32)
    depth[30:60, 50:70] = 0.30  # target bbox
    depth[30:60, 42:48] = 0.20  # separate obstacle outside target bbox

    masked = mask_bbox(depth, [50 / 120, 30 / 90, 70 / 120, 60 / 90])
    sectors = summarize_depth_image(masked)

    assert sectors['front'].hard_stop_fraction > 0.0
