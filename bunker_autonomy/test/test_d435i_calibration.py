from pathlib import Path

import numpy as np
from sensor_msgs.msg import CameraInfo

from bunker_autonomy.d435i_calibration_node import (
    camera_info_summary,
    load_view_settings,
    make_depth_overlay,
)
from bunker_autonomy.d435i_calibration_info import (
    find_latest_report,
    format_live_status,
    format_view_settings,
)


def test_calibration_loads_same_view_boundaries_as_navigation():
    config_path = Path(__file__).parents[1] / 'config' / 'autonomy_d435i.yaml'

    settings = load_view_settings(str(config_path))

    assert settings['roi_top_fraction'] == 0.20
    assert settings['roi_bottom_fraction'] == 0.82
    assert settings['left_sector_end_fraction'] == 0.333333
    assert settings['right_sector_start_fraction'] == 0.666667


def test_depth_overlay_marks_navigation_sector_boundaries():
    depth = np.full((60, 90), 2.0, dtype=np.float32)
    settings = load_view_settings('')

    overlay = make_depth_overlay(depth, settings)

    assert overlay.shape == (60, 90, 3)
    top = int(60 * settings['roi_top_fraction'])
    left_front = int(90 * settings['left_sector_end_fraction'])
    assert overlay[top, left_front].tolist() == [255, 255, 255]


def test_camera_info_report_contains_intrinsics_and_field_of_view():
    info = CameraInfo()
    info.header.frame_id = 'camera_color_optical_frame'
    info.width = 640
    info.height = 480
    info.distortion_model = 'plumb_bob'
    info.k = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [600.0, 0.0, 320.0, 0.0, 0.0, 600.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]

    summary = camera_info_summary(info)

    assert summary['frame_id'] == 'camera_color_optical_frame'
    assert summary['width'] == 640
    assert 55.0 < summary['horizontal_fov_deg'] < 57.0
    assert 43.0 < summary['vertical_fov_deg'] < 45.0


def test_calibration_info_selects_latest_report(tmp_path):
    older = tmp_path / 'd435i_calibration_20260723_100000.yaml'
    newer = tmp_path / 'd435i_calibration_20260723_110000.yaml'
    older.write_text('created_at: old\n')
    newer.write_text('created_at: new\n')

    assert find_latest_report(str(tmp_path)) == newer


def test_calibration_info_formats_view_and_live_camera_status():
    view_lines = format_view_settings(load_view_settings(''))
    status_lines = format_live_status(
        '{"camera_info":{"frame_id":"camera","width":640,"height":480,'
        '"horizontal_fov_deg":69.0,"vertical_fov_deg":42.0},'
        '"depth_size":[640,480],"sectors":{"front":{"clear":true,'
        '"clearance_m":1.2,"valid_fraction":0.9}},'
        '"report_path":"/tmp/report.yaml"}'
    )

    assert any('Active navigation view' in line for line in view_lines)
    assert any('LIVE D435i CALIBRATION READY' in line for line in status_lines)
    assert any('640x480' in line for line in status_lines)
    assert any('clearance_m=1.2' in line for line in status_lines)
