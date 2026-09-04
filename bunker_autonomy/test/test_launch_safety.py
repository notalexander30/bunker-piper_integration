import pytest

from bunker_autonomy.launch_safety import resolve_output_cmd_vel_topic


def test_dry_run_always_uses_debug_velocity_topic():
    assert resolve_output_cmd_vel_topic('dry_run') == '/cmd_vel_debug'
    assert (
        resolve_output_cmd_vel_topic('dry_run', '/cmd_vel_debug')
        == '/cmd_vel_debug'
    )


def test_dry_run_rejects_cmd_vel_or_custom_output_overrides():
    with pytest.raises(RuntimeError):
        resolve_output_cmd_vel_topic('dry_run', '/cmd_vel')
    with pytest.raises(RuntimeError):
        resolve_output_cmd_vel_topic('dry_run', '/some_other_topic')


def test_drive_mode_defaults_to_cmd_vel_and_allows_an_override():
    assert resolve_output_cmd_vel_topic('drive') == '/cmd_vel'
    assert resolve_output_cmd_vel_topic('drive', '/test_drive_output') == (
        '/test_drive_output'
    )


def test_invalid_mode_is_rejected():
    with pytest.raises(RuntimeError):
        resolve_output_cmd_vel_topic('invalid')
