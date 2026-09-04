import json

from bunker_autonomy.show_safety_events import (
    find_latest_safety_event_log,
    format_safety_event,
    read_recent_events,
)


def test_format_safety_event_is_short_and_attributed():
    line = format_safety_event(
        {
            'wall_time': '2026-07-14T10:00:00.000',
            'stopped': True,
            'primary_reason': 'depth_stale',
            'reasons': ['depth_stale', 'lidar_obstacle'],
        }
    )

    assert line == (
        '2026-07-14T10:00:00.000 | STOP | primary=depth_stale | '
        'reasons=depth_stale,lidar_obstacle'
    )


def test_latest_log_and_recent_event_filtering(tmp_path):
    old = tmp_path / 'safety_events_20260714_090000.jsonl'
    new = tmp_path / 'safety_events_20260714_100000.jsonl'
    old.write_text('{}\n')
    rows = [
        {'event': 'sample'},
        {
            'event': 'safety_transition',
            'stopped': True,
            'primary_reason': 'lidar_stale',
        },
        {
            'event': 'safety_transition',
            'stopped': False,
            'primary_reason': 'clear',
        },
    ]
    new.write_text('\n'.join(json.dumps(row) for row in rows) + '\n')

    assert find_latest_safety_event_log(str(tmp_path)) == str(new)
    events = read_recent_events(str(new), limit=1)
    assert len(events) == 1
    assert events[0]['primary_reason'] == 'clear'
