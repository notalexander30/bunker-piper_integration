import json

from bunker_autonomy.label_autonomy_run import (
    append_operator_label,
    find_latest_log_file,
    parse_operator_label,
)


def test_parse_operator_label_with_reason():
    parsed = parse_operator_label(
        'bad_route: turned left when front was open',
        source='test',
    )

    assert parsed == {
        'label': 'bad_route',
        'note': 'turned left when front was open',
        'raw': 'bad_route: turned left when front was open',
        'source': 'test',
    }


def test_parse_operator_label_without_reason():
    parsed = parse_operator_label('good', source='test')

    assert parsed['label'] == 'good'
    assert parsed['note'] == ''
    assert parsed['raw'] == 'good'


def test_append_operator_label_writes_jsonl_row(tmp_path):
    log_file = tmp_path / 'trash_mission_20260709_120000.jsonl'

    row = append_operator_label(
        str(log_file),
        'unsafe_close: passed too near person',
        source='test_cli',
    )

    written = json.loads(log_file.read_text().strip())
    assert written == row
    assert written['event'] == 'operator_label'
    assert written['operator_label']['label'] == 'unsafe_close'
    assert written['operator_label']['note'] == 'passed too near person'
    assert written['operator_label']['source'] == 'test_cli'


def test_find_latest_log_file_uses_prefix(tmp_path):
    old_log = tmp_path / 'trash_mission_20260709_120000.jsonl'
    new_log = tmp_path / 'trash_mission_20260709_121000.jsonl'
    other_log = tmp_path / 'other_20260709_122000.jsonl'
    old_log.write_text('{}\n')
    new_log.write_text('{}\n')
    other_log.write_text('{}\n')

    old_time = 100.0
    new_time = 200.0
    other_time = 300.0
    old_log.touch()
    new_log.touch()
    other_log.touch()
    import os

    os.utime(old_log, (old_time, old_time))
    os.utime(new_log, (new_time, new_time))
    os.utime(other_log, (other_time, other_time))

    assert find_latest_log_file(str(tmp_path), 'trash_mission') == str(new_log)
