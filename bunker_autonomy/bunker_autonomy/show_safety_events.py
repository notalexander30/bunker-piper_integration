#!/usr/bin/env python3

import argparse
import glob
import json
import os
from typing import Any, Dict, Optional


DEFAULT_LOG_DIRECTORY = '~/vlm_results/autonomy_logs'


def find_latest_safety_event_log(log_directory: str) -> Optional[str]:
    pattern = os.path.join(os.path.expanduser(log_directory), 'safety_events_*.jsonl')
    candidates = glob.glob(pattern)
    if not candidates:
        return None
    # Overlay filesystems can assign identical mtimes to files created in quick
    # succession. The timestamped filename provides a deterministic tie-break.
    return max(candidates, key=lambda path: (os.stat(path).st_mtime_ns, path))


def format_safety_event(row: Dict[str, Any]) -> str:
    state = 'STOP' if bool(row.get('stopped', True)) else 'CLEAR'
    timestamp = str(row.get('wall_time', 'unknown-time'))
    primary = str(row.get('primary_reason', 'unknown'))
    reasons = row.get('reasons', [])
    if not isinstance(reasons, list):
        reasons = [str(reasons)]
    reason_text = ','.join(str(reason) for reason in reasons) or 'none'
    return '%s | %s | primary=%s | reasons=%s' % (
        timestamp,
        state,
        primary,
        reason_text,
    )


def read_recent_events(path: str, limit: int = 20):
    with open(path, encoding='utf-8') as event_file:
        rows = []
        for line in event_file:
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(row, dict) and row.get('event') == 'safety_transition':
                rows.append(row)
    return rows[-max(1, int(limit)):]


def main(args=None):
    parser = argparse.ArgumentParser(description='Show recent autonomy stop triggers.')
    parser.add_argument('--log-directory', default=DEFAULT_LOG_DIRECTORY)
    parser.add_argument('--last', type=int, default=20)
    parsed = parser.parse_args(args=args)

    path = find_latest_safety_event_log(parsed.log_directory)
    if path is None:
        print('No safety event log found. Start autonomy with data logging enabled.')
        return

    print('Safety event log: %s' % path)
    for row in read_recent_events(path, parsed.last):
        print(format_safety_event(row))


if __name__ == '__main__':
    main()
