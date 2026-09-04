#!/usr/bin/env python3

import argparse
import glob
import json
import os
from datetime import datetime
from typing import Dict, Optional


DEFAULT_LOG_DIRECTORY = '~/vlm_results/autonomy_logs'
DEFAULT_LOG_PREFIX = 'trash_mission'


def parse_operator_label(text: str, source: str = 'unknown') -> Dict[str, str]:
    raw = str(text or '').strip()
    if ':' in raw:
        label, note = raw.split(':', 1)
        label = label.strip()
        note = note.strip()
    else:
        parts = raw.split(None, 1)
        label = parts[0].strip() if parts else ''
        note = parts[1].strip() if len(parts) > 1 else ''

    return {
        'label': label or 'unlabeled',
        'note': note,
        'raw': raw,
        'source': source,
    }


def find_latest_log_file(
    log_directory: str = DEFAULT_LOG_DIRECTORY,
    log_prefix: str = DEFAULT_LOG_PREFIX,
) -> Optional[str]:
    directory = os.path.expanduser(log_directory)
    pattern = os.path.join(directory, '%s_*.jsonl' % log_prefix)
    matches = glob.glob(pattern)
    if not matches:
        return None
    return max(matches, key=os.path.getmtime)


def build_operator_label_row(label_text: str, source: str = 'post_run_cli') -> Dict:
    return {
        'wall_time': datetime.now().isoformat(timespec='milliseconds'),
        'event': 'operator_label',
        'operator_label': parse_operator_label(label_text, source=source),
    }


def append_operator_label(log_file: str, label_text: str, source: str = 'post_run_cli') -> Dict:
    row = build_operator_label_row(label_text, source=source)
    with open(os.path.expanduser(log_file), 'a', encoding='utf-8') as handle:
        handle.write(json.dumps(row, sort_keys=True) + '\n')
    return row


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Append an operator label to an autonomy JSONL log.'
    )
    parser.add_argument(
        'label_text',
        nargs='+',
        help='Label text, for example: "bad_route: turned into left obstacle".',
    )
    parser.add_argument(
        '--log-file',
        help='Specific JSONL log file. Defaults to the newest trash_mission log.',
    )
    parser.add_argument(
        '--log-directory',
        default=DEFAULT_LOG_DIRECTORY,
        help='Directory used when --log-file is omitted.',
    )
    parser.add_argument(
        '--prefix',
        default=DEFAULT_LOG_PREFIX,
        help='Log filename prefix used when --log-file is omitted.',
    )
    args = parser.parse_args()

    label_text = ' '.join(args.label_text)
    log_file = args.log_file or find_latest_log_file(args.log_directory, args.prefix)
    if log_file is None:
        raise SystemExit(
            'No log file found in %s with prefix %s.'
            % (os.path.expanduser(args.log_directory), args.prefix)
        )

    row = append_operator_label(log_file, label_text, source='post_run_cli')
    parsed = row['operator_label']
    print(
        'Appended operator_label label=%s note=%s to %s'
        % (parsed['label'], parsed['note'], os.path.expanduser(log_file))
    )


if __name__ == '__main__':
    main()
