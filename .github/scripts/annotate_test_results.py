#!/usr/bin/env python3
"""Expose failing colcon/JUnit test names as GitHub Actions annotations."""

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


def escape(value):
    return str(value).replace('%', '%25').replace('\r', '%0D').replace('\n', '%0A')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('result_dir', type=Path)
    args = parser.parse_args()

    for result_file in args.result_dir.rglob('*.xml'):
        try:
            root = ET.parse(result_file).getroot()
        except ET.ParseError:
            continue
        for case in root.iter('testcase'):
            problem = case.find('failure')
            if problem is None:
                problem = case.find('error')
            if problem is None:
                continue
            test_name = '.'.join(filter(None, [case.get('classname'), case.get('name')]))
            details = (problem.get('message') or problem.text or 'Test failed').strip()
            print(
                '::error '
                f'title={escape(test_name)},file={escape(result_file)}::'
                f'{escape(details[:2000])}'
            )


if __name__ == '__main__':
    main()
