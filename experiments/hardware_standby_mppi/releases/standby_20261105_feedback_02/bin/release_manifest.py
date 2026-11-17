#!/usr/bin/python3
"""Inspect or explicitly freeze this integrated release; never launch ROS."""
import argparse
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT/'config/release_manifest.json'
EXCLUDE = {'run', 'logs', '__pycache__', '.git'}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def inventory():
    result = {}
    for path in sorted(ROOT.rglob('*')):
        relative = path.relative_to(ROOT)
        if (not path.is_file() or any(part in EXCLUDE for part in relative.parts)
                or relative.parts[:2] == ('sensors', 'build')
                or path.suffix == '.pyc' or path == MANIFEST):
            continue
        result[str(relative)] = digest(path)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--write', action='store_true', help='Freeze after all collaborating integrations and tests finish')
    mode.add_argument('--verify', action='store_true', help='Compare all current release files against its frozen manifest')
    args = parser.parse_args()
    files = inventory()
    if args.verify:
        if not MANIFEST.is_file():
            raise SystemExit('This integration has not been frozen yet.')
        expected = json.loads(MANIFEST.read_text())['files']
        mismatch = sorted(key for key in set(expected)|set(files) if expected.get(key) != files.get(key))
        print(json.dumps({'frozen_files_pass': not mismatch, 'mismatch': mismatch, 'files': len(files)}, indent=2))
        if mismatch:
            raise SystemExit(1)
        return
    output = dict(schema=1, version='standby_20261105_feedback_02',
        namespace='/standby_trial_20261105_feedback_02', mode='read_only_real_feedback_and_shadow',
        hardware_motion_enabled=False, hardware_actuator_publishers=False,
        parent_release='standby_20261105_01',
        excluded_runtime_or_build_artifacts=['run/','logs/','__pycache__/','sensors/build/'],files=files)
    if args.write:
        temporary = MANIFEST.with_suffix('.tmp')
        temporary.write_text(json.dumps(output, ensure_ascii=False, indent=2)+'\n')
        temporary.replace(MANIFEST)
    print(json.dumps({'release': str(ROOT), 'files': len(files), 'manifest_written': args.write,
                      'status': 'frozen' if args.write else 'integration_in_progress_not_frozen'}))


if __name__ == '__main__':
    main()
