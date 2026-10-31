#!/usr/bin/env python3
"""Post-run directed-cursor lineage checks; never participates in control."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import struct


PATTERN = re.compile(r'\[DirectedReference\] ([^\r\n]+)')


def fnv(reference):
    value = 14695981039346656037
    for row in reference:
        for number in row:
            for byte in struct.pack('<d', 0. if number == 0. else number):
                value = ((value ^ byte) * 1099511628211) & ((1 << 64) - 1)
    return value


def phases(reference):
    gear = next((1 if r[3] > 0 else -1 for r in reference if r[3]), 1)
    result, begin = [], 0
    for i in range(1, len(reference)):
        speed = reference[i][3]
        direction = 1 if speed > 0 else -1 if speed < 0 else 0
        if direction and direction != gear:
            result.append((begin, i - 1, gear))
            begin, gear = i, direction
    result.append((begin, len(reference) - 1, gear))
    return result


def audit(trial):
    trial = Path(trial).resolve()
    references = [json.loads(line) for line in
                  (trial / 'consumed_references.jsonl').read_text().splitlines()]
    by_hash = {}
    for row in references:
        by_hash.setdefault(fnv(row['reference']), []).append(row)
    # roslaunch output=log redirects the C++ library's stdout to its host
    # Python node log, not to roslaunch's own stdout. Require one completed
    # node stream rather than merge/deduplicate unrelated ROS log copies.
    logfiles = [p for p in (trial / 'roslogs').glob('*/base_mppi-*-stdout.log')
                if not p.is_symlink() and not p.parent.is_symlink()]
    if len(logfiles) != 1:
        raise ValueError('Expected one local controller stdout log, found %d' % len(logfiles))
    logfile = logfiles[0]
    events, errors, active = [], [], None
    for match in PATTERN.finditer(logfile.read_text(errors='replace')):
        try:
            fields = dict(part.split('=', 1) for part in match.group(1).split())
            record = dict(t=float(fields['t']), event=fields['event'],
                          ref_fnv64=fields['ref_fnv64'],
                          measured=[float(n) for n in fields['measured'].split(',')],
                          checked_cusp_distance=float(fields['checked_cusp_distance']))
            record.update({k: int(fields[k]) for k in
                           ('from', 'phase', 'count', 'gear', 'begin', 'end')})
            candidates = by_hash.get(int(record['ref_fnv64'], 16), [])
            if not candidates:
                raise ValueError('cursor reference has no complete consumed-reference match')
            # FNV is telemetry only. Keep the cryptographic raw transaction
            # identity separately, and disclose duplicate telemetry matches.
            source = min(candidates, key=lambda r: abs(r['t'] - record['t']))
            record.update(reference_sha256=source['sha256'],
                          reference_revision=source['revision'],
                          telemetry_reference_matches=len(candidates))
            ranges = phases(source['reference'])
            if record['count'] != len(ranges) or not 0 <= record['phase'] < len(ranges):
                raise ValueError('phase inventory differs from the full consumed native reference')
            expected = ranges[record['phase']]
            if tuple(record[k] for k in ('begin', 'end', 'gear')) != expected:
                raise ValueError('active range/gear differs from independently reconstructed native phase')
            if not all(math.isfinite(n) for n in record['measured']) or len(record['measured']) != 5:
                raise ValueError('invalid measured state')
            if record['event'] == 'phase_start':
                if record['phase'] != 0 or record['from'] != -1:
                    raise ValueError('a new native reference must start at phase zero')
            elif record['event'] == 'phase_switch':
                if active != (record['ref_fnv64'], record['from']):
                    raise ValueError('phase switch has no matching actual predecessor event')
                if record['phase'] != record['from'] + 1:
                    raise ValueError('phase switch skipped/backtracked a direction segment')
                cusp = source['reference'][ranges[record['from']][1]]
                distance = math.hypot(record['measured'][0] - cusp[0],
                                      record['measured'][1] - cusp[1])
                if abs(distance - record['checked_cusp_distance']) > 1e-8:
                    raise ValueError('reported cusp distance differs from actual measured geometry')
                if distance > .05 + 1e-9 or abs(record['measured'][3]) > .025 + 1e-9:
                    raise ValueError('phase switch violates measured position/linear-speed gate')
            else:
                raise ValueError('unknown directed cursor event')
            active = (record['ref_fnv64'], record['phase'])
            events.append(record)
        except (ValueError, KeyError, IndexError, TypeError) as exc:
            errors.append(dict(raw=match.group(0), error=str(exc)))
    return dict(passed=bool(events) and not errors, event_count=len(events),
                switch_count=sum(e['event'] == 'phase_switch' for e in events),
                trial=str(trial), events=events, errors=errors,
                controller_stdout_path=str(logfile),
                controller_stdout_sha256=hashlib.sha256(logfile.read_bytes()).hexdigest(),
                scope='Logged actual-state phase/range/gear lineage only; no claim of execution or collision safety. Not logging a switch is not evidence that a cusp was traversed.')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--trial', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    if a.out.exists():
        raise FileExistsError('Refusing to overwrite directed audit: ' + str(a.out))
    report = audit(a.trial)
    a.out.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'events'}, indent=2))
    if not report['passed']:
        raise RuntimeError('Directed-reference audit failed: ' + str(a.out))
