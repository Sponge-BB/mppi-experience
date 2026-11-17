#!/usr/bin/env python3
"""Run already-built variants serially and compare EVERY uncapped double."""
import os
os.environ.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MALLOC_ARENA_MAX='2')
import argparse
import hashlib
import json
from pathlib import Path
import statistics
import subprocess
import sys
import numpy as np

CASE = Path(__file__).resolve().parent


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as source:
        for block in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def verify_list(path):
    for line in path.read_text().splitlines():
        digest, filename = line.split(maxsplit=1)
        if sha(Path(filename)) != digest:
            raise RuntimeError('Source/binary changed since build: ' + filename)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--threads', type=int, choices=(1, 2, 3), default=3)
    parser.add_argument('--repeats', type=int, choices=range(1, 11), default=3)
    parser.add_argument('--label', default='final')
    args = parser.parse_args()
    if not args.label.replace('-', '').replace('_', '').isalnum():
        raise ValueError('Label must contain only letters, digits, - or _')
    fixtures = CASE / 'fixtures'
    metadata = json.loads((fixtures / 'metadata.json').read_text())
    for frame in metadata['frames']:
        if sha(fixtures / frame['file']) != frame['sha256']:
            raise RuntimeError('Fixture changed: ' + frame['name'])
    run = CASE / ('runs_' + args.label)
    run.mkdir(exist_ok=False)
    results = {}
    for variant in ('before', 'after'):
        verify_list(CASE / 'build' / (variant + '.inputs.after.sha256'))
        verify_list(CASE / 'build' / (variant + '.executable.sha256'))
        output = run / variant
        output.mkdir()
        with (run / (variant + '.jsonl')).open('w') as log, (run / (variant + '.stderr')).open('w') as err:
            subprocess.run(['nice', '-n', '10', str(CASE / 'build' / (variant + '-worker-benchmark')),
                            str(fixtures), str(args.repeats), str(args.threads), str(output)],
                           stdout=log, stderr=err, check=True, timeout=180)
        verify_list(CASE / 'build' / (variant + '.inputs.after.sha256'))
        rows = [json.loads(line) for line in (run / (variant + '.jsonl')).read_text().splitlines()]
        if len(rows) != len(metadata['frames']) * args.repeats:
            raise RuntimeError('Incomplete timing output: ' + variant)
        per_frame = {}
        for frame in metadata['frames']:
            selected = [row for row in rows if row['frame'] == frame['name']]
            per_frame[frame['name']] = dict(group=frame['group'], samples=len(selected),
                median_compute_ms=statistics.median(row['compute_ms'] for row in selected),
                median_submit_wait_ms=statistics.median(row['submit_wait_ms'] for row in selected))
        results[variant] = dict(peak_rss_mib=max(row['peak_rss_kib'] for row in rows)/1024.,
                               per_frame=per_frame,
                               source_manifest=(CASE/'build'/(variant+'.inputs.after.sha256')).read_text(),
                               binary_manifest=(CASE/'build'/(variant+'.executable.sha256')).read_text())
    comparisons = []
    failed = False
    for frame in metadata['frames']:
        name, count = frame['name'], frame['voxels']
        paths = [run / variant / (name + '.distance.f64') for variant in ('before', 'after')]
        if any(path.stat().st_size != count * 8 for path in paths):
            raise RuntimeError('Raw field size mismatch: ' + name)
        a, b = [np.memmap(path, dtype=np.float64, mode='r', shape=(count,)) for path in paths]
        mismatch = nonfinite = 0
        maximum = 0.
        first = []
        for begin in range(0, count, 65536):
            av, bv = a[begin:begin+65536], b[begin:begin+65536]
            nonfinite += int(np.count_nonzero(~np.isfinite(av) | ~np.isfinite(bv)))
            bad = np.flatnonzero(av != bv)
            mismatch += int(bad.size)
            maximum = max(maximum, float(np.max(np.abs(av - bv))))
            first.extend(dict(index=begin+int(i), before=float(av[i]), after=float(bv[i]))
                         for i in bad[:max(0, 5-len(first))])
        comparisons.append(dict(frame=name, group=frame['group'], voxels=count,
                                uncapped_mismatch_count=mismatch, nonfinite_count=nonfinite,
                                max_abs_difference=maximum, first_mismatches=first,
                                before_sha256=sha(paths[0]), after_sha256=sha(paths[1])))
        failed |= bool(mismatch or nonfinite)
        del a, b, av, bv
    summary = dict(threads=args.threads, repeats=args.repeats, source_sha256_matches=True,
                   native_uncapped_exact_match=not failed, comparisons=comparisons, variants=results,
                   timing_scope='Worker computation/submit-wait only; excludes fixture IO, fusion, GridMap commit/export, ROS and MPPI.',
                   real_scope=metadata['real_scope'], synthetic_scope=metadata['synthetic_scope'])
    (run / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(dict(output=str(run), exact_match=not failed,
                         compared_voxels=sum(frame['voxels'] for frame in metadata['frames']),
                         peak_rss_mib={k:v['peak_rss_mib'] for k,v in results.items()})))
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
