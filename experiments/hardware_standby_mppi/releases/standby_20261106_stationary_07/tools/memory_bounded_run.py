#!/usr/bin/env python3
"""Run ONE offline Python check with a hard address-space and time limit."""
import argparse
import json
import os
from pathlib import Path
import resource
import subprocess
import sys
import time

RELEASE = Path(__file__).resolve().parents[1]


def available_kib():
    for line in Path('/proc/meminfo').read_text().splitlines():
        if line.startswith('MemAvailable:'):
            return int(line.split()[1])
    raise RuntimeError('MemAvailable missing')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--label', required=True)
    p.add_argument('args', nargs=argparse.REMAINDER)
    a = p.parse_args()
    if available_kib() < 3072*1024:
        raise SystemExit('Not starting: less than 3 GiB available memory')
    args = a.args[1:] if a.args[:1] == ['--'] else a.args
    if not args:
        p.error('Python arguments required')
    def limits():
        resource.setrlimit(resource.RLIMIT_AS, (1024**3, 1024**3))
        resource.setrlimit(resource.RLIMIT_CPU, (90, 95))
        resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    env = dict(os.environ, PYTHONNOUSERSITE='1', PYTHONDONTWRITEBYTECODE='1')
    for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
        env[key] = '1'
    env['PYTHONPATH'] = ':'.join(str(RELEASE/p) for p in ('runtime','core','python','tools')) + ':/opt/ros/noetic/lib/python3/dist-packages'
    start = time.monotonic()
    before = available_kib()
    result = subprocess.run(['/usr/bin/python3']+args, env=env, cwd=str(RELEASE),
                            preexec_fn=limits, timeout=120)
    summary = dict(label=a.label, exit_code=result.returncode,
                   elapsed_s=round(time.monotonic()-start, 3),
                   peak_rss_kib=resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss,
                   address_space_limit_mib=1024, single_thread_math=True,
                   available_before_kib=before, available_after_kib=available_kib())
    path = RELEASE/'reports/resource_usage.jsonl'
    path.parent.mkdir(exist_ok=True)
    with path.open('a') as f:
        f.write(json.dumps(summary)+'\n')
    print(json.dumps(summary), flush=True)
    return result.returncode


if __name__ == '__main__':
    sys.exit(main())
