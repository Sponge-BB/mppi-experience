#!/usr/bin/env python3
"""Isolated V8 horizon ablation. Never changes or installs production code."""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import shutil
import subprocess

HERE = Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build(base, out):
    base, out = base.resolve(), out.resolve()
    m = json.loads((base / 'build_manifest.json').read_text())
    assert sha(base / 'build/libonline_mppi.so') == m['mppi_library_sha256']
    old = (base / 'source/online_kernel.hpp').read_text()
    marker = 'std::min(19,int(std::floor((c.max_prediction-std::max(0.,z.t-z.stamp)-.01)/c.rdt)))'
    assert old.count(marker) == 3, 'Expected BaseMPPI, V7 and V8 horizon caps'
    new = old.replace(marker, marker.replace('std::min(19,', 'std::min(60,'))
    # No score, action primitive, geometry, acceleration, or control period changes.
    out.mkdir(parents=True, exist_ok=False)
    (out / 'source').mkdir(); (out / 'build').mkdir()
    (out / 'source/online_kernel.hpp').write_text(new)
    shutil.copy2(base / 'source/online_api.cpp', out / 'source/online_api.cpp')
    parent = Path(m['parent_build'])
    flags = (parent / 'build/CMakeFiles/online_mppi.dir/flags.make').read_text()
    inc = next(x.split('=', 1)[1].strip() for x in flags.splitlines() if x.startswith('CXX_INCLUDES ='))
    obj = out / 'build/online_api.cpp.o'
    subprocess.run(['/usr/bin/c++', '-std=gnu++17', '-O2', '-DNDEBUG', '-fPIC',
                    *shlex.split(inc), '-c', str(out / 'source/online_api.cpp'), '-o', str(obj)], check=True)
    link = shlex.split((parent / 'build/CMakeFiles/online_mppi.dir/link.txt').read_text())
    link[link.index('-o') + 1] = str(out / 'build/libonline_mppi.so')
    link = [str(obj) if x == 'CMakeFiles/online_mppi.dir/online_api.cpp.o' else x for x in link]
    subprocess.run(link, cwd=str(parent / 'build'), check=True)
    m.update(controller_revision='v8_speed_horizon_ablation', experiment_profile='no_uncertainty',
             ablation_parent=str(base), ablation_parent_sha256=sha(base / 'build/libonline_mppi.so'),
             controller_changes=['Only raise three dynamic rollout step caps from 19 to 60; max_prediction still bounds every rollout.',
                                 'No ROS executables copied; library is simulation-only.',
                                 'Inherited grid proposal risk lookahead remains 1.5 s; full candidate checking uses configured longer horizon.',
                                 'Inherited BaseMPPI sample count still trades samples against horizon; outer primitive counts unchanged.'],
             mppi_library_sha256=sha(out / 'build/libonline_mppi.so'),
             ablation_source_sha256={p.name: sha(p) for p in (out / 'source').iterdir()},
             ablation_builder_sha256=sha(Path(__file__)))
    (out / 'build_manifest.json').write_text(json.dumps(m, indent=2) + '\n')
    print(out, flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--base', type=Path, default=HERE / 'builds/mppi_v8_09')
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args(); build(a.base, a.out)
