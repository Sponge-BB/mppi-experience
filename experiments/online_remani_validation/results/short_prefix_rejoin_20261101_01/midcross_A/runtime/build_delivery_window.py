#!/usr/bin/env python3
"""Isolated slow CV coverage extension; global prediction limits stay fixed."""
import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess

import build_deferred_quality as parent
from build_feasibility_stop import source_inventory
from build_shared_local_safety import NATIVE, checked_hash, sha
from build_urdf_spheres_b import FROZEN, _flags, recipes, verify_recipe
from run_coupled_objective import check_build

HERE = Path(__file__).resolve().parent
BASE = parent.OUT
OUT = HERE/'builds/delivery_window_20261101_01'
FILES = ('delivery_window.hpp', 'delivery_window_transform.py', 'test_delivery_window.cpp')
REBUILT = ('poly_traj_optimizer.cpp.o', 'libexp_opt.a', 'remani_planner_node')


def prepare():
    manifest = parent.check()
    OUT.mkdir(exist_ok=False)
    shutil.copytree(BASE/'source', OUT/'source')
    (OUT/'build').mkdir()
    for name in FILES:
        shutil.copy2(HERE/name, OUT/'source'/name)
    from delivery_window_transform import transform_coverage
    path = OUT/'source/coverage_collision.hpp'
    path.write_text(transform_coverage(path.read_text()))
    reused = {}
    for path in (BASE/'build').iterdir():
        if path.name not in REBUILT:
            os.link(path, OUT/'build'/path.name)
            reused[path.name] = sha(path)
    recipe = recipes(OUT)
    recipe['compile_commands'] = [c for c in recipe['compile_commands']
                                  if Path(c['source']).name == 'poly_traj_optimizer.cpp']
    recipe['archive_commands'] = [c for c in recipe['archive_commands']
                                  if c[2].endswith('/libexp_opt.a')]
    recipe['link_commands'] = {'remani_planner_node': recipe['link_commands']['remani_planner_node']}
    link = recipe['link_commands']['remani_planner_node']
    link[link.index(str(FROZEN/'build/libexp_env.a'))] = str(OUT/'build/libexp_env.a')
    link.append('-lz')
    _, includes = _flags('exp_opt')
    recipe['window_test_command'] = ['/usr/bin/c++', '-std=gnu++17', '-O2',
        '-I'+str(OUT/'source/include'), '-I'+str(OUT/'source'), *includes,
        str(OUT/'source/test_delivery_window.cpp'), '-L/opt/ros/noetic/lib',
        '-Wl,-rpath,/opt/ros/noetic/lib', '-lrostime', '-lcpp_common', '-lrosconsole',
        '-pthread', '-o', str(OUT/'build/test_delivery_window')]
    verify_recipe(recipe)
    manifest = copy.deepcopy(manifest)
    manifest.update(prepared_only=True, source_root=str(OUT/'source'), parent_build=str(BASE),
        slow_delivery_window_experiment=True, delivery_window_parent_sha256=sha(BASE/'build_manifest.json'),
        delivery_window_reused_sha256=reused, delivery_window_recipe=recipe,
        delivery_window_implementation_sha256={n: sha(HERE/n) for n in ('build_delivery_window.py', *FILES)},
        isolated_source_sha256=source_inventory(OUT/'source'), production_modified=False,
        delivery_window_scope='refresh-only frozen slow CV model extension by 1.5s; soft/native share extended copy; original observation and trajectory epochs, global predictor, MPPI and latest gates unchanged')
    (OUT/'build_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return check()


def check():
    parent.check()
    manifest = json.loads((OUT/'build_manifest.json').read_text())
    checked_hash(BASE/'build_manifest.json', manifest['delivery_window_parent_sha256'])
    for name, digest in manifest['delivery_window_implementation_sha256'].items():
        checked_hash(HERE/name, digest)
    for name, digest in manifest['delivery_window_reused_sha256'].items():
        checked_hash(OUT/'build'/name, digest)
    if source_inventory(OUT/'source') != manifest['isolated_source_sha256']:
        raise ValueError('Child sources changed')
    verify_recipe(manifest['delivery_window_recipe'])
    if not manifest['prepared_only']:
        for name, digest in manifest['delivery_window_compiled_sha256'].items():
            checked_hash(OUT/'build'/name, digest)
        checked_hash(OUT/'delivery_window_check.json', manifest['delivery_window_check_sha256'])
        check_build(OUT)
    return manifest


def compile_child():
    manifest = check()
    if not manifest['prepared_only']:
        raise RuntimeError('Completed child is immutable')
    recipe = manifest['delivery_window_recipe']
    for command in recipe['compile_commands']:
        print('COMPILE', Path(command['source']).name, flush=True)
        subprocess.run(command['command'], cwd=recipe['cwd'], check=True)
    for command in recipe['archive_commands']:
        subprocess.run(command, cwd=recipe['cwd'], check=True)
    subprocess.run(recipe['link_commands']['remani_planner_node'], cwd=recipe['cwd'], check=True)
    subprocess.run(recipe['window_test_command'], cwd=recipe['cwd'], check=True)
    test = subprocess.run([str(OUT/'build/test_delivery_window')], capture_output=True, text=True, check=True)
    record = json.loads(test.stdout)
    if not record['pass']:
        raise ValueError('Slow delivery window coverage check failed')
    (OUT/'delivery_window_check.json').write_text(json.dumps(record, indent=2)+'\n')
    manifest.update(prepared_only=False, native_binaries_sha256={n: sha(OUT/'build'/n) for n in NATIVE},
        delivery_window_compiled_sha256={n: sha(OUT/'build'/n) for n in (*REBUILT, 'test_delivery_window')},
        delivery_window_check_sha256=sha(OUT/'delivery_window_check.json'))
    (OUT/'build_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return check()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('prepare', 'compile', 'check'))
    args = parser.parse_args()
    {'prepare': prepare, 'compile': compile_child, 'check': check}[args.action]()
    print('DELIVERY_WINDOW_BUILD', args.action, OUT, 'OK', flush=True)
