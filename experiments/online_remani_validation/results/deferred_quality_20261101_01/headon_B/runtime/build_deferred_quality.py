#!/usr/bin/env python3
"""Immutable SIL child for periodic versus one deferred candidate check."""
import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess

import build_feasible_quality_guard as parent
from build_feasibility_stop import source_inventory
from build_shared_local_safety import NATIVE, checked_hash, sha
from build_urdf_spheres_b import FROZEN, _flags, recipes, verify_recipe
from run_coupled_objective import check_build

HERE = Path(__file__).resolve().parent
BASE = parent.OUT
OUT = HERE/'builds/deferred_quality_20261101_01'
FILES = ('deferred_quality.hpp', 'deferred_quality_transform.py', 'test_deferred_quality.cpp')
REBUILT = ('poly_traj_optimizer.cpp.o', 'libexp_opt.a', 'remani_planner_node')


def prepare():
    manifest = parent.check()
    OUT.mkdir(exist_ok=False)
    shutil.copytree(BASE/'source', OUT/'source')
    (OUT/'build').mkdir()
    for name in FILES:
        shutil.copy2(HERE/name, OUT/'source'/name)
    from deferred_quality_transform import transform_gate, transform_optimizer
    for name, transform in (('early_delivery_gate.hpp', transform_gate),
                            ('poly_traj_optimizer.cpp', transform_optimizer)):
        path = OUT/'source'/name
        path.write_text(transform(path.read_text()))
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
    recipe['deferred_test_command'] = ['/usr/bin/c++', '-std=gnu++17', '-O2',
        '-I'+str(OUT/'source/include'), '-I'+str(OUT/'source'), *includes,
        str(OUT/'source/test_deferred_quality.cpp'), '-L/opt/ros/noetic/lib',
        '-Wl,-rpath,/opt/ros/noetic/lib', '-lrostime', '-lcpp_common', '-lrosconsole',
        '-pthread', '-o', str(OUT/'build/test_deferred_quality')]
    verify_recipe(recipe)
    manifest = copy.deepcopy(manifest)
    manifest.update(prepared_only=True, source_root=str(OUT/'source'), parent_build=str(BASE),
        deferred_quality_experiment=True, deferred_parent_sha256=sha(BASE/'build_manifest.json'),
        deferred_reused_sha256=reused, deferred_recipe=recipe,
        deferred_implementation_sha256={n: sha(HERE/n) for n in ('build_deferred_quality.py', *FILES)},
        isolated_source_sha256=source_inventory(OUT/'source'), production_modified=False,
        deferred_scope='q1 first-safe identical; q>1 same-budget periodic vs deferred native check; .15s reserve inside 1.2s; first safe fallback; original final/latest gates and MPPI retained')
    (OUT/'build_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return check()


def check():
    parent.check()
    manifest = json.loads((OUT/'build_manifest.json').read_text())
    checked_hash(BASE/'build_manifest.json', manifest['deferred_parent_sha256'])
    for name, digest in manifest['deferred_implementation_sha256'].items():
        checked_hash(HERE/name, digest)
    for name, digest in manifest['deferred_reused_sha256'].items():
        checked_hash(OUT/'build'/name, digest)
    if source_inventory(OUT/'source') != manifest['isolated_source_sha256']:
        raise ValueError('Child sources changed')
    verify_recipe(manifest['deferred_recipe'])
    if not manifest['prepared_only']:
        for name, digest in manifest['deferred_compiled_sha256'].items():
            checked_hash(OUT/'build'/name, digest)
        checked_hash(OUT/'deferred_check.json', manifest['deferred_check_sha256'])
        check_build(OUT)
    return manifest


def compile_child():
    manifest = check()
    if not manifest['prepared_only']:
        raise RuntimeError('Completed child is immutable')
    recipe = manifest['deferred_recipe']
    for command in recipe['compile_commands']:
        print('COMPILE', Path(command['source']).name, flush=True)
        subprocess.run(command['command'], cwd=recipe['cwd'], check=True)
    for command in recipe['archive_commands']:
        subprocess.run(command, cwd=recipe['cwd'], check=True)
    subprocess.run(recipe['link_commands']['remani_planner_node'], cwd=recipe['cwd'], check=True)
    subprocess.run(recipe['deferred_test_command'], cwd=recipe['cwd'], check=True)
    test = subprocess.run([str(OUT/'build/test_deferred_quality')],
                          capture_output=True, text=True, check=True)
    record = json.loads(test.stdout)
    if not record['pass']:
        raise ValueError('Deferred accepted-candidate routing test failed')
    (OUT/'deferred_check.json').write_text(json.dumps(record, indent=2)+'\n')
    manifest.update(prepared_only=False, native_binaries_sha256={n: sha(OUT/'build'/n) for n in NATIVE},
        deferred_compiled_sha256={n: sha(OUT/'build'/n) for n in (*REBUILT, 'test_deferred_quality')},
        deferred_check_sha256=sha(OUT/'deferred_check.json'))
    (OUT/'build_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return check()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('prepare', 'compile', 'check'))
    args = parser.parse_args()
    {'prepare': prepare, 'compile': compile_child, 'check': check}[args.action]()
    print('DEFERRED_QUALITY_BUILD', args.action, OUT, 'OK', flush=True)
