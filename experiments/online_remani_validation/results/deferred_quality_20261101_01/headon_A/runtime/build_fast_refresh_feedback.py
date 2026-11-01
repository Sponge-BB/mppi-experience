#!/usr/bin/env python3
"""Isolated feedback trigger build; optimizer and MPPI libraries unchanged."""
import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess

import build_half_speed_contract as parent
from build_feasibility_stop import source_inventory
from build_shared_local_safety import NATIVE, checked_hash, sha
from build_urdf_spheres_b import FROZEN, _flags, recipes, verify_recipe
from run_coupled_objective import check_build

HERE = Path(__file__).resolve().parent
BASE = HERE/'builds/half_speed_contract_20261101_01'
OUT = HERE/'builds/fast_refresh_feedback_20261101_01'
FILES = ('fast_refresh_feedback.hpp', 'fast_refresh_transform.py', 'test_fast_refresh_feedback.cpp')
REBUILT = ('remani_replan_fsm.cpp.o', 'remani_planner_node')


def prepare():
    manifest = parent.check()
    OUT.mkdir(exist_ok=False)
    shutil.copytree(BASE/'source', OUT/'source')
    (OUT/'build').mkdir()
    for name in FILES:
        shutil.copy2(HERE/name, OUT/'source'/name)
    from fast_refresh_transform import transform_fsm
    path = OUT/'source/remani_replan_fsm.cpp'
    path.write_text(transform_fsm(path.read_text()))
    reused = {}
    for path in (BASE/'build').iterdir():
        if path.name not in REBUILT:
            os.link(path, OUT/'build'/path.name)
            reused[path.name] = sha(path)
    recipe = recipes(OUT)
    recipe['compile_commands'] = [c for c in recipe['compile_commands']
                                  if Path(c['source']).name == 'remani_replan_fsm.cpp']
    recipe['archive_commands'] = []
    recipe['link_commands'] = {'remani_planner_node': recipe['link_commands']['remani_planner_node']}
    link = recipe['link_commands']['remani_planner_node']
    link[link.index(str(FROZEN/'build/libexp_env.a'))] = str(OUT/'build/libexp_env.a')
    link.append('-lz')
    _, includes = _flags('remani_planner_node')
    recipe['feedback_test_command'] = ['/usr/bin/c++', '-std=gnu++17', '-O2',
        '-I'+str(OUT/'source/include'), '-I'+str(OUT/'source'), *includes,
        str(OUT/'source/test_fast_refresh_feedback.cpp'), '-L/opt/ros/noetic/lib',
        '-Wl,-rpath,/opt/ros/noetic/lib', '-lroscpp', '-lrosconsole', '-lrostime',
        '-lcpp_common', '-lroscpp_serialization', '-pthread', '-o',
        str(OUT/'build/test_fast_refresh_feedback')]
    verify_recipe(recipe)
    manifest = copy.deepcopy(manifest)
    manifest.update(prepared_only=True, source_root=str(OUT/'source'), parent_build=str(BASE),
        fast_refresh_feedback_experiment=True, fast_refresh_parent_sha256=sha(BASE/'build_manifest.json'),
        fast_refresh_reused_sha256=reused, fast_refresh_recipe=recipe,
        fast_refresh_implementation_sha256={n: sha(HERE/n) for n in ('build_fast_refresh_feedback.py', *FILES)},
        isolated_source_sha256=source_inventory(OUT/'source'), production_modified=False,
        fast_refresh_scope='add one slow-refresh request per validated dynamic avoidance episode; no change to optimizer, native gates, actual MPPI algorithm, rollout or brake validator')
    (OUT/'build_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return check()


def check():
    parent.check()
    manifest = json.loads((OUT/'build_manifest.json').read_text())
    checked_hash(BASE/'build_manifest.json', manifest['fast_refresh_parent_sha256'])
    for name, digest in manifest['fast_refresh_implementation_sha256'].items():
        checked_hash(HERE/name, digest)
    for name, digest in manifest['fast_refresh_reused_sha256'].items():
        checked_hash(OUT/'build'/name, digest)
    if source_inventory(OUT/'source') != manifest['isolated_source_sha256']:
        raise ValueError('Child sources changed')
    verify_recipe(manifest['fast_refresh_recipe'])
    if not manifest['prepared_only']:
        for name, digest in manifest['fast_refresh_compiled_sha256'].items():
            checked_hash(OUT/'build'/name, digest)
        checked_hash(OUT/'fast_refresh_check.json', manifest['fast_refresh_check_sha256'])
        check_build(OUT)
    return manifest


def compile_child():
    manifest = check()
    if not manifest['prepared_only']:
        raise RuntimeError('Completed child is immutable')
    recipe = manifest['fast_refresh_recipe']
    for command in recipe['compile_commands']:
        print('COMPILE', Path(command['source']).name, flush=True)
        subprocess.run(command['command'], cwd=recipe['cwd'], check=True)
    subprocess.run(recipe['link_commands']['remani_planner_node'], cwd=recipe['cwd'], check=True)
    subprocess.run(recipe['feedback_test_command'], cwd=recipe['cwd'], check=True)
    test = subprocess.run([str(OUT/'build/test_fast_refresh_feedback')],
                          capture_output=True, text=True, check=True)
    record = json.loads(test.stdout)
    if not record['pass']:
        raise ValueError('Feedback admission unit test failed')
    (OUT/'fast_refresh_check.json').write_text(json.dumps(record, indent=2)+'\n')
    manifest.update(prepared_only=False, native_binaries_sha256={n: sha(OUT/'build'/n) for n in NATIVE},
        fast_refresh_compiled_sha256={n: sha(OUT/'build'/n) for n in (*REBUILT, 'test_fast_refresh_feedback')},
        fast_refresh_check_sha256=sha(OUT/'fast_refresh_check.json'))
    (OUT/'build_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return check()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=('prepare', 'compile', 'check'))
    args = parser.parse_args()
    {'prepare': prepare, 'compile': compile_child, 'check': check}[args.action]()
    print('FAST_REFRESH_BUILD', args.action, OUT, 'OK', flush=True)
