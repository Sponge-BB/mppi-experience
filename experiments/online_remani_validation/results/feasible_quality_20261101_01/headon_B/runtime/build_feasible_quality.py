#!/usr/bin/env python3
"""Isolated same-budget first-safe checkpoint versus feasible polishing build."""
import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess

import build_fast_refresh_feedback as parent
from build_feasibility_stop import source_inventory
from build_shared_local_safety import NATIVE, checked_hash, sha
from build_urdf_spheres_b import FROZEN, _flags, recipes, verify_recipe
from run_coupled_objective import check_build

HERE = Path(__file__).resolve().parent
BASE = parent.OUT
OUT = HERE/'builds/feasible_quality_20261101_01'
FILES = ('feasible_quality_polish.hpp', 'feasible_quality_transform.py',
         'test_feasible_quality_polish.cpp')
REBUILT = ('poly_traj_optimizer.cpp.o', 'libexp_opt.a', 'remani_planner_node')


def prepare():
    manifest = parent.check()
    OUT.mkdir(exist_ok=False)
    shutil.copytree(BASE/'source', OUT/'source')
    (OUT/'build').mkdir()
    for name in FILES:
        shutil.copy2(HERE/name, OUT/'source'/name)
    from feasible_quality_transform import transform_gate, transform_optimizer
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
    recipe['quality_test_command'] = ['/usr/bin/c++', '-std=gnu++17', '-O2',
        '-I'+str(OUT/'source/include'), '-I'+str(OUT/'source'), *includes,
        str(OUT/'source/test_feasible_quality_polish.cpp'), '-o',
        str(OUT/'build/test_feasible_quality_polish')]
    verify_recipe(recipe)
    manifest = copy.deepcopy(manifest)
    manifest.update(prepared_only=True, source_root=str(OUT/'source'), parent_build=str(BASE),
        feasible_quality_experiment=True, quality_parent_sha256=sha(BASE/'build_manifest.json'),
        quality_reused_sha256=reused, quality_recipe=recipe,
        quality_implementation_sha256={n: sha(HERE/n) for n in ('build_feasible_quality.py', *FILES)},
        isolated_source_sha256=source_inventory(OUT/'source'), production_modified=False,
        quality_scope='same-budget retain first native-safe accepted iterate; optional lower-original-cost native-safe replacements; original gates, objective, MPPI and production unchanged')
    (OUT/'build_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return check()


def check():
    parent.check()
    manifest = json.loads((OUT/'build_manifest.json').read_text())
    checked_hash(BASE/'build_manifest.json', manifest['quality_parent_sha256'])
    for name, digest in manifest['quality_implementation_sha256'].items():
        checked_hash(HERE/name, digest)
    for name, digest in manifest['quality_reused_sha256'].items():
        checked_hash(OUT/'build'/name, digest)
    if source_inventory(OUT/'source') != manifest['isolated_source_sha256']:
        raise ValueError('Child sources changed')
    verify_recipe(manifest['quality_recipe'])
    if not manifest['prepared_only']:
        for name, digest in manifest['quality_compiled_sha256'].items():
            checked_hash(OUT/'build'/name, digest)
        checked_hash(OUT/'quality_check.json', manifest['quality_check_sha256'])
        check_build(OUT)
    return manifest


def compile_child():
    manifest = check()
    if not manifest['prepared_only']:
        raise RuntimeError('Completed child is immutable')
    recipe = manifest['quality_recipe']
    for command in recipe['compile_commands']:
        print('COMPILE', Path(command['source']).name, flush=True)
        subprocess.run(command['command'], cwd=recipe['cwd'], check=True)
    for command in recipe['archive_commands']:
        subprocess.run(command, cwd=recipe['cwd'], check=True)
    subprocess.run(recipe['link_commands']['remani_planner_node'], cwd=recipe['cwd'], check=True)
    subprocess.run(recipe['quality_test_command'], cwd=recipe['cwd'], check=True)
    test = subprocess.run([str(OUT/'build/test_feasible_quality_polish')],
                          capture_output=True, text=True, check=True)
    record = json.loads(test.stdout)
    if not record['pass']:
        raise ValueError('Feasible quality checkpoint test failed')
    (OUT/'quality_check.json').write_text(json.dumps(record, indent=2)+'\n')
    manifest.update(prepared_only=False, native_binaries_sha256={n: sha(OUT/'build'/n) for n in NATIVE},
        quality_compiled_sha256={n: sha(OUT/'build'/n) for n in (*REBUILT, 'test_feasible_quality_polish')},
        quality_check_sha256=sha(OUT/'quality_check.json'))
    (OUT/'build_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return check()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('prepare', 'compile', 'check'))
    args = parser.parse_args()
    {'prepare': prepare, 'compile': compile_child, 'check': check}[args.action]()
    print('FEASIBLE_QUALITY_BUILD', args.action, OUT, 'OK', flush=True)
