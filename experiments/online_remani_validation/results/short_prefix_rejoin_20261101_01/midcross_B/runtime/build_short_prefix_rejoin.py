#!/usr/bin/env python3
"""Isolated short-prefix search-intent child; production and MPPI unchanged."""
import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess

import build_delivery_window as parent
from build_feasibility_stop import source_inventory
from build_shared_local_safety import NATIVE, checked_hash, sha
from build_urdf_spheres_b import FROZEN, _flags, recipes, verify_recipe
from run_coupled_objective import check_build

HERE = Path(__file__).resolve().parent
BASE = parent.OUT
OUT = HERE / 'builds/short_prefix_rejoin_20261101_01'
FILES = ('short_prefix_rejoin.hpp', 'short_prefix_rejoin_transform.py', 'test_short_prefix_rejoin.cpp')
REBUILT = ('planner_manager.cpp.o', 'kino_astar.cpp.o', 'libexp_search.a', 'remani_planner_node')


def prepare():
    manifest = parent.check()
    OUT.mkdir(exist_ok=False)
    shutil.copytree(BASE / 'source', OUT / 'source')
    (OUT / 'build').mkdir()
    for name in FILES:
        shutil.copy2(HERE / name, OUT / 'source' / name)
    from short_prefix_rejoin_transform import transform_manager, transform_search
    for name, transform in (('planner_manager.cpp', transform_manager), ('kino_astar.cpp', transform_search)):
        path = OUT / 'source' / name
        path.write_text(transform(path.read_text()))
    reused = {}
    for path in (BASE / 'build').iterdir():
        if path.name not in REBUILT:
            os.link(path, OUT / 'build' / path.name)
            reused[path.name] = sha(path)
    recipe = recipes(OUT)
    recipe['compile_commands'] = [row for row in recipe['compile_commands']
        if Path(row['source']).name in ('planner_manager.cpp', 'kino_astar.cpp')]
    recipe['archive_commands'] = [row for row in recipe['archive_commands'] if row[2].endswith('/libexp_search.a')]
    recipe['link_commands'] = {'remani_planner_node': recipe['link_commands']['remani_planner_node']}
    link = recipe['link_commands']['remani_planner_node']
    link[link.index(str(FROZEN / 'build/libexp_env.a'))] = str(OUT / 'build/libexp_env.a')
    link.append('-lz')
    _, includes = _flags('exp_search')
    recipe['rejoin_test_command'] = ['/usr/bin/c++', '-std=gnu++17', '-O2',
        '-I' + str(OUT / 'source/include'), '-I' + str(OUT / 'source'), *includes,
        str(OUT / 'source/test_short_prefix_rejoin.cpp'), '-L/opt/ros/noetic/lib',
        '-Wl,-rpath,/opt/ros/noetic/lib', '-lrostime', '-lcpp_common', '-lrosconsole',
        '-pthread', '-o', str(OUT / 'build/test_short_prefix_rejoin')]
    verify_recipe(recipe)
    manifest = copy.deepcopy(manifest)
    manifest.update(prepared_only=True, source_root=str(OUT / 'source'), parent_build=str(BASE),
        short_prefix_rejoin_experiment=True, rejoin_parent_sha256=sha(BASE / 'build_manifest.json'),
        rejoin_reused_sha256=reused, rejoin_recipe=recipe,
        rejoin_implementation_sha256={name: sha(HERE / name) for name in ('build_short_prefix_rejoin.py', *FILES)},
        isolated_source_sha256=source_inventory(OUT / 'source'), production_modified=False,
        rejoin_scope='Only short-prefix fallback with a fresh opposite-gear fast hint: uniform per-search switch penalty0 instead of original; all other costs, start state, native/latest and MPPI checks unchanged')
    (OUT / 'build_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return check()


def check():
    parent.check()
    manifest = json.loads((OUT / 'build_manifest.json').read_text())
    checked_hash(BASE / 'build_manifest.json', manifest['rejoin_parent_sha256'])
    for name, digest in manifest['rejoin_implementation_sha256'].items():
        checked_hash(HERE / name, digest)
    for name, digest in manifest['rejoin_reused_sha256'].items():
        checked_hash(OUT / 'build' / name, digest)
    if source_inventory(OUT / 'source') != manifest['isolated_source_sha256']:
        raise ValueError('Child source inventory changed')
    verify_recipe(manifest['rejoin_recipe'])
    if not manifest['prepared_only']:
        for name, digest in manifest['rejoin_compiled_sha256'].items():
            checked_hash(OUT / 'build' / name, digest)
        checked_hash(OUT / 'short_prefix_rejoin_check.json', manifest['rejoin_check_sha256'])
        check_build(OUT)
    return manifest


def compile_child():
    manifest = check()
    if not manifest['prepared_only']:
        raise RuntimeError('Completed child build is immutable')
    recipe = manifest['rejoin_recipe']
    for row in recipe['compile_commands']:
        print('COMPILE', Path(row['source']).name, flush=True)
        subprocess.run(row['command'], cwd=recipe['cwd'], check=True)
    for command in recipe['archive_commands']:
        subprocess.run(command, cwd=recipe['cwd'], check=True)
    subprocess.run(recipe['link_commands']['remani_planner_node'], cwd=recipe['cwd'], check=True)
    subprocess.run(recipe['rejoin_test_command'], cwd=recipe['cwd'], check=True)
    test = subprocess.run([str(OUT / 'build/test_short_prefix_rejoin')], capture_output=True, text=True, check=True)
    evidence = json.loads(test.stdout)
    if not evidence['pass']:
        raise ValueError('Search-intent checks failed')
    (OUT / 'short_prefix_rejoin_check.json').write_text(json.dumps(evidence, indent=2) + '\n')
    manifest.update(prepared_only=False, native_binaries_sha256={name: sha(OUT / 'build' / name) for name in NATIVE},
        rejoin_compiled_sha256={name: sha(OUT / 'build' / name) for name in (*REBUILT, 'test_short_prefix_rejoin')},
        rejoin_check_sha256=sha(OUT / 'short_prefix_rejoin_check.json'))
    (OUT / 'build_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return check()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('prepare', 'compile', 'check'))
    args = parser.parse_args()
    {'prepare': prepare, 'compile': compile_child, 'check': check}[args.action]()
    print('SHORT_PREFIX_REJOIN_BUILD', args.action, OUT, 'OK', flush=True)
