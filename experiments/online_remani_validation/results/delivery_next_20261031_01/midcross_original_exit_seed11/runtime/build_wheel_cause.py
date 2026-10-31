#!/usr/bin/env python3
"""Prepare/build a private optimizer-only, fixed-input wheel-cause diagnostic.

Parent sources are copied. Immutable parent artifacts are hardlinked, never
compiler/archive/link outputs. No historical build or production file is edited.
"""
import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess

from build_shared_local_safety import NATIVE, checked_hash, sha
from build_feasibility_stop import source_inventory
from build_urdf_spheres_b import recipes, verify_recipe, ARCHIVES, FROZEN
from build_candidate_solver_pin import check as check_parent
from run_coupled_objective import check_build, check_space
from patch_wheel_cause import transform_optimizer, transform_finish


HERE = Path(__file__).resolve().parent
BASE = HERE / 'builds/candidate_audit_20261030_03'
DEFAULT = HERE / 'builds/wheel_cause_20261030_04'
HELPERS = ('wheel_cause_replay.hpp', 'wheel_cause_replay.inc')
IMPLEMENTATION = ('build_wheel_cause.py', 'patch_wheel_cause.py', *HELPERS)
REBUILT = ('poly_traj_optimizer.cpp.o', 'libexp_opt.a', 'remani_planner_node')
RESERVE_BYTES = 300 * 1024 ** 2
BUILD_ALLOWANCE_BYTES = 12 * 1024 ** 2


def recipe(out):
    out = Path(out).resolve()
    result = recipes(out)
    result['compile_commands'] = [item for item in result['compile_commands']
                                  if item['target'] == 'exp_opt']
    result['archive_commands'] = [command for command in result['archive_commands']
                                  if command[2].endswith('/libexp_opt.a')]
    link = result['link_commands']['remani_planner_node']
    old = str(FROZEN / 'build/libexp_env.a')
    if link.count(old) != 1:
        raise ValueError('Expected exactly one environment archive')
    link[link.index(old)] = str(out / 'build/libexp_env.a')
    link.append('-lz')
    result['link_commands'] = {'remani_planner_node': link}
    outputs = [item['object'] for item in result['compile_commands']]
    outputs += [command[2] for command in result['archive_commands']]
    outputs += [command[command.index('-o') + 1]
                for command in result['link_commands'].values()]
    if sorted(outputs) != sorted(str(out / 'build' / name) for name in REBUILT):
        raise ValueError('Only optimizer object/archive and planner link may be written')
    return result


def _parent_artifacts():
    paths = sorted((BASE / 'build').iterdir())
    if any(path.is_symlink() or not path.is_file() for path in paths):
        raise ValueError('Expected only regular immutable parent build artifacts')
    if not set(REBUILT).issubset(path.name for path in paths):
        raise ValueError('Parent optimizer/planner artifact inventory is incomplete')
    return [path.name for path in paths if path.name not in REBUILT]


def _check_space(path):
    free = check_space(path, RESERVE_BYTES + BUILD_ALLOWANCE_BYTES)
    print('STORAGE free_mib=%d reserve_mib=300 build_allowance_mib=12' %
          (free // 1024 ** 2), flush=True)
    return free


def prepare(out=DEFAULT):
    out = Path(out).resolve()
    if out.exists():
        raise FileExistsError(out)
    if BASE in out.parents or out in BASE.parents:
        raise ValueError('Child must be independent of its immutable parent')
    parent = check_parent(BASE)
    if parent.get('prepared_only'):
        raise ValueError('Completed candidate-audit solver-pin parent required')
    # Check before creating any child files; no historical output is reclaimed.
    free = _check_space(HERE)
    reused = _parent_artifacts()
    result = recipe(out)
    verify_recipe(result)
    out.mkdir(parents=True)
    shutil.copytree(BASE / 'source', out / 'source')
    (out / 'build').mkdir()
    for name in HELPERS:
        shutil.copy2(HERE / name, out / 'source' / name)
    for name, transform in (('poly_traj_optimizer.cpp', transform_optimizer),
                            ('candidate_audit_finish.inc', transform_finish)):
        path = out / 'source' / name
        path.write_text(transform(path.read_text()))
    for name in reused:
        os.link(BASE / 'build' / name, out / 'build' / name)
    manifest = copy.deepcopy(parent)
    manifest.update(
        prepared_only=True, parent_build=str(BASE), source_root=str(out / 'source'),
        wheel_cause_experiment=True,
        wheel_cause_parent_manifest_sha256=sha(BASE / 'build_manifest.json'),
        wheel_cause_implementation_sha256={name: sha(HERE / name) for name in IMPLEMENTATION},
        wheel_cause_reused_artifacts_sha256={name: sha(out / 'build' / name) for name in reused},
        wheel_cause_reuse_mode='hardlinks; excluded all optimizer/archive/planner outputs',
        wheel_cause_recipe=result,
        wheel_cause_rebuilt_artifacts=list(REBUILT),
        wheel_cause_source_changes=['poly_traj_optimizer.cpp', 'candidate_audit_finish.inc', *HELPERS],
        wheel_cause_storage=dict(free_before_prepare_bytes=free, reserve_bytes=RESERVE_BYTES,
                                 build_allowance_bytes=BUILD_ALLOWANCE_BYTES,
                                 historical_files_deleted=False),
        wheel_cause_preserved=dict(native_acceptance=True, native_wheel_limits=True,
                                   boundary_pvaj=True, gear=True, collision_constraints=True,
                                   diagnostic_candidates_published=False),
        production_modified=False, production_default=False,
        scope='same-input paused native wheel-cause diagnosis; not an online performance report',
        isolated_source_sha256=source_inventory(out / 'source'))
    (out / 'build_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    check(out)
    return out


def check(out=DEFAULT):
    out = Path(out).resolve()
    manifest = json.loads((out / 'build_manifest.json').read_text())
    if not manifest.get('wheel_cause_experiment'):
        raise ValueError('Not a wheel-cause diagnostic child')
    check_parent(BASE)
    checked_hash(BASE / 'build_manifest.json', manifest['wheel_cause_parent_manifest_sha256'])
    for name, digest in manifest['wheel_cause_implementation_sha256'].items():
        checked_hash(HERE / name, digest)
    if source_inventory(out / 'source') != manifest['isolated_source_sha256']:
        raise ValueError('Child source changed')
    if recipe(out) != manifest['wheel_cause_recipe']:
        raise ValueError('Recipe changed')
    verify_recipe(manifest['wheel_cause_recipe'])
    reused = manifest['wheel_cause_reused_artifacts_sha256']
    if sorted(reused) != _parent_artifacts() or set(reused).intersection(REBUILT):
        raise ValueError('Immutable artifact inventory overlaps compiler/link outputs')
    for name, digest in reused.items():
        child, parent = out / 'build' / name, BASE / 'build' / name
        if child.is_symlink() or not child.samefile(parent):
            raise ValueError('Immutable artifact is not its recorded parent hardlink: ' + name)
        checked_hash(child, digest)
        checked_hash(parent, digest)
    if not manifest['prepared_only']:
        for name, digest in manifest['wheel_cause_compiled_artifacts_sha256'].items():
            checked_hash(out / 'build' / name, digest)
        check_build(out)
    return manifest


def compile_prepared(out=DEFAULT):
    out = Path(out).resolve()
    manifest = check(out)
    if not manifest['prepared_only']:
        raise ValueError('Already compiled; no in-place rebuild')
    _check_space(out)
    # Do not overwrite partial failed outputs or permit a reused inode as output.
    for name in REBUILT:
        path = out / 'build' / name
        if path.exists() or path.is_symlink():
            raise FileExistsError('Retain partial output; use a fresh reviewed child: ' + str(path))
    result = manifest['wheel_cause_recipe']
    for item in result['compile_commands']:
        print('COMPILE', item['target'], Path(item['source']).name, flush=True)
        subprocess.run(item['command'], check=True)
    for command in result['archive_commands']:
        subprocess.run(command, check=True)
    for command in result['link_commands'].values():
        subprocess.run(command, cwd=result['cwd'], check=True)
    check(out)
    manifest.update(
        prepared_only=False,
        native_binaries_sha256={name: sha(out / 'build' / name) for name in NATIVE},
        mppi_library_sha256=sha(out / 'build/libonline_mppi.so'),
        urdf_spheres_b_archive_sha256={name: sha(out / 'build' / name) for name in ARCHIVES},
        wheel_cause_compiled_artifacts_sha256={name: sha(out / 'build' / name) for name in REBUILT})
    (out / 'build_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    check(out)
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=DEFAULT)
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--compile', action='store_true')
    group.add_argument('--check', action='store_true')
    group.add_argument('--print-recipe', action='store_true')
    args = parser.parse_args()
    if args.print_recipe:
        print(json.dumps(recipe(args.out), indent=2))
    else:
        result = compile_prepared(args.out) if args.compile else check(args.out) if args.check else prepare(args.out)
        print('CHECKED' if args.check else result)


if __name__ == '__main__':
    main()
