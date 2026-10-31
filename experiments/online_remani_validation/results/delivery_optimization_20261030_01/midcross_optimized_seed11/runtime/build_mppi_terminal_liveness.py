#!/usr/bin/env python3
"""Prepare/compile one isolated, reversible MPPI terminal-yaw branch fix.

Only online_api.cpp is compiled and libonline_mppi.so is linked.  All native
REMANI/controller/detector binaries and the two native archives are byte-reused.
Default action is preparation only; no ROS, hardware or shutdown is started.
"""
import argparse
import copy
import json
from pathlib import Path
import shutil
import subprocess

from build_shared_local_safety import NATIVE, checked_hash, compile_recipe, sha
from patch_mppi_terminal_phase import BASE_SHA256, SWITCH, project_off, transform_mppi
from run_coupled_objective import check_build
from run_dual_timescale import verify_production


HERE = Path(__file__).resolve().parent
BASE = HERE / 'builds/seed_parameterization_20261025_01'
ARCHIVES = ('libexp_opt.a', 'libexp_search.a')
PATCH_FILE = 'mppi_v12.hpp'


def source_inventory(directory):
    inventory = {}
    for path in directory.rglob('*'):
        if path.is_symlink():
            raise ValueError('Symlink in immutable source inventory: ' + str(path))
        if path.is_file():
            inventory[path.relative_to(directory).as_posix()] = sha(path)
    return inventory


def validate(base):
    base = Path(base).resolve()
    if base != BASE.resolve():
        raise ValueError('This controlled patch requires the frozen seed_parameterization_20261025_01 parent')
    manifest, _ = check_build(base)
    if not manifest.get('seed_parameterization_experiment') or manifest.get('prepared_only'):
        raise ValueError('Completed seed-parameterization parent required')
    sources = manifest.get('isolated_source_sha256', {})
    if not sources or source_inventory(base / 'source') != sources:
        raise ValueError('Parent complete source inventory differs from manifest')
    checked_hash(base / 'source' / PATCH_FILE, BASE_SHA256)
    native = manifest.get('native_binaries_sha256', {})
    if set(native) != set(NATIVE):
        raise ValueError('Parent must attest exactly three native binaries')
    for name in NATIVE:
        checked_hash(base / 'build' / name, native[name])
    checked_hash(base / 'build/libonline_mppi.so', manifest['mppi_library_sha256'])
    archives = manifest.get('seed_parameterization_archive_sha256', {})
    if set(archives) != set(ARCHIVES):
        raise ValueError('Parent must attest both native archives')
    for name in ARCHIVES:
        checked_hash(base / 'build' / name, archives[name])
    verify_production(base)
    return manifest


def compile_commands(parent, out):
    """Read the inherited recipe, explicitly include cstdlib outside namespaces."""
    parent, out = Path(parent).resolve(), Path(out).resolve()
    recipe = compile_recipe(parent, out)
    compile_cmd = list(recipe['compile_command'])
    compile_cmd[1:1] = ['-include', 'cstdlib']
    link_cmd = [recipe['expected_object'] if item == recipe['old_object'] else item
                for item in recipe['link']]
    if link_cmd.count(recipe['expected_object']) != 1:
        raise ValueError('MPPI link must use exactly one newly compiled API object')
    link_cmd[link_cmd.index('-o')+1] = str(out / 'build/libonline_mppi.so')
    return dict(recipe=recipe, compile_command=compile_cmd, link_command=link_cmd,
                cwd=str(parent / 'build'))


def validate_recipe(commands):
    recipe = commands['recipe']
    checked_hash(Path(recipe['parent_flags_path']), recipe['flags_sha256'])
    checked_hash(Path(recipe['parent_link_path']), recipe['link_recipe_sha256'])
    for name, digest in recipe['parent_link_archives_sha256'].items():
        checked_hash(Path(name), digest)


def verify_reuse(out, manifest):
    for name, digest in manifest['mppi_terminal_liveness_parent_native_sha256'].items():
        checked_hash(out / 'build' / name, digest)
    for name, digest in manifest['mppi_terminal_liveness_parent_archive_sha256'].items():
        checked_hash(out / 'build' / name, digest)


def prepare(base, out):
    raw_out = Path(out)
    if raw_out.is_symlink():
        raise ValueError('Output may not be a symlink')
    base, out = Path(base).resolve(), raw_out.resolve()
    manifest = validate(base)
    parent = Path(manifest['parent_build']).resolve()
    for frozen in (base, parent):
        if out == frozen or frozen in out.parents or out in frozen.parents:
            raise ValueError('Output must be independent of immutable builds')
    if out.exists():
        raise FileExistsError('Refusing to overwrite an existing build: ' + str(out))
    original = (base / 'source' / PATCH_FILE).read_text()
    modified = transform_mppi(original)
    if project_off(modified) != original:
        raise ValueError('Exact OFF projection failed')
    manifest_hash = sha(base / 'build_manifest.json')
    commands = compile_commands(parent, out)
    validate_recipe(commands)
    out.mkdir(parents=True, exist_ok=False)
    (out / 'build').mkdir()
    shutil.copytree(base / 'source', out / 'source')
    (out / 'source' / PATCH_FILE).write_text(modified)
    # Deliberately do not copy loose objects, CMake caches or whole build trees.
    # The old MPPI library is only a preparation placeholder; it is replaced
    # before prepared_only may become false.
    for name in (*NATIVE, *ARCHIVES, 'libonline_mppi.so'):
        shutil.copy2(base / 'build' / name, out / 'build' / name)
    sources = source_inventory(out / 'source')
    expected_sources = dict(manifest['isolated_source_sha256'])
    expected_sources[PATCH_FILE] = sha(out / 'source' / PATCH_FILE)
    if sources != expected_sources:
        raise ValueError('Prepared source differs outside the one declared header')
    result = copy.deepcopy(manifest)
    result.update(
        prepared_only=True,
        mppi_terminal_liveness_experiment=True,
        mppi_terminal_phase_experiment=True,
        mppi_terminal_liveness_prepared_only=True,
        mppi_terminal_liveness_parent=str(base),
        mppi_terminal_liveness_parent_manifest_sha256=manifest_hash,
        mppi_terminal_liveness_parent_native_sha256=dict(manifest['native_binaries_sha256']),
        mppi_terminal_liveness_parent_archive_sha256=dict(manifest['seed_parameterization_archive_sha256']),
        mppi_terminal_liveness_parent_mppi_sha256=manifest['mppi_library_sha256'],
        mppi_terminal_liveness_parent_source_sha256=dict(manifest['isolated_source_sha256']),
        mppi_terminal_liveness_parent_policy_unchanged=manifest.get('mppi_policy_unchanged'),
        mppi_terminal_liveness_builder_sha256=sha(Path(__file__)),
        mppi_terminal_liveness_patcher_sha256=sha(HERE / 'patch_mppi_terminal_phase.py'),
        mppi_terminal_liveness_compile_commands=commands,
        mppi_terminal_liveness_exact_off_projection=True,
        mppi_terminal_liveness_scope='fixed-arm numerical SIL dynamic-history terminal-yaw branch only; not Tracker, Gazebo, hardware or all MPPI liveness',
        terminal_phase_fix_switch=SWITCH,
        terminal_phase_fix_default=False,
        terminal_phase_fix_read_once_at_construction=True,
        mppi_policy_unchanged=False,
        mppi_terminal_liveness_policy_change='ON removes predicted-capture final-yaw override only; existing attainable connection-yaw branch remains',
        mppi_terminal_liveness_preserved=dict(
            weights=True, thresholds=True, proposals=True, candidate_generation=True,
            legacy_behavior=True, safety_checks=True, geometry=True, margin=True,
            wheel_limits=True, remani_binary=True, search_and_optimizer_archives=True),
        only_local_mppi_recompiled=True,
        production_modified=False,
        production_default=False,
        isolated_source_sha256=sources)
    verify_reuse(out, result)
    checked_hash(out / 'build/libonline_mppi.so', manifest['mppi_library_sha256'])
    if sha(base / 'build_manifest.json') != manifest_hash:
        raise ValueError('Parent manifest changed during preparation')
    validate(base)
    (out / 'build_manifest.json').write_text(json.dumps(result, indent=2) + '\n')
    print('PREPARED MPPI TERMINAL LIVENESS', out, flush=True)
    return out


def compile_prepared(out):
    out = Path(out).resolve()
    manifest = json.loads((out / 'build_manifest.json').read_text())
    if (not manifest.get('mppi_terminal_liveness_experiment') or
            not manifest.get('prepared_only') or
            not manifest.get('mppi_terminal_liveness_prepared_only')):
        raise ValueError('Fresh prepared terminal-liveness build required')
    base = Path(manifest['mppi_terminal_liveness_parent']).resolve()
    parent_manifest = validate(base)
    checked_hash(base / 'build_manifest.json', manifest['mppi_terminal_liveness_parent_manifest_sha256'])
    checked_hash(Path(__file__), manifest['mppi_terminal_liveness_builder_sha256'])
    checked_hash(HERE / 'patch_mppi_terminal_phase.py', manifest['mppi_terminal_liveness_patcher_sha256'])
    if source_inventory(out / 'source') != manifest['isolated_source_sha256']:
        raise ValueError('Prepared source inventory drift')
    if project_off((out / 'source' / PATCH_FILE).read_text()) != (base / 'source' / PATCH_FILE).read_text():
        raise ValueError('Prepared patch is not exact/reversible')
    verify_reuse(out, manifest)
    commands = compile_commands(Path(parent_manifest['parent_build']), out)
    if commands != manifest['mppi_terminal_liveness_compile_commands']:
        raise ValueError('Compile/link recipe changed since preparation')
    validate_recipe(commands)
    subprocess.run(commands['compile_command'], check=True)
    subprocess.run(commands['link_command'], cwd=commands['cwd'], check=True)
    # Completion is published only after revalidating all frozen inputs.
    validate(base)
    checked_hash(base / 'build_manifest.json', manifest['mppi_terminal_liveness_parent_manifest_sha256'])
    validate_recipe(commands)
    verify_reuse(out, manifest)
    if source_inventory(out / 'source') != manifest['isolated_source_sha256']:
        raise ValueError('Prepared sources changed while compiling')
    mppi_hash = sha(out / 'build/libonline_mppi.so')
    if mppi_hash == manifest['mppi_terminal_liveness_parent_mppi_sha256']:
        raise ValueError('New MPPI library unexpectedly equals the unpatched parent')
    manifest.update(
        prepared_only=False, mppi_terminal_liveness_prepared_only=False,
        controller_revision=parent_manifest['controller_revision'] + '_terminal_phase_switch',
        execution_parent=str(base),
        native_binaries_sha256=dict(parent_manifest['native_binaries_sha256']),
        mppi_library_sha256=mppi_hash,
        mppi_terminal_liveness_archive_sha256=dict(manifest['mppi_terminal_liveness_parent_archive_sha256']))
    (out / 'build_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    check_build(out)
    verify_production(out)
    print('BUILT MPPI TERMINAL LIVENESS', out, flush=True)
    return out


def build(base, out):
    return compile_prepared(prepare(base, out))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', type=Path, default=BASE)
    parser.add_argument('--out', type=Path, required=True, help='new, nonexistent independent build directory')
    parser.add_argument('--compile', action='store_true', help='explicitly compile/link MPPI after preparation')
    parser.add_argument('--finish-only', action='store_true', help='compile an already prepared directory')
    args = parser.parse_args()
    if args.finish_only and not args.compile:
        parser.error('--finish-only requires --compile')
    if not args.finish_only:
        prepare(args.base, args.out)
    if args.compile:
        compile_prepared(args.out)
