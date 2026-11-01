#!/usr/bin/env python3
"""MPPI-only isolated realtime/profile child of the frozen native-3D 04 build.

Prepare is separate from compilation. No native planner/optimizer/search/model/
environment rebuild, ROS process, production edit, CMake-cache copy, or overwrite.
"""
import argparse
import copy
import json
from pathlib import Path
import re
import shutil
import subprocess

import build_native_collision_3d as previous
import build_shared_local_safety as mppi_helper
from build_shared_local_safety import NATIVE, checked_hash, sha
from run_coupled_objective import check_build
from run_dual_timescale import verify_production

HERE = Path(__file__).resolve().parent
BASE = HERE / 'builds/native_collision_3d_20261027_04'
DEFAULT_OUT = HERE / 'builds/native_realtime_20261027_01'
PATCHER = 'patch_native_realtime_mppi.py'
HEADER = 'native_realtime_probe.hpp'
BUILD_INPUTS = ('build_native_realtime.py', PATCHER, HEADER)
PATCHED = ('online_kernel.hpp', 'online_api.cpp', 'native_collision_3d.hpp')
ARCHIVES = previous.ARCHIVES
REUSED = (*NATIVE, *ARCHIVES)
PRODUCTS = (*REUSED, 'libonline_mppi.so')
SWITCHES = ('NATIVE_REALTIME_DEMAND', 'NATIVE_REALTIME_PROFILE', 'NATIVE_REALTIME_REPLAY')
SOURCE_CONTRACT = dict(patched=list(PATCHED), added=[HEADER],
    query_header_change='profile hooks only; collision numeric rules unchanged',
    only_recompiled_translation_unit='online_api.cpp',
    switches_default_false=list(SWITCHES),
    native_planner_optimizer_model_environment_recompiled=False,
    native_geometry_thresholds_and_self_collision_policy_unchanged=True,
    native_time_budgets_and_iteration_limit_unchanged=True)


def validate(base):
    base = Path(base).resolve()
    if base != BASE.resolve():
        raise ValueError('Requires frozen native_collision_3d_20261027_04 parent')
    manifest = previous.check(base)
    if manifest.get('prepared_only') or not manifest.get('native_collision_3d_experiment'):
        raise ValueError('Completed native collision 04 parent required')
    if previous.native_recipe.source_inventory(base / 'source') != manifest['isolated_source_sha256']:
        raise ValueError('Complete parent source inventory drift')
    if set(manifest['native_binaries_sha256']) != set(NATIVE):
        raise ValueError('Incomplete parent native products')
    if set(manifest['native_collision_3d_archive_sha256']) != set(ARCHIVES):
        raise ValueError('Incomplete parent native archive products')
    for name, digest in reused_hashes(manifest).items():
        checked_hash(base / 'build' / name, digest)
    checked_hash(base / 'build/libonline_mppi.so', manifest['mppi_library_sha256'])
    prove_mppi_only_header_dependency(base)
    return manifest


def reused_hashes(manifest):
    return {**manifest['native_binaries_sha256'], **manifest['native_collision_3d_archive_sha256']}


def prove_mppi_only_header_dependency(base):
    """Attested source-level include-chain proof for reusing all native TUs.

    native_collision_3d.hpp is included only by online_kernel.hpp, which is
    included only by online_api.cpp. The native exporter does not include this
    MPPI numerical header. All other inherited headers are byte-preserved.
    """
    directory = Path(base) / 'source'
    consumers = {name: [] for name in ('native_collision_3d.hpp', 'online_kernel.hpp')}
    expression = re.compile(r'^\s*#\s*include\s*[<"]([^>"]+)[>"]', re.MULTILINE)
    for path in directory.rglob('*'):
        if path.suffix not in ('.cpp', '.h', '.hpp') or not path.is_file():
            continue
        for dependency in expression.findall(path.read_text()):
            if Path(dependency).name in consumers:
                consumers[Path(dependency).name].append(path.relative_to(directory).as_posix())
    if consumers != {'native_collision_3d.hpp': ['online_kernel.hpp'], 'online_kernel.hpp': ['online_api.cpp']}:
        raise ValueError('Native query header is not isolated to the MPPI translation unit: ' + str(consumers))
    return consumers


def _mppi_recipe(manifest, out):
    frozen_parent = Path(manifest['parent_build'])
    item = mppi_helper.compile_recipe(frozen_parent, out)
    link = [item['expected_object'] if value == item['old_object'] else value for value in item['link']]
    link[link.index('-o') + 1] = str(out / 'build/libonline_mppi.so')
    item.update(actual_link_command=link, cwd=str(frozen_parent / 'build'))
    return item


def recipe(manifest, base, out):
    """Reuse the parent's frozen compiler baseline, never re-attest drift.

    Checking the old recorded hashes happens before reading current compiler
    recipes. The newly derived old-output recipe must exactly equal that old
    record, then only source/object/output paths are retargeted to this child.
    """
    frozen = manifest['native_collision_3d_recipe']
    previous.verify_recipes(frozen)
    if _mppi_recipe(manifest, base) != frozen['mppi']:
        raise ValueError('Inherited parent MPPI recipe differs from frozen compiler baseline')
    return dict(mppi=_mppi_recipe(manifest, out),
        frozen_compile_inputs_sha256=copy.deepcopy(frozen['frozen_compile_inputs_sha256']),
        unchanged_native_link_dependencies_sha256=copy.deepcopy(frozen['native']['reused_link_dependencies_sha256']))


def verify_recipe(value):
    item = value['mppi']
    checked_hash(Path(item['parent_flags_path']), item['flags_sha256'])
    checked_hash(Path(item['parent_link_path']), item['link_recipe_sha256'])
    for path, digest in item['parent_link_archives_sha256'].items():
        checked_hash(Path(path), digest)
    for key in ('frozen_compile_inputs_sha256', 'unchanged_native_link_dependencies_sha256'):
        for path, digest in value[key].items():
            checked_hash(Path(path), digest)


def transformed_sources(base):
    import patch_native_realtime_mppi as patch
    originals = {name: (base / 'source' / name).read_text() for name in PATCHED}
    changed = patch.patch_sources(originals)
    if set(changed) != set(PATCHED):
        raise ValueError('Realtime patch must return exactly the three declared MPPI files')
    for name, text in changed.items():
        if not isinstance(text, str) or text == originals[name]:
            raise ValueError('Missing declared realtime source transformation: ' + name)
        if hasattr(patch, 'project_off') and patch.project_off(text, name) != originals[name]:
            raise ValueError('Exact realtime OFF projection failed: ' + name)
    return changed


def verify_sources(out, manifest, base):
    inventory = previous.native_recipe.source_inventory(out / 'source')
    if inventory != manifest['isolated_source_sha256']:
        raise ValueError('Child complete source inventory drift')
    expected = dict(manifest['native_realtime_parent_source_sha256'])
    expected.update({name: sha(out / 'source' / name) for name in (*PATCHED, HEADER)})
    if inventory != expected:
        raise ValueError('Source changed outside declared three MPPI files plus probe header')
    for name, expected_text in transformed_sources(base).items():
        if (out / 'source' / name).read_text() != expected_text:
            raise ValueError('Deterministic realtime source transformation differs: ' + name)
    checked_hash(out / 'source' / HEADER, manifest['native_realtime_inputs_sha256'][HEADER])
    prove_mppi_only_header_dependency(out)


def prepare(base, out):
    raw = Path(out)
    if raw.is_symlink():
        raise ValueError('Output may not be a symlink')
    base, out = Path(base).resolve(), raw.resolve()
    parent = validate(base)
    for frozen in (base, Path(parent['parent_build']).resolve()):
        if out == frozen or frozen in out.parents or out in frozen.parents:
            raise ValueError('Output must be independent of immutable parent builds')
    if out.exists():
        raise FileExistsError('Refusing to overwrite existing output: ' + str(out))
    if HEADER in parent['isolated_source_sha256']:
        raise ValueError('Fresh realtime probe header required')
    changed = transformed_sources(base)
    compile_recipe = recipe(parent, base, out)
    verify_recipe(compile_recipe)
    inputs = {name: checked_hash(HERE / name, sha(HERE / name)) for name in BUILD_INPUTS}
    parent_hash = sha(base / 'build_manifest.json')
    out.mkdir(parents=True, exist_ok=False)
    (out / 'build').mkdir()
    shutil.copytree(base / 'source', out / 'source')
    for name, text in changed.items():
        (out / 'source' / name).write_text(text)
    shutil.copy2(HERE / HEADER, out / 'source' / HEADER)
    for name in PRODUCTS:
        shutil.copy2(base / 'build' / name, out / 'build' / name)
    result = copy.deepcopy(parent)
    result.update(prepared_only=True, native_realtime_experiment=True,
        native_realtime_prepared_only=True, native_realtime_parent=str(base),
        native_realtime_parent_manifest_sha256=parent_hash,
        native_realtime_parent_source_sha256=dict(parent['isolated_source_sha256']),
        native_realtime_parent_native_sha256=dict(parent['native_binaries_sha256']),
        native_realtime_parent_archive_sha256=dict(parent['native_collision_3d_archive_sha256']),
        native_realtime_parent_mppi_sha256=parent['mppi_library_sha256'],
        native_realtime_parent_build_inputs_sha256=copy.deepcopy(parent['native_collision_3d_inputs_sha256']),
        native_realtime_inputs_sha256=inputs, native_realtime_recipe=compile_recipe,
        native_realtime_reused_binary_sha256=reused_hashes(parent),
        native_realtime_source_contract=copy.deepcopy(SOURCE_CONTRACT),
        native_realtime_switches={name: False for name in SWITCHES},
        native_realtime_include_proof=prove_mppi_only_header_dependency(out),
        native_realtime_scope='isolated SIL only; demand optimization and opt-in diagnostics, no native collision rule/budget change',
        native_realtime_only_mppi_recompiled=True,
        production_modified=False, production_default=False,
        isolated_source_sha256=previous.native_recipe.source_inventory(out / 'source'))
    verify_sources(out, result, base)
    validate(base)
    checked_hash(base / 'build_manifest.json', parent_hash)
    verify_recipe(compile_recipe)
    for name, digest in inputs.items():
        checked_hash(HERE / name, digest)
    (out / 'build_manifest.json').write_text(json.dumps(result, indent=2) + '\n')
    print('PREPARED NATIVE REALTIME', out, flush=True)
    return out


def check(out):
    out = Path(out).resolve()
    path = out / 'build_manifest.json'
    if path.is_symlink() or not path.is_file():
        raise ValueError('Missing regular realtime manifest')
    manifest = json.loads(path.read_text())
    if manifest.get('native_realtime_experiment') is not True:
        raise ValueError('Not a native realtime child')
    base = Path(manifest['native_realtime_parent']).resolve()
    parent = validate(base)
    checked_hash(base / 'build_manifest.json', manifest['native_realtime_parent_manifest_sha256'])
    for recorded, expected in (
            ('native_realtime_parent_source_sha256', parent['isolated_source_sha256']),
            ('native_realtime_parent_native_sha256', parent['native_binaries_sha256']),
            ('native_realtime_parent_archive_sha256', parent['native_collision_3d_archive_sha256']),
            ('native_realtime_parent_mppi_sha256', parent['mppi_library_sha256']),
            ('native_realtime_parent_build_inputs_sha256', parent['native_collision_3d_inputs_sha256'])):
        if manifest.get(recorded) != expected:
            raise ValueError('Recorded realtime parent provenance differs: ' + recorded)
    if (manifest.get('native_realtime_source_contract') != SOURCE_CONTRACT or
            manifest.get('native_realtime_switches') != {name: False for name in SWITCHES} or
            manifest.get('native_realtime_only_mppi_recompiled') is not True):
        raise ValueError('Realtime source/switch/recompile contract differs')
    inputs = manifest.get('native_realtime_inputs_sha256', {})
    if set(inputs) != set(BUILD_INPUTS):
        raise ValueError('Incomplete realtime build-input inventory')
    for name, digest in inputs.items():
        checked_hash(HERE / name, digest)
    verify_sources(out, manifest, base)
    if manifest.get('native_realtime_include_proof') != prove_mppi_only_header_dependency(out):
        raise ValueError('Recorded MPPI-only include proof differs')
    expected_recipe = recipe(parent, base, out)
    if expected_recipe != manifest['native_realtime_recipe']:
        raise ValueError('Realtime compiler/link recipe differs from frozen parent')
    verify_recipe(expected_recipe)
    reuse = manifest.get('native_realtime_reused_binary_sha256', {})
    if reuse != reused_hashes(parent):
        raise ValueError('Realtime native/archive reuse does not exactly match parent')
    for name, digest in reuse.items():
        checked_hash(out / 'build' / name, digest)
    if (manifest.get('native_binaries_sha256') != parent['native_binaries_sha256'] or
            manifest.get('native_collision_3d_archive_sha256') != parent['native_collision_3d_archive_sha256']):
        raise ValueError('Native products must remain byte-identical to 04')
    if manifest.get('prepared_only') != manifest.get('native_realtime_prepared_only'):
        raise ValueError('Realtime prepared/completed markers disagree')
    if manifest['prepared_only']:
        checked_hash(out / 'build/libonline_mppi.so', parent['mppi_library_sha256'])
    else:
        checked_hash(out / 'build/libonline_mppi.so', manifest['mppi_library_sha256'])
        if manifest['mppi_library_sha256'] == parent['mppi_library_sha256']:
            raise ValueError('Rebuilt realtime MPPI library unexpectedly unchanged')
        check_build(out)
        verify_production(out)
    return manifest


def compile_prepared(out):
    out = Path(out).resolve()
    manifest = check(out)
    if not manifest['prepared_only']:
        raise ValueError('Fresh prepared realtime child required')
    item = manifest['native_realtime_recipe']['mppi']
    # Exactly two commands, one translation unit. No ar/native compile command.
    subprocess.run(item['compile_command'], check=True)
    subprocess.run(item['actual_link_command'], cwd=item['cwd'], check=True)
    base = Path(manifest['native_realtime_parent'])
    validate(base)
    checked_hash(base / 'build_manifest.json', manifest['native_realtime_parent_manifest_sha256'])
    verify_recipe(manifest['native_realtime_recipe'])
    verify_sources(out, manifest, base)
    for name, digest in manifest['native_realtime_inputs_sha256'].items():
        checked_hash(HERE / name, digest)
    for name, digest in manifest['native_realtime_reused_binary_sha256'].items():
        checked_hash(out / 'build' / name, digest)
    new_hash = sha(out / 'build/libonline_mppi.so')
    if new_hash == manifest['native_realtime_parent_mppi_sha256']:
        raise ValueError('Realtime MPPI library unexpectedly unchanged')
    manifest.update(prepared_only=False, native_realtime_prepared_only=False, mppi_library_sha256=new_hash)
    (out / 'build_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    check(out)
    print('BUILT NATIVE REALTIME', out, flush=True)
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', type=Path, default=BASE)
    parser.add_argument('--out', type=Path, default=DEFAULT_OUT)
    action = parser.add_mutually_exclusive_group()
    action.add_argument('--prepare', action='store_true')
    action.add_argument('--compile', action='store_true')
    action.add_argument('--check', action='store_true')
    args = parser.parse_args()
    if args.check:
        check(args.out)
    elif args.compile:
        compile_prepared(args.out)
    else:
        prepare(args.base, args.out)


if __name__ == '__main__':
    main()
