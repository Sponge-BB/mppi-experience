#!/usr/bin/env python3
"""Isolated reversible native-3D collision child; production/parent untouched.

Rebuild only optimizer, three planner TUs, and MPPI's online_api TU. Reuse the
attested model/environment/search, controller, and tracker. No CMake cache copy,
ROS launch, simulation, larger iteration cap, or budget change is performed.
"""
import argparse
import copy
import json
from pathlib import Path
import shutil
import subprocess

import build_budget_scaling as parent_builder
import build_feasibility_stop as native_recipe
import build_shared_local_safety as mppi_recipe
from build_shared_local_safety import NATIVE, checked_hash, sha
from run_coupled_objective import check_build
from run_dual_timescale import verify_production

HERE = Path(__file__).resolve().parent
BASE = HERE / 'builds/budget_scaling_20261027_01'
DEFAULT_OUT = HERE / 'builds/native_collision_3d_20261027_03'
HEADERS = ('native_collision_export.hpp', 'native_collision_3d.hpp')
PATCHERS = ('patch_native_collision_planner.py', 'patch_native_collision_mppi.py')
BUILD_INPUTS = (*HEADERS, *PATCHERS, 'build_native_collision_3d.py',
                'build_budget_scaling.py', 'build_feasibility_stop.py',
                'build_shared_local_safety.py', 'run_coupled_objective.py',
                'run_dual_timescale.py', 'audit_dual_timescale.py')
PLANNER_PATCHED = ('planner_manager.cpp', 'coupling_bridge.hpp', 'coupling_native_integration.hpp')
MPPI_PATCHED = ('online_kernel.hpp', 'mppi_v12.hpp', 'online_api.cpp')
PATCHED = PLANNER_PATCHED + MPPI_PATCHED
ARCHIVES = native_recipe.ARCHIVES
PRODUCTS = (*NATIVE, *ARCHIVES, 'libonline_mppi.so')
REUSED = ('mm_controller_node', 'dynamic_obstacle_tracker_node', 'libexp_search.a')


def validate(base):
    base = Path(base).resolve()
    if base != BASE.resolve():
        raise ValueError('Requires frozen budget_scaling_20261027_01 parent')
    manifest = parent_builder.check(base)
    if manifest.get('prepared_only') or not manifest.get('budget_scaling_experiment'):
        raise ValueError('Completed budget-scaling parent required')
    if native_recipe.source_inventory(base / 'source') != manifest['isolated_source_sha256']:
        raise ValueError('Complete parent source inventory drift')
    for name, digest in manifest['budget_scaling_archive_sha256'].items():
        checked_hash(base / 'build' / name, digest)
    checked_hash(base / 'build/libonline_mppi.so', manifest['mppi_library_sha256'])
    for name, digest in manifest['native_binaries_sha256'].items():
        checked_hash(base / 'build' / name, digest)
    return manifest


def transformed_sources(base):
    import patch_native_collision_planner as planner
    import patch_native_collision_mppi as mppi
    output = {}
    for name in PLANNER_PATCHED:
        original = (base / 'source' / name).read_text()
        output[name] = planner.transform(original, name)
        if planner.project_off(output[name], name) != original:
            raise ValueError('Exact parent projection failed: ' + name)
    originals = {name: (base / 'source' / name).read_text() for name in MPPI_PATCHED}
    output.update(mppi.patch_sources(originals))
    if set(output) != set(PATCHED) or any(output[name] == (base / 'source' / name).read_text() for name in PATCHED):
        raise ValueError('Missing/undeclared native collision transformation')
    return output


def recipes(manifest, out):
    parent = Path(manifest['parent_build'])
    native = native_recipe.compile_commands(parent, out)
    mppi = mppi_recipe.compile_recipe(parent, out)
    link = [mppi['expected_object'] if x == mppi['old_object'] else x for x in mppi['link']]
    link[link.index('-o') + 1] = str(out / 'build/libonline_mppi.so')
    mppi['actual_link_command'] = link
    mppi['cwd'] = str(parent / 'build')
    # The inherited flags reference frozen native headers outside the small
    # child source tree. Attest them explicitly as compiler inputs, in addition
    # to the already-audited archive/object/link-recipe hashes.
    compile_inputs = {}
    for path in (parent / 'source').rglob('*'):
        if path.suffix in ('.h', '.hpp', '.cpp') and path.is_file():
            if path.is_symlink():
                raise ValueError('Symlinked frozen compiler input: ' + str(path))
            compile_inputs[str(path)] = sha(path)
    return dict(native=native, mppi=mppi, frozen_compile_inputs_sha256=compile_inputs)


def verify_recipes(value):
    native_recipe.validate_recipe(value['native'])
    item = value['mppi']
    checked_hash(Path(item['parent_flags_path']), item['flags_sha256'])
    checked_hash(Path(item['parent_link_path']), item['link_recipe_sha256'])
    for path, digest in item['parent_link_archives_sha256'].items():
        checked_hash(Path(path), digest)
    for path, digest in value['frozen_compile_inputs_sha256'].items():
        checked_hash(Path(path), digest)


def verify_sources(out, manifest, base):
    inventory = native_recipe.source_inventory(out / 'source')
    if inventory != manifest['isolated_source_sha256']:
        raise ValueError('Child complete source inventory drift')
    expected = dict(manifest['native_collision_3d_parent_source_sha256'])
    expected.update({name: sha(out / 'source' / name) for name in (*PATCHED, *HEADERS)})
    if expected != inventory:
        raise ValueError('Source changed outside six declared transforms and two headers')
    transformed = transformed_sources(base)
    for name, text in transformed.items():
        if (out / 'source' / name).read_text() != text:
            raise ValueError('Deterministic parent source transformation differs: ' + name)
    for name in HEADERS:
        checked_hash(out / 'source' / name, manifest['native_collision_3d_inputs_sha256'][name])


def prepare(base, out):
    raw = Path(out)
    if raw.is_symlink():
        raise ValueError('Output may not be a symlink')
    base, out = Path(base).resolve(), raw.resolve()
    manifest = validate(base)
    for frozen in (base, Path(manifest['parent_build']).resolve()):
        if out == frozen or frozen in out.parents or out in frozen.parents:
            raise ValueError('Output must be independent of immutable parent builds')
    if out.exists():
        raise FileExistsError('Refusing to overwrite existing output: ' + str(out))
    for name in HEADERS:
        if name in manifest['isolated_source_sha256']:
            raise ValueError('Fresh native collision header required')
    modified = transformed_sources(base)
    recipe = recipes(manifest, out)
    verify_recipes(recipe)
    inputs = {name: checked_hash(HERE / name, sha(HERE / name)) for name in BUILD_INPUTS}
    parent_manifest_hash = sha(base / 'build_manifest.json')
    out.mkdir(parents=True, exist_ok=False)
    (out / 'build').mkdir()
    shutil.copytree(base / 'source', out / 'source')
    for name, text in modified.items():
        (out / 'source' / name).write_text(text)
    for name in HEADERS:
        shutil.copy2(HERE / name, out / 'source' / name)
    for name in PRODUCTS:
        shutil.copy2(base / 'build' / name, out / 'build' / name)
    result = copy.deepcopy(manifest)
    result.update(prepared_only=True, native_collision_3d_experiment=True,
        # This child reuses boundary-admission-capable code, but does not run
        # that older static/coupled-only experiment. Do not inherit its active
        # classification and thereby reject the predeclared dynamic scenes.
        native_collision_3d_parent_boundary_experiment=bool(manifest.get('boundary_admission_experiment')),
        boundary_admission_experiment=False,
        native_collision_3d_prepared_only=True, native_collision_3d_parent=str(base),
        native_collision_3d_parent_manifest_sha256=parent_manifest_hash,
        native_collision_3d_parent_source_sha256=dict(manifest['isolated_source_sha256']),
        native_collision_3d_parent_native_sha256=dict(manifest['native_binaries_sha256']),
        native_collision_3d_parent_archive_sha256=dict(manifest['budget_scaling_archive_sha256']),
        native_collision_3d_parent_mppi_sha256=manifest['mppi_library_sha256'],
        native_collision_3d_inputs_sha256=inputs, native_collision_3d_recipe=recipe,
        native_collision_3d_reused_binary_sha256={name: sha(base / 'build' / name) for name in REUSED},
        native_collision_3d_switch='NATIVE_COLLISION_3D', native_collision_3d_default=False,
        native_collision_3d_export_switch='NATIVE_COLLISION_EXPORT',
        native_collision_3d_protocol='little-endian-f64 header32/body7/dynamic7/native-xmajor-ESDF-v1',
        native_collision_3d_preserved=dict(native_3d_hard_gate=True, native_wheel_constraints=True,
            native_time_budget=True, native_iteration_cap_300=True, original_search_archive=True,
            original_model_and_environment_archives=True, original_piper_self_collision_policy=True,
            original_controller_tracker=True, original_control_modes=True,
            forecast_coupling_weights_zero_required=True, fixed_arm_only=True,
            nonzero_dynamic_uncertainty_explicitly_unsupported=True),
        native_collision_3d_scope='isolated fixed-arm SIL experiment; no continuous-time or hardware safety certification',
        native_collision_3d_clock_semantics=dict(heartbeat='private queue publication time',
            esdf='honest legacy-queue snapshot acquisition, not original observation time',
            dynamic='actual thread-safe DynamicPointSnapshot observation stamp',
            static_cache='may remain unchanged during solve; crop-domain queries fail closed'),
        production_modified=False, production_default=False,
        isolated_source_sha256=native_recipe.source_inventory(out / 'source'))
    verify_sources(out, result, base)
    validate(base)
    checked_hash(base / 'build_manifest.json', parent_manifest_hash)
    (out / 'build_manifest.json').write_text(json.dumps(result, indent=2) + '\n')
    print('PREPARED NATIVE COLLISION 3D', out, flush=True)
    return out


def check(out):
    out = Path(out).resolve()
    path = out / 'build_manifest.json'
    if path.is_symlink() or not path.is_file():
        raise ValueError('Missing regular manifest')
    manifest = json.loads(path.read_text())
    if not manifest.get('native_collision_3d_experiment'):
        raise ValueError('Not a native collision 3D child')
    if manifest.get('boundary_admission_experiment') is not False:
        raise ValueError('Native 3D child must disable the inherited static-only admission experiment')
    base = Path(manifest['native_collision_3d_parent']).resolve()
    parent = validate(base)
    checked_hash(base / 'build_manifest.json', manifest['native_collision_3d_parent_manifest_sha256'])
    for recorded, expected in (
            ('native_collision_3d_parent_source_sha256', parent['isolated_source_sha256']),
            ('native_collision_3d_parent_native_sha256', parent['native_binaries_sha256']),
            ('native_collision_3d_parent_archive_sha256', parent['budget_scaling_archive_sha256']),
            ('native_collision_3d_parent_mppi_sha256', parent['mppi_library_sha256'])):
        if manifest.get(recorded) != expected:
            raise ValueError('Recorded parent provenance differs: ' + recorded)
    inputs = manifest.get('native_collision_3d_inputs_sha256', {})
    if set(inputs) != set(BUILD_INPUTS):
        raise ValueError('Incomplete native collision build-input inventory')
    for name, digest in inputs.items():
        checked_hash(HERE / name, digest)
    verify_sources(out, manifest, base)
    recipe = recipes(parent, out)
    if recipe != manifest['native_collision_3d_recipe']:
        raise ValueError('Native collision compile/link recipe drift')
    verify_recipes(recipe)
    reuse = manifest.get('native_collision_3d_reused_binary_sha256', {})
    if set(reuse) != set(REUSED):
        raise ValueError('Incomplete reused product inventory')
    for name, digest in reuse.items():
        checked_hash(out / 'build' / name, digest)
    if manifest.get('prepared_only') != manifest.get('native_collision_3d_prepared_only'):
        raise ValueError('Prepared/completed markers disagree')
    if manifest['prepared_only']:
        for name in PRODUCTS:
            checked_hash(out / 'build' / name, sha(base / 'build' / name))
    else:
        if manifest.get('native_collision_3d_archive_sha256') != {name: sha(out / 'build' / name) for name in ARCHIVES}:
            raise ValueError('Native collision archive inventory differs')
        check_build(out)
        verify_production(out)
    return manifest


def compile_prepared(out):
    out = Path(out).resolve()
    manifest = check(out)
    if not manifest['prepared_only']:
        raise ValueError('Fresh prepared native collision child required')
    recipe = manifest['native_collision_3d_recipe']
    optimizer, *planner = recipe['native']['compile_commands']
    subprocess.run(optimizer['command'], check=True)
    subprocess.run(recipe['native']['archive_command'], check=True)
    for command in planner:
        subprocess.run(command['command'], check=True)
    subprocess.run(recipe['native']['link_command'], cwd=recipe['native']['cwd'], check=True)
    subprocess.run(recipe['mppi']['compile_command'], check=True)
    subprocess.run(recipe['mppi']['actual_link_command'], cwd=recipe['mppi']['cwd'], check=True)
    base = Path(manifest['native_collision_3d_parent'])
    validate(base)
    checked_hash(base / 'build_manifest.json', manifest['native_collision_3d_parent_manifest_sha256'])
    verify_recipes(recipe)
    verify_sources(out, manifest, base)
    for name, digest in manifest['native_collision_3d_reused_binary_sha256'].items():
        checked_hash(out / 'build' / name, digest)
    manifest.update(prepared_only=False, native_collision_3d_prepared_only=False,
        native_collision_3d_archive_sha256={name: sha(out / 'build' / name) for name in ARCHIVES},
        native_binaries_sha256={name: sha(out / 'build' / name) for name in NATIVE},
        mppi_library_sha256=sha(out / 'build/libonline_mppi.so'))
    if manifest['native_binaries_sha256']['remani_planner_node'] == manifest['native_collision_3d_parent_native_sha256']['remani_planner_node']:
        raise ValueError('Planner unexpectedly unchanged')
    if manifest['mppi_library_sha256'] == manifest['native_collision_3d_parent_mppi_sha256']:
        raise ValueError('MPPI unexpectedly unchanged')
    (out / 'build_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    check(out)
    print('BUILT NATIVE COLLISION 3D', out, flush=True)
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
