#!/usr/bin/env python3
"""Selective isolated builder for same-input budget/time-scaling diagnostics.

Preparation copies only the complete small source tree and six final products.
Compilation is explicit and rebuilds the optimizer plus three planner TUs;
search, MPPI, controller, detector and the immutable native model/environment
dependencies are byte-reused. No ROS graph or production build is started.
"""
import argparse
import copy
import json
from pathlib import Path
import shutil
import subprocess

import build_feasibility_stop as previous
from build_shared_local_safety import NATIVE, checked_hash, sha
from run_coupled_objective import check_build
from run_dual_timescale import verify_production


HERE = Path(__file__).resolve().parent
BASE = HERE / 'builds/feasibility_stop_20261026_01'
DEFAULT_OUT = HERE / 'builds/budget_scaling_20261027_01'
ARCHIVES = previous.ARCHIVES
PRODUCTS = (*NATIVE, *ARCHIVES, 'libonline_mppi.so')
REUSED = previous.REUSED
PATCHED = ('poly_traj_optimizer.cpp', 'delivery_replay_probe.hpp')
HEADER = 'budget_scaling.hpp'
PATCHER = 'patch_budget_scaling.py'


def validate(base):
    """Check the latest parent *and* its already-attested seed/stop chain."""
    base = Path(base).resolve()
    if base != BASE.resolve():
        raise ValueError('Requires the frozen feasibility_stop_20261026_01 parent')
    manifest, _ = check_build(base)
    if (not manifest.get('feasibility_stop_experiment') or manifest.get('prepared_only') or
            manifest.get('feasibility_stop_prepared_only')):
        raise ValueError('Completed feasibility-stop parent required')
    if previous.source_inventory(base / 'source') != manifest.get('isolated_source_sha256'):
        raise ValueError('Parent complete source inventory differs from manifest')
    native = manifest.get('native_binaries_sha256', {})
    if set(native) != set(NATIVE):
        raise ValueError('Parent must identify exactly the three native products')
    archives = manifest.get('feasibility_stop_archive_sha256', {})
    if set(archives) != set(ARCHIVES):
        raise ValueError('Parent must identify both native archives')
    for name, digest in {**native, **archives}.items():
        checked_hash(base / 'build' / name, digest)
    checked_hash(base / 'build/libonline_mppi.so', manifest['mppi_library_sha256'])
    # Do not bypass the older hard-coded source attestation. The trusted parent
    # manifest provides its exact gate/header/builder versions; its inverse OFF
    # projection must still recover the validated seed-parameterization parent.
    checked_hash(HERE / 'build_feasibility_stop.py', manifest['feasibility_stop_builder_sha256'])
    checked_hash(HERE / 'patch_feasibility_stop.py', manifest['feasibility_stop_patcher_sha256'])
    checked_hash(HERE / previous.HEADER, manifest['feasibility_stop_header_sha256'])
    checked_hash(base / 'source' / previous.HEADER, manifest['feasibility_stop_header_sha256'])
    seed = Path(manifest['feasibility_stop_parent']).resolve()
    seed_manifest = previous.validate(seed)
    checked_hash(seed / 'build_manifest.json', manifest['feasibility_stop_parent_manifest_sha256'])
    for recorded, expected in (
            ('feasibility_stop_parent_source_sha256', seed_manifest['isolated_source_sha256']),
            ('feasibility_stop_parent_native_sha256', seed_manifest['native_binaries_sha256']),
            ('feasibility_stop_parent_archive_sha256', seed_manifest['seed_parameterization_archive_sha256']),
            ('feasibility_stop_parent_mppi_sha256', seed_manifest['mppi_library_sha256'])):
        if manifest.get(recorded) != expected:
            raise ValueError('Parent seed provenance differs: ' + recorded)
    previous.verify_sources(base, manifest, seed)
    previous.verify_reuse(base, manifest)
    verify_production(base)
    return manifest


def verify_sources(out, manifest, base):
    import patch_budget_scaling as patch
    sources = previous.source_inventory(out / 'source')
    if sources != manifest.get('isolated_source_sha256'):
        raise ValueError('Budget-scaling source inventory differs from manifest')
    expected = dict(manifest['budget_scaling_parent_source_sha256'])
    for name in (*PATCHED, HEADER):
        expected[name] = sha(out / 'source' / name)
    if sources != expected:
        raise ValueError('Source changed outside declared optimizer/replay/header scope')
    for name, kind in zip(PATCHED, ('optimizer', 'replay')):
        if patch.project_off((out / 'source' / name).read_text(), kind=kind) != (base / 'source' / name).read_text():
            raise ValueError('Exact parent source projection failed: ' + name)
    if sources[HEADER] != manifest['budget_scaling_header_sha256']:
        raise ValueError('Copied scaling header differs from prepared source')


def verify_reuse(out, manifest):
    expected = manifest.get('budget_scaling_reused_binary_sha256', {})
    if set(expected) != set(REUSED):
        raise ValueError('Incomplete unchanged-product attestation')
    for name, digest in expected.items():
        checked_hash(out / 'build' / name, digest)


def prepare(base, out):
    import patch_budget_scaling as patch
    raw_out = Path(out)
    if raw_out.is_symlink():
        raise ValueError('Output may not be a symlink')
    base, out = Path(base).resolve(), raw_out.resolve()
    manifest = validate(base)
    for frozen in (base, Path(manifest['parent_build']).resolve()):
        if out == frozen or frozen in out.parents or out in frozen.parents:
            raise ValueError('Output must be independent of immutable builds')
    if out.exists():
        raise FileExistsError('Refusing to overwrite existing build: ' + str(out))
    header = HERE / HEADER
    if not header.is_file() or header.is_symlink() or HEADER in manifest['isolated_source_sha256']:
        raise ValueError('Fresh regular scaling header required')
    modified = {}
    for name, transform, kind in zip(PATCHED, (patch.transform_optimizer, patch.transform_replay), ('optimizer', 'replay')):
        original = (base / 'source' / name).read_text()
        modified[name] = transform(original)
        if modified[name] == original or patch.project_off(modified[name], kind=kind) != original:
            raise ValueError('Non-reversible or absent declared transformation: ' + name)
    recipe = previous.compile_commands(Path(manifest['parent_build']), out)
    previous.validate_recipe(recipe)
    parent_hash = sha(base / 'build_manifest.json')
    out.mkdir(parents=True, exist_ok=False)
    (out / 'build').mkdir()
    shutil.copytree(base / 'source', out / 'source')
    for name, text in modified.items():
        (out / 'source' / name).write_text(text)
    shutil.copy2(header, out / 'source' / HEADER)
    for name in PRODUCTS:
        shutil.copy2(base / 'build' / name, out / 'build' / name)
    result = copy.deepcopy(manifest)
    result.update(prepared_only=True, budget_scaling_experiment=True, budget_scaling_prepared_only=True,
        budget_scaling_parent=str(base), budget_scaling_parent_manifest_sha256=parent_hash,
        budget_scaling_parent_source_sha256=dict(manifest['isolated_source_sha256']),
        budget_scaling_parent_native_sha256=dict(manifest['native_binaries_sha256']),
        budget_scaling_parent_archive_sha256=dict(manifest['feasibility_stop_archive_sha256']),
        budget_scaling_parent_mppi_sha256=manifest['mppi_library_sha256'],
        budget_scaling_builder_sha256=sha(Path(__file__)),
        budget_scaling_patcher_sha256=sha(HERE / PATCHER), budget_scaling_header_sha256=sha(header),
        budget_scaling_recipe=recipe,
        budget_scaling_reused_binary_sha256={name: sha(base / 'build' / name) for name in REUSED},
        budget_scaling_exact_parent_source_projection=True,
        budget_scaling_switch='FEASIBILITY_STOP_GATE', budget_scaling_default=False,
        budget_scaling_diagnostic_phases=list(patch.PHASES),
        budget_scaling_scope='new same-process paused static fixed-input numerical SIL; budget/stop/time-scaling diagnosis only; not online delivery, hardware or Gazebo',
        budget_scaling_matrix=dict(phases=list(patch.PHASES), budgets_s=list(patch.BUDGETS),
            diagnostic_candidates_never_published=True, same_original_input_preserved=True,
            scaling_included_in_treatment_deadline=True, no_budget_refresh=True),
        budget_scaling_preserved=dict(search_archive=True, wheel_limits=True, hard_acceptance=True,
            collision_geometry_and_margin=True, mppi_library=True, controller_and_tracker_binaries=True,
            original_boundary_inputs_and_native_effective_boundary_policy=True,
            no_new_boundary_rewrite=True, native_gear_position_and_angle_variables=True,
            gear_signs=True, locked_arm=True),
        budget_scaling_parent_diagnostic_declarations=copy.deepcopy(manifest.get('diagnostic_declarations')),
        diagnostic_declarations=dict(unchanged_wheel_and_geometry=True, admission_off=True,
            clearance_recovery_off=True, legacy_prefix_off=True, legacy_rolling_off=True,
            prediction_soft_weights=0., normal_budget_s=.6, longer_budget_diagnostic_only=True,
            accepted_iterates_not_cost_callback_trials=True, fixed_input_same_binary=True,
            private_diagnostic_candidates_never_published=True),
        production_modified=False, production_default=False,
        isolated_source_sha256=previous.source_inventory(out / 'source'))
    verify_sources(out, result, base)
    verify_reuse(out, result)
    validate(base)
    checked_hash(base / 'build_manifest.json', parent_hash)
    (out / 'build_manifest.json').write_text(json.dumps(result, indent=2) + '\n')
    print('PREPARED BUDGET SCALING', out, flush=True)
    return out


def check(out):
    import patch_budget_scaling as patch
    out = Path(out).resolve()
    path = out / 'build_manifest.json'
    if path.is_symlink() or not path.is_file():
        raise ValueError('Missing regular build manifest')
    manifest = json.loads(path.read_text())
    if not manifest.get('budget_scaling_experiment'):
        raise ValueError('Requires the isolated budget-scaling build')
    matrix = manifest.get('budget_scaling_matrix', {})
    if (manifest.get('budget_scaling_diagnostic_phases') != list(patch.PHASES) or
            matrix.get('phases') != list(patch.PHASES) or
            matrix.get('budgets_s') != list(patch.BUDGETS) or
            matrix.get('diagnostic_candidates_never_published') is not True or
            matrix.get('scaling_included_in_treatment_deadline') is not True or
            matrix.get('no_budget_refresh') is not True):
        raise ValueError('Recorded phase/budget/safety contract differs')
    base = Path(manifest['budget_scaling_parent']).resolve()
    parent = validate(base)
    checked_hash(base / 'build_manifest.json', manifest['budget_scaling_parent_manifest_sha256'])
    for recorded, expected in (
            ('budget_scaling_parent_source_sha256', parent['isolated_source_sha256']),
            ('budget_scaling_parent_native_sha256', parent['native_binaries_sha256']),
            ('budget_scaling_parent_archive_sha256', parent['feasibility_stop_archive_sha256']),
            ('budget_scaling_parent_mppi_sha256', parent['mppi_library_sha256'])):
        if manifest.get(recorded) != expected:
            raise ValueError('Recorded parent provenance differs: ' + recorded)
    checked_hash(Path(__file__), manifest['budget_scaling_builder_sha256'])
    checked_hash(HERE / PATCHER, manifest['budget_scaling_patcher_sha256'])
    checked_hash(HERE / HEADER, manifest['budget_scaling_header_sha256'])
    verify_sources(out, manifest, base)
    verify_reuse(out, manifest)
    recipe = previous.compile_commands(Path(parent['parent_build']), out)
    if recipe != manifest['budget_scaling_recipe']:
        raise ValueError('Compile/link recipe changed after preparation')
    previous.validate_recipe(recipe)
    prepared = manifest.get('prepared_only')
    if prepared != manifest.get('budget_scaling_prepared_only'):
        raise ValueError('Prepared/completed markers disagree')
    if prepared:
        for name in PRODUCTS:
            checked_hash(out / 'build' / name, sha(base / 'build' / name))
    else:
        if manifest.get('budget_scaling_archive_sha256') != {
                name: sha(out / 'build' / name) for name in ARCHIVES}:
            raise ValueError('Completed archive inventory differs')
        check_build(out)
        verify_production(out)
    return manifest


def compile_prepared(out):
    out = Path(out).resolve()
    manifest = check(out)
    if not manifest.get('prepared_only'):
        raise ValueError('Fresh prepared budget-scaling build required')
    recipe = manifest['budget_scaling_recipe']
    optimizer, *planner = recipe['compile_commands']
    subprocess.run(optimizer['command'], check=True)
    subprocess.run(recipe['archive_command'], check=True)
    for command in planner:
        subprocess.run(command['command'], check=True)
    subprocess.run(recipe['link_command'], cwd=recipe['cwd'], check=True)
    base = Path(manifest['budget_scaling_parent'])
    validate(base)
    checked_hash(base / 'build_manifest.json', manifest['budget_scaling_parent_manifest_sha256'])
    previous.validate_recipe(recipe)
    verify_sources(out, manifest, base)
    verify_reuse(out, manifest)
    archives = {name: sha(out / 'build' / name) for name in ARCHIVES}
    if archives['libexp_opt.a'] == manifest['budget_scaling_parent_archive_sha256']['libexp_opt.a']:
        raise ValueError('Optimizer archive unexpectedly unchanged')
    if sha(out / 'build/remani_planner_node') == manifest['budget_scaling_parent_native_sha256']['remani_planner_node']:
        raise ValueError('Planner binary unexpectedly unchanged')
    manifest.update(prepared_only=False, budget_scaling_prepared_only=False,
        budget_scaling_archive_sha256=archives, feasibility_stop_archive_sha256=archives,
        state_contract_archive_sha256=archives, delivery_stability_archive_sha256=archives,
        seed_parameterization_archive_sha256=archives,
        native_binaries_sha256={name: sha(out / 'build' / name) for name in NATIVE})
    (out / 'build_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    check(out)
    print('BUILT BUDGET SCALING', out, flush=True)
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', type=Path, default=BASE)
    parser.add_argument('--out', type=Path, default=DEFAULT_OUT)
    action = parser.add_mutually_exclusive_group()
    action.add_argument('--prepare', action='store_true', help='prepare only (default); no compiler')
    action.add_argument('--compile', action='store_true', help='compile an existing prepared build')
    action.add_argument('--check', action='store_true', help='read-only complete provenance check')
    args = parser.parse_args()
    if args.check:
        check(args.out)
        print('CHECKED BUDGET SCALING', args.out.resolve(), flush=True)
    elif args.compile:
        compile_prepared(args.out)
    else:
        prepare(args.base, args.out)


if __name__ == '__main__':
    main()
