#!/usr/bin/env python3
"""Fixed-reference same-binary OFF/ON terminal-branch regression; no ROS.

The generic trial and its measured 50 ms deadline/full-body publication and
braking validator are reused unchanged.  This is numerical fixed-reference
regression, not online REMANI replanning, perception, Gazebo or a vehicle test.
"""
import argparse
import json
import os
from pathlib import Path
import traceback

from build_shared_local_safety import NATIVE, checked_hash, sha
from build_mppi_terminal_liveness import ARCHIVES, source_inventory
from patch_mppi_terminal_phase import SWITCH, project_off
from run_coupled_objective import check_build
from run_dual_timescale import verify_production
from run_mppi_rejoin_diagnosis import CRITERIA, Kernel, trial


HERE = Path(__file__).resolve().parent
BUILD = HERE / 'builds/mppi_terminal_liveness_20261026_01'
SOURCES = {
    'source01': HERE / 'results/resource_contention/validation_02/static_remani_01',
    'source02': HERE / 'results/resource_contention/validation_02/static_remani_02',
}
CASES = (
    ('source01', 'native_turn'), ('source01', 'offset_p010'),
    ('source01', 'offset_n010'), ('source01', 'static_rejoin'),
    ('source02', 'dynamic_crossing'),
)
SOURCE_FILES = ('trajectories.json', 'state.csv', 'limits.json', 'geometry.csv', 'geometry.json')
RUNTIME_FILES = (
    'run_mppi_terminal_regression.py', 'run_mppi_rejoin_diagnosis.py',
    'dynamic_scenarios.py', 'native_reference.py', 'simulator.py',
    'build_mppi_terminal_liveness.py', 'patch_mppi_terminal_phase.py',
    'build_shared_local_safety.py', 'run_coupled_objective.py',
    'run_dual_timescale.py', 'audit_dual_timescale.py',
)
SCOPE = (
    'Fixed accepted native-reference bundle, ideal synchronous measurements, fixed-arm full projected body; '
    'same MPPI binary with constructor-only terminal-yaw-branch switch OFF/ON. '
    'Unchanged generic Kernel.solve_validate/trial, 50ms measured decision deadline, '
    'fresh rollout and braking checks; original limits/margin/shape. '
    'Fixed numerical world stepping; not online REMANI replan, ROS, detector, Gazebo or hardware. '
    'Cases without retained dynamic history may use unchanged LegacyMPPI and are compatibility regressions, '
    'not proof that the patched dynamic-history branch was exercised. '
    'No outcome-selected reruns; failed navigation and deadline results are retained.'
)


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n')


def validate(build):
    manifest, _ = check_build(build)
    if (not manifest.get('mppi_terminal_liveness_experiment') or
            manifest.get('prepared_only') or manifest.get('mppi_terminal_liveness_prepared_only')):
        raise ValueError('Completed terminal-liveness build required')
    if manifest.get('terminal_phase_fix_switch') != SWITCH:
        raise ValueError('Unexpected terminal switch contract')
    actual_sources = source_inventory(build / 'source')
    if actual_sources != manifest['isolated_source_sha256']:
        raise ValueError('Complete isolated source inventory mismatch')
    if set(manifest['native_binaries_sha256']) != set(NATIVE):
        raise ValueError('Expected exactly three attested native binaries')
    for name in NATIVE:
        checked_hash(build / 'build' / name, manifest['native_binaries_sha256'][name])
        if manifest['native_binaries_sha256'][name] != manifest['mppi_terminal_liveness_parent_native_sha256'][name]:
            raise ValueError('Native binary was changed by MPPI-only experiment')
    archive_hashes = manifest['mppi_terminal_liveness_archive_sha256']
    if set(archive_hashes) != set(ARCHIVES):
        raise ValueError('Expected exactly two attested native archives')
    for name in ARCHIVES:
        checked_hash(build / 'build' / name, archive_hashes[name])
        if archive_hashes[name] != manifest['mppi_terminal_liveness_parent_archive_sha256'][name]:
            raise ValueError('Native archive was changed by MPPI-only experiment')
    parent = Path(manifest['mppi_terminal_liveness_parent'])
    checked_hash(parent / 'build_manifest.json', manifest['mppi_terminal_liveness_parent_manifest_sha256'])
    parent_sources = manifest['mppi_terminal_liveness_parent_source_sha256']
    for name, digest in parent_sources.items():
        checked_hash(parent / 'source' / name, digest)
    original = (parent / 'source/mppi_v12.hpp').read_text()
    if project_off((build / 'source/mppi_v12.hpp').read_text()) != original:
        raise ValueError('Exact OFF source projection failed')
    for name in ('online_kernel.hpp', 'online_api.cpp', 'shared_collision.hpp'):
        if actual_sources[name] != parent_sources[name]:
            raise ValueError('Unchanged kernel/API/safety source drift: ' + name)
    verify_production(build)
    return manifest


def freeze(build):
    manifest = validate(build)
    return dict(
        build=str(build), build_manifest_sha256=sha(build / 'build_manifest.json'),
        isolated_source_sha256=manifest['isolated_source_sha256'],
        native_binaries_sha256=manifest['native_binaries_sha256'],
        archives_sha256=manifest['mppi_terminal_liveness_archive_sha256'],
        mppi_library_sha256=manifest['mppi_library_sha256'],
        production_sources_sha256=manifest['production_sources_sha256'],
        sources={key: str(path) for key, path in SOURCES.items()},
        reference_source_sha256={
            key: {name: sha(path / name) for name in SOURCE_FILES}
            for key, path in SOURCES.items()},
        runtime_sources_sha256={name: sha(HERE / name) for name in RUNTIME_FILES},
        criteria=json.loads(json.dumps(CRITERIA)))


def planned_jobs():
    jobs = []
    for index, (source_key, case) in enumerate(CASES):
        # Predeclared alternating pair order; no choice depends on results.
        for enabled in ((False, True) if index % 2 == 0 else (True, False)):
            jobs.append(dict(case=case, source_key=source_key,
                             source=str(SOURCES[source_key]), enabled=enabled,
                             seed=1, duration_s=55.,
                             historical_failure_regression=source_key == 'source02',
                             name=source_key + '_' + case + '_fix_' + ('on' if enabled else 'off')))
    return jobs


def check_pair(pair):
    """Both arms must start from exactly the same fixed-reference fixture."""
    if len(pair) != 2:
        raise ValueError('Expected a complete OFF/ON pair')
    left, right = [item['metadata'] for item in pair]
    keys = ('case', 'seed', 'initial_state', 'initial_offset_m', 'reference_source',
            'reference_sha256', 'reference_messages_sha256', 'library_sha256',
            'geometry_sha256', 'limits', 'plant_method_sha256', 'criteria',
            'duration_s', 'dt_s', 'plant_substep_s', 'search_stride',
            'static_rejoin_certificate')
    mismatched = [key for key in keys if left.get(key) != right.get(key)]
    if mismatched:
        raise ValueError('OFF/ON fixture mismatch: ' + ', '.join(mismatched))
    return dict(case=left['case'], fixture_equal=True, compared_fields=list(keys),
                same_binary_sha256=left['library_sha256'])


def run(build, out):
    build, out = Path(build).resolve(), Path(out).resolve()
    if out.exists():
        raise FileExistsError('Refusing to overwrite regression output: ' + str(out))
    frozen = freeze(build)
    jobs = planned_jobs()
    out.mkdir(parents=True, exist_ok=False)
    protocol = dict(scope=SCOPE, jobs=jobs, frozen=frozen, switch=SWITCH,
                    constructor_flag_only=True, search_stride=5,
                    actual_deadline_ms=CRITERIA['deadline_ms'],
                    alternative_deadline_ms=CRITERIA['alternative_deadline_ms'],
                    generic_kernel_and_validator_unchanged=True,
                    no_full_runtime_copy=True)
    write_json(out / 'protocol.json', protocol)
    records, pairs = [], []
    old_flag = os.environ.get(SWITCH)

    def summary():
        return dict(scope=SCOPE, completed=len(records), requested=len(jobs),
                    jobs=records, pair_checks=pairs,
                    interface_or_input_error=None,
                    all_inputs_unchanged=True)

    try:
        for job in jobs:
            if freeze(build) != frozen:
                raise ValueError('Frozen inputs changed before ' + job['name'])
            os.environ[SWITCH] = '1' if job['enabled'] else '0'
            target = out / job['name']
            result = trial(build, SOURCES[job['source_key']], target, job['case'], seed=job['seed'],
                           duration=job['duration_s'], search_stride=5, kernel_type=Kernel,
                           experiment_context=dict(
                               scope=SCOPE, terminal_phase_fix=job['enabled'],
                               switch=SWITCH, switch_value=os.environ[SWITCH],
                               same_binary_sha256=frozen['mppi_library_sha256'],
                               native_online_replanning=False))
            if freeze(build) != frozen:
                raise ValueError('Frozen inputs changed during ' + job['name'])
            metadata = json.loads((target / 'metadata.json').read_text())
            records.append(dict(**job, path=str(target), result=result, metadata=metadata,
                                result_sha256=sha(target / 'result.json')))
            current_pair = [item for item in records if item['case'] == job['case']
                            and item['source_key'] == job['source_key']]
            if len(current_pair) == 2:
                pairs.append(check_pair(current_pair))
            write_json(out / 'summary.json', summary())
            print(json.dumps(dict(trial=job['name'], passed=result['passed'],
                                  collision=result['collision'],
                                  deadline_misses=result['deadline_misses'],
                                  final_state=result['final_state']), ensure_ascii=False), flush=True)
    except Exception as error:
        partial = summary()
        try:
            still_frozen = freeze(build) == frozen
        except Exception:
            still_frozen = None
        partial.update(interface_or_input_error=str(error), all_inputs_unchanged=still_frozen,
                       traceback=traceback.format_exc(),
                       interrupted_trial=jobs[len(records)] if len(records) < len(jobs) else None)
        write_json(out / 'summary.json', partial)
        raise
    finally:
        if old_flag is None:
            os.environ.pop(SWITCH, None)
        else:
            os.environ[SWITCH] = old_flag
    if freeze(build) != frozen:
        raise ValueError('Frozen inputs changed at regression completion')
    complete = summary()
    complete.update(complete=True, frozen_final=frozen,
                    formal_success_criteria='Generic collision-free AND sustained-safe-rejoin AND settled endpoint AND bounded commands; all individual failures retained')
    write_json(out / 'summary.json', complete)
    return complete


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, default=BUILD)
    parser.add_argument('--out', type=Path, required=True, help='new independent result directory')
    args = parser.parse_args()
    run(args.build, args.out)
