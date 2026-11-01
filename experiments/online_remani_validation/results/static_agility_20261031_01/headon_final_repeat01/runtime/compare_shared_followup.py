#!/usr/bin/env python3
"""Six new shared-V12 online trials, with eighteen verified campaign controls.

Reused control directories remain read-only absolute symlinks. No trial in the
completed controls suite is rerun, rewritten, or treated as a newly run trial.
"""
import argparse
from datetime import datetime, timedelta, timezone
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

from compare_shared_collision import SCENES, VARIANTS, specs
from dynamic_scenarios import CASES, scene_definition

HERE = Path(__file__).resolve().parent
NATIVE = ('remani_planner_node', 'mm_controller_node', 'dynamic_obstacle_tracker_node')
CONTROL_VARIANTS = ('remani', 'v12', 'remani_shared')
OPERATIONAL_PYTHON = ('simulator.py', 'dynamic_scenarios.py', 'mppi_node.py',
                      'local_execution.py', 'native_reference.py', 'classified_frame.py',
                      'progress.py', 'shared_collision_node.py', 'search_recording.py', 'prepare.py', 'run.py')
COMMON_RAW = ('manifest.json', 'result.json', 'run_status.json', 'trajectories.json',
              'consumed_references.jsonl', 'reference_events.jsonl', 'state.csv',
              'mppi_diagnostics.json', 'scenario.json', 'limits.json', 'geometry.csv',
              'geometry.json', 'parameters.yaml')


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text())


def require(condition, message):
    if not condition:
        raise ValueError(message)


def verify_build(path):
    path = Path(path).resolve()
    manifest = read_json(path / 'build_manifest.json')
    require(set(manifest.get('native_binaries_sha256', {})) == set(NATIVE), 'incomplete native binary inventory: ' + str(path))
    for name, expected in manifest['native_binaries_sha256'].items():
        require(sha(path / 'build' / name) == expected, 'native binary hash mismatch: ' + str(path / name))
    require(sha(path / 'build/libonline_mppi.so') == manifest['mppi_library_sha256'], 'MPPI library hash mismatch: ' + str(path))
    if manifest.get('shared_collision'):
        require(sha(path / 'source/shared_collision.hpp') == manifest['shared_collision_source_sha256'], 'shared implementation hash mismatch: ' + str(path))
    for name, expected in manifest.get('production_sources_sha256', {}).items():
        relative = Path(name)
        require(not relative.is_absolute() and '..' not in relative.parts, 'unsafe production inventory path')
        require(sha(HERE.parents[1] / relative) == expected, 'production source changed since controls were run: ' + name)
    return manifest


def expected_scene(scene):
    value = scene_definition(scene)
    if scene in CASES:
        value['time_origin'] = 'motion'
    return value


def finished_trial(path, spec, duration):
    """Failure outcomes are retained; require complete, internally labelled data."""
    path = Path(path)
    for name in COMMON_RAW:
        require((path / name).is_file(), 'missing completed raw record: ' + str(path / name))
    manifest, result, status = (read_json(path / name) for name in ('manifest.json', 'result.json', 'run_status.json'))
    native = spec['variant'].startswith('remani')
    checks = dict(mode='remani' if native else 'mppi', scene=spec['scene'], duration=duration,
                  scenario_start='motion', planner_variant='current', perception_mode='classified_atomic',
                  local_prediction_s=3., reference_delay_s=0., local_execution_owner=not native,
                  shared_collision=spec['variant'].endswith('_shared'), record_search=not native)
    for key, value in checks.items():
        require(manifest.get(key) == value, 'control setting mismatch: ' + str(path) + ' / ' + key)
    require(not manifest.get('debug_native', False), 'debugger timing cannot be reused as campaign control')
    require(result.get('scene') == spec['scene'] and result.get('scenario_start') == 'motion', 'result scene/time origin differs from manifest')
    require(isinstance(result.get('reason'), str) and bool(result['reason']), 'missing final trial outcome')
    require(status.get('result_written') is True and 'required_process_failed' in status, 'trial lacks committed completion status')
    require(manifest.get('scenario') == expected_scene(spec['scene']), 'declared actor configuration changed')
    require(read_json(path / 'scenario.json') == manifest['scenario'], 'scenario file differs from trial declaration')
    raw = read_json(path / 'trajectories.json')
    require(isinstance(raw, list), 'raw native trajectory log is not a list')
    for row in raw:
        require(isinstance(row, dict) and all(k in row for k in ('t', 'action', 'id', 'stamp', 'polynomials', 'message_sha256')),
                'incomplete raw native trajectory record: ' + str(path))
    refs = [json.loads(line) for line in (path / 'consumed_references.jsonl').read_text().splitlines()]
    for row in refs:
        require(isinstance(row, dict) and all(k in row for k in ('t', 'revision', 'epoch', 'reference', 'message_hashes', 'sha256')),
                'incomplete recorded native reference: ' + str(path))
    for name in ('reference_events.jsonl',):
        for line in (path / name).read_text().splitlines():
            require(isinstance(json.loads(line), dict), 'invalid reference event: ' + str(path))
    require(read_json(path / 'limits.json').get('max_prediction') == 3., 'actual local forecast limit differs from 3 s')
    if not native:
        require((path / 'execution_rollouts.jsonl').is_file(), 'missing local execution record: ' + str(path))
    if manifest['shared_collision']:
        require((path / 'collision_snapshots.jsonl').is_file() and (path / 'shared_collision_config.json').is_file(), 'missing shared collision provenance')
    files = list(COMMON_RAW)
    if not native:
        files.append('execution_rollouts.jsonl')
    if manifest['shared_collision']:
        files.extend(('collision_snapshots.jsonl', 'shared_collision_config.json'))
    return manifest, dict(raw_file_sha256={name: sha(path / name) for name in files},
                          native_raw_message_count=len(raw), recorded_reference_count=len(refs),
                          original_result=result['reason'], original_required_process_failed=status['required_process_failed'])


def preflight(controls, new_build, out):
    controls, new_build, out = (Path(p).resolve() for p in (controls, new_build, out))
    require(controls.is_dir(), 'completed controls suite not found')
    require(not out.exists(), 'refusing to overwrite follow-up output: ' + str(out))
    require(out != controls and controls not in out.parents, 'follow-up output must be outside the completed controls suite')
    require(out != new_build and new_build not in out.parents, 'follow-up output must not be inside the build')
    campaign = read_json(controls / 'suite_manifest.json')
    require(campaign.get('campaign_type') != 'followup_with_reused_controls', 'controls must be the completed original campaign, not a reuse chain')
    require(campaign.get('scenes') == list(SCENES) and campaign.get('variants') == list(VARIANTS), 'control campaign has different scenes or variants')
    duration = float(campaign['duration'])
    original_specs = campaign['specs']
    require(len(original_specs) == 24, 'control campaign must contain all 24 declared trials')
    require(len({s['key'] for s in original_specs}) == 24, 'duplicate original trial keys')
    source_by_key = {s['key']: s for s in original_specs}
    old_build = Path(source_by_key['empty/v12']['build']).resolve()
    target = verify_build(new_build)
    require(target.get('only_local_mppi_recompiled') is True and target.get('shared_collision') is True, 'follow-up requires MPPI-only shared safety rebuild')
    parent = Path(target['local_safety_parent']).resolve()
    require(parent != new_build, 'safety rebuild must retain its immutable parent')
    parent_manifest = verify_build(parent)
    require(sha(parent / 'build_manifest.json') == target['local_safety_base_manifest_sha256'], 'safety rebuild parent manifest changed')
    require(target['native_binaries_sha256'] == parent_manifest['native_binaries_sha256'], 'native binaries changed in MPPI-only rebuild')
    require(target['shared_collision_source_sha256'] == parent_manifest['shared_collision_source_sha256'], 'shared collision model changed in MPPI-only rebuild')
    require(target.get('experiment_parameter_overrides') == parent_manifest.get('experiment_parameter_overrides'), 'rebuild changed inherited launch parameters')
    manifests = {new_build: target, parent: parent_manifest, old_build: verify_build(old_build)}
    plan = specs(old_build, new_build)
    require(set(source_by_key) == {s['key'] for s in plan}, 'campaign spec set changed')
    for spec in plan:
        original = source_by_key[spec['key']]
        require(all(original.get(k) == spec[k] for k in ('scene', 'variant', 'repeat')), 'campaign labels/repetitions changed')
        source = controls / spec['key']
        require(source.is_dir() and not source.is_symlink(), 'control must be an original completed directory, not another reused link: ' + str(source))
        # All 24 original runs must have ended; do not read an active record.
        recorded, evidence = finished_trial(source, spec, duration)
        source_build = Path(recorded['build']).resolve()
        require(source_build == Path(original['build']).resolve(), 'trial build differs from original campaign spec')
        if source_build not in manifests:
            manifests[source_build] = verify_build(source_build)
        source_manifest = manifests[source_build]
        require(recorded['native_binaries_sha256'] == source_manifest['native_binaries_sha256'], 'recorded native binary identity changed')
        require(recorded['mppi_library_sha256'] == source_manifest['mppi_library_sha256'], 'recorded MPPI library identity changed')
        if spec['variant'].endswith('_shared'):
            require(recorded.get('shared_collision_source_sha256') == source_manifest['shared_collision_source_sha256'], 'recorded shared implementation identity changed')
            require(source_manifest['native_binaries_sha256'] == target['native_binaries_sha256'], 'shared native control is not binary-identical to the target')
            require(source_manifest['shared_collision_source_sha256'] == target['shared_collision_source_sha256'], 'shared control has a different collision model')
        else:
            require(source_manifest['native_binaries_sha256'] == manifests[old_build]['native_binaries_sha256'], 'original native control identity differs')
            require(source_manifest['mppi_library_sha256'] == manifests[old_build]['mppi_library_sha256'], 'original MPPI control identity differs')
        # Added analysis/report scripts do not affect the simulator/controller;
        # every Python component actually used in online runs must be unchanged.
        for name in OPERATIONAL_PYTHON:
            expected = recorded.get('experiment_sources_sha256', {}).get(name)
            require(expected and sha(HERE / name) == expected, 'operational Python changed since controls were run: ' + name)
            require(sha(source / 'runtime' / name) == expected, 'archived runtime source hash mismatch: ' + str(source / name))
        reused = spec['variant'] in CONTROL_VARIANTS
        comparison_build = new_build if spec['variant'].endswith('_shared') else old_build
        comparison_manifest = manifests[comparison_build]
        library_equal = source_manifest['mppi_library_sha256'] == comparison_manifest['mppi_library_sha256']
        if reused and spec['variant'] != 'remani_shared':
            require(library_equal, 'reused MPPI/original control library changed')
        if reused and not library_equal:
            require(recorded['mode'] == 'remani' and not recorded['local_execution_owner'], 'library inequality allowed only for MPPI-disabled native shared controls')
        spec.update(record_origin='reused_current_campaign_control' if reused else 'new_online_trial',
                    original_record_path=str(source if reused else out / spec['key']),
                    previous_campaign_record_path=str(source), original_recorded_build=str(source_build),
                    equivalence_target_build=str(comparison_build),
                    reused_binary_equality=dict(native_binaries_equal=True, mppi_library_equal=library_equal,
                        mppi_library_executed=recorded['mode'] == 'mppi',
                        mppi_library_difference_allowed=bool(reused and spec['variant'] == 'remani_shared'),
                        shared_implementation_equal=True if recorded['shared_collision'] else None),
                    control_record_verification=evidence)
        if reused:
            spec['build'] = str(source_build)  # Preserve the actual recorded build.
    require(sum(s['record_origin'] == 'new_online_trial' for s in plan) == 6, 'follow-up must run exactly six updated shared trials')
    return controls, new_build, out, campaign, duration, plan


def verify_reused_records(plan):
    for spec in plan:
        if spec['record_origin'] != 'reused_current_campaign_control':
            continue
        source = Path(spec['original_record_path'])
        for name, expected in spec['control_record_verification']['raw_file_sha256'].items():
            require(sha(source / name) == expected, 'reused raw record changed during follow-up: ' + str(source / name))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controls', type=Path, required=True)
    parser.add_argument('--shared-build', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--check-only', action='store_true', help='read-only verification; no ROS, output creation, or rendering')
    args = parser.parse_args()
    controls, new_build, out, campaign, duration, plan = preflight(args.controls, args.shared_build, args.out)
    if args.check_only:
        print(json.dumps(dict(passed=True, specs=len(plan), new_online_trials=6,
                              reused_current_campaign_controls=18, controls_suite=str(controls)), indent=2))
        return
    out.mkdir(parents=True, exist_ok=False)
    manifest = dict(campaign)
    created = datetime.now(timezone(timedelta(hours=8)))
    manifest.update(specs=plan, campaign_type='followup_with_reused_controls', controls_suite=str(controls),
                    timestamp=created.date().isoformat(), created_at_beijing=created.isoformat(),
                    original_campaign_timestamp=campaign.get('timestamp'),
                    new_online_trials=6, reused_current_campaign_controls=18,
                    original_campaign_manifest_sha256=sha(controls / 'suite_manifest.json'),
                    shared_followup_build=str(new_build),
                    shared_followup_build_manifest_sha256=sha(new_build / 'build_manifest.json'),
                    followup_runner_sha256=sha(Path(__file__)),
                    comparison='Six new shared V12 trials against eighteen byte-verified controls from the completed current campaign; NOT twenty-four new trials or paired-RNG repeats.',
                    reuse_policy='Absolute read-only trial-directory links; native binaries unchanged, shared implementation unchanged; only the new shared MPPI library is rerun.',
                    caveats=list(campaign.get('caveats', [])) + [
                        '18 control records are reused from the named completed campaign, not rerun in the follow-up.',
                        'The native shared-control metadata references its original MPPI library, which it did not execute; its native binaries and shared header equal the follow-up target.',
                        'Native RRT and OS scheduling are not paired or seeded across reused controls and new runs.'])
    (out / 'suite_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    for spec in plan:
        if spec['record_origin'] == 'reused_current_campaign_control':
            target = out / spec['key']; target.parent.mkdir(parents=True, exist_ok=True)
            target.symlink_to(Path(spec['original_record_path']), target_is_directory=True)
    from run import run
    incomplete = []
    for spec in plan:
        if spec['record_origin'] != 'new_online_trial':
            continue
        destination = out / spec['key']
        trial_args = SimpleNamespace(out=destination, build=new_build, scene=spec['scene'], duration=duration,
            mode='mppi', perception='classified', execution='local', projection_progress=True,
            planner_variant='current', scenario_start='motion', debug_native=False,
            reference_delay=0., record_search=True, prediction_horizon=3.)
        print('START NEW ONLINE TRIAL', spec['key'], flush=True)
        try:
            run(trial_args)
        except Exception as exc:
            destination.mkdir(parents=True, exist_ok=True)
            (destination / 'suite_error.json').write_text(json.dumps(dict(error=str(exc))) + '\n')
            print('TRIAL FAILED; RECORD RETAINED', spec['key'], str(exc), flush=True)
        try:
            actual, evidence = finished_trial(destination, spec, duration)
            build_manifest = read_json(new_build / 'build_manifest.json')
            require(actual['native_binaries_sha256'] == build_manifest['native_binaries_sha256'], 'new trial used unexpected native binaries')
            require(actual['mppi_library_sha256'] == build_manifest['mppi_library_sha256'], 'new trial used unexpected MPPI library')
            require(actual['shared_collision_source_sha256'] == build_manifest['shared_collision_source_sha256'], 'new trial used unexpected shared collision implementation')
            spec['new_online_record_verification'] = evidence
        except Exception as exc:
            incomplete.append(dict(key=spec['key'], error=str(exc)))
        completed = [dict(key=s['key'], record_origin=s['record_origin'],
                          result=read_json(out / s['key'] / 'result.json')['reason'])
                     for s in plan if (out / s['key'] / 'result.json').is_file()]
        (out / 'progress.json').write_text(json.dumps(completed, indent=2) + '\n')
    verify_reused_records(plan)
    (out / 'suite_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    if incomplete:
        (out / 'incomplete_trials.json').write_text(json.dumps(incomplete, indent=2) + '\n')
        raise RuntimeError('All six new attempts ended, but some raw records are incomplete; no full-suite rendering was started.')
    # Heavy audits and images only begin after all six online runs have ended.
    # The report must honor record_origin and never write through reused links.
    from report_shared_collision import report
    report(out, render=True)
    verify_reused_records(plan)


if __name__ == '__main__':
    main()
