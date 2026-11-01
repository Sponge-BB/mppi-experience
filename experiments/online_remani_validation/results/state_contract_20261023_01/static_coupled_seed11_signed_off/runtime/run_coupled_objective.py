#!/usr/bin/env python3
"""Isolated native ROS verification of genuine slow/fast objective coupling.

This is numerical software-in-the-loop, NOT Gazebo or a physical-robot test.
Old rolling-target / prefix groups are disabled in every treatment.  Only the
recorded engineering switch and optimizer weights differ between treatments.
"""
import argparse
from datetime import datetime, timedelta, timezone
import hashlib
import gzip
import json
import math
import os
from pathlib import Path
import random
import re
import shutil
import socket
import subprocess
import time
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
PORT = 11441
GROUPS = ('native', 'engineering', 'execution', 'handover', 'coupled')
MIN_FREE_BYTES = 400 * 1024 ** 2
HARD_FREE_BYTES = 150 * 1024 ** 2
RUNTIME_CONTRACT = ('mppi_node.py', 'local_execution.py', 'simulator.py', 'native_reference.py',
                    'progress.py', 'classified_frame.py', 'dynamic_scenarios.py',
                    'dual_timescale_observer.py', 'search_recording.py', 'shared_collision_node.py',
                    'collab_seed_export.py', 'run_collab_factorial.py', 'run_resource_comparison.py')
ENGINEERING_SCOPE = ('additional projected qualification/soft cost only; retain native 3D, '
                     'self-collision, wheel and long-horizon dynamic guards; '
                     'not full dynamic feasibility equivalence and not an endpoint-stagnation fix')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def treatment(group, execution_weight=1., handover_weight=1.):
    if group not in GROUPS:
        raise ValueError('unknown coupling group: ' + str(group))
    if any(not math.isfinite(v) or v < 0 for v in (execution_weight, handover_weight)):
        raise ValueError('coupling weights must be finite and nonnegative')
    return dict(group=group, engineering=group != 'native',
                execution_weight=execution_weight if group in ('execution', 'coupled') else 0.,
                handover_weight=handover_weight if group in ('handover', 'coupled') else 0.)


def check_space(path, reserve_bytes=MIN_FREE_BYTES):
    if reserve_bytes < HARD_FREE_BYTES:
        raise ValueError('Storage reserve cannot be below the 150 MiB hard safety floor')
    free = shutil.disk_usage(path).free
    if free < reserve_bytes:
        raise RuntimeError('Less than %d MiB free; existing results retained, no new trial started' % (reserve_bytes // 1024 ** 2))
    return free


def ordered_jobs(plan):
    groups, scenes, seeds = plan['groups'], plan['scenes'], plan['seeds']
    for values in (groups, scenes, seeds):
        if not isinstance(values, list) or not values or len(set(values)) != len(values):
            raise ValueError('Plan dimensions must be nonempty unique lists')
    if any(group not in GROUPS for group in groups):
        raise ValueError('Unknown plan treatment')
    if any(not isinstance(scene, str) or not re.fullmatch('[a-z0-9_]+', scene) for scene in scenes):
        raise ValueError('Unsafe scene name')
    if any(not isinstance(seed, int) or isinstance(seed, bool) for seed in seeds):
        raise ValueError('Plan seeds must be integers')
    jobs = [(scene, group, seed) for seed in seeds for scene in scenes for group in groups]
    random.Random(20261022).shuffle(jobs)
    return jobs


def job_name(job):
    return '%s_%s_seed%02d' % job


def original_sha(path):
    if path.exists():
        return sha(path)
    archive = path.with_name(path.name + '.gz')
    if not archive.is_file() or archive.is_symlink():
        raise ValueError('Missing audited input: ' + str(path))
    h = hashlib.sha256()
    with gzip.open(archive, 'rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def validate_completed_trial(trial, plan, build_manifest, job):
    def read(name):
        path = trial / name
        if path.is_symlink() or not path.is_file():
            raise ValueError('Existing trial is incomplete or has symlink metadata: ' + str(path))
        return json.loads(path.read_text())
    manifest, status, audit = read('manifest.json'), read('run_status.json'), read('trial_audit.json')
    settings, expected = manifest.get('coupling_experiment', {}), treatment(job[1], plan['execution_weight'], plan['handover_weight'])
    expected.update(seed=job[2], legacy_rolling=False, legacy_prefix=False, diagnostic_pause=False)
    if manifest.get('scene') != job[0] or any(settings.get(k) != v for k, v in expected.items()):
        raise ValueError('Existing trial does not match planned treatment: ' + str(trial))
    if manifest.get('duration') != plan['duration_s'] or manifest.get('record_search') != plan['search']:
        raise ValueError('Existing trial duration/search changed: ' + str(trial))
    if manifest.get('build') != plan['build'] or manifest.get('build_manifest_sha256') != plan['build_manifest_sha256']:
        raise ValueError('Existing trial build provenance differs: ' + str(trial))
    for key in ('native_binaries_sha256', 'mppi_library_sha256'):
        if manifest.get(key) != build_manifest.get(key):
            raise ValueError('Existing trial executable hash differs: ' + key)
    if audit.get('valid_experiment') is not True or audit.get('errors'):
        raise ValueError('Cannot resume past an invalid trial or select only successful outcomes: ' + str(trial))
    if (status.get('returncode') != 0 or status.get('runner_error') or status.get('stop_errors') or
            status.get('required_process_failed') or not status.get('result_written') or
            not status.get('graph_audited') or not status.get('production_hashes_unchanged') or
            not status.get('build_manifest_unchanged')):
        raise ValueError('Existing trial status does not establish intact completion: ' + str(trial))
    read('result.json')  # collision / timeout is an acceptable retained outcome
    read('analysis/summary.json')
    if sha(trial / 'coupling_parameters.json') != manifest.get('parameters_sha256'):
        raise ValueError('Recorded coupling parameters changed: ' + str(trial))
    if not audit.get('inputs_sha256'):
        raise ValueError('Existing trial lacks independently audited input hashes')
    for name, digest in audit['inputs_sha256'].items():
        if Path(name).name != name or original_sha(trial / name) != digest:
            raise ValueError('Existing audited input changed: ' + name)
    runtime = {}
    for name in RUNTIME_CONTRACT:
        digest = manifest.get('experiment_sources_sha256', {}).get(name)
        if digest is None or sha(trial / 'runtime' / name) != digest:
            raise ValueError('Existing execution/instrumentation snapshot changed: ' + name)
        runtime[name] = digest
    return runtime


def resume_inventory(out, build, overrides=None):
    """Read-only, fail-closed inventory. Never rerun or discard existing output."""
    plan_path, stopped_path = out / 'experiment_plan.json', out / 'batch_stopped.json'
    plan = json.loads(plan_path.read_text())
    if (out / 'batch_complete.json').exists():
        raise ValueError('Batch already complete; no trial may be rerun')
    if not stopped_path.is_file():
        raise ValueError('Resume requires the retained original batch_stopped.json')
    stopped = json.loads(stopped_path.read_text())
    if plan.get('build') != str(build) or sha(build / 'build_manifest.json') != plan.get('build_manifest_sha256'):
        raise ValueError('Resume build must exactly match original frozen build')
    if any(plan.get(key) for key in ('paired', 'legacy_rolling', 'legacy_prefix')):
        raise ValueError('Not an unpaused objective-coupling plan')
    for key, value in (overrides or {}).items():
        if value is not None and value != plan.get(key):
            raise ValueError('Resume cannot alter original plan field: ' + key)
    jobs = ordered_jobs(plan)
    if stopped.get('expected_trials') != len(jobs):
        raise ValueError('Original stop record disagrees with planned trial count')
    old_completed = [(r['scene'], r['group'], r['seed']) for r in stopped.get('completed', [])]
    if old_completed != jobs[:len(old_completed)] or any(r.get('valid') is not True for r in stopped.get('completed', [])):
        raise ValueError('Original completed list is not a valid prefix of the declared order')
    names = {job_name(job): job for job in jobs}
    unexpected = [p.name for p in out.iterdir() if p.is_dir() and p.name not in names
                  and ((p / 'manifest.json').exists() or (p / 'run_status.json').exists())]
    if unexpected:
        raise ValueError('Unplanned trial directories present: ' + str(unexpected))
    build_manifest = json.loads((build / 'build_manifest.json').read_text())
    if build_manifest.get('production_sources_sha256') != plan.get('production_sources_sha256'):
        raise ValueError('Production snapshot differs from original plan')
    completed, runtime = [], None
    missing_seen = False
    for job in jobs:
        trial = out / job_name(job)
        if not trial.exists():
            missing_seen = True
            continue
        if missing_seen or trial.is_symlink() or not trial.is_dir():
            raise ValueError('Existing trials are not an intact prefix; refusing outcome selection')
        observed = validate_completed_trial(trial, plan, build_manifest, job)
        if runtime is not None and observed != runtime:
            raise ValueError('Previously completed trials have different execution/instrumentation snapshots')
        runtime = observed
        completed.append(job)
    if len(completed) < len(old_completed):
        raise ValueError('An originally completed trial has disappeared')
    if len(completed) > len(old_completed) and not list((out / 'continuations').glob('*.json')):
        raise ValueError('Unrecorded extra completed trials; continuation provenance missing')
    return dict(plan=plan, plan_sha256=sha(plan_path), original_stop_sha256=sha(stopped_path),
                completed=completed, pending=jobs[len(completed):], runtime_sources=runtime,
                expected_trials=len(jobs))


def check_build(build):
    from audit_dual_timescale import build_integrity
    manifest = json.loads((build / 'build_manifest.json').read_text())
    if not manifest.get('coupling_objective') or not manifest.get('dual_timescale_probe'):
        raise RuntimeError('Requires an isolated coupling-objective build with native timeline instrumentation')
    integrity = build_integrity(build, {})
    if not integrity['pass_integrity']:
        raise RuntimeError('Build integrity failed: ' + str(integrity['errors']))
    return manifest, integrity


def configure_shared_collision(out, enabled):
    """Configure only this new trial; baseline does not inherit shared geometry.

    The same compiled planner selects engineering behavior via the recorded
    COUPLING_ENGINEERING switch.  Its companion Python graph must agree.
    """
    path = out / 'experiment.launch'
    tree = ET.parse(path)
    launch = tree.getroot()
    base_nodes = [n for n in launch.findall('node') if n.get('name') == 'base_mppi']
    shared_nodes = [n for n in launch.findall('node') if n.get('name') == 'shared_collision_model']
    if len(base_nodes) != 1 or (enabled and len(shared_nodes) != 1):
        raise ValueError('shared collision launch prerequisites are missing or ambiguous')
    params = [p for p in base_nodes[0].findall('param') if p.get('name') == 'shared_collision']
    if len(params) != 1:
        raise ValueError('expected exactly one MPPI shared_collision switch')
    params[0].set('value', str(bool(enabled)).lower())
    if not enabled:
        for node in shared_nodes:
            launch.remove(node)
    tree.write(path, encoding='unicode')
    manifest_path = out / 'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    manifest['shared_collision'] = bool(enabled)
    if not enabled:
        manifest['nodes'] = [n for n in manifest.get('nodes', []) if n.get('name') != 'shared_collision_model']
    manifest['shared_collision_runtime_enabled'] = bool(enabled)
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')


def prepare_trial(out, build, scene, group, seed, duration, execution_weight=1.,
                  handover_weight=1., search=False):
    from run_collab_factorial import prepare_trial as prepare_legacy_trial
    # Reuse source snapshotting, seed export, observer wiring and bounded real
    # search export.  Legacy A means NO rolling subgoal and NO MPPI prefix.
    prepare_legacy_trial(out, build, scene, 'A', seed, duration,
                         paired=False, record_search=search)
    settings = treatment(group, execution_weight, handover_weight)
    configure_shared_collision(out, settings['engineering'])
    settings.update(seed=seed, legacy_rolling=False, legacy_prefix=False,
                    diagnostic_pause=False, genuine_online_closed_loop=True,
                    engineering_scope=ENGINEERING_SCOPE,
                    scope='fixed-arm native ROS numerical SIL; no hardware drivers or Gazebo')
    (out / 'coupling_parameters.json').write_text(json.dumps(settings, indent=2) + '\n')
    manifest_path = out / 'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    manifest['legacy_factorial_runtime'] = manifest.pop('factorial')
    manifest['legacy_factorial_runtime']['scope'] = 'runtime helper only; NOT an A-D factorial trial'
    manifest['coupling_experiment'] = settings
    manifest['parameters_sha256'] = sha(out / 'coupling_parameters.json')
    manifest['engineering_note'] = ENGINEERING_SCOPE
    manifest['build_manifest_sha256'] = sha(build / 'build_manifest.json')
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    return settings


def run_trial(out, build, scene, group, seed, duration=48., execution_weight=1.,
              handover_weight=1., search=False, expected_build_manifest_sha256=None,
              expected_runtime_sources=None, reserve_bytes=MIN_FREE_BYTES,
              diagnostic_capture=False, domain_recovery=False, clearance_recovery=False,
              state_contract=False, signed_low_speed=False):
    from run_collab_factorial import compress_new_records
    from run_dual_timescale import verify_production, stop_owned, audit_graph
    from run_resource_comparison import Monitor
    from report_dual_timescale import summarize_trial
    from audit_dual_timescale import audit_trial
    deadline = os.environ.get('REMANI_EXPERIMENT_DEADLINE')
    if deadline:
        end = datetime.fromisoformat(deadline)
        if end.tzinfo is None:
            raise ValueError('Experiment deadline must include a timezone')
        if (end-datetime.now(timezone.utc)).total_seconds() < duration+105:
            raise RuntimeError('Insufficient time before scheduled experiment cleanup deadline; no new trial')
    check_space(out.parent, reserve_bytes)
    build_manifest, integrity = check_build(build)
    if domain_recovery and not build_manifest.get('domain_recovery_experiment'):
        raise ValueError('Recoverable trials require the separate domain-recovery build')
    if clearance_recovery and not build_manifest.get('clearance_recovery_experiment'):
        raise ValueError('Clearance recovery requires the separate clearance-recovery build')
    if build_manifest.get('clearance_recovery_experiment') and (not domain_recovery or group=='native'):
        raise ValueError('Clearance pair requires projected engineering scope and identical enabled domain recovery')
    if (state_contract or signed_low_speed) and not build_manifest.get('state_contract_experiment'):
        raise ValueError('State capture/treatment requires the separate state-contract build')
    if build_manifest.get('state_contract_experiment') and (
            not state_contract or not diagnostic_capture or not domain_recovery or
            clearance_recovery or group=='native'):
        raise ValueError('State pair requires capture, diagnostics, projected engineering/domain recovery and no AL')
    if signed_low_speed and not state_contract:
        raise ValueError('Signed low-speed treatment requires exact state capture')
    if expected_build_manifest_sha256 is not None and sha(build / 'build_manifest.json') != expected_build_manifest_sha256:
        raise RuntimeError('Build manifest changed during batch; refusing to mix treatment binaries')
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1', PORT)) == 0:
            raise RuntimeError('Isolated master already running; refusing to attach to an existing ROS graph')
    before = verify_production(build)
    settings = prepare_trial(out, build, scene, group, seed, duration,
                             execution_weight, handover_weight, search)
    if build_manifest.get('domain_recovery_experiment'):
        settings['domain_recovery'] = bool(domain_recovery)
        settings['terminal_solver_failure_fail_closed'] = True
        params = out / 'coupling_parameters.json'
        params.write_text(json.dumps(settings, indent=2)+'\n')
        path = out / 'manifest.json'
        manifest = json.loads(path.read_text())
        manifest['coupling_experiment'] = settings
        manifest['parameters_sha256'] = sha(params)
        path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
    if build_manifest.get('clearance_recovery_experiment'):
        settings.update(clearance_recovery=bool(clearance_recovery),clearance_rho=1e5,
            clearance_validation_reserve_s=.04,clearance_restart_limit=1,
            clearance_weighting='fixed K8 collocation weights, not duration-dependent integral')
        params=out/'coupling_parameters.json';params.write_text(json.dumps(settings,indent=2)+'\n')
        path=out/'manifest.json';manifest=json.loads(path.read_text())
        manifest['coupling_experiment']=settings;manifest['parameters_sha256']=sha(params)
        path.write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    if diagnostic_capture:
        if not json.loads((build / 'build_manifest.json').read_text()).get('delivery_diagnostics'):
            raise ValueError('Diagnostic capture requires the separate diagnostic build')
        (out / 'delivery_diagnostics').mkdir(exist_ok=False)
        path = out / 'manifest.json'
        manifest = json.loads(path.read_text())
        manifest['delivery_diagnostics'] = dict(enabled=True,
            scope='candidate/failure capture only; not a performance-comparison trial')
        path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    if state_contract:
        settings.update(state_contract_capture=True, signed_low_speed=bool(signed_low_speed),
            raw_odom_identity='fnv1a64_ros1_serialized_standard',
            state_scope='fixed arm; base raw/processed/native boundary/epoch/exact MPPI input',
            capture_timing_note='synchronous diagnostic I/O; not a performance comparison')
        params=out/'coupling_parameters.json';params.write_text(json.dumps(settings,indent=2)+'\n')
        path=out/'manifest.json';manifest=json.loads(path.read_text())
        manifest['coupling_experiment']=settings;manifest['parameters_sha256']=sha(params)
        if 'state_contract_records.py' not in manifest.get('experiment_sources_sha256', {}):
            raise ValueError('State helper was not included in the frozen runtime')
        path.write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    if expected_runtime_sources is not None:
        current = json.loads((out / 'manifest.json').read_text())['experiment_sources_sha256']
        if any(current.get(name) != digest for name, digest in expected_runtime_sources.items()):
            raise RuntimeError('Prepared control/instrumentation snapshot differs from original measured trials; no ROS started')
    env = {k: v for k, v in os.environ.items()
           if not re.search('TOKEN|SECRET|PASSWORD|CREDENTIAL|AUTH|API_KEY', k, re.I)
           and not k.startswith(('FAILURE_', 'RESOURCE_', 'COLLAB_', 'COUPLING_', 'STATE_CONTRACT_'))}
    env.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT, ROS_IP='127.0.0.1',
               ROS_HOSTNAME='localhost', ROS_HOME=str(out / 'ros_home'),
               ROS_LOG_DIR=str(out / 'roslogs'),
               ROS_PACKAGE_PATH=str(out / 'ros_pkgs') + ':' + env.get('ROS_PACKAGE_PATH', ''),
               DUAL_TIMELINE_PATH=str(out / 'dual_timeline.jsonl'),
               RESOURCE_TRACE_PATH=str(out / 'solver_trace.jsonl'),
               FAILURE_TRACE_PATH=str(out / 'failure_trace.jsonl'),
               FAILURE_PROFILE_DISABLED='1', FAILURE_OWNED_PID_FILE=str(out / 'owned_pids.txt'),
               COLLAB_ROLLING='0', COLLAB_PREFIX='0', COLLAB_PAIR='0', COLLAB_SEED=str(seed),
               COUPLING_ENGINEERING=str(int(settings['engineering'])),
               COUPLING_EXEC_WEIGHT=str(settings['execution_weight']),
               COUPLING_HAND_WEIGHT=str(settings['handover_weight']))
    if build_manifest.get('domain_recovery_experiment'):
        env['COUPLING_DOMAIN_RECOVERY'] = str(int(domain_recovery))
    if build_manifest.get('clearance_recovery_experiment'):
        env['COUPLING_FEASIBILITY_RECOVERY'] = str(int(clearance_recovery))
    if diagnostic_capture:
        env['COUPLING_DIAGNOSTIC_DIR'] = str((out / 'delivery_diagnostics').resolve())
    if state_contract:
        env['STATE_CONTRACT_CAPTURE']='1'
        env['STATE_CONTRACT_SIGNED_LOW_SPEED']=str(int(signed_low_speed))
    started = time.monotonic()
    audited = False
    failure = None
    process = None
    monitor = None
    monitor_started = False
    stop_errors = []
    try:
        with (out / 'launch_stdout.log').open('w') as log:
            process = subprocess.Popen(['roslaunch', '-p', str(PORT), str(out / 'experiment.launch')],
                                       env=env, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            try:
                monitor = Monitor(process.pid, out)
                monitor.start()
                monitor_started = True
                while process.poll() is None and not (out / 'result.json').exists():
                    elapsed = time.monotonic() - started
                    owned = monitor.registered
                    if '/algorithm_simulator' in owned and '/base_mppi' in owned:
                        pid_file = out / 'owned_pids.txt'
                        if not pid_file.exists():
                            pid_file.write_text('%d %d\n' % (owned['/algorithm_simulator'], owned['/base_mppi']))
                    if not audited and elapsed > 13:
                        audit_graph(out)
                        audited = True
                    if elapsed > duration + 75:
                        raise RuntimeError('trial wall-time deadline exceeded')
                    time.sleep(.2)
                if (out / 'result.json').exists():
                    time.sleep(.4)
            finally:
                try:
                    stop_owned(process)
                except Exception as exc:
                    stop_errors.append('owned ROS shutdown: %s: %s' % (type(exc).__name__, exc))
                finally:
                    if monitor_started:
                        try:
                            monitor.close()
                        except Exception as exc:
                            stop_errors.append('CPU monitor shutdown: %s: %s' % (type(exc).__name__, exc))
    except Exception as exc:
        failure = '%s: %s' % (type(exc).__name__, exc)
    if stop_errors:
        failure = '; '.join(([failure] if failure else []) + stop_errors)
    try:
        production_unchanged = before == verify_production(build)
    except Exception as exc:
        production_unchanged = False
        failure = '; '.join(([failure] if failure else []) + ['post-run production audit: ' + str(exc)])
    try:
        build_manifest_unchanged = expected_build_manifest_sha256 is None or sha(build / 'build_manifest.json') == expected_build_manifest_sha256
        if not build_manifest_unchanged:
            failure = '; '.join(([failure] if failure else []) + ['build manifest changed during measured trial'])
    except Exception as exc:
        build_manifest_unchanged = False
        failure = '; '.join(([failure] if failure else []) + ['post-run build audit: ' + str(exc)])
    logs = (out / 'launch_stdout.log').read_text(errors='replace') if (out / 'launch_stdout.log').exists() else ''
    status = dict(returncode=process.returncode if process is not None else None,
                  result_written=(out / 'result.json').exists(), graph_audited=audited,
                  required_process_failed=bool(re.search(r'REQUIRED process \[[^\]]+\] has died', logs)),
                  monitored_node_pids=monitor.registered if monitor is not None else {},
                  runner_error=failure, stop_errors=stop_errors, elapsed_wall_s=time.monotonic() - started,
                  production_hashes_unchanged=production_unchanged,
                  build_manifest_unchanged=build_manifest_unchanged,
                  coupling_group=group, free_bytes_after_run=shutil.disk_usage(out).free)
    (out / 'run_status.json').write_text(json.dumps(status, indent=2) + '\n')
    analysis_error = None
    try:
        if status['result_written']:
            summarize_trial(out, out / 'analysis')
        audit = audit_trial(out, integrity, out / 'analysis/summary.json' if status['result_written'] else None)
        (out / 'trial_audit.json').write_text(json.dumps(audit, ensure_ascii=False, indent=2) + '\n')
    except Exception as exc:
        analysis_error = '%s: %s' % (type(exc).__name__, exc)
        audit = dict(valid_experiment=False, errors=['analysis/audit error: ' + analysis_error],
                     navigation_outcome=None, failed_data_retained=True)
        (out / 'trial_audit.json').write_text(json.dumps(audit, indent=2) + '\n')
    finally:
        # Only THIS newly created trial is archived.  Byte-for-byte recoverable
        # gzip files are SHA checked before their uncompressed duplicates go.
        compress_new_records(out)
    valid = bool(audit['valid_experiment'])
    print('TRIAL_COMPLETE', out.name, 'valid=' + str(valid),
          'audit_error=' + str(analysis_error), flush=True)
    return valid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--groups', nargs='+', choices=GROUPS)
    parser.add_argument('--scenes', nargs='+')
    parser.add_argument('--seeds', nargs='+', type=int)
    parser.add_argument('--duration', type=float)
    parser.add_argument('--execution-weight', type=float)
    parser.add_argument('--handover-weight', type=float)
    parser.add_argument('--search', action='store_true', default=None, help='bounded real candidate subset; key GIF trials only')
    parser.add_argument('--continue-on-invalid', action='store_true')
    parser.add_argument('--resume', action='store_true', help='inherit and strictly validate an interrupted original plan')
    parser.add_argument('--resume-check-only', action='store_true', help='read-only resume inventory; never launch or archive')
    parser.add_argument('--archive-snapshots', action='store_true', help='post-trial tar.xz only after all recorded ROS processes exit')
    parser.add_argument('--reserve-mib', type=int, default=400, help='storage-only reserve; minimum 200 MiB, hard internal floor 150 MiB')
    args = parser.parse_args()
    if args.resume_check_only and not args.resume:
        parser.error('--resume-check-only requires --resume')
    if args.reserve_mib < 200:
        parser.error('--reserve-mib must be at least 200; this changes storage policy, not the algorithm')
    out, build = args.out.resolve(), args.build.resolve()
    if args.resume:
        check_build(build)
        overrides = dict(groups=args.groups, scenes=args.scenes, seeds=args.seeds,
                         duration_s=args.duration, execution_weight=args.execution_weight,
                         handover_weight=args.handover_weight, search=args.search)
        inventory = resume_inventory(out, build, overrides)
        plan = inventory['plan']
        if args.resume_check_only:
            print(json.dumps({key: value for key, value in inventory.items()
                              if key not in ('plan', 'runtime_sources')}, ensure_ascii=False, indent=2))
            return
        check_space(out, args.reserve_mib * 1024 ** 2)
        check_build(build)
        continuations = out / 'continuations'
        continuations.mkdir(exist_ok=True)
        record_path = continuations / ('resume_%s_%d.json' % (datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S'), os.getpid()))
        record = dict(plan_sha256=inventory['plan_sha256'], original_stop_sha256=inventory['original_stop_sha256'],
                      runner_sha256=sha(Path(__file__)), completed_before=inventory['completed'],
                      pending_original_order=inventory['pending'], completed_new=[], status='running',
                      reserve_mib=args.reserve_mib, original_reserve_bytes=plan.get('minimum_free_bytes'),
                      archive_snapshots_after_trial=args.archive_snapshots,
                      algorithm_parameters_unchanged=True,
                      storage_policy_note='post-run archival and reserve only; original plan/stop record unchanged')
        with record_path.open('x') as stream:
            json.dump(record, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
        try:
            for scene, group, seed in inventory['pending']:
                valid = run_trial(out / job_name((scene, group, seed)), build, scene, group, seed,
                                  plan['duration_s'], plan['execution_weight'], plan['handover_weight'],
                                  plan['search'], plan['build_manifest_sha256'],
                                  inventory['runtime_sources'], args.reserve_mib * 1024 ** 2)
                record['completed_new'].append(dict(scene=scene, group=group, seed=seed, valid=valid))
                record_path.write_text(json.dumps(record, ensure_ascii=False, indent=2) + '\n')
                if not valid:
                    raise RuntimeError('Resumed trial invalid; retained without rerun or outcome filtering')
                if args.archive_snapshots:
                    from archive_coupling_snapshots import archive_trial
                    archived = archive_trial(out / job_name((scene, group, seed)))
                    record.setdefault('storage_archives', []).append(dict(trial=job_name((scene, group, seed)),
                        skipped=archived.get('skipped', False), space_saved_bytes=archived.get('space_saved_bytes')))
                    record_path.write_text(json.dumps(record, ensure_ascii=False, indent=2) + '\n')
        except Exception as exc:
            record.update(status='stopped', reason='%s: %s' % (type(exc).__name__, exc))
            record_path.write_text(json.dumps(record, ensure_ascii=False, indent=2) + '\n')
            raise
        # Read-only final validation establishes all 15 UNIQUE original jobs;
        # no original metadata is rewritten and no missing trial is ignored.
        final = resume_inventory(out, build)
        if final['pending']:
            raise RuntimeError('Resume ended with unmeasured planned trials')
        if sha(out / 'experiment_plan.json') != inventory['plan_sha256'] or sha(out / 'batch_stopped.json') != inventory['original_stop_sha256']:
            raise RuntimeError('Original plan/stop record changed during continuation')
        record['status'] = 'complete'
        record_path.write_text(json.dumps(record, ensure_ascii=False, indent=2) + '\n')
        with (out / 'batch_complete.json').open('x') as stream:
            json.dump(dict(complete=True, trials=final['expected_trials'], all_integrity_valid=True,
                           resumed=True, plan_sha256=inventory['plan_sha256'],
                           original_stop_sha256=inventory['original_stop_sha256'], continuation=str(record_path)), stream, indent=2)
            stream.write('\n')
        print('BATCH_COMPLETE', out, flush=True)
        return
    args.groups = args.groups if args.groups is not None else list(GROUPS)
    args.scenes = args.scenes if args.scenes is not None else ['static', 'dynamic_head_on_center', 'dynamic_four']
    args.seeds = args.seeds if args.seeds is not None else [1]
    args.duration = args.duration if args.duration is not None else 48.
    args.execution_weight = args.execution_weight if args.execution_weight is not None else 1.
    args.handover_weight = args.handover_weight if args.handover_weight is not None else 1.
    args.search = args.search if args.search is not None else False
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('duration must be finite and positive')
    treatment('coupled', args.execution_weight, args.handover_weight)
    check_space(out.parent, args.reserve_mib * 1024 ** 2)
    manifest, _ = check_build(build)
    out.mkdir(parents=True, exist_ok=False)
    plan = dict(date=datetime.now(timezone(timedelta(hours=8))).isoformat(), timezone='Asia/Shanghai', build=str(build),
                groups=args.groups, scenes=args.scenes, seeds=args.seeds, duration_s=args.duration,
                execution_weight=args.execution_weight, handover_weight=args.handover_weight,
                search=args.search, legacy_rolling=False, legacy_prefix=False, paired=False,
                scope='online native ROS numerical SIL; fixed arm, synthetic perception and lagged differential plant; NOT Gazebo/real robot',
                production_sources_sha256=manifest['production_sources_sha256'],
                runner_sha256=sha(Path(__file__)), build_manifest_sha256=sha(build / 'build_manifest.json'),
                comparison='native serial baseline; additional projected qualification/soft cost; execution-only; handover-only; both objectives',
                engineering_scope=ENGINEERING_SCOPE,
                randomization='same declared scene/controller seed; online feedback diverges; not identical-input paired solves',
                minimum_free_bytes=args.reserve_mib * 1024 ** 2,
                archive_snapshots_after_trial=args.archive_snapshots,
                objective_weights_status='pilot weights unless independently frozen in a subsequent validation batch')
    (out / 'experiment_plan.json').write_text(json.dumps(plan, ensure_ascii=False, indent=2) + '\n')
    jobs = ordered_jobs(plan)
    completed = []
    try:
        for scene, group, seed in jobs:
            valid = run_trial(out / ('%s_%s_seed%02d' % (scene, group, seed)), build,
                              scene, group, seed, args.duration,
                              args.execution_weight, args.handover_weight, args.search,
                              plan['build_manifest_sha256'], reserve_bytes=args.reserve_mib * 1024 ** 2)
            completed.append(dict(scene=scene, group=group, seed=seed, valid=valid))
            if not valid and not args.continue_on_invalid:
                raise RuntimeError('Invalid setup/interface trial retained; inspect before starting more trials')
            if args.archive_snapshots:
                from archive_coupling_snapshots import archive_trial
                archive_trial(out / job_name((scene, group, seed)))
    except Exception as exc:
        (out / 'batch_stopped.json').write_text(json.dumps(dict(
            complete=False, completed=completed, expected_trials=len(jobs),
            reason='%s: %s' % (type(exc).__name__, exc)), indent=2) + '\n')
        raise
    (out / 'batch_complete.json').write_text(json.dumps(dict(complete=True,
        trials=len(completed), all_integrity_valid=all(r['valid'] for r in completed)), indent=2) + '\n')
    print('BATCH_COMPLETE', out, flush=True)


if __name__ == '__main__':
    main()
