#!/usr/bin/env python3
"""Isolated native ROS verification of genuine slow/fast objective coupling.

This is numerical software-in-the-loop, NOT Gazebo or a physical-robot test.
Old rolling-target / prefix groups are disabled in every treatment.  Only the
recorded engineering switch and optimizer weights differ between treatments.
"""
import argparse
from datetime import datetime, timedelta, timezone
import hashlib
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


def check_space(path):
    free = shutil.disk_usage(path).free
    if free < MIN_FREE_BYTES:
        raise RuntimeError('Less than 400 MiB free; existing results retained, no new trial started')
    return free


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
              handover_weight=1., search=False, expected_build_manifest_sha256=None):
    from run_collab_factorial import compress_new_records
    from run_dual_timescale import verify_production, stop_owned, audit_graph
    from run_resource_comparison import Monitor
    from report_dual_timescale import summarize_trial
    from audit_dual_timescale import audit_trial
    check_space(out.parent)
    _, integrity = check_build(build)
    if expected_build_manifest_sha256 is not None and sha(build / 'build_manifest.json') != expected_build_manifest_sha256:
        raise RuntimeError('Build manifest changed during batch; refusing to mix treatment binaries')
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1', PORT)) == 0:
            raise RuntimeError('Isolated master already running; refusing to attach to an existing ROS graph')
    before = verify_production(build)
    settings = prepare_trial(out, build, scene, group, seed, duration,
                             execution_weight, handover_weight, search)
    env = {k: v for k, v in os.environ.items()
           if not re.search('TOKEN|SECRET|PASSWORD|CREDENTIAL|AUTH|API_KEY', k, re.I)
           and not k.startswith(('FAILURE_', 'RESOURCE_', 'COLLAB_', 'COUPLING_'))}
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
    parser.add_argument('--groups', nargs='+', choices=GROUPS, default=list(GROUPS))
    parser.add_argument('--scenes', nargs='+', default=['static', 'dynamic_head_on_center', 'dynamic_four'])
    parser.add_argument('--seeds', nargs='+', type=int, default=[1])
    parser.add_argument('--duration', type=float, default=48.)
    parser.add_argument('--execution-weight', type=float, default=1.)
    parser.add_argument('--handover-weight', type=float, default=1.)
    parser.add_argument('--search', action='store_true', help='bounded real candidate subset; key GIF trials only')
    parser.add_argument('--continue-on-invalid', action='store_true')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('duration must be finite and positive')
    treatment('coupled', args.execution_weight, args.handover_weight)
    out, build = args.out.resolve(), args.build.resolve()
    check_space(out.parent)
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
                minimum_free_bytes=MIN_FREE_BYTES,
                objective_weights_status='pilot weights unless independently frozen in a subsequent validation batch')
    (out / 'experiment_plan.json').write_text(json.dumps(plan, ensure_ascii=False, indent=2) + '\n')
    jobs = [(scene, group, seed) for seed in args.seeds for scene in args.scenes for group in args.groups]
    random.Random(20261022).shuffle(jobs)
    completed = []
    try:
        for scene, group, seed in jobs:
            valid = run_trial(out / ('%s_%s_seed%02d' % (scene, group, seed)), build,
                              scene, group, seed, args.duration,
                              args.execution_weight, args.handover_weight, args.search,
                              plan['build_manifest_sha256'])
            completed.append(dict(scene=scene, group=group, seed=seed, valid=valid))
            if not valid and not args.continue_on_invalid:
                raise RuntimeError('Invalid setup/interface trial retained; inspect before starting more trials')
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
