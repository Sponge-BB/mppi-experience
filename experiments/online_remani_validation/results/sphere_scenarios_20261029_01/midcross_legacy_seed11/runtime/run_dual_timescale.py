#!/usr/bin/env python3
"""Read-only-instrumented V12 closed loops on an isolated ROS master.

This runner never starts vehicle drivers. It changes only experiment output,
the optional reference-delivery delay, and the instrumentation environment.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import time
import xml.etree.ElementTree as ET
from xmlrpc.client import ServerProxy

from prepare import HERE, ROOT, PORT, prepare
from run_resource_comparison import Monitor


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify_production(build):
    manifest = json.loads((build / 'build_manifest.json').read_text())
    expected = manifest['production_sources_sha256']
    changed = [name for name, digest in expected.items() if sha(ROOT / name) != digest]
    if changed:
        raise RuntimeError('Production snapshot changed; do not silently mix baselines: ' + str(changed))
    return expected


def stop_owned(proc):
    if proc.poll() is not None:
        return
    os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=12)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGTERM)
        proc.wait(timeout=8)


def audit_graph(out):
    code, _, state = ServerProxy('http://127.0.0.1:%d' % PORT).getSystemState('/dual_audit')
    if code != 1:
        raise RuntimeError('Cannot read isolated ROS graph')
    publishers = dict(state[0])
    errors = []
    if publishers.get('/sim/cmd_vel') != ['/base_mppi']:
        errors.append('MPPI is not sole simulated velocity publisher')
    if publishers.get('/cmd_vel'):
        errors.append('Real vehicle command topic has publishers')
    if publishers.get('/mm_controller_node/trajectory_progress') != ['/base_mppi']:
        errors.append('MPPI is not sole planning-progress publisher')
    if '/real_base_tracker' in {n for _, ns in state[0] for n in ns}:
        errors.append('Tracker unexpectedly active in MPPI experiment')
    result = dict(publishers=publishers, errors=errors, verified=not errors)
    (out / 'publisher_audit.json').write_text(json.dumps(result, indent=2) + '\n')
    if errors:
        raise RuntimeError('; '.join(errors))


def run_trial(out, build, scene, duration=48., delay=0.):
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1', PORT)) == 0:
            raise RuntimeError('Isolated port already occupied; will not attach to another run')
    before = verify_production(build)
    prepare(out, build, 'mppi', scene, duration, projection_progress=True,
            scenario_start='motion', perception='classified', execution='local',
            reference_delay=delay, record_search=False, prediction_horizon=3.)
    tree = ET.parse(out / 'experiment.launch')
    pkg = out / 'ros_pkgs/online_remani_experiment'
    (pkg / 'dual_timescale_observer.py').symlink_to(out / 'runtime/dual_timescale_observer.py')
    node = ET.SubElement(tree.getroot(), 'node', pkg='online_remani_experiment',
                         type='dual_timescale_observer.py', name='dual_timescale_observer',
                         output='log', required='true', args='--out ' + str(out))
    tree.write(out / 'experiment.launch', encoding='unicode')
    env = {k: v for k, v in os.environ.items()
           if not re.search('TOKEN|SECRET|PASSWORD|CREDENTIAL|AUTH|API_KEY', k, re.I)
           and not k.startswith(('FAILURE_', 'RESOURCE_'))}
    env.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT, ROS_IP='127.0.0.1',
               ROS_HOSTNAME='localhost', ROS_HOME=str(out / 'ros_home'),
               ROS_LOG_DIR=str(out / 'roslogs'),
               ROS_PACKAGE_PATH=str(out / 'ros_pkgs') + ':' + env.get('ROS_PACKAGE_PATH', ''),
               DUAL_TIMELINE_PATH=str(out / 'dual_timeline.jsonl'),
               RESOURCE_TRACE_PATH=str(out / 'solver_trace.jsonl'),
               FAILURE_TRACE_PATH=str(out / 'failure_trace.jsonl'),
               FAILURE_PROFILE_DISABLED='1')
    started = time.monotonic()
    audited = False
    failure = None
    with (out / 'launch_stdout.log').open('w') as log:
        proc = subprocess.Popen(['roslaunch', '-p', str(PORT), str(out / 'experiment.launch')],
                                env=env, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)
        monitor = Monitor(proc.pid, out)
        monitor.start()
        try:
            while proc.poll() is None and not (out / 'result.json').exists():
                elapsed = time.monotonic() - started
                if elapsed > duration + 60:
                    raise RuntimeError('Experiment wall-time deadline expired')
                if not audited and elapsed > 13:
                    audit_graph(out)
                    audited = True
                time.sleep(.2)
            # Give the observer a final chance to tail the last native events.
            if (out / 'result.json').exists():
                time.sleep(.4)
        except Exception as exc:
            failure = '%s: %s' % (type(exc).__name__, exc)
        finally:
            stop_owned(proc)
            monitor.close()
    log = (out / 'launch_stdout.log').read_text(errors='replace')
    status = dict(returncode=proc.returncode, result_written=(out / 'result.json').exists(),
                  required_process_failed=bool(re.search(r'REQUIRED process \[[^\]]+\] has died', log)),
                  graph_audited=audited, monitored_node_pids=monitor.registered,
                  elapsed_wall_s=time.monotonic() - started,
                  runner_error=failure,
                  production_hashes_unchanged=before == verify_production(build))
    (out / 'run_status.json').write_text(json.dumps(status, indent=2) + '\n')
    if failure or not status['result_written'] or status['required_process_failed'] or not audited:
        raise RuntimeError('Invalid trial; retained for audit: ' + str(out))
    if not {'/base_mppi', '/remani_planner_node', '/dual_timescale_observer'}.issubset(monitor.registered):
        raise RuntimeError('Expected experiment nodes missing from host PID audit')
    print('TRIAL_COMPLETE', out.name, json.loads((out / 'result.json').read_text())['reason'], flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--build', type=Path, default=HERE / 'builds/dual_timeline_v12_01')
    p.add_argument('--repeats', type=int, default=2)
    p.add_argument('--duration', type=float, default=48.)
    p.add_argument('--scenes', nargs='+', default=['empty', 'static', 'dynamic_head_on_center', 'dynamic_four'])
    p.add_argument('--delays', nargs='+', type=float, default=[0.])
    a = p.parse_args()
    if a.repeats < 1 or any(d < 0 or d > 2 for d in a.delays):
        p.error('Positive repeats and reference delays in [0,2] required')
    out, build = a.out.resolve(), a.build.resolve()
    before = verify_production(build)
    out.mkdir(parents=True, exist_ok=False)
    plan = dict(date='2026-10-18 Asia/Shanghai', build=str(build), scenes=a.scenes,
                repeats=a.repeats, delays_s=a.delays, duration_s=a.duration,
                scope='isolated native ROS numerical simulation; fixed arm; NOT Gazebo or physical robot',
                controller='unchanged V12 MPPI, sole simulated base executor',
                manipulated_variable='reference delivery delay only; no algorithm tuning',
                randomization='MPPI inherited fixed online seed; native search inherited random_device. '
                              'Repeated online loops are not identical-input causal pairs.',
                rendering='offline after experiments, never concurrently with measured execution',
                production_sources_sha256=before, runner_sha256=sha(Path(__file__)))
    (out / 'experiment_plan.json').write_text(json.dumps(plan, ensure_ascii=False, indent=2) + '\n')
    for repeat in range(a.repeats):
        scenes = a.scenes if repeat % 2 == 0 else list(reversed(a.scenes))
        delays = a.delays if repeat % 2 == 0 else list(reversed(a.delays))
        for scene in scenes:
            for delay in delays:
                name = '%s_delay%03d_%02d' % (scene, round(delay * 1000), repeat + 1)
                run_trial(out / name, build, scene, a.duration, delay)
    after = verify_production(build)
    (out / 'batch_complete.json').write_text(json.dumps(dict(complete=True,
        production_hashes_unchanged=before == after), indent=2) + '\n')
    print('BATCH_COMPLETE', out, flush=True)


if __name__ == '__main__':
    main()
