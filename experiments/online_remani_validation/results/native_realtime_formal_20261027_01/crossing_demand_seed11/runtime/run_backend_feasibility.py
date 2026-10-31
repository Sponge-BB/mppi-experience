#!/usr/bin/env python3
"""Four new fixed-seed paused backend captures; no hardware or publication."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import socket
import subprocess
import time

from run_coupled_objective import HERE, PORT, check_build, check_space, prepare_trial, sha
from run_delivery_stability import resume_owned
from run_dual_timescale import audit_graph, stop_owned, verify_production
from run_resource_comparison import Monitor

JOBS = [dict(seed=11, ordinal=1), dict(seed=12, ordinal=2),
        dict(seed=11, ordinal=2), dict(seed=12, ordinal=1)]
PREFIXES = ('FAILURE_', 'RESOURCE_', 'COLLAB_', 'COUPLING_', 'STATE_CONTRACT_',
            'BOUNDARY_ADMISSION', 'DELIVERY_', 'SEED_PARAMETERIZATION_')


def diagnostic_environment(out, seed, ordinal, inherited=None):
    inherited = os.environ if inherited is None else inherited
    env = {k:v for k,v in inherited.items()
        if not re.search('TOKEN|SECRET|PASSWORD|CREDENTIAL|AUTH|API_KEY', k, re.I)
        and not k.startswith(PREFIXES)}
    env.update(ROS_MASTER_URI='http://127.0.0.1:%d'%PORT, ROS_IP='127.0.0.1', ROS_HOSTNAME='localhost',
        ROS_HOME=str(out/'ros_home'), ROS_LOG_DIR=str(out/'roslogs'),
        ROS_PACKAGE_PATH=str(out/'ros_pkgs')+':'+env.get('ROS_PACKAGE_PATH',''),
        DUAL_TIMELINE_PATH=str(out/'dual_timeline.jsonl'), RESOURCE_TRACE_PATH=str(out/'solver_trace.jsonl'),
        FAILURE_TRACE_PATH=str(out/'failure_trace.jsonl'), FAILURE_PROFILE_DISABLED='1',
        FAILURE_OWNED_PID_FILE=str(out/'owned_pids.txt'), COUPLING_ENGINEERING='1',
        COUPLING_EXEC_WEIGHT='0', COUPLING_HAND_WEIGHT='0', COUPLING_DOMAIN_RECOVERY='1',
        COUPLING_FEASIBILITY_RECOVERY='0', COUPLING_DIAGNOSTIC_DIR=str(out/'delivery_diagnostics'),
        COLLAB_ROLLING='0', COLLAB_PREFIX='0', COLLAB_PAIR='0', COLLAB_SEED=str(seed),
        STATE_CONTRACT_CAPTURE='1', STATE_CONTRACT_SIGNED_LOW_SPEED='1',
        BOUNDARY_ADMISSION='0', BOUNDARY_ADMISSION_STATIC_SCENE='1', DELIVERY_SEARCH_PROBE='1',
        SEED_PARAMETERIZATION_FIX='1', DELIVERY_REPLAY_KIND='first_refresh',
        DELIVERY_REPLAY_REFRESH_ORDINAL=str(ordinal), DELIVERY_REPLAY_LOG=str(out/'delivery_replay.jsonl'),
        DELIVERY_REPLAY_DONE_PATH=str(out/'diagnostic_complete.json'))
    return env


def prepare_capture(out, build, seed, ordinal):
    settings = prepare_trial(out, build, 'static', 'coupled', seed, 55., 0., 0., False)
    settings.update(state_contract_capture=True, signed_low_speed=True, domain_recovery=True,
        boundary_admission=False, boundary_admission_static_scene=True, clearance_recovery=False,
        seed_parameterization_fix=True, diagnostic_pause=True, genuine_online_closed_loop=False,
        navigation_metrics_valid=False, refresh_ordinal=ordinal,
        ordinal_semantics='Nth eligible normal 0.6s optimizer entry after a reference exists; not FSM request ordinal',
        scope='new same-process paused native static capture, not exact historical replay or navigation benchmark')
    path = out/'coupling_parameters.json'
    path.write_text(json.dumps(settings, indent=2)+'\n')
    manifest_path = out/'manifest.json'; manifest = json.loads(manifest_path.read_text())
    manifest.update(coupling_experiment=settings, parameters_sha256=sha(path),
        backend_feasibility_diagnostic=True, navigation_metrics_valid=False,
        delivery_diagnostics=dict(enabled=True, scope='canonical initial/accepted iterates, audited after each solver outside budget'),
        backend_feasibility_environment=dict(seed_parameterization_fix='1', replay_kind='first_refresh',
            refresh_ordinal=str(ordinal), candidate_publication=False))
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False)+'\n')
    (out/'delivery_diagnostics').mkdir(exist_ok=False)
    return manifest['experiment_sources_sha256']


def run_capture(out, build, seed, ordinal, manifest_sha, expected_runtime=None):
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1', PORT)) == 0:
            raise RuntimeError('Existing ROS11441 graph; refuse attach')
    check_space(out.parent, 2*1024**3)
    check_build(build)
    if sha(build/'build_manifest.json') != manifest_sha:
        raise RuntimeError('Frozen build manifest changed')
    before = verify_production(build)
    runtime = prepare_capture(out, build, seed, ordinal)
    if expected_runtime is not None and runtime != expected_runtime:
        raise RuntimeError('Full runtime drift before ROS launch')
    env = diagnostic_environment(out, seed, ordinal)
    proc = monitor = None; error = None; audited = False; started = time.monotonic()
    try:
        with (out/'launch_stdout.log').open('w') as log:
            proc = subprocess.Popen(['roslaunch', '-p', str(PORT), str(out/'experiment.launch')],
                env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            monitor = Monitor(proc.pid, out); monitor.start()
            try:
                while proc.poll() is None:
                    elapsed = time.monotonic()-started; owned = monitor.registered
                    if '/algorithm_simulator' in owned and '/base_mppi' in owned and not (out/'owned_pids.txt').exists():
                        (out/'owned_pids.txt').write_text('%d %d\n'%(owned['/algorithm_simulator'], owned['/base_mppi']))
                    if elapsed > 13 and not audited:
                        audit_graph(out); audited = True
                    if (out/'diagnostic_complete.json').exists():
                        if not audited:
                            audit_graph(out); audited = True
                        time.sleep(.5)  # Allow outer RAII traces to flush; not measured solver time.
                        break
                    if (out/'result.json').exists():
                        break  # Retain capture-not-reached; do not select or rerun.
                    if elapsed > 300:
                        raise RuntimeError('Diagnostic wall limit exceeded; incomplete data retained')
                    time.sleep(.2)
            finally:
                resume_owned(monitor.registered)
                try:
                    stop_owned(proc)
                finally:
                    monitor.close()
    except Exception as exc:
        error = type(exc).__name__+': '+str(exc)
    after = verify_production(build)
    logs = (out/'launch_stdout.log').read_text(errors='replace') if (out/'launch_stdout.log').exists() else ''
    status = dict(returncode=proc.returncode if proc else None, runner_error=error,
        graph_audited=audited, diagnostic_complete=(out/'diagnostic_complete.json').exists(),
        navigation_metrics_valid=False, result_written=(out/'result.json').exists(),
        elapsed_wall_s=time.monotonic()-started, monitored_node_pids=monitor.registered if monitor else {},
        production_hashes_unchanged=before == after,
        build_manifest_unchanged=sha(build/'build_manifest.json') == manifest_sha,
        required_process_failed=bool(re.search(r'REQUIRED process \[[^\]]+\] has died', logs)),
        scope='paused fixed inputs; no diagnostic candidate publication or online timing claim')
    (out/'run_status.json').write_text(json.dumps(status, indent=2)+'\n')
    from run_collab_factorial import compress_new_records
    compress_new_records(out)
    if (error or status['returncode'] != 0 or status['required_process_failed'] or not audited or
            not status['production_hashes_unchanged'] or not status['build_manifest_unchanged']):
        raise RuntimeError('Diagnostic interface/integrity failure; retained '+str(out))
    print('BACKEND_CAPTURE', out.name, 'captured='+str(status['diagnostic_complete']), flush=True)
    return runtime, status['diagnostic_complete']


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--build', type=Path, default=HERE/'builds/seed_parameterization_20261025_01')
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args(); build, out = a.build.resolve(), a.out.resolve()
    m, _ = check_build(build)
    if not m.get('seed_parameterization_experiment') or m.get('prepared_only'):
        raise ValueError('Requires compiled seed-fixed diagnostic build')
    check_space(out.parent, 2*1024**3); out.mkdir(parents=True, exist_ok=False)
    plan = dict(created=datetime.now(timezone.utc).isoformat(), build=str(build),
        build_manifest_sha256=sha(build/'build_manifest.json'), diagnostic_jobs=JOBS,
        scene='static', seed_parameterization_fix=True, diagnostic_budgets_s=[.6, .6, 1.5],
        execution_weight=0., handover_weight=0., boundary_admission=False, clearance_recovery=False,
        domain_recovery=True, signed_low_speed=True, no_outcome_selected_rerun=True,
        navigation_metrics_valid=False, diagnostic_candidates_published=False,
        ordinal_semantics='eligible optimizer call after reference, not FSM request or historical request ID',
        limitations=['newly captured inputs, not exact historical failed-request replays',
            'frozen observations/first fg are checked, full native map internal memory is not dumped',
            'post-solver candidate audits excluded from LBFGS budget; no live speed/safety claim'])
    (out/'plan.json').write_text(json.dumps(plan, indent=2)+'\n')
    completed, runtime = [], None
    try:
        for job in JOBS:
            name = 'static_diagnostic_seed%d_refresh%d'%(job['seed'], job['ordinal'])
            observed, captured = run_capture(out/name, build, job['seed'], job['ordinal'],
                plan['build_manifest_sha256'], runtime)
            if runtime is None:
                runtime = observed
            completed.append(dict(trial=name, kind='paused', captured=captured, **job))
            (out/'progress.json').write_text(json.dumps(completed, indent=2)+'\n')
    except Exception as exc:
        (out/'stopped.json').write_text(json.dumps(dict(completed=completed, reason=str(exc)), indent=2)+'\n')
        raise
    (out/'complete.json').write_text(json.dumps(dict(complete=True, completed=completed), indent=2)+'\n')
    print('BACKEND_FEASIBILITY_CAPTURES_COMPLETE', out, flush=True)


if __name__ == '__main__':
    main()
