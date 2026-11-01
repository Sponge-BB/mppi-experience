#!/usr/bin/env python3
"""One prespecified dynamic input, two paused native backend budgets; never hardware."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import socket
import subprocess
import time
import xml.etree.ElementTree as ET

from run_coupled_objective import HERE, PORT, check_build, check_space, prepare_trial, sha
from run_delivery_stability import resume_owned
from run_dual_timescale import audit_graph, stop_owned, verify_production
from run_resource_comparison import Monitor
from run_sphere_scenarios import patch_trial, register_scenes

BUILD = HERE / 'builds/native67_probe_20261030_01'
OUT = HERE / 'results/native67_probe_20261030_01'
PHASES = ('native67_original_060', 'native67_extended_300')
PREDICATE = ('first normal 0.6s optimizer entry after reference exists, ordinary request '
             '(not dynamic_repair), valid nonempty native dynamic snapshot; '
             'ineligible entries do not consume refresh ordinal; no outcome selection')


def diagnostic_environment(out, seed, ordinal, inherited=None):
    inherited = os.environ if inherited is None else inherited
    prefixes = ('FAILURE_', 'RESOURCE_', 'COLLAB_', 'COUPLING_', 'STATE_CONTRACT_',
                'BOUNDARY_ADMISSION', 'DELIVERY_', 'SEED_PARAMETERIZATION_',
                'FEASIBILITY_', 'BUDGET_', 'SOLVER_SCALING_', 'NATIVE_')
    env = {k: v for k, v in inherited.items()
           if not re.search('TOKEN|SECRET|PASSWORD|CREDENTIAL|AUTH|API_KEY', k, re.I)
           and not k.startswith(prefixes)}
    env.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT, ROS_IP='127.0.0.1',
        ROS_HOSTNAME='localhost', ROS_HOME=str(out/'ros_home'), ROS_LOG_DIR=str(out/'roslogs'),
        ROS_PACKAGE_PATH=str(out/'ros_pkgs')+':'+env.get('ROS_PACKAGE_PATH', ''),
        DUAL_TIMELINE_PATH=str(out/'dual_timeline.jsonl'),
        RESOURCE_TRACE_PATH=str(out/'solver_trace.jsonl'),
        FAILURE_TRACE_PATH=str(out/'failure_trace.jsonl'),
        FAILURE_PROFILE_DISABLED='1',
        FAILURE_OWNED_PID_FILE=str(out/'owned_pids.txt'),
        COUPLING_ENGINEERING='1', COUPLING_EXEC_WEIGHT='0', COUPLING_HAND_WEIGHT='0',
        COUPLING_DOMAIN_RECOVERY='1', COUPLING_FEASIBILITY_RECOVERY='0',
        COUPLING_DIAGNOSTIC_DIR='', COLLAB_ROLLING='0', COLLAB_PREFIX='0', COLLAB_PAIR='0',
        COLLAB_SEED=str(seed), STATE_CONTRACT_CAPTURE='1', STATE_CONTRACT_SIGNED_LOW_SPEED='1',
        BOUNDARY_ADMISSION='0', SEED_PARAMETERIZATION_FIX='1', FEASIBILITY_STOP_GATE='0',
        DELIVERY_REPLAY_KIND='first_refresh', DELIVERY_REPLAY_REFRESH_ORDINAL=str(ordinal),
        DELIVERY_REPLAY_LOG=str(out/'delivery_replay.jsonl'),
        DELIVERY_REPLAY_DONE_PATH=str(out/'diagnostic_complete.json'))
    return env


def prepare_capture(out, build, seed=11, ordinal=1):
    register_scenes()
    settings = prepare_trial(out, build, 'sphere_timing_midcross', 'engineering',
                             seed, 55., 0., 0., True)
    # These are the exact fresh-scene, 67-sphere and light-recording settings
    # used in yesterday's timing batch. No old static projection harness.
    patch_trial(out, settings, 'midcross', 'b47')
    # The native online runtime explicitly disables old diagnostic replay in
    # each node's launch environment. Enable ONLY this planner's new probe.
    tree = ET.parse(out/'experiment.launch')
    nodes = {node.get('name'): node for node in tree.getroot().findall('node')}
    planner = nodes['remani_planner_node']
    for key, value in {'DELIVERY_REPLAY_KIND': 'first_refresh',
                       'FAILURE_PROFILE_DISABLED': '1'}.items():
        current = planner.findall("env[@name='%s']" % key)
        if current:
            current[0].set('value', value)
        else:
            ET.SubElement(planner, 'env', name=key, value=value)
    for node_name in ('remani_planner_node', 'base_mppi'):
        node_env = {env.get('name'): env.get('value') for env in nodes[node_name].findall('env')}
        for key in ('NATIVE_COLLISION_3D', 'NATIVE_REALTIME_DEMAND', 'NATIVE_DYNAMIC_FAST'):
            if node_env.get(key) != '1':
                raise ValueError('Missing unchanged native execution switch: '+node_name+'/'+key)
    tree.write(out/'experiment.launch', encoding='unicode')
    settings.update(state_contract_capture=True, signed_low_speed=True, domain_recovery=True,
        boundary_admission=False, clearance_recovery=False, seed_parameterization_fix=True,
        diagnostic_pause=True, genuine_online_closed_loop=False, navigation_metrics_valid=False,
        refresh_ordinal=ordinal, capture_predicate=PREDICATE,
        diagnostic_phases=list(PHASES), diagnostic_budgets_s=[.6, 3.],
        constraints_unchanged=True, objective_unchanged=True, initialization_unchanged=True,
        constraints_relaxed=False, supplemental_projected_final_gate=False,
        original_stop_policy=True, feasibility_stop_veto=False,
        detailed_profiling_only_inside_paused_solver=True,
        scope='new paused dynamic same-input backend diagnosis; no navigation improvement claim')
    path = out/'coupling_parameters.json'
    path.write_text(json.dumps(settings, ensure_ascii=False, indent=2)+'\n')
    path = out/'manifest.json'
    manifest = json.loads(path.read_text())
    manifest.update(coupling_experiment=settings, parameters_sha256=sha(out/'coupling_parameters.json'),
        native67_probe_experiment=True, navigation_metrics_valid=False,
        delivery_diagnostics=dict(enabled=False, native_probe_light_candidates=True,
            scope='initial/final canonical curves and scalar solve profile only; no full-map replay files'),
        diagnostic_candidates_published=False)
    manifest['timing_only_capture']['scope'] = 'light transport and records; paused backend profiling, not online timing comparison'
    manifest['timing_only_capture']['common_to_both_collision_models'] = False
    manifest['experiment_sources_sha256'][Path(__file__).name] = sha(Path(__file__))
    import shutil
    shutil.copy2(Path(__file__), out/'runtime'/Path(__file__).name)
    path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
    return manifest


def run_capture(out, build, seed=11, ordinal=1):
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1', PORT)) == 0:
            raise RuntimeError('Private ROS port already occupied; refuse attach')
    check_space(out.parent)
    build_manifest, _ = check_build(build)
    if not build_manifest.get('native67_probe_experiment') or build_manifest.get('prepared_only'):
        raise ValueError('Requires completed isolated native67 probe build')
    build_sha = sha(build/'build_manifest.json')
    before = verify_production(build)
    prepare_capture(out, build, seed, ordinal)
    env = diagnostic_environment(out, seed, ordinal)
    proc = monitor = None
    error = None
    audited = False
    cleanup_errors = []
    started = time.monotonic()
    try:
        with (out/'launch_stdout.log').open('w') as log:
            proc = subprocess.Popen(['roslaunch', '-p', str(PORT), str(out/'experiment.launch')],
                env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            monitor = Monitor(proc.pid, out)
            monitor.start()
            try:
                while proc.poll() is None:
                    elapsed = time.monotonic()-started
                    owned = monitor.registered
                    if '/algorithm_simulator' in owned and '/base_mppi' in owned and not (out/'owned_pids.txt').exists():
                        (out/'owned_pids.txt').write_text('%d %d\n' %
                            (owned['/algorithm_simulator'], owned['/base_mppi']))
                    if elapsed > 13 and not audited:
                        audit_graph(out)
                        audited = True
                    if (out/'diagnostic_complete.json').exists():
                        if not audited:
                            audit_graph(out)
                            audited = True
                        time.sleep(.5)
                        break
                    if (out/'result.json').exists():
                        break  # Capture not reached is retained, never silently replaced.
                    if elapsed > 180:
                        raise RuntimeError('180 s capture wall limit reached; incomplete data retained')
                    time.sleep(.2)
            finally:
                try:
                    resume_owned(monitor.registered)
                except Exception as exc:
                    cleanup_errors.append('resume: '+str(exc))
                try:
                    stop_owned(proc)
                except Exception as exc:
                    cleanup_errors.append('stop: '+str(exc))
                finally:
                    monitor.close()
    except Exception as exc:
        error = type(exc).__name__+': '+str(exc)
    after = verify_production(build)
    logs = (out/'launch_stdout.log').read_text(errors='replace') if (out/'launch_stdout.log').exists() else ''
    status = dict(returncode=proc.returncode if proc else None, runner_error=error,
        cleanup_errors=cleanup_errors, graph_audited=audited,
        diagnostic_complete=(out/'diagnostic_complete.json').exists(),
        navigation_metrics_valid=False, result_written=(out/'result.json').exists(),
        elapsed_wall_s=time.monotonic()-started,
        monitored_node_pids=monitor.registered if monitor else {},
        production_hashes_unchanged=before == after,
        build_manifest_unchanged=sha(build/'build_manifest.json') == build_sha,
        required_process_failed=bool(re.search(r'REQUIRED process \[[^\]]+\] has died', logs)),
        scope='paused fixed inputs; diagnostic candidates never published')
    (out/'run_status.json').write_text(json.dumps(status, indent=2)+'\n')
    from run_collab_factorial import compress_new_records
    compress_new_records(out)
    if (error or cleanup_errors or status['returncode'] != 0 or status['required_process_failed'] or
            not audited or not status['production_hashes_unchanged'] or not status['build_manifest_unchanged']):
        raise RuntimeError('Capture setup/integrity failed; retained: '+str(out))
    print('NATIVE67_CAPTURE', out.name, 'captured='+str(status['diagnostic_complete']), flush=True)
    return status


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--build', type=Path, default=BUILD)
    p.add_argument('--out', type=Path, default=OUT)
    a = p.parse_args()
    build, out = a.build.resolve(), a.out.resolve()
    check_space(out.parent)
    out.mkdir(parents=True, exist_ok=False)
    plan = dict(created=datetime.now(timezone.utc).isoformat(), build=str(build),
        build_manifest_sha256=sha(build/'build_manifest.json'),
        trials=[dict(directory='midcross_b67_seed11_eligible1', seed=11, ordinal=1)],
        scene='sphere_timing_midcross', goal=[6., 0.], bounds=[-1.5, 7.5, -2., 2.],
        actor=[3., -1.6, 0., .18, .23], collision_spheres=67,
        capture_predicate=PREDICATE, diagnostic_phases=list(PHASES), budgets_s=[.6, 3.],
        no_outcome_selected_rerun=True, navigation_metrics_valid=False,
        diagnostic_candidates_published=False,
        limitations=['one new input, not exact replay of yesterday\'s failed request',
            'same-process paused scene isolates backend budget; not proof of live delivery improvement',
            'only budget changes; no wheel relaxation, no new 2D final gate or feasibility stop rule',
            'initial/final native audits occur outside solver budget; not continuous collision certification'])
    (out/'plan.json').write_text(json.dumps(plan, ensure_ascii=False, indent=2)+'\n')
    job = plan['trials'][0]
    try:
        status = run_capture(out/job['directory'], build, job['seed'], job['ordinal'])
    except Exception as exc:
        (out/'stopped.json').write_text(json.dumps(dict(reason=str(exc)), indent=2)+'\n')
        raise
    completed = dict(**job, captured=status['diagnostic_complete'])
    (out/'complete.json').write_text(json.dumps(dict(complete=True, completed=[completed]), indent=2)+'\n')


if __name__ == '__main__':
    main()
