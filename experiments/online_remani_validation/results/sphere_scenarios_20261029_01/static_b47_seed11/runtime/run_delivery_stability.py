#!/usr/bin/env python3
"""Two unchanged-policy static loops then four NEW paused solver captures.

No hardware. No candidate produced by a diagnostic solve is published. Paused
captures are NOT navigation/performance trials or historical complete replays.
"""
import argparse
from datetime import datetime,timezone
import json
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import time

from run_coupled_objective import HERE,PORT,check_build,check_space,prepare_trial,run_trial,sha
from run_dual_timescale import audit_graph,stop_owned,verify_production
from run_resource_comparison import Monitor


def set_diagnostic_settings(out,settings,ordinal):
    settings.update(state_contract_capture=True,signed_low_speed=True,domain_recovery=True,
        boundary_admission=False,boundary_admission_static_scene=True,clearance_recovery=False,
        diagnostic_pause=True,genuine_online_closed_loop=False,
        navigation_metrics_valid=False,refresh_ordinal=ordinal,
        scope='newly captured same-process paused native static solver diagnostic; no hardware or historical replay')
    params=out/'coupling_parameters.json';params.write_text(json.dumps(settings,indent=2)+'\n')
    path=out/'manifest.json';m=json.loads(path.read_text())
    m.update(coupling_experiment=settings,parameters_sha256=sha(params),
        delivery_stability_diagnostic=True,navigation_metrics_valid=False,
        delivery_diagnostics=dict(enabled=True,scope='diagnostic frozen frame/canonical accepted iterates, not a speed benchmark'))
    path.write_text(json.dumps(m,indent=2,ensure_ascii=False)+'\n')


def resume_owned(monitored):
    # Only the two PIDs registered by THIS experiment, after checking ROS names.
    for node in ('/algorithm_simulator','/base_mppi'):
        pid=monitored.get(node)
        if not isinstance(pid,int) or pid<=1:continue
        try:
            raw=(Path('/proc')/str(pid)/'cmdline').read_bytes().decode(errors='replace')
            if '__name:='+node[1:] in raw:os.kill(pid,signal.SIGCONT)
        except ProcessLookupError:pass
        except FileNotFoundError:pass


def run_diagnostic(out,build,seed,ordinal,expected_runtime=None):
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1',PORT))==0:raise RuntimeError('Existing ROS11441 graph; refuse attach')
    check_space(out.parent,2*1024**3);m,_=check_build(build);before=verify_production(build)
    manifest_sha=sha(build/'build_manifest.json')
    settings=prepare_trial(out,build,'static','coupled',seed,55.,0.,0.,False)
    set_diagnostic_settings(out,settings,ordinal)
    runtime=json.loads((out/'manifest.json').read_text())['experiment_sources_sha256']
    if expected_runtime is not None and runtime!=expected_runtime:raise RuntimeError('Runtime changed before diagnostic, no ROS started')
    (out/'delivery_diagnostics').mkdir(exist_ok=False)
    env={k:v for k,v in os.environ.items() if not re.search('TOKEN|SECRET|PASSWORD|CREDENTIAL|AUTH|API_KEY',k,re.I)
        and not k.startswith(('FAILURE_','RESOURCE_','COLLAB_','COUPLING_','STATE_CONTRACT_','BOUNDARY_ADMISSION','DELIVERY_'))}
    env.update(ROS_MASTER_URI='http://127.0.0.1:%d'%PORT,ROS_IP='127.0.0.1',ROS_HOSTNAME='localhost',
        ROS_HOME=str(out/'ros_home'),ROS_LOG_DIR=str(out/'roslogs'),
        ROS_PACKAGE_PATH=str(out/'ros_pkgs')+':'+env.get('ROS_PACKAGE_PATH',''),
        DUAL_TIMELINE_PATH=str(out/'dual_timeline.jsonl'),RESOURCE_TRACE_PATH=str(out/'solver_trace.jsonl'),
        FAILURE_TRACE_PATH=str(out/'failure_trace.jsonl'),FAILURE_PROFILE_DISABLED='1',
        FAILURE_OWNED_PID_FILE=str(out/'owned_pids.txt'),COUPLING_ENGINEERING='1',
        COUPLING_EXEC_WEIGHT='0',COUPLING_HAND_WEIGHT='0',COUPLING_DOMAIN_RECOVERY='1',
        COUPLING_DIAGNOSTIC_DIR=str(out/'delivery_diagnostics'),COLLAB_ROLLING='0',COLLAB_PREFIX='0',COLLAB_PAIR='0',
        COLLAB_SEED=str(seed),STATE_CONTRACT_CAPTURE='1',STATE_CONTRACT_SIGNED_LOW_SPEED='1',
        BOUNDARY_ADMISSION='0',BOUNDARY_ADMISSION_STATIC_SCENE='1',DELIVERY_SEARCH_PROBE='1',
        DELIVERY_REPLAY_KIND='first_refresh',DELIVERY_REPLAY_REFRESH_ORDINAL=str(ordinal),
        DELIVERY_REPLAY_LOG=str(out/'delivery_replay.jsonl'),DELIVERY_REPLAY_DONE_PATH=str(out/'diagnostic_complete.json'))
    proc=None;monitor=None;error=None;audited=False;started=time.monotonic()
    try:
        with (out/'launch_stdout.log').open('w') as log:
            proc=subprocess.Popen(['roslaunch','-p',str(PORT),str(out/'experiment.launch')],env=env,
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            monitor=Monitor(proc.pid,out);monitor.start()
            try:
                while proc.poll() is None:
                    elapsed=time.monotonic()-started;owned=monitor.registered
                    if '/algorithm_simulator' in owned and '/base_mppi' in owned and not (out/'owned_pids.txt').exists():
                        (out/'owned_pids.txt').write_text('%d %d\n'%(owned['/algorithm_simulator'],owned['/base_mppi']))
                    if elapsed>13 and not audited:audit_graph(out);audited=True
                    if (out/'diagnostic_complete.json').exists():
                        if not audited:audit_graph(out);audited=True
                        time.sleep(.5);break  # outer trace RAII flush, not measurement
                    if (out/'result.json').exists():break
                    if elapsed>300:raise RuntimeError('Diagnostic wall limit, keep incomplete without rerun')
                    time.sleep(.2)
            finally:
                resume_owned(monitor.registered)
                try:stop_owned(proc)
                finally:monitor.close()
    except Exception as exc:error=type(exc).__name__+': '+str(exc)
    after=verify_production(build)
    if before!=after or sha(build/'build_manifest.json')!=manifest_sha:error='Integrity changed after diagnostic'
    logs=(out/'launch_stdout.log').read_text(errors='replace') if (out/'launch_stdout.log').exists() else ''
    status=dict(returncode=proc.returncode if proc else None,runner_error=error,graph_audited=audited,
        diagnostic_complete=(out/'diagnostic_complete.json').exists(),navigation_metrics_valid=False,
        result_written=(out/'result.json').exists(),elapsed_wall_s=time.monotonic()-started,
        monitored_node_pids=monitor.registered if monitor else {},production_hashes_unchanged=before==after,
        build_manifest_unchanged=sha(build/'build_manifest.json')==manifest_sha,
        required_process_failed=bool(re.search(r'REQUIRED process \[[^\]]+\] has died',logs)),
        scope='paused request repeats; not navigation duration, realtime or safety comparison')
    (out/'run_status.json').write_text(json.dumps(status,indent=2)+'\n')
    from run_collab_factorial import compress_new_records
    compress_new_records(out)
    if error or status['required_process_failed'] or not audited:raise RuntimeError('Diagnostic interface failure: '+str(status))
    print('DIAGNOSTIC_CAPTURE',out.name,'complete='+str(status['diagnostic_complete']),flush=True)
    return runtime,status['diagnostic_complete']


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--build',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);a=p.parse_args();build=a.build.resolve();out=a.out.resolve()
    m,_=check_build(build)
    if not m.get('delivery_stability_experiment') or m.get('delivery_stability_prepared_only'):
        raise ValueError('Requires compiled delivery diagnostic build')
    check_space(out.parent,2*1024**3);out.mkdir(parents=True,exist_ok=False)
    plan=dict(created=datetime.now(timezone.utc).isoformat(),build=str(build),build_manifest_sha256=sha(build/'build_manifest.json'),
        scene='static',normal_seeds=[11,12],diagnostic_jobs=[dict(seed=11,ordinal=1),dict(seed=12,ordinal=2),
            dict(seed=11,ordinal=2),dict(seed=12,ordinal=1)],execution_weight=0.,handover_weight=0.,
        diagnostic_budgets_s=[.6,.6,1.5],admission=False,domain_recovery=True,signed_low_speed=True,
        declaration='soft prediction coupling disabled to isolate delivery; differs from prior weights1/1 batch, not a causal speed comparison',
        no_outcome_selected_rerun=True,navigation_metrics_diagnostics_valid=False)
    (out/'plan.json').write_text(json.dumps(plan,indent=2)+'\n');done=[];runtime=None
    try:
        for key in list(os.environ):
            if key.startswith(('DELIVERY_REPLAY_','DELIVERY_SEARCH_')):os.environ.pop(key)
        os.environ['DELIVERY_SEARCH_PROBE']='1'
        for seed in plan['normal_seeds']:
            name='static_capture_seed%d'%seed
            valid=run_trial(out/name,build,'static','coupled',seed,48.,0.,0.,False,
                plan['build_manifest_sha256'],expected_runtime_sources=runtime,reserve_bytes=2*1024**3,
                diagnostic_capture=True,domain_recovery=True,state_contract=True,signed_low_speed=True,boundary_admission=False)
            observed=json.loads((out/name/'manifest.json').read_text())['experiment_sources_sha256']
            if runtime is None:runtime=observed
            if observed!=runtime:raise RuntimeError('Full snapshot drift')
            done.append(dict(trial=name,kind='online',valid=valid))
            (out/'progress.json').write_text(json.dumps(done,indent=2)+'\n')
            if not valid:raise RuntimeError('Online interface failure; all data retained')
        for job in plan['diagnostic_jobs']:
            name='static_diagnostic_seed%d_refresh%d'%(job['seed'],job['ordinal'])
            _,complete=run_diagnostic(out/name,build,job['seed'],job['ordinal'],runtime)
            done.append(dict(trial=name,kind='paused',captured=complete))
            (out/'progress.json').write_text(json.dumps(done,indent=2)+'\n')
    except Exception as exc:
        (out/'stopped.json').write_text(json.dumps(dict(completed=done,reason=str(exc)),indent=2)+'\n');raise
    (out/'complete.json').write_text(json.dumps(dict(complete=True,completed=done),indent=2)+'\n')
    print('DELIVERY_STABILITY_BATCH_COMPLETE',out,flush=True)


if __name__=='__main__':main()
