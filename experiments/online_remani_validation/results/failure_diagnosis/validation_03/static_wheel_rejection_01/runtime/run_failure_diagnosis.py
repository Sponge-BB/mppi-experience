#!/usr/bin/env python3
"""Freeze actual live solver requests and run controlled native V12 re-solves."""
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
from xmlrpc.client import ServerProxy
from run_resource_comparison import prepare_trial, Monitor, PORT, HERE


def run_one(out,build,scene,kind):
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1',PORT))==0:raise RuntimeError('isolated master already in use')
    prepare_trial(out,build,'coupled',scene,85.,paired=False)
    env={k:v for k,v in os.environ.items() if not re.search('TOKEN|SECRET|PASSWORD|CREDENTIAL|AUTH|API_KEY',k,re.I)}
    env.update(ROS_MASTER_URI=f'http://127.0.0.1:{PORT}',ROS_IP='127.0.0.1',ROS_HOSTNAME='localhost',
        ROS_HOME=str(out/'ros_home'),ROS_LOG_DIR=str(out/'roslogs'),RESOURCE_TRACE_PATH=str(out/'solver_trace.jsonl'),
        FAILURE_TRACE_PATH=str(out/'failure_trace.jsonl'),FAILURE_DIAG_KIND=kind,
        FAILURE_OWNED_PID_FILE=str(out/'owned_pids.txt'),FAILURE_DONE_PATH=str(out/'diagnostic_complete.json'))
    env['ROS_PACKAGE_PATH']=str(out/'ros_pkgs')+':'+env.get('ROS_PACKAGE_PATH','')
    started=time.monotonic();monitor=None;status={};owned=[]
    with (out/'launch_stdout.log').open('w') as log:
        proc=subprocess.Popen(['roslaunch','-p',str(PORT),str(out/'experiment.launch')],env=env,
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        monitor=Monitor(proc.pid,out);monitor.start()
        try:
            while proc.poll() is None and time.monotonic()-started<160:
                if 'cannot launch node of type' in (out/'launch_stdout.log').read_text(errors='replace'):
                    raise RuntimeError('isolated node launch failed')
                sim=monitor.registered.get('/algorithm_simulator');mppi=monitor.registered.get('/base_mppi')
                if sim and mppi and not owned:
                    owned=[sim,mppi];(out/'owned_pids.txt').write_text(f'{sim} {mppi}\n')
                    code,_,graph=ServerProxy(f'http://127.0.0.1:{PORT}').getSystemState('/failure_diagnosis_audit')
                    if code!=1:raise RuntimeError('could not audit isolated graph')
                    publishers=dict(graph[0]);(out/'publisher_audit.json').write_text(json.dumps(publishers,indent=2)+'\n')
                    if publishers.get('/cmd_vel'):raise RuntimeError('unremapped cmd_vel publisher')
                    if publishers.get('/sim/cmd_vel')!=['/base_mppi']:
                        raise RuntimeError('unexpected command authority '+str(publishers.get('/sim/cmd_vel')))
                if (out/'diagnostic_complete.json').exists():
                    # Let the original RAII scopes flush after the matrix.
                    time.sleep(.5);break
                if (out/'result.json').exists():break
                time.sleep(.2)
        finally:
            for pid in owned:
                try:os.kill(pid,signal.SIGCONT)
                except ProcessLookupError:pass
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=8)
            monitor.close()
        text=(out/'launch_stdout.log').read_text(errors='replace')
        status=dict(scene=scene,requested=kind,elapsed_wall_s=time.monotonic()-started,
                    diagnostic_complete=(out/'diagnostic_complete.json').exists(),
                    process_failure=bool(re.search(r'REQUIRED process \[[^\]]+\] has died',text)),
                    node_pids=monitor.registered,returncode=proc.returncode,
                    navigation_metrics_valid=False,
                    scope='solver diagnostic pauses own simulator/MPPI; navigation timing after pause is excluded')
        (out/'diagnosis_status.json').write_text(json.dumps(status,indent=2)+'\n')
    print('FINISHED',out.name,status['diagnostic_complete'],flush=True)
    if not status['diagnostic_complete'] or status['process_failure']:
        raise RuntimeError('diagnostic incomplete: '+str(out))
    if not json.loads((out/'diagnostic_complete.json').read_text())['snapshot_stable']:
        raise RuntimeError('diagnostic snapshot changed')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--build',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--repeats',type=int,default=2)
    p.add_argument('--only',choices=['all','static','dynamic'],default='all');a=p.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    plan=dict(date='2026-10-16 Asia/Shanghai',scope='isolated native ROS numerical solver diagnosis, not Gazebo/physical',
        build=str(a.build.resolve()),repeats=a.repeats,budgets_s=[.6,1.5,3.],wheel_limits=[3.28,3.5],
        observer_control='same request re-solved with leaf timing enabled/disabled',
        scripts_sha256={name:hashlib.sha256((HERE/name).read_bytes()).hexdigest() for name in
            ['build_failure_diagnosis.py','failure_probe.hpp','run_failure_diagnosis.py','freeze_reference_probe.cpp']})
    (out/'experiment_plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    for repeat in range(a.repeats):
        cases=[('static','wheel_rejection'),('dynamic_four','timeout')]
        if repeat%2:cases.reverse()
        for scene,kind in cases:
            if a.only!='all' and (a.only=='static')!=(scene=='static'):continue
            run_one(out/f'{scene}_{kind}_{repeat+1:02d}',a.build.resolve(),scene,kind)


if __name__=='__main__':main()
