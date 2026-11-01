#!/usr/bin/env python3
"""Replay the frozen native map and identical optimizer; no robot or simulator."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

import numpy as np
from audit_delivery_iterates import loads_preserving_cpp_zero

HERE=Path(__file__).resolve().parent
PARENT=HERE/'builds/endpoint_time_20261030_01'
TRIAL=HERE/'results/endpoint_time_20261030_01/midcross_wheel_seed11'
OUT=HERE/'results/root_cause_20261030_01'
NATIVE=OUT/'native_source'


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def build(version=1):
    suffix='' if version==1 else '_v%d'%version
    target=NATIVE/('rootcause_native_replay'+suffix)
    if target.exists():return target
    m=json.loads((PARENT/'build_manifest.json').read_text())
    recipe=m['endpoint_time_recipe']
    c=recipe['compile_commands'][0]['command'].copy()
    c[c.index('-c')+1]=str(HERE/'rootcause_native_replay.cpp')
    obj=NATIVE/('rootcause_native_replay'+suffix+'.o');c[c.index('-o')+1]=str(obj)
    c.insert(1,'-I'+str(NATIVE))
    link=recipe['link_commands']['remani_planner_node'].copy()
    link=[arg for arg in link if not arg.endswith('.cpp.o')]
    link.insert(1,str(obj));link[link.index('-o')+1]=str(target)
    commands=dict(compile=c,link=link,cpp_sha256=sha(HERE/'rootcause_native_replay.cpp'),
        helper_sha256=sha(NATIVE/'candidate_audit.hpp'),parent_manifest_sha256=sha(PARENT/'build_manifest.json'))
    for artifact in ('libexp_opt.a','libexp_search.a','libexp_model.a','libexp_env.a'):
        commands[artifact]=sha(PARENT/'build'/artifact)
    with (NATIVE/('compile'+suffix+'.log')).open('w') as log:
        subprocess.run(c,stdout=log,stderr=subprocess.STDOUT,check=True)
        subprocess.run(link,cwd=recipe['cwd'],stdout=log,stderr=subprocess.STDOUT,check=True)
    commands['binary_sha256']=sha(target)
    (NATIVE/('manifest'+suffix+'.json')).write_text(json.dumps(commands,indent=2)+'\n')
    print('BUILT',target,flush=True)
    return target


def prepare(path,solves,original=False,followup=False,conditioning=False,cold=False):
    rows=[loads_preserving_cpp_zero(l) for l in (TRIAL/'candidate_audit/endpoint_time_replay.jsonl').open()]
    runs=[r for r in rows if r['kind']=='run'];checks=[]
    def stripped(run,stage):
        return dict(input={k:run[k] for k in ('effective_initial_states','effective_final_states','directions','limits')},
            stage=dict(x=stage['x'],parts=[dict(times=p['times']) for p in stage['parts']]))
    for r in runs:
        for i,s in enumerate(r['stages']):
            checks.append(dict(label=r['label']+('_initial' if i==0 else '_last'),expected_cost=s['cost'],**stripped(r,s)))
    component_path=HERE/'results/rootcause_components_20261030_02/summary.json'
    if component_path.exists():
        comp=json.loads(component_path.read_text())
        row=next(s for s in comp['stages'] if s['run']=='endpoint150_060' and s['stage']=='accepted')
        g=np.asarray(row['component_gradients']['known_full_local'])
        r=runs[1];s=r['stages'][-1];updated=stripped(r,s)
        updated['stage']['x']=(np.asarray(s['x'])-1e-5*g/np.linalg.norm(g)).tolist()
        checks.append(dict(label='endpoint_last_small_descent',expected_cost=-1.,**updated))
    planned=[]
    if solves:
        endpoint=runs[1]
        for label,stage,budget in (
            ('endpoint_initial_060',endpoint['stages'][0],.6),
            ('endpoint_initial_120',endpoint['stages'][0],1.2),
            ('endpoint_last_restart_060',endpoint['stages'][-1],.6)):
            planned.append(dict(label=label,budget_s=budget,**stripped(endpoint,stage)))
    if original:
        r=runs[0]
        for budget in (.6,1.2,3.):
            planned.append(dict(label='original_initial_%03d'%round(budget*100),budget_s=budget,**stripped(r,r['stages'][0])))
    if followup:
        previous=[json.loads(l) for l in (OUT/'native_run_02/results.jsonl').open()]
        stopped=next(r for r in previous if r.get('label')=='original_initial_300')['stages'][-1]
        checkpoint=stripped(runs[0],runs[0]['stages'][0]);checkpoint['stage']['x']=stopped['x']
        checks.append(dict(label='original_converged_collision_checkpoint',expected_cost=stopped['cost'],**checkpoint))
        planned.append(dict(label='original_collision_restart_060',budget_s=.6,**checkpoint))
        planned.append(dict(label='original_delta1e6_300',budget_s=3.,delta=1e-6,**stripped(runs[0],runs[0]['stages'][0])))
    if conditioning:
        previous=[json.loads(l) for l in (OUT/'native_run_02/results.jsonl').open()]
        stopped=next(r for r in previous if r.get('label')=='original_initial_300')['stages'][-1]
        checkpoint=stripped(runs[0],runs[0]['stages'][0]);checkpoint['stage']['x']=stopped['x']
        checks.append(dict(label='original_converged_collision_checkpoint',expected_cost=stopped['cost'],**checkpoint))
        for scaled in (False,True):
            planned.append(dict(label='collision_checkpoint_'+('diagonal' if scaled else 'unscaled'),
                budget_s=.6,delta=1e-6,diagonal_precondition=scaled,**checkpoint))
    if cold:
        planned.append(dict(label='original_initial_diagonal_120',budget_s=1.2,
            diagonal_precondition=True,delta=1e-3,**stripped(runs[0],runs[0]['stages'][0])))
    sim=json.loads((TRIAL/'candidate_audit/map.json').read_text())['sim']
    data=dict(sim=sim,checks=checks,solves=planned,source_sha256=sha(__file__),
        snapshot_metadata_sha256=sha(TRIAL/'candidate_audit/map.json'),historical_case_imported=True)
    path.write_text(json.dumps(data,indent=2)+'\n');return data


def run(binary,number,solves,original=False,followup=False,conditioning=False,cold=False):
    dest=OUT/('native_run_%02d'%number);dest.mkdir(exist_ok=False)
    source=dest/'input.json';prepare(source,solves,original,followup,conditioning,cold)
    env=os.environ.copy()
    for key in list(env):
        if key.startswith(('FAILURE_','RESOURCE_','COLLAB_','COUPLING_','STATE_CONTRACT_','DELIVERY_',
                           'NATIVE_','CANDIDATE_','WHEEL_CAUSE_','ENDPOINT_TIME_','BUDGET_','SEED_PARAMETERIZATION_','FEASIBILITY_')):env.pop(key)
    port=11463
    env.update(ROS_MASTER_URI='http://127.0.0.1:%d'%port,ROS_IP='127.0.0.1',ROS_HOSTNAME='localhost',
        ROS_HOME=str(dest/'ros_home'),ROS_LOG_DIR=str(dest/'roslogs'),
        ROS_PACKAGE_PATH=str(HERE.parents[1]/'src')+':'+env.get('ROS_PACKAGE_PATH',''),
        NATIVE_COLLISION_3D='1',DELIVERY_FAST_QUERY='1',DELIVERY_ACCEPT_CANDIDATE='1',
        FAILURE_PROFILE_DISABLED='1',COUPLING_ENGINEERING='0',COUPLING_EXEC_WEIGHT='0',COUPLING_HAND_WEIGHT='0')
    processes=[]
    try:
        with (dest/'master.log').open('w') as master_log,(dest/'replay.log').open('w') as replay_log:
            master=subprocess.Popen(['/opt/ros/noetic/bin/roscore','-p',str(port)],env=env,
                stdout=master_log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(master)
            deadline=time.monotonic()+20.
            while True:
                if master.poll() is not None:raise RuntimeError('private ROS master exited')
                try:
                    with socket.create_connection(('127.0.0.1',port),timeout=.1):break
                except OSError:
                    if time.monotonic()>deadline:raise RuntimeError('private ROS master startup timeout')
                    time.sleep(.1)
            subprocess.run(['/opt/ros/noetic/bin/rosparam','load',str(TRIAL/'parameters.yaml')],env=env,
                stdout=replay_log,stderr=subprocess.STDOUT,check=True)
            proc=subprocess.Popen([str(binary),str(source),str(TRIAL/'candidate_audit/map.json'),str(dest/'results.jsonl')],
                env=env,stdout=replay_log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(proc)
            code=proc.wait(timeout=180.)
            if code:raise RuntimeError('native replay exited %d; inspect replay.log'%code)
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGTERM)
                try:proc.wait(timeout=8.)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
    results=[json.loads(l) for l in (dest/'results.jsonl').open()]
    for r in results:
        if r['kind']=='check':print(r['label'],'cost=',r['cost'],'native=',r['native_pass'],r['native_reason'],flush=True)
        else:
            passed=[s for s in r['stages'] if s['native_pass']]
            print(r['label'],'eval=',r['evaluations'],'accepted=',r['accepted_steps'],
                'first_pass=',passed[0]['elapsed_s'] if passed else None,
                'final_pass=',r['stages'][-1]['native_pass'],flush=True)
    (dest/'status.json').write_text(json.dumps(dict(complete=True,native_libraries_unchanged=True,
        complete_native_snapshot_replayed=True,simulator_started=False,controller_started=False,
        binary_sha256=sha(binary),input_sha256=sha(source),results_sha256=sha(dest/'results.jsonl')),indent=2)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--build',action='store_true');p.add_argument('--run',type=int)
    p.add_argument('--solves',action='store_true');p.add_argument('--original',action='store_true');p.add_argument('--followup',action='store_true')
    p.add_argument('--conditioning',action='store_true');p.add_argument('--cold',action='store_true');p.add_argument('--version',type=int,default=1);a=p.parse_args()
    binary=build(a.version) if a.build else NATIVE/('rootcause_native_replay'+('' if a.version==1 else '_v%d'%a.version))
    if a.run is not None:run(binary,a.run,a.solves,a.original,a.followup,a.conditioning,a.cold)
