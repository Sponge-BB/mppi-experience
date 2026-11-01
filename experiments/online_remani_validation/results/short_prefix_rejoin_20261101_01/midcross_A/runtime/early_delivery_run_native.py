#!/usr/bin/env python3
"""Cheap coordinates in both arms; only B checks accepted iterates online."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

from run_cheap_scaling_native import HERE, PARENT, TRIAL, WORKSPACE, OLD_NATIVE, sha, freeze_text, request

OUT=HERE/'results/early_delivery_20261031_01'
NATIVE=OUT/'native_source'


def build():
    NATIVE.mkdir(parents=True,exist_ok=True)
    sources={}
    for path in (OLD_NATIVE/'candidate_audit.hpp',HERE/'cheap_variable_scaling.hpp',
                 HERE/'early_delivery_gate.hpp',HERE/'early_delivery_native_replay.cpp'):
        freeze_text(path,NATIVE/path.name);sources[str(path)]=sha(path)
    target=NATIVE/'early_delivery_native_replay';manifest=NATIVE/'manifest.json'
    if target.exists():
        old=json.loads(manifest.read_text())
        if old['source_sha256']!=sources or old['binary_sha256']!=sha(target):raise ValueError('Stale build')
        return target
    parent=json.loads((PARENT/'build_manifest.json').read_text());recipe=parent['endpoint_time_recipe']
    command=recipe['compile_commands'][0]['command'].copy()
    command[command.index('-c')+1]=str(NATIVE/'early_delivery_native_replay.cpp')
    obj=NATIVE/'early_delivery_native_replay.o';command[command.index('-o')+1]=str(obj)
    command.insert(1,'-I'+str(NATIVE))
    link=[a for a in recipe['link_commands']['remani_planner_node'] if not a.endswith('.cpp.o')]
    link.insert(1,str(obj));link[link.index('-o')+1]=str(target)
    evidence=dict(source_sha256=sources,parent_manifest_sha256=sha(PARENT/'build_manifest.json'),
        compile_command=command,link_command=link,
        shared_libraries={name:sha(PARENT/'build'/name) for name in ('libexp_opt.a','libexp_search.a','libexp_model.a','libexp_env.a')})
    with (NATIVE/'compile.log').open('w') as log:
        subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
        subprocess.run(link,cwd=recipe['cwd'],stdout=log,stderr=subprocess.STDOUT,check=True)
    evidence['binary_sha256']=sha(target);manifest.write_text(json.dumps(evidence,indent=2)+'\n')
    print('BUILT',target,flush=True);return target


def prepare(path,trial=TRIAL,repeats=3,checkpoint=False):
    item,cost,gradient_hash,name,proof=request(trial,checkpoint)
    checks=[dict(label=name+'_initial',expected_cost=cost,expected_gradient_hash=gradient_hash,**item)]
    solves=[]
    for repeat in range(repeats):
        order=(False,True) if repeat%2==0 else (True,False)
        for index,early in enumerate(order):
            solves.append(dict(label=('early' if early else 'current')+'_repeat%d'%(repeat+1),
                repeat=repeat+1,order_index=repeat*2+index,cheap_scaling=True,early_delivery=early,total_budget_s=1.2,**item))
    data=dict(schema='early_delivery_native_input_v1',request_name=name,source_proof=proof,
        sim=json.loads((trial/'candidate_audit/map.json').read_text())['sim'],checks=checks,solves=solves,
        runner_sha256=sha(__file__),initialization_unchanged=True,physical_constraints_unchanged=True,
        scale_proxy_unchanged=True,objective_unchanged=True,native_gate_unchanged=True,stopping_delta=1e-3,
        predeclared_rule='B: accepted step wheel preview on native 10ms grid, native IsTrajSafe at most every 100ms since previous check start, exit only after pass inside deadline. A: current exit. Both retain real final native gate; actual total and lateness recorded.',
        nominal_total_budget_s=1.2,all_realtime_gate_cost_counted=True,
        a_tail_validation_may_overrun_and_is_reported=True,posthoc_feasible_is_not_live_exit=True)
    path.write_text(json.dumps(data,indent=2)+'\n');return data


def finalize(dest,binary,wall_s):
    data=json.loads((dest/'input.json').read_text());raw=dest/'results.jsonl'
    rows=[json.loads(line) for line in raw.open()];check=next(r for r in rows if r['kind']=='check')
    solves=[r for r in rows if r['kind']=='solve']
    if len(solves)!=len(data['solves']):raise ValueError('Incomplete comparison')
    for row in solves:
        if row['initial_native_x_hash']!=check['x_hash'] or row['initial_native_gradient_hash']!=check['gradient_hash'] or row['initial_native_cost']!=check['cost']:
            raise ValueError('Physical initial x/f/g differs')
        if row['live_early_selected']:
            selected=[c for c in row['live_checks'] if c['selected']]
            if len(selected)!=1 or not selected[0]['wheel_pass'] or not selected[0]['native_checked'] or not selected[0]['native_pass'] or selected[0]['budget_expired']:
                raise ValueError('Early exit lacks a real predeadline native certificate')
            if not row['live_candidate_native_pass']:raise ValueError('Repeated native final gate disagrees with live check on fixed map')
        print(row['label'],'prepare=',row['setup_s'],'real_end=',row['live_total_return_s'],
            'native=',row['live_candidate_native_pass'],'ontime=',row['live_on_time_valid_candidate'],
            'real_checks=',row['live_native_check_count'],'early_selected=',row['live_early_selected'],flush=True)
    (dest/'status.json').write_text(json.dumps(dict(complete=True,request_name=data['request_name'],
        source_proof=data['source_proof'],wall_s=wall_s,physical_initial_identity=True,initial_native_gradient_hash=check['gradient_hash'],
        real_final_gate_timed_before_posthoc_audits=True,posthoc_candidates_not_selected=True,
        initial_x_hash=check['x_hash'],binary_sha256=sha(binary),input_sha256=sha(dest/'input.json'),results_sha256=sha(raw),
        simulator_started=False,controller_started=False,actual_online_publication_measured=False),indent=2)+'\n')


def run(binary,number,trial=TRIAL,repeats=3,checkpoint=False):
    dest=OUT/('native_run_%02d'%number);dest.mkdir(parents=True,exist_ok=False)
    source=dest/'input.json';prepare(source,trial,repeats,checkpoint)
    env=os.environ.copy()
    for key in list(env):
        if key.startswith(('FAILURE_','RESOURCE_','COLLAB_','COUPLING_','STATE_CONTRACT_','DELIVERY_',
                           'NATIVE_','CANDIDATE_','WHEEL_CAUSE_','ENDPOINT_TIME_','BUDGET_','SEED_PARAMETERIZATION_','FEASIBILITY_')):
            env.pop(key)
    port=11465
    env.update(ROS_MASTER_URI='http://127.0.0.1:%d'%port,ROS_IP='127.0.0.1',ROS_HOSTNAME='localhost',
        ROS_HOME=str(dest/'ros_home'),ROS_LOG_DIR=str(dest/'roslogs'),
        ROS_PACKAGE_PATH=str(WORKSPACE/'src')+':'+env.get('ROS_PACKAGE_PATH',''),NATIVE_COLLISION_3D='1',
        DELIVERY_FAST_QUERY='1',DELIVERY_ACCEPT_CANDIDATE='1',FAILURE_PROFILE_DISABLED='1',
        COUPLING_ENGINEERING='0',COUPLING_EXEC_WEIGHT='0',COUPLING_HAND_WEIGHT='0')
    owned=[];began=time.monotonic()
    try:
        with (dest/'master.log').open('w') as master_log,(dest/'replay.log').open('w') as replay_log:
            master=subprocess.Popen(['/opt/ros/noetic/bin/roscore','-p',str(port)],env=env,
                stdout=master_log,stderr=subprocess.STDOUT,start_new_session=True);owned.append(master)
            deadline=time.monotonic()+20.
            while True:
                if master.poll() is not None:raise RuntimeError('Private master exited')
                try:
                    with socket.create_connection(('127.0.0.1',port),timeout=.1):break
                except OSError:
                    if time.monotonic()>deadline:raise RuntimeError('Private master timeout')
                    time.sleep(.1)
            subprocess.run(['/opt/ros/noetic/bin/rosparam','load',str(trial/'parameters.yaml')],env=env,
                stdout=replay_log,stderr=subprocess.STDOUT,check=True)
            child=subprocess.Popen([str(binary),str(source),str(trial/'candidate_audit/map.json'),str(dest/'results.jsonl')],
                env=env,stdout=replay_log,stderr=subprocess.STDOUT,start_new_session=True);owned.append(child)
            code=child.wait(timeout=240.)
            if code:raise RuntimeError('Native replay exited %d; inspect replay.log'%code)
    finally:
        for child in reversed(owned):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGTERM)
                try:child.wait(timeout=8.)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
    finalize(dest,binary,time.monotonic()-began)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--build',action='store_true');p.add_argument('--run',type=int)
    p.add_argument('--case-root',type=Path,default=TRIAL);p.add_argument('--repeats',type=int,default=3)
    p.add_argument('--checkpoint',action='store_true');p.add_argument('--prepare',type=Path)
    a=p.parse_args();binary=build() if a.build else NATIVE/'early_delivery_native_replay'
    if a.prepare is not None:prepare(a.prepare.resolve(),a.case_root.resolve(),a.repeats,a.checkpoint)
    if a.run is not None:run(binary,a.run,a.case_root.resolve(),a.repeats,a.checkpoint)
