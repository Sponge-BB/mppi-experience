#!/usr/bin/env python3
"""Two-arm native replay with one joint preparation/solver wall-time budget."""
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
from audit_delivery_iterates import loads_preserving_cpp_zero, vector_hash

HERE=Path(__file__).resolve().parent
WORKSPACE=HERE.parents[1]
PARENT=HERE/'builds/endpoint_time_20261030_01'
TRIAL=HERE/'results/endpoint_time_20261030_01/midcross_wheel_seed11'
OUT=HERE/'results/cheap_scaling_20261030_01'
NATIVE=OUT/'native_source'
OLD_NATIVE=HERE/'results/root_cause_20261030_01/native_source'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def freeze_text(source,target):
    text=Path(source).read_text()
    if target.exists():
        if target.read_text()!=text:raise RuntimeError('Different frozen source exists: '+str(target))
        return
    patch='*** Begin Patch\n*** Add File: '+str(target)+'\n'+''.join('+'+line+'\n' for line in text.splitlines())+'*** End Patch\n'
    subprocess.run(['apply_patch'],input=patch,text=True,check=True)


def build():
    NATIVE.mkdir(parents=True,exist_ok=True)
    frozen={}
    for source in (OLD_NATIVE/'candidate_audit.hpp',HERE/'cheap_variable_scaling.hpp',HERE/'cheap_scaling_native_replay.cpp'):
        target=NATIVE/source.name;freeze_text(source,target);frozen[str(source)]=sha(source)
    manifest_path=NATIVE/'manifest.json';target=NATIVE/'cheap_scaling_native_replay'
    if target.exists():
        saved=json.loads(manifest_path.read_text())
        if saved['frozen_source_sha256']!=frozen or saved['binary_sha256']!=sha(target):
            raise RuntimeError('Existing binary provenance mismatch')
        return target
    parent_manifest=json.loads((PARENT/'build_manifest.json').read_text())
    recipe=parent_manifest['endpoint_time_recipe'];command=recipe['compile_commands'][0]['command'].copy()
    command[command.index('-c')+1]=str(NATIVE/'cheap_scaling_native_replay.cpp')
    obj=NATIVE/'cheap_scaling_native_replay.o';command[command.index('-o')+1]=str(obj)
    command.insert(1,'-I'+str(NATIVE))
    link=[a for a in recipe['link_commands']['remani_planner_node'] if not a.endswith('.cpp.o')]
    link.insert(1,str(obj));link[link.index('-o')+1]=str(target)
    record=dict(compile=command,link=link,frozen_source_sha256=frozen,
        parent_manifest_sha256=sha(PARENT/'build_manifest.json'),shared_native_libraries={})
    for artifact in ('libexp_opt.a','libexp_search.a','libexp_model.a','libexp_env.a'):
        record['shared_native_libraries'][artifact]=sha(PARENT/'build'/artifact)
    with (NATIVE/'compile.log').open('w') as log:
        subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
        subprocess.run(link,cwd=recipe['cwd'],stdout=log,stderr=subprocess.STDOUT,check=True)
    record['binary_sha256']=sha(target)
    manifest_path.write_text(json.dumps(record,indent=2)+'\n')
    print('BUILT',target,flush=True)
    return target


def request(trial,checkpoint=False):
    case_path=trial/'candidate_audit/case.json';case=loads_preserving_cpp_zero(case_path.read_text())
    meta_path=trial/'candidate_audit/map.json';meta=json.loads(meta_path.read_text())
    checks=case['frozen_checks']
    if not all(checks[k] for k in ('sim_frozen','raw_esdf_stable','overlay_key_stable','final_matches_saved_and_actual_verdict')):
        raise ValueError('Frozen case invariants incomplete')
    stage=case['stages'][0]
    if stage['evaluation_begin_key']['sequence']!=meta['overlay_sequence'] or stage['evaluation_end_key']['sequence']!=meta['overlay_sequence']:
        raise ValueError('Captured initial objective used a different overlay from the saved map')
    name=trial.parent.name+'__'+trial.name;gradient_hash=''
    original_run=None
    for file in ('endpoint_time_replay.jsonl','wheel_cause_replay.jsonl'):
        path=trial/'candidate_audit'/file
        if path.exists():
            rows=[loads_preserving_cpp_zero(line) for line in path.open()]
            original_run=next(r for r in rows if r.get('label')=='original_060')
            if original_run['stages'][0]['x']!=stage['x']:
                raise ValueError('Case and replay initial coordinates differ')
            gradient_hash=original_run['solver']['first_gradient_hash'];break
    if original_run is not None:
        limits=original_run['limits']
    else:
        limits=dict(wheel_limit=case['params']['wheel_limit'],alpha_limit=case['params']['alpha_limit'])
    item=dict(input={k:case[k] for k in ('effective_initial_states','effective_final_states','directions')},
        stage=dict(x=stage['x'],parts=[dict(times=p['times']) for p in stage['parts']]));item['input']['limits']=limits
    expected_cost=stage['cost'];scope='independent captured original optimizer request'
    if checkpoint:
        if trial.resolve()!=TRIAL.resolve():raise ValueError('Checkpoint only belongs to the latest captured request')
        previous=HERE/'results/root_cause_20261030_01/native_run_02/results.jsonl'
        rows=[json.loads(l) for l in previous.open()]
        saved=next(r for r in rows if r.get('label')=='original_initial_300')['stages'][-1]
        item['stage']['x']=saved['x'];expected_cost=saved['cost'];gradient_hash=''
        check_path=HERE/'results/root_cause_20261030_01/native_run_03/results.jsonl'
        found=next(r for r in (json.loads(l) for l in check_path.open()) if r.get('label')=='original_converged_collision_checkpoint')
        gradient_hash=found['gradient_hash'];name+='__collision_checkpoint'
        scope='checkpoint of the latest request; not another independent request'
    proof=dict(case_path=str(case_path),case_sha256=sha(case_path),map_metadata_sha256=sha(meta_path),
        parameters_sha256=sha(trial/'parameters.yaml'),captured_request_id=case['request_id'],
        captured_input_hash=case['input_hash'],input_scope=scope,independent_request=not checkpoint,
        initial_overlay_matches_saved_map=True,original_run_available=original_run is not None)
    return item,expected_cost,gradient_hash,name,proof


def prepare(path,trial=TRIAL,repeats=3,checkpoint=False):
    item,cost,gradient_hash,name,proof=request(trial,checkpoint)
    checks=[dict(label=name+'_initial',expected_cost=cost,expected_gradient_hash=gradient_hash,**item)]
    planned=[]
    for repeat in range(repeats):
        order=(False,True) if repeat%2==0 else (True,False)
        for which,scaled in enumerate(order):
            planned.append(dict(label=('cheap' if scaled else 'unscaled')+'_repeat%d'%(repeat+1),
                repeat=repeat+1,order_index=repeat*2+which,cheap_scaling=scaled,total_budget_s=1.2,**item))
    data=dict(schema='cheap_scaling_native_input_v1',sim=json.loads((trial/'candidate_audit/map.json').read_text())['sim'],
        request_name=name,checks=checks,solves=planned,source_proof=proof,runner_sha256=sha(__file__),
        stopping_delta=1e-3,physical_objective_unchanged=True,final_gate_unchanged=True,
        budget_scope='cheap Context construction + scale estimation + complete native solver, 1.2 seconds total',
        historical_feasibility_scope='all accepted candidates reconstructed and gated after the timed solve; no early publication')
    path.write_text(json.dumps(data,indent=2)+'\n');return data


def finalize_existing(number,binary,recovered=False,wall_s=None):
    dest=OUT/('native_run_%02d'%number)
    source=dest/'input.json';data=json.loads(source.read_text())
    raw=dest/'results.jsonl';rows=[json.loads(line) for line in raw.open()]
    baseline=next(row for row in rows if row['kind']=='check')
    solves=[row for row in rows if row['kind']=='solve']
    planned={row['label']:row for row in data['solves']}
    if len(solves)!=len(planned) or len({row['label'] for row in solves})!=len(planned):
        raise ValueError('Missing or duplicate completed comparison arms')
    identity=[]
    for row in solves:
        x=np.asarray(planned[row['label']]['stage']['x'],dtype=np.float64)
        scale=np.asarray(row['scales'],dtype=np.float64)
        initial=x+scale*np.zeros(len(x)) if row['cheap_scaling'] else x.copy()
        if not np.array_equal(x,initial):raise ValueError('Scaled coordinates change the numerical initial x')
        changed=np.flatnonzero(x.view(np.uint64)!=initial.view(np.uint64))
        for index in changed:
            if x[index]!=0. or initial[index]!=0. or int(x.view(np.uint64)[index]^initial.view(np.uint64)[index])!=(1<<63):
                raise ValueError('Initial bit difference is not only signed zero')
        if vector_hash(x.tolist())!=baseline['x_hash'] or vector_hash(initial.tolist())!=row['initial_native_x_hash']:
            raise ValueError('Observed initial hash does not match independently reconstructed coordinates')
        if row['initial_native_gradient_hash']!=baseline['gradient_hash'] or row['initial_native_cost']!=baseline['cost']:
            raise ValueError('Physical initial cost or unmodified gradient hash differs')
        identity.append(dict(label=row['label'],numeric_x_equal=True,initial_cost_exactly_equal=True,
            original_gradient_hash_exactly_equal=True,source_x_hash=baseline['x_hash'],
            observed_x_hash=row['initial_native_x_hash'],reconstructed_x_hash=vector_hash(initial.tolist()),
            bitwise_x_equal=len(changed)==0,changed_indices=changed.tolist(),differences_only_signed_zero=True,
            max_absolute_x_difference=float(np.max(abs(initial-x)))))
    previous=NATIVE/'run_cheap_scaling_native_before_signed_zero.py'
    runner_matches_current=data['runner_sha256']==sha(__file__)
    runner_matches_preserved=previous.exists() and data['runner_sha256']==sha(previous)
    if not (runner_matches_current or runner_matches_preserved):raise ValueError('Input runner provenance is not attested')
    proof=dict(complete=True,passed=True,checks=identity,input_sha256=sha(source),results_sha256=sha(raw),
        original_run_validation_failed=bool(recovered),recovered_without_rerun=bool(recovered),
        preserved_runner_sha256=sha(previous) if previous.exists() else None,
        input_runner_sha256=data['runner_sha256'],validator_runner_sha256=sha(__file__),
        explanation='The exact physical x and f0, and the original native gradient hash match. Adding scale*zero changes only negative zero signs in fixed-arm coordinates; original bit hashes are retained.')
    proof_path=dest/'initial_identity_verification.json'
    if proof_path.exists():raise ValueError('Refusing to overwrite an existing identity proof')
    proof_path.write_text(json.dumps(proof,indent=2)+'\n')
    for row in solves:
        passed=[stage for stage in row['stages'] if stage['native_pass']]
        print(row['label'],'prepare=',row['setup_s'],'solver=',row['solver_s'],
            'posthoc_first_feasible_total=',passed[0]['prepare_plus_solver_elapsed_s'] if passed else None,
            'returned=',row['returned_audit']['native_pass'],'timeout_policy=',row['existing_timeout_policy_audit'],flush=True)
    status=dest/'status.json'
    if status.exists():raise ValueError('Refusing to overwrite completed run status')
    status.write_text(json.dumps(dict(complete=True,request_name=data['request_name'],wall_s=wall_s,
        source_proof=data['source_proof'],initial_native_hash_identity=all(v['bitwise_x_equal'] for v in identity),
        initial_native_physical_identity=True,initial_native_cost_gradient_hash_identity=True,
        signed_zero_identity_proof=str(proof_path),signed_zero_identity_proof_sha256=sha(proof_path),
        original_run_validation_failed=bool(recovered),
        original_failure_reason='strict initial x bit-hash rejected only signed-zero differences' if recovered else None,
        finalized_without_rerun=bool(recovered),shared_budget_prepare_plus_solver_s=1.2,
        physical_objective_unchanged=True,native_gate_unchanged=True,actual_online_publication_measured=False,
        posthoc_first_feasible_is_not_publication=True,simulator_started=False,controller_started=False,
        binary_sha256=sha(binary),input_sha256=sha(source),results_sha256=sha(raw)),indent=2)+'\n')


def run(binary,number,trial=TRIAL,repeats=3,checkpoint=False):
    dest=OUT/('native_run_%02d'%number);dest.mkdir(parents=True,exist_ok=False)
    source=dest/'input.json';data=prepare(source,trial,repeats,checkpoint)
    env=os.environ.copy()
    for key in list(env):
        if key.startswith(('FAILURE_','RESOURCE_','COLLAB_','COUPLING_','STATE_CONTRACT_','DELIVERY_',
                           'NATIVE_','CANDIDATE_','WHEEL_CAUSE_','ENDPOINT_TIME_','BUDGET_','SEED_PARAMETERIZATION_','FEASIBILITY_')):
            env.pop(key)
    port=11464
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
                if master.poll() is not None:raise RuntimeError('Private ROS master exited')
                try:
                    with socket.create_connection(('127.0.0.1',port),timeout=.1):break
                except OSError:
                    if time.monotonic()>deadline:raise RuntimeError('Private ROS master timeout')
                    time.sleep(.1)
            subprocess.run(['/opt/ros/noetic/bin/rosparam','load',str(trial/'parameters.yaml')],env=env,
                stdout=replay_log,stderr=subprocess.STDOUT,check=True)
            child=subprocess.Popen([str(binary),str(source),str(trial/'candidate_audit/map.json'),str(dest/'results.jsonl')],
                env=env,stdout=replay_log,stderr=subprocess.STDOUT,start_new_session=True);owned.append(child)
            code=child.wait(timeout=240.)
            if code:raise RuntimeError('Replay exited %d; inspect replay.log'%code)
    finally:
        for child in reversed(owned):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGTERM)
                try:child.wait(timeout=8.)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
    finalize_existing(number,binary,wall_s=time.monotonic()-began)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build',action='store_true');parser.add_argument('--run',type=int)
    parser.add_argument('--case-root',type=Path,default=TRIAL);parser.add_argument('--repeats',type=int,default=3)
    parser.add_argument('--checkpoint',action='store_true');parser.add_argument('--prepare',type=Path)
    parser.add_argument('--finalize-existing',type=int)
    args=parser.parse_args()
    binary=build() if args.build else NATIVE/'cheap_scaling_native_replay'
    if args.prepare is not None:prepare(args.prepare.resolve(),args.case_root.resolve(),args.repeats,args.checkpoint)
    if args.finalize_existing is not None:finalize_existing(args.finalize_existing,binary,recovered=True)
    if args.run is not None:run(binary,args.run,args.case_root.resolve(),args.repeats,args.checkpoint)
