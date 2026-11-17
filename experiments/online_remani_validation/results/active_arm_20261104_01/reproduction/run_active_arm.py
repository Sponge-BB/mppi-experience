#!/usr/bin/env python3
"""Prepare/run one owned localhost ROS active-arm numerical experiment."""
import argparse
from datetime import datetime, timezone
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
import xmlrpc.client

import numpy as np
import yaml

import build_active_arm as builder
from active_arm_model import PiperArmModel, DEFAULT_URDF
from active_arm_scene import register_scene, scene_definition, straight_path_precheck, Q_START, Q_RETRACT
from prepare import prepare, PORT

HERE=Path(__file__).resolve().parent
ROOT=HERE/'results/active_arm_20261104_01'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def set_env(node,name,value):
    old=node.findall("env[@name='%s']"%name)
    if len(old)>1:raise ValueError('ambiguous env '+name)
    if old:old[0].set('value',str(value))
    else:ET.SubElement(node,'env',name=name,value=str(value))


def prepare_trial(tag,enabled,seed,duration,coupled=True,profile=False):
    builder.check();register_scene()
    if not tag.replace('_','').isalnum():raise ValueError('invalid tag')
    ROOT.mkdir(exist_ok=True)
    out=ROOT/tag
    if out.exists():raise FileExistsError(str(out))
    if __import__('shutil').disk_usage(ROOT).free<512*1024**2:raise RuntimeError('insufficient disk reserve')
    prepare(out,builder.OUT,'mppi',scene_definition()['scene'],duration,False,'current','motion',
            perception='classified',execution='local',prediction_horizon=3.)
    params_path=out/'parameters.yaml';params=yaml.safe_load(params_path.read_text())
    model=PiperArmModel.from_defaults()
    for prefix in ('/remani_planner_node/','/mm_controller_node/'):
        params[prefix+'fsm/init_state']=[0.,0.,*np.rad2deg(Q_START).tolist()]
        params[prefix+'mm/manipulator_min_pos']=(model.lower if enabled else Q_START).tolist()
        params[prefix+'mm/manipulator_max_pos']=(model.upper if enabled else Q_START).tolist()
        params[prefix+'mm/manipulator_max_vel']=float(np.min(model.velocity_limits))
        params[prefix+'mm/manipulator_max_acc']=float(np.min(model.acceleration_limits))
        params[prefix+'optimization/self_safe_margin']=0.
        params[prefix+'optimization/weight_manipulator_hold']=.3
        # A limit/servo model shared by both groups. Bounds in the slow A
        # planner remain fixed; the physical plant always uses true limits.
    params_path.write_text(yaml.safe_dump(params,sort_keys=True))
    config=dict(schema=1,created=datetime.now(timezone.utc).isoformat(),group='B_active' if enabled else 'A_fixed',
        enabled=bool(enabled),coupled=bool(coupled and enabled),seed=seed,duration=duration,
        profile=bool(profile),profile_timings_are_not_benchmark=bool(profile),
        q_start=Q_START.tolist(),postures=[Q_RETRACT.tolist()],goal=scene_definition()['goal'],
        arm_limits={k:np.asarray(v).tolist() for k,v in zip(('lower','upper','velocity','acceleration'),
            (model.lower,model.upper,model.velocity_limits,model.acceleration_limits))},
        urdf_path=str(DEFAULT_URDF),parameters_path=str(params_path),scene_definition=scene_definition(),
        horizon=20,random_samples=6,rate_hz=10.,deadline_s=.20,
        geometry='native DH 47 fixed +20 moving spheres; URDF limits; not mesh equivalence',
        actuator='atomic base and six joint velocity command; 250ms lease; common stop',
        predictor='bounded lagged base and acceleration-limited joints; 50ms collision samples',
        slow_joint_acceleration_boundary='explicit zero approximation; effort never used as acceleration')
    (out/'active_arm_config.json').write_text(json.dumps(config,indent=2)+'\n')
    (out/'scenario.json').write_text(json.dumps(scene_definition(),ensure_ascii=False,indent=2)+'\n')
    (out/'geometry_precheck.json').write_text(json.dumps(straight_path_precheck(model),indent=2)+'\n')
    spheres=model.body_spheres(Q_START)
    np.savetxt(out/'geometry.csv',spheres[:,[0,1,3]],delimiter=',')
    geometry=json.loads((out/'geometry.json').read_text())
    geometry.update(arm_q_deg=np.rad2deg(Q_START).tolist(),active_arm=True,
        source='native model; geometry.csv is initial planar projection only, not active collision authorization')
    (out/'geometry.json').write_text(json.dumps(geometry,indent=2)+'\n')
    tree=ET.parse(out/'experiment.launch');launch=tree.getroot()
    nodes={n.get('name'):n for n in launch.findall('node')}
    # Native 2-D shared collision publisher is not an authority for this new
    # 3-D executor and must not reinterpret the elevated cylinder as full-height.
    if 'shared_collision_model' in nodes:launch.remove(nodes['shared_collision_model'])
    native=nodes['remani_planner_node']
    old=ET.parse(HERE/'results/short_prefix_rejoin_20261101_01/midcross_B/experiment.launch').getroot()
    old_native=next(n for n in old.findall('node') if n.get('name')=='remani_planner_node')
    for item in old_native.findall('env'):
        name,value=item.get('name'),item.get('value')
        if '/results/' in value:value=str(out/Path(value).name)
        set_env(native,name,value)
    for name,value in dict(ACTIVE_ARM_ENABLED=int(enabled),ACTIVE_ARM_COUPLING=int(enabled and coupled),
            COLLAB_PREFIX=int(enabled and coupled),COUPLING_EXEC_WEIGHT=0,
            COUPLING_HAND_WEIGHT=int(enabled and coupled),COUPLING_DELIVERY_DELAY=1.1,
            COUPLING_ENGINEERING=0,SHORT_PREFIX_REJOIN=0,CANDIDATE_AUDIT_FREEZE_SOLVER=0,
            STATE_CONTRACT_CAPTURE=0,STATE_CONTRACT_SIGNED_LOW_SPEED=0,TEMPORAL_COLLISION_CAPTURE=0,
            DUAL_TIMELINE_PATH=str(out/'dual_timeline.jsonl'),FAILURE_TRACE_PATH=str(out/'failure_trace.jsonl')).items():
        set_env(native,name,value)
    pkg=out/'ros_pkgs/online_remani_experiment'
    for name in ('active_arm_node.py','active_arm_simulator.py'):
        (pkg/name).symlink_to(out/'runtime'/name)
        (out/'runtime'/name).chmod(0o755)
    nodes['base_mppi'].set('type','active_arm_node.py');nodes['base_mppi'].set('name','active_arm_mppi')
    nodes['algorithm_simulator'].set('type','active_arm_simulator.py')
    tree.write(out/'experiment.launch',encoding='unicode')
    m=json.loads((out/'manifest.json').read_text())
    m.update(active_arm_experiment=config,scope='native REMANI + new full-state sampling executor, numerical ROS only',
        scenario=scene_definition(),build_manifest_sha256=sha(builder.OUT/'build_manifest.json'),
        protocol='whole-body seed v2 and atomic whole-body command v1',
        source_sha256={p.name:sha(p) for p in (out/'runtime').glob('active_arm*') if p.is_file()},
        configuration_sha256={p.name:sha(p) for p in (params_path,out/'active_arm_config.json',out/'scenario.json')},
        production_modified=False)
    (out/'manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2)+'\n')
    return out


def run_trial(tag,enabled,seed=11,duration=50.,coupled=True,profile=False):
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1',PORT))==0:raise RuntimeError('experiment ROS port occupied')
    out=prepare_trial(tag,enabled,seed,duration,coupled,profile)
    env={k:v for k,v in os.environ.items() if not re.search('TOKEN|SECRET|PASSWORD|CREDENTIAL|AUTH|API_KEY',k,re.I)}
    for key in list(env):
        if key.startswith(('ACTIVE_ARM_','COLLAB_','COUPLING_','NATIVE_','DELIVERY_','FAILURE_','DUAL_')):del env[key]
    env.update(ROS_MASTER_URI='http://127.0.0.1:%d'%PORT,ROS_IP='127.0.0.1',ROS_HOSTNAME='localhost',
        ROS_HOME=str(out/'ros_home'),ROS_LOG_DIR=str(out/'roslogs'),PYTHONNOUSERSITE='1',
        OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    env['ROS_PACKAGE_PATH']=str(out/'ros_pkgs')+':'+env.get('ROS_PACKAGE_PATH','')
    started=time.monotonic();stopped=False;graph_done=False
    with (out/'launch_stdout.log').open('w') as log:
        proc=subprocess.Popen(['roslaunch','-p',str(PORT),str(out/'experiment.launch')],env=env,
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            while proc.poll() is None and not (out/'result.json').exists() and time.monotonic()-started<duration+50:
                if not graph_done and time.monotonic()-started>6:
                    master=xmlrpc.client.ServerProxy('http://127.0.0.1:%d'%PORT)
                    code,_,graph=master.getSystemState('/active_arm_runner_audit')
                    if code!=1:raise RuntimeError('cannot audit owned ROS graph')
                    pubs={name:nodes for name,nodes in graph[0]}
                    actuator=pubs.get('/sim/whole_body_command',[])
                    expected={'/'+n.get('name').strip('/') for n in ET.parse(out/'experiment.launch').getroot().findall('node')}|{'/rosout'}
                    actual={name for group in graph for topic,names in group for name in names}
                    audit=dict(master='127.0.0.1:%d'%PORT,actual_nodes=sorted(actual),expected_nodes=sorted(expected),
                        sole_whole_body_publisher=actuator,unknown_nodes=sorted(actual-expected),
                        use_sim_time=master.getParam('/active_arm_runner_audit','/use_sim_time')[2])
                    (out/'graph_audit.json').write_text(json.dumps(audit,indent=2)+'\n')
                    if actuator!=['/active_arm_mppi'] or actual-expected or not audit['use_sim_time']:
                        raise RuntimeError('unexpected ROS execution authority or node')
                    graph_done=True
                time.sleep(.3)
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGTERM)
                    try:proc.wait(timeout=8)
                    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                stopped=True
    text=(out/'launch_stdout.log').read_text(errors='replace')
    failed=bool(re.search(r'REQUIRED process \[[^\]]+\] has died',text))
    status=dict(result_written=(out/'result.json').exists(),required_process_failed=failed,
        roslaunch_returncode=proc.returncode,owned_process_group_stopped=stopped,
        wall_s=time.monotonic()-started,all_failures_retained=True)
    (out/'run_status.json').write_text(json.dumps(status,indent=2)+'\n')
    print(json.dumps(dict(trial=str(out),**status)),flush=True)
    if not status['result_written'] or failed:raise RuntimeError('Invalid ROS run retained: '+str(out))
    print((out/'result.json').read_text(),flush=True)
    return out


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--tag',required=True);p.add_argument('--enabled',type=int,choices=(0,1),required=True)
    p.add_argument('--seed',type=int,default=11);p.add_argument('--duration',type=float,default=50.)
    p.add_argument('--coupled',type=int,choices=(0,1),default=1);p.add_argument('--prepare-only',action='store_true')
    p.add_argument('--profile',action='store_true')
    a=p.parse_args()
    fn=prepare_trial if a.prepare_only else run_trial
    fn(a.tag,bool(a.enabled),a.seed,a.duration,bool(a.coupled),a.profile)
