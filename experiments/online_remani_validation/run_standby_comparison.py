#!/usr/bin/env python3
"""Run exact native, unchanged current and explicitly extended standby ROS cases."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import socket
import subprocess
import time
import xml.etree.ElementTree as ET
import xmlrpc.client

import numpy as np
import yaml
import run_active_arm as scaffold
import build_native_piper_idle as native_builder
from active_arm_model import PiperArmModel
from standby_arm_scene import scene_definition, geometry_precheck

HERE=Path(__file__).resolve().parent
ROOT=HERE/'results/standby_arm_influence_20261105_01'
NATIVE=HERE/'builds/native_piper_idle_20261105_01'
PORT=11441
MODES=('native_piper','current_unmodified','current_standby_extension')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stage(out,name):
    (out/'preparation_status.json').write_text(json.dumps(dict(stage=name,time=time.time()),indent=2)+'\n')
    print('PREPARATION',out.name,name,flush=True)


def cached_geometry(model,scene):
    signature=dict(fixed=model.fixed_spheres.tolist(),mount=model.mount.tolist(),
        lower=model.lower.tolist(),upper=model.upper.tolist(),velocity=model.velocity_limits.tolist(),
        acceleration=model.acceleration_limits.tolist(),arm_radius=model.arm_radius,
        ground_clearance=model.ground_clearance,scene=scene)
    digest=hashlib.sha256(json.dumps(signature,sort_keys=True).encode()).hexdigest()
    path=ROOT/'common_geometry_precheck.json'
    if not path.exists():raise RuntimeError('run and save the isolated geometry precheck first')
    saved=json.loads(path.read_text())
    if saved['model_scene_sha256']!=digest or not saved['result']['passed']:
        raise RuntimeError('cached geometry does not match this model and scene')
    return saved['result']


def remap(node,source,target):
    for item in list(node.findall('remap')):
        if item.get('from')==source:node.remove(item)
    ET.SubElement(node,'remap',{'from':source,'to':target})


def clone_prepared(source_tag,tag,mode,seed=11):
    """Copy validated configuration only, never a previous run's observations."""
    source=ROOT/source_tag;out=ROOT/tag
    if out.exists():raise FileExistsError(out)
    if not (source/'prepared.json').exists():raise RuntimeError('source configuration is not ready')
    out.mkdir()
    names=('active_arm_config.json','config.json','parameters.yaml','scenario.json','limits.json',
           'geometry_precheck.json','geometry.csv','geometry.json','shared_collision_config.json',
           'effective_real_params.json','experiment.launch')
    for name in names:
        p=source/name
        if p.exists():(out/name).write_text(p.read_text().replace(str(source),str(out)))
    shutil.copytree(source/'runtime',out/'runtime',ignore=shutil.ignore_patterns('__pycache__'))
    shutil.copytree(source/'ros_pkgs',out/'ros_pkgs',symlinks=True)
    for link in (out/'ros_pkgs').rglob('*'):
        if link.is_symlink():
            target=os.readlink(link);link.unlink();link.symlink_to(target.replace(str(source),str(out)))
    config=json.loads((out/'config.json').read_text())
    config.update(mode=mode,group=mode,label=mode,seed=seed,
        controller_status='new explicit standby extension' if mode.endswith('extension') else 'unaltered existing algorithm')
    (out/'config.json').write_text(json.dumps(config,ensure_ascii=False,indent=2)+'\n')
    original=json.loads((out/'active_arm_config.json').read_text());original['seed']=seed
    (out/'active_arm_config.json').write_text(json.dumps(original,indent=2)+'\n')
    tree=ET.parse(out/'experiment.launch')
    node=next(n for n in tree.getroot().findall('node') if n.get('name') in ('active_arm_mppi','standby_whole_body_mppi'))
    node.set('name','standby_whole_body_mppi' if mode.endswith('extension') else 'active_arm_mppi')
    node.set('type','standby_mppi_node.py' if mode.endswith('extension') else 'active_arm_node.py')
    tree.write(out/'experiment.launch',encoding='unicode')
    manifest=json.loads((source/'manifest.json').read_text())
    manifest.update(mode=mode,seed=seed,configuration_only_clone_of=source_tag,
        nodes=[dict(n.attrib) for n in tree.getroot().findall('node')],
        runtime_sha256={p.name:sha(p) for p in (out/'runtime').glob('*.py')},
        parameters_sha256=sha(out/'parameters.yaml'),config_sha256=sha(out/'config.json'))
    (out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    (out/'prepared.json').write_text(json.dumps(dict(mode=mode,manifest_sha256=sha(out/'manifest.json'),
        launch_sha256=sha(out/'experiment.launch'),config_sha256=sha(out/'config.json')),indent=2)+'\n')
    for name in ('result.json','state.csv','cycles.jsonl','launch_stdout.log','run_status.json'):
        if (out/name).exists():raise AssertionError('run data copied into a new trial')
    stage(out,'ready_to_launch')
    return out


def prepare(tag,mode,seed=11,duration=24.):
    if mode not in MODES:raise ValueError(mode)
    ROOT.mkdir(exist_ok=True)
    out=ROOT/tag
    if (out/'prepared.json').exists():
        print('PREPARATION',tag,'already_ready',flush=True)
        return out
    scaffold.ROOT=ROOT
    if out.exists():
        if not all((out/n).exists() for n in ('active_arm_config.json','parameters.yaml','experiment.launch','runtime')):
            raise RuntimeError('incomplete scaffold requires inspection: '+str(out))
        print('PREPARATION',tag,'resume_existing_scaffold',flush=True)
    else:
        os.environ['ROS_HOME']=str(ROOT/'preparation_ros_home')
        os.environ['ROS_LOG_DIR']=str(ROOT/'preparation_roslogs')
        out=scaffold.prepare_trial(tag,True,seed,duration,True,False)
    stage(out,'scaffold_ready')
    params=yaml.safe_load((out/'parameters.yaml').read_text())
    scene=scene_definition();scene['duration_s']=duration
    model=PiperArmModel.from_parameters(out/'parameters.yaml')
    for prefix in ('/remani_planner_node/','/mm_controller_node/'):
        params[prefix+'fsm/target_type']=1
        params[prefix+'fsm/init_state']=[0.,0.,*np.rad2deg(scene['q_start']).tolist()]
        params[prefix+'fsm/init_gripper_close']=False
        params[prefix+'fsm/capture_startup_standby']=False
        params[prefix+'mm/manipulator_min_pos']=model.lower.tolist()
        params[prefix+'mm/manipulator_max_pos']=model.upper.tolist()
    params['/remani_planner_node/grid_map/pose_type']=2
    params['/remani_planner_node/fsm/local_execution_owner']=mode!='native_piper'
    (out/'parameters.yaml').write_text(yaml.safe_dump(params,sort_keys=True))
    cfg=json.loads((out/'active_arm_config.json').read_text())
    cfg.update(mode=mode,label=mode,scene_definition=scene,scene=scene,
        q_start=scene['q_start'],postures=[scene['posture_prior']],goal=None,
        navigation_goal=None,standby_anchor=[0.,0.,0.],warmup_s=7.,duration=duration,
        rate_hz=5.,deadline_s=.4,random_samples=6,horizon=20,
        lidar_offset=[params['/remani_planner_node/grid_map/lidar_offset_'+a] for a in 'xyz'],
        physical_base_constraint='free differential-drive dynamics, no zero override',
        scope='live localhost ROS, shared free whole-body numerical plant; separate standby extension',
        controller_status='new explicit standby extension' if mode.endswith('extension') else 'unaltered existing algorithm',
        parameter_source='shared exported current Piper configuration with only declared common initial-state/idle wiring overrides')
    (out/'config.json').write_text(json.dumps(cfg,ensure_ascii=False,indent=2)+'\n')
    # Keep the unchanged current entry reading its original config contract.
    if mode=='current_unmodified':
        original=json.loads((out/'active_arm_config.json').read_text())
        original.update(scene_definition=scene,goal=None)
        (out/'active_arm_config.json').write_text(json.dumps(original,indent=2)+'\n')
    (out/'scenario.json').write_text(json.dumps(scene,ensure_ascii=False,indent=2)+'\n')
    stage(out,'geometry_precheck_started')
    (out/'geometry_precheck.json').write_text(json.dumps(cached_geometry(model,scene),ensure_ascii=False,indent=2)+'\n')
    stage(out,'geometry_precheck_complete')
    tree=ET.parse(out/'experiment.launch');launch=tree.getroot()
    nodes={n.get('name'):n for n in launch.findall('node')}
    # Direct measured joints replace only redundant synthetic sensor plumbing.
    for name in ('filter_joint_states',):
        if name in nodes:launch.remove(nodes[name])
    sim=nodes['algorithm_simulator'];sim.set('type','standby_comparison_simulator.py')
    sim.set('name','standby_comparison_plant')
    for item in list(sim.findall('param')):
        if item.get('name')!='out':sim.remove(item)
    executor=nodes['active_arm_mppi']
    if mode=='native_piper':
        launch.remove(executor)
        for name in ('dynamic_obstacle_tracker','pcl_manager_filter','passthrough_z','crop_box_filter'):
            if name in nodes:launch.remove(nodes[name])
        stage(out,'native_build_check_started')
        native_builder.check()
        stage(out,'native_build_check_complete')
        for name in ('remani_planner_node','mm_controller_node'):
            link=out/'ros_pkgs/online_remani_experiment'/name
            link.unlink();link.symlink_to(NATIVE/'build'/name)
        planner=nodes['remani_planner_node']
        for item in list(planner.findall('env')):planner.remove(item)
        contract=json.loads((NATIVE/'runtime_contract.json').read_text())
        for key,value in contract['planner_remaps'].items():remap(planner,key,value)
        for key,value in contract['controller_remaps'].items():remap(nodes['mm_controller_node'],key,value)
    elif mode=='current_standby_extension':
        executor.set('name','standby_whole_body_mppi');executor.set('type','standby_mppi_node.py')
    runtime=out/'runtime';pkg=out/'ros_pkgs/online_remani_experiment'
    for src in HERE.glob('*.py'):
        shutil.copyfile(src,runtime/src.name)
    for name in ('standby_comparison_simulator.py','standby_mppi_node.py'):
        (runtime/name).chmod(0o755)
        link=pkg/name
        if not link.exists():link.symlink_to(runtime/name)
    tree.write(out/'experiment.launch',encoding='unicode')
    node_list=[dict(n.attrib) for n in launch.findall('node')]
    if any('piper' in n['pkg'] or n['type'] in ('base_control.py','joint_cmd_bridge.py') for n in node_list):
        raise RuntimeError('unexpected hardware node')
    manifest=dict(mode=mode,seed=seed,nodes=node_list,physical_base_locked=False,
        native_baseline_commit='f138ec82dce5c58be2e7028548ed317efe541c4f' if mode=='native_piper' else None,
        binary_sha256={name:sha(pkg/name) for name in ('remani_planner_node','mm_controller_node')},
        runtime_sha256={p.name:sha(p) for p in runtime.glob('*.py')},
        parameters_sha256=sha(out/'parameters.yaml'),config_sha256=sha(out/'config.json'),
        no_navigation_goal=True,production_modified=False,
        comparison_note='native and existing current remain unaltered in standby; standby extension is separately labelled')
    (out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    (out/'prepared.json').write_text(json.dumps(dict(mode=mode,manifest_sha256=sha(out/'manifest.json'),
        launch_sha256=sha(out/'experiment.launch'),config_sha256=sha(out/'config.json')),indent=2)+'\n')
    stage(out,'ready_to_launch')
    return out


def run(tag,mode,seed=11,duration=24.,prepare_only=False,run_prepared=False):
    out=ROOT/tag if run_prepared else prepare(tag,mode,seed,duration)
    if prepare_only:return out
    if not (out/'prepared.json').exists():raise RuntimeError('prepare this case first')
    if (out/'result.json').exists() or (out/'launch_stdout.log').exists():
        raise RuntimeError('will not overwrite an actual run: '+str(out))
    saved=json.loads((out/'prepared.json').read_text())
    if saved['mode']!=mode or saved['launch_sha256']!=sha(out/'experiment.launch'):
        raise RuntimeError('prepared mode or launch changed')
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1',PORT))==0:raise RuntimeError('owned experiment port occupied')
    env={k:v for k,v in os.environ.items() if not re.search('TOKEN|SECRET|PASSWORD|CREDENTIAL|AUTH|API_KEY',k,re.I)}
    for key in list(env):
        if key.startswith(('ACTIVE_ARM_','COLLAB_','COUPLING_','NATIVE_','DELIVERY_','FAILURE_','DUAL_')):del env[key]
    env.update(ROS_MASTER_URI='http://127.0.0.1:%d'%PORT,ROS_IP='127.0.0.1',ROS_HOSTNAME='localhost',
        ROS_HOME=str(out/'ros_home'),ROS_LOG_DIR=str(out/'roslogs'),PYTHONNOUSERSITE='1',
        OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    env['ROS_PACKAGE_PATH']=str(out/'ros_pkgs')+':'+env.get('ROS_PACKAGE_PATH','')
    started=time.monotonic();audit_done=False;run_error=None;readiness_errors=[]
    with (out/'launch_stdout.log').open('w') as stream:
        proc=subprocess.Popen(['roslaunch','-p',str(PORT),str(out/'experiment.launch')],env=env,
            stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            while proc.poll() is None and not (out/'result.json').exists() and time.monotonic()-started<duration+90:
                if not audit_done and time.monotonic()-started>8:
                    # roslaunch may still be parsing parameters or starting
                    # its master at eight seconds on a busy workstation.
                    # A refused connection is readiness, not experiment failure.
                    try:
                        master=xmlrpc.client.ServerProxy('http://127.0.0.1:%d'%PORT)
                        code,_,graph=master.getSystemState('/standby_comparison_audit')
                    except (OSError,xmlrpc.client.Error) as exc:
                        if not readiness_errors or readiness_errors[-1]!=type(exc).__name__:
                            readiness_errors.append(type(exc).__name__)
                        if time.monotonic()-started>60:
                            raise RuntimeError('ROS master did not become ready within 60 seconds') from exc
                        time.sleep(.3)
                        continue
                    if code!=1:raise RuntimeError('ROS master state unavailable')
                    pubs={topic:names for topic,names in graph[0]}
                    audit=dict(use_sim_time=master.getParam('/standby_comparison_audit','/use_sim_time')[2],
                        navigation_goal_publishers=pubs.get('/move_base_simple/goal',[]),
                        whole_body_command_publishers=pubs.get('/sim/whole_body_command',[]),
                        all_publishers=pubs,master='127.0.0.1:%d'%PORT)
                    expected=[] if mode=='native_piper' else ['/active_arm_mppi' if mode=='current_unmodified' else '/standby_whole_body_mppi']
                    if audit['navigation_goal_publishers']:
                        raise RuntimeError('unexpected navigation goal publisher')
                    if audit['whole_body_command_publishers']!=expected:
                        if time.monotonic()-started>60:
                            raise RuntimeError('expected actuator owner did not become ready')
                        time.sleep(.3)
                        continue
                    (out/'graph_audit.json').write_text(json.dumps(audit,indent=2)+'\n')
                    audit_done=True
                time.sleep(.3)
        except Exception as exc:
            run_error=repr(exc)
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGTERM)
                    try:proc.wait(timeout=8)
                    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
    log=(out/'launch_stdout.log').read_text(errors='replace')
    failed=bool(re.search(r'REQUIRED process \[[^\]]+\] has died',log))
    status=dict(result_written=(out/'result.json').exists(),required_process_failed=failed,
        roslaunch_returncode=proc.returncode,wall_s=time.monotonic()-started,graph_audit_written=audit_done,
        run_error=run_error,readiness_errors=readiness_errors)
    (out/'run_status.json').write_text(json.dumps(status,indent=2)+'\n')
    if mode=='current_unmodified' and (out/'active_arm_cycles.jsonl').exists():
        shutil.copyfile(out/'active_arm_cycles.jsonl',out/'cycles.jsonl')
    if mode=='native_piper':shutil.copyfile(out/'native_commands.jsonl',out/'cycles.jsonl')
    print(json.dumps(dict(trial=tag,**status)),flush=True)
    if not status['result_written'] or failed or run_error or not audit_done:
        raise RuntimeError('invalid ROS case retained: '+str(out))
    print((out/'result.json').read_text(),flush=True)
    return out


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--tag',required=True);p.add_argument('--mode',choices=MODES,required=True)
    p.add_argument('--seed',type=int,default=11);p.add_argument('--duration',type=float,default=24.)
    p.add_argument('--prepare-only',action='store_true')
    p.add_argument('--run-prepared',action='store_true')
    a=p.parse_args();run(a.tag,a.mode,a.seed,a.duration,a.prepare_only,a.run_prepared)
