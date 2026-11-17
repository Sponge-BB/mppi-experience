#!/usr/bin/env python3
"""Reproduce one elevated sudden-box comparison on an owned localhost ROS master."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import shutil
import subprocess
import time
import xml.etree.ElementTree as ET
import xmlrpc.client

import yaml

HERE = Path(__file__).resolve().parent
CASE = HERE/'results/arm_box_sim_20261114-210347'
PORT = 11441


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''): h.update(block)
    return h.hexdigest()


def prepare(group, duration=45., seed=11):
    import ast
    import build_active_arm as builder
    import arm_box_scene as scene
    from active_arm_model import PiperArmModel
    builder.check()
    frozen = HERE/'results/active_arm_20261104_01/v2_B_active_seed11'
    out=CASE/group
    if out.exists():raise FileExistsError(str(out))
    out.mkdir(parents=True)
    for name in ['effective_real_params.json','limits.json','geometry.csv','geometry.json','parameters.yaml']:
        shutil.copy2(frozen/name,out/name)
    runtime=out/'runtime';runtime.mkdir()
    # Freeze only the actual local Python dependency closure, not all 492
    # unrelated experiment scripts or a newly parsed real-robot launch.
    pending=['active_arm_node','arm_box_simulator','arm_box_scene'];seen=set()
    while pending:
        name=pending.pop()
        if name in seen:continue
        source=(HERE if name.startswith('arm_box_') else frozen/'runtime')/(name+'.py')
        if not source.is_file():continue
        seen.add(name);text=source.read_text();shutil.copy2(source,runtime/source.name)
        for node in ast.walk(ast.parse(text)):
            modules=[alias.name for alias in node.names] if isinstance(node,ast.Import) else ([node.module] if isinstance(node,ast.ImportFrom) and node.module else [])
            for module in modules:
                local=module.split('.')[0]
                if (frozen/'runtime'/(local+'.py')).is_file() or (local.startswith('arm_box_') and (HERE/(local+'.py')).is_file()):pending.append(local)
    package=out/'ros_pkgs/online_remani_experiment';package.mkdir(parents=True)
    shutil.copy2(frozen/'ros_pkgs/online_remani_experiment/package.xml',package/'package.xml')
    for name in ['remani_planner_node','mm_controller_node','dynamic_obstacle_tracker_node']:
        (package/name).symlink_to(builder.OUT/'build'/name)
    for name in ['active_arm_node.py','arm_box_simulator.py']:
        (runtime/name).chmod(0o755);(package/name).symlink_to(runtime/name)
    params_path=out/'parameters.yaml';params=yaml.safe_load(params_path.read_text())
    params['/remani_planner_node/fsm/local_execution_owner']=group!='native'
    params_path.write_text(yaml.safe_dump(params,sort_keys=True))
    model_path=out/'model/mm.urdf';model_path.parent.mkdir()
    config=json.loads((frozen/'active_arm_config.json').read_text())
    shutil.copy2(config['urdf_path'],model_path)
    config.update(group=group,seed=seed,duration=duration,coupled=group=='mppi',parameters_path=str(params_path),urdf_path=str(model_path),
        description={'native':'REMANI native controller','mppi':'REMANI + whole-body MPPI','fast_only':'Initial REMANI reference + MPPI; subsequent slow planning frozen'}[group],
        scene_definition=scene.scene_definition(),goal=scene.scene_definition()['goal'],physical_base_locked=False,
        comparison='Same original full-DOF limits, initial pose and plant; native versus MPPI',slow_planner_frozen_after_initial_reference=group=='fast_only')
    (out/'active_arm_config.json').write_text(json.dumps(config,ensure_ascii=False,indent=2)+'\n')
    (out/'scenario.json').write_text(json.dumps(scene.scene_definition(),ensure_ascii=False,indent=2)+'\n')
    check=scene.straight_path_precheck(PiperArmModel.from_parameters(params_path,model_path))
    if not check['passed']:raise RuntimeError('Scene is not an arm-only geometric test')
    (out/'geometry_precheck.json').write_text(json.dumps(check,indent=2)+'\n')
    tree=ET.ElementTree(ET.fromstring((frozen/'experiment.launch').read_text().replace(str(frozen),str(out))));launch=tree.getroot()
    nodes={n.get('name'):n for n in launch.findall('node')}
    sim=nodes['algorithm_simulator'];sim.set('type','arm_box_simulator.py')
    for item in sim.findall('param'):
        if item.get('name')=='local_execution_owner':item.set('value',str(group!='native').lower())
        if item.get('name')=='scene':item.set('value',scene.scene_definition()['scene'])
        if item.get('name')=='duration':item.set('value',str(duration))
    for name,value in {'ACTIVE_ARM_ENABLED':1,'ACTIVE_ARM_COUPLING':int(group=='mppi'),'COLLAB_PREFIX':int(group=='mppi'),'COUPLING_HAND_WEIGHT':int(group=='mppi')}.items():
        old=nodes['remani_planner_node'].find("env[@name='%s']"%name)
        if old is None:old=ET.SubElement(nodes['remani_planner_node'],'env',name=name)
        old.set('value',str(value))
    if group=='native':
        launch.remove(nodes['active_arm_mppi'])
        for item in launch.findall('param'):
            if item.get('name')=='/online_validation/mode':item.set('value','remani')
    if group=='fast_only':
        from build_arm_box_fast_only import OUT,check as check_build
        check_build();link=package/'remani_planner_node';link.unlink();link.symlink_to(OUT/'build/remani_planner_node')
    tree.write(out/'experiment.launch',encoding='unicode')
    manifest=dict(group=group,created=datetime.now().isoformat(),new_box_experiment=True,
        frozen_simulation_configuration={str(frozen/n):sha(frozen/n) for n in ['effective_real_params.json','limits.json','geometry.csv','geometry.json','parameters.yaml','experiment.launch']},
        nodes=[dict(n.attrib) for n in launch.findall('node')],slow_planner_frozen_after_initial_reference=group=='fast_only',
        comparison_scope='Original completed active-arm numerical ROS framework; full-DOF native versus MPPI; not hardware/Gazebo',
        source_sha256={str(q.relative_to(out)):sha(q) for q in runtime.glob('*.py')},
        runtime_sha256={str(q.relative_to(out)):sha(q) for q in [params_path,out/'experiment.launch',out/'active_arm_config.json',out/'scenario.json',model_path]},
        actual_planner_sha256=sha(package/'remani_planner_node'),native_binaries_sha256={n:sha(package/n) for n in ['remani_planner_node','mm_controller_node','dynamic_obstacle_tracker_node']},
        baseline_not_fixed_arm=True,production_modified=False)
    (out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    print('Prepared',group,'with',len(seen),'frozen Python modules',flush=True)
    return out


def resources(root_pid):
    entries=[]
    for path in Path('/proc').glob('[0-9]*/stat'):
        try:
            fields=path.read_text().rpartition(')')[2].split()
            entries.append((int(path.parent.name),int(fields[1]),max(0,int(fields[21]))))
        except (OSError,ValueError,IndexError):pass
    owned={root_pid}
    while True:
        children={pid for pid,parent,_ in entries if parent in owned}
        if children<=owned:break
        owned.update(children)
    rss=sum(pages for pid,_,pages in entries if pid in owned)*os.sysconf('SC_PAGE_SIZE')
    available=next(int(row.split()[1])*1024 for row in Path('/proc/meminfo').read_text().splitlines() if row.startswith('MemAvailable:'))
    return rss/1024**2,available/1024**2


def run(group, duration=45., seed=11):
    with socket.socket() as s:
        if s.connect_ex(('127.0.0.1',PORT))==0:raise RuntimeError('Experiment port already in use')
    out = CASE/group
    if out.exists() and (out/'manifest.json').is_file():
        if (out/'result.json').exists():raise RuntimeError('Refusing to overwrite an existing trial result')
        manifest=json.loads((out/'manifest.json').read_text())
        for field in ['source_sha256','runtime_sha256']:
            for relative, expected in manifest[field].items():
                if sha(out/relative)!=expected:raise RuntimeError('Prepared input changed: '+relative)
        if sha(out/'ros_pkgs/online_remani_experiment/remani_planner_node')!=manifest['actual_planner_sha256']:
            raise RuntimeError('Prepared planner changed')
        for name in ['launch_stdout.log','run_status.json']:
            old=out/name
            if old.exists():
                saved=out/('unstarted_'+datetime.now().strftime('%Y%m%d-%H%M%S')+'_'+name)
                old.rename(saved)
    else:
        out = prepare(group,duration,seed)
    env=os.environ.copy()
    for k in list(env):
        if k.startswith(('ACTIVE_ARM_','COLLAB_','COUPLING_','NATIVE_','DELIVERY_','FAILURE_','DUAL_')):env.pop(k)
    env.pop('ROS_HOSTNAME',None)
    env.update(ROS_MASTER_URI='http://127.0.0.1:%d'%PORT,ROS_IP='127.0.0.1',
        ROS_HOME=str(out/'ros_home'),ROS_LOG_DIR=str(out/'roslogs'),PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',
        OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MALLOC_ARENA_MAX='2')
    env['ROS_PACKAGE_PATH']=str(out/'ros_pkgs')+':'+env.get('ROS_PACKAGE_PATH','')
    start=time.monotonic();peak=0.;minimum=1e30;reason=None;audit_done=False
    with (out/'launch_stdout.log').open('w') as log:
        proc=subprocess.Popen(['nice','-n','10','roslaunch','-p',str(PORT),str(out/'experiment.launch')],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            while proc.poll() is None and not (out/'result.json').exists():
                rss,available=resources(proc.pid);peak=max(peak,rss);minimum=min(minimum,available)
                if available<1500:reason='available_RAM_below_1500MiB'
                if rss>1800:reason='own_simulation_RSS_above_1800MiB'
                if time.monotonic()-start>duration+65:reason='wall_time_limit'
                if reason:break
                if not audit_done and time.monotonic()-start>5:
                    try:
                        master=xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']);code,_,graph=master.getSystemState('/arm_box_runner')
                        if code!=1:raise RuntimeError('ROS graph unavailable')
                        actual={name for part in graph for topic,names in part for name in names}
                        expected={'/'+n.get('name').strip('/') for n in ET.parse(out/'experiment.launch').getroot().findall('node')}|{'/rosout'}
                        pubs=dict(graph[0]);whole=pubs.get('/sim/whole_body_command',[])
                        valid=not(actual-expected) and (whole==[] if group=='native' else whole==['/active_arm_mppi'])
                        (out/'graph_audit.json').write_text(json.dumps(dict(actual_nodes=sorted(actual),expected_nodes=sorted(expected),whole_body_publishers=whole,valid=valid,hardware_nodes=False),indent=2)+'\n')
                        if not valid:raise RuntimeError('Unexpected authority in isolated graph')
                        audit_done=True
                    except (ConnectionRefusedError,OSError):pass
                time.sleep(.3)
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGTERM)
                    try:proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
    failed='REQUIRED process [' in (out/'launch_stdout.log').read_text(errors='replace')
    status=dict(group=group,result_written=(out/'result.json').exists(),graph_audited=audit_done,
        stop_reason=reason,required_process_failure=failed,wall_s=time.monotonic()-start,
        peak_group_rss_mib=peak,lowest_available_mib=minimum,return_code=proc.returncode,owned_process_group_stopped=True)
    (out/'run_status.json').write_text(json.dumps(status,indent=2)+'\n');print(json.dumps(status),flush=True)
    if reason or failed or not status['result_written'] or not audit_done:raise RuntimeError('Invalid simulation retained: '+str(out))
    result=json.loads((out/'result.json').read_text())
    print(json.dumps({k:v for k,v in result.items() if not isinstance(v,(list,dict))}),flush=True)
    return out


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('group',choices=['native','mppi','fast_only']);p.add_argument('--duration',type=float,default=45.);p.add_argument('--seed',type=int,default=11);p.add_argument('--prepare-only',action='store_true');p.add_argument('--case',type=Path,default=CASE);a=p.parse_args();CASE=a.case.resolve()
    (prepare if a.prepare_only else run)(a.group,a.duration,a.seed)
