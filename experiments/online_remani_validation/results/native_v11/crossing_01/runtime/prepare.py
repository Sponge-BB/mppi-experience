#!/usr/bin/env python3
"""Generate native ROS graphs, never include hardware drivers."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
import numpy as np
import roslaunch
import yaml

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
PORT=11441
NODES={'remani_planner_node','mm_controller_node','real_base_tracker','dynamic_obstacle_tracker',
       'pcl_manager_filter','passthrough_z','crop_box_filter','filter_joint_states'}


def prepare(out,build,mode,scene,duration=45,projection_progress=False,planner_variant='current',scenario_start='goal',debug_native=False,perception='auto',execution='auto',reference_delay=0.):
    out=Path(out).resolve();out.mkdir(parents=True,exist_ok=False);build=Path(build).resolve()
    for name in ('remani_planner_node','mm_controller_node','dynamic_obstacle_tracker_node'):
        if not (build/'build'/name).is_file():raise RuntimeError('missing isolated binary: '+name)
    spec=importlib.util.spec_from_file_location('offline_export',ROOT/'experiments/base_mppi_cbf_compare/run.py')
    exporter=importlib.util.module_from_spec(spec);spec.loader.exec_module(exporter)
    original,limits=exporter.export_configuration(out)
    params={k:v for k,v in original.items() if k.startswith(tuple('/'+n+'/' for n in NODES))}
    q=np.deg2rad(params['/remani_planner_node/fsm/init_state'][2:]).tolist()
    overrides={
        '/remani_planner_node/mm/manipulator_min_pos':q,
        '/remani_planner_node/mm/manipulator_max_pos':q,
        '/remani_planner_node/fsm/manual_target_end_arm_standby':False,
        '/remani_planner_node/fsm/manual_restore_standby_after_goal':False,
        '/remani_planner_node/fsm/manual_use_work_arm_waypoint':False,
    }
    build_manifest=json.loads((build/'build_manifest.json').read_text())
    if planner_variant not in build_manifest.get('planner_variants',['current','no_wait']):raise ValueError('unsupported planner variant for this isolated build')
    if execution=='local' and mode!='mppi':raise ValueError('local execution requires MPPI-only mode')
    classified=build_manifest.get('classified_cloud',False) if perception=='auto' else perception=='classified'
    local_execution=(build_manifest.get('local_execution_owner',False) if execution=='auto' else execution=='local') and mode=='mppi'
    if local_execution and (not classified or not build_manifest.get('local_execution_owner',False)):
        raise ValueError('local execution requires classified V10 build')
    if local_execution and planner_variant!='current':raise ValueError('V10 supports current native planner only')
    overrides['/remani_planner_node/fsm/local_execution_owner']=local_execution
    if local_execution:
        overrides['/remani_planner_node/fsm/goal_position_tolerance']=limits['goal_pos']
        overrides['/remani_planner_node/fsm/goal_yaw_tolerance']=limits['goal_yaw']
    if reference_delay<0 or reference_delay>10 or (reference_delay and not local_execution):raise ValueError('reference delay requires local execution, within 0..10s')
    if classified and not build_manifest.get('classified_cloud',False):raise ValueError('build has no atomic classified-cloud publisher')
    profile=build_manifest.get('experiment_profile','original')
    if profile=='no_uncertainty':
        if mode=='mppi_cbf':raise ValueError('no_uncertainty v3 comparison excludes CBF')
        for key in list(params):
            if '/dynamic_point_flow/' in key and key.rsplit('/',1)[-1] in ('position_uncertainty','velocity_uncertainty','acceleration_bound'):
                overrides[key]=0.0
        limits.update(pos_unc=0.0,vel_unc=0.0,acc_unc=0.0)
        (out/'limits.json').write_text(json.dumps(limits,indent=2)+'\n')
    params.update(overrides)
    pkg=out/'ros_pkgs/online_remani_experiment';pkg.mkdir(parents=True)
    (pkg/'package.xml').write_text('<package format="2"><name>online_remani_experiment</name><version>0.0.1</version><description>Isolated algorithm experiment</description><maintainer email="sim@example.invalid">sim</maintainer><license>BSD</license></package>')
    for name in ('remani_planner_node','mm_controller_node','dynamic_obstacle_tracker_node'):
        binary=name+'_no_wait' if name=='remani_planner_node' and planner_variant=='no_wait' else name
        (pkg/name).symlink_to(build/'build'/binary)
    runtime=out/'runtime';runtime.mkdir()
    for source in HERE.glob('*.py'):shutil.copy2(source,runtime/source.name)
    for name in ('simulator.py','mppi_node.py'):
        (pkg/name).symlink_to(runtime/name)
    (pkg/'progress.py').symlink_to(runtime/'progress.py')
    (out/'parameters.yaml').write_text(yaml.safe_dump(params,sort_keys=True))
    launch=ET.Element('launch')
    ET.SubElement(launch,'param',name='/use_sim_time',value='true',type='bool')
    ET.SubElement(launch,'param',name='/online_validation/enabled',value='true',type='bool')
    ET.SubElement(launch,'param',name='/online_validation/mode',value=mode)
    ET.SubElement(launch,'rosparam',file=str(out/'parameters.yaml'),command='load')
    cfg=roslaunch.config.load_config_default([str(ROOT/'src/mm_remani/launch/exp0_real.launch')],PORT,verbose=False)
    for node in cfg.nodes:
        if node.name not in NODES:continue
        if node.name=='real_base_tracker' and mode!='remani':continue
        node_type=node.type;node_package=node.package
        if node.name in ('remani_planner_node','mm_controller_node','dynamic_obstacle_tracker'):
            node_package='online_remani_experiment'
        n=ET.SubElement(launch,'node',pkg=node_package,type=node_type,name=node.name,ns=node.namespace,output='log',required='true')
        if debug_native and node.name=='remani_planner_node':
            n.set('launch-prefix','gdb -batch -ex "set pagination off" -ex "handle SIGINT nostop noprint pass" -ex run -ex "thread apply all bt" -ex quit --args')
        if node.args:n.set('args',node.args)
        for source,target in node.remap_args:
            if target=='/livox/lidar_pcl2':target='/sim/raw_cloud'
            ET.SubElement(n,'remap',{'from':source,'to':target})
        if node.name=='real_base_tracker':ET.SubElement(n,'remap',{'from':'/cmd_vel','to':'/sim/cmd_vel'})
        if node.name=='mm_controller_node' and mode!='remani' and projection_progress:
            ET.SubElement(n,'remap',{'from':'~trajectory_progress','to':'/sim/native_controller_progress'})
    for parent,child in [('world','map'),('map','odom')]:
        ET.SubElement(launch,'node',pkg='tf2_ros',type='static_transform_publisher',name='sim_'+parent+'_to_'+child,args=f'0 0 0 0 0 0 {parent} {child}')
    offset=[params['/remani_planner_node/grid_map/lidar_offset_'+axis] for axis in 'xyz']
    ET.SubElement(launch,'node',pkg='tf2_ros',type='static_transform_publisher',name='sim_lidar_tf',args=' '.join(map(str,offset))+' 0 0 0 base_link livox_frame')
    sim=ET.SubElement(launch,'node',pkg='online_remani_experiment',type='simulator.py',name='algorithm_simulator',output='log',required='true')
    for key,value in {'out':str(out),'scene':scene,'duration':str(duration),'auto_goal':'true','scenario_start':scenario_start,'local_execution_owner':str(local_execution).lower()}.items():
        ET.SubElement(sim,'param',name=key,value=value)
    if mode!='remani':
        node=ET.SubElement(launch,'node',pkg='online_remani_experiment',type='mppi_node.py',name='base_mppi',output='log',required='true')
        for key,value in {'out':str(out),'mode':mode,'library':str(build/'build/libonline_mppi.so'),'projection_progress':str(projection_progress).lower(),'classified_cloud':str(classified).lower(),'local_execution_owner':str(local_execution).lower(),'reference_delay':str(reference_delay)}.items():ET.SubElement(node,'param',name=key,value=value)
    ET.ElementTree(launch).write(out/'experiment.launch',encoding='unicode')
    resolved=roslaunch.config.load_config_default([str(out/'experiment.launch')],PORT,verbose=False)
    for key,value in params.items():
        if resolved.params[key].value!=value:raise RuntimeError('parameter drift: '+key)
    if any(n.type in ('base_control.py','joint_cmd_bridge.py') or 'piper' in n.package for n in resolved.nodes):raise RuntimeError('hardware node in experiment')
    manifest={'mode':mode,'scene':scene,'duration':duration,'scenario_start':scenario_start,'debug_native':debug_native,'port':PORT,'parameter_overrides':overrides,'experiment_profile':profile,
        'reference_source':'actual native /planning/trajectory PolynomialTraj; no synthetic fallback',
        'mppi_revision':build_manifest.get('controller_revision','unknown'),
        'perception_mode':'classified_atomic' if classified else 'merged',
        'nodes':[{'name':n.name,'type':n.type,'package':n.package} for n in resolved.nodes],
        'native_binaries_sha256':{n:hashlib.sha256((pkg/n).read_bytes()).hexdigest() for n in ('remani_planner_node','mm_controller_node','dynamic_obstacle_tracker_node')},
        'mppi_library_sha256':hashlib.sha256((build/'build/libonline_mppi.so').read_bytes()).hexdigest(),
        'scope':'native REMANI ROS planning/replanning/controller/detector; synthetic sensor+lagged differential plant; NOT Gazebo or physical robot',
        'native_safety_chain':'local selected-rollout + braking validator owns base execution; native guide collision requests refresh' if local_execution else 'unchanged and shared; MPPI must obey safety_speed_limit and abort trajectories',
        'local_execution_owner':local_execution,
        'reference_delay_s':reference_delay,
        'build':str(build),'planner_variant':planner_variant,'progress_contract':'measured_path_projection' if mode!='remani' and projection_progress else 'native_virtual_clock',
        'experiment_sources_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in HERE.iterdir() if p.suffix in ('.py','.cpp','.txt')}}
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n')
    print(out/'experiment.launch',flush=True)
    return out


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True);p.add_argument('--build',type=Path,required=True)
    p.add_argument('--mode',choices=['remani','mppi','mppi_cbf'],default='remani');p.add_argument('--scene',choices=['empty','static','crossing'],default='empty');p.add_argument('--duration',type=float,default=45)
    a=p.parse_args();prepare(a.out,a.build,a.mode,a.scene,a.duration)
