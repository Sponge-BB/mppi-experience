#!/usr/bin/env python3
"""Read-only production profile extraction; generate an isolated Gazebo launch."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import roslaunch
import rospkg
import yaml
from tf.transformations import euler_matrix

WORKSPACE=Path(__file__).resolve().parents[3]
NAMES={'remani_planner_node','mm_controller_node','real_base_tracker',
       'dynamic_obstacle_tracker','pcl_manager_filter','passthrough_z','crop_box_filter','filter_joint_states'}
PREFIXES=tuple('/'+n+'/' for n in NAMES)


def pose_matrix(element):
    values=[float(x) for x in element.findtext('pose','0 0 0 0 0 0').split()]
    if len(values)!=6:raise ValueError('unsupported SDF pose')
    m=euler_matrix(*values[3:]);m[:3,3]=values[:3];return m


def world_points(path,spacing=.08):
    """Sample collision BOX surfaces with ancestor transforms; ignore floor plane."""
    clouds=[]
    def walk(element,parent):
        transform=parent@pose_matrix(element)
        for collision in element.findall('collision'):
            geom=collision.find('geometry')
            if geom.find('plane') is not None:continue
            box=geom.find('box')
            if box is None:raise ValueError('unsupported static collision geometry: '+ET.tostring(geom).decode())
            half=np.array([float(x)/2 for x in box.findtext('size').split()])
            points=[]
            for axis in range(3):
                others=[i for i in range(3) if i!=axis]
                a,b=[np.linspace(-half[i],half[i],int(math.ceil(2*half[i]/spacing))+1) for i in others]
                for sign in (-1,1):
                    for u in a:
                        for v in b:
                            p=np.ones(4);p[axis]=sign*half[axis];p[others[0]]=u;p[others[1]]=v;points.append(p)
            mat=transform@pose_matrix(collision)
            clouds.append((np.asarray(points)@mat.T)[:,:3])
        for child in list(element):
            if child.tag in ('world','model','link'):walk(child,transform)
    walk(ET.parse(path).getroot(),np.eye(4))
    if not clouds:raise ValueError('no box collision geometry in Gazebo world')
    points=np.concatenate(clouds);points=points[(points[:,2]>=.13)&(points[:,2]<=1.70)]
    return np.unique(np.round(points,5),axis=0).astype(np.float32)


def prepare(out):
    out=Path(out).resolve();out.mkdir(parents=True,exist_ok=False)
    rp=rospkg.RosPack();real=Path(rp.get_path('mm_remani'))/'launch/exp0_real.launch'
    cfg=roslaunch.config.load_config_default([str(real)],11431,verbose=False)
    shared={k:p.value for k,p in cfg.params.items() if k.startswith(PREFIXES)}
    (out/'real_profile.yaml').write_text(yaml.safe_dump(shared,sort_keys=True))
    robot=ET.parse(Path(rp.get_path('my_description'))/'robot/mm.urdf')
    root=robot.getroot()
    # Local generated URDF only. Retain the physical wheels/arm, replace sensor
    # sampling with an explicitly approximate MID360-facing ray model.
    frame='livox_frame'
    root.find("joint[@name='velodyne_base_scan_joint']/origin").set('xyz','0 0 0')
    root.find("joint[@name='velodyne_base_scan_joint']/child").set('link',frame)
    root.find("link[@name='velodyne']").set('name',frame)
    gz=root.find("gazebo[@reference='velodyne']");gz.set('reference',frame)
    sensor=gz.find('sensor');sensor.find('update_rate').text='10'
    scan=sensor.find('ray/scan')
    for name,value in [('horizontal/samples','720'),('horizontal/min_angle',str(-math.pi)),
                       ('horizontal/max_angle',str(math.pi)),('vertical/samples','16'),
                       ('vertical/min_angle',str(math.radians(-7))),('vertical/max_angle',str(math.radians(52)))]:
        scan.find(name).text=value
    sensor.find('ray/range/min').text='0.10';sensor.find('ray/range/max').text='20.0'
    plugin=sensor.find('plugin');plugin.find('topicName').text='/sim/lidar_raw'
    plugin.find('frameName').text=frame;plugin.find('min_range').text='.10';plugin.find('max_range').text='20.0'
    offset=[float(cfg.params['/remani_planner_node/grid_map/lidar_offset_'+axis].value) for axis in 'xyz']
    root.find("joint[@name='velodyne_base_mount_joint']/origin").set('xyz',' '.join(map(str,offset)))
    # Matching planner joint limits; controller command slew is additionally bounded.
    mm='/remani_planner_node/mm/'
    for i in range(6):
        limit=root.find(f"joint[@name='joint{i+1}']/limit")
        if limit is not None:limit.set('velocity',str(shared[mm+'manipulator_max_vel']))
    robot.write(out/'robot.urdf',encoding='unicode')
    world=Path(rp.get_path('mm_gazebo'))/'world/world_2.world'
    np.save(out/'static_map.npy',world_points(world),allow_pickle=False)
    launch=ET.Element('launch')
    ET.SubElement(launch,'arg',name='gui',default='false')
    ET.SubElement(launch,'arg',name='rviz',default='false')
    ET.SubElement(launch,'param',name='/use_sim_time',value='true',type='bool')
    ET.SubElement(launch,'param',name='/gazebo_validation/enabled',value='true',type='bool')
    ET.SubElement(launch,'param',name='robot_description',textfile=str(out/'robot.urdf'))
    include=ET.SubElement(launch,'include',file='$(find gazebo_ros)/launch/empty_world.launch')
    for name,value in [('world_name',str(world)),('paused','true'),('use_sim_time','true'),('gui','$(arg gui)')]:
        ET.SubElement(include,'arg',name=name,value=value)
    arm=np.deg2rad(shared['/remani_planner_node/fsm/init_state'][2:])
    args='-param robot_description -urdf -model mm_robot -x 0 -y 0 -z 0.115 -unpause'
    args+=''.join(f' -J joint{i+1} {q}' for i,q in enumerate(arm))+' -J joint7 0 -J joint8 0'
    ET.SubElement(launch,'node',pkg='gazebo_ros',type='spawn_model',name='spawn_mm_robot',args=args,output='log')
    ET.SubElement(launch,'rosparam',file=str(Path(rp.get_path('mm_gazebo'))/'config/piper_gazebo_control.yaml'),command='load',ns='/gazebo')
    controllers=['joint_state_controller']+[w+'_wheel_velocity_controller' for w in ('fr','fl','bl','br')]+[f'joint{i}_position_controller' for i in range(1,9)]
    ET.SubElement(launch,'node',pkg='controller_manager',type='spawner',name='controller_spawner',ns='/gazebo',args=' '.join(controllers),output='log')
    rsp=ET.SubElement(launch,'node',pkg='robot_state_publisher',type='robot_state_publisher',name='robot_state_publisher')
    ET.SubElement(rsp,'remap',{'from':'joint_states','to':'/gazebo/joint_states'})
    for node in cfg.nodes:
        if node.name not in NAMES:continue
        attrs={'pkg':node.package,'type':node.type,'name':node.name,'ns':node.namespace,'output':'log'}
        if node.args:attrs['args']=node.args
        n=ET.SubElement(launch,'node',attrs)
        for source,target in node.remap_args:
            if target=='/livox/lidar_pcl2':target='/sim/lidar_raw'
            ET.SubElement(n,'remap',{'from':source,'to':target})
        if node.name=='real_base_tracker':ET.SubElement(n,'remap',{'from':'/cmd_vel','to':'/sim/cmd_vel'})
    ET.SubElement(launch,'rosparam',file=str(out/'real_profile.yaml'),command='load')
    for parent,child in [('world','map'),('map','odom')]:
        ET.SubElement(launch,'node',pkg='tf2_ros',type='static_transform_publisher',name=f'validation_{parent}_to_{child}',args=f'0 0 0 0 0 0 {parent} {child}')
    io=ET.SubElement(launch,'node',pkg='mm_gazebo',type='validation_io.py',name='validation_io',output='log',required='true')
    ET.SubElement(io,'param',name='static_map_file',value=str(out/'static_map.npy'))
    ET.SubElement(launch,'node',pkg='rviz',type='rviz',name='rviz',args='-d $(find mm_remani)/rviz/exp0_gazebo.rviz',attrib={'if':'$(arg rviz)'})
    ET.ElementTree(launch).write(out/'validation.launch',encoding='unicode')
    # Resolve the generated graph without starting any node; compare effective values.
    sim=roslaunch.config.load_config_default([str(out/'validation.launch')],11431,verbose=False)
    different={k:{'real':v,'sim':sim.params[k].value if k in sim.params else None} for k,v in shared.items() if k not in sim.params or sim.params[k].value!=v}
    if different:raise ValueError('shared parameter mismatch: '+str(different))
    if any(n.package in ('piper','livox_ros_driver2') or n.type in ('base_control.py','joint_cmd_bridge.py','odom_openloop_publisher.py','cmd_vel_to_model_twist.py') for n in sim.nodes):
        raise ValueError('hardware or nonphysical motion node entered simulation')
    sources=[real,Path(rp.get_path('mm_remani'))/'config/real_speed.yaml',Path(rp.get_path('my_description'))/'robot/mm.urdf',world]
    manifest={'shared_parameter_count':len(shared),'parameter_differences':different,
        'sources_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
        'nodes':[n.name for n in sim.nodes],
        'differences':['Gazebo ground-truth localization, not FAST-LIO/ICP',
        'approximate 720x16 ray sensor, 10 Hz, not a MID360 simulator',
        'physical URDF wheel radius 0.08 / track 0.411 retained; planner uses original real profile',
        'static map sampled from Gazebo collision geometry, not the real-room PCD',
        'simulated arm position control with real-profile command slew; not CAN motor identification'],
        'safety':'isolated localhost ROS master required; no hardware drivers; /sim/cmd_vel only'}
    (out/'sync_report.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps({'launch':str(out/'validation.launch'),'matched_parameters':len(shared),'map_points':len(np.load(out/'static_map.npy'))},indent=2))
    return out


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True)
    prepare(p.parse_args().out)
