#!/usr/bin/python3
"""One serial, headless comparison trial on the current main project."""
import argparse
from collections import deque
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time
import xml.etree.ElementTree as ET

for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'
os.environ['PYTHONNOUSERSITE'] = '1'
os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:11466'
os.environ['ROS_IP'] = '127.0.0.1'
os.environ.pop('ROS_HOSTNAME', None)

import psutil
import numpy as np
import roslaunch
import rospy
from rospy.numpy_msg import numpy_msg
import yaml
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from quadrotor_msgs.msg import PolynomialTraj
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray, String

ROOT = Path(__file__).resolve().parents[3]


class SnapshotHistory:
    """Small offline-only history; delta records share an immutable ESDF array."""
    def __init__(self):
        self.records = deque(maxlen=12)
        self.grid_key = self.distance = None

    def add(self, wire):
        a = np.asarray(wire, dtype=np.float64)
        if a.ndim != 1 or len(a) < 32 or not np.isfinite(a[:32]).all():
            return None
        h = a[:32]
        shape = h[5:8].astype(int)
        nb, nd = int(h[12]), int(h[13])
        cells = int(np.prod(shape)); prefix = 32+7*(nb+nd)
        if (tuple(h[:2]) != (3.,1.) or np.any(shape < 2) or not np.array_equal(shape,h[5:8])
                or not 0 < cells <= 500000 or nb < 1 or nd < 0 or nb != h[12] or nd != h[13]
                or 8*(prefix+cells) > 8*1024**2
                or h[20] not in (0.,1.) or len(a) != prefix+(cells if h[20] == 1 else 0)):
            return None
        key = tuple(h[[5,6,7,8,9,10,11,14]])
        if h[20] == 1:
            self.distance = a[prefix:].copy()
            self.distance.setflags(write=False)
            self.grid_key = key
        elif self.distance is None or key != self.grid_key:
            return None
        record = (a[:prefix].copy(), self.distance)
        record[0].setflags(write=False)
        self.records.append(record)
        while self.byte_count() > 8*1024**2:
            self.records.popleft()
        return record

    def byte_count(self):
        fields = {id(distance): distance.nbytes for _,distance in self.records}
        return sum(fields.values())+sum(prefix.nbytes for prefix,_ in self.records)

    def find(self, sequence):
        return next((record for record in reversed(self.records) if record[0][2] == sequence), None)

    @staticmethod
    def full_wire(record):
        wire = np.concatenate(record)
        wire[20] = 1.  # Offline serialization only; input/header records stay unchanged.
        return wire


def prepare(args):
    out = args.out.resolve(); out.mkdir(parents=True, exist_ok=False)
    config = roslaunch.ROSLaunchConfig()
    roslaunch.XmlLoader().load(str(ROOT/'src/mm_remani/launch/exp0_real.launch'), config,
                              argv=['use_mppi:=true', 'hardware_enabled:=false'], verbose=False)
    prefixes = ('/remani_planner_node/', '/mm_controller_node/', '/whole_body_mppi/',
                '/dynamic_obstacle_tracker/', '/real_base_tracker/', '/joint_cmd_bridge/')
    params = {k: v.value for k, v in config.params.items() if k.startswith(prefixes)}
    # grid_map offsets are relative to base_link, not ground. The original
    # localization bridge places that planar frame 0.115 m above ground.
    base_z = float(config.params['/fastlio_to_odom/base_z'].value)
    lidar_offset = [float(params['/remani_planner_node/grid_map/lidar_offset_'+axis]) for axis in 'xyz']
    bridge_offset = [float(config.params['/fastlio_to_odom/lidar_offset_'+axis].value) for axis in 'xyz']
    if not np.isfinite([base_z, *lidar_offset]).all() or not np.allclose(lidar_offset, bridge_offset, rtol=0., atol=1e-12):
        raise ValueError('planner and localization lidar geometry must agree')
    params['/use_sim_time'] = True
    profile = yaml.safe_load((ROOT/'src/mm_remani/config/mppi.yaml').read_text())
    overrides = {}
    for name in ('remani_planner_node', 'mm_controller_node'):
        for key, val in {'fsm/target_type': 1, 'fsm/init_state': [0.,0.,0.,110.,-60.,0.,30.,0.],
                         'fsm/init_gripper_close': False, 'fsm/capture_startup_standby': False}.items():
            overrides['/'+name+'/'+key] = val
    for key, val in {'grid_map/map_size_x':4., 'grid_map/map_size_y':4., 'grid_map/map_size_z':2.5,
                     'grid_map/local_update_range_x':2., 'grid_map/local_update_range_y':2.,
                     'grid_map/local_update_range_z':2.5, 'grid_map/virtual_ceil_height':-1.,
                     'grid_map/self_filter_enable':False, 'mppi/crop_half_extent':1.,
                     'mppi/retain_complete_snapshot':True,
                     'mppi/enable': True}.items():
        overrides['/remani_planner_node/'+key] = val
    if args.goal_x is not None:
        # Shared navigation task settings, for native and MPPI alike.
        overrides.update({'/remani_planner_node/fsm/'+k:v for k,v in {
            'manual_use_work_arm_waypoint':False, 'manual_target_end_arm_standby':False,
            'manual_restore_standby_after_goal':False, 'goal_position_tolerance':.05}.items()})
    # The common scenario moves at 0.08 m/s, below the hardware tracker profile.
    # These are common sensing settings, not oracle velocities or future paths.
    tracker = {'input_topic':'/sim/cloud', 'static_map_topic':'/sim/static_cloud',
               'velocity_deadband':.015, 'dynamic_enter_speed':.04, 'dynamic_exit_speed':.02,
               'minimum_dynamic_displacement':.02, 'stationary_background_maximum_speed':.015,
               'maximum_stationary_background_anchors':0}
    overrides.update({'/dynamic_obstacle_tracker/'+k:v for k,v in tracker.items()})
    for key in ('position_uncertainty','velocity_uncertainty','acceleration_bound'):
        name='/remani_planner_node/dynamic_point_flow/'+key
        overrides[name]=params[name]*args.uncertainty_scale
    overrides.update({'/whole_body_mppi/'+k:v for k,v in
                      dict(execute=True, simulation=True, arm_enabled=args.group != 'mppi_fixed_arm',
                           seed=args.seed, arm_position_servo=args.arm_execution_model == 'position_servo',
                           startup_to_standby=False).items()})
    limits = profile['base_limits']
    for key, value in {'tracker_max_v':limits['vmax'], 'tracker_max_w':limits['wmax'],
                       'tracker_max_linear_accel':limits['accel'],
                       'tracker_max_angular_accel':limits['angular_accel']}.items():
        overrides['/real_base_tracker/'+key] = value
    for key, value in dict(out=str(out), scene=args.scene, duration=args.duration,
                           sensor_model=args.sensor_model, base_z=base_z, lidar_offset=lidar_offset,
                           goal=[] if args.goal_x is None else [args.goal_x,0.,0.],
                           realtime_factor=args.realtime_factor, base_limits=limits,
                           arm_velocity=profile['arm_velocity'],
                           arm_acceleration=profile['arm_acceleration']).items():
        overrides['/standby_simulator/'+key] = value
    params.update(overrides)
    (out/'parameters.yaml').write_text(yaml.safe_dump(params))
    (out/'planner_params.yaml').write_text(yaml.safe_dump(
        {k:v for k,v in params.items() if k.startswith('/remani_planner_node/')}))
    (out/'simulation_overrides.json').write_text(json.dumps(overrides, indent=2)+'\n')
    metadata = dict(group=args.group, seed=args.seed, scene=args.scene,
        source='current main project; native control means MPPI disabled, not untouched upstream',
        physical_base_locked=False, controller_profile=profile, duration_s=args.duration,
        navigation_goal=None if args.goal_x is None else [args.goal_x,0.,0.],
        uncertainty_scale=args.uncertainty_scale,
        arm_execution_model=args.arm_execution_model,
        sensor_model=args.sensor_model,
        sensor_geometry=dict(base_link_ground_z_m=base_z, base_to_lidar_xyz_m=lidar_offset,
            lidar_ground_z_m=base_z+lidar_offset[2], base_to_lidar_rpy_rad=[0.,0.,0.],
            source_parameters=['/fastlio_to_odom/base_z', '/remani_planner_node/grid_map/lidar_offset_{x,y,z}',
                               '/fastlio_to_odom/lidar_offset_{x,y,z}'],
            elevation_deg=[-7.,52.] if args.sensor_model=='mid360' else None,
            minimum_range_m=.1 if args.sensor_model=='mid360' else None,
            cylinder_occlusion=args.sensor_model=='mid360', robot_occlusion=False,
            static_map_is_complete_known_prior=True, physical_calibration_performed=False,
            limitations='geometric envelope only; no scan pattern, reflectivity or range noise; '
                        'ideal emits full sampled surfaces with no visibility filtering'),
        command_topics=['/sim/cmd_vel','/sim/joint_ctrl_single'], master=os.environ['ROS_MASTER_URI'])
    (out/'experiment.json').write_text(json.dumps(metadata, indent=2)+'\n')
    launch = ET.Element('launch')
    ET.SubElement(launch, 'rosparam', file=str(out/'parameters.yaml'), command='load')
    selected = {'remani_planner_node', 'mm_controller_node', 'dynamic_obstacle_tracker'}
    if args.group != 'native': selected.add('whole_body_mppi')
    replacements = {'/odom':'/sim/odom', '/mm/joint_states':'/sim/joint_states',
                    '/velodyne_points':'/sim/cloud', '/gazebo/gripper_state':'/sim/gripper_state'}
    for original in config.nodes:
        if original.name not in selected: continue
        node = ET.SubElement(launch, 'node', pkg=original.package, type=original.type,
                             name=original.name, output='log', required='true')
        remaps = dict(original.remap_args)
        remaps = {k:replacements.get(v,v) for k,v in remaps.items()}
        remaps.update({'/map_generator/global_cloud':'/sim/static_cloud',
                      '/odom':'/sim/odom', '/joint_states_single':'/sim/joint_states',
                      '/cmd_vel':'/sim/cmd_vel', '/joint_ctrl_single':'/sim/joint_ctrl_single'})
        if original.name == 'remani_planner_node': remaps['~gripper_state']='/sim/gripper_state'
        for source, target in remaps.items(): ET.SubElement(node, 'remap', {'from':source, 'to':target})
    if args.group == 'native':
        for pkg, script, name in [('mm_remani','real_base_tracker.py','real_base_tracker'),
                                  ('mm_controller','joint_cmd_bridge.py','joint_cmd_bridge')]:
            node = ET.SubElement(launch,'node',pkg=pkg,type=script,name=name,output='log',required='true')
            for source,target in {'/odom':'/sim/odom', '/cmd_vel':'/sim/cmd_vel',
                                  '/joint_ctrl_single':'/sim/joint_ctrl_single'}.items():
                ET.SubElement(node,'remap',{'from':source,'to':target})
    ET.SubElement(launch,'node',pkg='mm_remani',type='standby_simulator.py',
                  name='standby_simulator',output='log')
    ET.ElementTree(launch).write(str(out/'trial.launch'),encoding='unicode')
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--group',choices=['native','mppi_fixed_arm','mppi_arm'],required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--seed',type=int,default=11)
    parser.add_argument('--scene',choices=['empty','arm_crossing','navigation_arm_crossing','navigation_arm_enter_blind'],default='arm_crossing')
    parser.add_argument('--sensor-model',choices=['ideal','mid360'],default='mid360',
                        help='mid360: geometric FOV/range/cylinder visibility; ideal: complete sampled surfaces')
    parser.add_argument('--goal-x',type=float,default=None,
                        help='Optional real REMANI goal in world coordinates; default is standby')
    parser.add_argument('--duration',type=float,default=24.)
    parser.add_argument('--realtime-factor',type=float,choices=[.5,1.],default=1.)
    parser.add_argument('--uncertainty-scale',type=float,choices=[0.,1.],default=1.,
                        help='0 is a diagnostic ablation only; never a hardware recommendation')
    parser.add_argument('--arm-execution-model', choices=['velocity_legacy', 'position_servo'],
                        default='position_servo', help='nominal numerical plant model; simulation only')
    args = parser.parse_args()
    if args.goal_x is not None and (not np.isfinite(args.goal_x) or not .1<=args.goal_x<=.55):
        parser.error('--goal-x must be between 0.1 and 0.55 m inside the audited arena')
    if args.scene.startswith('navigation_') and args.goal_x is None:
        parser.error('navigation scenes require --goal-x')
    with socket.socket() as probe:
        if probe.connect_ex(('127.0.0.1',11466)) == 0:
            raise RuntimeError('port 11466 already in use; refusing to join an existing graph')
    out=prepare(args)
    os.environ['ROS_LOG_DIR']=str(out/'roslog')
    log=(out/'launch.log').open('w')
    proc=subprocess.Popen(['roslaunch','--port','11466',str(out/'trial.launch')],
                          env=os.environ.copy(),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    began=time.monotonic(); peak=0; start=None; last={}; commands={'base':0,'arm':0}
    delivered=deque(maxlen=20)
    snapshot_history=SnapshotHistory(); recording_lock=threading.Lock()
    streams=[]; result={'runner_reason':'not_started'}
    try:
        import xmlrpc.client
        for _ in range(150):
            if proc.poll() is not None: raise RuntimeError('roslaunch exited; see launch.log')
            try:
                server=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
                if server.getPid('/sim_trial')[0] == 1 and server.getParam('/sim_trial','/use_sim_time')[2] is True: break
            except OSError: pass
            time.sleep(.1)
        else: raise RuntimeError('ROS master startup timeout')
        rospy.init_node('standby_trial_recorder',disable_signals=True)
        status=(out/'mppi_status.jsonl').open('w',buffering=1)
        snapshots=(out/'snapshot_headers.jsonl').open('w',buffering=1)
        observations=(out/'dynamic_observations.jsonl').open('w',buffering=1)
        navigation=(out/'navigation_events.jsonl').open('w',buffering=1)
        streams=[status,snapshots,observations,navigation]
        def event(kind, data):
            navigation.write(json.dumps(dict(kind=kind, ros_time=rospy.get_time(),
                elapsed_s=None if start is None else rospy.get_time()-start, data=data))+'\n')
        def on_trajectory(msg):
            event('trajectory',dict(stamp=msg.header.stamp.to_sec(),frame=msg.header.frame_id,
                action=msg.action,trajectory_id=msg.trajectory_id,singul=msg.singul,
                pieces=[dict(num_dim=p.num_dim,num_order=p.num_order,duration=p.duration,
                             data=list(p.data)) for p in msg.trajectory]))
        def on_progress(msg):
            event('progress',dict(stamp=msg.header.stamp.to_sec(),start=msg.vector.x,
                                 phase=msg.vector.y,scale=msg.vector.z))
        def on_finish(msg):
            event('finish',bool(msg.data))
            if msg.data and 'goal_sent' in last:
                last.setdefault('finish_after_goal',rospy.get_time())
        def save_failure_if_available():
            with recording_lock:
                pending = last.get('pending_failure')
                if pending is None or last.get('saved_failure'):
                    return
                requested = pending['status']['candidate_diagnostics']['snapshot_header']
                record = snapshot_history.find(requested[2])
                if record is None:
                    return
                last['saved_failure'] = True
            np.save(str(out/'failure_snapshot.npy'),snapshot_history.full_wire(record),allow_pickle=False)
            (out/'failure_status.json').write_text(json.dumps(pending,indent=2)+'\n')
        def on_status(msg):
            value=json.loads(msg.data); last['status']=value
            if value.get('state')=='command_delivered':delivered.append(rospy.get_time())
            record=dict(elapsed_s=None if start is None else rospy.get_time()-start,
                        ros_time=rospy.get_time(),status=value)
            status.write(json.dumps(record)+'\n')
            requested=value.get('candidate_diagnostics',{}).get('snapshot_header',[])
            if value.get('state','').startswith('no_feasible') and len(requested)==32:
                with recording_lock: last.setdefault('pending_failure',record)
                save_failure_if_available()
        def on_snapshot(msg):
            if len(msg.data)<32:return
            h=list(msg.data[:32]);last['snapshot']=h
            key=(h[18],h[19],h[13])
            nb,nd=int(h[12]),int(h[13]); offset=32+7*nb
            if (key != last.get('observation_key') and 0<=nd<=4096 and nb>=0
                    and len(msg.data)>=offset+7*nd):
                # Observations only, once per source update. The controller
                # never reads this offline audit stream or scenario truth.
                observations.write(json.dumps(dict(
                    elapsed_s=None if start is None else rospy.get_time()-start,
                    ros_time=rospy.get_time(),header=h,
                    points=np.asarray(msg.data[offset:offset+7*nd]).reshape((-1,7)).tolist()))+'\n')
                last['observation_key']=key
            with recording_lock:
                record=snapshot_history.add(msg.data)
                save_dynamic=(record is not None and h[4]==1 and h[19]==1 and h[13]>0
                              and 'saved_dynamic' not in last)
                if save_dynamic: last['saved_dynamic']=True
            if save_dynamic:
                np.save(str(out/'first_dynamic_snapshot.npy'),snapshot_history.full_wire(record),allow_pickle=False)
            save_failure_if_available()
            snapshots.write(json.dumps(dict(elapsed_s=None if start is None else rospy.get_time()-start,
                                            header=h))+'\n')
        def count(name):
            def callback(_): commands[name]+=1
            return callback
        rospy.Subscriber('/mppi/status',String,on_status,queue_size=100)
        rospy.Subscriber('/mppi/collision_snapshot',numpy_msg(Float64MultiArray),on_snapshot,queue_size=1,buff_size=8*1024**2)
        rospy.Subscriber('/sim/cmd_vel',Twist,count('base'),queue_size=10)
        rospy.Subscriber('/sim/joint_ctrl_single',JointState,count('arm'),queue_size=10)
        rospy.Subscriber('/planning/trajectory',PolynomialTraj,on_trajectory,queue_size=50)
        rospy.Subscriber('/mm_controller_node/trajectory_progress',Vector3Stamped,on_progress,queue_size=50)
        rospy.Subscriber('/planning/safety_speed_limit',Float64,lambda m:event('speed_limit',m.data),queue_size=30)
        rospy.Subscriber('/tracking_fault',Bool,lambda m:event('tracking_fault',bool(m.data)),queue_size=30)
        rospy.Subscriber('/planning/finish',Bool,on_finish,queue_size=10)
        start_pub=rospy.Publisher('/sim/start',Bool,queue_size=1,latch=True)
        goal_pub=rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1)
        reason='wall_timeout'; last_print=0
        while time.monotonic()-began < 150+args.duration/args.realtime_factor:
            try: rss=sum(p.memory_info().rss for p in [psutil.Process(proc.pid)]+psutil.Process(proc.pid).children(recursive=True) if p.is_running())
            except psutil.Error: rss=0
            peak=max(peak,rss)
            if psutil.virtual_memory().available < 1200*1024**2 or rss>1600*1024**2:
                reason='memory_guard';break
            if (out/'result.json').exists(): reason='plant_finished';break
            if proc.poll() is not None: reason='roslaunch_exited';break
            h=last.get('snapshot',[])
            ready=(len(h)==32 and h[4]==1 and h[19]==1 and commands['base']>0 and commands['arm']>0)
            if args.group!='native':
                ready=ready and sum(rospy.get_time()-t<1.5 for t in list(delivered))>=5
            if args.goal_x is not None:
                ready=ready and goal_pub.get_num_connections()>=(1 if args.group=='native' else 2)
            if ready and start is None:
                start=rospy.get_time();start_pub.publish(Bool(True))
                if args.goal_x is not None:
                    goal=PoseStamped();goal.header.stamp=rospy.Time.now();goal.header.frame_id='world'
                    goal.pose.position.x=args.goal_x;goal.pose.orientation.w=1.
                    last['goal_sent']=goal.header.stamp.to_sec()
                    event('goal',dict(stamp=last['goal_sent'],x=args.goal_x,y=0.,yaw=0.))
                    goal_pub.publish(goal)
                print('START',args.group,'seed',args.seed,'rss_MiB',round(rss/1024**2),flush=True)
            if start is None and time.monotonic()-began>100:
                reason='readiness_timeout';break
            if time.monotonic()-last_print>15:
                print('progress',args.group,'elapsed',None if start is None else round(rospy.get_time()-start,2),
                      'status',last.get('status',{}).get('state'), 'snapshot_valid',h[4] if h else None,
                      'rss_MiB',round(rss/1024**2),flush=True);last_print=time.monotonic()
            time.sleep(.2)
        with recording_lock:
            recording_summary=dict(first_dynamic_saved=bool(last.get('saved_dynamic')),
                first_failure_saved=bool(last.get('saved_failure')),
                cached_records=len(snapshot_history.records),cached_bytes=snapshot_history.byte_count())
        result=dict(runner_reason=reason,peak_graph_rss_mib=peak/1024**2,
                    wall_s=time.monotonic()-began,start_ros_time=start,last_status=last.get('status'),
                    last_snapshot=last.get('snapshot'),commands=commands,
                    goal_sent_ros_time=last.get('goal_sent'),
                    finish_after_goal_ros_time=last.get('finish_after_goal'),
                    snapshot_recording=recording_summary)
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
        for stream in streams:stream.close()
        log.close()
        (out/'runner.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
