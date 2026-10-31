#!/usr/bin/env python3
"""Native controller clock + actual Python Tracker, no hardware publication."""
import argparse
import contextlib
import copy
import csv
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import textwrap
import threading
import time
from unittest.mock import patch
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import yaml
from simulator import Simulator, CASES, body_clearance

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
sys.path.insert(0,str(ROOT/'src/mm_remani/scripts'))
from real_base_tracker import RealBaseTracker


class Sink:
    def __init__(self,*args,**kwargs):self.last=None
    def publish(self,message):self.last=copy.deepcopy(message)


@contextlib.contextmanager
def tracker_clock(parameters,clock):
    def noop(*args,**kwargs):pass
    def param(name,default=None):return parameters.get('/real_base_tracker/'+name.lstrip('~'),default)
    with contextlib.ExitStack() as stack:
        for name in ('init_node','Subscriber','Timer','spin','on_shutdown','loginfo','logwarn','logerr',
                     'loginfo_throttle','logwarn_throttle','logerr_throttle'):
            stack.enter_context(patch.object(rospy,name,noop))
        stack.enter_context(patch.object(rospy,'Publisher',Sink));stack.enter_context(patch.object(rospy,'get_param',param))
        stack.enter_context(patch.object(rospy.Time,'now',lambda:rospy.Time.from_sec(1000+clock[0])))
        yield


class NativeClock:
    def __init__(self,binary,fixture,out,env):
        self.log=(out/'native_clock.log').open('a')
        self.proc=subprocess.Popen([str(binary),str(fixture)],env=env,stdin=subprocess.PIPE,stdout=subprocess.PIPE,
                                   stderr=self.log,text=True,bufsize=1)
        while True:
            line=self.proc.stdout.readline()
            if line.startswith('READY'):break
            if not line:raise RuntimeError('native clock failed during initialization')
            self.log.write(line)
    def step(self,t,state,reset=False,virtual=0.):
        self.proc.stdin.write(' '.join(map(str,[t,*state[:3],int(reset),virtual]))+'\n');self.proc.stdin.flush()
        while True:
            line=self.proc.stdout.readline()
            if line.startswith('ROW '):return np.fromstring(line[4:],sep=' ')
            if not line:raise RuntimeError('native clock stopped')
            self.log.write(line)
    def close(self):
        self.proc.stdin.close();self.proc.wait(timeout=5);self.log.close()


def command(tracker,row,state,omega_override=None):
    odom=Odometry();odom.header.stamp=rospy.Time.now();odom.pose.pose.position.x=state[0];odom.pose.pose.position.y=state[1]
    odom.pose.pose.orientation.z=math.sin(state[2]/2);odom.pose.pose.orientation.w=math.cos(state[2]/2)
    tracker.odom_cb(odom)
    message=Twist();message.linear.x,message.linear.y,message.linear.z=row[5:8]
    message.angular.x,message.angular.y,message.angular.z=row[8:11]
    if omega_override is not None:message.angular.z=omega_override
    tracker.car_cmd_cb(message);tracker.control_loop(None)
    result=tracker.cmd_pub.last
    error=tracker.err_pub.last.data if tracker.err_pub.last else [float('nan')]*4
    return np.array([result.linear.x,result.angular.z]),np.array(error)


def write_fixture(path,messages):
    with path.open('w') as f:
        f.write(str(len(messages))+'\n')
        for message in messages:
            f.write(f"{message['singul']} {len(message['polynomials'])}\n")
            for piece in message['polynomials']:
                if piece['num_dim']!=8 or piece['num_order']!=7:raise ValueError('unexpected native polynomial shape')
                f.write(str(piece['duration'])+' '+' '.join(map(str,piece['data']))+'\n')


def latch_tests(binary,out,env,params):
    c=np.zeros((8,8));c[0,-2]=.15
    fixture=out/'linear_reference.txt';write_fixture(fixture,[dict(singul=1,polynomials=[dict(duration=20.,num_dim=8,num_order=7,data=c.flatten(order='F').tolist())])])
    records=[]
    for name,prior,ey,ex,yaw in [('held_lateral_pos',True,.13,0.,0.),('held_lateral_neg',True,-.13,0.,0.),
        ('not_held_lateral',False,.13,0.,0.),('held_recoverable',True,.03,0.,0.),
        ('held_longitudinal',True,0.,.08,0.)]:
        native=NativeClock(binary,fixture,out,env);clock=[0.]
        try:
            with tracker_clock(params,clock):
                tracker=RealBaseTracker()
                if prior:native.step(0.,np.array([.15,-.17,0,0,0]),True,1.)
                # Actual pose relative to ref=(.15,0), yaw=0 for this fixture.
                state=np.array([.15-ex,-ey,yaw,0,0]);clock[0]=.02
                row=native.step(.02,state,not prior,1.);u,error=command(tracker,row,state)
                records.append(dict(case=name,prior_hold=prior,ex=ex,ey=ey,yaw=yaw,
                    hold=bool(row[2]),time_scale=row[1],virtual_time=row[0],v=u[0],w=u[1],reference_vx=row[8]))
        finally:native.close()
    checks={
        'held_positive_stuck':records[0]['hold'] and abs(records[0]['v'])+abs(records[0]['w'])<1e-10,
        'held_negative_stuck':records[1]['hold'] and abs(records[1]['v'])+abs(records[1]['w'])<1e-10,
        'same_error_without_hold_moves':not records[2]['hold'] and abs(records[2]['v'])+abs(records[2]['w'])>1e-5,
        'inside_recovery_releases':not records[3]['hold'],
        'longitudinal_feedback_remains':records[4]['hold'] and abs(records[4]['v'])>1e-5}
    checks={key:bool(value) for key,value in checks.items()}
    (out/'latch_cases.json').write_text(json.dumps(dict(cases=records,checks=checks,passed=all(checks.values())),indent=2)+'\n')
    if not all(checks.values()):raise RuntimeError('native freeze mechanism test failed '+str(checks))


def fixed_reference(binary,out,env,source):
    messages=json.loads((source/'trajectories.json').read_text());starts=[i for i,m in enumerate(messages) if m['action']==1 and m['id']==1]
    if len(starts)<2:raise RuntimeError('source has no replacement native trajectory')
    start=starts[-1];bundle=[m for m in messages[start:] if m['action']==1]
    fixture=out/'native_reference.txt';write_fixture(fixture,bundle)
    duration=sum(p['duration'] for m in bundle for p in m['polynomials'])
    states=np.genfromtxt(source/'state.csv',names=True,delimiter=',');index=np.argmin(abs(states['t']-messages[start]['t']))
    initial=np.array([states[index][k] for k in ('x','y','yaw','v','w')]);limits=json.loads((source/'limits.json').read_text())
    params=yaml.safe_load((source/'parameters.yaml').read_text());result=[]
    # Use the exact plant integration function; change only its response-time
    # constant to a parameter. All velocity/acceleration limits remain present.
    plant_source=textwrap.dedent(inspect.getsource(Simulator.integrate));assert plant_source.count('/.12')==1
    namespace=dict(np=np,math=math,CASES=CASES,body_clearance=body_clearance)
    exec(compile(plant_source.replace('/.12','/self.diagnosis_tau'),'<diagnostic exact plant>','exec'),namespace)
    integrate=namespace['integrate']
    variants=[('current',{}),('response_003',{'tau':.03}),('tracker_angular_accel_2',{'angular_accel':2.}),
              ('plant_wheel_speed_plus15pct',{'wheel_factor':1.15}),
              ('consistent_yaw_feedforward',{'yaw_ff':'reference_yaw_difference'})]
    for name,change in variants:
        native=NativeClock(binary,fixture,out,env);clock=[0.];rows=[];config=dict(tau=.12,wheel_factor=1.,angular_accel=1.)
        config.update(change);configuration=params.copy();configuration['/real_base_tracker/tracker_max_angular_accel']=config['angular_accel']
        plant=Simulator.__new__(Simulator);plant.s=initial.copy();plant.lock=threading.Lock();plant.command=np.zeros(2)
        plant.command_time=0.;plant.command_lease=.3;plant.limits=limits.copy();plant.limits['wheel_v']*=config['wheel_factor']
        plant.scene='held_reference_diagnostic';plant.t=0.;plant.diagnosis_tau=config['tau']
        previous_yaw=None
        try:
            with tracker_clock(configuration,clock):
                tracker=RealBaseTracker()
                for k in range(2250):
                    t=k*.02;clock[0]=t;plant.t=t
                    row=native.step(t,plant.s,k==0,0.)
                    reference_rate=0. if previous_yaw is None else math.atan2(math.sin(row[7]-previous_yaw),math.cos(row[7]-previous_yaw))/.02
                    previous_yaw=row[7]
                    override=max(-.55,min(.55,reference_rate)) if config.get('yaw_ff') else None
                    u,error=command(tracker,row,plant.s,override)
                    h=.5*limits['track'];c=plant.limits
                    ratio=max(1.,abs(u[0])/c['vmax'],abs(u[1])/c['wmax'],abs(u[0]-h*u[1])/c['wheel_v'],abs(u[0]+h*u[1])/c['wheel_v'])
                    rows.append([t,*plant.s,*u,*row[:5],*error,ratio,tracker.tracking_fault,row[10],tracker.target_w,reference_rate])
                    plant.command=u;plant.command_time=t;integrate(plant,.02)
        finally:native.close()
        array=np.array(rows);np.savetxt(out/(name+'.csv'),array,delimiter=',',comments='',
            header='t,x,y,yaw,v,w,cmd_v,cmd_w,virtual_time,time_scale,hold,position_error,yaw_error,tracker_dist,ex,ey,eyaw,plant_clip_ratio,tracking_fault,native_ff,used_ff,reference_yaw_rate')
        held=array[:,10]>.5;first=float(array[np.flatnonzero(held)[0],0]) if held.any() else None
        result.append(dict(variant=name,config=config,first_freeze_s=first,hold_seconds=float(held.sum()*.02),
            final_lateral_m=float(array[-1,15]),max_abs_lateral_m=float(np.max(np.abs(array[:,15]))),
            final_position_error_m=float(array[-1,13]),final_virtual_time_s=float(array[-1,8]),reference_duration_s=duration,
            command_clip_fraction=float(np.mean(array[:,17]>1+1e-9)),tracking_fault_fraction=float(np.mean(array[:,18]>.5))))
    report=dict(source=str(source),scope='held previously accepted native reference in an empty numerical world; no online replan or real-time transport; NOT historical closed-loop replay',
        initial_state=initial.tolist(),native_reference_sha256=hashlib.sha256(fixture.read_bytes()).hexdigest(),
        production_tracker_sha256=hashlib.sha256((ROOT/'src/mm_remani/scripts/real_base_tracker.py').read_bytes()).hexdigest(),
        plant_source_sha256=hashlib.sha256(plant_source.encode()).hexdigest(),variants=result)
    (out/'tracking_comparison.json').write_text(json.dumps(report,indent=2)+'\n');return report


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--build',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--source',type=Path,default=HERE/'results/resource_contention/validation_02/static_remani_01');a=p.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    with socket.socket() as s:
        if s.connect_ex(('127.0.0.1',11441))==0:raise RuntimeError('isolated ROS master already active')
    env=os.environ.copy();env.update(ROS_MASTER_URI='http://127.0.0.1:11441',ROS_IP='127.0.0.1',ROS_HOSTNAME='localhost',
        ROS_HOME=str(out/'ros_home'),ROS_LOG_DIR=str(out/'roslogs'))
    log=(out/'master.log').open('w');master=subprocess.Popen(['roscore','-p','11441'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        for _ in range(100):
            with socket.socket() as s:
                if s.connect_ex(('127.0.0.1',11441))==0:break
            time.sleep(.1)
        params=yaml.safe_load((a.source/'parameters.yaml').read_text());binary=a.build.resolve()/'build/freeze_reference_probe'
        latch_tests(binary,out,env,params);fixed_reference(binary,out,env,a.source.resolve())
    finally:
        os.killpg(master.pid,signal.SIGINT)
        try:master.wait(timeout=10)
        except subprocess.TimeoutExpired:os.killpg(master.pid,signal.SIGTERM);master.wait(timeout=5)
        log.close()
    print('TRACKING_COMPLETE',out,flush=True)


if __name__=='__main__':main()
