#!/usr/bin/env python3
"""Shared, physically free whole-body ROS plant for standby comparisons."""
import csv
import json
import math
from pathlib import Path
import threading
import time

import numpy as np
import rospy
import tf2_ros
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs import point_cloud2 as pc2
from std_msgs.msg import Bool, Header, Float64MultiArray

from active_arm_model import PiperArmModel
from active_arm_simulator import WholeBodyPlant
from standby_arm_scene import actors_3d_at, obstacle_points
from simulator import isolation


class StandbyPlant:
    def __init__(self):
        self.out=Path(rospy.get_param('~out'))
        self.config=json.loads((self.out/'config.json').read_text())
        self.mode=self.config['mode'];self.scene=self.config['scene_definition']
        self.model=PiperArmModel.from_parameters(self.out/'parameters.yaml',self.config['urdf_path'])
        self.limits=json.loads((self.out/'limits.json').read_text())
        # Compatibility metadata only; actor motion and free physics are unchanged.
        plant_scene=dict(self.scene,schema='active_arm_scene_v1',q_retract=self.scene['posture_prior'])
        self.plant=WholeBodyPlant(self.model,self.limits,self.config['q_start'],plant_scene)
        self.lock=threading.Lock();self.t=0.;self.command=np.zeros(8);self.cmd_stamp=-1e9
        self.sequence=-1;self.lease=.25;self.native_pose=np.zeros(3);self.native_velocity=np.zeros(2)
        self.native_q=np.asarray(self.config['q_start']).copy();self.native_dq=np.zeros(6)
        self.native_stamp=-1e9;self.joint_stamp=-1e9
        self.rows=[];self.obstacles=[];self.goals=0;self.native_count=0;self.whole_count=0
        self.raw=(self.out/'native_commands.jsonl').open('w',buffering=1)
        self.clock=rospy.Publisher('/clock',Clock,queue_size=2)
        self.odom=rospy.Publisher('/odom',Odometry,queue_size=2)
        self.joints=rospy.Publisher('/mm/joint_states',JointState,queue_size=2)
        self.gripper=rospy.Publisher('/mm/mani/gripper_state',Bool,queue_size=1,latch=True)
        self.raw_cloud=rospy.Publisher('/sim/raw_cloud',PointCloud2,queue_size=1)
        self.observed=rospy.Publisher('/sim/observed_cloud_world',PointCloud2,queue_size=1)
        self.global_cloud=rospy.Publisher('/map_generator/global_cloud',PointCloud2,queue_size=1,latch=True)
        self.active=rospy.Publisher('/sim/experiment_active',Bool,queue_size=1,latch=True)
        self.estop=rospy.Publisher('/sim/experiment_estop',Bool,queue_size=1,latch=True)
        self.tf=tf2_ros.TransformBroadcaster()
        rospy.Subscriber('/sim/whole_body_command',Float64MultiArray,self.atomic,queue_size=1)
        rospy.Subscriber('/mm_controller_node/car_cmd',Twist,self.car_cmd,queue_size=1)
        rospy.Subscriber('/mm_controller_node/joint_cmd',JointTrajectoryControllerState,self.joint_cmd,queue_size=1)
        rospy.Subscriber('/move_base_simple/goal',PoseStamped,self.goal,queue_size=1)
        self.offset=np.asarray(self.config['lidar_offset'])
        self.warmup=float(self.config.get('warmup_s',7.))
        xmin,xmax,ymin,ymax=self.scene['arena']
        self.static=np.array([(x,y,z) for x in np.arange(xmin,xmax+.01,.08)
            for y in (ymin,ymax) for z in np.arange(.1,1.61,.12)]+
            [(x,y,z) for x in (xmin,xmax) for y in np.arange(ymin,ymax+.01,.08)
             for z in np.arange(.1,1.61,.12)],dtype=np.float32)

    def goal(self,message):
        self.goals+=1

    def atomic(self,message):
        a=np.asarray(message.data,dtype=float)
        if self.mode=='native_piper':return
        with self.lock:
            if (a.shape!=(12,) or not np.isfinite(a).all() or a[0]!=1. or
                    not 0<a[2]<=.25 or a[3]<=self.sequence or
                    not -.05<=1000+self.t-a[1]<a[2]):return
            self.sequence=int(a[3]);self.command=a[4:].copy()
            self.cmd_stamp=a[1]-1000;self.lease=a[2];self.whole_count+=1

    def car_cmd(self,message):
        if self.mode!='native_piper':return
        p=np.array([message.linear.x,message.linear.y,message.linear.z])
        v=np.array([message.angular.x,message.angular.y])
        if not np.isfinite(np.r_[p,v]).all():return
        with self.lock:
            self.native_pose=p;self.native_velocity=v;self.native_stamp=self.t;self.native_count+=1
            self.raw.write(json.dumps(dict(t=self.t-self.warmup,type='car_cmd',pose=p.tolist(),velocity=v.tolist()))+'\n')

    def joint_cmd(self,message):
        if self.mode!='native_piper':return
        q=np.asarray(message.desired.positions);dq=np.asarray(message.desired.velocities)
        if q.shape!=(6,) or dq.shape!=(6,) or not np.isfinite(np.r_[q,dq]).all():return
        with self.lock:
            self.native_q=q;self.native_dq=dq;self.joint_stamp=self.t;self.native_count+=1
            self.raw.write(json.dumps(dict(t=self.t-self.warmup,type='joint_cmd',q=q.tolist(),dq=dq.tolist()))+'\n')

    def controls(self,now):
        with self.lock:
            if self.mode!='native_piper':
                return self.command.copy() if -.05<=now-self.cmd_stamp<self.lease else np.zeros(8)
            state=self.plant.state
            u=np.zeros(8)
            if -.05<=now-self.native_stamp<.3:
                delta=self.native_pose[:2]-state[:2];c,s=np.cos(state[2]),np.sin(state[2])
                u[0]=self.native_velocity@np.array([c,s])+delta@np.array([c,s])
                yaw_error=(self.native_pose[2]-state[2]+np.pi)%(2*np.pi)-np.pi
                u[1]=1.5*yaw_error+.8*(delta@np.array([-s,c]))
            if -.05<=now-self.joint_stamp<.3:
                u[2:]=self.native_dq+1.8*(self.native_q-state[5:11])-.2*state[11:17]
            return u

    def state_publish(self):
        stamp=rospy.Time.from_sec(1000+self.t);z=self.plant.state
        self.clock.publish(Clock(stamp))
        odom=Odometry();odom.header=Header(stamp=stamp,frame_id='odom');odom.child_frame_id='base_link'
        odom.pose.pose.position.x,odom.pose.pose.position.y=z[:2]
        odom.pose.pose.orientation.z=math.sin(z[2]/2);odom.pose.pose.orientation.w=math.cos(z[2]/2)
        odom.twist.twist.linear.x,odom.twist.twist.angular.z=z[3:5];self.odom.publish(odom)
        joint=JointState();joint.header=Header(stamp=stamp);joint.name=['joint%d'%i for i in range(1,7)]
        joint.position=z[5:11].tolist();joint.velocity=z[11:17].tolist();joint.effort=[0.]*6
        self.joints.publish(joint);self.gripper.publish(Bool(False))
        trans=TransformStamped();trans.header=odom.header;trans.child_frame_id='base_link'
        trans.transform.translation.x,trans.transform.translation.y=z[:2];trans.transform.rotation=odom.pose.pose.orientation
        self.tf.sendTransform(trans)
        return stamp

    def cloud_publish(self,stamp):
        objects=obstacle_points(self.t-self.warmup,self.scene,spacing=.020)
        self.observed.publish(pc2.create_cloud_xyz32(Header(stamp=stamp,frame_id='world'),objects))
        world=np.concatenate([self.static,objects]);z=self.plant.state
        c,s=math.cos(z[2]),math.sin(z[2]);rotation=np.array([[c,s,0.],[-s,c,0.],[0.,0.,1.]])
        local=(world-np.r_[z[:2],0.])@rotation.T-self.offset
        self.raw_cloud.publish(pc2.create_cloud_xyz32(Header(stamp=stamp,frame_id='livox_frame'),local))

    def save(self,reason):
        header=['t','x','y','yaw','v','w']+['q%d'%i for i in range(1,7)]+['dq%d'%i for i in range(1,7)]
        header+=['clearance','configuration_gap','cmd_v','cmd_w']+['cmd_dq%d'%i for i in range(1,7)]
        for name,columns,rows in [('state',header,self.rows),('obstacles','t,x,y,r,id,vx,vy,z_min,z_max'.split(','),self.obstacles)]:
            with (self.out/(name+'.csv')).open('w') as stream:
                writer=csv.writer(stream);writer.writerow(columns);writer.writerows(rows)
        base=np.asarray([row[1:6] for row in self.rows])
        q=np.asarray([row[6:12] for row in self.rows])
        angle=np.unwrap(base[:,2])
        result=dict(reason=reason,collision_physical=self.plant.collided,joint_limit_violation=self.plant.joint_violation,
            elapsed=self.t-self.warmup,first_violation=self.plant.first_violation,
            min_external_gap_m=self.plant.min_external_gap,min_configuration_gap_m=self.plant.min_configuration_gap,
            max_base_displacement_m=float(np.linalg.norm(base[:,:2]-base[0,:2],axis=1).max()),
            base_path_length_m=float(np.linalg.norm(np.diff(base[:,:2],axis=0),axis=1).sum()),
            max_base_yaw_change_rad=float(abs(angle-angle[0]).max()),
            base_cumulative_turn_rad=float(abs(np.diff(angle)).sum()),
            final_base_error_m=float(np.linalg.norm(base[-1,:2]-base[0,:2])),
            final_base_yaw_error_rad=float(abs(angle[-1]-angle[0])),
            max_arm_change_deg=float(np.rad2deg(abs(q-q[0]).max())),
            final_arm_error_deg=float(np.rad2deg(abs(q[-1]-q[0]).max())),
            goal_messages_received=self.goals,native_commands_received=self.native_count,whole_body_commands_received=self.whole_count,
            physical_base_locked=False,maximum_integration_step_s=self.plant.max_substep,
            final_state=self.plant.state.tolist(),scope='actual ROS nodes, free whole-body numerical plant; no hardware',
            native_adapter='pose/vxy command to bounded differential velocity; joint q/dq to bounded joint velocity')
        (self.out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        self.raw.close()

    def run(self):
        started=time.monotonic();last=started;next_cloud=0.;next_record=0.;reason='shutdown'
        self.estop.publish(Bool(False));self.active.publish(Bool(False))
        try:
            while not rospy.is_shutdown():
                now=time.monotonic();previous=self.t;self.t=now-started;dt=self.t-previous
                count=max(1,int(math.ceil(dt/.005)));step=dt/count
                for k in range(count):
                    subtime=previous+(k+1)*step;command=self.controls(subtime)
                    self.plant.step(command,step,subtime-self.warmup,subtime-self.warmup)
                stamp=self.state_publish()
                elapsed=self.t-self.warmup
                if self.t>=next_cloud:
                    self.cloud_publish(stamp);self.active.publish(Bool(elapsed>=0));self.estop.publish(Bool(False))
                    next_cloud=self.t+.1
                    self.global_cloud.publish(pc2.create_cloud_xyz32(Header(stamp=stamp,frame_id='world'),self.static))
                if elapsed>=0 and self.t>=next_record:
                    z=self.plant.state
                    self.rows.append([elapsed,*z[:5],*z[5:11],*z[11:17],self.plant.external_gap,self.plant.configuration_gap,*self.controls(self.t)])
                    for a in actors_3d_at(elapsed,self.scene):self.obstacles.append([elapsed]+[a[k] for k in ('x','y','r','id','vx','vy','z_min','z_max')])
                    next_record=self.t+.02
                if self.plant.collided:reason='collision';break
                if self.plant.joint_violation:reason='joint_limit_violation';break
                if elapsed>=self.config['duration']:reason='completed';break
                time.sleep(.002)
        finally:
            self.active.publish(Bool(False));self.estop.publish(Bool(True))
            if self.rows:self.save(reason)


if __name__=='__main__':
    isolation();rospy.init_node('standby_comparison_plant')
    if not rospy.get_param('/online_validation/enabled',False) or not rospy.get_param('/use_sim_time',False):
        raise RuntimeError('isolated numerical execution only')
    StandbyPlant().run()
