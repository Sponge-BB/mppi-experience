#!/usr/bin/env python3
"""Numerical ROS plant + synthetic sensor, never a hardware driver."""
import csv
import hashlib
import io
import json
import math
import os
import threading
import time
from pathlib import Path
from urllib.parse import urlparse
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped, TransformStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs import point_cloud2 as pc2
from std_msgs.msg import Header, Bool, Float64, String
from rosgraph_msgs.msg import Clock, Log
from quadrotor_msgs.msg import PolynomialTraj
from control_msgs.msg import JointTrajectoryControllerState


def isolation():
    uri=urlparse(os.environ.get('ROS_MASTER_URI',''))
    if uri.hostname not in ('127.0.0.1','localhost') or uri.port!=11441:
        raise RuntimeError('This experiment requires localhost:11441; refusing other ROS masters')


def polynomial(msg, step=.10):
    rows=[]
    offset=0.
    for piece in msg.trajectory:
        coef=np.array(piece.data).reshape((piece.num_dim,piece.num_order+1),order='F')
        for t in np.linspace(0,piece.duration,max(2,int(piece.duration/step)+1)):
            pos=np.array([np.polyval(c,t) for c in coef])
            vel=np.array([np.polyval(np.polyder(c),t) for c in coef])
            speed=np.linalg.norm(vel[:2])*msg.singul
            yaw=math.atan2(vel[1]*msg.singul,vel[0]*msg.singul) if abs(speed)>1e-5 else float('nan')
            rows.append([offset+t,pos[0],pos[1],yaw,speed,*pos[2:]])
        offset+=piece.duration
    return np.array(rows)


class Simulator:
    def __init__(self):
        self.out=Path(rospy.get_param('~out'));self.scene=rospy.get_param('~scene')
        self.duration=float(rospy.get_param('~duration'));self.q=np.deg2rad(rospy.get_param('/remani_planner_node/fsm/init_state')[2:])
        self.limits=json.loads((self.out/'limits.json').read_text());self.body=np.loadtxt(str(self.out/'geometry.csv'),delimiter=',')
        self.s=np.zeros(5);self.command=np.zeros(2);self.command_time=-1e9;self.limit=-1.;self.progress=[0.,0.,0.]
        self.t=0.;self.goal_time=None;self.trigger=False;self.health=False;self.dynamic_n=0;self.dynamic_age=1e6
        self.scenario_start=rospy.get_param('~scenario_start','goal');self.motion_time=None
        self.traj_count=0;self.arm_error=0.;self.min_clearance=1e6;self.collided=False
        self.lock=threading.Lock();self.events=[];self.raw_trajs=[];self.diagnostics=[];self.rows=[];self.obstacle_rows=[]
        self.odom_pub=rospy.Publisher('/odom',Odometry,queue_size=2)
        self.joint_pub=rospy.Publisher('/gazebo/joint_states',JointState,queue_size=2)
        self.gripper_pub=rospy.Publisher('/mm/mani/gripper_state',Bool,queue_size=1,latch=True)
        self.clock_pub=rospy.Publisher('/clock',Clock,queue_size=2)
        self.raw_pub=rospy.Publisher('/sim/raw_cloud',PointCloud2,queue_size=1)
        self.map_pub=rospy.Publisher('/map_generator/global_cloud',PointCloud2,queue_size=1,latch=True)
        self.goal_pub=rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1)
        self.tf=tf2_ros.TransformBroadcaster()
        self.local_execution_owner=rospy.get_param('~local_execution_owner',False)
        self.command_lease=.075 if self.local_execution_owner else .3
        if self.local_execution_owner:rospy.Subscriber('/sim/local_execution_cmd',TwistStamped,self.local_cmd,queue_size=1)
        else:rospy.Subscriber('/sim/cmd_vel',Twist,self.cmd,queue_size=1)
        rospy.Subscriber('/planning/safety_speed_limit',Float64,self.safety,queue_size=2)
        rospy.Subscriber('/planning/trajectory',PolynomialTraj,self.trajectory,queue_size=20)
        rospy.Subscriber('/traj_start_trigger',PoseStamped,lambda m:setattr(self,'trigger',True),queue_size=2)
        rospy.Subscriber('/dynamic_obstacle_tracker/perception_healthy',Bool,lambda m:setattr(self,'health',m.data),queue_size=2)
        rospy.Subscriber('/dynamic_points',PointCloud2,self.dynamic,queue_size=2)
        rospy.Subscriber('/mm_controller_node/trajectory_progress',Vector3Stamped,lambda m:setattr(self,'progress',[m.vector.x,m.vector.y,m.vector.z]),queue_size=2)
        rospy.Subscriber('/mm_controller_node/joint_cmd',JointTrajectoryControllerState,self.arm,queue_size=2)
        rospy.Subscriber('/rosout_agg',Log,self.log,queue_size=2000)
        rospy.Subscriber('/sim/mppi_diagnostics',String,lambda m:self.diagnostics.append(json.loads(m.data)),queue_size=100)
        zs=np.arange(.15,1.41,.12);xs=np.arange(-1.5,4.51,.06);ys=np.arange(-2.,2.01,.06)
        self.static=np.array([(x,y,z) for x in xs for y in (-2.,2.) for z in zs]+
                             [(x,y,z) for x in (-1.5,4.5) for y in ys for z in zs],dtype=np.float32)
        self.offset=np.array([rospy.get_param('/remani_planner_node/grid_map/lidar_offset_'+a) for a in 'xyz'])

    def cmd(self,m):
        with self.lock:self.command=np.array([m.linear.x,m.angular.z]);self.command_time=self.t

    def local_cmd(self,m):
        stamp=m.header.stamp.to_sec()-1000
        with self.lock:
            if m.header.frame_id!='world' or stamp<self.command_time or not -.05<=self.t-stamp<self.command_lease:return
            if not np.all(np.isfinite([m.twist.linear.x,m.twist.angular.z])):return
            self.command=np.array([m.twist.linear.x,m.twist.angular.z]);self.command_time=stamp

    def safety(self,m):
        self.limit=m.data;self.events.append({'t':self.t,'kind':'safety_limit','value':m.data})

    def dynamic(self,m):
        self.dynamic_n=m.width*m.height;self.dynamic_age=1000+self.t-m.header.stamp.to_sec()

    def trajectory(self,m):
        points=polynomial(m) if m.action==1 else np.empty((0,0))
        if len(points):self.arm_error=max(self.arm_error,float(np.max(np.abs(points[:,5:]-self.q))))
        if m.action==1 and m.trajectory_id==1:self.traj_count+=1
        raw=io.BytesIO();m.serialize(raw)
        canonical=PolynomialTraj().deserialize(raw.getvalue());canonical.header.seq=0
        canonical_stream=io.BytesIO();canonical.serialize(canonical_stream)
        record={'t':self.t,'stamp':m.header.stamp.to_sec(),'id':m.trajectory_id,'action':m.action,'singul':m.singul,'frame':m.header.frame_id,'message_sha256':hashlib.sha256(raw.getvalue()).hexdigest(),
            'canonical_message_sha256':hashlib.sha256(canonical_stream.getvalue()).hexdigest(),
            'polynomials':[{'duration':p.duration,'num_dim':p.num_dim,'num_order':p.num_order,'data':list(p.data)} for p in m.trajectory],'points':points.tolist()}
        self.raw_trajs.append(record);self.events.append({k:v for k,v in record.items() if k not in ('points','polynomials')})

    def arm(self,m):
        if len(m.desired.positions)==6:self.arm_error=max(self.arm_error,float(np.max(np.abs(np.array(m.desired.positions)-self.q))))

    def log(self,m):
        self.events.append({'t':self.t,'kind':'rosout','name':m.name,'level':m.level,'message':m.msg})

    def obstacles(self):
        anchor=self.motion_time if self.scenario_start=='motion' else self.goal_time
        if anchor is None:return []
        elapsed=self.t-anchor
        if self.scene=='static' and elapsed>=4:return [(1.8,0.,.23)]
        if self.scene=='crossing' and 1<=elapsed<=14:return [(1.65,-1.6+.26*(elapsed-1),.23)]
        return []

    def integrate(self,dt):
        with self.lock:u=self.command.copy() if self.t-self.command_time<self.command_lease else np.zeros(2)
        c=self.limits;h=.5*c['track']
        k=max(1.,abs(u[0])/c['vmax'],abs(u[1])/c['wmax'],abs(u[0]-h*u[1])/c['wheel_v'],abs(u[0]+h*u[1])/c['wheel_v']);u/=k
        n=max(1,int(math.ceil(dt/.005)));step=dt/n
        for _ in range(n):
            a=(u-self.s[3:])/.12;k=max(1.,abs(a[0])/c['accel'],abs(a[1])/c['angular_accel'],abs(a[0]-h*a[1])/c['wheel_a'],abs(a[0]+h*a[1])/c['wheel_a']);a/=k
            new=self.s[3:]+step*a;vm,wm=(new+self.s[3:])/2
            yaw=self.s[2]+.5*step*wm;self.s[:2]+=step*vm*np.array([math.cos(yaw),math.sin(yaw)])
            self.s[2]=math.atan2(math.sin(self.s[2]+step*wm),math.cos(self.s[2]+step*wm));self.s[3:]=new

    def publish_state(self):
        stamp=rospy.Time.from_sec(1000+self.t);self.clock_pub.publish(Clock(stamp))
        o=Odometry();o.header=Header(stamp=stamp,frame_id='odom');o.child_frame_id='base_link'
        o.pose.pose.position.x,o.pose.pose.position.y=self.s[:2];o.pose.pose.orientation.z=math.sin(self.s[2]/2);o.pose.pose.orientation.w=math.cos(self.s[2]/2)
        o.twist.twist.linear.x,o.twist.twist.angular.z=self.s[3:];self.odom_pub.publish(o)
        tf=TransformStamped();tf.header=o.header;tf.child_frame_id='base_link';tf.transform.translation.x,tf.transform.translation.y=self.s[:2];tf.transform.rotation=o.pose.pose.orientation;self.tf.sendTransform(tf)
        j=JointState();j.header=Header(stamp=stamp);j.name=['joint'+str(i+1) for i in range(6)];j.position=self.q.tolist();j.velocity=[0.]*6;j.effort=[0.]*6;self.joint_pub.publish(j);self.gripper_pub.publish(Bool(False))
        return stamp

    def cloud(self,stamp):
        points=[self.static]
        for x,y,r in self.obstacles():
            points.append(np.array([(x+r*math.cos(a),y+r*math.sin(a),z) for a in np.arange(0,2*math.pi,.12) for z in np.arange(.15,1.41,.1)]))
        world=np.concatenate(points);delta=world-np.array([self.s[0],self.s[1],0.]);c,s=math.cos(self.s[2]),math.sin(self.s[2]);rot=np.array([[c,s,0],[-s,c,0],[0,0,1]])
        local=delta@rot.T-self.offset;local=local[np.linalg.norm(local[:,:2],axis=1)<5.]
        self.raw_pub.publish(pc2.create_cloud_xyz32(Header(stamp=stamp,frame_id='livox_frame'),local))

    def record(self):
        c,s=math.cos(self.s[2]),math.sin(self.s[2]);centers=self.body[:,:2]@np.array([[c,s],[-s,c]])+self.s[:2]
        distances=[2-abs(p[1])-b[2] for p,b in zip(centers,self.body)]
        distances += [min(p[0]+1.5,4.5-p[0])-b[2] for p,b in zip(centers,self.body)]
        for ox,oy,r in self.obstacles():
            distances.extend(np.linalg.norm(centers-np.array([ox,oy]),axis=1)-self.body[:,2]-r)
            self.obstacle_rows.append([self.t,ox,oy,r])
        clearance=float(min(distances));self.min_clearance=min(self.min_clearance,clearance);self.collided|=clearance<0
        self.rows.append([self.t,*self.s,*self.command,self.limit,clearance,self.traj_count,self.dynamic_n,int(self.health),*self.progress,self.arm_error])

    def run(self):
        start=time.monotonic();last=start;next_cloud=0.;next_map=0.;settled=0.
        reason='shutdown'
        while not rospy.is_shutdown():
            now=time.monotonic();self.t=now-start;dt=now-last;last=now
            if dt>.25:self.events.append({'t':self.t,'kind':'plant_loop_gap','dt':dt})
            self.integrate(dt);stamp=self.publish_state()
            if self.motion_time is None and self.goal_time is not None and self.s[0]>=.10 and self.s[3]>=.05:
                self.motion_time=self.t;self.events.append({'t':self.t,'kind':'motion_anchor','x':float(self.s[0]),'v':float(self.s[3])})
            if self.t>=next_cloud:self.cloud(stamp);next_cloud=self.t+.1
            # A static PCD is a latched immutable map, not a new map every scan.
            # Changing its stamp resets the native detector's tracks/health gate.
            if self.t>=next_map:self.map_pub.publish(pc2.create_cloud_xyz32(Header(stamp=stamp,frame_id='world'),self.static));next_map=float('inf')
            if self.goal_time is None and self.t>7 and self.trigger and self.goal_pub.get_num_connections():
                goal=PoseStamped();goal.header=Header(stamp=stamp,frame_id='world');goal.pose.position.x=3.;goal.pose.orientation.w=1.;self.goal_pub.publish(goal);self.goal_time=self.t;self.events.append({'t':self.t,'kind':'goal'})
            self.record()
            at_goal=np.linalg.norm(self.s[:2]-[3.,0.])<self.limits['goal_pos'] and abs(self.s[2])<self.limits['goal_yaw'] and abs(self.s[3])<self.limits['goal_speed']
            settled=settled+dt if at_goal else 0.
            if settled>1:reason='goal';break
            if self.arm_error>1e-5:reason='invalid_fixed_arm';break
            if self.t>(self.goal_time+self.duration if self.goal_time is not None else 20):reason='timeout';break
            time.sleep(max(0.,.02-(time.monotonic()-now)))
        for filename,header,rows in [('state.csv','t,x,y,yaw,v,w,cmd_v,cmd_w,safety_limit,clearance,trajectory_count,dynamic_points,perception_healthy,trajectory_start,virtual_time,time_scale,arm_error',self.rows),('obstacles.csv','t,x,y,r',self.obstacle_rows)]:
            with (self.out/filename).open('w') as f:w=csv.writer(f);w.writerow(header.split(','));w.writerows(rows)
        for name,value in [('events',self.events),('trajectories',self.raw_trajs),('mppi_diagnostics',self.diagnostics)]:
            (self.out/(name+'.json')).write_text(json.dumps(value,indent=2)+'\n')
        result={'reason':reason,'goal_sent':self.goal_time,'motion_anchor':self.motion_time,'scenario_start':self.scenario_start,'trajectory_bundles':self.traj_count,'min_clearance':self.min_clearance,'collision_conservative_projection':self.collided,'arm_max_error':self.arm_error,'final_state':self.s.tolist(),'elapsed':self.t,'scene':self.scene,'scope':'native ROS algorithm SIL, not Gazebo/real vehicle'}
        (self.out/'result.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':
    isolation();rospy.init_node('algorithm_simulator')
    if not rospy.get_param('/online_validation/enabled',False) or not rospy.get_param('/use_sim_time',False):raise RuntimeError('missing simulation interlock')
    Simulator().run()
