#!/usr/bin/env python3
"""Experimental MPPI/CBF consumer of actual native REMANI trajectories."""
import ctypes as ct
import json
import math
import threading
import time
from pathlib import Path
import numpy as np
from scipy.ndimage import distance_transform_edt
import rospy
import tf2_ros
from geometry_msgs.msg import Twist,Vector3Stamped,PoseStamped
from nav_msgs.msg import Odometry,Path as RosPath
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2
from std_msgs.msg import Bool,Float64,String
from quadrotor_msgs.msg import PolynomialTraj
from simulator import isolation,polynomial
from progress import project
from native_reference import NativeReference,sampled_reference,fingerprint

DP=ct.POINTER(ct.c_double)
def ptr(a):return a.ctypes.data_as(DP)

def execution_gate(result,reason,limit,bundle_changed,shadow_only):
    """Final command authority: shadow candidates can NEVER reach the plant."""
    if 0<=limit<=1e-3:result[:2]=0;reason='native_safety_stop'
    elif limit>0:result[0]=np.clip(result[0],-limit,limit)
    if bundle_changed:result[:2]=0;reason='trajectory_changed'
    if shadow_only:result[:2]=0;reason='native_abort_shadow'
    return reason


class Node:
    def __init__(self):
        out=Path(rospy.get_param('~out'));self.out=out;self.saved_failures=set();self.failure_streak=0;self.stall_streak=0;self.mode=rospy.get_param('~mode')
        limits=json.loads((out/'limits.json').read_text());p=np.array(list(limits.values()),dtype=np.float64)
        body=np.ascontiguousarray(np.loadtxt(str(out/'geometry.csv'),delimiter=','),dtype=np.float64)
        self.lib=ct.CDLL(rospy.get_param('~library'));self.lib.online_create.argtypes=[DP,DP,ct.c_int,ct.c_int];self.lib.online_create.restype=ct.c_void_p
        self.lib.online_step.argtypes=[ct.c_void_p,DP,DP,ct.c_int,DP,ct.c_int,ct.c_double,DP,ct.c_int,ct.c_int,ct.c_double,ct.c_double,ct.c_double,ct.c_int,DP];self.lib.online_step.restype=ct.c_int
        self.lib.online_feedback.argtypes=[ct.c_void_p,ct.c_double,ct.c_double];self.lib.online_destroy.argtypes=[ct.c_void_p]
        self.handle=self.lib.online_create(ptr(p),ptr(body),len(body),1)
        if not self.handle:raise RuntimeError('MPPI kernel initialization failed')
        self.v2=hasattr(self.lib,'online_debug');self.kernel_bundle=-1
        self.v3=hasattr(self.lib,'online_status')
        self.v4=hasattr(self.lib,'online_local_route')
        self.v6=hasattr(self.lib,'online_avoidance_debug')
        if self.v6:self.lib.online_avoidance_debug.argtypes=[ct.c_void_p,DP]
        self.v7=hasattr(self.lib,'online_decision_debug')
        if self.v7:self.lib.online_decision_debug.argtypes=[ct.c_void_p,DP]
        self.v8=hasattr(self.lib,'online_recovery_debug')
        if self.v8:self.lib.online_recovery_debug.argtypes=[ct.c_void_p,DP]
        if self.v4:
            self.lib.online_local_route.argtypes=[ct.c_void_p,DP,ct.c_int];self.lib.online_local_route.restype=ct.c_int
        self.shadow_refs=[];self.shadow_times=[];self.shadow_start=0.
        if self.v3:self.lib.online_status.argtypes=[ct.c_void_p,DP]
        if self.v2:
            if self.mode!='mppi':raise RuntimeError('MPPI v2 is an MPPI-only experiment')
            self.lib.online_reset.argtypes=[ct.c_void_p]
            self.lib.online_debug.argtypes=[ct.c_void_p,DP]
        self.lock=threading.Lock();self.state=None;self.odom_stamp=0.;self.refs=[];self.ref_changed=0.;self.ref_start=0.;self.bundle=0
        self.native=NativeReference();self.revision=0;self.last_logged_reference=None
        self.reference_log=(out/'consumed_references.jsonl').open('w',buffering=1)
        self.reference_events=(out/'reference_events.jsonl').open('w',buffering=1)
        self.times=[];self.projection=rospy.get_param('~projection_progress',False);self.projected_time=0.;self.progress_bundle=-1
        self.safety=-1.;self.healthy=False;self.static=np.empty((0,3));self.cloud_stamp=0.;self.obs_stamp=0.;self.obs=np.empty((0,6))
        self.gx=-7.5;self.gy=-5.;self.res=.05;self.nx=300;self.ny=200;self.field=None
        self.tf=tf2_ros.Buffer();self.listener=tf2_ros.TransformListener(self.tf)
        self.pub=rospy.Publisher('/sim/cmd_vel',Twist,queue_size=1)
        self.diag=rospy.Publisher('/sim/mppi_diagnostics',String,queue_size=100)
        self.reference_pub=rospy.Publisher('/sim/mppi/native_reference',RosPath,queue_size=1,latch=True)
        self.fault=rospy.Publisher('/tracking_fault',Bool,queue_size=1,latch=True)
        self.progress_pub=rospy.Publisher('/mm_controller_node/trajectory_progress',Vector3Stamped,queue_size=1) if self.projection else None
        rospy.Subscriber('/odom',Odometry,self.odom,queue_size=1)
        rospy.Subscriber('/planning/trajectory',PolynomialTraj,self.trajectory,queue_size=20)
        rospy.Subscriber('/planning/safety_speed_limit',Float64,lambda m:setattr(self,'safety',m.data),queue_size=1)
        rospy.Subscriber('/dynamic_obstacle_tracker/perception_healthy',Bool,lambda m:setattr(self,'healthy',m.data),queue_size=1)
        rospy.Subscriber('/map_generator/global_cloud',PointCloud2,self.map,queue_size=1)
        rospy.Subscriber('/velodyne_points',PointCloud2,self.cloud,queue_size=1)
        rospy.Subscriber('/dynamic_points',PointCloud2,self.dynamic,queue_size=1)

    def odom(self,m):
        q=m.pose.pose.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        with self.lock:self.state=np.array([m.pose.pose.position.x,m.pose.pose.position.y,yaw,m.twist.twist.linear.x,m.twist.twist.angular.z]);self.odom_stamp=m.header.stamp.to_sec()

    def trajectory(self,m):
        with self.lock:
            event=self.native.feed(m,rospy.get_time())
            if event=='notice':return
            if event in ('cancel','invalid'):
                if self.v4 and self.refs:
                    self.shadow_refs=[r.copy() for r in self.refs];self.shadow_times=[r.copy() for r in self.times];self.shadow_start=self.ref_start
                self.reference_pub.publish(RosPath())
            else:self.shadow_refs=[];self.shadow_times=[]
            self.refs=self.native.refs;self.times=self.native.times;self.ref_start=self.native.start
            # Append also invalidates the current kernel result and local guide.
            self.bundle=self.native.revision;self.revision=self.native.revision
            self.ref_changed=self.native.changed
            self.reference_events.write(json.dumps({'t':rospy.get_time()-1000,'event':event,'revision':self.revision,'epoch':self.native.epoch,'id':m.trajectory_id,'stamp':m.header.stamp.to_sec(),'error':self.native.error,'message_hashes':self.native.messages})+'\n')

    def map(self,m):
        self.static=np.array(list(pc2.read_points(m,field_names=['x','y','z'],skip_nans=True)),dtype=np.float64).reshape((-1,3))

    def cloud(self,m):
        try:
            tf=self.tf.lookup_transform('world',m.header.frame_id,m.header.stamp,rospy.Duration(.025)).transform
            a=np.array(list(pc2.read_points(m,field_names=['x','y','z'],skip_nans=True)),dtype=np.float64).reshape((-1,3))
            q=tf.rotation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));c,s=math.cos(yaw),math.sin(yaw)
            a=a@np.array([[c,s,0],[-s,c,0],[0,0,1]])+np.array([tf.translation.x,tf.translation.y,tf.translation.z])
            a=np.concatenate([self.static,a]);a=a[(a[:,2]>.08)&(a[:,2]<1.6)]
            ix=np.floor((a[:,0]-self.gx)/self.res).astype(int);iy=np.floor((a[:,1]-self.gy)/self.res).astype(int)
            mask=(ix>=0)&(iy>=0)&(ix<self.nx)&(iy<self.ny);grid=np.ones((self.ny,self.nx),dtype=bool);grid[iy[mask],ix[mask]]=False
            field=np.maximum(0.,distance_transform_edt(grid)*self.res-math.sqrt(2)*self.res)
            with self.lock:self.field=np.ascontiguousarray(field);self.cloud_stamp=m.header.stamp.to_sec()
        except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):pass

    def dynamic(self,m):
        if m.header.frame_id!='world':return
        a=np.array(list(pc2.read_points(m,field_names=['x','y','vx','vy','track_id'],skip_nans=True)),dtype=np.float64).reshape((-1,5))
        obs=[]
        for track in np.unique(a[:,4]):
            points=a[a[:,4]==track];center=(points[:,:2].min(axis=0)+points[:,:2].max(axis=0))/2
            radius=np.linalg.norm(points[:,:2]-center,axis=1).max()+.025;velocity=np.median(points[:,2:4],axis=0)
            obs.append([*center,*velocity,radius,1.])
        with self.lock:self.obs=np.ascontiguousarray(obs,dtype=np.float64).reshape((-1,6));self.obs_stamp=m.header.stamp.to_sec()

    def run(self):
        rate=rospy.Rate(20)
        while not rospy.is_shutdown():
            now=rospy.get_time();wall=time.monotonic()
            with self.lock:
                state=None if self.state is None else self.state.copy();refs=[r.copy() for r in self.refs];times=[t.copy() for t in self.times];field=self.field;obs=self.obs.copy();stamp=self.obs_stamp;cloud_stamp=self.cloud_stamp;odom_stamp=self.odom_stamp;bundle=self.bundle;ref_start=self.ref_start;changed=self.ref_changed
                message_hashes=list(self.native.messages);reference_error=self.native.error;epoch=self.native.epoch
                shadowing=self.v4 and not refs and bool(self.shadow_refs)
                if shadowing:refs=[r.copy() for r in self.shadow_refs];times=[t.copy() for t in self.shadow_times];ref_start=self.shadow_start
            local_route=[];local_error=None
            if self.v4 and bundle==self.kernel_bundle:
                points=np.zeros((1024,2));count=self.lib.online_local_route(self.handle,ptr(points),len(points));points=points[:count]
                if len(points)>1 and state is not None:
                    edge=np.diff(points,axis=0);delta=state[:2]-points[:-1]
                    fraction=np.clip(np.sum(edge*delta,axis=1)/np.maximum(1e-12,np.sum(edge*edge,axis=1)),0,1)
                    local_error=float(np.linalg.norm(delta-fraction[:,None]*edge,axis=1).min())
                    local_route=points[::max(1,len(points)//40)].tolist()
            result=np.zeros(8);cost_debug=None;phase_debug=None;reason='ok';code=-1;refs_count=0;projection_error=0.;reference_hash=None
            if self.progress_pub is not None:
                progress=Vector3Stamped();progress.header.stamp=rospy.Time.now();progress.header.frame_id='measured_path_projection'
                if epoch!=self.progress_bundle:self.projected_time=0.;self.progress_bundle=epoch
                if state is not None and refs and not shadowing and now>=ref_start and now-odom_stamp<=.2:
                    self.projected_time,scale,projection_error=project(state,np.concatenate(refs),np.concatenate(times),self.projected_time)
                    progress.vector.x=ref_start;progress.vector.y=self.projected_time;progress.vector.z=scale
                self.progress_pub.publish(progress)
            if state is None or now-odom_stamp>.2:reason='odom_stale'
            elif reference_error:reason='invalid_native_trajectory'
            elif not refs or now<ref_start or now-changed<.05:reason='waiting_native_trajectory'
            elif field is None or now-cloud_stamp>.35 or not self.healthy:reason='perception_unhealthy'
            elif now-stamp>.35:reason='dynamic_stale'
            elif (local_error if local_error is not None else projection_error)>.6:reason='projection_lost'
            else:
                reference=sampled_reference(refs,state[2]);refs_count=len(reference)
                # Native polynomial endpoints have zero derivative. Fill heading
                # from adjacent nonzero derivative, never from obstacle truth.
                reference_hash=fingerprint(reference)
                if self.v2 and bundle!=self.kernel_bundle:
                    self.lib.online_reset(self.handle);self.kernel_bundle=bundle
                code=self.lib.online_step(self.handle,ptr(state),ptr(reference),len(reference),ptr(obs),len(obs),max(0.,now-stamp),ptr(field),self.nx,self.ny,self.gx,self.gy,self.res,int(self.mode=='mppi_cbf'),ptr(result))
                if self.v2:
                    values=np.zeros(8);self.lib.online_debug(self.handle,ptr(values))
                    cost_debug=dict(zip(('total','path','uncertainty_risk','collision','reverse','selected_index','stop_candidate','reverse_candidate'),values.tolist()))
                if self.v3:
                    values=np.zeros(8);self.lib.online_status(self.handle,ptr(values))
                    phase_debug={'phase':('track','avoid','rejoin','predictive_avoid')[int(values[0])],'side':int(values[1]),'path_error':values[2],'conflict_time':values[3],'clear_cycles':int(values[4]),'guide_points':int(values[5]),'guide_failed':bool(values[6]),'strategy_code':int(values[7])}
                    if hasattr(self.lib,'online_revision'):
                        phase_debug['rejoin_arc']=phase_debug.pop('conflict_time')
                    if self.v6:
                        values=np.zeros(8);self.lib.online_avoidance_debug(self.handle,ptr(values))
                        phase_debug['predictive']=dict(zip(('nominal_min_gap','active','feasible_candidates','best_index','selected_min_gap','selected_progress','low_speed_cost_integral','unverified_brake'),values.tolist()))
                        if values[1]:cost_debug['scope']='V5 proposal only; selected V6 feasibility is in local_phase.predictive'
                    if self.v7:
                        values=np.zeros(12);self.lib.online_decision_debug(self.handle,ptr(values))
                        phase_debug['decision']=dict(zip(('mode','first_conflict_s','response_budget_s','nominal_gap','committed_side','selected_index','lateral_candidates_enabled','path_error','predicted_progress_rate','selected_cost','clear_cycles','max_predicted_lateral'),values.tolist()))
                        phase_debug['decision']['mode_name']=('monitor','speed_adjust','detour','rejoin','unverified_brake')[int(values[0])]
                        cost_debug['scope']='V5 proposal, possibly unused; selected V7 cost is in local_phase.decision'
                    if self.v8:
                        values=np.zeros(10);self.lib.online_recovery_debug(self.handle,ptr(values))
                        phase_debug['recovery']=dict(zip(('expanded','v7_proposal_failed','added_candidates','feasible_candidates','v7_candidate_feasible','best_continuation_gap','feasible_continuations','selected_motion_cost','selected_total_cost','unverified_brake'),values.tolist()))
                        if values[0]:
                            phase_debug['phase']='recovery' if not values[9] else 'unverified_brake'
                            phase_debug['guide_failed']=bool(values[9])
                            phase_debug['predictive']['best_index_scope']='legacy V7 index; use recovery diagnostics for V8'
                        cost_debug['scope']='V5 proposal only; V8 recovery and selected cost are in local_phase'
                if code:result[:2]=0;reason='kernel_error'
            computation_ms=(time.monotonic()-wall)*1000
            stalled=reason=='ok' and state is not None and refs_count>1 and abs(state[3])<.02 and abs(result[0])<.015 and abs(result[1])<.05 and np.linalg.norm(state[:2]-reference[-1,:2])>.35
            self.stall_streak=self.stall_streak+1 if stalled else 0
            if self.stall_streak>=20 and 'local_stall' not in self.saved_failures:
                np.savez_compressed(str(self.out/'local_stall.npz'),state=state,reference=reference,obs=obs,age=max(0.,now-stamp),field=field,result=result,grid=[self.gx,self.gy,self.res],t=now-1000)
                self.saved_failures.add('local_stall')
            self.failure_streak=self.failure_streak+1 if result[5] else 0
            if result[5]:
                label='persistent_qp_failure' if self.failure_streak>=20 else 'first_qp_failure'
                if label not in self.saved_failures:
                    np.savez_compressed(str(self.out/(label+'.npz')),state=state,reference=reference,obs=obs,age=max(0.,now-stamp),field=field,result=result,grid=[self.gx,self.gy,self.res],t=now-1000)
                    self.saved_failures.add(label)
            if computation_ms>50:result[:2]=0;reason='deadline_miss'
            # The planner safety topic is latched and change-only: do not age it.
            limit=self.safety
            # Do not execute a result computed for a cancelled/replaced bundle.
            with self.lock:
                bundle_changed=bundle!=self.bundle
                # Serialize publication with replacement/cancel callbacks.
                reason=execution_gate(result,reason,limit,bundle_changed,shadowing)
                cmd=Twist();cmd.linear.x=result[0];cmd.angular.z=result[1];self.pub.publish(cmd)
            if reference_hash is not None and not shadowing and (bundle,reference_hash)!=self.last_logged_reference:
                self.reference_log.write(json.dumps({'t':now-1000,'revision':bundle,'epoch':epoch,'stamp':ref_start,'source':'/planning/trajectory','message_hashes':message_hashes,'sha256':reference_hash,'reference':reference.tolist(),'times':np.concatenate(times).tolist()})+'\n')
                self.last_logged_reference=(bundle,reference_hash)
                path=RosPath();path.header.stamp=rospy.Time.now();path.header.frame_id='world'
                for x,y,yaw,_ in reference:
                    pose=PoseStamped();pose.header=path.header;pose.pose.position.x=x;pose.pose.position.y=y;pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2);path.poses.append(pose)
                with self.lock:
                    if bundle==self.bundle:self.reference_pub.publish(path)
            self.lib.online_feedback(self.handle,result[0],result[1]);self.fault.publish(Bool(reason in ('odom_stale','perception_unhealthy','dynamic_stale','kernel_error','projection_lost')))
            self.diag.publish(String(json.dumps({'t':now-1000,'reason':reason,'bundle':bundle,'reference_sha256':reference_hash,'reference_source':'/planning/trajectory','reference_error':reference_error,'reference_points':refs_count,'projection_error':projection_error,'local_projection_error':local_error,'local_route':local_route,'shadow_only':shadowing,'projected_time':self.projected_time,'obstacle_tracks':len(obs),'observed_obstacles':obs.tolist(),'cloud_age':now-cloud_stamp,'dynamic_age':now-stamp,'limit':limit,'applied_v':result[0],'applied_w':result[1],'nominal_v':result[2],'nominal_w':result[3],'cbf_intervention':bool(result[4]),'qp_failed':bool(result[5]),'stop_unverified':bool(result[6]),'kernel_ms':result[7],'callback_ms':computation_ms,'code':code,'cost_components':cost_debug,'local_phase':phase_debug})))
            rate.sleep()
        self.lib.online_destroy(self.handle)


if __name__=='__main__':
    isolation();rospy.init_node('base_mppi')
    if not rospy.get_param('/online_validation/enabled',False):raise RuntimeError('missing simulation interlock')
    Node().run()
