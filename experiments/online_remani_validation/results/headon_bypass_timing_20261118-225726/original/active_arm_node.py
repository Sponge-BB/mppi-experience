#!/usr/bin/env python3
"""Isolated whole-body executor consuming actual REMANI transactions and ESDF."""
import io
import json
import math
from pathlib import Path
import threading
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String, UInt8MultiArray

from active_arm_model import PiperArmModel
from active_arm_mppi import Snapshot, WholeBodyMPPI
from active_arm_reference import WholeBodySession
from collab_seed_export import goal_token
from simulator import isolation


class Node:
    def __init__(self):
        self.out=Path(rospy.get_param('~out'))
        self.config=json.loads((self.out/'active_arm_config.json').read_text())
        self.model=PiperArmModel.from_parameters(self.out/'parameters.yaml',self.config['urdf_path'])
        al=self.config['arm_limits']
        self.kernel=WholeBodyMPPI(self.model,json.loads((self.out/'limits.json').read_text()),
            [al[k] for k in ('lower','upper','velocity','acceleration')],
            self.config['postures'],enabled=self.config['enabled'],seed=self.config['seed'],
            horizon=self.config.get('horizon',20),random_samples=self.config.get('random_samples',8),
            coverage_reserve=self.config.get('deadline_s',.20))
        self.lock=threading.Lock();self.session=WholeBodySession()
        self.state=None;self.odom_stamp=0.;self.joint_stamp=0.;self.q=None;self.dq=None
        self.snapshot=None;self.snapshot_error='waiting';self.sequence=0
        self.episode=0;self.episode_active=False;self.clear_count=0
        self.profile=None;self.profile_input=None
        self.first_failure_input=None
        self.first_intervention_input=None
        self.log=(self.out/'active_arm_cycles.jsonl').open('w',buffering=1)
        self.events=(self.out/'active_arm_reference_events.jsonl').open('w',buffering=1)
        self.cmd=rospy.Publisher('/sim/whole_body_command',Float64MultiArray,queue_size=1)
        self.seed=rospy.Publisher('/sim/mppi_seed_states',Float64MultiArray,queue_size=1)
        self.refresh=rospy.Publisher('/planning/local_refresh_request',Float64MultiArray,queue_size=1)
        rospy.Subscriber('/odom',Odometry,self.odom,queue_size=1)
        rospy.Subscriber('/mm/joint_states',JointState,self.joints,queue_size=1)
        rospy.Subscriber('/sim/native_collision_3d',UInt8MultiArray,self.collision,queue_size=1,buff_size=32*1024**2)
        rospy.Subscriber('/move_base_simple/goal',PoseStamped,self.goal,queue_size=1)
        rospy.Subscriber('/planning/execution_state',String,self.execution_state,queue_size=5)
        rospy.Subscriber('/planning/execution_reference',String,self.reference,queue_size=5)

    def odom(self,msg):
        q=msg.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        v=np.array([msg.pose.pose.position.x,msg.pose.pose.position.y,yaw,
                    msg.twist.twist.linear.x,msg.twist.twist.angular.z])
        with self.lock:self.state=v;self.odom_stamp=msg.header.stamp.to_sec()

    def joints(self,msg):
        try:
            ids=[msg.name.index('joint'+str(i)) for i in range(1,7)]
            q=np.array([msg.position[i] for i in ids]);dq=np.array([msg.velocity[i] for i in ids])
            if not np.isfinite(np.r_[q,dq]).all():return
        except (ValueError,IndexError):return
        with self.lock:self.q=q;self.dq=dq;self.joint_stamp=msg.header.stamp.to_sec()

    def collision(self,msg):
        try:
            wire=np.frombuffer(bytes(msg.data),dtype='<f8')
            snapshot=Snapshot.decode(wire,self.snapshot)
            with self.lock:self.snapshot=snapshot;self.snapshot_error=None
        except (ValueError,TypeError) as exc:
            with self.lock:self.snapshot_error=str(exc)

    def goal(self,msg):
        with self.lock:self.session.goal(msg);self.kernel.previous=None

    def execution_state(self,msg):
        with self.lock:self.session.state(msg.data)

    def reference(self,msg):
        with self.lock:
            try:event=self.session.transaction(msg.data,rospy.get_time())
            except Exception as exc:
                event='invalid';self.session.guide=None;self.session.invalid=str(exc)
            self.events.write(json.dumps({'t':rospy.get_time()-1000,'event':event,
                'revision':self.session.revision,'generation':self.session.generation,
                'reason':self.session.invalid,
                'hashes':self.session.guide.hashes if self.session.guide else []})+'\n')

    def capture(self):
        with self.lock:
            state=None if self.state is None or self.q is None else np.r_[self.state,self.q,self.dq]
            return (state,self.odom_stamp,self.joint_stamp,self.snapshot,self.session.guide,
                    self.session.generation,self.session.revision,self.session.authority(),self.session.goal_key)

    def feedback(self,prediction,controls,stamp,odom,joint,snapshot,generation,revision,key,info,base_reference):
        rows=[]
        for i,z in enumerate(prediction):
            u=controls[min(i,len(controls)-1)]
            rows.append([i*self.kernel.dt,*z[:5],u[0],u[1],*z[5:11],*z[11:17]])
        # v2: 11 header fields, then twenty-column checked full-state rows.
        h=[2.,stamp,float(generation),float(revision),float(goal_token(key or '')),
           odom,float(snapshot.header[18]),float(snapshot.header[18]),20.,float(len(rows)),joint]
        self.seed.publish(Float64MultiArray(data=h+np.asarray(rows).ravel().tolist()))
        nominal=self.kernel.guide(prediction[0],base_reference)
        moving_away=bool(info.get('intervention')) and (info.get('arm_motion',0.)>.04 or
                     np.linalg.norm(controls[0,:2]-nominal)>.045)
        if moving_away:
            if not self.episode_active:self.episode+=1
            self.episode_active=True;self.clear_count=0
        elif self.episode_active:
            self.clear_count+=1
            if self.clear_count>=4:self.episode_active=False
        self.refresh.publish(Float64MultiArray(data=[1.,stamp,float(generation),float(revision),
            float(goal_token(key or '')),float(max(1,self.episode)),float(moving_away),1.,info['min_gap']]))

    def run(self):
        hz=self.config.get('rate_hz',10.);deadline=self.config.get('deadline_s',.20)
        rate=rospy.Rate(hz)
        try:
            while not rospy.is_shutdown():
                begin=time.monotonic();now=rospy.get_time();self.sequence+=1
                state,odom,joint,snapshot,guide,generation,revision,reason,key=self.capture()
                info={};selected=None;prediction=None;phase=None;published=None;base_reference=None
                if reason=='ok':
                    if state is None or not -.05<=now-odom<=.2:reason='odom_stale'
                    elif not -.05<=now-joint<=.2:reason='joint_stale'
                    elif snapshot is None or not snapshot.fresh(now):reason='collision_snapshot_stale'
                    elif not self.model.check(state[5:11],state[11:17])['valid']:reason='configuration_invalid'
                    else:
                        phase=guide.associate(state)
                        # One whole-body association chooses the executable
                        # local portion; base projections cannot jump to an
                        # earlier crossing/gear branch elsewhere in the curve.
                        first=max(0,phase-2);last=len(guide.rows)
                        gear=-1 if guide.rows[phase,4]<-.01 else 1
                        for j in range(phase+1,len(guide.rows)):
                            if guide.rows[j,4]*gear<-.01:last=j;break
                        base_reference=guide.base[first:last]
                        arm_reference=guide.rows[min(len(guide.rows)-1,phase+2),5:11]
                        profile_this=bool(self.config.get('profile')) and self.profile is None and state[0]>.65
                        if profile_this:
                            import cProfile
                            self.profile=cProfile.Profile();self.profile.enable()
                            self.profile_input=(state.copy(),base_reference.copy(),arm_reference.copy(),snapshot,now)
                        planned,info=self.kernel.plan(state,base_reference,arm_reference,snapshot,now)
                        if info.get('intervention') and self.first_intervention_input is None:
                            self.first_intervention_input=(state.copy(),base_reference.copy(),
                                arm_reference.copy(),snapshot,now,'first_intervention')
                        if profile_this:self.profile.disable()
                        if planned is None:
                            reason=info.get('failure_reason','no_feasible_whole_body_candidate')
                            # Keep one exact input in memory, write it after the
                            # episode so compression cannot pause the controller.
                            if self.first_failure_input is None:
                                self.first_failure_input=(state.copy(),base_reference.copy(),
                                    arm_reference.copy(),snapshot,now,reason)
                        else:selected,prediction=planned
                if selected is not None:
                    latest,odom2,joint2,snap2,guide2,gen2,rev2,auth2,key2=self.capture()
                    published=rospy.get_time()
                    if (gen2,rev2)!=(generation,revision) or auth2!='ok':reason='reference_changed'
                    elif not -.05<=published-odom2<=.2 or not -.05<=published-joint2<=.2:reason='publication_state_stale'
                    elif snap2 is None or not snap2.fresh(published):reason='publication_collision_stale'
                    else:
                        _,good,gap,checked=self.kernel.evaluate(latest,selected[None],base_reference,arm_reference,snap2,published)
                        lease_ok,lease_gap=self.kernel.lease_brake(latest,selected,snap2,published)
                        if not good[0]:reason='latest_rollout_rejected'
                        elif not lease_ok:reason='lease_braking_rejected'
                        elif time.monotonic()-begin>deadline:reason='deadline_missed'
                        else:
                            state,odom,joint,snapshot=latest,odom2,joint2,snap2
                            prediction=checked[0];info['min_gap']=float(gap[0])
                            info['lease_brake_gap']=lease_gap
                stamp=rospy.get_time()
                if published is not None and reason=='ok' and stamp-published>.065:reason='validation_to_command_delay'
                with self.lock:
                    if reason=='ok' and ((self.session.generation,self.session.revision)!=(generation,revision) or self.session.authority()!='ok'):
                        reason='authority_changed_after_validation'
                    if reason!='ok' and self.state is not None and self.q is not None:
                        stop_state=np.r_[self.state,self.q,self.dq];stop_snapshot=self.snapshot
                        if stop_snapshot is not None and stop_snapshot.fresh(stamp):
                            stopped,stop_gap=self.kernel.lease_brake(stop_state,None,stop_snapshot,stamp,lease=0.)
                            info['immediate_stop_valid']=stopped;info['immediate_stop_gap']=stop_gap
                        else:info['immediate_stop_valid']=False
                    command=selected[0] if selected is not None and reason=='ok' else np.zeros(8)
                    # Final authorization and atomic command publication share
                    # a lock with cancel/reference callbacks.
                    self.cmd.publish(Float64MultiArray(data=[1.,stamp,.25,float(self.sequence),*command.tolist()]))
                    if reason=='ok':
                        self.feedback(prediction,selected,published,odom,joint,snapshot,generation,revision,key,info,base_reference)
                elapsed=time.monotonic()-begin
                row={'t':stamp-1000,'cycle':self.sequence,'reason':reason,'callback_ms':elapsed*1000,
                     'generation':generation,'revision':revision,'phase_index':phase,
                     'odom_stamp':odom,'joint_stamp':joint,'command':command.tolist(),
                     'state':state.tolist() if state is not None else None,**info}
                row['rollout_epoch']=published
                row['validation_to_command_ms']=1000*(stamp-published) if published is not None else None
                row['snapshot_header']=snapshot.header.tolist() if snapshot is not None else None
                if prediction is not None and reason=='ok':
                    row['rollout']=prediction[::2].tolist()
                self.log.write(json.dumps(row,allow_nan=False)+'\n')
                rate.sleep()
        finally:
            self.log.close();self.events.close()
            for filename,record in [('first_failure_input',self.first_failure_input),
                                    ('first_intervention_input',self.first_intervention_input)]:
                if record is None:continue
                state,reference,arm,snapshot,epoch,reason=record
                values=dict(state=state,reference=reference,arm_reference=arm,header=snapshot.header,
                    field=snapshot.distance,epoch=np.array(epoch),reason=np.array(reason),
                    velocities=np.asarray([v for v,t in snapshot.groups]).reshape((-1,3)))
                for i,(_,tree) in enumerate(snapshot.groups):values['dynamic_'+str(i)]=tree.data
                np.savez_compressed(self.out/(filename+'.npz'),**values)
            if self.profile is not None:
                import pstats
                with (self.out/'controller_profile.txt').open('w') as stream:
                    pstats.Stats(self.profile,stream=stream).sort_stats('cumtime').print_stats(35)
                state,reference,arm,snapshot,epoch=self.profile_input
                values=dict(state=state,reference=reference,arm_reference=arm,header=snapshot.header,
                            field=snapshot.distance,epoch=np.array(epoch),
                            velocities=np.asarray([v for v,t in snapshot.groups]).reshape((-1,3)))
                for i,(_,tree) in enumerate(snapshot.groups):values['dynamic_'+str(i)]=tree.data
                np.savez_compressed(self.out/'controller_input.npz',**values)


if __name__=='__main__':
    isolation();rospy.init_node('active_arm_mppi')
    if not rospy.get_param('/online_validation/enabled',False) or not rospy.get_param('/use_sim_time',False):
        raise RuntimeError('missing numerical simulation interlock')
    Node().run()
