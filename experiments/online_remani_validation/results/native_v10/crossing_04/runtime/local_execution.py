"""Goal-scoped native guides and single fast local execution authority (SIL)."""
import ctypes as ct
import io
import json
import math
import time
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, Vector3Stamped
from nav_msgs.msg import Path as RosPath
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from quadrotor_msgs.msg import PolynomialTraj
from native_reference import NativeReference, sampled_reference, fingerprint
from progress import project

DP=ct.POINTER(ct.c_double)
def ptr(a):return a.ctypes.data_as(DP)
def message_key(m):
    stream=io.BytesIO();m.serialize(stream);return stream.getvalue().hex()


class ExecutionSession:
    """A task key is carried INSIDE every complete reference transaction."""
    def __init__(self):
        self.goal_key=None;self.generation=0;self.states={};self.enabled=False;self.hard_stop=True
        self.guide=None;self.invalid=None;self.guide_cancelled=False;self.revision=0
        self.hard_cancel=False

    def goal(self,m):
        self.goal_key=message_key(m);self.generation+=1
        self.guide=None;self.invalid=None;self.guide_cancelled=False
        self.hard_cancel=False
        self.enabled,self.hard_stop=self.states.get(self.goal_key,(False,True))

    def state(self,text):
        parts=text.split('\n')
        if len(parts)!=3 or parts[1] not in ('0','1') or parts[2] not in ('0','1'):
            self.enabled=False;self.hard_stop=True;self.invalid='invalid_execution_state';return
        key=parts[0];value=(parts[1]=='1',parts[2]=='1');self.states[key]=value
        if len(self.states)>4:self.states.pop(next(iter(self.states)))
        if key==self.goal_key:self.enabled,self.hard_stop=value

    def transaction(self,text,now):
        # Parse all segments before committing any of them. No quiet-period
        # guess and no movement on a partially received native trajectory.
        parts=text.split('\n')
        if not parts or parts[0]!=self.goal_key:return 'other_goal',None
        try:
            count=int(parts[1])
            if not 1<=count<=1000 or len(parts)!=count+2 or len(text)>4000000:
                raise ValueError('invalid complete reference transaction')
            messages=[PolynomialTraj().deserialize(bytes.fromhex(s)) for s in parts[2:]]
            if count==1 and messages[0].action in (2,5):
                self.guide_cancelled=True;self.hard_cancel=True;self.revision+=1
                return 'cancel',messages
            native=NativeReference()
            for i,m in enumerate(messages):
                event=native.feed(m,now)
                if event!=('replace' if i==0 else 'append'):
                    raise ValueError(native.error or 'unexpected reference action')
            self.revision+=1;native.revision=self.revision
            native.epoch=self.guide.epoch+1 if self.guide else 1
            self.guide=native;self.invalid=None;self.guide_cancelled=False
            self.hard_cancel=False
            return 'replace',messages
        except Exception as exc:
            # ROS deserialization can raise struct.error or SerializationError.
            self.guide=None;self.invalid='invalid_native_transaction: '+str(exc)
            return 'invalid',None

    def authority(self):
        if self.invalid:return self.invalid
        if self.hard_cancel:return 'native_abort'
        if self.hard_stop:return 'native_hard_stop'
        if not self.enabled:return 'no_active_task'
        if self.guide is None:return 'waiting_native_trajectory'
        return 'ok'


def configure(node):
    lib=node.lib
    if not hasattr(lib,'online_validate_execution') or not node.classified:
        raise RuntimeError('local execution requires V10 kernel and atomic classified perception')
    lib.online_selected_controls.argtypes=[ct.c_void_p,DP,ct.c_int];lib.online_selected_controls.restype=ct.c_int
    lib.online_reference_changed.argtypes=[ct.c_void_p]
    lib.online_validate_execution.argtypes=[ct.c_void_p,DP,DP,ct.c_int,DP,ct.c_int,ct.c_double,ct.c_double,
        DP,ct.c_int,ct.c_int,ct.c_double,ct.c_double,ct.c_double,DP,ct.c_int,DP]
    lib.online_validate_execution.restype=ct.c_int
    node.execution=ExecutionSession();node.execution_log=(node.out/'execution_rollouts.jsonl').open('w',buffering=1)
    node.reference_delay=float(rospy.get_param('~reference_delay',0.));node.pending_execution_transactions=[]
    node.execution_pub=rospy.Publisher('/planning/local_execution_rollout',RosPath,queue_size=1)
    node.local_cmd_pub=rospy.Publisher('/sim/local_execution_cmd',TwistStamped,queue_size=1)
    node.arm_stamp=0.;node.arm_error=float('inf')
    node.expected_arm=np.deg2rad(json.loads((node.out/'geometry.json').read_text())['arm_q_deg'])
    def goal(m):
        with node.lock:node.execution.goal(m)
    def state(m):
        with node.lock:node.execution.state(m.data)
    def commit(text):
            event,messages=node.execution.transaction(text,rospy.get_time())
            if event=='other_goal':return
            guide=node.execution.guide
            if guide is not None and event=='replace':
                node.native=guide;node.refs=guide.refs;node.times=guide.times;node.ref_start=guide.start
                node.bundle=guide.revision;node.ref_changed=guide.changed
            elif event=='invalid':node.refs=[]
            node.reference_events.write(json.dumps({'t':rospy.get_time()-1000,'event':event,
                'revision':node.execution.revision,'epoch':guide.epoch if guide else 0,
                'goal_key':node.execution.goal_key,'id':1 if event=='replace' else 0,
                'stamp':guide.start if guide else 0,'error':node.execution.invalid,
                'message_hashes':guide.messages if guide else []})+'\n')
    node.commit_execution_transaction=commit
    def transaction(m):
        with node.lock:
            try:first=PolynomialTraj().deserialize(bytes.fromhex(m.data.split('\n')[2]))
            except Exception:commit(m.data);return
            if node.reference_delay>0 and node.execution.guide is not None and first.action==1:
                node.pending_execution_transactions.append((rospy.get_time()+node.reference_delay,m.data))
            else:commit(m.data)
    def arm(m):
        try:
            values=np.array([m.position[m.name.index('joint'+str(i))] for i in range(1,7)])
            error=float(np.max(np.abs(values-node.expected_arm)))
        except (ValueError,IndexError):error=float('inf')
        with node.lock:node.arm_stamp=m.header.stamp.to_sec();node.arm_error=error
    rospy.Subscriber('/move_base_simple/goal',PoseStamped,goal,queue_size=1)
    rospy.Subscriber('/planning/execution_state',String,state,queue_size=10)
    rospy.Subscriber('/planning/execution_reference',String,transaction,queue_size=10)
    rospy.Subscriber('/mm/joint_states',JointState,arm,queue_size=1)


def run(node):
    rate=rospy.Rate(20);last_reference=None;kernel_task=-1;kernel_revision=-1
    while not rospy.is_shutdown():
        wall=time.monotonic();now=rospy.get_time();result=np.zeros(8);validation=np.zeros(6)
        checked=np.zeros((650,8));validated_count=0;controls=np.empty((0,2));code=-1;validation_code=-1
        reference=None;reference_hash=None
        with node.lock:
            ready=[item for item in node.pending_execution_transactions if item[0]<=now]
            node.pending_execution_transactions=[item for item in node.pending_execution_transactions if item[0]>now]
            for due,text in ready:node.commit_execution_transaction(text)
            session=node.execution;generation=session.generation;guide=session.guide
            reason=session.authority();revision=session.revision;cancelled=session.guide_cancelled
            state=None if node.state is None else node.state.copy();field=node.field;obs=node.obs.copy()
            cloud_stamp=node.cloud_stamp;stamp=node.obs_stamp;odom_stamp=node.odom_stamp
            healthy=node.healthy;arm_stamp=node.arm_stamp;arm_error=node.arm_error
            classified_error=node.classified_error;classification=dict(node.classification)
        if reason=='ok':
            if state is None or not -.05<=now-odom_stamp<=.2:reason='odom_stale'
            elif not healthy or field is None or not -.05<=now-cloud_stamp<=.35:reason='perception_unhealthy'
            elif not -.05<=now-stamp<=.35:reason='dynamic_stale'
            elif not -.05<=now-arm_stamp<=.2 or arm_error>.05:reason='fixed_arm_invalid'
            else:
                reference=sampled_reference(guide.refs,state[2]);reference_hash=fingerprint(reference)
                if kernel_task!=generation:
                    node.lib.online_reset(node.handle);kernel_task=generation;kernel_revision=-1
                if kernel_revision!=guide.revision:
                    node.lib.online_reference_changed(node.handle);kernel_revision=guide.revision
                code=node.lib.online_step(node.handle,ptr(state),ptr(reference),len(reference),ptr(obs),len(obs),
                    max(0.,now-stamp),ptr(field),node.nx,node.ny,node.gx,node.gy,node.res,0,ptr(result))
                if code:reason='kernel_error'
                elif result[6]:reason='no_feasible_candidate'
                else:
                    count=node.lib.online_selected_controls(node.handle,None,0)
                    controls=np.zeros((count,2))
                    node.lib.online_selected_controls(node.handle,ptr(controls),count)
                    if count<1 or not np.allclose(controls[0],result[:2],rtol=0,atol=1e-9):reason='selected_control_mismatch'
        # Validate on the LATEST same-frame collision snapshot and measured
        # state. Hold the lock through publication, so goal/cancel/perception
        # callbacks cannot change the safety inputs between check and command.
        with node.lock:
            published=rospy.get_time();session=node.execution
            current_state=None if node.state is None else node.state.copy()
            field=node.field;obs=node.obs.copy();stamp=node.obs_stamp;cloud_stamp=node.cloud_stamp;odom_stamp=node.odom_stamp
            if session.generation!=generation:reason='goal_changed'
            elif session.authority()!='ok':reason=session.authority()
            elif current_state is None or not -.05<=published-odom_stamp<=.2:reason='odom_stale'
            elif not node.healthy or field is None or not -.05<=published-cloud_stamp<=.35:reason='perception_unhealthy'
            elif not -.05<=published-stamp<=.35:reason='dynamic_stale'
            elif not -.05<=published-node.arm_stamp<=.2 or node.arm_error>.05:reason='fixed_arm_invalid'
            if reason=='ok':
                validation_code=node.lib.online_validate_execution(node.handle,ptr(current_state),ptr(controls),len(controls),
                    ptr(obs),len(obs),max(0.,published-stamp),max(0.,published-odom_stamp),ptr(field),
                    node.nx,node.ny,node.gx,node.gy,node.res,ptr(checked),len(checked),ptr(validation))
                if validation_code:reason='local_validation_failed'
                else:validated_count=int(validation[3])
            computation_ms=(time.monotonic()-wall)*1000
            if computation_ms>50:reason='deadline_miss'
            final_time=rospy.get_time()
            if not -.05<=final_time-odom_stamp<=.2:reason='odom_stale'
            elif not -.05<=final_time-cloud_stamp<=.35:reason='perception_unhealthy'
            if reason!='ok':result[:2]=0;validated_count=0
            cmd=Twist();cmd.linear.x=result[0];cmd.angular.z=result[1];node.pub.publish(cmd)
            issued=TwistStamped();issued.header.frame_id='world';issued.header.stamp=rospy.Time.from_sec(published)
            issued.twist=cmd;node.local_cmd_pub.publish(issued)
            path=RosPath();path.header.frame_id='world';path.header.stamp=rospy.Time.from_sec(published)
            if validated_count:
                for row in checked[:validated_count:5]:
                    pose=PoseStamped();pose.header.frame_id='world';pose.header.stamp=rospy.Time.from_sec(published+row[0])
                    pose.pose.position.x=row[1];pose.pose.position.y=row[2]
                    pose.pose.orientation.z=math.sin(row[3]/2);pose.pose.orientation.w=math.cos(row[3]/2)
                    path.poses.append(pose)
            node.execution_pub.publish(path)
        if reference is not None and last_reference!=(generation,guide.revision,reference_hash):
            node.reference_log.write(json.dumps({'t':now-1000,'revision':guide.revision,'epoch':guide.epoch,
                'stamp':guide.start,'source':'/planning/trajectory','transport':'complete goal-keyed transaction',
                'message_hashes':guide.messages,'sha256':reference_hash,'reference':reference.tolist(),
                'times':np.concatenate(guide.times).tolist()})+'\n')
            path=RosPath();path.header.frame_id='world';path.header.stamp=rospy.Time.from_sec(now)
            for x,y,yaw,v in reference:
                pose=PoseStamped();pose.header=path.header;pose.pose.position.x=x;pose.pose.position.y=y
                pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2);path.poses.append(pose)
            node.reference_pub.publish(path);last_reference=(generation,guide.revision,reference_hash)
        if node.progress_pub is not None and reference is not None and state is not None:
            if node.progress_bundle!=guide.epoch:node.projected_time=0.;node.progress_bundle=guide.epoch
            node.projected_time,scale,error=project(state,np.concatenate(guide.refs),np.concatenate(guide.times),node.projected_time)
            progress=Vector3Stamped();progress.header.frame_id='measured_path_projection';progress.header.stamp=rospy.Time.from_sec(now)
            progress.vector.x=guide.start;progress.vector.y=node.projected_time;progress.vector.z=scale;node.progress_pub.publish(progress)
        node.lib.online_feedback(node.handle,result[0],result[1])
        row={'t':now-1000,'reason':reason,'bundle':revision,'goal_generation':generation,
            'reference_sha256':reference_hash,'reference_source':'/planning/trajectory','reference_error':session.invalid,
            'reference_points':len(reference) if reference is not None else 0,'shadow_only':False,
            'guide_cancelled':cancelled,'execution_authority':'validated_local_rollout','authorized':bool(validated_count),
            'pending_reference_updates':len(node.pending_execution_transactions),
            'local_route':checked[:validated_count:10,1:3].tolist(),'local_phase':None,
            'obstacle_tracks':len(obs),'cloud_age':published-cloud_stamp,'dynamic_age':published-stamp,
            'limit':node.safety,'limit_scope':'native reference advisory only in cooperative mode',
            'applied_v':result[0],'applied_w':result[1],'nominal_v':result[2],'nominal_w':result[3],
            'stop_unverified':bool(result[6]),'kernel_ms':result[7],'callback_ms':computation_ms,'code':code,
            'validation_code':validation_code,'validation':validation.tolist(),'fixed_arm_error':node.arm_error,
            'perception_mode':'classified_atomic','classification':classification,'classified_error':classified_error}
        node.diag.publish(String(json.dumps(row)))
        node.execution_log.write(json.dumps({'t':published-1000,'reason':reason,'goal_generation':generation,
            'guide_revision':revision,'cloud_stamp':cloud_stamp,'odom_stamp':odom_stamp,
            'lease_s':.075,'command':result[:2].tolist(),'validation':validation.tolist(),
            'rollout':checked[:validated_count].tolist(),'controls':controls.tolist() if validated_count else []})+'\n')
        try:rate.sleep()
        except rospy.ROSInterruptException:break
    node.lib.online_destroy(node.handle)
