#!/usr/bin/env python3
"""No SDK import, ROS master, socket, or hardware: exercise AST-loaded callbacks."""
import ast
import math
import threading
import xml.etree.ElementTree as ET
from enum import Enum
from pathlib import Path
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parents[3]
class Clock:
    value = 1000.0
    def time(self): return self.value
    def monotonic(self): return self.value
    def perf_counter(self): return self.value
clock = Clock()
class Stamp:
    def __init__(self, value=0): self.value = value
    def to_sec(self): return self.value
    @classmethod
    def from_sec(cls, value): return cls(value)
class JointState:
    def __init__(self):
        self.header=NS(stamp=Stamp())
        self.name=[]; self.position=[]; self.velocity=[]; self.effort=[]
class ExecutionState:
    def __init__(self):
        self.header=NS(stamp=Stamp())
        self.enabled=False; self.stop_latched=False; self.feedback_valid=False
        self.command_active=False
        self.valid_until=Stamp()
class Pub:
    def __init__(self): self.messages=[]
    def publish(self, msg): self.messages.append(msg)
rospy = NS(Time=Stamp, logwarn=lambda *a: None, logwarn_throttle=lambda *a: None,
           logerr=lambda *a: None, loginfo=lambda *a: None)
def response(*args, **kwargs): return NS(args=args, **kwargs)
namespace = dict(rospy=rospy, JointState=JointState, math=math, time=clock,
                 threading=threading, Bool=NS, TriggerResponse=response,
                 EnableResponse=response, GoZeroResponse=response,
                 SetBoolResponse=response, GripperResponse=response, PiperStatusMsg=NS,
                 PiperExecutionState=ExecutionState, ET=ET)
source=ROOT/'src/piper_ros/src/piper/scripts/piper_ctrl_single_node.py'
tree=ast.parse(source.read_text())
selected=[n for n in tree.body if isinstance(n,(ast.ClassDef,ast.Assign))]
exec(compile(ast.Module(body=selected,type_ignores=[]), str(source), 'exec'), namespace)
Node=namespace['C_PiperRosNode']; names=namespace['ARM_JOINT_NAMES']
groups=namespace['SDK_JOINT_GROUPS']
class SDK:
    def __init__(self):
        self.calls=[]; self.fail_joint=False; self.fail_stop=False
        self._C_PiperInterface__fps_counter=NS(lock=threading.Lock(),fps_data={},last_time={})
        self._C_PiperInterface__arm_joint_msgs_mtx=threading.Lock()
        self._C_PiperInterface__arm_joint_msgs=NS(time_stamp=0, joint_state=NS(**{'joint_%d'%i:0 for i in range(1,7)}))
        self._C_PiperInterface__arm_motor_info_low_spd_mtx=threading.Lock()
        self._C_PiperInterface__arm_motor_info_low_spd=NS(time_stamp=0, **{'motor_%d'%i:NS(foc_status=NS(driver_enable_status=False)) for i in range(1,7)})
        errors={key:False for i in range(1,7) for key in
                ('joint_%d_angle_limit'%i, 'communication_status_joint_%d'%i)}
        self._C_PiperInterface__arm_status_mtx=threading.Lock()
        self._C_PiperInterface__arm_status=NS(time_stamp=0, arm_status=NS(
            ctrl_mode=0, arm_status=0, mode_feed=1, teach_status=0, motion_status=0,
            trajectory_num=0, err_code=0, err_status=NS(**errors)))
    def EnableArm(self,*a): self.calls.append(('enable',a))
    def MotionCtrl_1(self,*a):
        self.calls.append(('stop',a))
        if self.fail_stop: raise RuntimeError('fake stop TX error')
    def MotionCtrl_2(self,*a): self.calls.append(('mode',a))
    def JointCtrl(self,*a):
        self.calls.append(('joint',a))
        if self.fail_joint: raise RuntimeError('fake TX error')
    def GetArmGripperMsgs(self): return NS(time_stamp=clock.time()-.001,gripper_state=NS(grippers_angle=40000))
    def GetArmStatus(self): return self._C_PiperInterface__arm_status
def node():
    clock.value=1000.0
    n=Node.__new__(Node); n.piper=SDK(); n.joint_pub=Pub(); n.arm_status_pub=Pub()
    n.execution_state_pub=Pub(); n._execution_source_stamp=0.
    n.command_timeout=.3; n.max_command_age=.3; n.fixed_vel=1
    n._command_lock=threading.RLock(); n._command_latched=False
    n._command_deadline=None; n._last_command_stamp=0.; n._command_seen=False
    n._feedback_stamp=0.; n._feedback_monotonic=0.; n._feedback_position=None
    n._feedback_velocity=None
    n.native_startup_to_standby=False; n._native_handoff=False; n._native_ready=False
    n._native_started=None; n._native_last_enable=None; n._native_last_target=None
    n._native_settled_since=None; n._native_motors_seen=False
    n._native_target=None; n._native_target_counts=None
    n._last_joint_counts=(0,0,0); n._previous_feedback=None
    n._last_arm_status_count=0; n._last_arm_status_stamp=0.
    n._watchdog_done=threading.Event(); n._C_PiperRosNode__enable_flag=False
    n.gripper_exist=False; n.gripper_val_mutiple=1; n.block_ctrl_flag=False
    return n
def feed(n, pairs=(0,1,2), raw=0, enabled=True, status=True, publish=True):
    counter=n.piper._C_PiperInterface__fps_counter
    f=n.piper._C_PiperInterface__arm_joint_msgs
    for pair in pairs:
        key=groups[pair]; counter.fps_data[key]=counter.fps_data.get(key,0)+1
        counter.last_time[key]=clock.perf_counter()
        for joint in (pair*2, pair*2+1):
            setattr(f.joint_state,'joint_%d'%(joint+1),raw[joint] if isinstance(raw,(list,tuple)) else raw)
    f.time_stamp=clock.time()-.002
    motor=n.piper._C_PiperInterface__arm_motor_info_low_spd
    motor.time_stamp=clock.time()-.002
    for i in range(1,7):
        key='ArmMotorDriverInfoLowSpd_%d'%i
        counter.fps_data[key]=counter.fps_data.get(key,0)+1
        counter.last_time[key]=clock.perf_counter()
        getattr(motor,'motor_%d'%i).foc_status.driver_enable_status=enabled
    if status:
        counter.fps_data['ArmStatus']=counter.fps_data.get('ArmStatus',0)+1
        counter.last_time['ArmStatus']=clock.perf_counter()
        n.piper._C_PiperInterface__arm_status.time_stamp=clock.time()-.002
    if publish: n.PublishArmJointAndGripper()
def command(n, stamp=None, q=None):
    m=JointState(); m.name=list(names); m.position=list(n._feedback_position if q is None else q)
    m.header.stamp=Stamp(clock.time()-.001 if stamp is None else stamp)
    return m
checks=[]
n=node(); feed(n,pairs=(0,)); assert not n.joint_pub.messages
feed(n,pairs=(1,)); assert not n.joint_pub.messages
feed(n,pairs=(2,), raw=1000); assert len(n.joint_pub.messages)==1
out=n.joint_pub.messages[-1]
assert out.header.stamp.to_sec()==clock.time()-.002
assert math.isclose(out.position[4],math.pi/180) and not out.velocity
n.PublishArmJointAndGripper(); assert len(n.joint_pub.messages)==1
checks.append('three-pair completeness, exact angle units, source stamp, no cached republish')
clock.value+=.05; feed(n,raw=1000); out=n.joint_pub.messages[-1]
assert math.isclose(out.velocity[0],math.pi/180/.05,rel_tol=1e-10)
assert out.velocity[4]==0
clock.value+=.05; feed(n,pairs=(0,1),raw=2000)
assert len(n.joint_pub.messages)==2
clock.value+=.2; n.PublishArmJointAndGripper(); assert len(n.joint_pub.messages)==2
checks.append('position-derived velocity and missing/stale pair withholding')

for consumer in ('_check_command_timeout','PublishExecutionState'):
    n=node(); feed(n); assert n._request_enable(True)
    clock.value+=.05; feed(n); n.joint_callback(command(n))
    published=len(n.joint_pub.messages); old_stamp=n._feedback_stamp
    clock.value+=.16; feed(n,raw=1000,publish=False)
    sdk_stamp=n.piper._C_PiperInterface__arm_joint_msgs.time_stamp
    assert clock.time()-old_stamp>.15 and clock.time()-sdk_stamp<.003
    assert n._command_deadline-clock.time()>.13
    getattr(n,consumer)()
    assert not n._command_latched and n.GetEnableFlag(), consumer
    assert n._feedback_stamp==sdk_stamp and n._feedback_velocity is None
    assert len(n.joint_pub.messages)==published  # Health sampling has no ROS publication side effect.
    n.PublishExecutionState(); state=n.execution_state_pub.messages[-1]
    assert state.enabled and state.feedback_valid and state.command_active
    assert state.valid_until.to_sec()>clock.time() and state.header.stamp.to_sec()==sdk_stamp
    n.PublishArmJointAndGripper(); n.PublishArmJointAndGripper()
    assert len(n.joint_pub.messages)==published+1
    assert n.joint_pub.messages[-1].header.stamp.to_sec()==sdk_stamp
    assert not n.joint_pub.messages[-1].velocity  # A 160ms gap does not manufacture zero speed.
    clock.value+=.04; feed(n,raw=1100,publish=False)
    assert n._feedback_fresh()
    expected=100*math.pi/180000/.04
    assert all(math.isclose(v,expected,rel_tol=1e-9) for v in n._feedback_velocity)
    assert len(n.joint_pub.messages)==published+1
    n.PublishArmJointAndGripper(); n.PublishArmJointAndGripper()
    assert len(n.joint_pub.messages)==published+2
    latest=n.joint_pub.messages[-1]
    assert latest.header.stamp.to_sec()==clock.time()-.002
    assert all(math.isclose(v,expected,rel_tol=1e-9) for v in latest.velocity)
checks.append('watchdog/execution-state sample complete fresh SDK input despite stalled ROS publisher; one source-stamped publication and real velocity')

for update in ('none','stamp_only','counts_only'):
    n=node(); feed(n)
    accepted=(n._feedback_stamp,n._feedback_monotonic,n._last_joint_counts)
    clock.value+=.05
    if update!='none':
        feed(n,pairs=() if update=='stamp_only' else (0,1,2),publish=False)
        if update=='counts_only': n.piper._C_PiperInterface__arm_joint_msgs.time_stamp=accepted[0]
    assert n._feedback_fresh()
    n.PublishArmJointAndGripper()
    assert (n._feedback_stamp,n._feedback_monotonic,n._last_joint_counts)==accepted, update
    assert len(n.joint_pub.messages)==1
    clock.value+=.11
    assert not n._feedback_fresh(), update
    assert (n._feedback_stamp,n._feedback_monotonic,n._last_joint_counts)==accepted, update
checks.append('repeated SDK reads, source stamp alone or receive counts alone cannot renew accepted source/steady ages or republish')

for invalid in ('missing_pair','pair_spread','future_stamp','nonfinite_stamp'):
    n=node(); feed(n); assert n._request_enable(True)
    clock.value+=.05; feed(n); n.joint_callback(command(n))
    accepted=(n._feedback_stamp,n._feedback_monotonic,n._last_joint_counts)
    published=len(n.joint_pub.messages)
    clock.value+=.16
    feed(n,pairs=(0,1) if invalid=='missing_pair' else (0,1,2),publish=False)
    if invalid=='pair_spread':
        n.piper._C_PiperInterface__fps_counter.last_time[groups[0]]-=.051
    if invalid=='future_stamp': n.piper._C_PiperInterface__arm_joint_msgs.time_stamp=clock.time()+.001
    if invalid=='nonfinite_stamp': n.piper._C_PiperInterface__arm_joint_msgs.time_stamp=float('nan')
    recorded=[]; saved_logerr=rospy.logerr
    rospy.logerr=lambda fmt,*args:recorded.append(fmt % args if args else fmt)
    try: n._check_command_timeout()
    finally: rospy.logerr=saved_logerr
    assert n._command_latched and n.piper.calls[-1][0]=='stop', invalid
    assert (n._feedback_stamp,n._feedback_monotonic,n._last_joint_counts)==accepted, invalid
    n.PublishArmJointAndGripper(); assert len(n.joint_pub.messages)==published
    detail=next(line for line in recorded if 'joint feedback expired' in line)
    assert all(field in detail for field in ('accepted_source_age_ms=', 'sdk_source_age_ms=',
        'pair_update_age_ms=', 'pair_spread_ms=', 'pair_advanced=')), detail
    if invalid=='missing_pair': assert 'pair_advanced=(True, True, False)' in detail, detail
checks.append('independent sampling rejects lost pair, >50ms pair spread and future/nonfinite source; timeout log identifies real SDK timing')
n=node(); assert not n._request_enable(True) and not n.piper.calls
feed(n,enabled=False); assert n._request_enable(True)
n._check_command_timeout(); assert not n._command_latched
clock.value+=.04; feed(n,enabled=False); n.joint_callback(command(n))
assert len(n.piper.calls)==1
clock.value+=.04; feed(n,enabled=True); valid=command(n)
# A solve started before manual enable must be rejected without sending or stopping.
n.joint_callback(command(n, stamp=999.999))
assert len(n.piper.calls)==1 and not n._command_latched
n.joint_callback(valid)
assert [c[0] for c in n.piper.calls]==['enable','mode','joint']
assert n.piper.calls[-2][1][2]==1
n.joint_callback(valid); assert len(n.piper.calls)==3
checks.append('enable requires feedback; pending motor enable waits; fresh command; replay rejected')
for bad in ([],[0]*5,[float('nan')]*6):
    m=command(n,q=bad); n.joint_callback(m)
assert len(n.piper.calls)==3
m=command(n); m.name=list(reversed(names)); n.joint_callback(m)
assert len(n.piper.calls)==3
m=command(n,stamp=clock.time()+.01); n.joint_callback(m)
assert len(n.piper.calls)==3
checks.append('malformed, nonfinite, unordered and future command rejection')
count=len(n.piper.calls)
clock.value+=.31; feed(n); n._check_command_timeout()
assert not n._command_latched and n.GetEnableFlag() and n._command_seen
assert n._command_deadline is None and len(n.piper.calls)==count
hold_stamp=n._last_command_stamp
assert hold_stamp==clock.time()
n.PublishExecutionState(); held=n.execution_state_pub.messages[-1]
assert held.enabled and held.feedback_valid and not held.stop_latched and not held.command_active
assert held.header.stamp.to_sec()==clock.time()-.002 and held.valid_until.to_sec()>clock.time()
# Even a recently queued plan must not restart movement across the expiry.
n.joint_callback(command(n,stamp=hold_stamp-.01))
n.joint_callback(command(n,stamp=hold_stamp))
assert len(n.piper.calls)==count and n._command_deadline is None
clock.value+=.04; feed(n); n._check_command_timeout()
assert n._last_command_stamp==hold_stamp and len(n.piper.calls)==count
fresh_command=command(n)
n.joint_callback(fresh_command); n.PublishExecutionState()
assert [kind for kind,_ in n.piper.calls[count:]]==['mode','joint']
assert n._last_command_stamp==fresh_command.header.stamp.to_sec()
assert not n._command_latched and n.GetEnableFlag() and n._command_deadline>clock.time()
assert n.execution_state_pub.messages[-1].command_active
checks.append('successful-command expiry holds without CAN/disable; rejects queued stamps; fresh plan resumes without reset')

for lost in ('joint','status','motor','disabled','arm_status','err_code'):
    n=node(); feed(n); assert n._request_enable(True)
    clock.value+=.05; feed(n); n.joint_callback(command(n))
    clock.value+=.31; feed(n); n._check_command_timeout()
    assert n._command_deadline is None and n.GetEnableFlag() and not n._command_latched
    count=len(n.piper.calls)
    clock.value+=.16
    feed(n,pairs=() if lost=='joint' else (0,1,2),enabled=lost!='disabled',status=lost!='status')
    if lost=='motor':
        n.piper._C_PiperInterface__fps_counter.last_time['ArmMotorDriverInfoLowSpd_6']-=.2
    if lost in ('arm_status','err_code'):
        setattr(n.piper._C_PiperInterface__arm_status.arm_status,lost,1)
    n._check_command_timeout()
    assert n._command_latched and not n.GetEnableFlag(), lost
    assert [kind for kind,_ in n.piper.calls[count:]]==['stop'], lost
    count=len(n.piper.calls)
    clock.value+=.01; feed(n)
    n.joint_callback(command(n)); n._check_command_timeout()
    assert len(n.piper.calls)==count and not n._request_enable(True), lost
checks.append('holding still hard-stops on real joint/status/motor loss or faults; healthy recovery cannot unlatch')

n=node(); feed(n); assert n._request_enable(True)
clock.value+=.05; feed(n); n.joint_callback(command(n))
clock.value+=.31  # Lease and source feedback are both expired.
n._check_command_timeout()
assert n._command_latched and not n.GetEnableFlag() and n.piper.calls[-1][0]=='stop'
checks.append('real feedback loss takes priority when the command lease expires at the same time')

n=node(); feed(n); assert n._request_enable(True)
clock.value+=.05; feed(n)
for joint in range(6):
    for sign in (-1,1):
        q=list(n._feedback_position); q[joint]+=sign*.030001
        count=len(n.piper.calls); deadline=n._command_deadline
        n.joint_callback(command(n,q=q))
        assert len(n.piper.calls)==count and n._command_deadline==deadline
        assert not n._command_seen and not n._command_latched
q=list(n._feedback_position); q[0]+=.03; q[5]-=.03
n.joint_callback(command(n,q=q))
assert [kind for kind,_ in n.piper.calls]==['enable','mode','joint']
assert n._command_seen and not n._command_latched
checks.append('leased commands independently reject any joint lead above .03rad; exact positive/negative boundary is accepted')
n=node(); feed(n); n._request_enable(True); clock.value+=.05; feed(n); n.joint_callback(command(n))
clock.value+=.16; n._check_command_timeout(); assert n._command_latched
checks.append('independent feedback expiration stops before command lease')
original_logerr = rospy.logerr
for cause in ('command', 'feedback'):
    recorded=[]
    rospy.logerr=lambda fmt,*args:recorded.append(fmt % args if args else fmt)
    n=node(); feed(n); assert n._request_enable(True)
    if cause=='command':
        clock.value+=.31; feed(n)
    else:
        clock.value+=.16
    n._check_command_timeout()
    assert n._command_latched
    expected=('command expired (first_command_received=False)' if cause=='command'
              else 'joint feedback expired')
    assert any(expected in line for line in recorded),recorded
    calls=len(n.piper.calls)
    assert not n._request_enable(True) and len(n.piper.calls)==calls
rospy.logerr=original_logerr
checks.append('stop diagnostics distinguish initial command timeout from feedback loss without relaxing either')
n=node(); feed(n); n._request_enable(True); clock.value+=.05; feed(n); n.joint_callback(command(n))
n.piper._C_PiperInterface__fps_counter.last_time['ArmMotorDriverInfoLowSpd_6']-=1
n._check_command_timeout(); assert n._command_latched
checks.append('one stale motor enable state stops even when other five update')
for failure in ('motor6_stale', 'source_stale', 'motor4_missing'):
    n=node(); feed(n); assert n._request_enable(True)
    clock.value+=.05; feed(n); n.joint_callback(command(n))
    counter=n.piper._C_PiperInterface__fps_counter
    motor=n.piper._C_PiperInterface__arm_motor_info_low_spd
    if failure=='motor6_stale': counter.last_time['ArmMotorDriverInfoLowSpd_6']-=.151
    if failure=='source_stale': motor.time_stamp=clock.time()-.151
    if failure=='motor4_missing': counter.fps_data['ArmMotorDriverInfoLowSpd_4']=0
    original_check=n._motors_enabled_fresh; sampled=[]; recorded=[]
    def recover_after_checked_sample(diagnostics=None):
        result=original_check(diagnostics)
        sampled.append(dict(diagnostics))
        feed(n,publish=False)  # SDK recovers before the stop message is formed.
        return result
    n._motors_enabled_fresh=recover_after_checked_sample
    saved_logerr=rospy.logerr
    rospy.logerr=lambda fmt,*args:recorded.append(fmt % args if args else fmt)
    try: n._check_command_timeout()
    finally: rospy.logerr=saved_logerr
    assert len(sampled)==1 and not sampled[0]['fresh'], failure
    captured=sampled[0]
    assert captured['enabled']==(True,)*6
    assert len(captured['received'])==len(captured['update_age_ms'])==6
    if failure=='motor6_stale': assert captured['update_age_ms']==(0.,0.,0.,0.,0.,151.)
    if failure=='source_stale': assert math.isclose(captured['source_age_ms'],151.,abs_tol=1e-6)
    if failure=='motor4_missing': assert captured['received']==(True,True,True,False,True,True)
    line=next(s for s in recorded if 'motor feedback unavailable/disabled' in s)
    assert 'motor_fresh=False' in line and 'received=%s'% (captured['received'],) in line
    assert 'update_age_ms=%s'% (captured['update_age_ms'],) in line
    assert 'enabled=%s'% (captured['enabled'],) in line
    assert 'sdk_source_age_ms=%.3f'%captured['source_age_ms'] in line
    assert n._command_latched and not n.GetEnableFlag()
    assert sum(kind=='stop' for kind,_ in n.piper.calls)==1
    assert original_check()  # Recovery cannot undo the already latched decision.
checks.append('motor loss logs all six fields from the failed sample despite immediate SDK recovery')
n=node(); feed(n); assert n._request_enable(True)
clock.value+=.05; feed(n); n.joint_callback(command(n))
n.piper._C_PiperInterface__arm_motor_info_low_spd.motor_3.foc_status.driver_enable_status=False
recorded=[]; saved_logerr=rospy.logerr
rospy.logerr=lambda fmt,*args:recorded.append(fmt % args if args else fmt)
try: n._check_command_timeout()
finally: rospy.logerr=saved_logerr
assert n._command_latched and n.piper.calls[-1][0]=='stop'
line=next(s for s in recorded if 'motor feedback unavailable/disabled' in s)
assert 'motor_fresh=True' in line and 'enabled=(True, True, False, True, True, True)' in line
assert 'update_age_ms=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)' in line
checks.append('fresh motor disable is distinct from stale feedback in the same unchanged hard-stop path')
n=node(); feed(n,status=False)
assert not n._request_enable(True) and not n.piper.calls
n.PublishArmState(); assert not n.arm_status_pub.messages
feed(n); n.PublishArmState(); assert len(n.arm_status_pub.messages)==1
saved=n.arm_status_pub.messages[-1]
f=n.piper._C_PiperInterface__arm_status
f.arm_status.err_code=7; f.arm_status.err_status.joint_2_angle_limit=True
assert saved.err_code==0 and not saved.joint_2_angle_limit
f.arm_status.err_code=0; f.arm_status.err_status.joint_2_angle_limit=False
n.PublishArmState(); assert len(n.arm_status_pub.messages)==1
clock.value+=.01; feed(n); n.PublishArmState(); assert len(n.arm_status_pub.messages)==2
counter=n.piper._C_PiperInterface__fps_counter
f.time_stamp+=.001  # Stamp alone is not a new SDK receive event.
n.PublishArmState(); assert len(n.arm_status_pub.messages)==2
counter.fps_data['ArmStatus']+=1
f.time_stamp=n._last_arm_status_stamp  # Count alone cannot republish an old source stamp.
n.PublishArmState(); assert len(n.arm_status_pub.messages)==2
checks.append('ArmStatus requires received source, publishes only advancing stamp/count, copies all scalar flags')
for invalid in ('missing_count','old_wall','future_wall','nonfinite_wall','old_receive','future_receive'):
    n=node(); feed(n)
    f=n.piper._C_PiperInterface__arm_status
    counter=n.piper._C_PiperInterface__fps_counter
    if invalid=='missing_count': counter.fps_data['ArmStatus']=0
    if invalid=='old_wall': f.time_stamp-=.2
    if invalid=='future_wall': f.time_stamp=clock.time()+.001
    if invalid=='nonfinite_wall': f.time_stamp=float('nan')
    if invalid=='old_receive': counter.last_time['ArmStatus']-=.2
    if invalid=='future_receive': counter.last_time['ArmStatus']=clock.time()+.001
    n.PublishArmState(); assert not n.arm_status_pub.messages, invalid
    assert not n._request_enable(True) and not n.piper.calls, invalid
checks.append('ArmStatus source count, wall and monotonic ages gate publication and enable')
n=node(); feed(n); assert n._request_enable(True)
clock.value+=.05; feed(n); n.joint_callback(command(n))
n.PublishArmState(); count=len(n.arm_status_pub.messages)
clock.value+=.16; feed(n,status=False)  # Joints/motors live; only status frames have stopped.
n.PublishArmState(); assert len(n.arm_status_pub.messages)==count
n._check_command_timeout(); assert n._command_latched and n.piper.calls[-1][0]=='stop'
tx_count=len(n.piper.calls)
clock.value+=.01; feed(n); n.PublishArmState()
assert not n._request_enable(True)
n.joint_callback(command(n)); n._check_command_timeout()
assert len(n.piper.calls)==tx_count
checks.append('ArmStatus loss alone stops before live command expiry; fresh normal status cannot release latch')
for bad_field in ('arm_status','err_code'):
    n=node(); feed(n); assert n._request_enable(True)
    clock.value+=.05; feed(n)
    setattr(n.piper._C_PiperInterface__arm_status.arm_status,bad_field,1)
    n.joint_callback(command(n))
    assert n._command_latched and [c[0] for c in n.piper.calls]==['enable','stop']
checks.append('fresh abnormal ArmStatus stops at command entry without sending JointCtrl')
n=node(); n.PublishExecutionState(); report=n.execution_state_pub.messages[-1]
assert report.header.stamp.to_sec()==0 and report.valid_until.to_sec()==0
assert not report.enabled and not report.feedback_valid and not report.stop_latched
feed(n,enabled=False); n.PublishExecutionState(); report=n.execution_state_pub.messages[-1]
assert report.feedback_valid and not report.enabled and not report.stop_latched
assert report.header.stamp.to_sec()==clock.time()-.002
assert n._request_enable(True)
assert not n.execution_state_pub.messages[-1].enabled  # Wait for all six hardware enables.
clock.value+=.01; feed(n); n.PublishExecutionState(); report=n.execution_state_pub.messages[-1]
assert report.enabled and report.feedback_valid and not report.stop_latched
assert math.isclose(report.valid_until.to_sec(),report.header.stamp.to_sec()+.15)
checks.append('execution state denies missing input and manual-disabled/pending motors; retains actual CAN source stamp')
n=node(); feed(n); assert n._request_enable(True)
clock.value+=.147; feed(n,status=False); n.PublishExecutionState()
queued=n.execution_state_pub.messages[-1]
assert queued.enabled and queued.feedback_valid
assert math.isclose(queued.valid_until.to_sec()-clock.time(),.001,abs_tol=1e-10)
source=queued.header.stamp.to_sec(); expiry=queued.valid_until.to_sec()
clock.value+=.002  # A queued positive message has expired, despite just arriving.
assert clock.time()>queued.valid_until.to_sec()
n._check_command_timeout(); stopped=n.execution_state_pub.messages[-1]
assert stopped.header.stamp.to_sec()==source and stopped.stop_latched
assert not stopped.enabled and not stopped.feedback_valid and stopped.valid_until.to_sec()==0
clock.value+=.01; feed(n); n.PublishExecutionState(); recovered=n.execution_state_pub.messages[-1]
assert recovered.stop_latched and not recovered.enabled and not recovered.feedback_valid
assert recovered.header.stamp.to_sec()==source and recovered.valid_until.to_sec()==0
assert queued.header.stamp.to_sec()==source and queued.valid_until.to_sec()==expiry
checks.append('queued positive execution state cannot gain validity; same-source stop and recovered feedback retain latch')
for limiting_source in ('joint_pair','motor_6','command_lease'):
    n=node(); feed(n); assert n._request_enable(True)
    counter=n.piper._C_PiperInterface__fps_counter
    if limiting_source=='joint_pair': counter.last_time[groups[0]]-=.14
    if limiting_source=='motor_6': counter.last_time['ArmMotorDriverInfoLowSpd_6']-=.14
    if limiting_source=='command_lease': n._command_deadline=clock.time()+.01
    n.PublishExecutionState(); report=n.execution_state_pub.messages[-1]
    assert report.enabled and report.feedback_valid, limiting_source
    assert math.isclose(report.valid_until.to_sec(),clock.time()+.01,abs_tol=1e-10), limiting_source
    stamp=report.header.stamp.to_sec(); expiry=report.valid_until.to_sec()
    clock.value+=.005; n.PublishExecutionState(); repeated=n.execution_state_pub.messages[-1]
    assert repeated.header.stamp.to_sec()==stamp
    assert math.isclose(repeated.valid_until.to_sec(),expiry,abs_tol=1e-10)
    clock.value=expiry
    if limiting_source=='command_lease':
        feed(n); n._check_command_timeout()
        assert n.execution_state_pub.messages[-1].stop_latched
    else:
        n.PublishExecutionState()
        assert not n.execution_state_pub.messages[-1].feedback_valid
checks.append('execution expiry uses oldest joint pair/motor source or active command lease; repeats do not renew')
n=node(); feed(n); n._request_enable(True); clock.value+=.05; feed(n)
n.piper.fail_joint=True; n.piper.fail_stop=True
n.joint_callback(command(n)); assert n._command_latched and not n.GetEnableFlag()
report=n.execution_state_pub.messages[-1]
assert report.stop_latched and not report.enabled and not report.feedback_valid
assert report.valid_until.to_sec()==0
count=len(n.piper.calls); n.joint_callback(command(n)); assert len(n.piper.calls)==count
checks.append('send failure and quick-stop failure preserve latch')
n=node(); feed(n); assert n._request_enable(True)
clock.value+=.05; feed(n); n.joint_callback(command(n))
next_stamp=n._last_command_stamp+.001
clock.value+=.28; feed(n)
def expire_lease_during_send(*args, _send=n.piper.JointCtrl, _node=n):
    _send(*args); clock.value+=.03; feed(_node)  # Sources remain healthy.
n.piper.JointCtrl=expire_lease_during_send
n.joint_callback(command(n,stamp=next_stamp))
assert n._feedback_fresh() and n._arm_status_normal_fresh() and n._motors_enabled_fresh()
assert n._command_latched and not n.GetEnableFlag() and n.piper.calls[-1][0]=='stop'
checks.append('expiry during CAN send still hard-stops even with fresh joint/status/all-motor sources')
n=node(); feed(n); assert n._request_enable(True)
def fail_publication(message): raise RuntimeError('fake ROS publication error')
n.execution_state_pub.publish=fail_publication
iterations=iter((False,True)); n._watchdog_done=NS(wait=lambda timeout:next(iterations))
n._command_watchdog()
assert n._command_latched and n.piper.calls[-1][0]=='stop'
checks.append('execution-state publication error cannot kill watchdog without latching and attempting quick-stop')
n=node(); feed(n); n._request_enable(True)
count=len(n.piper.calls)
n.pos_callback(NS()); assert not n.handle_go_zero_service(NS()).status
assert not n.handle_reset_service(NS()).success
assert not n.handle_gripper_service(NS()).status
assert len(n.piper.calls)==count
n._shutdown_commands(); assert n._command_latched and n._watchdog_done.is_set()
checks.append('pos/gozero/reset/gripper bypass disabled and shutdown stops')
n=node(); n.command_timeout=0; feed(n); n._C_PiperRosNode__enable_flag=True
n.PublishExecutionState(); assert not n.execution_state_pub.messages
n.piper._C_PiperInterface__arm_status.time_stamp=0.
n.PublishArmState(); n.PublishArmState(); assert len(n.arm_status_pub.messages)==2
m=command(n,stamp=0,q=[.2]*6); n.joint_callback(m)
assert [c[0] for c in n.piper.calls]==['mode','joint']
checks.append('native mode retains unstamped larger-target commands and repeated cached status publication compatibility')
bridge_path=ROOT/'src/mm_remani/scripts/real_arm_joint_bridge.py'
bridge_ns=dict(rospy=rospy, JointState=JointState,math=math)
tree=ast.parse(bridge_path.read_text()); selected=[x for x in tree.body if isinstance(x,ast.ClassDef)]
exec(compile(ast.Module(body=selected,type_ignores=[]),str(bridge_path),'exec'),bridge_ns)
bridge=bridge_ns['RealArmJointBridge'].__new__(bridge_ns['RealArmJointBridge']); bridge.pub=Pub()
m=JointState(); m.header.stamp=Stamp(1000); m.name=list(names)+['gripper']; m.position=[0]*6+[.04]; m.velocity=[.1]*7
bridge.joint_cb(m); out=bridge.pub.messages[-1]
assert out.position[-2:]==[.04,.04] and out.velocity==[.1]*8 and out.header is m.header
m.position.pop(); bridge.joint_cb(m); assert len(bridge.pub.messages)==1
checks.append('bridge retains velocities/source header, gripper metres and rejects missing position')
filter_path=ROOT/'src/mm_remani/scripts/filter_joint_states.py'
filter_ns=dict(rospy=rospy, JointState=JointState,math=math,pub=Pub())
tree=ast.parse(filter_path.read_text()); selected=[x for x in tree.body if isinstance(x,(ast.FunctionDef,ast.Assign))]
exec(compile(ast.Module(body=selected,type_ignores=[]),str(filter_path),'exec'),filter_ns)
cb=filter_ns['joint_cb']; m=JointState(); m.name=list(names); m.position=[0]*6; m.header.stamp=Stamp(1000); cb(m)
assert not filter_ns['pub'].messages
m=JointState(); m.name=list(names); m.position=[.01]*6; m.header.stamp=Stamp(1000.05); cb(m)
assert all(math.isclose(v,.2,rel_tol=1e-10) for v in filter_ns['pub'].messages[-1].velocity)
m.name.pop(); m.position.pop(); cb(m); assert len(filter_ns['pub'].messages)==1
checks.append('filter never fills missing joints/velocities with zero; derives real velocity')

# Native startup is tested through the same watchdog/command callbacks with
# advancing fake CAN source samples; no ROS, SDK or hardware is constructed.
original_initial = ast.literal_eval(next(line.split(':',1)[1].strip() for line in
    (ROOT/'src/mm_remani_planner/plan_manage/config/exp0_param.yaml').read_text().splitlines()
    if line.strip().startswith('init_state ')))
robot_description = (ROOT/'src/my_description/robot/mm.urdf').read_text()
native_params = {'~native_startup_to_standby':True,
                 '/remani_planner_node/fsm/init_state':original_initial,
                 '/robot_description':robot_description}
rospy.get_param = lambda key,*default: native_params[key] if key in native_params else default[0]
def native_node():
    n=node(); n._configure_native_startup(); return n
def native_feed(n, dt=.10, at_target=True, enabled=True, status=True):
    clock.value+=dt
    feed(n,raw=n._native_target_counts if at_target else 0,enabled=enabled,status=status)
    n._check_command_timeout()
def native_ready():
    n=native_node(); feed(n,enabled=False); n._check_command_timeout()
    for _ in range(8): native_feed(n)
    assert n._native_ready and not n._command_latched
    return n

n=native_node(); assert not n.piper.calls
assert all(math.isclose(q,math.radians(d)) for q,d in zip(n._native_target,original_initial[2:]))
for key,bad in (('~native_startup_to_standby','false'),
                ('/remani_planner_node/fsm/init_state',[0]*7),
                ('/remani_planner_node/fsm/init_state',[0]*7+[float('nan')]),
                ('/remani_planner_node/fsm/init_state',[0,0,300,0,0,0,0,0]),
                ('/remani_planner_node/fsm/init_state',[0,0,0,0,0,0,0,45]),
                ('/robot_description','<robot/>')):
    saved=native_params[key]; native_params[key]=bad
    n=node()
    try:
        n._configure_native_startup()
        raise AssertionError('invalid native target/config accepted: '+key)
    except ValueError:
        pass
    assert not n.piper.calls
    native_params[key]=saved
n=node(); n.command_timeout=0
try:
    n._configure_native_startup(); raise AssertionError('native startup without lease accepted')
except ValueError:
    pass
checks.append('native target reads original 8-value config; finite/URDF/wrist/lease validation precedes CAN')

n=native_node(); n._check_command_timeout(); clock.value+=100; n._check_command_timeout()
assert not n.piper.calls and n._native_started is None
feed(n,enabled=False); n._check_command_timeout()
assert [kind for kind,_ in n.piper.calls]==['enable'] and n._native_started==clock.time()
stamp=n._last_command_stamp
for _ in range(6): native_feed(n,at_target=False,enabled=False)
assert not n._command_latched and n._command_deadline is None
assert n._last_command_stamp==stamp and not n._native_ready
for _ in range(100):
    native_feed(n,at_target=False,enabled=False)
    if n._command_latched: break
assert n._command_latched and not n._native_motors_seen
assert 1 <= sum(k=='enable' for k,_ in n.piper.calls) <= 10
assert not any(k=='joint' for k,_ in n.piper.calls)
count=len(n.piper.calls); native_feed(n)
assert len(n.piper.calls)==count and not n._request_enable(True)
checks.append('native enable waits for real source input, retries at 1s, stops within 10s without command lease')

n=native_node(); feed(n,enabled=False); n._check_command_timeout()
native_feed(n,at_target=False)
native_feed(n)  # Large change: real measured velocity prevents ready.
assert not n._native_ready and max(abs(v) for v in n._feedback_velocity)>.02
before=len(n.piper.calls); n.joint_callback(command(n,q=[.2]*6))
assert len(n.piper.calls)==before and not n._command_seen
send_times=[]; previous_count=sum(k=='joint' for k,_ in n.piper.calls)
for _ in range(24):
    native_feed(n,dt=.05)
    count=sum(k=='joint' for k,_ in n.piper.calls)
    if count>previous_count: send_times.append(clock.time())
    previous_count=count
assert n._native_ready and n._command_deadline is None
assert all(b-a>=.20-1e-10 for a,b in zip(send_times,send_times[1:]))
assert all(args[2]==15 for kind,args in n.piper.calls if kind=='mode')
assert all(args==n._native_target_counts for kind,args in n.piper.calls if kind=='joint')
n.PublishExecutionState(); state=n.execution_state_pub.messages[-1]
assert state.enabled and state.feedback_valid and not state.command_active
assert state.valid_until.to_sec()>clock.time() and n._last_command_stamp==1000.
checks.append('native target uses original speed15 at at most5Hz; motion/settling gates ready; no early external command')

n=native_ready(); n.gripper_exist=True
native_feed(n)
assert n._feedback_velocity is not None and all(v==0 for v in n._feedback_velocity)
assert n._native_ready  # Gripper presence does not reset the six-arm velocity.
clock.value+=.05
raw=list(n._native_target_counts); raw[0]+=2000
feed(n,raw=raw); n.PublishExecutionState()
assert not n._native_ready and not n.execution_state_pub.messages[-1].enabled
count=len(n.piper.calls); n.joint_callback(command(n))
assert len(n.piper.calls)==count and not n._command_seen
for _ in range(8): native_feed(n)
assert n._native_ready
checks.append('native readiness uses true six-joint velocity and is withdrawn on pose drift before command entry')

n=native_ready()
for _ in range(1400): native_feed(n)
assert not n._command_latched and n._command_deadline is None
assert sum(k=='enable' for k,_ in n.piper.calls)==1
assert not n._command_seen and not n._native_handoff
n.PublishExecutionState(); state=n.execution_state_pub.messages[-1]
assert state.enabled and not state.command_active and state.valid_until.to_sec()>clock.time()
checks.append('ready arm holds its original target beyond 138s while awaiting MPPI without renewing an external lease')

# Current handoff readiness can be withdrawn after first arrival. It must not
# retroactively turn a long perception wait into a failed startup arrival.
count=len(n.piper.calls)
clock.value+=.1; feed(n,raw=n._native_target_counts)
n._feedback_velocity=(.025,0.,0.,0.,0.,0.)
n._check_command_timeout(); n.PublishExecutionState()
assert n._native_standby_reached and not n._native_ready and not n._command_latched
assert not n.execution_state_pub.messages[-1].enabled
n.joint_callback(command(n))
assert not n._native_handoff and not n._command_seen
assert all(kind=='joint' or kind=='mode' for kind,_ in n.piper.calls[count:])
native_feed(n); assert not n._native_ready
for _ in range(4): native_feed(n)
assert n._native_ready and not n._command_latched
checks.append('late settling drift withdraws handoff without reusing initial60s timeout, then settles again')

n=native_ready(); old_stamp=n._last_command_stamp
for bad in (command(n,stamp=old_stamp),command(n,stamp=clock.time()+.1),command(n,q=[0]*5)):
    n.joint_callback(bad)
assert not n._native_handoff and not n._command_seen and n._command_deadline is None
clock.value+=.01; feed(n,raw=n._native_target_counts)
first=command(n); before=len(n.piper.calls)
n.joint_callback(first)
assert n._native_handoff and n._command_seen and not n._native_ready
assert [k for k,_ in n.piper.calls[before:]]==['mode','joint']
assert n.piper.calls[-2][1][2]==n.fixed_vel==1
assert 0 < n._command_deadline-clock.time() <= .3
n.PublishExecutionState(); state=n.execution_state_pub.messages[-1]
assert state.command_active and state.enabled and state.feedback_valid
for _ in range(2): native_feed(n)
assert len(n.piper.calls)==before+2  # Native sender is permanently gone.
native_feed(n,dt=.11)
assert not n._command_latched and n.GetEnableFlag() and n._native_handoff
assert n._command_deadline is None and len(n.piper.calls)==before+2
n.PublishExecutionState(); state=n.execution_state_pub.messages[-1]
assert state.enabled and state.feedback_valid and not state.command_active
count=len(n.piper.calls); native_feed(n)
assert len(n.piper.calls)==count  # Never resume the native fixed-target sender.
n.joint_callback(command(n)); n.PublishExecutionState()
assert [kind for kind,_ in n.piper.calls[count:]]==['mode','joint']
assert n.execution_state_pub.messages[-1].command_active and n._native_handoff
checks.append('native handoff is permanent; external expiry holds without native fallback; a fresh external command resumes')

n=native_ready(); clock.value+=.01; feed(n,raw=n._native_target_counts)
n.piper.fail_joint=True; n.joint_callback(command(n))
assert n._command_latched and not n._native_handoff and not n._command_seen
assert not n.execution_state_pub.messages[-1].command_active
count=len(n.piper.calls); native_feed(n)
assert len(n.piper.calls)==count
checks.append('failed first external send locks stop, never acknowledges handoff or resumes native target sends')

class FakeTransport:
    class CAN_STATUS(Enum):
        SEND_MESSAGE_SUCCESS=0
        SEND_MESSAGE_FAILED=1
        CAN_BUS_NOT_ACTIVE=2
    def __init__(self):
        self.active=True; self.fail_bus=False; self.sent=[]
        self.bus=NS(send=self.send)
    def send(self, message, timeout=None):
        self.sent.append((message,timeout))
        if self.fail_bus: raise RuntimeError('fake bus.send failure')
    def SendCanMessage(self, identifier, data):
        if not self.active: return self.CAN_STATUS.CAN_BUS_NOT_ACTIVE
        try:
            self.bus.send((identifier,data))
        except Exception:
            return self.CAN_STATUS.SEND_MESSAGE_FAILED  # Installed SDK swallows the exception.
        return self.CAN_STATUS.SEND_MESSAGE_SUCCESS

n=node(); transport=FakeTransport()
n.piper._C_PiperInterface__arm_can=transport
n._configure_command_transport()
assert transport.SendCanMessage(0x155,[0,0]) is transport.CAN_STATUS.SEND_MESSAGE_SUCCESS
transport.bus.send('explicit larger timeout',timeout=.8)
assert [timeout for _,timeout in transport.sent]==[.005,.005]
for failure in ('swallowed_bus_exception','inactive_bus'):
    transport.fail_bus=failure=='swallowed_bus_exception'
    transport.active=failure!='inactive_bus'
    try:
        transport.SendCanMessage(0x155,[0,0])
        raise AssertionError('failed SDK return was accepted: '+failure)
    except RuntimeError as exc:
        assert 'Piper CAN send failed:' in str(exc)
    n=native_ready(); clock.value+=.01; feed(n,raw=n._native_target_counts)
    transport=FakeTransport()
    transport.fail_bus=failure=='swallowed_bus_exception'
    transport.active=failure!='inactive_bus'
    n.piper._C_PiperInterface__arm_can=transport
    n._configure_command_transport()
    def sdk_joint_send(*args, _node=n, _transport=transport):
        _node.piper.calls.append(('joint',args))
        _transport.SendCanMessage(0x155,args[:2])  # SDK JointCtrl does not raise on a failed enum.
    n.piper.JointCtrl=sdk_joint_send
    old_stamp=n._last_command_stamp
    n.joint_callback(command(n))
    assert n._command_latched and not n.GetEnableFlag(), failure
    assert not n._native_handoff and not n._command_seen and n._command_deadline is None, failure
    assert n._last_command_stamp==old_stamp and n.piper.calls[-1][0]=='stop', failure
    assert not n.execution_state_pub.messages[-1].command_active, failure
checks.append('CAN transport forces5ms; SDK swallowed-failure/inactive enums raise before command commit or native handoff')

n=native_ready(); clock.value+=.01; feed(n,raw=n._native_target_counts)
original_send=n.piper.JointCtrl
def delayed_send(*args):
    original_send(*args); clock.value+=.15
n.piper.JointCtrl=delayed_send
n.joint_callback(command(n))
assert n._command_latched and not n._native_handoff and not n._command_seen
assert not n.execution_state_pub.messages[-1].command_active
checks.append('first external CAN send cannot acknowledge stale feedback or extend the pre-send deadline')

for lost in ('joint','status','motor','disabled'):
    n=native_ready()
    for _ in range(610): native_feed(n)
    clock.value+=.16
    feed(n,pairs=() if lost=='joint' else (0,1,2),raw=n._native_target_counts,
         enabled=lost!='disabled',status=lost!='status')
    if lost=='motor':
        n.piper._C_PiperInterface__fps_counter.last_time['ArmMotorDriverInfoLowSpd_6']-=.2
    n._check_command_timeout()
    assert n._command_latched and not n._native_ready, lost
    assert sum(k=='enable' for k,_ in n.piper.calls)==1, lost
    assert not n.execution_state_pub.messages[-1].command_active
checks.append('native pre-handoff still stops on stale joint/status/any motor or dropped motor enable, with no auto re-enable')

n=native_node(); feed(n,enabled=False); n._check_command_timeout()
for _ in range(610):
    native_feed(n,at_target=False)
    if n._command_latched: break
assert n._command_latched and 60 <= clock.time()-n._native_started < 60.2
assert sum(k=='enable' for k,_ in n.piper.calls)==1
checks.append('unreached standby stops at finite60s measured from first enable request')

n=node(); feed(n); assert n._request_enable(True)
n.PublishExecutionState(); assert not n.execution_state_pub.messages[-1].command_active
clock.value+=.05; feed(n); n.joint_callback(command(n)); n.PublishExecutionState()
assert n.execution_state_pub.messages[-1].command_active
clock.value+=.31; feed(n); n.PublishExecutionState()
assert not n.execution_state_pub.messages[-1].command_active
n._check_command_timeout(); n.PublishExecutionState()
assert not n._command_latched and n._command_deadline is None
assert n.execution_state_pub.messages[-1].enabled and n.execution_state_pub.messages[-1].feedback_valid
assert not n.execution_state_pub.messages[-1].command_active
checks.append('ordinary leased mode acknowledges only an unexpired sent command; healthy hold keeps source-based execution feedback')
print('%d groups passed, no SDK/ROS/CAN imports:'%len(checks))
for item in checks: print(' - '+item)
