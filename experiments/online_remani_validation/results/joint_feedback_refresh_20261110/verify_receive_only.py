#!/usr/bin/python3
"""Compare feedback acceptance on real CAN input, with every CAN send blocked."""
import ast
from collections import Counter
import json
import math
from pathlib import Path
import resource
import threading
import time
from types import SimpleNamespace as NS
resource.setrlimit(resource.RLIMIT_AS, (768*1024**2, 768*1024**2))
from can.interfaces.socketcan.socketcan import SocketcanBus
from piper_sdk import C_PiperInterface
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
sends=[]
def refuse_send(*args,**kwargs):
    sends.append(time.time())
    raise RuntimeError('Receive-only probe refuses CAN sends')
SocketcanBus.send=refuse_send
SocketcanBus._send_once=refuse_send
SocketcanBus._send_periodic_internal=refuse_send
class Stamp:
    def __init__(self,value=0.): self.value=value
    def to_sec(self): return self.value
    @classmethod
    def from_sec(cls,value): return cls(value)
class JointState:
    def __init__(self):
        self.header=NS(stamp=Stamp());self.name=[];self.position=[];self.velocity=[]
class Pub:
    def __init__(self): self.count=0
    def publish(self,msg): self.count+=1

def load(path,arm):
    tree=ast.parse(path.read_text())
    scope={'threading':threading,'math':math,'time':time,'JointState':JointState,'Bool':NS,
           'rospy':NS(Time=Stamp)}
    exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n,(ast.ClassDef,ast.Assign))],
        type_ignores=[]),str(path),'exec'),scope)
    cls=scope['C_PiperRosNode'];n=cls.__new__(cls);n.piper=arm
    n._command_lock=threading.RLock();n._feedback_stamp=0.;n._feedback_monotonic=0.
    n._feedback_position=None;n._feedback_velocity=None;n._last_joint_counts=(0,0,0)
    n._previous_feedback=None;n.gripper_exist=False;n.joint_pub=Pub()
    return n

def counters():
    return {k:int((Path('/sys/class/net/can1/statistics')/k).read_text()) for k in
            ['rx_packets','rx_errors','rx_dropped','tx_packets','tx_errors']}

before=counters();start=time.monotonic();cpu=time.process_time()
arm=C_PiperInterface(can_name='can1');transport=arm._C_PiperInterface__arm_can
transport.SendCanMessage=refuse_send
arm.ConnectPort(start_thread=False,piper_init=False)
nodes={'before':load(HERE/'piper_ctrl_single_node.py.before_fix',arm),
       'after':load(ROOT/'src/piper_ros/src/piper/scripts/piper_ctrl_single_node.py',arm)}
done=threading.Event();counts=Counter();failures=[];checked=Counter();mismatches=Counter();delayed=Counter()
def receive():
    while not done.is_set():
        try:
            msg=transport.bus.recv(timeout=.02)
            if msg is not None:
                arm.ParseCANFrame(msg);counts[msg.arbitration_id]+=1
        except Exception as exc:
            failures.append(str(exc));done.set()
thread=threading.Thread(target=receive,daemon=True);thread.start()
next_check=0.
try:
    while time.monotonic()-start<10 and not done.is_set() and not sends:
        elapsed=time.monotonic()-start
        # Every second, omit telemetry publication for 350 ms; CAN still receives.
        delayed_publication=.45<elapsed%1<.80
        if not delayed_publication:
            for n in nodes.values(): n.PublishArmJointAndGripper()
        if elapsed>1 and elapsed>=next_check:
            next_check=elapsed+.02
            for label,n in nodes.items():
                ok=n._feedback_fresh();checked[label]+=1
                if not ok:mismatches[label]+=1
                if delayed_publication and not ok:delayed[label]+=1
        time.sleep(.005)
    done.set();thread.join(timeout=.5)
    time.sleep(.17)
    stale_rejected={label:not n._feedback_fresh() for label,n in nodes.items()}
finally:
    done.set();thread.join(timeout=.5);arm.DisconnectPort()
after=counters()
report={'duration_s':time.monotonic()-start,'cpu_one_core_percent':(time.process_time()-cpu)/(time.monotonic()-start)*100,
        'frames':sum(counts.values()),'joint_counts':{hex(k):counts[k] for k in [0x2a5,0x2a6,0x2a7]},
        'checks':dict(checked),'unavailable_checks':dict(mismatches),'unavailable_during_publication_delay':dict(delayed),
        'reject_after_receiving_stops':stale_rejected,'parse_failures':failures,'send_attempts_blocked':len(sends),
        'counter_delta':{k:after[k]-before[k] for k in before},
        'rss_max_kib':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss}
(HERE/'receive_only_result.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
if sends or failures or not counts[0x2a5] or not all(stale_rejected.values()):raise SystemExit(1)
