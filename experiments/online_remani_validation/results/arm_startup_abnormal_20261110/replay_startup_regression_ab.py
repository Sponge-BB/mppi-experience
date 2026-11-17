import os
for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'): os.environ[k]='1'
import ast, collections, json, math, threading, time
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
ROOT=Path('/home/l/remani_ws')
OUT=ROOT/'experiments/online_remani_validation/results/arm_startup_abnormal_20261110'
EVENTS=ROOT/'experiments/online_remani_validation/results/startup_listen_20261110_200318/events.jsonl'
clock=[0.]
class Pub:
 def __init__(self): self.messages=[]
 def publish(self, m): self.messages.append(m)
class Stamp:
 def __init__(self, t): self.t=t
 def to_sec(self): return self.t
class Twist:
 def __init__(self): self.linear=NS(x=0.,y=0.,z=0.);self.angular=NS(x=0.,y=0.,z=0.)
class String:
 def __init__(self,data):self.data=data
class Bool(String):pass
class Reference:
 def __init__(self):
  self.lock=threading.RLock();self.active=False;self.goal_stamp=None;self.calls=0
 def plan(self,*args,**kw):
  self.calls+=1
  raise RuntimeError('Unexpected planning entry before native standby')

def load(source):
 parsed=ast.parse(source.read_text())
 keep=[]
 for item in parsed.body:
  if isinstance(item,(ast.FunctionDef,ast.ClassDef)):keep.append(item)
  elif isinstance(item,ast.Assign) and any(isinstance(t,ast.Name) and t.id in ('JOINTS','REFERENCE_TRANSITION_REJECTIONS','LOCAL_RETRY_REJECTIONS','REFERENCE_WAIT_HOLD_REASONS') for t in item.targets):keep.append(item)
 env={'np':np,'math':math,'time':time,'json':json,'threading':threading,'Twist':Twist,'String':String,'Bool':Bool,'rospy':NS(get_time=lambda:clock[0],loginfo=lambda *a:None,logwarn=lambda *a:None)}
 exec(compile(ast.Module(body=keep,type_ignores=[]),str(source),'exec'),env)
 return env['WholeBodyNode']

def run(cls,events):
 node=cls.__new__(cls); node.lock=threading.RLock();node.execute=True;node.simulation=False
 node.startup_complete=False;node.native_arm_startup=True;node.native_handoff_sent=False;node.native_handoff_stamp=0.
 node.startup_target=np.deg2rad([0,29.85,-60.098,-.002,54.276,.003]);node.startup_settled_since=None
 node.enable_feedback_stamp=node.enable_feedback_until=node.execution_stamp=node.execution_valid_until=0.
 node.execution_enabled=node.execution_latched=node.execution_command_active=False;node.execution_revision=0
 node.sent_command=False;node.sent_navigation_goal=None;node.reference=Reference();node.joints=None;node.joint_stamp=0.
 node.kernel=None;node.base=None;node.snapshot=None;node.base_stamp=0.;node.anchor=node.arm_anchor=None;node.input_errors={}
 node.model=NS(lower=np.full(6,-np.inf),upper=np.full(6,np.inf))
 # Permissive joint bounds isolate the regression being tested; real on_joints
 # still requires complete finite position/velocity and true source stamps.
 for name in ('base_pub','arm_pub','enable_pub','fault_pub','info_pub'):setattr(node,name,Pub())
 counts=collections.Counter();statuses=[];exec_values=collections.Counter()
 for item in events:
  clock[0]=item['received'];topic=item['topic']
  if topic=='/piper_execution_state':
   msg=NS(**{k:item[k] for k in ('enabled','command_active','stop_latched','feedback_valid')},header=NS(stamp=Stamp(item['stamp'])),valid_until=Stamp(item['valid_until']))
   node.on_execution_state(msg);counts['execution_samples']+=1;exec_values[str((item['enabled'],item['feedback_valid'],item['stop_latched']))]+=1
  elif topic=='/joint_states_single':
   msg=NS(name=item['names'],position=item['position'],velocity=item['velocity'],header=NS(stamp=Stamp(item['stamp'])))
   node.on_joints(msg);counts['joint_samples']+=1
  elif topic=='/mppi/status':
   node.cycle();counts['cycle_calls']+=1;statuses.append(json.loads(node.info_pub.messages[-1].data)['state'])
 result=dict(counts)
 result.update(arm_publications=len(node.arm_pub.messages),enable_publications=len(node.enable_pub.messages),fault_publications=len(node.fault_pub.messages),plan_calls=node.reference.calls,status_counts=dict(collections.Counter(statuses)),execution_combinations=dict(exec_values),base_nonzero=sum(bool(m.linear.x or m.linear.y or m.angular.z) for m in node.base_pub.messages),startup_complete=node.startup_complete)
 return result,statuses

items=[json.loads(x) for x in EVENTS.read_text().splitlines()]
# Separate actual startup episodes at the > 1 minute logging gap.
runs=[]
for item in items:
 if not runs or item['received']-runs[-1][-1]['received']>60:runs.append([])
 runs[-1].append(item)
versions={'before':ROOT/'experiments/online_remani_validation/results/navigation_rear_20261110/before/whole_body_mppi.py','current':ROOT/'src/mm_remani/scripts/whole_body_mppi.py'}
report={'scope':'Offline replay of actual captured source-stamped execution/joint messages and original recorded cycle times; actual AST class methods for cycle/native_startup_gate/on_execution_state/on_joints/reject; no ROS or CAN imported/started. Joint bounds intentionally permissive to isolate execution gate; no hardware firmware/physical model claim.','runs':[]}
for i,events in enumerate(runs):
 if not any(x['topic']=='/mppi/status' for x in events):continue
 entry={'run':i+1,'start':events[0]['received'],'end':events[-1]['received']};seq={}
 for name,path in versions.items(): entry[name],seq[name]=run(load(path),events)
 entry['identical_status_sequence']=seq['before']==seq['current'];entry['identical_summary']=entry['before']==entry['current']
 assert entry['identical_status_sequence'] and entry['identical_summary']
 assert entry['current']['arm_publications']==entry['current']['enable_publications']==entry['current']['plan_calls']==entry['current']['base_nonzero']==0
 report['runs'].append(entry)
(OUT/'startup_before_after_replay.json').write_text(json.dumps(report,indent=2)+'\n')
(OUT/'replay_startup_regression_ab.py').write_text(Path(__file__).read_text())
print(json.dumps(report,indent=2))
