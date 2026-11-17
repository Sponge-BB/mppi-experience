#!/usr/bin/python3
"""Full adapter -> actual plan with original recorded native-clock coupling.
No ROS nodes or commands. Clear one-sphere test geometry; no hardware prediction.
"""
import argparse,sys,importlib.util,json,time,hashlib,copy
from pathlib import Path
import numpy as np
from types import SimpleNamespace as NS
ROOT=Path('/home/l/remani_ws'); WORK=Path('/tmp/remani_unified_validation_20261114')
parser=argparse.ArgumentParser();parser.add_argument('--variant',choices=['baseline','current'],required=True);parser.add_argument('--output',required=True);parser.add_argument('--extended',action='store_true');args=parser.parse_args()
source_root=WORK/'baseline' if args.variant=='baseline' else ROOT
sys.path.insert(0,str(source_root/'src/mm_remani/scripts/mppi'))
import active_arm_mppi,native_reference_adapter,native_reference
original=ROOT/'experiments/online_remani_validation/results/frozen_guidance_capture_20261114-161009/analysis_rootcause'
spec=importlib.util.spec_from_file_location('coupled_env',original/'coupled_native_clock.py');env=importlib.util.module_from_spec(spec);exec(compile((original/'coupled_native_clock.py').read_text().replace('bool(k._pause_align)', "bool(getattr(k, '_tracking_capture', getattr(k, '_pause_align', False)))"), str(original/'coupled_native_clock.py'), 'exec'), env.__dict__)
output=Path(args.output);output.mkdir(parents=True,exist_ok=True)
fixture=env.load_fixture();gate=json.loads((original/'coupled_recorded_delivery_gate.json').read_text())
class ClearScene:
 def query(self,spheres,times):return np.ones(len(times)),np.zeros(len(times))
def body_spheres_batch(self,q,base):
 s=np.zeros((len(q),1,4));s[:,0,:2]=base[:,:2];s[:,0,3]=.01;return s
env.Model.body_spheres_batch=body_spheres_batch
env.Model.configuration_gaps_batch=lambda self,q: np.ones(len(q))
scene=ClearScene()
def make_adapter(fixture,k):
 a=native_reference_adapter.NativeReferenceAdapter(k,k,resume_distance=env.PARAMS['base_tracking_resume_distance'])
 a.arm_pieces=[]
 for index,block in enumerate(fixture['blocks']):
  pieces=[NS(num_dim=8,num_order=7,duration=row['duration'],data=np.asarray(row['coefficients']).ravel(order='F').tolist()) for row in block]
  msg=NS(action=1,trajectory_id=index+1,singul=block[0]['gear'],trajectory=pieces,header=NS(frame_id='world',stamp=NS(to_sec=lambda:fixture['native_start'])))
  msg.serialize=lambda stream:stream.write(b'offline fixture')
  assert a.native.feed(msg,fixture['native_start']-1.) in ('replace','append')
  a.arm_pieces.extend((row['duration'],np.asarray(row['coefficients'])[2:]) for row in block)
 return a
env.make_adapter=make_adapter
def actual_plan(state,a,poly,phase,scale,t,k):
 now=fixture['native_start']+t
 if not a.active:
  a.active=True;a.goal_stamp=fixture['native_start']-.1;a.goal_received=a.goal_stamp
  a.goal_pose=np.array(fixture['goal']);a.native.start=fixture['native_start'];a.native.changed=a.native.start-1.
  a.native.last_id=len(a.native.refs);a.native.epoch=1;a.reason='native_reference_ready'
 a.clock=lambda:now;a.progress=(now,phase,scale)
 result,info=a.plan(state,np.zeros((2,4)),state[5:11],scene,now)
 actual_plan.calls+=1
 if result is None:
  actual_plan.failures.append(dict(t=t,phase=phase,scale=scale,info=info));return np.zeros(2)
 return np.array(env.base_command(result[0][0],k.limits))
cases=[('ideal',fixture,None,120.),('recorded_gate',fixture,gate,float(gate[-1]['t']))]
if args.extended:
 for label,dx,dy,yaw in [('lateral_positive',0.,.08,0.),('lateral_negative',0.,-.08,0.),('yaw_positive',0.,0.,.6),('yaw_negative',0.,0.,-.6)]:
  f=copy.deepcopy(fixture);f['state'][0]+=dx;f['state'][1]+=dy;f['state'][2]+=yaw
  cases.append((label,f,None,120.))
summary=[]
for name,fixture,events,seconds in cases:
 actual_plan.calls=0;actual_plan.failures=[];began=time.monotonic()
 result=env.simulate(fixture=fixture,guide=actual_plan,delivery_events=events,max_seconds=seconds)
 trace=result.pop('trace');result['wall_seconds']=time.monotonic()-began;result['variant']=args.variant
 result['scope']='Full production NativeReferenceAdapter.plan -> WholeBodyMPPI.plan -> original independent 5ms bounded/lagged Plant; real two-gear polynomial and original 100Hz native phase clock. Clear one-sphere scene, ideal fixed arm, no full robot collision/map/replanning/terminal FSM. Recorded gates copied by source walltime when specified, no new actuator authority or ROS publishing. This is not hardware validation.'
 result['actual_plan_calls']=actual_plan.calls;result['actual_plan_failures']=actual_plan.failures
 result['source_sha256']={str(p.relative_to(source_root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [source_root/'src/mm_remani/scripts/mppi/active_arm_mppi.py',source_root/'src/mm_remani/scripts/mppi/native_reference_adapter.py']}
 (output/(name+'_report.json')).write_text(json.dumps(result,indent=2)+'\n')
 (output/(name+'_trace.jsonl')).write_text(''.join(json.dumps(row,separators=(',',':'))+'\n' for row in trace))
 selected={k:result[k] for k in ['clock_completed','elapsed','final_phase','final_goal_position_error','freeze_events','resume_events','frozen_seconds','scale_transition_command_flips','nonzero_sign_changes','nonzero_sign_changes_scale_mode','nonzero_sign_changes_endpoint_2_5cm','wall_seconds']};selected.update(name=name,actual_plan_calls=actual_plan.calls,actual_plan_failures=len(actual_plan.failures));summary.append(selected);print(json.dumps(selected),flush=True)
(output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
