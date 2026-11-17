#!/usr/bin/python3
"""Full adapter -> actual plan with original recorded native-clock coupling.
No ROS nodes or commands. Clear one-sphere test geometry; no hardware prediction.
"""
import argparse,sys,importlib.util,json,time,hashlib,copy
from pathlib import Path
import numpy as np
from types import SimpleNamespace as NS
ROOT=Path('/home/l/remani_ws'); WORK=Path('/tmp/remani_unified_validation_20261114')
parser=argparse.ArgumentParser();parser.add_argument('--variant',choices=['baseline','current'],required=True);parser.add_argument('--output',required=True);parser.add_argument('--extended',action='store_true');parser.add_argument('--capacity-clock',action='store_true');args=parser.parse_args()
source_root=ROOT/'rollback/20261114-163406-before-unified-reference/files' if args.variant=='baseline' else ROOT
sys.path.insert(0,str(source_root/'src/mm_remani/scripts/mppi'))
import active_arm_mppi,native_reference_adapter,native_reference
original=Path(__file__).resolve().parent/'fixture'
if not original.exists(): original=ROOT/'experiments/online_remani_validation/results/frozen_guidance_capture_20261114-161009/analysis_rootcause'
spec=importlib.util.spec_from_file_location('coupled_env',original/'coupled_native_clock.py');env=importlib.util.module_from_spec(spec);exec(compile((original/'coupled_native_clock.py').read_text().replace('bool(k._pause_align)', "bool(getattr(k, '_tracking_capture', getattr(k, '_pause_align', False)))"), str(original/'coupled_native_clock.py'), 'exec'), env.__dict__)
class CapacityClock(env.NativeBaseClock):
 """Transcription of the C++ base execution capacity addition, not a tuned clock."""
 instances=[]
 def __init__(self,poly):
  super().__init__(poly);self.capacity_statistics=dict(samples=0,limited_samples=0,max_published_demand=0.,violations=0,max_published_linear=0.,max_published_angular=0.,max_published_wheel=0.,max_linear_excess=0.,max_angular_excess=0.,max_wheel_excess=0.)
  self.acceleration=[np.array([np.polyder(c,2) for c in a[:2]]) for a in poly.coeff]
  self.instances.append(self)
 def capacity_at(self,phase):
  i=min(len(self.poly.ends)-1,int(np.searchsorted(self.poly.ends,phase,side='left')))
  local=phase-(0. if i==0 else self.poly.ends[i-1])
  velocity=np.array([np.polyval(c,local) for c in self.poly.velocity[i]])
  acceleration=np.array([np.polyval(c,local) for c in self.acceleration[i]])
  speed=float(np.linalg.norm(velocity));omega=float(np.cross(velocity,acceleration)/speed**2) if speed>env.PARAMS['yaw_hold_speed'] else 0.
  limits=env.CFG['base_limits']
  demand=max(speed/limits['vmax'],abs(omega)/limits['wmax'],(speed+.5*limits['track']*abs(omega))/limits['wheel_v'])
  self.raw_capacity_components=(speed,abs(omega),speed+.5*limits['track']*abs(omega))
  return 1./max(1.,demand),demand
 def step(self,state,dt,t):
  p,v,speed,yaw,gear=self.poly.sample(self.phase)
  dist=float(np.linalg.norm(p-state[:2]));err=abs(float(env.wrap(yaw-state[2]))) if speed>env.PARAMS['yaw_hold_speed'] else 0.
  prev=self.hold
  if self.hold and dist<=env.PARAMS['base_tracking_resume_distance'] and err<=env.PARAMS['base_tracking_resume_yaw']:self.hold=False
  elif not self.hold and (dist>=env.PARAMS['base_tracking_stop_distance'] or err>=env.PARAMS['base_tracking_stop_yaw']):self.hold=True
  if prev!=self.hold:self.events.append(dict(t=t,event='freeze' if self.hold else 'resume',phase=self.phase,position_error=dist,yaw_error=err,gear=gear))
  capacity,_=self.capacity_at(self.phase)
  if self.hold:self.scale=0.
  else:
   scales=[capacity]
   for error,stem in [(dist,'distance'),(err,'yaw')]:
    slow=env.PARAMS['base_tracking_slow_'+stem];stop=env.PARAMS['base_tracking_stop_'+stem]
    ratio=max(0.,min(1.,(stop-error)/(stop-slow)))
    scales.append(1. if error<=slow else env.PARAMS['base_tracking_min_time_scale']+ratio*(1.-env.PARAMS['base_tracking_min_time_scale']))
   target=min(scales);rate=env.PARAMS['virtual_time_scale_rise_rate'] if target>self.scale else env.PARAMS['virtual_time_scale_fall_rate']
   self.scale=max(self.scale-rate*dt,min(self.scale+rate*dt,target))
  self.scale=min(self.scale,capacity)
  self.phase=min(self.poly.total,self.phase+dt*self.scale)
  next_capacity,demand=self.capacity_at(self.phase)
  self.scale=min(self.scale,next_capacity)
  stats=self.capacity_statistics;stats['samples']+=1;stats['limited_samples']+=int(next_capacity<1.)
  stats['max_published_demand']=max(stats['max_published_demand'],self.scale*demand)
  stats['violations']+=int(self.scale*demand>1.+1e-10)
  for component,value,limit in zip(('linear','angular','wheel'),self.raw_capacity_components,(env.CFG['base_limits']['vmax'],env.CFG['base_limits']['wmax'],env.CFG['base_limits']['wheel_v'])):
   stats['max_published_'+component]=max(stats['max_published_'+component],self.scale*value)
   stats['max_'+component+'_excess']=max(stats['max_'+component+'_excess'],self.scale*value-limit)
  return dist,err
if args.capacity_clock:env.NativeBaseClock=CapacityClock
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
 actual_plan.calls=0;actual_plan.failures=[];CapacityClock.instances=[];began=time.monotonic()
 result=env.simulate(fixture=fixture,guide=actual_plan,delivery_events=events,max_seconds=seconds)
 trace=result.pop('trace');result['wall_seconds']=time.monotonic()-began;result['variant']=args.variant
 result['scope']='Full production NativeReferenceAdapter.plan -> WholeBodyMPPI.plan -> original independent 5ms bounded/lagged Plant; real two-gear polynomial and original 100Hz native phase clock. Clear one-sphere scene, ideal fixed arm, no full robot collision/map/replanning/terminal FSM. Recorded gates copied by source walltime when specified, no new actuator authority or ROS publishing. This is not hardware validation.'
 result['actual_plan_calls']=actual_plan.calls;result['actual_plan_failures']=actual_plan.failures
 if args.capacity_clock:
  result['scope']=result['scope'].replace('original 100Hz native phase clock','capacity-limited 100Hz native phase clock transcription')
  result['capacity_clock']=dict(limits=env.CFG['base_limits'],statistics=CapacityClock.instances[-1].capacity_statistics,scope='Base-only transcription of source execution_scale: raw polynomial derivatives, same low-speed heading convention, target minimum, post-smoothing hard cap, and new-phase publication cap. Original tracking thresholds/rates preserved. Numerical code does not execute the C++ ROS controller process.')
 result['source_sha256']={str(p.relative_to(source_root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [source_root/'src/mm_remani/scripts/mppi/active_arm_mppi.py',source_root/'src/mm_remani/scripts/mppi/native_reference_adapter.py']}
 if args.capacity_clock:
  for relative in ('src/mm_controller/src/mm_controller_fsm.cpp','src/mm_controller/include/mm_controller/mm_controller_fsm.hpp','src/mm_remani/launch/exp0_real.launch','src/mm_remani/scripts/mppi/native_reference.py','src/mm_remani/scripts/whole_body_mppi.py','src/mm_remani/config/mppi.yaml'):
   result['source_sha256'][relative]=hashlib.sha256((ROOT/relative).read_bytes()).hexdigest()
  commands=np.asarray([row['command'] for row in trace]);limits=env.CFG['base_limits']
  result['actual_command_peaks']=dict(linear=float(np.max(np.abs(commands[:,0]))),angular=float(np.max(np.abs(commands[:,1]))),wheel=float(np.max(np.abs(commands[:,0])+.5*limits['track']*np.abs(commands[:,1]))))
 (output/(name+'_report.json')).write_text(json.dumps(result,indent=2)+'\n')
 (output/(name+'_trace.jsonl')).write_text(''.join(json.dumps(row,separators=(',',':'))+'\n' for row in trace))
 selected={k:result[k] for k in ['clock_completed','elapsed','final_phase','final_goal_position_error','freeze_events','resume_events','frozen_seconds','scale_transition_command_flips','nonzero_sign_changes','nonzero_sign_changes_scale_mode','nonzero_sign_changes_endpoint_2_5cm','wall_seconds']};selected.update(name=name,actual_plan_calls=actual_plan.calls,actual_plan_failures=len(actual_plan.failures))
 if args.capacity_clock:selected['capacity_statistics']=result['capacity_clock']['statistics']
 summary.append(selected);print(json.dumps(selected),flush=True)
(output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
