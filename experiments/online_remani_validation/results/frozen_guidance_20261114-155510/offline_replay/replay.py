#!/usr/bin/python3
"""Offline frozen proposal replay and independent-plant closed-loop checks."""
import argparse,ast,collections,hashlib,inspect,json,math,sys,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import yaml
ROOT=Path('/home/l/remani_ws')
P=argparse.ArgumentParser();P.add_argument('--modules',required=True);P.add_argument('--output',required=True);P.add_argument('--closed-loop',action='store_true');args=P.parse_args()
sys.path.insert(0,args.modules)
from active_arm_mppi import WholeBodyMPPI,wrap
from native_reference_adapter import NativeReferenceAdapter
HAS_TRACKING='tracking' in inspect.signature(WholeBodyMPPI.proposals).parameters
INPUT=Path(__file__).resolve().with_name('inputs.json')
data=json.loads(INPUT.read_text());cfg=yaml.safe_load((ROOT/'src/mm_remani/config/mppi.yaml').read_text())

def definitions(file,names,namespace):
 tree=ast.parse(file.read_text());nodes=[n for n in tree.body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name in names or isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id in names for t in n.targets)]
 exec(compile(ast.Module(body=nodes,type_ignores=[]),str(file),'exec'),namespace);return namespace
base_command=definitions(ROOT/'src/mm_remani/scripts/whole_body_mppi.py',{'base_command'},{})['base_command']
plant_ns=definitions(ROOT/'src/mm_remani/scripts/standby_simulator.py',{'Plant','Q_START','STEP'},{'np':np,'math':math})
Plant=plant_ns['Plant']
class Model:
 lower,upper=np.full(6,-4.),np.full(6,4.)
 velocity_limits,acceleration_limits=np.ones(6),np.full(6,2.)
 def check(self,q):return {'valid':bool(np.all((q>=self.lower)&(q<=self.upper)))}
 def body_spheres_batch(self,q,base):
  out=np.zeros((len(q),1,4));out[:,0,:3]=base;out[:,:,3]=.01;return out
 def configuration_gaps_batch(self,q):return np.ones(len(q))
class Clear:
 def query(self,spheres,times):return np.ones(len(times)),np.zeros(len(times))
def kernel():
 m=Model();return WholeBodyMPPI(m,dict(cfg['base_limits']),(m.lower,m.upper,np.asarray(cfg['arm_velocity']),np.asarray(cfg['arm_acceleration'])),[],horizon=cfg['horizon'],step=1./cfg['rate_hz'],random_samples=cfg['random_samples'])
def adapter_for(row,k):
 a=NativeReferenceAdapter(k,k,**({'resume_distance':.05} if HAS_TRACKING else {}));raw=data['natives'][row['native']]
 a.native.refs=[np.asarray(x) for x in raw['refs']];a.native.times=[np.asarray(x) for x in raw['times']];a.native.gears=raw['gears'];a.native.start=raw['start'];a.arm_pieces=[(t,np.asarray(c)) for t,c in raw['arm_pieces']];return a

def delivered(u,k,limit):
 out=np.array(base_command(u,k.limits))
 if limit is not None:
  if limit<=1e-3:out[:]=0
  else:out[0]=np.clip(out[0],-limit,limit)
 return out

def tracking(row,a,block,scale=0.):
 return dict(tracking=dict(gear=a.native.gears[block],pause_key=(row['native'],block,row['review']['phase']) if scale==0. else None,resume_distance=.05)) if HAS_TRACKING else {}

def metrics(rows):
 windows=[(0,240.116,'all'),(116.245,177.75,'long_yaw_freeze'),(184.695,203.445,'third_yaw_freeze'),(207.545,235.045,'late_position_freeze')]
 out=[]
 for lo,hi,name in windows:
  rr=[r for r in rows if lo<=r['rel']<=hi];flips=[]
  for prev,cur in zip(rr,rr[1:]):
   if prev['phase']==cur['phase'] and prev['command'][1]*cur['command'][1]<0 and min(abs(prev['command'][1]),abs(cur['command'][1]))>=.05:flips.append((prev,cur))
  out.append(dict(label=name,rows=len(rr),observed_command_matches=sum(r['observed_match'] for r in rr),samephase_flips=len(flips),samephase_flips_crossing_2_5cm=sum((a['distance']<.025)!=(b['distance']<.025) for a,b in flips),command_opposes_reference_yaw=sum(r['command'][1]*r['yaw_error']<-.001 for r in rr),max_abs_w=max(abs(r['command'][1]) for r in rr),max_abs_v=max(abs(r['command'][0]) for r in rr)))
 return out

def replay():
 k=kernel();rows=[]
 for r in data['rows']:
  s=np.array(r['state']);a=adapter_for(r,k);phase=r['review']['phase'];reference,block,end=a._base_reference(phase,0.,s[2]);arm=a._arm_samples(np.full(k.H,phase));controls,labels=k.proposals(s,reference,arm,[(1.,0.)],**tracking(r,a,block));u=delivered(controls[0,0],k,r['speed_limit']);obs=np.array(r['observed']);rr=r['review']['reconstructed_reference'];err=float(wrap(rr['yaw']-s[2]))
  rows.append(dict(rel=r['review']['rel'],phase=phase,native_gear=a.native.gears[block],distance=rr['distance_m'],yaw_error=err,command=u.tolist(),observed_match=bool(np.max(np.abs(u-obs))<1e-9),observed_max_error=float(np.max(np.abs(u-obs))),finite=bool(np.isfinite(controls).all())))
 return dict(summary=metrics(rows),all_finite=all(r['finite'] for r in rows),max_observed_error=max(r['observed_max_error'] for r in rows),rows=rows)

def closed_loop():
 cases=[]
 # Measured states at representative yaw freeze and position freeze onset.
 for lo,hi,name in [(116.245,177.75,'recorded_long_yaw'),(184.695,203.445,'recorded_third_yaw'),(207.545,235.045,'recorded_late_position')]:
  row=next(r for r in data['rows'] if lo<=r['review']['rel']<=hi);cases.append((name,row,np.asarray(row['state']),False))
 # All orientations with the same signed along/lateral offsets; +/- pi wrap.
 seed=data['rows'][0]
 for gear in (-1,1):
  for yaw in (0.,math.pi-.01,-math.pi+.01):
   for along,side,err in ((.024,0.,.7),(.026,0.,.7),(.025,.10,.7),(.12,.08,.7),(-.12,-.08,-.7)):
    refyaw=yaw;target=np.array([0.,0.,refyaw]);delta=along*np.array([math.cos(yaw),math.sin(yaw)])+side*np.array([-math.sin(yaw),math.cos(yaw)])
    s=np.r_[-delta,wrap(yaw-err),0.,0.,plant_ns['Q_START'],np.zeros(6)]
    raw=dict(refs=[[[0.,0.,yaw,gear*.05],[.01*gear*math.cos(yaw),.01*gear*math.sin(yaw),yaw,gear*.05]]],times=[[0.,1.]],gears=[gear],start=1.,arm_pieces=[[1.,np.c_[np.zeros((6,7)),plant_ns['Q_START']].tolist()]])
    key='synthetic_%d'%len(cases);data['natives'][key]=raw;r=dict(native=key,review=dict(phase=0.),speed_limit=None)
    cases.append(('gear_%s_yaw_%+.2f_along_%+.3f_side_%+.2f'%(gear,yaw,along,side),r,s,abs(side)<1e-8))
 out=[]
 for name,row,state,jitter in cases:
  k=kernel();a=adapter_for(row,k);plant=Plant(k.model,k.limits,k.vmax,k.amax);plant.state=state.copy();phase=row['review']['phase'];ref,_,_=a._base_reference(phase,0.,state[2]);target=np.array(ref[0,:3]);trace=[]
  # 20 s fixed-phase hold, with deterministic 2 mm measurement jitter only in
  # threshold fixtures. The plant itself is never teleported by this noise.
  for n in range(200):
   measured=plant.state.copy()
   if jitter:measured[:2]+=(-1 if n%2 else 1)*.002*np.array([math.cos(target[2]),math.sin(target[2])])
   ref,block,_=a._base_reference(phase,0.,measured[2]);arm=a._arm_samples(np.full(k.H,phase));result,info=k.plan(measured,ref,arm,Clear(),n*.1,**tracking(row,a,block))
   if result is None:raise AssertionError((name,n,info))
   control=result[0][0];u=delivered(control,k,row['speed_limit'])
   # Base is independently integrated at 5 ms. Arms held in this base-only
   # fixture, keeping collision query/plan active but outside hardware claims.
   for _ in range(20):plant.step(u,plant.state[5:11].copy(),.005)
   delta=target[:2]-plant.state[:2];trace.append(dict(t=(n+1)*.1,xy=float(np.linalg.norm(delta)),yaw=abs(float(wrap(target[2]-plant.state[2]))),v=float(u[0]),w=float(u[1]),mode=info['mode']))
  initial_xy=float(np.linalg.norm(target[:2]-state[:2]));initial_yaw=abs(float(wrap(target[2]-state[2])))
  flips=sum(a['w']*b['w']<0 and min(abs(a['w']),abs(b['w']))>=.05 for a,b in zip(trace,trace[1:]))
  ready=[r['t'] for r in trace if r['xy']<=.05 and r['yaw']<=.15]
  out.append(dict(name=name,initial_xy_m=initial_xy,initial_yaw_rad=initial_yaw,final_xy_m=trace[-1]['xy'],final_yaw_rad=trace[-1]['yaw'],max_xy_m=max(r['xy'] for r in trace),w_flips_ge_005=flips,all_nominal=all(r['mode']=='nominal' for r in trace),freeze_exit_ready_first_s=min(ready) if ready else None,final_freeze_exit_ready=trace[-1]['xy']<=.05 and trace[-1]['yaw']<=.15,trace=trace))
 return dict(scope='Actual kernel.plan selected controls; clear synthetic collision query/small geometry and independent production 5 ms Plant. Base numerical behavior only, no hardware or captured-map validation.',cases=out)

def boundary_and_motion():
 # Retain the troublesome recorded bearing/yaw relationship while crossing
 # only the 2.5 cm distance boundary. These are explicit synthetic readings.
 row=next(r for r in data['rows'] if 120.<r['review']['rel']<120.1);k=kernel();a=adapter_for(row,k);state=np.array(row['state']);phase=row['review']['phase'];ref,block,_=a._base_reference(phase,0.,state[2]);target=np.array(ref[0,:3]);radial=(target[:2]-state[:2]);radial/=np.linalg.norm(radial);state[3:5]=0.;commands=[]
 for distance in [.024,.026]*10:
  measured=state.copy();measured[:2]=target[:2]-distance*radial
  controls,_=k.proposals(measured,ref,a._arm_samples(np.full(k.H,phase)),[(1.,0.)],**tracking(row,a,block));commands.append(delivered(controls[0,0],k,None).tolist())
 boundary=dict(distances_m=[.024,.026]*10,commands=commands,w_flips_ge_005=sum(p[1]*q[1]<0 and min(abs(p[1]),abs(q[1]))>=.05 for p,q in zip(commands,commands[1:])))
 motion=[]
 for gear in (-1,1):
  for scale in (.01,.1,1.):
   key='motion_%s_%s'%(gear,scale);q=plant_ns['Q_START'];data['natives'][key]=dict(refs=[np.c_[gear*np.linspace(0.,1.,101),np.zeros((101,2)),np.full(101,gear*.05)].tolist()],times=[np.linspace(0.,20.,101).tolist()],gears=[gear],start=1.,arm_pieces=[[20.,np.c_[np.zeros((6,7)),q].tolist()]])
   row=dict(native=key,review=dict(phase=0.),speed_limit=None);k=kernel();a=adapter_for(row,k);plant=Plant(k.model,k.limits,k.vmax,k.amax);trace=[]
   for n in range(50):
    phase=n*.1*scale;ref,block,_=a._base_reference(phase,scale,plant.state[2]);arm=a._arm_samples(phase+scale*.1*np.arange(1,k.H+1));result,info=k.plan(plant.state.copy(),ref,arm,Clear(),n*.1,**tracking(row,a,block,scale));assert result is not None
    u=delivered(result[0][0],k,None)
    for _ in range(20):plant.step(u,plant.state[5:11].copy(),.005)
    trace.append(dict(v=float(u[0]),w=float(u[1]),x=float(plant.state[0]),yaw=float(plant.state[2]),mode=info['mode']))
   motion.append(dict(gear=gear,time_scale=scale,scaled_speed_mps=gear*.05*scale,first_command=[trace[0]['v'],trace[0]['w']],final_x_m=trace[-1]['x'],max_abs_yaw_rad=max(abs(r['yaw']) for r in trace),all_command_direction_correct=all(r['v']*gear>0. for r in trace),all_nominal=all(r['mode']=='nominal' for r in trace),trace=trace))
 return dict(boundary=boundary,motion=motion)
start=time.monotonic();out=dict(modules=args.modules,has_tracking=HAS_TRACKING,input_sha256=hashlib.sha256(INPUT.read_bytes()).hexdigest(),source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(args.modules).glob('*.py')},replay=replay(),extra=boundary_and_motion())
if args.closed_loop:out['closed_loop']=closed_loop()
out['wall_seconds']=time.monotonic()-start;Path(args.output).write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(dict(output=args.output,wall_seconds=out['wall_seconds'],replay=out['replay']['summary'],closed_loop=[{k:v for k,v in c.items() if k!='trace'} for c in out.get('closed_loop',{}).get('cases',[])]),indent=2))
