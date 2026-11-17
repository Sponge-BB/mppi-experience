import sys,importlib.util,json,hashlib,time
from pathlib import Path
import numpy as np
root=Path('/home/l/remani_ws');case=root/'experiments/online_remani_validation/results/unified_reference_20261114-163406';scripts=root/'src/mm_remani/scripts/mppi'
sys.path.insert(0,str(scripts));import active_arm_mppi as new
sp=importlib.util.spec_from_file_location('review_previous_mppi',root/'rollback/20261114-163406-before-unified-reference/files/src/mm_remani/scripts/mppi/active_arm_mppi.py');old=importlib.util.module_from_spec(sp);sys.modules[sp.name]=old;sp.loader.exec_module(old)
limits=dict(vmax=.25,wmax=.35,track=.37,wheel_v=.25,accel=.25,angular_accel=.8,wheel_a=.2975)
class Model: pass
def kernel(mod=new):
 k=mod.WholeBodyMPPI(Model(),limits,([-3]*6,[3]*6,[.15]*6,[.35]*6),[],enabled=False,horizon=5,random_samples=0);k._legacy_native=None;return k
rng=np.random.RandomState(315);ordinary_max=0.
for i in range(100):
 a,b=kernel(old),kernel();state=np.r_[rng.normal(size=3)*.1,np.zeros(14)];p=np.linspace(0,1,21);gear=(-1 if i%2 else 1);path=np.c_[gear*p,.05*np.sin(3*p),.15*np.cos(3*p),np.full(21,.15*gear)]
 ordinary_max=max(ordinary_max,float(np.max(np.abs(a.guide(state,path)-b.guide(state,path)))),float(np.max(np.abs(a.proposals(state,path,np.zeros(6))[0]-b.proposals(state,path,np.zeros(6))[0]))))
recoveries=[]
for gear in [-1,1]:
 for dx in [-.16,0,.16]:
  for dy in [-.2,-.06,0,.06,.2]:
   for yaw in [-np.pi+.01,-.5,0,.5,np.pi-.01]:
    k=kernel();z=np.zeros(17);z[2]=yaw;samples=np.tile([dx,dy,0,0,0,.1],(k.H+1,1));tracking=dict(gear=gear,identity=(1,gear),resume_distance=.05,base_samples=samples,yaw_hold_speed=.04);ref=samples[:,:4];success=None;trans=0;last=False
    for n in range(600):
     u=k.guide(z,ref,tracking=tracking);trans+=last!=k._tracking_capture;last=k._tracking_capture;z=k.advance(z,np.r_[u,np.zeros(6)],k.dt)
     if np.linalg.norm(z[:2]-[dx,dy])<=.05 and abs(new.wrap(z[2]))<=.15:
      success=(n+1)*k.dt;break
    recoveries.append(dict(gear=gear,initial_reference=[dx,dy],initial_yaw=yaw,success_seconds=success,position_error=float(np.linalg.norm(z[:2]-[dx,dy])),yaw_error=float(abs(new.wrap(z[2]))),capture_transitions=trans))
# Compare each candidate simulated alone to the same batch; no cross-candidate mode mutation.
k=kernel();z=np.zeros(17);s=np.tile([.08,.07,0,.05,.1,.1],(k.H+1,1));tr=dict(gear=1,identity=(5,),resume_distance=.05,base_samples=s,yaw_hold_speed=.04);modes=[(1.,0.),(.65,0.),(0.,0.),(.8,-.5),(.8,.5),(-.55,0.)];batch,_=k.proposals(z,s[:,:4],np.zeros(6),modes,tracking=tr);batch_max=0.
for i,mode in enumerate(modes):
 one=kernel();u,_=one.proposals(z,s[:,:4],np.zeros(6),[mode],tracking=tr);batch_max=max(batch_max,float(np.max(np.abs(u[0]-batch[i]))))
report=dict(ordinary_cases=100,ordinary_max_abs_difference=ordinary_max,frozen_recovery_cases=len(recoveries),frozen_recovery_passed=sum(x['success_seconds'] is not None for x in recoveries),frozen_recovery_failures=[x for x in recoveries if x['success_seconds'] is None],frozen_recovery_max_s=max(x['success_seconds'] or 0 for x in recoveries),batch_max_abs_difference=batch_max,scope='Independent production guide/plant numeric checks; no obstacle geometry, ROS or hardware; exact polynomial and recorded interruption integration separately validated.',runtime_sha256={str(scripts/f):hashlib.sha256((scripts/f).read_bytes()).hexdigest() for f in ['active_arm_mppi.py','native_reference.py','native_reference_adapter.py']})
(case/'independent_guide_review.json').write_text(json.dumps(report,indent=2)+'\n');(case/'independent_static_pose_rows.json').write_text(json.dumps(recoveries,indent=2)+'\n');print(json.dumps(report,indent=2))
