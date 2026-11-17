"""Approximate recorded replay, not synchronized live validation.
Sensor points use the last recorded planar pose and original mount offsets;
clock100 is synthetic. No ROS or hardware is accessed.
"""
import sys,json,time,cProfile,pstats,gzip
from pathlib import Path
import numpy as np
r=Path(__file__).resolve().parents[1]
run=sys.argv[1] if len(sys.argv)>1 else 'actual_stationary_06'
sys.path[:0]=[str(r/'core'),str(r/'runtime')]
from active_arm_model import PiperArmModel
from standby_whole_body_mppi import StandbyWholeBodyMPPI
from hardware_cloud_snapshot import SurfaceObservation,DynamicObservation,HardwareCloudSnapshot
from mppi_command_gateway import NativeLeaseValidator
f=np.load(r/'reports'/run/'snapshot.npz')
status_path=r/'reports'/run/'shadow_status.json'
with (status_path.open() if status_path.exists() else gzip.open(str(status_path)+'.gz','rt')) as stream:
    rows=json.load(stream)
row=next(s for s in reversed(rows) if s['kernel_diagnostics'])
state=np.array(row['state']); cfg=json.loads((r/'config/hardware_shadow.json').read_text()); a=cfg['arm_limits']
m=PiperArmModel.from_parameters(r/'model/parameters.yaml',r/'model/mm.urdf')
k=StandbyWholeBodyMPPI(m,cfg['limits'],[a[x] for x in ('lower','upper','velocity','acceleration')],cfg['postures'],horizon=8,random_samples=4)
y=state[2];R=np.array([[np.cos(y),-np.sin(y),0],[np.sin(y),np.cos(y),0],[0,0,1]])
def obs(p,name):return SurfaceObservation.create(p,100.,'odom','odom',name,max_points=15000 if name=='static' else 4000)
raw=f['raw'];raw=raw[np.any(raw!=0,axis=1)];raw=(raw+[.28,0,.117])@R.T+np.r_[state[:2],0]
filtered=(f['filtered']+[.28,0,.117])@R.T+np.r_[state[:2],0]
s=HardwareCloudSnapshot(obs(raw,'raw'),obs(filtered,'filtered'),obs(f['background'],'static'),DynamicObservation.create([],100.,'odom','odom'),100.,True)
reference=np.array([[*row['anchor'],0.]])
k.plan(state,reference,state[5:11],s,100.1)
profile=cProfile.Profile();profile.enable();t=time.perf_counter()
plan,info=k.plan(state,reference,state[5:11],s,100.1)
ms=(time.perf_counter()-t)*1000;profile.disable()
print('replay_only_plan_ms',ms,'feasible',info['feasible'])
pstats.Stats(profile).sort_stats('cumtime').print_stats(12)
v=NativeLeaseValidator(k);t=time.perf_counter()
ok=v(state,plan[0][0],s,100.1,.1) if plan else False
print('replay_only_final_validation',ok,'ms',(time.perf_counter()-t)*1000)
