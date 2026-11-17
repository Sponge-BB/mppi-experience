#!/usr/bin/python3
import ast,json,math,sys,time
from pathlib import Path
import numpy as np
ROOT=Path('/home/l/remani_ws');sys.path.insert(0,str(ROOT/'src/mm_remani/test'))
from test_mppi_timed_guidance import ready,run,pose_at,Model
from active_arm_mppi import wrap
source=ROOT/'src/mm_remani/scripts/standby_simulator.py';tree=ast.parse(source.read_text());names={'Plant','Q_START','STEP'}
nodes=[n for n in tree.body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name in names or isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id in names for t in n.targets)]
ns={'np':np,'math':math};exec(compile(ast.Module(body=nodes,type_ignores=[]),str(source),'exec'),ns);Plant=ns['Plant']
Model.lower=np.full(6,-3.);Model.upper=np.full(6,3.);Model.velocity_limits=np.full(6,.15);Model.acceleration_limits=np.full(6,.35);Model.check=lambda self,q:{'valid':True}
results=[]
for gear in [-1,1]:
 for lateral in [-.12,.12]:
  for yaw_error in [-.8,0.,.8]:
   adapter,kernel=ready(gear);target=pose_at(.4,gear);plant=Plant(kernel.model,kernel.limits,kernel.vmax,kernel.amax)
   plant.state=np.zeros(17);plant.state[:3]=target
   plant.state[:2]+=lateral*np.array([-np.sin(target[2]),np.cos(target[2])]);plant.state[2]+=yaw_error
   command=np.zeros(2);controls=[];failures=[];began=time.monotonic();recovered=False
   for tick in range(6001):
    t=tick*.005;dist=np.linalg.norm(plant.state[:2]-target[:2]);err=abs(wrap(plant.state[2]-target[2]))
    if dist<=.05 and err<=.15:recovered=True;break
    if tick%20==0:
     result,info=run(adapter,plant.state,0.,stamp=20.+t)
     if result is None:failures.append(dict(t=t,info=info));command=np.zeros(2)
     else:command=result[0][0,:2]
     controls.append(dict(t=t,command=command.tolist(),position_error=float(dist),yaw_error=float(err)))
    plant.step(command,plant.state[5:11],.005)
   results.append(dict(gear=gear,lateral=lateral,initial_yaw_error=yaw_error,recovered=recovered,elapsed=t,final_position_error=float(dist),final_yaw_error=float(err),plan_failures=failures,controls=controls,wall_seconds=time.monotonic()-began))
   print(json.dumps({k:v for k,v in results[-1].items() if k not in ('controls','plan_failures')}),flush=True)
output=Path('/home/l/remani_ws/experiments/online_remani_validation/results/unified_reference_20261114-163406/validation/frozen_adapter_recovery.json')
output.write_text(json.dumps(dict(scope='Actual adapter.plan/MPPI plan, stationary exact-polynomial reference at scale zero, independent original 5ms Plant; no fitted dynamics, no ROS, clear minimal scene, ideal arm. Not hardware validation.',cases=results),indent=2)+'\n')
