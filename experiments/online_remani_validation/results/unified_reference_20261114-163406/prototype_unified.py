import sys,json,numpy as np
from pathlib import Path
HERE=Path('/home/l/remani_ws/experiments/online_remani_validation/results/frozen_guidance_capture_20261114-161009/analysis_rootcause')
sys.path.insert(0,str(HERE));import coupled_native_clock as env
OUT=Path('/home/l/remani_ws/experiments/online_remani_validation/results/unified_reference_20261114-163406')
events=json.loads((HERE/'coupled_recorded_delivery_gate.json').read_text())
class Guide:
 def __init__(self,mode=0):self.capture=False;self.gear=None;self.transitions=[];self.mode=mode
 def __call__(self,z,a,p,phase,scale,t,k):
  pos,vel,speed,yaw,gear=p.sample(phase)
  c,s=np.cos(z[2]),np.sin(z[2]);delta=pos-z[:2];ex=c*delta[0]+s*delta[1];ey=-s*delta[0]+c*delta[1];dist=np.linalg.norm(delta)
  eyaw=float(env.wrap(yaw-z[2]));vr=gear*speed*scale
  i=min(len(p.ends)-1,int(np.searchsorted(p.ends,phase,side='left')));local=phase-(0 if i==0 else p.ends[i-1]);acc=np.array([np.polyval(np.polyder(v),local) for v in p.velocity[i]])
  wr=scale*np.cross(vel,acc)/max(1e-9,speed**2) if speed>.04 else 0.
  if self.gear!=gear:self.capture=False;self.gear=gear
  old=self.capture;self.capture=bool(dist>.035 if self.capture else abs(ey)>.05)
  if old!=self.capture:self.transitions.append((t,self.capture,dist))
  if self.capture:
   alpha=float(env.wrap(np.arctan2(delta[1],delta[0])+(np.pi if gear<0 else 0)-z[2]));v=gear*.9*dist*max(0,np.cos(alpha));w=2*alpha
  else:
   v=vr*np.cos(eyaw)+.9*ex;w=wr+1.0*vr*ey+1.8*eyaw
  return np.array([np.clip(v,-k.limits['vmax'],k.limits['vmax']),np.clip(w,-k.limits['wmax'],k.limits['wmax'])])
if __name__=='__main__':
 reports={}
 for name,gate in [('ideal',None),('recorded',events)]:
  guide=Guide();r=env.simulate(guide=guide,delivery_events=gate,max_seconds=60);trace=r.pop('trace');r['geometry_transitions']=guide.transitions
  (OUT/('prototype_'+name+'.json')).write_text(json.dumps(r,indent=2));(OUT/('prototype_'+name+'_trace.json')).write_text(json.dumps(trace))
  reports[name]={k:v for k,v in r.items() if k in ['clock_completed','elapsed','freeze_events','resume_events','frozen_seconds','nonzero_sign_changes','nonzero_sign_changes_scale_mode','nonzero_sign_changes_endpoint_2_5cm','final_endpoint_position_error','final_goal_yaw_error','max_reference_position_error','max_reference_yaw_error','geometry_transitions']}
 print(json.dumps(reports,indent=2))
