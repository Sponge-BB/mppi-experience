import sys,json,math,collections,ast
from pathlib import Path
import numpy as np,rosbag,yaml
ROOT=Path('/home/l/remani_ws');CASE=ROOT/'experiments/online_remani_validation/results/frozen_guidance_capture_20261114-161009';OUT=CASE/'analysis_rootcause'
sys.path.insert(0,str(ROOT/'src/mm_remani/scripts/mppi'))
from active_arm_mppi import WholeBodyMPPI,wrap
from native_reference_adapter import NativeReferenceAdapter
from native_reference import NativeReference
cfg=yaml.safe_load((ROOT/'src/mm_remani/config/mppi.yaml').read_text())
class Model:
 lower,upper=np.full(6,-4.),np.full(6,4.)
 velocity_limits,acceleration_limits=np.ones(6),np.full(6,2.)
 def check(self,q):return {'valid':True}
 def body_spheres_batch(self,q,b):raise RuntimeError('no collision replay')
m=Model();k=WholeBodyMPPI(m,cfg['base_limits'],(m.lower,m.upper,np.array(cfg['arm_velocity']),np.array(cfg['arm_acceleration'])),[],horizon=cfg['horizon'],step=.1,random_samples=4)
a=NativeReferenceAdapter(k,k)
ns={};tree=ast.parse((ROOT/'src/mm_remani/scripts/whole_body_mppi.py').read_text());exec(compile(ast.Module(body=[x for x in tree.body if isinstance(x,ast.FunctionDef) and x.name=='base_command'],type_ignores=[]),'base_command','exec'),ns)
base_command=ns['base_command'];bagfile=str(CASE/'latency.bag');events=[];statuses=[];goals=[];finishes=[];progress=[]
with rosbag.Bag(bagfile) as b:
 for topic,msg,t in b.read_messages(topics=['/mppi/status','/planning/trajectory','/move_base_simple/goal','/planning/finish','/mm_controller_node/trajectory_progress']):
  ts=t.to_sec()
  if topic=='/mppi/status':
   d=json.loads(msg.data);c=d.get('candidate_diagnostics',{});ci=c.get('capture_inputs',{});merged=dict(c);merged.update(d);merged.pop('candidate_diagnostics',None)
   statuses.append((ts,merged));events.append((ci.get('checked_at',ts),'status',merged))
  elif topic=='/planning/trajectory':events.append((ts,'trajectory',msg))
  elif topic=='/move_base_simple/goal':goals.append(dict(receipt=ts,source=msg.header.stamp.to_sec(),x=msg.pose.position.x,y=msg.pose.position.y,yaw=2*math.atan2(msg.pose.orientation.z,msg.pose.orientation.w)))
  elif topic=='/planning/finish':finishes.append(ts)
  else:progress.append([msg.header.stamp.to_sec(),msg.vector.x,msg.vector.y,msg.vector.z])
start=goals[0]['receipt'];end=finishes[0]
needbase={round(d['capture_inputs']['base_stamp'],6) for _,d in statuses if d.get('native_phase') is not None and 'capture_inputs'in d};needjoint={round(d['capture_inputs']['joint_stamp'],6) for _,d in statuses if d.get('native_phase') is not None and 'capture_inputs'in d};bases={};joints={}
with rosbag.Bag(bagfile) as b:
 for topic,msg,t in b.read_messages(topics=['/odom','/mm/joint_states']):
  key=round(msg.header.stamp.to_sec(),6)
  if topic=='/odom' and key in needbase:
   p,q=msg.pose.pose.position,msg.pose.pose.orientation;bases[key]=[p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),msg.twist.twist.linear.x,msg.twist.twist.angular.z]
  elif topic=='/mm/joint_states' and key in needjoint:
   ids=[msg.name.index('joint%d'%i) for i in range(1,7)];joints[key]=[msg.position[i]for i in ids]+[msg.velocity[i]for i in ids]
rows=[];native_messages=[];skipped=[]
for t,kind,d in sorted(events,key=lambda x:x[0]):
 if kind=='trajectory':
  action=a.native.feed(d,t)
  if action=='replace':a.arm_pieces=[]
  if action in('replace','append'):
   for p in d.trajectory:a.arm_pieces.append((p.duration,np.asarray(p.data).reshape((8,8),order='F')[2:,:]))
   native_messages.append(dict(receipt=t,source=a.native.start,kind=action,id=d.trajectory_id,gear=d.singul,durations=[p.duration for p in d.trajectory]))
  continue
 phase=d.get('native_phase');scale=d.get('native_time_scale')
 if phase is None or scale is None or not a.native.refs:continue
 c=d.get('capture_inputs',{});bk=round(c.get('base_stamp',0),6);jk=round(c.get('joint_stamp',0),6)
 if bk not in bases or jk not in joints:skipped.append([t,bk in bases,jk in joints]);continue
 s=np.array(bases[bk]+joints[jk]);path,block,blockend=a._base_reference(phase,scale,s[2]);gear=a.native.gears[block];arm=a._arm_samples(np.minimum(blockend,phase+scale*.1*np.arange(1,k.H+1)))
 track=dict(gear=gear,pause_key=(a.native.start,block,phase)if scale==0 else None,resume_distance=.05)
 previousalign=k._pause_align
 controls,labels=k.proposals(s,path,arm,[(1.,0.)],tracking=track);cmd=np.array(base_command(controls[0,0],k.limits));lim=d.get('base_safety_speed_limit')
 if lim is not None:
  if lim<=1e-3:cmd[:]=0
  else:cmd[0]=np.clip(cmd[0],-lim,lim)
 arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(path[:,:2],axis=0),axis=1))];dist=np.sum((s[None,:2]-path[:,:2])**2,axis=1);dist[arc>.30+.1*k.limits['vmax']]=np.inf;near=int(np.argmin(dist));look=min(len(path)-1,int(np.searchsorted(arc,arc[near]+.30)));delta=path[look,:2]-s[:2];bearing=float(wrap(math.atan2(delta[1],delta[0])+(math.pi if gear<0 else 0)))
 obs=[d.get('base_v'),d.get('base_w')];match=None if None in obs else bool(np.max(np.abs(cmd-np.array(obs)))<1e-8)
 r=dict(t=t,rel=t-start,state=d['state'],mode=d.get('mode'),phase=phase,scale=scale,gear=gear,block=block,native_start=a.native.start,xy=s[:2].tolist(),yaw=float(s[2]),measured_v=float(s[3]),measured_w=float(s[4]),current_ref=path[0].tolist(),end_ref=path[-1].tolist(),current_distance=float(np.linalg.norm(path[0,:2]-s[:2])),end_distance=float(np.linalg.norm(path[-1,:2]-s[:2])),current_yaw_error=float(wrap(path[0,2]-s[2])),bearing=bearing,bearing_error=float(wrap(bearing-s[2])),look=path[look,:2].tolist(),look_index=look,near_index=near,path_length_m=float(arc[-1]),pause_align=bool(k._pause_align),guide=cmd.tolist(),observed=obs,match=match,base_stamp=c['base_stamp'],joint_stamp=c['joint_stamp'],nominal_external_min_gap=d.get('nominal_external_min_gap'),external_min_gap=d.get('external_min_gap'))
 rows.append(r)
(OUT/'guide_replay_rows.jsonl').write_text(''.join(json.dumps(r,separators=(',',':'))+'\n'for r in rows))
# statuses source time preserves pending failure ordering, receipt ignored
navstates=collections.Counter(d['state']for t,d in statuses if start<=d.get('capture_inputs',{}).get('checked_at',t)<=end)
delivered=[r for r in rows if r['state']=='command_delivered'and r['mode']=='nominal'];flips=[]
for p,r in zip(delivered,delivered[1:]):
 if p['observed'][1]*r['observed'][1]<0 and min(abs(p['observed'][1]),abs(r['observed'][1]))>=.05:
  flips.append(dict(prev=p,cur=r,scale_mode_changed=(p['scale']==0)!=(r['scale']==0),phase_equal=p['phase']==r['phase'],block_changed=p['block']!=r['block'],align_changed=p['pause_align']!=r['pause_align']))
scale_transitions=[]
for p,r in zip(rows,rows[1:]):
 if (p['scale']==0)!=(r['scale']==0):scale_transitions.append(dict(prev=p,cur=r,guide_flip=p['guide'][1]*r['guide'][1]<0 and min(abs(p['guide'][1]),abs(r['guide'][1]))>=.05))
summary=dict(scope='Read-only exact source-stamp measured state; actual production proposals first control replay. No map data or full candidate feasibility replay. Stopped/expiry statuses without plans are omitted from numerical replay but counted. Source timestamps used.',goal=goals,native_messages=native_messages,finish=finishes,nav_duration=end-start,replay_rows=len(rows),delivered_nominal=len(delivered),matched=sum(r['match']for r in delivered),mismatches=[r for r in delivered if not r['match']],missing=skipped,nav_status_counts=dict(navstates),scales=dict(collections.Counter('frozen'if r['scale']==0 else'progressing'for r in delivered)),sign_flips=len(flips),flips_scale_mode_change=sum(r['scale_mode_changed']for r in flips),flips_same_phase=sum(r['phase_equal']for r in flips),flips_block_change=sum(r['block_changed']for r in flips),flips_align_change=sum(r['align_changed']for r in flips),scale_transitions=len(scale_transitions),scale_transition_guide_flips=sum(r['guide_flip']for r in scale_transitions),flips=flips,scale_transition_rows=scale_transitions)
(OUT/'guide_replay_report.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({k:v for k,v in summary.items()if k not in('mismatches','flips','scale_transition_rows')},indent=2))
print('FLIPS',[(round(f['prev']['rel'],3),round(f['cur']['rel'],3),f['prev']['scale'],f['cur']['scale'],f['prev']['observed'][1],f['cur']['observed'][1],f['prev']['current_yaw_error'],f['cur']['current_yaw_error'],f['prev']['current_distance'],f['cur']['current_distance'])for f in flips])
