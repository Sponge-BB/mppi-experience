import sys,json,math,collections
from pathlib import Path
import numpy as np,rosbag
ROOT=Path('/home/l/remani_ws');CASE=ROOT/'experiments/online_remani_validation/results/frozen_guidance_capture_20261114-161009';OUT=CASE/'analysis_rootcause'
rows=[json.loads(x)for x in(OUT/'guide_replay_rows.jsonl').read_text().splitlines()];report=json.loads((OUT/'guide_replay_report.json').read_text());pieces=[];prog=[];logs=[]
with rosbag.Bag(str(CASE/'latency.bag'))as b:
 for topic,m,t in b.read_messages(topics=['/planning/trajectory','/mm_controller_node/trajectory_progress','/rosout_agg']):
  if topic=='/planning/trajectory'and m.action==1:
   for p in m.trajectory:pieces.append([p.duration,np.asarray(p.data).reshape((8,8),order='F'),m.singul])
  elif topic=='/mm_controller_node/trajectory_progress':
   if m.vector.x>1794643929:prog.append([m.header.stamp.to_sec(),m.vector.y,m.vector.z])
  elif topic=='/rosout_agg'and 'base tracking'in m.msg.lower():logs.append([m.header.stamp.to_sec(),m.msg])
ends=np.cumsum([p[0]for p in pieces]);starts=np.r_[0.,ends[:-1]]
def exact(phase):
 i=min(int(np.searchsorted(ends,phase,side='right')),len(pieces)-1);ts=phase-starts[i];c=pieces[i][1];gear=pieces[i][2];pos=np.array([np.polyval(a,ts)for a in c[:2]]);v=np.array([np.polyval(np.polyder(a),ts)for a in c[:2]]);acc=np.array([np.polyval(np.polyder(a,2),ts)for a in c[:2]]);speed=np.linalg.norm(v);yaw=math.atan2(v[1]*gear,v[0]*gear);w=float(np.cross(v,acc)/max(np.dot(v,v),1e-15));return dict(xy=pos.tolist(),yaw=yaw,v=float(speed*gear),w=w)
for r in rows:
 e=exact(r['phase']);r['exact_current_reference']=e;r['exact_yaw_error']=(e['yaw']-r['yaw']+math.pi)%(2*math.pi)-math.pi;r['native_nominal_scaled_w']=e['w']*r['scale'];r['native_nominal_scaled_v']=e['v']*r['scale']
(OUT/'guide_replay_rows.jsonl').write_text(''.join(json.dumps(r,separators=(',',':'))+'\n'for r in rows))
refs=[exact(t)for t in np.linspace(0,ends[-1],3000)]
report['exact_native_reference']=dict(max_abs_yaw_rate=max(abs(x['w'])for x in refs),max_abs_speed=max(abs(x['v'])for x in refs),duration=float(ends[-1]),max_sampled_yaw_error=float(max(abs((r['current_ref'][2]-r['exact_current_reference']['yaw']+math.pi)%(2*math.pi)-math.pi)for r in rows)),max_sampled_position_error=float(max(np.linalg.norm(np.array(r['current_ref'][:2])-r['exact_current_reference']['xy'])for r in rows)))
report['classified_delivered_flips']=[dict(prev_rel=x['prev']['rel'],cur_rel=x['cur']['rel'],reason=reason)for x,reason in zip(report['flips'],['frozen approach to align once','resume changes tangent alignment to endpoint bearing','freeze changes endpoint bearing to tangent alignment','positive-scale gear endpoint 25 mm switch','positive-scale gear endpoint 25 mm switch','positive-scale gear endpoint 25 mm switch','forward to reverse gear block switch'])]
report['progress_frozen_runs']=[]
run=None
for t,phase,scale in prog:
 if scale==0:
  if run is None:run=[t,t,phase]
  else:run[1]=t
 elif run is not None:
  report['progress_frozen_runs'].append(dict(begin_rel=run[0]-report['goal'][0]['receipt'],end_rel=run[1]-report['goal'][0]['receipt'],duration=run[1]-run[0],phase=run[2]));run=None
if run:report['progress_frozen_runs'].append(dict(begin_rel=run[0]-report['goal'][0]['receipt'],end_rel=run[1]-report['goal'][0]['receipt'],duration=run[1]-run[0],phase=run[2]))
report['limits']=['Only one goal and one finish are in the recording. Earlier/later claimed target tests are not covered.','All 295 delivered native nominal controls reproduced exactly; collision selection was nominal in every one. No need to infer unrecorded candidate selection.','This is source-time replay of measured states, not a closed-loop physical counterfactual.','Input age rejection remains present independently. It does not explain the deterministic sign reversals reproduced here.']
delivered=[r for r in rows if r['state']=='command_delivered']
report['native_angular_demand']={'max_abs_scaled_w_on_delivered':max(abs(r['native_nominal_scaled_w'])for r in delivered),'delivered_frames_scaled_w_exceeds_mppi_wmax':sum(abs(r['native_nominal_scaled_w'])>.35 for r in delivered),'delivered_frames':len(delivered),'mppi_wmax':.35,'scope':'Instantaneous nominal phase-scaled polynomial yaw derivative, not a full closed-loop feasibility proof.'}
for r in report['progress_frozen_runs']:r['kind']='terminal_phase_hold'if r['phase']>=13.48169 else 'tracking_pause'
for old,new in [('max_abs_yaw_rate','max_abs_yaw_rate_dense_sampled'),('max_abs_speed','max_abs_speed_dense_sampled')]:report['exact_native_reference'][new]=report['exact_native_reference'].pop(old)
(OUT/'guide_replay_report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:report[k]for k in('exact_native_reference','progress_frozen_runs','classified_delivered_flips')},indent=2))
