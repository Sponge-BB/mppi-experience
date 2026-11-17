#!/usr/bin/env python3
"""Offline, bounded source-time joint-feedback audit. No ROS initialization."""
import os,sys,json,hashlib,datetime
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
sys.path[:0]=['/opt/ros/noetic/lib/python3/dist-packages','/home/l/remani_ws/devel/lib/python3/dist-packages']
import rosbag,numpy as np
from pathlib import Path
ROOT=Path(__file__).resolve().parent
origin=json.loads((ROOT/'original_geometry_failure_audit.json').read_text())
events=[r for r in origin['events'] if r['reason']=='no_feasible_candidate']
windows=[(r['diagnostics']['plan_time']-.4,r['diagnostics']['plan_time']+.3) for r in events]
samples=[]
with rosbag.Bag(str(ROOT/'capture.bag')) as bag:
 for _,msg,t in bag.read_messages(topics=['/joint_states_single']):
  source=msg.header.stamp.to_sec()
  if not any(lo<=source<=hi for lo,hi in windows):continue
  ids=[msg.name.index('joint'+str(i)) for i in range(1,7)]
  if any(i>=len(msg.velocity) for i in ids):continue
  samples.append(dict(source_ros=source,receive_ros=t.to_sec(),q=[msg.position[i] for i in ids],dq=[msg.velocity[i] for i in ids]))
out=[];tz=datetime.timezone(datetime.timedelta(hours=8))
for event in events:
 plan=event['diagnostics']['plan_time'];data=[r for r in samples if plan-.4<=r['source_ros']<=plan+.3];a=np.array([[r['source_ros']]+r['q']+r['dq'] for r in data]);target=np.asarray(event['diagnostics']['measured_state'][5:]);matches=np.flatnonzero(np.max(np.abs(a[:,1:]-target[None,:]),axis=1)<1e-12)
 pulse_i=int(matches[np.argmin(abs(a[matches,0]-plan))]) if len(matches) else None
 row=dict(seq=event['seq'],phase=event['phase'],plan_time=plan,plan_local=datetime.datetime.fromtimestamp(plan,tz).isoformat(),state_match_count=len(matches),matched_samples=[data[i] for i in matches],observed_samples=[r for r in data if plan-.25<=r['source_ros']<=plan+.25],joints={})
 if pulse_i is not None:
  pulse=a[pulse_i];previous=a[pulse_i-1];dt=pulse[0]-previous[0];calc=(pulse[1:7]-previous[1:7])/dt;row.update(previous_recorded_sample=data[pulse_i-1],matched_sample=data[pulse_i],previous_source_dt_s=float(dt),dq_recomputed_from_adjacent_positions=calc.tolist(),dq_recomputed_max_abs_error=float(np.max(abs(calc-pulse[7:]))),matched_source_age_at_plan_s=float(plan-pulse[0]),matched_receive_relative_to_plan_s=float(data[pulse_i]['receive_ros']-plan))
  row['pulse_apparent_acceleration_from_adjacent_dq']=((pulse[7:]-previous[7:])/dt).tolist()
  # Inspect at most two samples before/after the matched complete ROS sample.
  row['pulse_neighbours']=data[max(0,pulse_i-2):min(len(data),pulse_i+3)]
 for ji in (3,4,5):
  joint=ji+1;mask=(a[:,0]>=plan-.25)&(a[:,0]<=plan+.25);times=a[mask,0];q=a[mask,1+ji];v=a[mask,7+ji];stats=dict(raw_peak_abs_dq=float(np.max(abs(v))),position_min=float(q.min()),position_max=float(q.max()),position_range_rad=float(np.ptp(q)),nonzero_velocity_samples=int(np.count_nonzero(v)),sample_count=len(v),median_adjacent_dt_s=float(np.median(np.diff(times))),windows={})
  for duration in (.02,.05,.1):
   delta=q-np.interp(times-duration,a[:,0],a[:,1+ji]);vel=delta/duration;item=dict(peak_abs_dq=float(np.max(abs(vel))),p95_abs_dq=float(np.percentile(abs(vel),95)),max_abs_position_delta_rad=float(np.max(abs(delta))))
   if pulse_i is not None:
    ps=a[pulse_i,0];pd=a[pulse_i,1+ji]-np.interp(ps-duration,a[:,0],a[:,1+ji]);item.update(at_matched_state_dq=float(pd/duration),at_matched_state_delta_rad=float(pd))
   stats['windows'][str(duration)]=item
  row['joints'][str(joint)]=stats
 # Check adjacent-recorded derivative consistency throughout this window.
 dt=np.diff(a[:,0]);valid=(dt>0)&(dt<.15);derived=np.diff(a[:,1:7],axis=0)[valid]/dt[valid,None];reported=a[1:,7:][valid];err=np.max(abs(derived-reported),axis=1);row['window_difference_consistency']=dict(pairs=int(valid.sum()),matching_within_1e_6=int((err<1e-6).sum()),max_abs_error=float(err.max()))
 out.append(row)
paths=['src/piper_ros/src/piper/scripts/piper_ctrl_single_node.py','src/mm_remani/scripts/whole_body_mppi.py','/home/l/.local/lib/python3.8/site-packages/piper_sdk/interface/piper_interface.py','/home/l/.local/lib/python3.8/site-packages/piper_sdk/protocol/protocol_v2/piper_protocol_v2.py']
hashes={str(p):hashlib.sha256((Path('/home/l/remani_ws')/p if not p.startswith('/') else Path(p)).read_bytes()).hexdigest() for p in paths}
result=dict(method='Read-only source-time inspection of recorded complete ROS joint samples near five original no-feasible cycles. 20/50/100ms position differences are diagnostic scale comparisons, not a physical-velocity ground truth or a proposed deployed filter.',code_sha256=hashes,events=out,code_path_findings={
 'can_time_origin':'piper_protocol_v2.py:69 can_time_now=rx_can_frame.timestamp; feedback branches12/34/56 each set msg.time_stamp to that CAN receive timestamp, not a shared hardware sampling epoch',
 'sdk_joint_aggregate':'piper_interface.py:1763-1794 stores three pair updates into one six-position object; every pair update overwrites the single time_stamp',
 'driver_acceptance':'piper_ctrl_single_node.py:559-574 requires all three pair counters to advance, per-pair arrival ages<0.15s, pair spread<0.05s; copies six positions under SDK lock, one shared last-update stamp',
 'driver_velocity':'piper_ctrl_single_node.py:594-601 computes each dq=(q-old_q)/(shared_stamp-old_shared_stamp); no motor_speed field is used',
 'mppi_input':'whole_body_mppi.py:328-344 trusts finite fresh position/velocity arrays, checks q bounds, then stores dq as the measured initial velocity; no independent velocity estimate or compatibility check',
 'sdk_zeroing_exclusion':'__CalJointSDKLimit at1668 only clips to joint range when enabled; this code does not zero small near-zero positions',
 'unrecorded':'Raw CAN pair payloads, independent per-pair source receive stamps/counters, and high-speed per-motor speed are absent from bag, so the exact source of position stair steps/zero toggles cannot be proved or attributed to firmware/hardware from this record alone.'})
(ROOT/'dq_root_cause_analysis.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
for row in out:
 print(json.dumps(dict(seq=row['seq'],phase=row['phase'],match_count=row['state_match_count'],state_source=row.get('matched_sample',{}).get('source_ros'),age_ms=1000*row.get('matched_source_age_at_plan_s',0),dt_ms=1000*row.get('previous_source_dt_s',0),derivative_error=row.get('dq_recomputed_max_abs_error'),dq=row.get('matched_sample',{}).get('dq'),qdelta=[a-b for a,b in zip(row.get('matched_sample',{}).get('q',[]),row.get('previous_recorded_sample',{}).get('q',[]))],window_consistency=row['window_difference_consistency'],joints=row['joints']),ensure_ascii=False))
