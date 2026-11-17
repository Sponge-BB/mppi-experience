import os
for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[k]='1'
import json, math, collections, re
from pathlib import Path
from datetime import datetime, timezone, timedelta
import rosbag, rospy
import numpy as np
p=Path('experiments/online_remani_validation/results/drive_session_20261112-120325')
goal_receipt=1794456459.9303157
TOPICS=['/move_base_simple/goal','/cmd_vel','/odom','/mppi/status','/planning/trajectory','/planning/finish','/mm_controller_node/trajectory_progress','/rosout_agg','/planning/safety_speed_limit','/tracking_fault']
def clock(t):return datetime.fromtimestamp(t,timezone(timedelta(hours=8))).isoformat(timespec='milliseconds')
def stamp(m):return m.header.stamp.to_sec() if hasattr(m,'header') else None
def timefields(t,st=None):
 r={'receipt':t,'receipt_local':clock(t),'receipt_since_goal_s':round(t-goal_receipt,6)}
 if st is not None:
  r['source_stamp']=st
  if st>0:r.update(source_local=clock(st),receipt_minus_source_ms=round((t-st)*1000,3))
  else:r.update(source_local=None,source_stamp_valid=False)
 return r
cmd=[];odom=[];status=[];traj=[];finish=[];progress=[];logs=[];speeds=[];faults=[];goals=[];counts=collections.Counter();odom_frames=set();odom_children=set()
with rosbag.Bag(str(p/'capture.bag')) as b:
 end=b.get_end_time()
 for topic,m,bt in b.read_messages(topics=TOPICS,start_time=rospy.Time.from_sec(goal_receipt-.01)):
  t=bt.to_sec()
  if t<goal_receipt and topic!='/move_base_simple/goal':continue
  counts[topic]+=1
  if topic=='/cmd_vel':cmd.append([t,m.linear.x,m.angular.z])
  elif topic=='/odom':
   odom_frames.add(m.header.frame_id);odom_children.add(m.child_frame_id)
   q=m.pose.pose.orientation
   yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
   odom.append([t,stamp(m),m.pose.pose.position.x,m.pose.pose.position.y,yaw,m.twist.twist.linear.x,m.twist.twist.angular.z])
  elif topic=='/mppi/status':status.append((t,json.loads(m.data)))
  elif topic=='/planning/trajectory':
   item=timefields(t,stamp(m));item.update(id=m.trajectory_id,action=m.action,action_name={1:'ADD',2:'ABORT',3:'WARN_START',4:'WARN_FINAL',5:'WARN_IMPOSSIBLE'}.get(m.action,str(m.action)),gear=m.singul,pieces=len(m.trajectory),duration_s=sum(x.duration for x in m.trajectory))
   if m.trajectory:
    first=m.trajectory[0];last=m.trajectory[-1]
    a=np.asarray(first.data).reshape((-1,first.num_dim));z=np.asarray(last.data).reshape((-1,last.num_dim));v=np.zeros(last.num_dim)
    for row in z:v=v*last.duration+row
    item.update(start_xy=a[-1,:2].tolist(),end_xy=v[:2].tolist())
   traj.append(item)
  elif topic=='/planning/finish':finish.append(dict(timefields(t),finished=m.data))
  elif topic=='/mm_controller_node/trajectory_progress':progress.append([t,stamp(m),m.vector.x,m.vector.y,m.vector.z])
  elif topic=='/rosout_agg':logs.append(dict(timefields(t,stamp(m)),level=m.level,node=m.name,message=m.msg))
  elif topic=='/planning/safety_speed_limit':speeds.append(dict(timefields(t),limit=m.data))
  elif topic=='/tracking_fault':faults.append(dict(timefields(t),active=m.data))
  else:
   q=m.pose.orientation;goals.append(dict(timefields(t,stamp(m)),frame=m.header.frame_id,x=m.pose.position.x,y=m.pose.position.y,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))))

def quant(values):
 if not values:return {'n':0}
 a=np.asarray(values);return dict(n=len(a),min=float(a.min()),p50=float(np.median(a)),p95=float(np.quantile(a,.95)),max=float(a.max()))

def compact_status(t,s):
 keys=['state','mode','hold_reason','cycle_ms','solve_ms','base_v','base_w','native_phase','native_time_scale','base_safety_speed_limit','input_error','snapshot']
 r=dict(timefields(t),**{k:s[k] for k in keys if k in s})
 if 'candidate_diagnostics' in s:r['candidate_diagnostics']=s['candidate_diagnostics']
 return r

def nearest_status(t):
 if not status:return None
 i=min(range(len(status)),key=lambda i:abs(status[i][0]-t));a,s=status[i]
 return dict(receipt=a,offset_ms=round((a-t)*1000,3),state=s.get('state'),mode=s.get('mode'),hold_reason=s.get('hold_reason'))
# Command segments use zero-order hold until the next recorded command. Last
# command cannot prove actuator execution, and is not extended beyond bag end.
segments=[]
for i,(t,v,w) in enumerate(cmd):
 moving=abs(v)>1e-6 or abs(w)>1e-6
 if not segments or segments[-1]['moving']!=moving:
  segments.append(dict(start=t,end=t,moving=moving,messages=0,min_v=v,max_v=v,min_w=w,max_w=w))
 seg=segments[-1];seg['messages']+=1;seg['end']=cmd[i+1][0] if i+1<len(cmd) else end
 seg['min_v']=min(seg['min_v'],v);seg['max_v']=max(seg['max_v'],v);seg['min_w']=min(seg['min_w'],w);seg['max_w']=max(seg['max_w'],w)
for seg in segments:
 seg['duration_s']=seg['end']-seg['start'];seg['start_local']=clock(seg['start']);seg['end_local']=clock(seg['end']);seg['nearest_status_at_start']=nearest_status(seg['start'])
zero=[s for s in segments if not s['moving']];moving=[s for s in segments if s['moving']]
state_runs=[]
for t,s in status:
 label=(s.get('state'),s.get('mode'),s.get('hold_reason'))
 if not state_runs or tuple(state_runs[-1]['label'])!=label:state_runs.append(dict(label=list(label),first=t,last=t,count=0))
 state_runs[-1]['last']=t;state_runs[-1]['count']+=1
important=[r for r in logs if any(x in r['message'] for x in ('[FSM]','[FSM]:','Tracker reported','tracking fault','replan','Replan','EMERGENCY','goal','Goal','reach','Reach','trajectory','Trajectory'))]
# Explicitly retain all command-hold and no-feasible details, not only their counts.
failures=[compact_status(t,s) for t,s in status if s.get('state') in ('no_feasible_candidate','command_hold_or_braking_rejected_by_latest_state_or_map','arm_stop_latched')]
abortwindows=[]
for tr in traj:
 if tr['action']==2:
  t=tr['receipt'];abortwindows.append(dict(trajectory=tr,logs=[r for r in logs if t-1.5<=r['receipt']<=t+.5],mppi=[compact_status(a,s) for a,s in status if t-.7<=a<=t+.3],tracking_fault=[r for r in faults if t-.7<=r['receipt']<=t+.3]))
a=np.asarray(odom)
movement={}
if len(a):
 d=np.linalg.norm(np.diff(a[:,2:4],axis=0),axis=1)
 movement={'first':dict(timefields(a[0,0],a[0,1]),x=a[0,2],y=a[0,3],yaw=a[0,4]),'last':dict(timefields(a[-1,0],a[-1,1]),x=a[-1,2],y=a[-1,3],yaw=a[-1,4]),'net_displacement_m':float(np.linalg.norm(a[-1,2:4]-a[0,2:4])),'raw_xy_path_length_m':float(d.sum()),'raw_xy_step_m':quant(d.tolist()),'absolute_reported_v_mps':quant(abs(a[:,5]).tolist()),'integrated_absolute_reported_v_m':float(np.sum(abs(a[:-1,5])*np.diff(a[:,1]))),'caveat':'Odometry contains localization noise: raw xy path length includes standing drift; integrated reported speed is a separate estimate.'}
 movement['odom_frames']=sorted(odom_frames);movement['odom_child_frames']=sorted(odom_children)
 movement['goal_distance_m']=None
 movement['goal_distance_reason']='Goal is in world, odometry header is odom. No time-matched TF applied; direct cross-frame distance intentionally not reported.'
 active_end=moving[-1]['end'] if moving else goal_receipt
 active=a[a[:,0]<=active_end+1.]
 movement['motion_window_receipt_end']=active_end+1.
 movement['motion_window_end_local']=clock(active_end+1.)
 movement['motion_window_integrated_absolute_reported_v_m']=float(np.sum(abs(active[:-1,5])*np.diff(active[:,1])))
 movement['motion_window_net_displacement_odom_m']=float(np.linalg.norm(active[-1,2:4]-active[0,2:4]))
 movement['caveat']='All pose displacement/xy polyline quantities are within odom frame and are not world-goal distance or arrival evidence. Localization drift contributes to raw xy length; abs velocity integral accumulates standing noise, especially the ~193 s final hold. Motion-window integral ends 1 s after last nonzero command, remains odometry-derived rather than ground truth.'
prgroups={}
for row in progress:
 key=str(row[2]);g=prgroups.setdefault(key,{'start_stamp':row[2],'first':row,'last':row,'count':0,'min_scale':row[4],'max_scale':row[4]});g['last']=row;g['count']+=1;g['min_scale']=min(g['min_scale'],row[4]);g['max_scale']=max(g['max_scale'],row[4])
report={
 'scope':'Post-goal real capture, user reports continuing stop/go. No cloud arrays read, no ROS nodes started, no code changed.',
 'time_semantics':'bag time is recorder receipt. Source header stamps shown separately. cmd_vel and mppi/status have no source header. Status proximity is correlation, not proof of unique cause.',
 'goal_receipt':goal_receipt,'goal_local':clock(goal_receipt),'bag_end':end,'bag_end_local':clock(end),'duration_after_goal_s':end-goal_receipt,'topic_counts':dict(counts),'goal':goals,
 'mppi_state_counts':dict(collections.Counter(s.get('state') for t,s in status)),
 'mppi_delivery_modes':dict(collections.Counter(s.get('mode') for t,s in status if s.get('state')=='command_delivered')),
 'successful_cycle_ms':quant([s['cycle_ms'] for t,s in status if s.get('state')=='command_delivered']),
 'nonzero_command_count':sum(abs(v)>1e-6 or abs(w)>1e-6 for t,v,w in cmd),
 'zero_command_count':sum(abs(v)<=1e-6 and abs(w)<=1e-6 for t,v,w in cmd),
 'moving_to_zero_transitions':sum(segments[i-1]['moving'] and not s['moving'] for i,s in enumerate(segments) if i),
 'command_nonzero_duration_s':sum(s['duration_s'] for s in moving),'command_zero_duration_s':sum(s['duration_s'] for s in zero),
 'command_v_mps':quant([v for t,v,w in cmd]),'last_nonzero_command_end':moving[-1]['end'] if moving else None,'final_zero_duration_s':zero[-1]['duration_s'] if zero else None,'last_cmd':dict(timefields(cmd[-1][0]),v=cmd[-1][1],w=cmd[-1][2]) if cmd else None,
 'last_mppi':compact_status(*status[-1]) if status else None,
 'movement':movement,'trajectory_events':traj,'planning_finish_messages':finish,'safety_speed_limit_events':speeds,'tracking_fault_true':[r for r in faults if r['active']],
 'progress_by_reference':list(prgroups.values()),'progress_receipt_age_ms':quant([(r[0]-r[1])*1000 for r in progress]),
 'zero_command_segments':zero,'nonzero_command_segments':moving,'important_planner_logs':important,'failure_details':failures,'abort_windows':abortwindows,
 'warning_error_top_messages':collections.Counter((r['node']+': '+r['message']) for r in logs if r['level']>=4).most_common(20),
 'interpretation':'Reference cancellation is not arrival. FSM explicitly gave up target at source time 1794456500.063 and requested a new goal. No finish topic messages were recorded. Bag records 5 ADD and 5 ABORT messages; receipt counts do not prove no publisher/subscriber drops. Some ADD rows are gear blocks, not entire replans.'
}
(p/'navigation_timeline_analysis.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
lines=['2026-11-12 现场导航走停：离线时序复核',f'目标接收 {clock(goal_receipt)}；录制末尾 {clock(end)}；共 {end-goal_receipt:.3f}s。','时间定义：receipt 是录制接收时刻；source_stamp 是消息头源时间；/cmd_vel、/mppi/status 没有源时间头。',f'非零→零 {report["moving_to_zero_transitions"]} 次；非零命令累计 {report["command_nonzero_duration_s"]:.3f}s，零命令累计 {report["command_zero_duration_s"]:.3f}s。',f'实测定位净位移 {movement.get("net_displacement_m",0):.4f}m；速度积分路程 {movement.get("integrated_absolute_reported_v_m",0):.4f}m；原始定位折线 {movement.get("raw_xy_path_length_m",0):.4f}m（包含定位噪声）。','目标frame=world，里程计frame='+','.join(sorted(odom_frames))+'；未做对应时刻TF变换，不给出跨坐标系目标距离，也不用里程计位移证明到达。',f'实际非零命令窗口加1秒内的里程计速度积分 {movement["motion_window_integrated_absolute_reported_v_m"]:.4f}m（不是地面真值）；最终连续零命令 {zero[-1]["duration_s"]:.3f}s。','MPPI 状态计数：'+json.dumps(report['mppi_state_counts'],ensure_ascii=False),'实际轨迹消息：']
for r in traj:lines.append(f"receipt {r['receipt_local']} / source {r['source_local']} {r['action_name']} id={r['id']} gear={r['gear']} duration={r['duration_s']:.3f}s end={r.get('end_xy')}")
lines+=['关键规划日志（各行首为源时刻，括号为接收时刻）：']
for r in important:lines.append(f"{r['source_local']} ({r['receipt_local']}) {r['node']}: {r['message']}")
lines+=['明确终止原因：FSM源时刻12:08:20.063放弃当前目标并返回WAIT_TARGET，要求重发目标；这不是到达。','末状态：'+json.dumps(report['last_mppi'],ensure_ascii=False),'没有记录到 /planning/finish 消息；不能将 reference_cancelled 当成到达。','详细零速区间、ABORT 前后消息、进度及失败候选见 navigation_timeline_analysis.json。']
(p/'navigation_timeline_analysis.txt').write_text('\n'.join(lines)+'\n')
print(json.dumps({k:report[k] for k in ('duration_after_goal_s','mppi_state_counts','moving_to_zero_transitions','command_nonzero_duration_s','command_zero_duration_s','movement','trajectory_events','last_mppi')},ensure_ascii=False,indent=2))
