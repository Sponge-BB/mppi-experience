#!/usr/bin/env python3
import os,sys,json,datetime,collections
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
sys.path[:0]=['/opt/ros/noetic/lib/python3/dist-packages','/home/l/remani_ws/devel/lib/python3/dist-packages']
from pathlib import Path
import rosbag
ROOT=Path(__file__).resolve().parent
rows=[];goals=[];traj=[];statuscounts=collections.Counter();last_state=None;first_goal=None;first_abort=None
with rosbag.Bag(str(ROOT/'latency.bag')) as bag:
 for topic,msg,t,header in bag.read_messages(topics=['/move_base_simple/goal','/planning/trajectory','/tracking_fault','/planning/safety_speed_limit','/mppi/status','/rosout_agg','/planning/finish'],return_connection_header=True):
  tt=t.to_sec();caller=header.get('callerid',header.get(b'callerid',b''));caller=caller.decode() if isinstance(caller,bytes) else caller
  r={'receive':tt,'topic':topic,'callerid':caller}
  if topic=='/move_base_simple/goal':
   r.update(source=msg.header.stamp.to_sec(),frame=msg.header.frame_id,x=msg.pose.position.x,y=msg.pose.position.y);goals.append(r);first_goal=first_goal or tt
  elif topic=='/planning/trajectory':
   r.update(source=msg.header.stamp.to_sec(),action=msg.action,id=msg.trajectory_id,gear=msg.singul if hasattr(msg,'singul') else None,durations=[p.duration for p in msg.trajectory]);traj.append(r)
   if msg.action in (2,5):first_abort=first_abort or tt
  elif topic=='/mppi/status':
   if first_goal is None:continue
   d=json.loads(msg.data);statuscounts[d.get('state','unknown')]+=1
   # Keep changes and every geometry-failure record; skip repeated same waits.
   if d.get('state')==last_state and d.get('state') not in ['no_feasible_candidate','command_hold_or_braking_rejected_by_latest_state_or_map']:continue
   last_state=d.get('state');r.update(data=d)
  elif topic=='/rosout_agg':
   text=msg.msg
   if not any(x.lower() in text.lower() for x in ['[FSM]','collision','EMERGENCY','Depth Lost','Give up','lost_observation','tracking','abort','command rejected','timeout','timed out','native_reference']):continue
   r.update(source=msg.header.stamp.to_sec(),name=msg.name,message=text)
  else:r['data']=msg.data
  rows.append(r)
assert first_abort is not None
# Explicit short immediate sequence; include logs by source-time as transport can lag.
near=[r for r in rows if first_abort-2<=r['receive']<=first_abort+5 or (r.get('source') and first_abort-2<=r['source']<=first_abort+5)]
postgoal=[r for r in rows if first_goal is not None and r['receive']>=first_goal]
result={'method':'Read-only streaming bag; source timestamps retained separately from receiver order. Manual cancel excluded by user_observation.json. No controls published.','first_goal':first_goal,'first_abort':first_abort,'first_abort_relative_goal_s':first_abort-first_goal,'goals':goals,'trajectory_messages':traj,'postgoal_status_counts':dict(statuscounts),'immediate_first_abort_chain':near,'postgoal_relevant_events':postgoal}
(ROOT/'first_cancel_chain.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
tz=datetime.timezone(datetime.timedelta(hours=8))
print('GOAL',first_goal,'FIRST_ABORT',first_abort,'relative',first_abort-first_goal)
print('TRAJ',json.dumps(traj,ensure_ascii=False));print('STATUSCOUNTS',dict(statuscounts))
for r in near:
 z={k:v for k,v in r.items() if k!='data'}
 if r['topic']=='/mppi/status':
  d=r['data'];z['data']={k:v for k,v in d.items() if k not in ['candidate_diagnostics','snapshot']};z['diagnostics']=d.get('candidate_diagnostics',{})
 else:z=r
 print(json.dumps(z,ensure_ascii=False))
