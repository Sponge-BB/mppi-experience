import rosbag,json,math,collections,re,statistics,pathlib,bisect
P=pathlib.Path('/home/l/remani_ws/experiments/online_remani_validation/results/frozen_guidance_capture_20261114-161009')
O=P/'analysis_rootcause';O.mkdir(exist_ok=True)
def stats(v):
 a=sorted(v)
 return dict(n=len(a),mean=statistics.mean(a),min=a[0],median=statistics.median(a),p95=a[math.ceil(.95*len(a))-1],max=a[-1]) if a else {'n':0}
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def wrap(v):return math.atan2(math.sin(v),math.cos(v))
status=[];logs=[];progress=[];odom=[];commands=[];events=[];execution=[];fault=[];traces=collections.defaultdict(list)
with rosbag.Bag(str(P/'latency.bag')) as b:
 connections=[dict(topic=x.topic,datatype=x.datatype,callerid=x.header.get('callerid',b'').decode()) for x in b._get_connections()]
 capture=(b.get_start_time(),b.get_end_time())
 for topic,m,t in b.read_messages(topics=['/mppi/status','/rosout_agg','/mm_controller_node/trajectory_progress','/odom','/cmd_vel','/move_base_simple/goal','/planning/trajectory','/planning/finish','/planning/safety_speed_limit','/piper_execution_state','/tracking_fault']):
  bt=t.to_sec()
  if topic=='/mppi/status':
   d=json.loads(m.data);c=d.get('candidate_diagnostics') or {};ch=c.get('checked_inputs') or {};ca=c.get('capture_inputs') or {}
   d={k:v for k,v in d.items() if k not in ['candidate_diagnostics','arm_target','native_arm_reference','native_arm_sample_phases']}
   d.update(bag=bt,t=ch.get('checked_at',ca.get('checked_at',bt)),capture=ca,checked=ch)
   status.append(d)
  elif topic=='/rosout_agg':
   s=m.header.stamp.to_sec();logs.append(dict(t=s,bag=bt,name=m.name,level=m.level,msg=m.msg))
   if '[MapTrace]' in m.msg or '[LatencyTrace]' in m.msg:
    r=dict(re.findall(r'(\w+)=([^ ]+)',m.msg));r.update(t=s,bag=bt)
    if 'stage' in r:traces[r['stage']].append(r)
  elif topic=='/mm_controller_node/trajectory_progress':progress.append(dict(t=m.header.stamp.to_sec(),bag=bt,start=m.vector.x,phase=m.vector.y,scale=m.vector.z))
  elif topic=='/odom':odom.append(dict(t=m.header.stamp.to_sec(),bag=bt,x=m.pose.pose.position.x,y=m.pose.pose.position.y,yaw=yaw(m.pose.pose.orientation),v=m.twist.twist.linear.x,w=m.twist.twist.angular.z))
  elif topic=='/cmd_vel':commands.append(dict(t=bt,v=m.linear.x,w=m.angular.z))
  elif topic=='/piper_execution_state':execution.append(dict(t=m.header.stamp.to_sec(),bag=bt,enabled=m.enabled,stop_latched=m.stop_latched,feedback_valid=m.feedback_valid,command_active=m.command_active))
  elif topic=='/tracking_fault':fault.append(dict(t=bt,value=m.data))
  else:
   e=dict(topic=topic,bag=bt)
   if topic=='/move_base_simple/goal':e.update(t=m.header.stamp.to_sec(),x=m.pose.position.x,y=m.pose.position.y,yaw=yaw(m.pose.orientation))
   elif topic=='/planning/trajectory':e.update(t=m.header.stamp.to_sec(),trajectory_id=m.trajectory_id,action=m.action,gear=m.singul,duration=sum(p.duration for p in m.trajectory))
   else:e.update(t=bt,value=m.data)
   events.append(e)
status.sort(key=lambda r:r['t']);logs.sort(key=lambda r:r['t']);progress.sort(key=lambda r:r['t']);odom.sort(key=lambda r:r['t'])
t0=next(r['t'] for r in events if r['topic']=='/move_base_simple/goal')
tfinish=next(r['t'] for r in events if r['topic']=='/planning/finish')
def period(t):return 'before_goal' if t<t0 else 'navigation' if t<tfinish else 'after_finish'
def intervals(rows,classify):
 res=[]
 for r in rows:
  c=classify(r)
  if not res or res[-1]['value']!=c:res.append(dict(value=c,t_start=r['t'],t_end=r['t'],samples=1))
  else:res[-1]['t_end']=r['t'];res[-1]['samples']+=1
 for i,r in enumerate(res):
  r['duration_to_next_s']=(res[i+1]['t_start'] if i+1<len(res) else r['t_end'])-r['t_start'];r['rel_start']=r['t_start']-t0;r['rel_end']=r['t_end']-t0
 return res
phase_status={p:[r for r in status if period(r['t'])==p] for p in ['before_goal','navigation','after_finish']}
flags=['base_fresh','joint_fresh','map_age_ok','dynamic_age_ok','snapshot_valid','dynamic_valid']
state_summ={}
for p,rows in phase_status.items():
 bystate=collections.defaultdict(list)
 for r in rows:bystate[r.get('state')].append(r)
 state_summ[p]={}
 for state,rr in bystate.items():
  fc=collections.Counter(f for r in rr for f in flags if r['checked'].get(f) is False)
  state_summ[p][state]=dict(n=len(rr),false_flags=dict(fc),map_age_ms=stats([r['checked']['map_age_ms'] for r in rr if 'map_age_ms' in r['checked']]),cycle_ms=stats([r['cycle_ms'] for r in rr if 'cycle_ms' in r]))
nav=[r for r in status if t0<=r['t']<tfinish];navprogress=[r for r in progress if t0<=r['t']<=tfinish and r['start']>0]
freeze_intervals=intervals(navprogress,lambda r:'frozen' if r['scale']==0 else 'progressing')
for x in freeze_intervals:
 rr=[r for r in nav if x['t_start']<=r['t']<=x['t_end']];x['status_counts']=dict(collections.Counter(r['state'] for r in rr));x['phase_first']=next((r['phase'] for r in navprogress if r['t']>=x['t_start']),None);x['phase_last']=next((r['phase'] for r in reversed(navprogress) if r['t']<=x['t_end']),None)
 for k in ['map_age_ok','joint_fresh','base_fresh']:x[k+'_false']=sum(r['checked'].get(k) is False for r in rr)
log_events=[dict(r,rel=r['t']-t0) for r in logs if any(s in r['msg'] for s in ['[FSM]','[MMctrl]','MPPIRefresh','Safety','Emergency','timed out','stop latched','stream expired','Input/time basis unavailable']) and t0-3<=r['t']<=tfinish+4]
# Transitions between actual delivered angular signs; separate sources from bag timings.
sign_trans=[];last=None
for r in nav:
 if r.get('state')!='command_delivered' or abs(r.get('base_w',0))<.02:continue
 sign=1 if r['base_w']>0 else -1
 if last is not None and sign!=last['sign']:
  sign_trans.append(dict(t=r['t'],rel=r['t']-t0,prev_rel=last['t']-t0,w=r['base_w'],v=r.get('base_v'),phase=r.get('native_phase'),scale=r.get('native_time_scale'),gear=r.get('native_gear'),mode=r.get('mode'),selection=r.get('selection'),family=r.get('selected_family'),previous=last))
 last=dict(t=r['t'],sign=sign,w=r['base_w'],v=r.get('base_v'),phase=r.get('native_phase'),scale=r.get('native_time_scale'),gear=r.get('native_gear'),mode=r.get('mode'))
# Segment command zero vs nonzero spans; all ROS cmd comes from the sole recorded MPPI publisher.
cmdintervals=intervals([r for r in commands if t0<=r['t']<tfinish],lambda r:'zero' if abs(r['v'])<1e-4 and abs(r['w'])<1e-4 else 'nonzero')
for x in cmdintervals:
 rr=[r for r in nav if x['t_start']-.12<=r['t']<=x['t_end']+.01];x['near_status_counts']=dict(collections.Counter(r['state'] for r in rr))
trace_summ={}
for stage,rows in traces.items():
 rr=[r for r in rows if t0<=r['t']<tfinish];trace_summ[stage]=dict(n=len(rr))
 names={k for r in rr for k in r if k.endswith('_ms') and ',' not in str(r[k])}
 for k in names:
  try:trace_summ[stage][k]=stats([float(r[k]) for r in rr if k in r])
  except ValueError:pass
# Actual yaw motion per period
odom_summ={}
for p in phase_status:
 rr=[r for r in odom if period(r['t'])==p];d=[wrap(bb['yaw']-aa['yaw']) for aa,bb in zip(rr,rr[1:])];un=[0]
 for dv in d:un.append(un[-1]+dv)
 odom_summ[p]=dict(n=len(rr),yaw_abs_motion_deg=sum(map(abs,d))*180/math.pi,yaw_net_deg=sum(d)*180/math.pi,yaw_range_deg=(max(un)-min(un))*180/math.pi,xy_net_m=math.hypot(rr[-1]['x']-rr[0]['x'],rr[-1]['y']-rr[0]['y']) if rr else None)
report=dict(source=str(P/'latency.bag'),time_axis='MPPI checked_inputs.checked_at; log header.stamp; progress/odom source stamps; cmd/finish use bag receipt',capture=dict(start=capture[0],end=capture[1],duration_s=capture[1]-capture[0]),goal_stamp=t0,finish_stamp=tfinish,nav_duration_s=tfinish-t0,events=[dict(r,rel=r['t']-t0) for r in events],connections=connections,status_by_period=state_summ,freeze_intervals=freeze_intervals,log_events=log_events,delivered_angular_sign_changes=sign_trans,cmd_zero_nonzero_intervals=cmdintervals,map_trace_stats_during_navigation=trace_summ,log_source_to_bag_delay_s=stats([r['bag']-r['t'] for r in logs]),mppi_checked_to_bag_delay_s=stats([r['bag']-r['t'] for r in status]),piper_execution_states=dict(collections.Counter(str((r['enabled'],r['stop_latched'],r['feedback_valid'],r['command_active'])) for r in execution)),tracking_fault_values=dict(collections.Counter(str(r['value']) for r in fault)),odom_motion=odom_summ,limits=['Only one navigation goal was recorded. Cannot assert the recording contains a separately published post-arrival turnaround goal.','No collision field payload or physical driver transfer function is in this bag.','Timing source used whenever present; bag receipt lag is not processing latency.','Twist has no header so exact cmd origin time is unavailable; status checks provide source diagnostics.'])
(O/'timing_audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
(O/'navigation_timing_status.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in nav))
print(json.dumps(dict(nav_duration=report['nav_duration_s'],events=report['events'],statuses=state_summ,freeze=freeze_intervals,sign_changes=sign_trans,odom=odom_summ,rosout_delay=report['log_source_to_bag_delay_s']),ensure_ascii=False,indent=2))
