import bisect,collections,datetime,json,math,re
from pathlib import Path
A=Path('/home/l/remani_ws/experiments/online_remani_validation/results/dynamic_acceptance_20261117-103634/analysis')
def read(n):return [json.loads(l) for l in (A/(n+'.jsonl')).open()]
def clock(t):return datetime.datetime.fromtimestamp(t).strftime('%H:%M:%S.%f')[:-3]
def q(a):
 a=sorted(float(x) for x in a if x is not None and math.isfinite(x))
 if not a:return {}
 def pct(p):
  x=(len(a)-1)*p;lo=int(x);hi=min(lo+1,len(a)-1);return round(a[lo]+(a[hi]-a[lo])*(x-lo),3)
 return {'n':len(a),'p50':pct(.5),'p95':pct(.95),'max':round(a[-1],3)}
s=read('mppi_status');cmd=read('cmd_vel');logs=read('rosout_agg');odom=read('odom');progress=read('mm_controller_node_trajectory_progress');native=read('mm_controller_node_car_cmd');traj=read('planning_trajectory');goals=read('move_base_simple_goal');points=read('dynamic_points');piper=read('piper_execution_state')
for r in s:
 c=r['d'].get('candidate_diagnostics',{});r['source']=c.get('plan_time',c.get('capture_inputs',{}).get('checked_at',r['t']-r['d'].get('cycle_ms',0)/1000));r['end_est']=r['source']+r['d'].get('cycle_ms',0)/1000
windows=[('goal1_full',goals[0]['t'],1794883155.9394603),('goal2_start20s',goals[1]['t'],goals[1]['t']+20),('goal2_before_first_lidar_stop',goals[1]['t'],1794883295.489768),('goal2_full_test',goals[1]['t'],1794883517.3195465)]
result={};pulse_details=[];cancel_details=[]
for name,lo,hi in windows:
 ss=[r for r in s if lo<=r['source']<hi];cs=[r for r in cmd if lo<=r['t']<hi];os=[r for r in odom if lo<=r['source_t']<hi];ts=[r for r in traj if lo<=r['t']<hi];ps=[r for r in points if lo<=r['source_t']<hi];ls=[r for r in logs if lo<=r['source_t']<hi];counter=collections.Counter(r['d']['state'] for r in ss)
 rejection=[];timings=collections.defaultdict(list);flags=collections.Counter();modes=collections.Counter();succmodes=collections.Counter();snap_failed=collections.Counter();valid_plan_rejected=0
 for r in ss:
  d=r['d'];c=d.get('candidate_diagnostics',{});st=d['state'];checked=c.get('checked_inputs',{});cap=c.get('capture_inputs',{})
  modes[d.get('mode',c.get('mode','unknown'))]+=1
  if st=='command_delivered':succmodes[d.get('mode','unknown')]+=1
  if st!='command_delivered' and c.get('feasible',0)>0:valid_plan_rejected+=1
  for k in ('planning_elapsed_ms','first_step_check_ms','delivery_elapsed_ms','delivery_lock_wait_ms','delivery_lock_hold_ms'):
   if k in c:timings[k].append(c[k])
  timings['cycle_ms'].append(d.get('cycle_ms',0))
  if 'map_age_ms' in cap:timings['capture_map_age_ms'].append(cap['map_age_ms'])
  if 'snapshot_callback' in cap:timings['snapshot_decode_ms'].append(cap['snapshot_callback'].get('decode_ms',0))
  if st in ('collision_snapshot_unavailable','collision_snapshot_expired_during_plan','inputs_expired_before_delivery'):
   bad=tuple(k for k in ('map_age_ok','dynamic_age_ok','snapshot_valid','dynamic_valid','base_fresh','joint_fresh') if checked.get(k) is False);flags[str(bad)]+=1
   for k in bad:snap_failed[k]+=1
  if st=='command_delivered':
   for k in ('base_v','base_w'):timings['delivered_'+k].append(d.get(k,0))
 zeros=[];start=None;prev=False;trans=[]
 for i,r in enumerate(cs):
  z=abs(r['v'])<1e-8 and abs(r['w'])<1e-8
  if z and not prev:
   start=r['t'];before=cs[i-1] if i else None
   if before and (abs(before['v'])>.015 or abs(before['w'])>.05):trans.append({'t':start,'time':clock(start),'before':before,'zero':r})
  if not z and prev and start is not None:zeros.append((start,r['t']-start));start=None
  prev=z
 if prev and start is not None:zeros.append((start,hi-start))
 paired=collections.Counter();examples=[]
 for tr in trans:
  near=[r for r in ss if abs(r['t']-tr['t'])<=.08 and r['d']['state']!='command_delivered']
  if near:
   near.sort(key=lambda r:abs(r['t']-tr['t']));r=near[0];state=r['d']['state'];paired[state]+=1
   if len(examples)<16:
    ex={'time':tr['time'],'from_v':tr['before']['v'],'from_w':tr['before']['w'],'state':state,'receipt_dt_ms':round((r['t']-tr['t'])*1000,3),'cycle_ms':r['d'].get('cycle_ms'),'checked':{k:v for k,v in r['d'].get('candidate_diagnostics',{}).get('checked_inputs',{}).items() if k in ('map_age_ms','dynamic_age_ms','base_age_ms','joint_age_ms','map_age_ok','dynamic_age_ok','snapshot_valid','dynamic_valid','base_fresh','joint_fresh')}};examples.append(ex)
  else:paired['unmatched']+=1
 odist=math.hypot(os[-1]['x']-goals[0 if name.startswith('goal1') else 1]['x'],os[-1]['y']-goals[0 if name.startswith('goal1') else 1]['y']) if os else None
 result[name]={'start':clock(lo),'end':clock(hi),'duration_s':round(hi-lo,3),'mppi_cycles':len(ss),'states':dict(counter),'rejected_pct':round(100*(len(ss)-counter['command_delivered'])/len(ss),2),'feasible_candidate_but_rejected_count':valid_plan_rejected,'rejection_bad_flag_combinations':dict(flags),'rejection_bad_flags':dict(snap_failed),'modes':dict(modes),'delivered_modes':dict(succmodes),'timings':{k:q(v) for k,v in timings.items()},'cmd_count':len(cs),'zero_command_count':sum(abs(r['v'])<1e-8 and abs(r['w'])<1e-8 for r in cs),'zero_time_s':round(sum(d for t,d in zeros),3),'zero_runs_s':q([d for t,d in zeros]),'moving_to_zero_transitions':len(trans),'nearby_rejection_for_transition':dict(paired),'examples':examples,'trajectory_messages':len(ts),'trajectory_cancels':sum(t['action']==2 for t in ts),'max_dynamic_output_points':max((r['points'] for r in ps),default=0),'nonempty_dynamic_point_frames':sum(r['points']>0 for r in ps),'last_position_error_m':odist,'odom_end':os[-1] if os else None}
# exact source-time causes next to each cancellation
for t in [r for r in traj if r['action']==2]:
 cand=[l for l in logs if t['t']-.2<=l['source_t']<=t['t']+.1 and any(x in l['msg'] for x in ('LiveCollisionStop','Depth Lost','Live input timed out','tracking fault','Tracking fault','Emergency stop'))]
 cancel_details.append({'time':clock(t['t']),'t':t['t'],'logs':cand})
(A/'navigation_quantified.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');(A/'cancel_causes.json').write_text(json.dumps(cancel_details,ensure_ascii=False,indent=2)+'\n')
for name,d in result.items():print(name,json.dumps({k:v for k,v in d.items() if k not in ('examples','timings','odom_end')},ensure_ascii=False));print('timings',d['timings']);print('examples',d['examples'][:5])
print('cancel_count',len(cancel_details))
