import os,time,json,re,math,collections,statistics,resource,hashlib,datetime,bisect
from pathlib import Path
import rosbag
ROOT=Path('/home/l/remani_ws');CASE=ROOT/'experiments/online_remani_validation/results/dynamic_crossing_20261116-195203/navigation_segment';OUT=CASE/'analysis';OUT.mkdir(exist_ok=True)
started=time.monotonic();bagpath=CASE/'core.bag';traces=collections.defaultdict(list);warnings=collections.Counter();examples=collections.defaultdict(list);goals=[];tracker=collections.defaultdict(list);source_delays=[];seen=0
numeric=lambda x:float(x) if re.fullmatch(r'-?[0-9]+(?:\.[0-9]+)?(?:e[-+]?[0-9]+)?',x,re.I) else x
pattern=re.compile(r'([A-Za-z_][A-Za-z0-9_]*)=([^\s]+)')
with rosbag.Bag(str(bagpath),'r') as bag:
 bstart,bend=bag.get_start_time(),bag.get_end_time()
 topics=['/rosout_agg','/move_base_simple/goal','/dynamic_obstacle_tracker/input_age','/dynamic_obstacle_tracker/processing_ms']
 # Discard in-memory indexes of unselected topics; never changes bag bytes.
 bag._connection_indexes={cid:entries for cid,entries in bag._connection_indexes.items() if bag._connections[cid].topic in topics}
 import gc;gc.collect()
 for topic,msg,t in bag.read_messages(topics=topics):
  receipt=t.to_sec();seen+=1
  if seen%500==0 and resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024>240:raise RuntimeError('240MiB parser safety stop')
  if topic=='/move_base_simple/goal':goals.append({'receipt':receipt,'stamp':msg.header.stamp.to_sec(),'x':msg.pose.position.x,'y':msg.pose.position.y});continue
  if topic!='/rosout_agg':tracker[topic].append((receipt,float(msg.data)));continue
  text=msg.msg;stamp=msg.header.stamp.to_sec()
  tag=('MapTrace' if '[MapTrace]' in text else 'LatencyTrace' if '[LatencyTrace]' in text else None) if msg.name=='/remani_planner_node' else None
  if tag:
   row={k:numeric(v) for k,v in pattern.findall(text)};stage=str(row.pop('stage','unknown'));row.update(source_stamp=stamp,receipt=receipt,node=msg.name)
   traces[(msg.name,tag,stage)].append(row);source_delays.append((receipt-stamp)*1000)
  if msg.name=='/remani_planner_node' and any(x in text for x in ['[GridMap]','[MPPI snapshot]']):
   if any(x in text for x in ['timed out','exceeds','Discarded','Rejected','Failed','recovered','LocalESDF','coalesced']):
    category='timeout' if 'timed out' in text else 'pending_discard' if 'Discarded pending' in text else 'esdf_rejected' if 'Rejected' in text and 'ESDF' in text else 'budget_exceeded' if 'exceeds' in text else 'other'
    warnings[category]+=1
    if len(examples[category])<5:examples[category].append({'source_stamp':stamp,'receipt':receipt,'text':text[:2000]})
bag._connection_indexes.clear()
import gc;gc.collect()
phases=json.loads((CASE/'phase_marks.json').read_text())['marks'];nav_start=goals[0]['receipt'] if goals else next(x['wall'] for x in phases if x['phase']=='user_transfer_goal_published_running');nav_end=next((x['wall'] for x in phases if x['phase']=='user_cancelled_dynamic_test_analyze_navigation_only'),bend)
def stats(values):
 a=sorted(v for v in values if isinstance(v,(float,int)) and math.isfinite(v))
 if not a:return {'n':0}
 def p(q):
  ix=(len(a)-1)*q;lo=int(ix);hi=min(len(a)-1,lo+1);return a[lo]+(a[hi]-a[lo])*(ix-lo)
 return {'n':len(a),'min':a[0],'p50':p(.5),'p95':p(.95),'p99':p(.99),'max':a[-1],'mean':sum(a)/len(a)}
def scope(rows):return [r for r in rows if nav_start<=r.get('source_stamp',r.get('receipt',0))<=nav_end]
def summarize(rows):
 result={'records':len(rows)}
 if rows:
  times=[r['source_stamp'] for r in rows];result.update(first=min(times),last=max(times),observed_hz=(len(times)-1)/(max(times)-min(times)) if max(times)>min(times) else 0,interval_ms=stats((b-a)*1000 for a,b in zip(times,times[1:])))
  fields=sorted({k for r in rows for k,v in r.items() if isinstance(v,(float,int)) and k not in ['receipt','source_stamp']})
  result['numeric_fields']={k:stats(r[k] for r in rows if k in r) for k in fields}
 return result
stages={node+'|'+tag+'|'+stage:summarize(scope(rows)) for (node,tag,stage),rows in traces.items()}
planner=lambda tag,stage:scope(traces.get(('/remani_planner_node',tag,stage),[]))
fusions=planner('MapTrace','fusion');commits=planner('MapTrace','commit');submits=planner('MapTrace','submit');reads=planner('MapTrace','read');regions=planner('MapTrace','region');exports=planner('LatencyTrace','export_sample');outputs=planner('LatencyTrace','export_output')
by_obs={int(r['obs']):r for r in fusions if 'obs' in r};by_id={int(r['id']):r for r in submits if 'id' in r};read_by_rev={int(r['rev']):r for r in reads if r.get('complete')==1 and 'rev' in r}
chain=[]
for c in commits:
 f=by_obs.get(int(c.get('obs',-1)));s=by_id.get(int(c.get('id',-1)));r=read_by_rev.get(int(c.get('rev',-1)));row={'obs':c.get('obs'),'id':c.get('id'),'rev':c.get('rev'),'fusion':c.get('fusion'),'commit_log_stamp':c['source_stamp']}
 for k in ['queue_ms','compute_ms','ready_wait_ms','commit_ms','compute_cells','input_cells','regions']:row[k]=c.get(k)
 if f:
  row['source_to_fusion_ms']=f['input_age_ms'];row['pending_ms']=f['pending_ms'];row['occupancy_ms']=f['occupancy_ms'];row['measurement_stamp']=f['input'];row['measurement_to_commit_ms']=(c['source_stamp']-f['input'])*1000
 if s:
  row['snapshot_ms']=s.get('snapshot_ms');row['fusion_to_capture_ms']=(s.get('capture_mono',0)-0)*0 if not f else (s['source_stamp']-f['fusion'])*1000
 if r:row['commit_to_complete_read_ms']=(r['source_stamp']-c['source_stamp'])*1000;row['read_total_ms']=r.get('total_ms')
 row['fusion_to_commit_ms']=(c['source_stamp']-c.get('fusion',c['source_stamp']))*1000;chain.append(row)
# Snapshot headers are read as scalar metadata only, never full grids.
headers=[]
with (CASE/'snapshot_headers.jsonl').open() as f:
 for line in f:
  d=json.loads(line);h=d['header'];t=d['receive_ros']
  if nav_start<=t<=nav_end:
   headers.append({'receipt':t,'fusion':h[3],'wire':int(h[2]),'valid':bool(h[4]),'field':int(h[14]),'max_age_ms':h[15]*1000,'full':bool(h[20]),'dynamic_valid':bool(h[19]),'joint_stamp':h[24],'sample_acquired':h[17],'obs':int(h[29]),'accepted':int(h[30]),'fused':int(h[31]),'cells':int(h[5]*h[6]*h[7]),'bytes':d['bytes'],'map_age_receive_ms':(t-h[3])*1000,'sample_to_receive_ms':(t-h[17])*1000,'dynamic_age_receive_ms':(t-h[18])*1000,'joint_age_receive_ms':(t-h[24])*1000})
unique_fields={};unique_obs={}
for h in headers:unique_fields.setdefault(h['field'],h);unique_obs.setdefault(h['obs'],h)
first_exports={}
for e in exports:first_exports.setdefault(int(e.get('field',-1)),e)
commit_by_obs={int(c['obs']):c for c in commits};snapshot_chain=[]
for h in unique_fields.values():
 e=first_exports.get(h['field']);c=commit_by_obs.get(h['obs']);f=by_obs.get(h['obs']);row={'field':h['field'],'obs':h['obs'],'first_wire':h['wire'],'first_receipt':h['receipt'],'first_receive_age_ms':h['map_age_receive_ms']}
 if e:row['sample_to_receive_ms']=(h['receipt']-e['stamp'])*1000
 if c:row['commit_to_receive_ms']=(h['receipt']-c['source_stamp'])*1000
 if f:row['measurement_to_receive_ms']=(h['receipt']-f['input'])*1000
 snapshot_chain.append(row)
# CPU ticks are all process threads; schedstat belongs only to the process leader.
resource_metrics=collections.defaultdict(lambda:collections.defaultdict(list));previous={};mem=[]
HZ=os.sysconf('SC_CLK_TCK')
with (CASE/'resources.jsonl').open() as f:
 for line in f:
  d=json.loads(line);t=d['wall']
  if not nav_start<=t<=nav_end:continue
  mem.append(d.get('mem_kib',{}).get('MemAvailable',0)/1024)
  for p in d.get('robot_processes',[]):
   if not p.get('stat'):continue
   text=p['stat'];fields=text[text.rindex(')')+2:].split();cpu=(int(fields[11])+int(fields[12]))/HZ;name=text[text.index('(')+1:text.rindex(')')];sched=[int(v) for v in p.get('schedstat','0 0 0').split()];pid=p['pid'];old=previous.get(pid)
   if old and t>old[0]:
    dt=t-old[0];resource_metrics[name]['cpu_one_core_percent'].append(100*(cpu-old[1])/dt);resource_metrics[name]['leader_run_ms_per_second'].append((sched[0]-old[2][0])/1e6/dt);resource_metrics[name]['leader_runqueue_wait_ms_per_second'].append((sched[1]-old[2][1])/1e6/dt)
   previous[pid]=(t,cpu,sched)
parameters=json.loads((CASE/'startup_parameters.json').read_text())['parameters'];cfg=parameters.get('remani_planner_node',{});mp=cfg.get('grid_map',{});selected={k:v for k,v in mp.items() if any(x in k for x in ['warn','esdf','local_update','resolution','lidar','parallel','thread','map_size'])};selected['export_dynamic_max_age']=cfg.get('dynamic_point_flow',{}).get('max_age');selected['export_mppi']=cfg.get('mppi')
result={'schema':'perception_navigation_audit_v1','scope':{'case':str(CASE),'core_bag_bytes':bagpath.stat().st_size,'bag_start':bstart,'bag_end':bend,'navigation_start':nav_start,'navigation_end':nav_end,'goals':goals,'intentional_dynamic_test':False,'no_point_cloud_or_geometry_decode':True},'trace_stage_statistics_navigation':stages,'chain_statistics':{k:stats(r[k] for r in chain if isinstance(r.get(k),(float,int))) for k in sorted({k for r in chain for k in r}) if k not in ['obs','id','rev','fusion','measurement_stamp','commit_log_stamp']},'snapshot_chain_statistics':{k:stats(r[k] for r in snapshot_chain if k in r) for k in ['first_receive_age_ms','sample_to_receive_ms','commit_to_receive_ms','measurement_to_receive_ms']},'snapshot_headers_navigation':{'records':len(headers),'invalid':sum(not h['valid'] for h in headers),'map_age_over_export_TTL_at_receive':sum(h['map_age_receive_ms']>h['max_age_ms'] for h in headers),'dynamic_invalid':sum(not h['dynamic_valid'] for h in headers),'joint_age_over_150ms_at_receive':sum(h['joint_age_receive_ms']>150 for h in headers),'full_field_packets':sum(h['full'] for h in headers),'distinct_fields':len(unique_fields),'distinct_observations':len(unique_obs),'field_update_interval_ms':stats((b['receipt']-a['receipt'])*1000 for a,b in zip(unique_fields.values(),list(unique_fields.values())[1:])),'numeric':{k:stats(h[k] for h in headers) for k in ['map_age_receive_ms','sample_to_receive_ms','dynamic_age_receive_ms','joint_age_receive_ms','cells','bytes']},'first_rows':headers[:3]},'locality_evidence':{'region_full_counts':dict(collections.Counter(str(r.get('full','missing')) for r in regions)),'region_reuse_counts':dict(collections.Counter(str(r.get('reuse','missing')) for r in regions)),'worker_region_counts':dict(collections.Counter(str(r.get('regions','missing')) for r in commits)),'worker_compute_over_50ms':sum(c.get('compute_ms',0)>50 for c in commits),'worker_input_equals_compute':sum(c.get('compute_cells')==c.get('input_cells') for c in commits),'map_read_copied_occupancy':sum(r.get('occ_copy')==1 for r in reads),'map_read_copied_distance':sum(r.get('dist_copy')==1 for r in reads)},'tracker_successful_callback_statistics':{k:stats(v for t,v in values if nav_start<=t<=nav_end) for k,values in tracker.items()},'resources':{'host_available_MiB':stats(mem),'process_metrics':{n:{k:stats(v) for k,v in metrics.items()} for n,metrics in resource_metrics.items()},'boundary':'CPU ticks cover all threads; /proc/PID/schedstat covers only leader thread. No worker-thread schedstat or system-wide CPU frequency/runqueue trace was recorded.'},'map_warning_counts_whole_segment':dict(warnings),'map_warning_examples_whole_segment':dict(examples),'runtime_configuration':selected,'worst_compute_chains':sorted(chain,key=lambda x:x.get('compute_ms',0),reverse=True)[:8],'worst_first_snapshot_chains':sorted(snapshot_chain,key=lambda x:x.get('first_receive_age_ms',0),reverse=True)[:8],'trace_log_transport_delay_ms_all':stats(source_delays),'analysis':{'peak_rss_MiB':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,'elapsed_s':time.monotonic()-started,'selected_messages_read':seen},'boundaries':['Snapshot receiver age includes transport/callback delay and can exceed the validity check time; export_output trace distinguishes publisher-side age.','A 50ms compute warning budget is a timing diagnostic, not the 350ms map-authority TTL.','Healthy/fresh input does not imply a newly completed ESDF is available.','No claim that CPU saturation alone caused all latency; worker scheduler and locks are not directly traced.']}
# Interval correction: the recording-stage mark occurs after the final navigation finish.
last_finish = datetime.datetime.fromisoformat('2026-11-16T20:07:50.841+08:00').timestamp()
result['scope'].update(interval_label='First goal to collection-stage end; includes final post-finish idle.', last_finish=last_finish, post_finish_idle_included_s=nav_end-last_finish, navigation_end_name_boundary='Legacy key; collection-stage mark, not final navigation finish.')
result['boundaries'].append('LiDAR heartbeat timeouts at20:08:26/27 occur after final finish20:07:50.841 and do not explain preceding navigation stutter; strict results are separate.')
p=OUT/'perception_navigation_audit.json';p.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({'output':str(p),'scope':result['scope'],'chain':result['chain_statistics'],'snapshot_chain':result['snapshot_chain_statistics'],'locality':result['locality_evidence'],'snapshot':result['snapshot_headers_navigation'],'analysis':result['analysis']},indent=2))
