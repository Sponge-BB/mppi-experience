import json,re,collections,math
from pathlib import Path
import numpy as np
P=Path('/home/l/remani_ws/experiments/online_remani_validation/results/dynamic_geometry_capture_20261114-175950/analysis_clear_stutter')
START=1794650911.082;END=1794651028.0395
rows=[json.loads(l) for l in (P/'status.jsonl').open()]
rows=[r for r in rows if START<=r.get('source',r['bag'])<=END]
def stat(x):
 a=np.asarray(x,dtype=float);a=a[np.isfinite(a)]
 return {'n':len(a),'min':float(a.min()),'median':float(np.median(a)),'p95':float(np.percentile(a,95)),'max':float(a.max())} if len(a) else {'n':0}
measures=collections.defaultdict(list);by_state={};callbacks={};copies={};rowsbyfield=collections.defaultdict(list);bad=[];example=[]
for r in rows:
 checked=r.get('checked') or {};capture=r.get('capture') or {};h=checked.get('snapshot_header');cap_h=capture.get('snapshot_header');t=checked.get('checked_at')
 if h and t:
  vals=dict(map_age_ms=1e3*(t-h[3]),copy_acquisition_to_check_ms=1e3*(t-h[17]),cached_joint_age_ms=1e3*(t-h[24]),remaining_ms=350-1e3*(t-h[3]))
  cb=checked.get('snapshot_callback')
  if cb:
   callbacks[cb['wire_seq']]=(h,cb)
   vals.update(adoption_to_check_ms=1e3*(t-cb['adopted']),callback_enter_to_adopt_ms=1e3*(cb['adopted']-cb['enter']))
   if h[20]==1 and h[17]>=h[3]:
    copies[(h[14],h[17])]=dict(field=h[14],fusion=h[3],acquired=h[17],cb=cb)
  if cap_h:
   vals['capture_map_age_ms']=1e3*(capture['checked_at']-cap_h[3]); vals['capture_budget_ms']=350-vals['capture_map_age_ms']
   vals['capture_to_last_check_ms']=1e3*(t-capture['checked_at'])
  dst=by_state.setdefault(r['state'],collections.defaultdict(list))
  for k,v in vals.items():measures[k].append(v);dst[k].append(v)
  if h[4]!=1 and -50<=vals['map_age_ms']<=350:bad.append(dict(source=t,state=r['state'],header=h,values=vals))
  rowsbyfield[h[14]].append((t,r['state'],h))
for k,(h,cb) in callbacks.items():
 for key in ['read_lock_wait_ms','decode_ms','write_lock_wait_ms']:
  measures['callback_'+key].append(cb[key]);measures[('full_' if h[20]==1 else 'delta_')+key].append(cb[key])
for c in copies.values():
 measures['full_fusion_to_copy_acquire_ms'].append(1000*(c['acquired']-c['fusion']))
 measures['full_copy_acquire_to_callback_enter_ms'].append(1000*(c['cb']['enter']-c['acquired']))
 measures['full_fusion_to_callback_adopt_ms'].append(1000*(c['cb']['adopted']-c['fusion']))
seq=sorted(copies.values(),key=lambda c:c['acquired'])
measures['full_copy_acquisition_interval_ms']=[1000*(b['acquired']-a['acquired']) for a,b in zip(seq,seq[1:])]
pat=re.compile(r'window=([\d.]+)s mode=(\w+) occ=(\d+) avg/max=([\d.]+)/([\d.]+)ms esdf=(\d+) compute_avg/max=([\d.]+)/([\d.]+)ms main_commit_avg/max=([\d.]+)/([\d.]+)ms snapshot_avg/max=([\d.]+)/([\d.]+)ms age=([\d.]+)ms obs_lag=(\d+).*?clouds_coalesced=(\d+) async\(submit/commit/reject/occ_defer\)=(\d+)/(\d+)/(\d+)/(\d+)')
perf=[];regions=[];changed=[];mapdebug=0
for l in (P/'logs.jsonl').open():
 r=json.loads(l);t=r.get('source',r['bag']);msg=r['msg']
 if not START<=t<=END:continue
 if '[MapTrace]' in msg or '[LatencyTrace]' in msg:mapdebug+=1
 m=pat.search(msg)
 if m:
  v=m.groups();perf.append(dict(source=t,bag=r['bag'],window=float(v[0]),mode=v[1],occ=int(v[2]),occ_avg_ms=float(v[3]),occ_max_ms=float(v[4]),esdf=int(v[5]),compute_avg_ms=float(v[6]),compute_max_ms=float(v[7]),commit_avg_ms=float(v[8]),commit_max_ms=float(v[9]),snapshot_avg_ms=float(v[10]),snapshot_max_ms=float(v[11]),result_age_ms=float(v[12]),obs_lag=int(v[13]),clouds_coalesced=int(v[14]),submit=int(v[15]),commit=int(v[16]),reject=int(v[17]),occ_defer=int(v[18])))
 m=re.search(r'committed regions=(\d+) compute=(\d+)/(\d+) input-bound voxels, worker=([\d.]+)ms commit=([\d.]+)ms',msg)
 if m:regions.append(dict(source=t,regions=int(m[1]),compute_cells=int(m[2]),input_cells=int(m[3]),worker_ms=float(m[4]),commit_ms=float(m[5])))
 m=re.search(r'changed=(\d+) input_bound=(\d+)/(\d+) voxels, cap=([\d.]+)m; full=(\w+)',msg)
 if m:changed.append(dict(source=t,changed=int(m[1]),input_cells=int(m[2]),total_cells=int(m[3]),cap=float(m[4]),full=m[5]))
weighted={}
for key,weight in [('occ_avg_ms','occ'),('compute_avg_ms','esdf'),('commit_avg_ms','esdf'),('snapshot_avg_ms','submit')]:
 weighted[key]=sum(r[key]*r[weight] for r in perf)/sum(r[weight] for r in perf)
summary=dict(scope='Status source/check times, callback timestamps and 5s source-stamped pipeline summaries; no large bag read. Counters/averages cover complete 5s windows, not exact single-command critical paths.',period=[START,END],status_count=len(rows),states=dict(collections.Counter(r['state'] for r in rows)),metrics={k:stat(v) for k,v in measures.items()},by_state={s:{k:stat(v) for k,v in d.items()} for s,d in by_state.items()},invalid_header_but_age_within350ms=bad,unique_checked_wires=len(callbacks),full_field_copies_seen=len(copies),pipeline_windows=len(perf),weighted_pipeline_ms=weighted,pipeline_totals={k:sum(r[k] for r in perf) for k in ['window','occ','esdf','submit','commit','reject','occ_defer','clouds_coalesced']},pipeline_window_metrics={k:stat([r[k] for r in perf]) for k in ['occ_max_ms','compute_max_ms','commit_max_ms','snapshot_max_ms','result_age_ms']},region_samples=regions,changed_samples=changed,pipeline_rows=perf,map_debug_trace_count=mapdebug)
(P/'map_chain_review.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({k:v for k,v in summary.items() if k not in ['invalid_header_but_age_within350ms','pipeline_rows','region_samples','changed_samples','by_state']},indent=2))
print('invalid_yet_age_ok_n',len(bad),[(r['state'],r['values']['map_age_ms'],r['values']['cached_joint_age_ms'],r['header'][19],r['header'][22:24]) for r in bad])
print('regions',len(regions),collections.Counter(r['regions'] for r in regions),stat([r['compute_cells'] for r in regions]));print('changed',len(changed),stat([r['changed'] for r in changed]))
# Acquisitions reconstructed from all capture/check headers, including deltas.
# h17 remains cached on deltas; it is not a packet publication timestamp.
all_fields={};counts=collections.Counter();crossing=collections.Counter()
for r in rows:
 for stage in ['capture','checked']:
  c=r.get(stage) or {};h=c.get('snapshot_header')
  if h:all_fields[(h[14],h[17],h[3])]=h
 c=r.get('checked') or {};h=c.get('snapshot_header');old=(r.get('capture') or {}).get('snapshot_header')
 if not h:continue
 counts['header_invalid']+=h[4]!=1
 counts['header_invalid_and_map_expired']+=h[4]!=1 and not c['map_age_ok']
 counts['header_invalid_age_ok_dynamic_invalid']+=h[4]!=1 and c['map_age_ok'] and h[19]!=1
 if r['state']=='collision_snapshot_unavailable':
  counts['capture_rejected']+=1
  counts['capture_rejected_map_expired']+=not c['map_age_ok']
  counts['capture_header_valid_but_now_expired']+=h[4]==1 and not c['map_age_ok']
 if old and r['state'] in ['collision_snapshot_expired_during_plan','inputs_expired_before_delivery']:
  crossing[r['state']+('_same_field' if h[14]==old[14] else '_new_field')]+=1
summary['all_observed_fields']=len(all_fields)
summary['all_field_fusion_to_copy_acquisition_ms']=stat([1000*(h[17]-h[3]) for h in all_fields.values()])
acqs=sorted(set(h[17] for h in all_fields.values()))
summary['observed_field_acquisition_intervals_ms']=stat(np.diff(acqs)*1000.)
summary['validity_contingency']=dict(counts)
summary['expire_during_cycle_field_changes']=dict(crossing)
summary['limits']=['No MapTrace/LatencyTrace DEBUG events were captured in this window, so per-job prepare/queue/compute-ready wait/immutable copy/export wait cannot be separately measured.', 'h3 is occupancy fusion COMPLETION ROS time in grid_map.cpp, not LiDAR scan time. End-to-end sensor latency is therefore larger and not recoverable from h3 alone.', 'h17 is field cache refresh acquisition time before crop copy, retained on reused fields. h17-to-callback must not be called pure communication delay.', 'Full packet callback samples are a subset observed by status, so intervals between those samples are not the actual exporter rate.', 'All-field observed acquisition intervals may miss fields not selected at any status check; actual completed throughput is better derived from pipeline commit counts.', 'Five-second pipeline counters are source-time filtered but span their full window; do not interpret their component sums as a trace of one specific control.', 'Occasional slight negative adoption-to-check values arise because cycle time is captured before the lock/callback adoption; they do not represent physical time travel.']
(P/'map_chain_review.json').write_text(json.dumps(summary,indent=2)+'\n')
