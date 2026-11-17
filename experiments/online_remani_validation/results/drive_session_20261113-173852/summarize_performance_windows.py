#!/usr/bin/env python3
"""Offline metadata/log analysis; no ROS connection or robot commands."""
import collections
import datetime
import json
from pathlib import Path
import re
import statistics

OUT=Path(__file__).resolve().parent
base=json.loads((OUT/'performance_stutter_analysis.json').read_text())
logs=[json.loads(x) for x in (OUT/'performance_runtime_logs.jsonl').open()]
statuses=[json.loads(x) for x in (OUT/'performance_small_status.jsonl').open()]
pipeline=json.loads((OUT/'performance_pipeline_metrics.json').read_text())
complete_path=OUT/'runtime_logs/hai_navigation_logs.jsonl'
if complete_path.exists():
    # /rosout_agg in the bag missed safety/recovery events. Prefer the complete
    # source-stamped compact log for event boundaries and performance windows.
    logs=[]
    for line in complete_path.open():
        r=json.loads(line)
        logs.append(dict(source=r['source'],clock=datetime.datetime.fromtimestamp(r['source']).strftime('%H:%M:%S.%f')[:-3],node=r['node'],text=r['message']))
    pattern=re.compile(r'window=([\d.]+)s mode=(\w+) occ=(\d+) avg/max=([\d.]+)/([\d.]+)ms esdf=(\d+) compute_avg/max=([\d.]+)/([\d.]+)ms main_commit_avg/max=([\d.]+)/([\d.]+)ms snapshot_avg/max=([\d.]+)/([\d.]+)ms age=([\d.]+)ms')
    pipeline=[]
    for r in logs:
        m=pattern.search(r['text'])
        if not m:continue
        values=m.groups()
        keys=['window_s','mode','occ_n','occ_avg_ms','occ_max_ms','esdf_n','esdf_compute_avg_ms','esdf_compute_max_ms','main_commit_avg_ms','main_commit_max_ms','snapshot_avg_ms','snapshot_max_ms','age_ms']
        row={k:float(v) for k,v in zip(keys,values) if k!='mode'}
        row.update(source=r['source'],clock=r['clock'],text=r['text'],mode=values[1]);pipeline.append(row)
    (OUT/'performance_pipeline_metrics_complete_log.json').write_text(json.dumps(pipeline,indent=2)+'\n')
snapshots=[json.loads(x) for x in (OUT/'snapshot_headers.jsonl').open()]
clouds=collections.defaultdict(list)
for line in (OUT/'cloud_headers.jsonl').open():
    r=json.loads(line);clouds[r['topic']].append(r)

def clock(t):return datetime.datetime.fromtimestamp(t).strftime('%H:%M:%S.%f')[:-3]
def stats(a):
    a=sorted(a)
    if not a:return dict(n=0)
    return dict(n=len(a),min=a[0],median=statistics.median(a),p95=a[int((len(a)-1)*.95)],max=a[-1])

new=[];last=None
for r in snapshots:
    if r['header'][3]!=last:new.append(r);last=r['header'][3]

local=[]
for r in logs:
    m=re.search(r'changed=(\d+) input_bound=(\d+)/(\d+) voxels, cap=([\d.]+)m; full=(\w+)',r['text'])
    if m:
        local.append(dict(source=r['source'],clock=r['clock'],changed=int(m[1]),input_bound_voxels=int(m[2]),global_voxels=int(m[3]),cap=float(m[4]),full=m[5]))

giveups=[r for r in logs if 'Give up current target and return to WAIT_TARGET' in r['text']]
depth=[r for r in logs if r['text']=='Depth Lost! EMERGENCY_STOP']
recover=[r for r in logs if 'validated a new recovery trajectory' in r['text']]

def summary(name,start,end):
    ss=[r for r in statuses if start<=r['receive']<end]
    pp=[r for r in pipeline if start<=r['source']<end]
    hh=[r for r in snapshots if start<=r['receive_ros']<end]
    nn=[r for r in new if start<=r['receive_ros']<end]
    ll=[r for r in local if start<=r['source']<end]
    counts=collections.Counter(r['state'] for r in ss)
    sc=[r for r in ss if r['snapshot']]
    out=dict(name=name,start=start,end=end,start_clock=clock(start),end_clock=clock(end),
        status_n=len(ss),states=dict(counts),delivered_modes=dict(collections.Counter(r['mode'] for r in ss if r['state']=='command_delivered')),
        snapshot_input_rejections=sum(counts[k] for k in ['collision_snapshot_unavailable','collision_snapshot_expired_during_plan','inputs_expired_before_delivery']),
        explicit_map_age_over350_count=sum(r['snapshot']['map_age_ms']>350 for r in sc),
        map_age_over350_by_state=dict(collections.Counter(r['state'] for r in sc if r['snapshot']['map_age_ms']>350)),
        snapshot_invalid_flag_count=sum(not r['snapshot']['valid'] for r in sc),
        dynamic_invalid_count=sum(not r['snapshot']['dynamic_valid'] for r in sc),
        deadline95_rejections=counts['planning_deadline_exceeded']+counts['delivery_deadline_exceeded'],
        cycle_ms_by_state={k:stats([r['cycle_ms'] for r in ss if r['state']==k]) for k in counts},
        new_map_first_visible_age_ms=stats([(r['receive_ros']-r['header'][3])*1000 for r in nn]),
        new_map_interval_ms=stats([(b['header'][3]-a['header'][3])*1000 for a,b in zip(nn,nn[1:])]),
        first_map_copy_acq_minus_commit_ms=stats([(r['header'][17]-r['header'][3])*1000 for r in nn]),
        first_map_receive_minus_copy_acq_ms=stats([(r['receive_ros']-r['header'][17])*1000 for r in nn]),
        snapshot_packet_n=len(hh),snapshot_valid_fraction=(sum(r['header'][4]==1 for r in hh)/len(hh) if hh else None),
        snapshot_age_over350_at_receive_count=sum(r['receive_ros']-r['header'][3]>.35 for r in hh),
        local_changed=stats([r['changed'] for r in ll]),local_input_bound_voxels=stats([r['input_bound_voxels'] for r in ll]),
        pipeline_window_n=len(pp),pipeline_window_average_stats={k:stats([r[k] for r in pp]) for k in ['occ_avg_ms','esdf_compute_avg_ms','main_commit_avg_ms','snapshot_avg_ms']},
        pipeline_window_maxima={k:max([r[k] for r in pp],default=None) for k in ['occ_max_ms','esdf_compute_max_ms','main_commit_max_ms','snapshot_max_ms']},
        cloud_source_metadata={})
    for topic,rows in clouds.items():
        rr=[r for r in rows if start<=r['source_stamp']<end]
        stamps=sorted(set(r['source_stamp'] for r in rr))
        out['cloud_source_metadata'][topic]=dict(n=len(rr),source_interval_ms=stats([(b-a)*1000 for a,b in zip(stamps,stamps[1:])]),source_to_record_ms=stats([(r['receive_ros']-r['source_stamp'])*1000 for r in rr]))
    return out

goals=base['goals'];observation=[]
for i,g in enumerate(goals):observation.append(summary('goal%d_observation'%(i+1),g['source'],goals[i+1]['source'] if i+1<len(goals) else base['end']))
healthy=[]
for i,g in enumerate(goals):
    end=goals[i+1]['source'] if i+1<len(goals) else base['end']
    events=sorted([(r['source'],'giveup') for r in giveups if g['source']<r['source']<end]+[(r['source'],'depth') for r in depth if g['source']<r['source']<end]+[(r['source'],'recovery') for r in recover if g['source']<r['source']<end])
    begin=g['source'];paused=False;abandoned=False;part=0
    for t,event in events:
        if event in ('giveup','depth') and not paused:
            part+=1;healthy.append(summary('goal%d_no_perception_stop_part%d'%(i+1,part),begin,t));paused=True
            if event=='giveup':abandoned=True
        elif event=='recovery' and paused and not abandoned:begin=t;paused=False
    if not paused:
        part+=1;healthy.append(summary('goal%d_no_perception_stop_part%d'%(i+1,part),begin,end))

payload=dict(caveats=['Goal observation windows end at next goal or bag end; there was no planning/finish topic in this capture.',
    'Event boundaries and pipeline metrics use runtime_logs/hai_navigation_logs.jsonl when available. Aggregated /rosout_agg in the bag missed multiple Depth Lost/recovery messages; absence there was not proof of a continuous stop.',
    'No-perception-stop segments exclude source-log Depth Lost until validated recovery and truncate source-log Give up. They are not a claim of continuous physical motion.',
    'Source clocks are used for logs/goals/cloud windows; status and cmd have no source header and use recorded receive time.',
    'Each pipeline point summarizes preceding 5 seconds and may straddle a segment boundary; medians are medians of window averages.',
    'Cloud headers record all received callbacks after payload budget exhaustion; payload absence is not sensor absence.',
    'h3 is occupancy commit time and h17 ESDF-copy acquisition time; neither is raw sensor source time.',
    'Different paths, live points, occupancy changes and CPU load mean this is not a strict controlled A/B against 15:43.'],
    goals=goals,finish_topic=base['finish_topic'],observation_windows=observation,no_perception_stop_segments=healthy,
    giveup_logs=giveups,depth_lost_logs=depth,recovery_logs=recover,local_esdf_records=local)
(OUT/'map_performance_window_summary.json').write_text(json.dumps(payload,indent=2)+'\n')
for r in observation+healthy:
    print(json.dumps({k:r[k] for k in ['name','start_clock','end_clock','status_n','snapshot_input_rejections','explicit_map_age_over350_count','deadline95_rejections','new_map_first_visible_age_ms','new_map_interval_ms','snapshot_valid_fraction','pipeline_window_average_stats','local_input_bound_voxels']}))
