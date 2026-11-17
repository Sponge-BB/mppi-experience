#!/usr/bin/env python3
"""Stream recorded JSONL metadata; preserve source/receive time distinction."""
import collections
import json
import math
import os

ROOT=os.path.dirname(os.path.abspath(__file__))
BEGIN=1794555787.6311753
END=1794555819.9070578

def stats(values):
    a=sorted(v for v in values if math.isfinite(v))
    if not a:return dict(n=0)
    return dict(n=len(a),min=a[0],median=a[(len(a)-1)//2],p95=a[int((len(a)-1)*.95)],max=a[-1])

topics=collections.defaultdict(list)
host=[]
with open(os.path.join(ROOT,'latency_probe.jsonl')) as f:
    for line in f:
        row=json.loads(line)
        if row.get('event')=='host':host.append(row)
        elif row.get('topic'):topics[row['topic']].append(row)

def profile(begin,end):
    answer=dict(begin=begin,end=end,topics={})
    for topic,rr in topics.items():
        if topic=='/mppi/collision_snapshot':continue
        bysource=[r for r in rr if begin<=r.get('stamp',-1)<end]
        byreceive=[r for r in rr if begin<=r['recv_ros']<end]
        source=sorted(set(r['stamp'] for r in bysource))
        receive=sorted(r['recv_ros'] for r in byreceive)
        answer['topics'][topic]=dict(source_count=len(bysource),receive_count=len(byreceive),
            source_interval_ms=stats([(b-a)*1000 for a,b in zip(source,source[1:])]),
            receive_interval_ms=stats([(b-a)*1000 for a,b in zip(receive,receive[1:])]),
            source_to_receive_ms=stats([(r['recv_ros']-r['stamp'])*1000 for r in bysource]),
            source_gaps_over150ms=[dict(start=a,end=b,duration_ms=(b-a)*1000) for a,b in zip(source,source[1:]) if b-a>.15])
    sr=[r for r in topics['/mppi/collision_snapshot'] if begin<=r['recv_ros']<end]
    all_new=[];last_fusion=None
    firstfull=[];seen_versions=set()
    for r in topics['/mppi/collision_snapshot']:
        h=r['header']
        if h[3]!=last_fusion:
            all_new.append(r);last_fusion=h[3]
        if h[20]==1 and h[14] not in seen_versions:
            seen_versions.add(h[14]);firstfull.append(r)
    new=[r for r in all_new if begin<=r['recv_ros']<end]
    full=[r for r in firstfull if begin<=r['recv_ros']<end]
    fusion=[r['header'][3] for r in new]
    answer['snapshot']=dict(packets=len(sr),header_valid_fraction=(sum(r['header'][4]==1 for r in sr)/len(sr) if sr else None),
        new_map_first_visible_age_ms=stats([(r['recv_ros']-r['header'][3])*1000 for r in new]),
        new_map_first_visible_export_after_fusion_ms=stats([(r['header'][17]-r['header'][3])*1000 for r in new]),
        new_map_first_visible_receive_after_export_ms=stats([(r['recv_ros']-r['header'][17])*1000 for r in new]),
        new_map_interval_ms=stats([(b-a)*1000 for a,b in zip(fusion,fusion[1:])]),
        new_full_export_after_fusion_ms=stats([(r['header'][17]-r['header'][3])*1000 for r in full]),
        new_full_receive_after_export_ms=stats([(r['recv_ros']-r['header'][17])*1000 for r in full]),
        packet_map_age_ms=stats([(r['recv_ros']-r['header'][3])*1000 for r in sr]),
        map_age_over350ms_count=sum(r['recv_ros']-r['header'][3]>.35 for r in sr),
        new_map_gaps_over350ms=[dict(start=a,end=b,duration_ms=(b-a)*1000) for a,b in zip(fusion,fusion[1:]) if b-a>.35])
    # All stages preserve the same source stamp. Compare only exact matches;
    # absent source stamps are not invented as delayed/received points.
    lookup={k:{r['stamp']:r for r in v} for k,v in topics.items() if k!='/mppi/collision_snapshot'}
    paired={}
    for a,b in [('/cloud_registered','/livox/lidar_no_ground'),('/livox/lidar_no_ground','/velodyne_points'),('/velodyne_points','/velodyne_points_mapping'),('/cloud_registered','/velodyne_points_mapping')]:
        common=[stamp for stamp in lookup[a] if begin<=stamp<end and stamp in lookup[b]]
        paired[a+' -> '+b]=dict(n_exact_source_matches=len(common),receive_delta_ms=stats([(lookup[b][s]['recv_ros']-lookup[a][s]['recv_ros'])*1000 for s in common]))
    answer['same_source_pipeline_receive_deltas']=paired
    hh=[r for r in host if begin<=r['wall']<end]
    if len(hh)>1:
        a,b=hh[0],hh[-1];dt=b['mono']-a['mono'];hz=os.sysconf('SC_CLK_TCK')
        cpu_a=[int(x) for x in a['cpu'].split()[1:9]];cpu_b=[int(x) for x in b['cpu'].split()[1:9]]
        delta=[y-x for x,y in zip(cpu_a,cpu_b)];total=sum(delta)
        answer['host_sample_interval']=dict(start=a['wall'],end=b['wall'],duration_s=dt,
            busy_percent=(100*(1-(delta[3]+delta[4])/total) if total else None),
            process_one_core_percent={a['processes'][pid]['name'][0]:100*(b['processes'][pid]['ticks']-value['ticks'])/hz/dt for pid,value in a['processes'].items() if pid in b['processes']},
            swap_pages_in_delta=b['vmstat']['pswpin']-a['vmstat']['pswpin'],
            swap_pages_out_delta=b['vmstat']['pswpout']-a['vmstat']['pswpout'],
            major_faults_delta=b['vmstat']['pgmajfault']-a['vmstat']['pgmajfault'])
    return answer

report=dict(caveats=['Header stamps define cloud source windows; callback receive times define snapshot visibility windows.',
                      'h3 is occupancy commit time, h17 ESDF copy acquisition time, not raw sensor capture time.',
                      'CPU/process percentages use host samples inside the window, which are slightly shorter than its endpoints.',
                      'Exact source matching compares different subscriber arrivals and includes scheduling; negative tiny deltas are possible.'],
            navigation=profile(BEGIN,END),
            before_goal=profile(1794555735.3157542,BEGIN),
            after_finish=profile(END,1794555824.1816742))
with open(os.path.join(ROOT,'latency_probe_navigation_split.json'),'w') as f:json.dump(report,f,indent=2)
print(json.dumps(report['navigation'],indent=2))
