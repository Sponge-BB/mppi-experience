"""Summarize recorded stationary A/B samples; does not connect to ROS."""
from collections import Counter
import json
from pathlib import Path
import re

p=Path(__file__).parent
comparison={}
def stats(values):
    a=sorted(values)
    def percentile(q):
        x=q*(len(a)-1); lo=int(x); hi=min(lo+1,len(a)-1)
        return a[lo]+(a[hi]-a[lo])*(x-lo)
    return dict(zip(['min','p50','p95','max'],[percentile(q) for q in [0,.5,.95,1]]))
for name in ['local','full','local_repeat']:
    f=p/(name+'.json')
    if not f.exists():
        continue
    d=json.loads(f.read_text()); s=d['summary']; r=d['records']
    # Callbacks can arrive during subscriber registration, before the timed
    # sleep begins. Compare equal wall-clock windows, not the sleep duration.
    end=r['/mppi/collision_snapshot'][-1]['received']; begin=end-30.0
    r={t:[x for x in rr if begin<=x['received']<=end] for t,rr in r.items()}
    snap=r['/mppi/collision_snapshot']
    new=[b for a,b in zip(snap,snap[1:]) if b['header'][29]!=a['header'][29]]
    windows=[]; cells=[]
    for log in d['logs']:
        if not begin<=log['stamp']<=end: continue
        msg=log['msg']
        m=re.search(r'window=([\d.]+)s .*?esdf=(\d+) compute_avg/max=([\d.]+)/([\d.]+)ms main_commit_avg/max=([\d.]+)/([\d.]+)ms',msg)
        if m: windows.append([float(v) for v in m.groups()])
        m=re.search(r'LocalESDF.*?compute=(\d+)/(\d+) voxels.*?full=(\w+)',msg)
        if m and m[3]=='no':cells.append([int(m[1]),int(m[2])])
    total=sum(w[1] for w in windows)
    c={
        'window_start_ros':begin, 'window_end_ros':end, 'duration_s':30.0,
        'received_span_s':snap[-1]['received']-snap[0]['received'], 'snapshots':len(snap),
        'fresh_snapshot_fraction':sum(-.05<=x['received']-x['header'][3]<=x['header'][15] for x in snap)/len(snap),
        'first_arrival_age_ms':stats([1000*(x['received']-x['header'][3]) for x in new]),
        'new_map_arrival_period_ms':stats([1000*(b['received']-a['received']) for a,b in zip(new,new[1:])]),
        'received_new_maps':len(new),
        'compute_weighted_mean_ms':sum(w[1]*w[2] for w in windows)/total if total else None,
        'commit_weighted_mean_ms':sum(w[1]*w[4] for w in windows)/total if total else None,
        'compute_max_ms':max((w[3] for w in windows),default=None),
        'pipeline_windows':len(windows),
        'local_computation_fraction_range':[min(a/b for a,b in cells),max(a/b for a,b in cells)] if cells else None,
        'mppi_status_counts':dict(Counter(json.loads(x['value'])['state'] for x in r['/mppi/status'])),
        'dynamic_valid_fraction':s['dynamic_valid_fraction'],
        'dynamic_points_max':max(x['points'] for x in r['/dynamic_points']),
        'enabled_feedback_samples':sum(bool(x['enabled']) for x in r['/piper_execution_state']),
        'enabled_motor_samples':sum(any(x['value']) for x in r['/piper_motor_enable_states']),
        'command_publishers':s['command_publishers'],
        'min_mem_available_mib':s['min_mem_available_mib'],
    }
    assert not c['enabled_feedback_samples'] and not c['enabled_motor_samples']
    assert all(not x for x in c['command_publishers'].values())
    comparison[name]=c
(p/'comparison.json').write_text(json.dumps(comparison,indent=2)+'\n')
print(json.dumps(comparison,indent=2))
