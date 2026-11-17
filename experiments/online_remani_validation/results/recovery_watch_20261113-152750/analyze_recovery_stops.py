#!/usr/bin/env python3
"""Read small recorded topics only; no ROS node, network, or control."""
import os
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'
import collections
import datetime
import json
import math
import re
import resource
import rosbag

OUT = os.path.dirname(os.path.abspath(__file__))

def clock(t):
    return datetime.datetime.fromtimestamp(t).strftime('%H:%M:%S.%f')[:-3]

def stats(xs):
    xs = sorted(x for x in xs if x is not None and math.isfinite(x))
    if not xs:
        return {'n': 0}
    return dict(n=len(xs), min=xs[0], median=xs[(len(xs)-1)//2],
                p95=xs[int((len(xs)-1)*.95)], max=xs[-1])

status, cmds, odom, logs, goals, finishes, trajectories, limits, states = [], [], [], [], [], [], [], [], []
topics=['/mppi/status', '/cmd_vel', '/odom', '/rosout_agg', '/move_base_simple/goal',
        '/planning/finish', '/planning/trajectory', '/planning/safety_speed_limit', '/piper_execution_state']
with rosbag.Bag(os.path.join(OUT, 'capture.bag')) as bag:
    start, end = bag.get_start_time(), bag.get_end_time()
    for topic, m, t in bag.read_messages(topics=topics):
        rt = t.to_sec()
        if topic == '/mppi/status':
            d = json.loads(m.data)
            cd = d.get('candidate_diagnostics', {})
            status.append(dict(receive=rt, clock=clock(rt), state=d.get('state'), mode=d.get('mode'),
                               hold_reason=d.get('hold_reason'), v=d.get('base_v'), w=d.get('base_w'),
                               cycle_ms=d.get('cycle_ms'), solve_ms=d.get('solve_ms'),
                               snapshot=d.get('snapshot'), plan_source=cd.get('plan_time'),
                               phase=d.get('native_phase'), time_scale=d.get('native_time_scale'),
                               source_kind='JSON status has no header; receive time retained'))
        elif topic == '/cmd_vel':
            cmds.append(dict(receive=rt, clock=clock(rt), v=m.linear.x, w=m.angular.z,
                             moving=abs(m.linear.x)>.001 or abs(m.angular.z)>.001))
        elif topic == '/odom':
            odom.append(dict(receive=rt, source=m.header.stamp.to_sec(), x=m.pose.pose.position.x,
                             y=m.pose.pose.position.y, v=m.twist.twist.linear.x, w=m.twist.twist.angular.z))
        elif topic == '/rosout_agg':
            logs.append(dict(receive=rt, source=m.header.stamp.to_sec(), clock=clock(m.header.stamp.to_sec()),
                             node=m.name, text=m.msg, level=m.level))
        elif topic == '/move_base_simple/goal':
            goals.append(dict(receive=rt, source=m.header.stamp.to_sec(), clock=clock(m.header.stamp.to_sec()),
                              x=m.pose.position.x,y=m.pose.position.y))
        elif topic == '/planning/finish':
            finishes.append(dict(receive=rt, clock=clock(rt), value=m.data))
        elif topic == '/planning/safety_speed_limit':
            limits.append(dict(receive=rt, clock=clock(rt), value=m.data))
        elif topic == '/planning/trajectory':
            trajectories.append(dict(receive=rt, source=m.header.stamp.to_sec(), clock=clock(m.header.stamp.to_sec()),
                                     action=m.action, trajectory_id=m.trajectory_id))
        else:
            states.append(dict(receive=rt, source=m.header.stamp.to_sec(), enabled=m.enabled,
                               valid=m.feedback_valid, latched=m.stop_latched))

logs.sort(key=lambda r:r['source'])
stop_logs=[x for x in logs if x['text']=='Depth Lost! EMERGENCY_STOP']
recovery_logs=[x for x in logs if 'validated a new recovery trajectory' in x['text']]
timeouts=[x for x in logs if '[GridMap][LiDAR] Live input timed out' in x['text']]
suppressed=[x for x in logs if x['node']=='/robot_self_filter' and 'suppressed cloud' in x['text']]

def summarize(a,b):
    ss=[x for x in status if a<=x['receive']<b]
    cc=[x for x in cmds if a<=x['receive']<b]
    oo=[x for x in odom if a<=x['source']<b]
    ee=[x for x in states if a<=x['source']<b]
    counts=collections.Counter(x['state'] for x in ss)
    durations=collections.defaultdict(float)
    transitions=[]
    for x,y in zip(cc,cc[1:]):
        durations['moving' if x['moving'] else 'zero']+=y['receive']-x['receive']
        if x['moving']!=y['moving']:
            transitions.append(dict(receive=y['receive'],clock=y['clock'],moving=y['moving']))
    snap=[x for x in ss if x['snapshot']]
    return dict(start=a,end=b,start_clock=clock(a),end_clock=clock(b),n_states=len(ss),
                state_counts=dict(counts),delivered_mode_counts=dict(collections.Counter(x['mode'] for x in ss if x['state']=='command_delivered')),
                delivered_actual_nonzero=sum(x['state']=='command_delivered' and (abs(x['v'] or 0)>.001 or abs(x['w'] or 0)>.001) for x in ss),
                cycle_by_state={k:stats([x['cycle_ms'] for x in ss if x['state']==k]) for k in counts},
                map_age_by_state={k:stats([x['snapshot']['map_age_ms'] for x in snap if x['state']==k]) for k in counts},
                map_age_over350_by_state=dict(collections.Counter(x['state'] for x in snap if x['snapshot']['map_age_ms']>350)),
                dynamic_age_over350_by_state=dict(collections.Counter(x['state'] for x in snap if x['snapshot']['dynamic_age_ms']>350)),
                cmd_counts=dict(collections.Counter('moving' if x['moving'] else 'zero' for x in cc)),cmd_hold_duration_s=dict(durations),
                cmd_transition_count=len(transitions),transitions=transitions,
                odom_abs_speed=stats([abs(x['v']) for x in oo]),odom_first_last=([oo[0],oo[-1]] if oo else []),
                enabled_valid_latched_counts={str(k):v for k,v in collections.Counter((x['enabled'],x['valid'],x['latched']) for x in ee).items()},
                lidar750_timeout_count=sum(a<=x['source']<b for x in timeouts),
                depth_lost_count=sum(a<=x['source']<b for x in stop_logs),
                successful_perception_recovery_count=sum(a<=x['source']<b for x in recovery_logs),
                self_filter_suppressed_count=sum(a<=x['source']<b for x in suppressed))

stop_windows=[]
for s in stop_logs:
    rr=next((r for r in recovery_logs if r['source']>s['source']),None)
    next_stop=next((r for r in stop_logs if r['source']>s['source']),None)
    if next_stop and rr and rr['source']>next_stop['source']:
        rr=None
    stop_windows.append(dict(stop_source=s['source'],stop_clock=s['clock'],
                             recovery_source=rr['source'] if rr else None,
                             recovery_clock=rr['clock'] if rr else None,
                             hold_until_new_recovery_trajectory_s=(rr['source']-s['source'] if rr else None)))

runs=[]
for c in cmds:
    if goals and c['receive']<goals[0]['source']:
        continue
    if not runs or runs[-1]['moving']!=c['moving']:
        runs.append(dict(start=c['receive'],clock=c['clock'],moving=c['moving'],count=0,max_v=0.,max_w=0.))
    r=runs[-1];r['count']+=1;r['max_v']=max(r['max_v'],abs(c['v']));r['max_w']=max(r['max_w'],abs(c['w']))
for i,r in enumerate(runs):
    r['end']=runs[i+1]['start'] if i+1<len(runs) else end
    r['duration_s']=r['end']-r['start']
    r['state_counts']=dict(collections.Counter(x['state'] for x in status if r['start']-.03<=x['receive']<r['end']+.03))
    r['lidar750_stops']=[x['clock'] for x in stop_logs if r['start']-.3<=x['source']<r['end']+.3]

pipeline=[]
pattern=re.compile(r'window=([\d.]+)s mode=(\w+) occ=(\d+) avg/max=([\d.]+)/([\d.]+)ms esdf=(\d+) compute_avg/max=([\d.]+)/([\d.]+)ms main_commit_avg/max=([\d.]+)/([\d.]+)ms snapshot_avg/max=([\d.]+)/([\d.]+)ms age=([\d.]+)ms')
for log in logs:
    match=pattern.search(log['text'])
    if not match:
        continue
    values=match.groups()
    row={k:float(v) for k,v in zip(['window_s','mode','occ_n','occ_avg_ms','occ_max_ms','esdf_n','esdf_compute_avg_ms','esdf_compute_max_ms','main_commit_avg_ms','main_commit_max_ms','snapshot_avg_ms','snapshot_max_ms','age_ms'],values) if k!='mode'}
    row.update(source=log['source'],clock=log['clock'],text=log['text'],mode=values[1])
    pipeline.append(row)
active=[]
for i,g in enumerate(goals):
    stop=goals[i+1]['source'] if i+1<len(goals) else end
    completed=next((f for f in finishes if f['value'] and g['source']<f['receive']<stop),None)
    if completed:
        stop=completed['receive']
    active.append(summarize(g['source'],stop))

summary=dict(start=start,end=end,goals=goals,finish_topic=finishes,trajectory_events=trajectories,
             source_time_caveat='Logs/odom/goals/trajectories/execution use header source time. cmd_vel, Float64 speed limits, Bool finish and String status lack header: recorder receipt time only, never mislabeled as source time.',
             all=summarize(start,end),per_goal=[summarize(g['source'],goals[i+1]['source'] if i+1<len(goals) else end) for i,g in enumerate(goals)],
             active_navigation_windows=active,
             post_finish=(summarize(finishes[-1]['receive'],end) if finishes else None),
             stop_recovery_windows=stop_windows,safety_limits=limits,
             all_lidar750_timeout_logs=timeouts, self_filter_suppressed_logs=suppressed,
             peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024.)
with open(os.path.join(OUT,'recovery_stutter_analysis.json'),'w') as f:json.dump(summary,f,indent=2)
with open(os.path.join(OUT,'recovery_zero_nonzero_runs.json'),'w') as f:json.dump(runs,f,indent=2)
with open(os.path.join(OUT,'recovery_small_status.jsonl'),'w') as f:
    for r in status:f.write(json.dumps(r)+'\n')
with open(os.path.join(OUT,'recovery_runtime_logs.jsonl'),'w') as f:
    for r in logs:f.write(json.dumps(r)+'\n')
with open(os.path.join(OUT,'recovery_pipeline_metrics.json'),'w') as f:json.dump(pipeline,f,indent=2)
with open(os.path.join(OUT,'recovery_map_feedback_warnings.json'),'w') as f:
    json.dump([r for r in logs if any(k in r['text'] for k in ['[GridMap][deadline]','[GridMap][AsyncESDF]','suppressed cloud','dropping','Dropped','discarded','stale cloud'])],f,indent=2)
print(json.dumps({k:v for k,v in summary.items() if k not in ['all','per_goal','all_lidar750_timeout_logs','self_filter_suppressed_logs']},indent=2))
