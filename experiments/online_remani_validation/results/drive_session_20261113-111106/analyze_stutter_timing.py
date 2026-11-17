#!/usr/bin/env python3
"""Offline small-topic analysis; never connects to ROS or sends commands."""
import os
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OMP_NUM_THREADS', '1')
import bisect
import collections
import datetime
import json
import math
import resource
import rosbag

OUT = os.path.dirname(os.path.abspath(__file__))
GOAL = 1794539649.31747
REPORT = 1794539680.0

def clock(t):
    return datetime.datetime.fromtimestamp(t).strftime('%H:%M:%S.%f')[:-3]

def dist(a):
    a = sorted(v for v in a if v is not None and math.isfinite(v))
    if not a:
        return {'n': 0}
    return dict(n=len(a), min=a[0], median=a[(len(a)-1)//2],
                p95=a[int((len(a)-1)*.95)], max=a[-1])

statuses, cmds, executions, odom, faults, logs = [], [], [], [], [], []
topics = ['/mppi/status', '/cmd_vel', '/piper_execution_state', '/odom', '/rosout_agg', '/tracking_fault']
with rosbag.Bag(os.path.join(OUT, 'capture.bag')) as bag:
    for topic, msg, stamp in bag.read_messages(topics=topics):
        t = stamp.to_sec()
        if t < GOAL - 1:
            continue
        if topic == '/mppi/status':
            d = json.loads(msg.data)
            cd = d.get('candidate_diagnostics', {})
            row = {k: d.get(k) for k in ['state', 'mode', 'hold_reason', 'base_v', 'base_w', 'cycle_ms', 'solve_ms', 'native_phase', 'native_time_scale', 'feasible']}
            row.update(t=t, clock=clock(t), snapshot=d.get('snapshot'),
                       planning_elapsed_ms=cd.get('planning_elapsed_ms'),
                       delivery_elapsed_ms=cd.get('delivery_elapsed_ms'),
                       plan_time=cd.get('plan_time'), snapshot_header=cd.get('snapshot_header'),
                       reference_status=cd.get('reference_status'),
                       reference_revision=cd.get('reference_revision'),
                       selected_family=d.get('selected_family', cd.get('selected_family')),
                       external_min_gap=d.get('external_min_gap', cd.get('external_min_gap')))
            row['moving'] = d.get('state') == 'command_delivered' and (abs(d.get('base_v', 0)) > .001 or abs(d.get('base_w', 0)) > .001)
            statuses.append(row)
        elif topic == '/cmd_vel':
            cmds.append(dict(t=t, clock=clock(t), v=msg.linear.x, w=msg.angular.z,
                             moving=abs(msg.linear.x) > .001 or abs(msg.angular.z) > .001))
        elif topic == '/piper_execution_state':
            executions.append(dict(t=t, enabled=msg.enabled, valid=msg.feedback_valid,
                                   latched=msg.stop_latched, active=msg.command_active,
                                   source=msg.header.stamp.to_sec(), until=msg.valid_until.to_sec()))
        elif topic == '/odom':
            odom.append(dict(t=t, x=msg.pose.pose.position.x, y=msg.pose.pose.position.y,
                             v=msg.twist.twist.linear.x, w=msg.twist.twist.angular.z))
        elif topic == '/tracking_fault':
            faults.append(dict(t=t, fault=msg.data))
        elif topic == '/rosout_agg':
            logs.append(dict(t=t, clock=clock(t), source=msg.header.stamp.to_sec(), node=msg.name, level=msg.level, text=msg.msg))

stamps = [r['t'] for r in statuses]
for c in cmds:
    # stop_base/publication precedes /mppi/status in the same callback. Cross
    # topic TCPROS arrival may reorder a little; retain explicit association lag.
    i = bisect.bisect_left(stamps, c['t'] - .005)
    candidate = statuses[i:i+3]
    paired = min(candidate, key=lambda s: abs(s['t']-c['t'])) if candidate else None
    if paired and abs(paired['t']-c['t']) <= .08:
        c['paired_state'] = paired['state']
        c['paired_mode'] = paired['mode']
        c['status_lag_ms'] = (paired['t']-c['t'])*1000
    else:
        c['paired_state'] = 'unpaired'

end = max(r['t'] for r in statuses)

def summarize(start, stop):
    ss = [r for r in statuses if start <= r['t'] <= stop]
    cc = [r for r in cmds if start <= r['t'] <= stop]
    ee = [r for r in executions if start <= r['t'] <= stop]
    oo = [r for r in odom if start <= r['t'] <= stop]
    count = collections.Counter(r['state'] for r in ss)
    mode = collections.Counter((r['mode'] or '<none>') for r in ss if r['state'] == 'command_delivered')
    zero, moving = 0., 0.
    transitions = []
    for a, b in zip(cc, cc[1:]):
        span = max(0., min(stop, b['t']) - max(start, a['t']))
        if a['moving']:
            moving += span
        else:
            zero += span
        if a['moving'] != b['moving']:
            transitions.append(dict(t=b['t'], clock=b['clock'], to_moving=b['moving'], state=b['paired_state'], mode=b.get('paired_mode'), status_lag_ms=b.get('status_lag_ms')))
    return dict(start=start, stop=stop, start_clock=clock(start), stop_clock=clock(stop),
                n_status=len(ss), states=dict(count), state_percent={k: round(v/max(1, len(ss))*100, 3) for k,v in count.items()},
                delivered_mode_counts=dict(mode), actual_moving_deliveries=sum(r['moving'] for r in ss),
                n_cmd=len(cc), zero_commands=sum(not r['moving'] for r in cc), moving_commands=sum(r['moving'] for r in cc),
                command_zero_duration_s=zero, command_nonzero_duration_s=moving,
                zero_command_reasons=dict(collections.Counter(r['paired_state'] for r in cc if not r['moving'])),
                transition_count=len(transitions), transitions=transitions,
                cycle_ms_by_state={s: dist([r['cycle_ms'] for r in ss if r['state'] == s]) for s in count},
                plan_ms_by_state={s: dist([r['planning_elapsed_ms'] for r in ss if r['state'] == s]) for s in count},
                snapshot_age_ms_by_state={s: dist([(r['snapshot'] or {}).get('map_age_ms') for r in ss if r['state'] == s]) for s in count},
                execution_state_counts={str(k):v for k,v in collections.Counter((r['enabled'], r['valid'], r['latched']) for r in ee).items()},
                odom_abs_linear_mps=dist([abs(r['v']) for r in oo]),
                odom_start_end=([oo[0], oo[-1]] if oo else []),
                tracking_fault_true=sum(r['fault'] for r in faults if start <= r['t'] <= stop))

after = [r for r in cmds if r['t'] >= GOAL]
runs=[]
for c in after:
    if not runs or runs[-1]['moving'] != c['moving']:
        runs.append(dict(start=c['t'], start_clock=c['clock'], moving=c['moving'], commands=[]))
    runs[-1]['commands'].append(c)
for i, r in enumerate(runs):
    r['end'] = runs[i+1]['start'] if i+1<len(runs) else end
    r['duration_s'] = r['end']-r['start']
    r['reason_counts'] = dict(collections.Counter(c['paired_state'] for c in r['commands']))
    r['mode_counts'] = dict(collections.Counter(c.get('paired_mode') or '<none>' for c in r['commands']))
    r['v_abs_max'] = max(abs(c['v']) for c in r['commands'])
    r['w_abs_max'] = max(abs(c['w']) for c in r['commands'])
    r['n_commands'] = len(r.pop('commands'))

report = dict(goal_t=GOAL, goal_clock=clock(GOAL), bag_end=end,
              caveats=['Topic association uses recorder arrival time with +/-80ms nearest status, not command cycle identifier.',
                       'Snapshot h[3] is map occupancy commit time; age at status receipt is not pure source-cloud sensor age.',
                       'Zero-duration uses zero-order hold between received cmd_vel messages; serial/controller receipt was not recorded here.'],
              goal_to_user_report=summarize(GOAL, REPORT), all_after_goal=summarize(GOAL, end),
              command_runs=runs, driver_fault_logs=[r for r in logs if any(k in r['text'].lower() for k in ['piper', 'joint feedback', 'motor feedback'])],
              peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024.)
with open(os.path.join(OUT, 'stutter_timing_analysis.json'), 'w') as f:
    json.dump(report, f, indent=2)
with open(os.path.join(OUT, 'stutter_small_topics.jsonl'), 'w') as f:
    for r in statuses:
        f.write(json.dumps(r)+'\n')
with open(os.path.join(OUT, 'stutter_command_runs.json'), 'w') as f:
    json.dump(runs, f, indent=2)
with open(os.path.join(OUT, 'stutter_runtime_logs.jsonl'), 'w') as f:
    for r in logs:
        f.write(json.dumps(r)+'\n')
print(json.dumps({k: v for k, v in report.items() if k not in ['command_runs', 'driver_fault_logs']}, indent=2))
