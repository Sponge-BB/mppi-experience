#!/usr/bin/python3
"""Read CLOSED bags only, stream small evidence to disk, cap own RSS at 200 MiB."""
import argparse
import collections
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import resource
import time

import rosbag


def stamp(value):
    return value.to_sec() if hasattr(value, 'to_sec') else None


def local(t):
    return datetime.datetime.fromtimestamp(t).isoformat(timespec='microseconds')


def simple(value, depth=0):
    if depth > 5:
        return '<depth_limit>'
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if hasattr(value, 'to_sec'):
        return value.to_sec()
    if isinstance(value, (list, tuple)):
        return [simple(x, depth+1) for x in value[:96]]
    if hasattr(value, '__slots__'):
        return {k: simple(getattr(value, k), depth+1) for k in value.__slots__}
    return str(value)[:2000]


def qyaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class Samples:
    """At most 4096 deterministic reservoir samples per series, exact count/min/max."""
    def __init__(self):
        self.n = 0
        self.lo = None
        self.hi = None
        self.total = 0.
        self.values = []

    def add(self, value):
        if not isinstance(value, (int, float)) or not math.isfinite(value):
            return
        x = float(value)
        self.n += 1
        self.lo = x if self.lo is None else min(x, self.lo)
        self.hi = x if self.hi is None else max(x, self.hi)
        self.total += x
        if len(self.values) < 4096:
            self.values.append(x)
        else:
            # Deterministic pseudo-random reservoir, reproducible without RNG state.
            index = ((self.n*1103515245+12345) & 0x7fffffff) % self.n
            if index < 4096:
                self.values[index] = x

    def summary(self):
        a = sorted(self.values)
        return dict(count=self.n, min=self.lo, max=self.hi,
                    mean=self.total/self.n if self.n else None,
                    p50=a[int((len(a)-1)*.5)] if a else None,
                    p95=a[int((len(a)-1)*.95)] if a else None,
                    p99=a[int((len(a)-1)*.99)] if a else None,
                    quantile_samples=len(a), exact_quantiles=self.n <= 4096)


def trajectory_record(message):
    result = dict(action=message.action, trajectory_id=message.trajectory_id,
                  singul=message.singul, source_stamp=stamp(message.header.stamp))
    pieces = message.trajectory
    result['piece_count'] = len(pieces)
    result['duration_s'] = sum(stamp(p.duration) if hasattr(p.duration, 'to_sec')
                               else float(p.duration) for p in pieces)
    if pieces:
        def point(p, at_end):
            dim, order = p.num_dim, p.num_order
            duration = stamp(p.duration) if hasattr(p.duration, 'to_sec') else float(p.duration)
            out = []
            for axis in range(dim):
                v = 0.
                for k in range(order+1):
                    v = v*(duration if at_end else 0.)+p.data[k*dim+axis]
                out.append(v)
            return out
        result['begin'] = point(pieces[0], False)
        result['end'] = point(pieces[-1], True)
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('bag')
    ap.add_argument('--output', required=True)
    args = ap.parse_args()
    path = Path(args.bag).resolve()
    if path.name.endswith('.active') or path.suffix != '.bag' or not path.is_file():
        raise SystemExit('Only an existing CLOSED .bag file is accepted; no repair/reindex.')
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    streams = {name: (out/(name+'.jsonl')).open('w') for name in
               ('status', 'events', 'rosout', 'feedback', 'commands', 'dynamic')}
    def write(stream, row):
        streams[stream].write(json.dumps(row, ensure_ascii=False, separators=(',', ':'))+'\n')
    counts, states, modes, navstates = (collections.Counter() for _ in range(4))
    logs = collections.Counter()
    metrics = collections.defaultdict(Samples)
    goals, references, finishes, refreshes = [], [], [], []
    transitions = []
    previous_state = previous_mode = None
    first = last = first_goal = finish = None
    total = 0
    last_xy = None
    nav_distance = 0.
    latest = {}
    cmd_last_zero = None
    cmd_transitions = []
    status_contexts = collections.Counter()
    flags = collections.Counter()
    bag_info = dict(path=str(path), bytes=path.stat().st_size, mtime_ns=path.stat().st_mtime_ns)
    with rosbag.Bag(str(path), 'r', allow_unindexed=False) as bag:
        topic_types = {k: dict(type=v.msg_type, count=v.message_count)
                       for k, v in bag.get_type_and_topic_info().topics.items()}
        for topic, message, bt in bag.read_messages():
            total += 1
            if total % 1024 == 0 and resource.getrusage(resource.RUSAGE_SELF).ru_maxrss > 200*1024:
                raise MemoryError('Analysis exceeded 200 MiB; stopped own process without changing bag.')
            t = bt.to_sec()
            first = t if first is None else first
            last = t
            counts[topic] += 1
            nav = first_goal is not None and finish is None
            phase = 'navigation' if nav else ('after_finish' if finish is not None else 'before_goal')
            row = dict(t=t, local=local(t), topic=topic, phase=phase)
            if topic == '/move_base_simple/goal':
                row.update(source_stamp=stamp(message.header.stamp), frame=message.header.frame_id,
                           x=message.pose.position.x, y=message.pose.position.y, yaw=qyaw(message.pose.orientation))
                first_goal = t if first_goal is None else first_goal
                finish = None
                goals.append(row)
                write('events', row)
            elif topic == '/planning/trajectory':
                row.update(trajectory_record(message))
                references.append(row)
                write('events', row)
            elif topic in ('/planning/start', '/planning/finish', '/planning/mppi_refresh_request'):
                row['value'] = simple(message)
                write('events', row)
                if topic.endswith('finish'):
                    finishes.append(row)
                    if getattr(message, 'data', False):
                        finish = t
                if topic.endswith('mppi_refresh_request'):
                    refreshes.append(row)
            elif topic == '/mppi/status':
                try:
                    d = json.loads(message.data)
                except (ValueError, AttributeError):
                    flags['status_parse_error'] += 1
                    continue
                state = d.get('state', 'unknown')
                mode = str((d.get('reference_source'), d.get('mode')))
                states[state] += 1
                modes[mode] += 1
                status_contexts[phase+':'+state] += 1
                if nav:
                    navstates[state] += 1
                # Status is already scalar metadata, never includes a map array.
                row['status'] = d
                write('status', row)
                key = (state, mode)
                if key != (previous_state, previous_mode):
                    event = {k: row[k] for k in ('t', 'local', 'phase')}
                    event.update(event='mppi_transition', state=state, mode=mode,
                                 v=d.get('base_v'), w=d.get('base_w'))
                    write('events', event)
                    if nav:
                        transitions.append(event)
                    previous_state, previous_mode = key
                for k in ('cycle_ms', 'solve_ms', 'base_v', 'base_w', 'native_time_scale',
                          'external_min_gap', 'nominal_external_min_gap', 'native_phase'):
                    metrics[phase+'.'+k].add(d.get(k))
                diag = d.get('candidate_diagnostics', {})
                for stage in ('capture_inputs', 'checked_inputs'):
                    evidence = diag.get(stage, {})
                    for k in ('base_age_ms', 'joint_age_ms', 'map_age_ms', 'dynamic_age_ms'):
                        metrics[phase+'.'+stage+'.'+k].add(evidence.get(k))
                    for k in ('base_fresh','joint_fresh','map_age_ok','dynamic_age_ok','snapshot_valid','dynamic_valid'):
                        if k in evidence:
                            flags[phase+'.'+stage+'.'+k+'='+str(evidence[k])] += 1
                for k in ('delivery_lock_wait_ms','delivery_lock_hold_ms'):
                    metrics[phase+'.'+k].add(diag.get(k))
                latest['status'] = dict(t=t, state=state, v=d.get('base_v'), w=d.get('base_w'))
            elif topic == '/rosout_agg':
                row.update(source_stamp=stamp(message.header.stamp), level=message.level,
                           node=message.name, message=message.msg)
                write('rosout', row)
                if message.level >= 4:
                    # Bounded grouping, normalize numeric text rather than storing unique messages.
                    label = message.msg.split(':', 1)[0][:120]
                    if len(logs) < 300 or label in logs:
                        logs[phase+':'+message.name+':'+label] += 1
            elif topic in ('/cmd_vel', '/mm_controller_node/car_cmd'):
                row['value'] = simple(message)
                twist = getattr(message, 'twist', message)
                if hasattr(twist, 'linear') and hasattr(twist, 'angular'):
                    row.update(v=twist.linear.x, w=twist.angular.z)
                    metrics[phase+topic+'.v'].add(row['v'])
                    metrics[phase+topic+'.w'].add(row['w'])
                    if topic == '/cmd_vel':
                        zero = abs(row['v']) < .005 and abs(row['w']) < .02
                        if zero != cmd_last_zero:
                            ev = dict(t=t, local=local(t), phase=phase, event='cmd_zero_transition',
                                      zero=zero, v=row['v'], w=row['w'], latest_status=latest.get('status'))
                            write('events', ev)
                            if nav:
                                cmd_transitions.append(ev)
                            cmd_last_zero = zero
                write('commands', row)
            elif topic == '/odom':
                p = message.pose.pose.position
                row.update(source_stamp=stamp(message.header.stamp), x=p.x, y=p.y,
                           yaw=qyaw(message.pose.pose.orientation),
                           v=message.twist.twist.linear.x, w=message.twist.twist.angular.z)
                row['receipt_age_ms'] = (t-row['source_stamp'])*1000
                if nav and last_xy is not None:
                    nav_distance += math.hypot(p.x-last_xy[0], p.y-last_xy[1])
                last_xy = (p.x, p.y)
                metrics[phase+'.odom_receipt_age_ms'].add(row['receipt_age_ms'])
                metrics[phase+'.odom_v'].add(row['v'])
                metrics[phase+'.odom_w'].add(row['w'])
                latest['odom'] = row
                write('feedback', row)
            elif topic in ('/joint_states_single','/mm/joint_states','/joint_ctrl_single',
                            '/mm_controller_node/joint_cmd','/piper_execution_state','/tracking_fault',
                            '/planning/safety_speed_limit','/mm_controller_node/trajectory_progress'):
                row['value'] = simple(message)
                if hasattr(message, 'header'):
                    row['source_stamp'] = stamp(message.header.stamp)
                    metrics[phase+topic+'.receipt_age_ms'].add((t-row['source_stamp'])*1000)
                if topic == '/tracking_fault':
                    flags[phase+'.tracking_fault='+str(message.data)] += 1
                write('feedback', row)
            elif topic == '/dynamic_points':
                row.update(source_stamp=stamp(message.header.stamp), points=message.width*message.height,
                           frame=message.header.frame_id)
                metrics[phase+'.dynamic_points_count'].add(row['points'])
                metrics[phase+'.dynamic_points_receipt_age_ms'].add((t-row['source_stamp'])*1000)
                write('dynamic', row)
            elif 'dynamic_' in topic:
                row['value'] = simple(message)
                if hasattr(message, 'data'):
                    metrics[phase+topic].add(message.data)
                write('dynamic', row)
    for f in streams.values():
        f.close()
    summary = dict(bag=bag_info, topic_types=topic_types, messages=total, counts=counts,
                   first=first, last=last, first_local=local(first), last_local=local(last),
                   goals=goals, references=references, finishes=finishes, refreshes=refreshes,
                   states=states, modes=modes, navigation_states=navstates,
                   status_by_phase=status_contexts, flags=flags,
                   navigation_state_transitions=transitions, navigation_cmd_zero_transitions=cmd_transitions,
                   navigation_odom_path_m=nav_distance,
                   metrics={k:v.summary() for k,v in sorted(metrics.items())},
                   warning_groups=logs, elapsed_s=time.monotonic()-started,
                   max_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024.,
                   script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                   notes=['Only closed bag read; no repair/reindex; no ROS publications or services.',
                          'Bag receipt times and source stamps are distinct; logs retain both.',
                          'Metric reservoirs bounded at 4096; extrema/count/mean exact.',
                          'Navigation phase runs from goal receipt until finish=true; bag end is not task termination.',
                          'Zero command threshold abs(v)<0.005m/s AND abs(w)<0.02rad/s.',
                          'No full cloud or occupancy snapshot available to establish physical cause of collision cells.'])
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:summary[k] for k in ('messages','first_local','last_local','max_rss_mib','elapsed_s',
                                          'navigation_states','navigation_odom_path_m')},ensure_ascii=False))


if __name__ == '__main__':
    main()
