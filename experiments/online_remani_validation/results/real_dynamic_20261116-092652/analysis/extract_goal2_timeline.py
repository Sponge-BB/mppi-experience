#!/usr/bin/python3
"""Small, read-only timeline from a normally closed core.bag; never repairs bags."""
import collections
import datetime
import json
from pathlib import Path
import re
import resource
import time

import rosbag


CASE = Path(__file__).resolve().parents[1]
BAG = CASE/'core.bag'
OUT = CASE/'analysis/goal2_timeline.json'


def local(t):
    return datetime.datetime.fromtimestamp(t).isoformat(timespec='microseconds')


def entry(topic, message, bt):
    t = bt.to_sec()
    source = message.header.stamp.to_sec() if hasattr(message, 'header') else None
    return dict(topic=topic, receipt_t=t, receipt_local=local(t), source_t=source,
                source_local=local(source) if source else None,
                receipt_minus_source_ms=(t-source)*1000 if source else None)


def small_status(d):
    result = {k:d[k] for k in ('state','reference_source','mode','base_v','base_w',
                               'cycle_ms','solve_ms','native_phase','native_phase_source',
                               'native_time_scale','feasible','external_min_gap',
                               'nominal_external_min_gap','selected_family','hold_reason',
                               'base_safety_speed_limit','arm_motion') if k in d}
    diag = d.get('candidate_diagnostics', {})
    for stage in ('capture_inputs','checked_inputs'):
        v = diag.get(stage, {})
        result[stage] = {k:v[k] for k in ('checked_at','base_stamp','joint_stamp',
                     'base_age_ms','joint_age_ms','map_age_ms','dynamic_age_ms',
                     'base_fresh','joint_fresh','map_age_ok','dynamic_age_ok',
                     'snapshot_valid','dynamic_valid') if k in v}
        h = v.get('snapshot_header')
        if h and len(h) == 32:
            result[stage]['snapshot'] = {str(i):h[i] for i in (2,3,4,14,15,17,18,19,24,28,29)}
    return result


def main():
    if BAG.suffix != '.bag' or not BAG.is_file():
        raise SystemExit('Expected final closed core.bag. No active reads or reindex.')
    began = time.monotonic()
    topics = ['/move_base_simple/goal','/planning/trajectory','/planning/finish',
              '/planning/safety_speed_limit','/planning/mppi_refresh_request',
              '/rosout_agg','/mppi/status','/dynamic_points','/cmd_vel','/odom',
              '/mm_controller_node/trajectory_progress','/tracking_fault',
              '/dynamic_obstacle_tracker/perception_healthy',
              '/dynamic_obstacle_tracker/motion_qualified_track_count']
    goals, refs, finishes, limits, refreshes, logs, transitions = ([] for _ in range(7))
    compact_status = []
    counts, states, modes = (collections.Counter() for _ in range(3))
    source_delays = []
    dynamics = []
    cmd_intervals = []
    current_cmd = None
    goal2 = None
    last_key = None
    last_dyn_positive = None
    status_ring, odom_ring, progress_ring = (collections.deque(maxlen=8) for _ in range(3))
    abort_contexts = []
    last = None
    last_odom = None
    count = 0
    with rosbag.Bag(str(BAG), 'r', allow_unindexed=False) as bag:
        bag_topics = {k:dict(type=v.msg_type,count=v.message_count)
                      for k,v in bag.get_type_and_topic_info().topics.items()}
        for topic, m, bt in bag.read_messages(topics=topics):
            count += 1
            if count % 512 == 0 and resource.getrusage(resource.RUSAGE_SELF).ru_maxrss > 180*1024:
                raise MemoryError('Own RSS exceeded 180 MiB, abort without modifying bag.')
            counts[topic] += 1
            t = bt.to_sec()
            last = t
            r = entry(topic,m,bt)
            if topic == '/move_base_simple/goal':
                r.update(x=m.pose.position.x,y=m.pose.position.y,frame=m.header.frame_id,
                         orientation=dict(x=m.pose.orientation.x,y=m.pose.orientation.y,
                                          z=m.pose.orientation.z,w=m.pose.orientation.w))
                goals.append(r)
                if len(goals)==2:
                    goal2=t
            elif topic == '/planning/trajectory':
                r.update(action=m.action,trajectory_id=m.trajectory_id,gear=m.singul,
                         duration_s=sum(float(x.duration) for x in m.trajectory),
                         pieces=len(m.trajectory))
                refs.append(r)
                if goal2 is not None and m.action==2:
                    abort_contexts.append(dict(abort=r,status_before=list(status_ring),
                                               odom_before=list(odom_ring),
                                               progress_before=list(progress_ring)))
            elif topic == '/planning/finish':
                r['data']=bool(m.data)
                finishes.append(r)
            elif topic == '/planning/safety_speed_limit':
                r['value']=float(m.data)
                limits.append(r)
            elif topic == '/planning/mppi_refresh_request':
                names=('version','checked_at','goal','reference_start','odom','joint',
                       'map_stamp','dynamic_stamp','dynamic_required','episode')
                r['packet']=dict(zip(names,list(m.data)))
                r['source_t']=r['packet'].get('checked_at')
                if r['source_t']:
                    r['source_local']=local(r['source_t'])
                    r['receipt_minus_source_ms']=(t-r['source_t'])*1000
                refreshes.append(r)
            elif topic == '/rosout_agg' and goal2 is not None:
                if any(x in m.msg for x in ('[FSM]','[LiveCollisionStop]','Depth Lost',
                          '[GridMap][LiDAR]','Piper command stop','Aborting the trajectory',
                          'MPPI command rejected','DynamicPointFlow][active]')):
                    r.update(node=m.name,level=m.level,message=m.msg)
                    source_delays.append(r['receipt_minus_source_ms'])
                    logs.append(r)
            elif topic == '/mppi/status':
                try:
                    d=json.loads(m.data)
                except ValueError:
                    continue
                r.update(small_status(d))
                status_ring.append(r)
                if goal2 is not None:
                    states[d.get('state')]+=1
                    modes[str((d.get('reference_source'),d.get('mode')))]+=1
                    compact_status.append(r)
                    key=(d.get('state'),d.get('reference_source'),d.get('mode'))
                    if key!=last_key:
                        transitions.append({k:v for k,v in r.items() if k not in ('capture_inputs','checked_inputs')})
                        last_key=key
            elif topic == '/dynamic_points' and goal2 is not None:
                n=m.width*m.height
                positive=n>0
                if positive!=last_dyn_positive:
                    r.update(points=n,frame=m.header.frame_id)
                    dynamics.append(r)
                    last_dyn_positive=positive
            elif topic == '/cmd_vel' and goal2 is not None:
                v,w=m.linear.x,m.angular.z
                zero=abs(v)<.005 and abs(w)<.02
                if current_cmd is None or zero!=current_cmd['zero']:
                    if current_cmd is not None:
                        current_cmd['end_receipt_t']=t
                        current_cmd['duration_s']=t-current_cmd['receipt_t']
                        cmd_intervals.append(current_cmd)
                    current_cmd=dict(receipt_t=t,receipt_local=local(t),v=v,w=w,zero=zero,
                                     stamp_note='Twist has no source stamp; receipt time only.')
            elif topic == '/odom':
                r.update(frame=m.header.frame_id,child=m.child_frame_id,
                         x=m.pose.pose.position.x,y=m.pose.pose.position.y,
                         v=m.twist.twist.linear.x,w=m.twist.twist.angular.z)
                odom_ring.append(r)
                last_odom=r
            elif topic == '/mm_controller_node/trajectory_progress':
                r.update(reference_start=m.vector.x,phase=m.vector.y,time_scale=m.vector.z)
                progress_ring.append(r)
            elif topic == '/tracking_fault' and goal2 is not None and m.data:
                r['tracking_fault']=True
                logs.append(r)
    if current_cmd is not None:
        current_cmd.update(end_receipt_t=last,duration_s=last-current_cmd['receipt_t'],
                           interval_end_is_bag_boundary=True)
        cmd_intervals.append(current_cmd)
    result=dict(closed_bag=str(BAG),bag_size=BAG.stat().st_size,bag_topics=bag_topics,
                goals=goals,goal2_receipt_t=goal2,references=refs,finishes=finishes,
                safety_speed_limits=limits,mppi_refreshes=refreshes,
                goal2_states=states,goal2_modes=modes,goal2_status_transitions=transitions,
                goal2_status_compact=compact_status,goal2_dynamic_presence_changes=dynamics,
                goal2_logs_in_receipt_order=logs,
                goal2_logs_sorted_by_source=sorted(logs,key=lambda x:x.get('source_t') or x['receipt_t']),
                goal2_abort_contexts=abort_contexts,goal2_command_intervals=cmd_intervals,
                last_odom=last_odom,selected_topic_counts=counts,
                relevant_rosout_receipt_delay_ms=dict(count=len(source_delays),
                       max=max(source_delays) if source_delays else None),
                max_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
                elapsed_s=time.monotonic()-began,
                notes=['Only final core.bag was read, no active/reindex/ROS publications or code changes.',
                       'ROS log header time is event source time; bag time is recorder receipt. Logs have substantial delayed delivery.',
                       'Trajectory header is execution epoch, not publication time. ABORT/Bool/Float64/Twist have no source stamp.',
                       'No finish for goal2 must not be equated with permanent navigation failure; bag boundary ends observation.',
                       'Progress vector z is execution time scale, not gear.'])
    OUT.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(saved=str(OUT),bytes=OUT.stat().st_size,goals=goals,
                          goal2_states=states,goal2_modes=modes,
                          goal2_dynamic_presence_changes=dynamics,
                          max_rss_mib=result['max_rss_mib'],elapsed_s=result['elapsed_s']),ensure_ascii=False))


if __name__=='__main__':
    main()
