#!/usr/bin/python3
"""Stream the sealed small-message bag; distinguish exact gate evidence from post-reject logs."""
from collections import Counter
import json
import math
from pathlib import Path
import statistics
import rosbag

ROOT=Path(__file__).resolve().parent

def stats(values):
    values=sorted(float(x) for x in values if isinstance(x,(int,float)) and math.isfinite(x))
    if not values: return dict(n=0)
    def pct(p):
        pos=(len(values)-1)*p; i=int(pos); f=pos-i
        return values[i]*(1-f)+values[min(i+1,len(values)-1)]*f
    return dict(n=len(values),min=values[0],median=statistics.median(values),p95=pct(.95),max=values[-1])

def flags(gate):
    return [key for key in ('base_fresh','joint_fresh','map_age_ok','dynamic_age_ok',
                            'snapshot_valid','dynamic_valid') if gate.get(key) is False]

def main():
    status=json.loads((ROOT/'status.json').read_text())
    if status.get('phase')!='closed': raise SystemExit('Recorder still running; wait for sealed bag')
    rows=[]; goals=[]; finish=None; end=None
    with rosbag.Bag(str(ROOT/'latency.bag'),'r') as bag:
        end=bag.get_end_time()
        for topic,msg,t in bag.read_messages(topics=['/move_base_simple/goal','/planning/finish','/mppi/status']):
            stamp=t.to_sec()
            if topic=='/move_base_simple/goal':
                goals.append(dict(received=stamp,source=msg.header.stamp.to_sec(),
                                  xy=[msg.pose.position.x,msg.pose.position.y])); continue
            if not goals: continue
            if topic=='/planning/finish':
                if msg.data and finish is None: finish=stamp
                continue
            if finish is not None: continue
            data=json.loads(msg.data); diag=data.get('candidate_diagnostics',{})
            row=dict(received=stamp,state=data.get('state'),cycle_ms=data.get('cycle_ms'),
                     solve_ms=data.get('solve_ms'),mode=data.get('mode',diag.get('mode')),
                     capture=diag.get('capture_inputs',{}), checked=diag.get('checked_inputs',{}))
            for key in ('delivery_lock_wait_ms','delivery_lock_hold_ms','first_step_check_ms',
                        'planning_elapsed_ms','delivery_elapsed_ms','deadline_ms'):
                row[key]=diag.get(key)
            row['actual_false_flags']=flags(row['checked'])
            rows.append(row)
    all_rows=rows
    first_cancel=next((row['received'] for row in rows if str(row['state']).startswith('native_reference_cancelled')),None)
    last_delivered=max((row['received'] for row in rows if row['state']=='command_delivered'),default=0.)
    persistent_cancel=next((row['received'] for row in rows if row['received']>last_delivered
        and str(row['state']).startswith('native_reference_cancelled')),None)
    if persistent_cancel is not None:
        rows=[row for row in rows if row['received'] < persistent_cancel]
    counts=Counter(row['state'] for row in rows)
    rejected=[row for row in rows if row['state']!='command_delivered']
    delivered=[row for row in rows if row['state']=='command_delivered']
    by_reason={}
    for reason in counts:
        group=[row for row in rows if row['state']==reason]
        by_reason[reason]=dict(n=len(group),stages=dict(Counter(row['checked'].get('stage','absent') for row in group)),
            actual_false_flags=dict(Counter(flag for row in group for flag in row['actual_false_flags'])),
            false_flag_combinations=dict(Counter(','.join(row['actual_false_flags']) or 'none' for row in group)),
            gate_map_age_ms=stats(row['checked'].get('map_age_ms') for row in group),
            gate_dynamic_age_ms=stats(row['checked'].get('dynamic_age_ms') for row in group),
            lock_wait_ms=stats(row['delivery_lock_wait_ms'] for row in group),
            lock_hold_ms=stats(row['delivery_lock_hold_ms'] for row in group),
            first_step_check_ms=stats(row['first_step_check_ms'] for row in group),
            planning_elapsed_ms=stats(row['planning_elapsed_ms'] for row in group))
    callbacks={key:{} for key in ('base','joint','snapshot')}
    for row in rows:
        for gate in (row['capture'],row['checked']):
            for key, traces in callbacks.items():
                trace=gate.get(key+'_callback')
                if trace: traces[(trace.get('enter'),trace.get('adopted'))]=trace
    cb_summary={}
    for key,traces in callbacks.items():
        fields=('read_lock_wait_ms','decode_ms','write_lock_wait_ms') if key=='snapshot' else ('lock_wait_ms',)
        cb_summary[key]={field:stats(t.get(field) for t in traces.values()) for field in fields}
        cb_summary[key]['entry_to_adoption_ms']=stats((t['adopted']-t['enter'])*1000 for t in traces.values())
        cb_summary[key]['unique_observed_callbacks']=len(traces)
        cb_summary[key]['longest_examples']=sorted(traces.values(),key=lambda x:x['adopted']-x['enter'],reverse=True)[:8]
    crossings=[]
    for row in rejected:
        cap,gate=row['capture'],row['checked']
        if cap.get('map_age_ok') is True and gate.get('map_age_ok') is False:
            a,b=cap.get('snapshot_header'),gate.get('snapshot_header')
            crossings.append(dict(received=row['received'],state=row['state'],stage=gate.get('stage'),
                capture_age_ms=cap.get('map_age_ms'),gate_age_ms=gate.get('map_age_ms'),
                capture_wire=a[2] if a else None,gate_wire=b[2] if b else None,
                same_field=bool(a and b and a[14]==b[14]),same_wire=bool(a and b and a[2]==b[2]),
                lock_wait_ms=row['delivery_lock_wait_ms'],lock_hold_ms=row['delivery_lock_hold_ms'],
                first_step_check_ms=row['first_step_check_ms'],planning_elapsed_ms=row['planning_elapsed_ms']))
    analysis_end=persistent_cancel or finish or end
    summary=dict(goals=goals,window=dict(start=goals[0]['received'] if goals else None,end=analysis_end,
        first_cancel_received=first_cancel,persistent_cancel_received=persistent_cancel,finish_received=finish,bag_end=end,
        duration_s=analysis_end-goals[0]['received'] if goals else None),
        all_navigation_states=dict(Counter(row['state'] for row in all_rows)),
        pre_first_cancel_states=dict(Counter(row['state'] for row in all_rows if not first_cancel or row['received']<first_cancel)),
        post_first_cancel_states=dict(Counter(row['state'] for row in all_rows if first_cancel and row['received']>=first_cancel)),
        states=dict(counts),delivered=len(delivered),rejected=len(rejected),by_reason=by_reason,
        callback_timings=cb_summary,map_fresh_at_capture_expired_at_gate=dict(n=len(crossings),examples=crossings),
        delivery_lock_all=dict(wait_ms=stats(r['delivery_lock_wait_ms'] for r in rows),
            hold_ms=stats(r['delivery_lock_hold_ms'] for r in rows),first_step_ms=stats(r['first_step_check_ms'] for r in rows)),
        notes=['Uses gate checked_inputs and capture_inputs, never post-reject snapshot.',
               'Callback statistics include only callbacks retained in sampled gates; not all incoming callbacks.',
               'Delivery lock timing covers reference and input locks together.',
               'Snapshot validity false alone requires exporter/geometry evidence to explain.',
               'Main statistics exclude pre-goal standby and stop before reference cancellation after the last delivered command, else finish=true or bag end.',
               'Post-cancellation status counts are separate; a lingering cancel state is not the initiating failure.'])
    (ROOT/'mppi_gate_audit.json').write_text(json.dumps(summary,indent=2)+'\n')
    with (ROOT/'mppi_gate_rows.jsonl').open('w') as output:
        for row in all_rows: output.write(json.dumps(row)+'\n')
    print(json.dumps({key:summary[key] for key in ('window','states','delivery_lock_all')},indent=2))

if __name__=='__main__': main()
