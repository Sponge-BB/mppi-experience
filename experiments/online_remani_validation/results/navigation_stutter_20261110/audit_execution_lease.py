"""Read a bounded prefix of the existing recorder; no ROS imports or network."""
import bisect
from collections import Counter
from datetime import datetime, timezone, timedelta
import json
from pathlib import Path
import re

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent/'startup_listen_20261110_165922/events.jsonl'
TZ = timezone(timedelta(hours=8))
size = SOURCE.stat().st_size
rows, consumed, first, last = [], 0, None, None
with SOURCE.open('rb') as stream:
    while consumed < size:
        line = stream.readline()
        consumed += len(line)
        if not line or consumed > size:
            break
        try:
            row = json.loads(line)
        except ValueError:
            continue
        first = row['received'] if first is None else first
        last = row['received']
        if row['topic'] in ('/mppi/status', '/piper_execution_state', '/joint_ctrl_single', '/rosout_agg'):
            rows.append(row)

def local(stamp):
    return datetime.fromtimestamp(stamp, TZ).isoformat(timespec='milliseconds')

def quantiles(values):
    values = sorted(values)
    if not values:
        return None
    def at(q):
        x = (len(values)-1)*q
        a, b = int(x), min(int(x)+1, len(values)-1)
        return round(values[a]+(values[b]-values[a])*(x-a), 4)
    return dict(min=at(0), p50=at(.5), p95=at(.95), max=at(1))

execution = sorted((r for r in rows if r['topic']=='/piper_execution_state'), key=lambda r:r['received'])
commands = [r for r in rows if r['topic']=='/joint_ctrl_single']
statuses = [r for r in rows if r['topic']=='/mppi/status']
delivered = [r for r in statuses if r.get('state')=='command_delivered']
expiry = sorted((r for r in rows if r['topic']=='/rosout_agg' and
                 r.get('node')=='/piper_ctrl_single_node' and
                 'command stream expired' in r.get('message','')), key=lambda r:r['stamp'])
warnings = [r for r in rows if r['topic']=='/rosout_agg' and
            r.get('node')=='/whole_body_mppi' and
            'MPPI command rejected: waiting_explicit_arm_enable_or_normal_status ' in r.get('message','')]
abort = [r for r in rows if r['topic']=='/rosout_agg' and
         'abort old trajectory' in r.get('message','')]
exec_times = [r['received'] for r in execution]
endpoints = sorted(r['stamp']+.30 for r in commands)

def matched_lease(row):
    return bool(endpoints) and min(abs(row['valid_until']-t) for t in endpoints) <= .001

def stats(start, end):
    samples = [r for r in execution if start <= r['received'] <= end]
    healthy = [r for r in samples if r['enabled'] and r['feedback_valid'] and not r['stop_latched']]
    groups = {}
    for name, selected in (('all_healthy', healthy), ('command_active', [r for r in healthy if r['command_active']]),
                           ('healthy_hold', [r for r in healthy if not r['command_active']])):
        remaining = [(r['valid_until']-r['received'])*1000 for r in selected]
        groups[name] = dict(count=len(selected), remaining_at_recorder_ms=quantiles(remaining),
            source_age_at_recorder_ms=quantiles([(r['received']-r['stamp'])*1000 for r in selected]),
            remaining_le_ms={str(v):sum(x <= v for x in remaining) for v in (0, 5, 20, 50, 95)},
            valid_until_matches_observed_command_stamp_plus_300ms=sum(matched_lease(r) for r in selected))
    s = [r for r in statuses if start <= r['received'] <= end]
    return dict(start_local=local(start), end_local=local(end), execution_records=len(samples),
        enabled_false=sum(not r['enabled'] for r in samples), invalid_feedback=sum(not r['feedback_valid'] for r in samples),
        stop_latched=sum(r['stop_latched'] for r in samples), groups=groups,
        status_counts=dict(Counter(r['state'] for r in s)),
        driver_lease_expirations_by_log_source_time=sum(start <= r['stamp'] <= end for r in expiry))

waiting_rows = []
for status in statuses:
    if status['state']!='waiting_explicit_arm_enable_or_normal_status':
        continue
    # String status has no source timestamp. Match an available rospy log of
    # this rejection by its cycle duration and nearby recorder receipt only.
    candidates = [r for r in warnings if abs(r['received']-status['received']) < .15 and
                  abs(float(re.search(r'cycle=([0-9.]+)', r['message']).group(1))-status['cycle_ms']) < .006]
    warning = min(candidates, key=lambda r:abs(r['received']-status['received'])) if candidates else None
    event_time = warning['stamp'] if warning else status['received']
    index = bisect.bisect_right(exec_times, event_time)-1
    packet = execution[index] if index >= 0 else None
    next_packet = execution[index+1] if index+1 < len(execution) else None
    exp = min(expiry, key=lambda r:abs(r['stamp']-event_time)) if expiry else None
    row = dict(status_received=status['received'], event_time=event_time, local_time=local(event_time),
        event_time_kind='matched_rospy_log_source' if warning else 'recorder_receipt_only',
        cycle_ms=status['cycle_ms'],
        driver_expire_delta_ms=(exp['stamp']-event_time)*1000 if exp else None,
        matched_log_to_status_receive_ms=(status['received']-warning['stamp'])*1000 if warning else None)
    if packet:
        row['previous_recorded_execution'] = dict(packet,
            receipt_before_event_ms=(event_time-packet['received'])*1000,
            source_age_at_event_ms=(event_time-packet['stamp'])*1000,
            remaining_at_event_ms=(packet['valid_until']-event_time)*1000,
            exact_observed_command_lease=matched_lease(packet))
    if next_packet:
        row['next_recorded_execution'] = dict(next_packet,
            receipt_after_event_ms=(next_packet['received']-event_time)*1000,
            exact_observed_command_lease=matched_lease(next_packet))
    waiting_rows.append(row)

post_requested = datetime(2026, 11, 10, 17, 3, tzinfo=TZ).timestamp()
report = dict(
    source=str(SOURCE), source_bytes_at_read=size, complete_bytes_read=min(consumed,size),
    recorded_start=local(first), recorded_end=local(last),
    timing_contract=dict(received='Recorder time.time() at callback entry; not driver publication or MPPI receipt.',
        stamp='Execution header is actual last ArmStatus CAN source time; ROS log stamp is log creation time.',
        valid_until='Driver minimum of status/joints/six-motor source deadlines and an active old command lease.',
        sampling='Execution messages are sampled at >=50ms unless flags change; counts are recorded samples, not all driver packets.',
        matching='Lease match means valid_until within 1ms of an observed joint command source stamp + .30s. '
                 'Previous recorder packet is not necessarily the packet used inside MPPI; source ages do not expose all motor/joint sources.',
        goal_coverage='Recorder did not subscribe to goal; 17:03 republication cannot be verified here.'),
    full_recording=stats(first,last),
    observed_navigation_delivery_window=stats(delivered[0]['received'],delivered[-1]['received']),
    since_reported_1703_goal=stats(post_requested,last),
    waiting_summary=dict(count=len(waiting_rows),
        matched_log_source=sum(r['event_time_kind']=='matched_rospy_log_source' for r in waiting_rows),
        previous_packet_healthy=sum(r.get('previous_recorded_execution',{}).get('feedback_valid',False) and
                                   r.get('previous_recorded_execution',{}).get('enabled',False) for r in waiting_rows),
        previous_packet_expired_at_event=sum(r.get('previous_recorded_execution',{}).get('remaining_at_event_ms',1)>-1e9 and
                                            r.get('previous_recorded_execution',{}).get('remaining_at_event_ms',1)<=0 for r in waiting_rows),
        previous_packet_exact_lease=sum(r.get('previous_recorded_execution',{}).get('exact_observed_command_lease',False) for r in waiting_rows),
        driver_expiration_within_50ms=sum(abs(r['driver_expire_delta_ms'])<=50 for r in waiting_rows),
        cycle_ms=quantiles([r['cycle_ms'] for r in waiting_rows])),
    waiting_events=waiting_rows,
    tracker_abort_logs=[dict(source_stamp=r['stamp'], source_local=local(r['stamp']),
                            received=r['received'], message=r['message']) for r in abort],
    conclusion='Old lease clipping is directly visible in healthy-enabled packets and correlates with some waiting rejections. '
               'It is not proof of the exact MPPI callback state for each rejection, nor the cause of all pauses. '
               'After the last observed navigation delivery, cancelled-reference waiting is separate from command lease expiry.')
out=HERE/'execution_lease_evidence.json'
out.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:report[k] for k in ('recorded_end','observed_navigation_delivery_window',
                 'since_reported_1703_goal','waiting_summary','tracker_abort_logs')},indent=2))
