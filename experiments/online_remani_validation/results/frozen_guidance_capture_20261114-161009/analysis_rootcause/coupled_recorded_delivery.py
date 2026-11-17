#!/usr/bin/python3
"""Replay recorded acceptance/zero cadence while recomputing guide commands."""
import collections,json,time
from pathlib import Path
import coupled_native_clock as env
HERE=Path(__file__).resolve().parent
fixture=env.load_fixture();start=fixture['native_start']
statuses=[json.loads(l) for l in (HERE/'navigation_timing_status.jsonl').read_text().splitlines()]
events=[]
for s in statuses:
    # t was extracted from checked_inputs.checked_at, capture fallback, then receipt.
    delivered=s['state']=='command_delivered'
    zero=delivered and abs(s.get('base_v',0.))<1e-4 and abs(s.get('base_w',0.))<1e-4
    events.append(dict(t=s['t']-start,allow=delivered and not zero,state=s['state'],mode=s.get('mode'),recorded_zero=bool(zero),source_time=s['t'],receipt_time=s['bag']))
events.sort(key=lambda r:r['t'])
previous=[r for r in events if r['t']<0.]
active=([dict(previous[-1],t=0.)] if previous else [])+[r for r in events if r['t']>=0.]
began=time.monotonic();result=env.simulate(fixture=fixture,delivery_events=active,max_seconds=active[-1]['t'])
trace=result.pop('trace')
result['scope']+=' Recorded checked_at success/rejection/explicit-zero timestamps gate newly computed controls on a 5ms grid. No recorded nonzero command magnitudes are replayed. No plant fit, no added noise/lag, no fabricated delivery after the captured window.'
result['recorded_gate']=dict(time_axis='checked_inputs.checked_at; fallback capture then receipt, as extracted by timing_audit.py. Relative to native start, rounded upward to next 5ms simulation step.',source_status_count=len(statuses),all_status_counts=dict(collections.Counter(r['state'] for r in events)),window_seconds=active[-1]['t'],active_events=len(active),active_rejected=sum(not r['allow'] for r in active),active_explicit_zero=sum(r['recorded_zero'] for r in active),used_events=len(trace),used_rejected=sum(not r['delivery_event']['allow'] for r in trace),used_status_counts=dict(collections.Counter(r['delivery_event']['state'] for r in trace)),limitations=['checked_at precedes actual publish by a small unmeasured amount; Twist lacks a source header.','Reference and odometry remain ideal fresh in this simulation; only delivery cadence is imposed.','Rejection schedule is copied by wall-time, not recomputed from simulated map/feedback.','The native terminal alignment FSM is not simulated.'])
result['positive_scale_same_gear_endpoint_switches']=[r for r in result['nonzero_sign_change_rows'] if r['before']['scale']>0 and r['after']['scale']>0 and r['same_gear'] and r['endpoint_2_5cm_crossed']]
result['wall_seconds']=time.monotonic()-began
(HERE/'coupled_recorded_delivery_gate.json').write_text(json.dumps(active,indent=2)+'\n')
(HERE/'coupled_recorded_delivery_trace.jsonl').write_text(''.join(json.dumps(r,separators=(',',':'))+'\n' for r in trace))
(HERE/'coupled_recorded_delivery_report.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k not in ('flips','transitions','nonzero_sign_change_rows','positive_scale_same_gear_endpoint_switches','initial_state','final_state')},indent=2))
