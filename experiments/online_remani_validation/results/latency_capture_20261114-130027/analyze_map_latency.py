#!/usr/bin/python3
"""Stream bounded map-latency bag twice; never deserialize/store cloud arrays."""
import collections
import json
import math
from pathlib import Path
import re
import statistics
import sys
import rosbag

case = Path(sys.argv[1])
window_name = sys.argv[2] if len(sys.argv) > 2 else 'active1'
state = json.loads((case / 'status.json').read_text())
if state.get('phase') != 'closed':
    raise SystemExit('Refusing to read a capture that has not closed')
bagfile = case / 'latency.bag'

def quantiles(values):
    v = sorted(x for x in values if isinstance(x, (float, int)) and math.isfinite(x))
    if not v: return {'n': 0}
    def percentile(p):
        x = (len(v)-1)*p; i = int(x); j = min(i+1, len(v)-1)
        return v[i] + (v[j]-v[i])*(x-i)
    return dict(n=len(v), min=v[0], p50=percentile(.5), p95=percentile(.95), max=v[-1], mean=sum(v)/len(v))

def parse_trace(message, received):
    text = message.msg
    if '[MapTrace]' not in text and '[LatencyTrace]' not in text: return None
    record = {'t': message.header.stamp.to_sec(), 'received': received.to_sec(), 'node': message.name}
    for key, value in re.findall(r'(\w+)=([^\s]+)', text):
        try:
            if ',' in value: record[key] = [float(x) for x in value.split(',')]
            elif re.match(r'^-?\d+$', value): record[key] = int(value)
            else: record[key] = float(value)
        except ValueError: record[key] = value
    return record

with rosbag.Bag(str(bagfile)) as bag:
    goals, finishes, trajectories = [], [], []
    for topic, msg, t in bag.read_messages(topics=['/move_base_simple/goal', '/planning/finish', '/planning/trajectory']):
        if topic == '/move_base_simple/goal':
            goals.append({'received': t.to_sec(), 'source': msg.header.stamp.to_sec(),
                          'x': msg.pose.position.x, 'y': msg.pose.position.y})
        elif topic == '/planning/trajectory':
            trajectories.append({'received':t.to_sec(), 'action':int(msg.action), 'id':int(msg.trajectory_id)})
        else:
            finishes.append({'received': t.to_sec(), 'value': bool(getattr(msg, 'data', False))})
    if not goals: raise SystemExit('No recorded goal')
    start = goals[0]['received']
    finish = next((f['received'] for f in finishes if f['value'] and f['received'] >= start), None)
    end = finish if finish is not None else bag.get_end_time()
    bag_end = bag.get_end_time()
    cancels = [r['received'] for r in trajectories if r['action'] == 2 and r['received'] >= start]
    if window_name == 'active1' and cancels: end = cancels[0]
    elif window_name == 'active2' and len(cancels) >= 2:
        start = next(r['received'] for r in trajectories if r['action'] == 1 and cancels[0] < r['received'] < cancels[1])
        end = cancels[1]
    elif window_name == 'post_cancel' and cancels: start, end = cancels[-1], bag_end
    elif window_name != 'active1': raise SystemExit('Requested window not present')

traces = collections.defaultdict(list)
statuses = []
self_filter_logs = []
with rosbag.Bag(str(bagfile)) as bag:
    for topic, msg, received in bag.read_messages(topics=['/rosout_agg','/mppi/status']):
        rt = received.to_sec()
        if topic == '/rosout_agg':
            mt = msg.header.stamp.to_sec()
            if not start-3 <= mt <= end+1: continue
            record = parse_trace(msg, received)
            if record is not None: traces[record['stage']].append(record)
            elif start <= mt <= end and any(s in msg.msg.lower() for s in ('self-filter', 'self_filter','interpolation gap', 'scan-aligned', 'mapping evidence expired')):
                self_filter_logs.append({'t': mt, 'node':msg.name, 'msg':msg.msg})
        elif start <= rt <= end:
            data = json.loads(msg.data)
            statuses.append({'received':rt, 'data':data})
for rows in traces.values(): rows.sort(key=lambda r:r['t'])

fusion_by_obs = collections.defaultdict(list)
region_by_obs = collections.defaultdict(list)
submit_by_id = {}
read_by_version = collections.defaultdict(list)
output_by_wire = {}
sample_by_identity = collections.defaultdict(list)
for r in traces['fusion']: fusion_by_obs[(r['epoch'],r['obs'])].append(r)
for r in traces['region']: region_by_obs[(r['epoch'],r['obs'])].append(r)
for r in traces['submit']: submit_by_id[(r['epoch'],r['id'])]=r
for r in traces['read']:
    if r['complete']: read_by_version[(r['epoch'],r['obs'],r['rev'])].append(r)
for r in traces['export_sample']: sample_by_identity[(r['field'],r['obs'],round(r['fusion'],6))].append(r)
for r in traces['export_output']: output_by_wire[r['wire']]=r

def choose_before(rows, time, tolerance=.002):
    eligible=[r for r in rows if r['t'] <= time+tolerance]
    return max(eligible,key=lambda r:r['t']) if eligible else None

def same_fusion(a,b): return abs(a['fusion']-b['fusion']) < 1e-6

metrics=collections.defaultdict(list)
chains=[]
missing=collections.Counter()
for commit in traces['commit']:
    if not start <= commit['t'] <= end: continue
    identity=(commit['epoch'],commit['obs'])
    submit=submit_by_id.get((commit['epoch'],commit['id']))
    fusion=choose_before(fusion_by_obs.get(identity,[]),commit['t'])
    region=choose_before(region_by_obs.get(identity,[]), submit['t'] if submit else commit['t'])
    reads=[r for r in read_by_version.get((commit['epoch'],commit['obs'],commit['rev']),[]) if r['t'] >= commit['t']-.002 and same_fusion(r,commit)]
    read=min(reads,key=lambda r:r['t']) if reads else None
    chain={'epoch':commit['epoch'],'obs':commit['obs'],'id':commit['id'],'rev':commit['rev'],'fusion':commit['fusion'], 'commit':commit}
    for key,record in [('fusion_event',fusion),('region',region),('submit',submit),('read',read)]:
        if record is None: missing[key]+=1
        elif not same_fusion(record,commit): missing[key+'_fusion_mismatch']+=1
        else: chain[key]=record
    if 'fusion_event' in chain:
        metrics['fusion_input_age_ms'].append(fusion['input_age_ms'])
        metrics['fusion_pending_ms'].append(fusion['pending_ms'])
        metrics['fusion_occupancy_ms'].append(fusion['occupancy_ms'])
    if 'region' in chain:
        metrics['changed_cells'].append(region['changed'])
        metrics['region_prepare_ms'].append(region['prepare_ms'])
        if not region.get('full') and region.get('changed'):
            lo,hi=region['changed_lo'],region['changed_hi']
            metrics['changed_bbox_cells'].append(math.prod(hi[i]-lo[i]+1 for i in range(3)))
            metrics['compute_cells_per_changed_cell'].append(commit['compute_cells']/region['changed'])
    if 'submit' in chain:
        metrics['occupancy_snapshot_ms'].append(submit['snapshot_ms'])
        metrics['fusion_to_submit_log_ms'].append((submit['t']-commit['fusion'])*1000)
    for key in ('queue_ms','compute_ms','ready_wait_ms','commit_ms','prepare_ms','partition_ms','near_free_ms','negative_apply_ms','negative_fallbacks','regions','compute_cells','input_cells'):
        metrics['worker_'+key].append(commit[key])
    accounted=sum(commit['positive_zyx_ms'])+sum(commit['negative_zyx_ms'])+commit['prepare_ms']+commit['partition_ms']+commit['near_free_ms']+commit['negative_apply_ms']
    metrics['worker_unassigned_ms'].append(commit['compute_ms']-accounted)
    for which in ('positive','negative'):
        for axis,value in zip('zyx',commit[which+'_zyx_ms']):metrics[which+'_'+axis+'_ms'].append(value)
    metrics['fusion_to_commit_log_ms'].append((commit['t']-commit['fusion'])*1000)
    if 'read' in chain:
        metrics['fusion_to_read_complete_log_ms'].append((read['t']-commit['fusion'])*1000)
        metrics['commit_log_to_read_log_ms'].append((read['t']-commit['t'])*1000)
        for key in ('occ_ms','dist_ms','total_ms'): metrics['read_'+key].append(read[key])
    chains.append(chain)

adoptions={}
for status in statuses:
    diag=status['data'].get('candidate_diagnostics',{})
    for stage in ('capture_inputs','checked_inputs'):
        gate=diag.get(stage,{})
        h=gate.get('snapshot_header')
        cb=gate.get('snapshot_callback')
        if not h or not cb or len(h)!=32:continue
        wire=int(h[2]); field=int(h[14]); obs=int(h[29]); fusion=float(h[3])
        if int(cb.get('wire_seq',-1)) != wire:
            missing['callback_header_wire_mismatch']+=1;continue
        adoption={'wire':wire,'field':field,'obs':obs,'fusion':fusion,'callback':cb,
                  'gate_at':gate['checked_at'],'gate_stage':gate['stage'],'map_age_ms':gate['map_age_ms'],
                  'reason':status['data'].get('rejection_reason',status['data'].get('reason'))}
        if wire not in adoptions or adoption['gate_at'] < adoptions[wire]['gate_at']:adoptions[wire]=adoption

for adoption in adoptions.values():
    output=output_by_wire.get(adoption['wire'])
    if output is None: missing['adopted_wire_missing_output']+=1;continue
    if output['field']!=adoption['field'] or output['obs']!=adoption['obs'] or not same_fusion(output,adoption):
        missing['adopted_wire_identity_mismatch']+=1;continue
    adoption['output']=output
    samples=sample_by_identity.get((output['field'],output['obs'],round(output['fusion'],6)),[])
    sample=choose_before(samples,output['stamp'])
    if sample is None:missing['adopted_wire_missing_sample']+=1;continue
    adoption['sample']=sample
    reads=read_by_version.get((sample['epoch'],sample['obs'],sample['rev']),[])
    read=choose_before([r for r in reads if same_fusion(r,sample)],sample['acquired'])
    if read:adoption['read']=read
    else:missing['adopted_sample_missing_read']+=1
    cb=adoption['callback']
    for key in ('read_lock_wait_ms','decode_ms','write_lock_wait_ms'):metrics['snapshot_callback_'+key].append(cb[key])
    metrics['fusion_to_callback_enter_ms'].append((cb['enter']-adoption['fusion'])*1000)
    metrics['fusion_to_callback_adopted_ms'].append((cb['adopted']-adoption['fusion'])*1000)
    metrics['output_log_to_callback_enter_ms'].append((cb['enter']-output['stamp'])*1000)
    metrics['callback_enter_to_adopted_ms'].append((cb['adopted']-cb['enter'])*1000)
    metrics['adopted_to_first_checked_ms'].append((adoption['gate_at']-cb['adopted'])*1000)
    metrics['sample_ms'].append(sample['sample_ms'])
    metrics['sample_log_to_output_log_ms'].append((output['stamp']-sample['stamp'])*1000)
    metrics['output_ms'].append(output['output_ms'])
    if read:metrics['read_log_to_any_sample_acquired_ms'].append((sample['acquired']-read['t'])*1000)

# A new generation's first export is distinct from repeated samples of that generation.
first_samples = {}
for sample in traces['export_sample']:
    key=(sample['epoch'],sample['obs'],sample['rev'],round(sample['fusion'],6))
    if key not in first_samples or sample['acquired'] < first_samples[key]['acquired']:
        first_samples[key]=sample
for key,sample in first_samples.items():
    if not start <= sample['t'] <= end: continue
    reads=read_by_version.get(key[:3],[])
    read=choose_before([r for r in reads if same_fusion(r,sample)],sample['acquired'])
    if read:
        metrics['read_log_to_first_sample_acquired_ms'].append((sample['acquired']-read['t'])*1000)
        metrics['fusion_to_first_sample_acquired_ms'].append((sample['acquired']-sample['fusion'])*1000)
first_adoptions={}
for adoption in adoptions.values():
    sample=adoption.get('sample')
    if sample is None:continue
    key=(sample['epoch'],sample['obs'],sample['rev'],round(sample['fusion'],6))
    if key not in first_adoptions or adoption['callback']['adopted'] < first_adoptions[key]['callback']['adopted']:
        first_adoptions[key]=adoption
for adoption in first_adoptions.values():
    cb=adoption['callback']; sample=adoption['sample']; read=adoption.get('read')
    metrics['fusion_to_first_observed_adoption_ms'].append((cb['adopted']-adoption['fusion'])*1000)
    if read:metrics['read_log_to_first_observed_adoption_ms'].append((cb['adopted']-read['t'])*1000)
for read in traces['read']:
    if not start <= read['t'] <= end:continue
    if read['occ_copy']:metrics['all_occupancy_read_copy_ms'].append(read['occ_ms'])
    if read['dist_copy']:metrics['all_distance_read_copy_ms'].append(read['dist_ms'])
for stage in ('fusion','commit'):
    rows=[r for r in traces[stage] if start<=r['t']<=end]
    for before,after in zip(rows,rows[1:]):metrics[stage+'_generation_interval_ms'].append((after['t']-before['t'])*1000)

summary={'window':{'name':window_name,'trajectories':trajectories,'goal':goals[0],'start':start,'end':end,'finish':finish,'bag_end':bag_end,'duration_s':end-start},
         'trace_counts':{stage:sum(start<=r['t']<=end for r in rows) for stage,rows in traces.items()},
         'matched_committed_jobs':len(chains),'observed_adopted_wires':len(adoptions),
         'join_gaps':dict(missing),'metrics':{key:quantiles(values) for key,values in sorted(metrics.items())},
         'self_filter_logs':self_filter_logs,
         'caveats':['All stage durations use same generation/epoch/request/revision; wire identities also verify field+obs+fusion.',
                    'ROS log event times occur just after the measured stage, so cross-stage log gaps include logging overhead.',
                    'Output log follows publisher.publish; a callback can begin before that log timestamp.',
                    'Adoption metadata comes from MPPI gate reports: callbacks replaced before a gate are not enumerated.',
                    'Any-sample read delay includes repeated exports of the same map; first-sample metrics isolate initial delivery.',
                    'First observed adoption may miss earlier callbacks replaced before a control gate.',
                    'No full occupancy arrays were captured; changed bounding boxes do not prove which objects changed.']}
(case/('map_latency_'+window_name+'.json')).write_text(json.dumps(summary,indent=2)+'\n')
with (case/('map_generation_chains_'+window_name+'.jsonl')).open('w') as file:
    for row in chains:file.write(json.dumps(row)+'\n')
with (case/('map_adoption_chains_'+window_name+'.jsonl')).open('w') as file:
    for row in sorted(adoptions.values(),key=lambda r:r['gate_at']):file.write(json.dumps(row)+'\n')
with (case/('map_trace_events_'+window_name+'.jsonl')).open('w') as file:
    for row in sorted((r for rows in traces.values() for r in rows),key=lambda r:r['t']):file.write(json.dumps(row)+'\n')
print(json.dumps(summary,indent=2))
