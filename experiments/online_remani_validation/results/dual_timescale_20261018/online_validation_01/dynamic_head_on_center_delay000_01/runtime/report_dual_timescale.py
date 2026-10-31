#!/usr/bin/env python3
"""Evidence-only report for the coarse dual-time-scale diagnostic experiment.

Simulator actor appearance is NOT perception detection. MPPI file tail receipt
monotonic clocks are NOT the original consumption clocks. Native wall-clock
accounting uses only native monotonic begin/end events; nested spans are unions.
"""
import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


def load_json(path, default=None):
    return json.loads(path.read_text()) if path.exists() else default


def load_lines(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()] if path.exists() else []


def load_csv(path):
    return np.atleast_1d(np.genfromtxt(path, names=True, delimiter=',', encoding='utf8')) if path.exists() else np.empty(0,dtype=[('t',float)])


def union_duration(intervals, window=None):
    """Union duration, not sum; optionally clip to a measured wall window."""
    clipped=[]
    for begin,end in intervals:
        if window is not None:begin,end=max(begin,window[0]),min(end,window[1])
        if end>begin:clipped.append((begin,end))
    clipped.sort();total=0.;last=None
    for begin,end in clipped:
        if last is None:last=[begin,end]
        elif begin<=last[1]:last[1]=max(last[1],end)
        else:total+=last[1]-last[0];last=[begin,end]
    return total+(last[1]-last[0] if last is not None else 0.)


def pair_events(events, begin_event, end_event, key):
    active=defaultdict(list);closed=[];unmatched=[]
    for row in sorted(events,key=lambda r:r['mono']):
        if row.get('event')==begin_event:active[key(row)].append(row)
        elif row.get('event')==end_event:
            ident=key(row)
            if active[ident]:closed.append((active[ident].pop(),row))
            else:unmatched.append(dict(reason='end_without_begin',event=row))
    unmatched += [dict(reason='unclosed_begin',event=row) for rows in active.values() for row in rows]
    return closed,unmatched


def native_time(row, origin=1000.):
    return float(row['sim'])-origin


def write_csv(path, rows, fields=None):
    if fields is None:fields=list(dict.fromkeys(k for row in rows for k in row))
    with path.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({k:json.dumps(v,ensure_ascii=False) if isinstance(v,(dict,list)) else v for k,v in row.items()})


def publication_chain(publication, observer, refs, ref_events, diagnostics, origin=1000.):
    """Match exact transaction and segment hashes, never only temporal proximity."""
    transaction=str(publication.get('transaction_fnv64',''))
    receipts=[r for r in observer if r.get('event')=='reference_observer_receipt' and str(r.get('transaction_fnv64',''))==transaction]
    receipt=min(receipts,key=lambda r:r['mono']) if receipts else None
    result=dict(transaction_fnv64=transaction,publish_sim_s=native_time(publication,origin),
                publish_mono=publication['mono'],request_id=publication.get('request_id'),
                observer_receipt_matched=receipt is not None,
                receipt_time_is_transport_observation=True)
    if receipt is None:return result
    result['observer_receipt_sim_s']=native_time(receipt,origin)
    hashes=tuple(receipt.get('message_hashes',[]))
    if not hashes:
        result['match_error']='receipt has no segment hashes';return result
    result['message_hashes']=list(hashes)
    commits=[r for r in ref_events if tuple(r.get('message_hashes',[]))==hashes and r.get('event') not in ('cancel','invalid')]
    consumed=[r for r in refs if tuple(r.get('message_hashes',[]))==hashes]
    commit=min(commits,key=lambda r:r['t']) if commits else None
    consumption=min(consumed,key=lambda r:r['t']) if consumed else None
    result.update(commit_matched=commit is not None,consumption_matched=consumption is not None)
    if commit:
        result.update(commit_sim_s=commit['t'],commit_revision=commit.get('revision'))
    if consumption:
        reference_hash=consumption.get('sha256')
        result.update(consume_sim_s=consumption['t'],consume_revision=consumption.get('revision'),reference_sha256=reference_hash,
                      publish_to_consume_sim_s=consumption['t']-native_time(publication,origin))
        # These are the original source times, NOT Observer.tail()'s later mono.
        cycles=[r for r in diagnostics if r.get('reference_sha256')==reference_hash and r['t']>=consumption['t']-1e-9]
        if cycles:
            cycle=min(cycles,key=lambda r:r['t'])
            result.update(first_control_sim_s=cycle['t'],first_control_reason=cycle.get('reason'),
                          first_control_v=cycle.get('applied_v'),first_control_w=cycle.get('applied_w'),
                          publish_to_first_control_sim_s=cycle['t']-native_time(publication,origin))
        result['first_control_matched']=bool(cycles)
    result['complete_exact_chain']=bool(receipt and commit and consumption and result.get('first_control_matched'))
    return result


def integrate_diagnostics(diagnostics,start,end):
    """Actual MPPI output reasons, not native advisory clock freeze flags."""
    rows=sorted([r for r in diagnostics if r['t']<=end],key=lambda r:r['t'])
    seconds=defaultdict(float);zero=defaultdict(float);low_linear=defaultdict(float)
    longest_zero=0.;zero_start=None
    for index,row in enumerate(rows):
        begin=max(start,float(row['t']));finish=min(end,float(rows[index+1]['t']) if index+1<len(rows) else end)
        if finish<=begin:continue
        reason=row.get('reason','unknown');dt=finish-begin
        seconds[reason]+=dt
        v,w=float(row.get('applied_v',0)),float(row.get('applied_w',0))
        iszero=abs(v)<1e-6 and abs(w)<1e-6
        if iszero:
            zero[reason]+=dt
            if zero_start is None:zero_start=begin
            longest_zero=max(longest_zero,finish-zero_start)
        else:zero_start=None
        if abs(v)<.02:low_linear[reason]+=dt
    return dict(reason_duration_s=dict(seconds),zero_command_duration_s=dict(zero),
                low_linear_command_duration_s=dict(low_linear),longest_both_zero_command_s=longest_zero,
                zero_definition='abs(applied_v)<1e-6 and abs(applied_w)<1e-6; includes terminal settling',
                low_linear_note='includes in-place turning; not automatically a stop',
                integration='piecewise held diagnostic output; does not prove exact motor-controller actuation')


def summarize_trial(source,out):
    source=Path(source).resolve();out=Path(out);out.mkdir(parents=True,exist_ok=False)
    native=load_lines(source/'dual_timeline.jsonl')
    if not native:raise ValueError('Missing coarse native timeline: '+str(source))
    observer=load_lines(source/'observer_events.jsonl');resource=load_lines(source/'solver_trace.jsonl')
    failures=[r for r in load_lines(source/'failure_trace.jsonl') if r.get('kind')=='solve' and r.get('label')=='normal']
    refs=load_lines(source/'consumed_references.jsonl');ref_events=load_lines(source/'reference_events.jsonl')
    diagnostics=load_json(source/'mppi_diagnostics.json',[])
    state,obs=load_csv(source/'state.csv'),load_csv(source/'obstacles.csv')
    result=load_json(source/'result.json',{});status=load_json(source/'run_status.json',{})
    origin=1000.
    anchor=float(result.get('motion_anchor') or result.get('goal_sent') or state['t'][0])
    end=float(min(result.get('elapsed',state['t'][-1]),state['t'][-1]))
    appearance=float(obs['t'].min()) if len(obs) else None
    blocked=[r for r in native if r.get('event')=='cooperative_check' and r.get('blocked') and (appearance is None or native_time(r,origin)>=appearance)]
    first_block=min(blocked,key=lambda r:r['mono']) if blocked else None
    triggers=[r for r in native if r.get('event')=='refresh_trigger' and (first_block is None or r['mono']>=first_block['mono'])]
    first_trigger=min(triggers,key=lambda r:r['mono']) if triggers else None
    pairs,unmatched_requests=pair_events(native,'request_begin','request_end',lambda r:r['request_id'])
    stages,unmatched_stages=pair_events(native,'stage_begin','stage_end',lambda r:(r.get('request_id'),r.get('stage')))
    waits,unmatched_waits=pair_events(native,'wait_begin','wait_end',lambda r:r['wait_key'])
    resource_pairs,_=pair_events(resource,'begin','end',lambda r:(r.get('id'),r.get('stage')))
    request_rows=[]
    for begin,finish in pairs:
        request_id=begin['request_id'];interval=(begin['mono'],finish['mono'])
        matched=[(a,b) for a,b in stages if a.get('request_id')==request_id]
        by_stage=defaultdict(list)
        for a,b in matched:by_stage[a.get('stage')].append((a['mono'],b['mono']))
        row=dict(request_id=request_id,start_sim_s=native_time(begin,origin),end_sim_s=native_time(finish,origin),
                 start_mono=begin['mono'],end_mono=finish['mono'],request_wall_ms=1000*(finish['mono']-begin['mono']),
                 success=finish.get('success'),local_execution_owner=begin.get('local_execution_owner'),
                 start_x=begin.get('start_x'),start_y=begin.get('start_y'),start_yaw=begin.get('start_yaw'),
                 stage_wall_ms={name:1000*union_duration(spans,interval) for name,spans in by_stage.items()},
                 search_results=[dict(status=b.get('status'),success=b.get('success'),
                                      status_name={1:'REACH_HORIZON',2:'REACH_END',3:'NO_PATH',4:'REACH_END_BUT_SHOT_FAILS',5:'START_COLLISION',6:'GOAL_COLLISION'}.get(b.get('status'),'unknown'))
                                 for a,b in matched if a.get('stage')=='search'],
                 acceptance_rejections=[r.get('reason') for r in native if r.get('request_id')==request_id and r.get('event')=='acceptance_reject'])
        # Initialization includes search. manager_replan includes initialization
        # and optimizer. Never add all these nested measurements together.
        partition=[span for name in ('initialization','optimizer','live_acceptance') for span in by_stage[name]]
        row['other_request_wall_ms']=max(0.,row['request_wall_ms']-1000*union_duration(partition,interval))
        row['search_inside_initialization']=True
        optimizer_resource=[(a,b) for a,b in resource_pairs if a.get('stage')=='optimizer' and interval[0]-.001<=a['mono']<=b['mono']<=interval[1]+.001]
        optimized=[]
        for a,b in optimizer_resource:
            candidates=[f for f in failures if str(f.get('input_hash'))==str(b.get('input_hash')) and abs(float(f.get('sim',0))-float(a.get('sim',0)))<.1]
            f=min(candidates,key=lambda f:abs(f['sim']-a['sim'])) if candidates else None
            optimized.append(dict(resource_optimizer_id=a.get('id'),wall_ms=b.get('wall_ms'),thread_cpu_ms=b.get('thread_cpu_ms'),
                                  runqueue_wait_ms=b.get('runqueue_wait_ms'),budget_s=b.get('budget_s'),iterations=b.get('iterations'),
                                  input_hash=str(b.get('input_hash')),success=b.get('success'),failure_trace_matched=f is not None,
                                  reason=f.get('reason') if f else None,rejection_value=f.get('rejection_value') if f else None,
                                  max_left_omega=f.get('max_left_omega') if f else None,max_right_omega=f.get('max_right_omega') if f else None))
        row['optimizer_calls']=optimized
        if row['success']:row['outcome_class']='accepted'
        elif row['acceptance_rejections']:row['outcome_class']='live_acceptance_reject'
        elif optimized:row['outcome_class']='optimizer_reject_or_timeout'
        elif row['search_results'] and any(not r.get('success') for r in row['search_results']):row['outcome_class']='search_failure'
        else:row['outcome_class']='unclassified_failure'
        request_rows.append(row)
    publications=sorted([r for r in native if r.get('event')=='reference_publish'],key=lambda r:r['mono'])
    chains=[publication_chain(p,observer,refs,ref_events,diagnostics,origin) for p in publications]
    refreshed=[(p,c) for p,c in zip(publications,chains) if first_block and p['mono']>=first_block['mono']]
    first_publication,first_chain=refreshed[0] if refreshed else (None,None)
    # Request/retry wall-accounting has an exact native wall window. Actor→
    # consumption latency is a separate simulation-time metric, never mixed in.
    accounting=None
    if first_block and first_publication:
        window=(first_block['mono'],first_publication['mono'])
        request_spans=[(a['mono'],b['mono']) for a,b in pairs]
        wait_spans=[(a['mono'],b['mono']) for a,b in waits]
        req=union_duration(request_spans,window);wait=union_duration(wait_spans,window)
        occupied=union_duration(request_spans+wait_spans,window)
        accounting=dict(window='first native blocked check to first later native reference publication',
                        total_wall_s=window[1]-window[0],request_union_wall_s=req,
                        closed_explicit_wait_union_wall_s=wait,request_and_wait_overlap_wall_s=req+wait-occupied,
                        unattributed_wall_s=max(0.,window[1]-window[0]-occupied),
                        wait_by_key_union_wall_s={key:union_duration([(a['mono'],b['mono']) for a,b in waits if a['wait_key']==key],window)
                                                 for key in sorted({a['wait_key'] for a,b in waits})},
                        interpretation='closed measured intervals only; wait keys can overlap; unattributed is not assumed CPU contention')
    latency=dict(actor_truth_appearance_sim_s=appearance,
                 actor_note='first recorded simulator actor sample; NOT perception identification',
                 first_native_blocked_sim_s=native_time(first_block,origin) if first_block else None,
                 first_refresh_trigger_sim_s=native_time(first_trigger,origin) if first_trigger else None,
                 first_refresh_publish_sim_s=native_time(first_publication,origin) if first_publication else None,
                 first_refresh_consume_sim_s=first_chain.get('consume_sim_s') if first_chain else None,
                 first_refresh_control_sim_s=first_chain.get('first_control_sim_s') if first_chain else None)
    if appearance is not None:
        for label in ('first_native_blocked','first_refresh_trigger','first_refresh_publish','first_refresh_consume','first_refresh_control'):
            value=latency[label+'_sim_s'];latency['appearance_to_'+label+'_sim_s']=value-appearance if value is not None else None
    after_block=[r for r in request_rows if first_block and r['start_mono']>=first_block['mono'] and (not first_publication or r['start_mono']<=first_publication['mono'])]
    stop=integrate_diagnostics(diagnostics,anchor,end)
    # Actual measured nearly-zero motion is independent of both advisory time
    # scale and the selected nominal command. Include its thresholds explicitly.
    measured=0.
    for index,row in enumerate(state[:-1]):
        if row['t']<anchor:continue
        if abs(row['v'])<.01 and abs(row['w'])<.03:measured+=max(0,min(end,state['t'][index+1])-row['t'])
    stop.update(measured_nearly_zero_motion_s=measured,measured_thresholds=dict(v_abs_m_s=.01,w_abs_rad_s=.03),
                native_clock_freeze_used_as_stop_evidence=False)
    inputs=[source/name for name in ('dual_timeline.jsonl','observer_events.jsonl','solver_trace.jsonl','failure_trace.jsonl',
                                    'state.csv','obstacles.csv','result.json','run_status.json','mppi_diagnostics.json',
                                    'consumed_references.jsonl','reference_events.jsonl') if (source/name).exists()]
    summary=dict(source=str(source),scope=result.get('scope'),scene=result.get('scene'),result=result.get('reason'),
                 anchor_sim_s=anchor,end_sim_s=end,clock_origin_ros_s=origin,latency=latency,
                 native_wall_accounting=accounting,requests=request_rows,request_count_until_first_refresh=len(after_block),
                 request_outcomes_until_first_refresh=dict(Counter(row['outcome_class'] for row in after_block)),
                 optimizer_reasons_until_first_refresh=dict(Counter(call.get('reason','unmatched') for row in after_block for call in row['optimizer_calls'])),
                 reference_chains=chains,stop_analysis=stop,
                 unmatched=dict(requests=unmatched_requests,stages=unmatched_stages,waits=unmatched_waits),
                 run_status=status,publisher_audit=load_json(source/'publisher_audit.json',{}),
                 source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},
                 limitations=['No Tracker-freeze explanation applied to MPPI-owned execution.',
                              'Small repeated closed loops; native random search not identical-input causal pairs.',
                              'Simulator appearance is not detector identification; observer receipt is not publisher time.',
                              'Unclosed waits excluded from attributed duration; not silently extended to run end.',
                              'Clock fractions and wall delays are not mixed; profiler overhead not subtracted.'])
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    write_csv(out/'requests.csv',request_rows)
    write_csv(out/'reference_chains.csv',chains)
    return summary


def report_batch(batch,out):
    batch=Path(batch).resolve();out=Path(out);out.mkdir(parents=True,exist_ok=False)
    sources=sorted(p.parent for p in batch.rglob('dual_timeline.jsonl') if '/runtime/' not in str(p))
    summaries=[]
    for source in sources:
        if not (source/'result.json').exists():continue
        summaries.append(summarize_trial(source,out/source.name))
    rows=[]
    for s in summaries:
        l=s['latency'];w=s['native_wall_accounting'] or {}
        rows.append(dict(trial=Path(s['source']).name,scene=s['scene'],result=s['result'],
                         actor_to_native_blocked_sim_s=l.get('appearance_to_first_native_blocked_sim_s'),
                         actor_to_reference_publish_sim_s=l.get('appearance_to_first_refresh_publish_sim_s'),
                         actor_to_reference_consume_sim_s=l.get('appearance_to_first_refresh_consume_sim_s'),
                         actor_to_first_control_sim_s=l.get('appearance_to_first_refresh_control_sim_s'),
                         requests_until_first_refresh=s['request_count_until_first_refresh'],
                         native_window_wall_s=w.get('total_wall_s'),requests_union_wall_s=w.get('request_union_wall_s'),
                         explicit_wait_union_wall_s=w.get('closed_explicit_wait_union_wall_s'),unattributed_wall_s=w.get('unattributed_wall_s'),
                         zero_command_s=sum(s['stop_analysis']['zero_command_duration_s'].values()),
                         measured_nearly_zero_motion_s=s['stop_analysis']['measured_nearly_zero_motion_s'],
                         request_outcomes=s['request_outcomes_until_first_refresh'],
                         optimizer_reasons=s['optimizer_reasons_until_first_refresh']))
    write_csv(out/'batch_summary.csv',rows)
    result=dict(batch=str(batch),trials=len(summaries),rows=rows,
                interpretation='descriptive diagnostic records, no claim of statistical superiority or stable p95',
                methods='exact native monotonic interval unions; exact reference transaction/segment/route-hash matching')
    (out/'batch_summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch',type=Path,required=True);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();result=report_batch(args.batch,args.out)
    print(json.dumps(result,ensure_ascii=False,indent=2))
