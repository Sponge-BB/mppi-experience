#!/usr/bin/env python3
"""Offline integrity audit, not a navigation-success filter.

Run only after the measured batch ends. A collision or navigation timeout is a
valid retained outcome. Incomplete waits/stages at normal termination are
explicitly right-censored; crashed nodes or contradicted interfaces are invalid.
No ROS master, publisher, rendering, compilation, or source modification occurs.
"""
import argparse
from collections import Counter
from contextlib import redirect_stdout
import hashlib
import io
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent.parent


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path,default=None):
    return json.loads(path.read_text()) if path.exists() else default


def lines(path):
    return [json.loads(x) for x in path.read_text().splitlines() if x.strip()] if path.exists() else []


def fnv64(text):
    value=1469598103934665603
    for byte in text.encode('utf-8'):value=((value^byte)*1099511628211)&0xffffffffffffffff
    return str(value)


def event_audit(events,resource):
    errors=[];warnings=[];requests={};active=None;stack=[];waits={};closed_waits=[];stage_pairs=[]
    last_mono=None;resource_links=[];publications=[]
    for index,row in enumerate(events):
        event=row.get('event');request=row.get('request_id',0);mono=row.get('mono')
        if mono is None or not isinstance(mono,(int,float)):
            errors.append('event has no numeric monotonic clock at row %d'%index);continue
        if last_mono is not None and mono<last_mono:errors.append('native event clock moves backward at row %d'%index)
        last_mono=mono
        if event=='request_begin':
            if active is not None:errors.append('overlapping native requests')
            if request<=0 or request in requests:errors.append('invalid/reused native request ID')
            requests[request]={'begin':row,'end':None};active=request
        elif event=='request_end':
            if request!=active or request not in requests:errors.append('orphan/mismatched request end')
            else:requests[request]['end']=row
            if stack:errors.append('request ended with live child stages')
            active=None
        elif event in ('stage_begin','stage_end','resource_link','manager_result','acceptance_pass','acceptance_reject','reference_publish'):
            if active is None or request!=active:errors.append('child event outside its native request: '+str(event))
        if event=='stage_begin':stack.append(((request,row.get('stage')),row))
        elif event=='stage_end':
            key=(request,row.get('stage'))
            if not stack or stack[-1][0]!=key:errors.append('stage end violates strict nesting: '+str(key))
            else:
                _,begin=stack.pop();stage_pairs.append((begin,row))
                if mono<begin['mono']:errors.append('negative stage interval')
        elif event=='wait_begin':
            key=row.get('wait_key')
            if key in waits:errors.append('duplicate open wait: '+str(key))
            waits[key]=row
        elif event=='wait_end':
            key=row.get('wait_key')
            if key not in waits:errors.append('wait end has no begin: '+str(key))
            else:closed_waits.append((waits.pop(key),row))
        elif event=='resource_link':resource_links.append(row)
        elif event=='reference_publish':publications.append(row)
    resource_begin={row['id']:row for row in resource if row.get('event')=='begin' and row.get('stage')=='replan'}
    resource_end={row['id']:row for row in resource if row.get('event')=='end' and row.get('stage')=='replan'}
    for link in resource_links:
        ident=link.get('resource_replan_id');request=link['request_id'];begin=resource_begin.get(ident);end=resource_end.get(ident)
        if begin is None:errors.append('resource scope ID absent from solver_trace: '+str(ident));continue
        native=requests.get(request,{})
        if begin['mono']<native.get('begin',begin)['mono']-.001:errors.append('resource scope begins outside its parent request')
        if end is not None and native.get('end') is not None and end['mono']>native['end']['mono']+.001:
            errors.append('resource scope ends outside its parent request')
        if end is None:warnings.append('resource replan scope right-censored: '+str(ident))
    unclosed_requests=[k for k,v in requests.items() if v['end'] is None]
    if unclosed_requests:warnings.append('request(s) right-censored at log end')
    if stack:warnings.append('stage(s) right-censored at log end')
    if waits:warnings.append('wait(s) right-censored at log end; no duration silently imputed')
    if not requests:errors.append('no native request events')
    return dict(pass_integrity=not errors,errors=errors,warnings=warnings,
                events=len(events),requests=len(requests),closed_requests=len(requests)-len(unclosed_requests),
                closed_stages=len(stage_pairs),closed_waits=len(closed_waits),resource_links=len(resource_links),
                right_censored_requests=unclosed_requests,
                right_censored_stages=[dict(request_id=k[0],stage=k[1],begin_mono=r['mono']) for k,r in stack],
                right_censored_waits=list(waits.values()),publications=len(publications),
                nesting='strict native request/stage stack; stages are nested, never summed twice')


def reference_chain_audit(trial,timeline,observer,refs,commits,report=None):
    # The independent native coefficient audit is intentionally reused, with
    # image generation disabled. Canonical ROS bytes remove public header.seq.
    from audit_native_reference import recorded_message
    canonical={}
    for record in load(trial/'trajectories.json',[]):
        stream=io.BytesIO();recorded_message(record).serialize(stream);raw=stream.getvalue()
        canonical[hashlib.sha256(raw).hexdigest()]=raw
    diagnostics=load(trial/'mppi_diagnostics.json',[]);errors=[];warnings=[];chains=[]
    publications=[r for r in timeline if r.get('event')=='reference_publish']
    receipts=[r for r in observer if r.get('event')=='reference_observer_receipt']
    for publication in publications:
        token=str(publication['transaction_fnv64'])
        receipt=next((r for r in receipts if str(r.get('transaction_fnv64'))==token and r['mono']>=publication['mono']-.05),None)
        chain=dict(transaction_fnv64=token,request_id=publication['request_id'],publish_sim=publication['sim'])
        if receipt is None:
            final_observer=max((r.get('mono',0) for r in observer),default=0)
            if publication['mono']>=final_observer-.1:
                warnings.append('publication transport observation right-censored at observer shutdown: '+token)
                chain['observer_shutdown_censored']=True
            else:errors.append('published transaction not seen by observer: '+token)
            chains.append(chain);continue
        hashes=tuple(receipt.get('message_hashes',[]));earliest=publication['sim']-1000.-.05
        same=[r for r in commits if tuple(r.get('message_hashes',[]))==hashes and r.get('event')=='replace' and r['t']>=earliest]
        commit=min(same,key=lambda r:r['t']) if same else None
        same=[r for r in refs if tuple(r.get('message_hashes',[]))==hashes and r['t']>=earliest]
        consumed=min(same,key=lambda r:r['t']) if same else None
        chain.update(observer_receipt=True,commit=commit is not None,consumption=consumed is not None)
        if commit:
            try:
                text=commit['goal_key']+'\n'+str(len(hashes))+''.join('\n'+canonical[h].hex() for h in hashes)
                reconstructed_sha=hashlib.sha256(text.encode()).hexdigest()
                reconstructed_fnv=fnv64(text)
                chain.update(reconstructed_transaction_fnv64=reconstructed_fnv,
                             reconstructed_transaction_sha256=reconstructed_sha,commit_revision=commit['revision'])
                if reconstructed_fnv!=token or reconstructed_sha!=receipt.get('transaction_sha256'):
                    errors.append('full transaction bytes differ from independent native coefficients: '+token)
                if fnv64(commit['goal_key'])!=publication.get('goal_fnv64'):
                    errors.append('goal key mismatch in transaction: '+token)
            except KeyError as exc:errors.append('cannot reconstruct published native transaction: '+str(exc))
        else:
            # A reference may be pending under the deliberately tested delay at
            # normal trial end. Preserve that as censoring, not algorithm failure.
            warnings.append('reference commit not observed (possibly delivery right-censored): '+token)
        if consumed:
            matching=[r for r in diagnostics if r.get('bundle')==consumed['revision'] and r.get('reference_sha256')==consumed['sha256'] and r['t']>=consumed['t']-1e-9]
            chain.update(consume_revision=consumed['revision'],consume_sim=consumed['t']+1000.,
                         reference_sha256=consumed['sha256'],first_control_cycle=bool(matching))
            if not matching:errors.append('consumed reference has no matching MPPI cycle: '+token)
            if commit and consumed['revision']!=commit['revision']:errors.append('commit/consumption revision mismatch: '+token)
        elif commit:warnings.append('committed reference consumption right-censored: '+token)
        chains.append(chain)
    for ref in refs:
        hashes=tuple(ref.get('message_hashes',[]))
        if not any(tuple(r.get('message_hashes',[]))==hashes for r in receipts):errors.append('consumed reference has no transaction receipt')
        if not any(tuple(r.get('message_hashes',[]))==hashes and r.get('event')=='replace' for r in commits):
            errors.append('consumed reference has no committed goal-scoped transaction')
    reporter_match=None
    if report:
        reported=load(report,{})
        reported_chains={str(r['transaction_fnv64']):r for r in reported.get('reference_chains',[])}
        for chain in chains:
            other=reported_chains.get(chain['transaction_fnv64'])
            if other is None:errors.append('reporter omitted native publication transaction');continue
            if chain.get('consumption') and (chain.get('consume_revision')!=other.get('consume_revision') or
                    abs(chain['consume_sim']-1000.-other.get('consume_sim_s',-1e9))>1e-8):
                errors.append('reporter consumption time/revision differs from independent audit')
        reporter_match=not errors
    return dict(pass_integrity=not errors,errors=errors,warnings=warnings,chains=chains,
                reporter_crosscheck=reporter_match,clock_note='source ROS event times used; observer tail mono is never treated as MPPI receipt')


def advisory_freeze_audit(observer,refs):
    identities={(r['revision'],r['sha256']):r for r in refs}
    latest=None;odom=None;matched=0;nonzero=0;measured=0;examples=[]
    for row in sorted(observer,key=lambda x:x['mono']):
        if row.get('event')=='progress' and row.get('role')=='native_controller_advisory':latest=row
        elif row.get('event')=='measured_motion':odom=row
        elif row.get('event')=='mppi_cycle' and latest:
            reference=identities.get((row.get('bundle'),row.get('reference_sha256')))
            if not reference or latest.get('time_scale')!=0 or latest.get('reference_start',0)<=1000:continue
            if abs(latest['reference_start']-reference['stamp'])>1e-6 or not 0<=row['sim']-latest['sim']<.2:continue
            matched+=1
            moving=abs(row.get('applied_v') or 0)>.02 or abs(row.get('applied_w') or 0)>.04
            if moving:
                nonzero+=1
                if odom and 0<=row['sim']-odom['sim']<.2 and (abs(odom['v'])>.02 or abs(odom['w'])>.04):measured+=1
                if len(examples)<4:examples.append(dict(sim=row['sim'],reference_revision=row['bundle'],
                    native_virtual_time=latest['virtual_time'],v=row.get('applied_v'),w=row.get('applied_w'),reason=row.get('reason')))
    return dict(valid_same_reference_advisory_zero_cycles=matched,mppi_nonzero_command_cycles=nonzero,
                also_measured_nonzero_motion_cycles=measured,examples=examples,
                interpretation='advisory controller-clock hold is not MPPI stop; commands are separate from measured motion',
                matching='exact consumed revision/hash and same native start stamp; latest advisory and odom age <0.2s',
                nonzero_thresholds=dict(abs_v_m_s=.02,abs_w_rad_s=.04))


def build_integrity(build,plan):
    manifest=load(build/'build_manifest.json',{});errors=[]
    expected=manifest.get('production_sources_sha256',{})
    changed=[name for name,digest in expected.items() if not (ROOT/name).exists() or sha(ROOT/name)!=digest]
    if not expected or changed:errors.append('production source snapshot missing/changed')
    if plan and plan.get('production_sources_sha256')!=expected:errors.append('batch production snapshot differs from build')
    binaries={}
    for name,digest in manifest.get('native_binaries_sha256',{}).items():
        actual=sha(build/'build'/name);binaries[name]=actual
        if actual!=digest:errors.append('native binary hash mismatch: '+name)
    if len(binaries)!=3:errors.append('expected three native binary hashes')
    mppi=sha(build/'build/libonline_mppi.so')
    if mppi!=manifest.get('mppi_library_sha256'):errors.append('MPPI library hash mismatch')
    return dict(pass_integrity=not errors,errors=errors,production_files=len(expected),production_changed=changed,
                native_binaries_sha256=binaries,mppi_library_sha256=mppi,build=str(build),
                scope='current files vs recorded before/after hashes; no claim of historical in-memory executable attestation')


def audit_trial(trial,build_check,report=None):
    from audit_native_reference import audit as native_audit
    errors=[];warnings=[];manifest=load(trial/'manifest.json',{});status=load(trial/'run_status.json',{})
    graph=load(trial/'publisher_audit.json',{});result=load(trial/'result.json',{})
    if not manifest or not status or not result:errors.append('missing trial manifest, run_status or result')
    if status.get('required_process_failed') or status.get('runner_error') or not status.get('result_written'):
        errors.append('required node failed or runner/setup did not complete')
    if status.get('returncode')!=0:errors.append('nonzero roslaunch returncode')
    if not status.get('graph_audited') or not graph.get('verified') or graph.get('errors'):errors.append('graph audit did not pass')
    if not status.get('production_hashes_unchanged'):errors.append('runner production before/after hashes differ')
    publishers=graph.get('publishers',{})
    if publishers.get('/sim/cmd_vel')!=['/base_mppi']:errors.append('MPPI not sole simulation velocity publisher')
    if publishers.get('/mm_controller_node/trajectory_progress')!=['/base_mppi']:errors.append('MPPI not sole planner-progress publisher')
    if publishers.get('/cmd_vel'):errors.append('unremapped real control publisher present')
    if any('/real_base_tracker' in nodes for nodes in publishers.values()):errors.append('Tracker unexpectedly active')
    required={'/base_mppi','/remani_planner_node','/dual_timescale_observer'}
    if not required.issubset(status.get('monitored_node_pids',{})):errors.append('required measured host node PID missing')
    if manifest.get('native_binaries_sha256')!=build_check['native_binaries_sha256']:errors.append('trial native binary hashes differ from build')
    if manifest.get('mppi_library_sha256')!=build_check['mppi_library_sha256']:errors.append('trial MPPI library hash differs from build')
    if manifest.get('mode')!='mppi' or not manifest.get('local_execution_owner'):errors.append('not MPPI-owned execution mode')
    if not manifest.get('progress_contract')=='measured_path_projection':errors.append('planner progress contract mismatch')
    for name,digest in manifest.get('experiment_sources_sha256',{}).items():
        if Path(name).suffix!='.py':continue
        runtime=trial/'runtime'/name
        if not runtime.exists() or sha(runtime)!=digest:
            runtime_core={'mppi_node.py','local_execution.py','simulator.py','native_reference.py','progress.py',
                          'classified_frame.py','dynamic_scenarios.py','dual_timescale_observer.py','search_recording.py'}
            (errors if name in runtime_core else warnings).append('runtime Python snapshot mismatch: '+name)
    try:
        tree=ET.parse(trial/'experiment.launch')
        node=next(n for n in tree.getroot().findall('node') if n.get('name')=='base_mppi')
        library=next(p.get('value') for p in node.findall('param') if p.get('name')=='library')
        if sha(library)!=build_check['mppi_library_sha256']:errors.append('launched MPPI library differs from manifest')
    except (StopIteration,OSError,ET.ParseError) as exc:errors.append('cannot verify launched MPPI library: '+str(exc))
    timeline=lines(trial/'dual_timeline.jsonl');observer=lines(trial/'observer_events.jsonl')
    refs=lines(trial/'consumed_references.jsonl');commits=lines(trial/'reference_events.jsonl')
    phases=event_audit(timeline,lines(trial/'solver_trace.jsonl'));errors+=phases['errors'];warnings+=phases['warnings']
    solves=lines(trial/'failure_trace.jsonl')
    if any(r.get('profile_enabled') or r.get('diagnostic_only') for r in solves):errors.append('expensive profiling/diagnostic pause enabled in online run')
    if not any(r.get('event')=='observer_start' for r in observer):errors.append('observer start record absent')
    if not any(r.get('event')=='observer_stop' for r in observer):warnings.append('observer shutdown record absent; final file tail may be censored')
    capture=io.StringIO()
    try:
        with redirect_stdout(capture):native_ok=native_audit(trial,False)
        native_result=load(trial/'reference_audit.json',{})
        absent_only=(not refs and native_result.get('failures')==['no live native reference consumed by kernel'])
        if absent_only and not native_result.get('required_process_failed'):
            native_ok=None
            warnings.append('native reference consumption contract not applicable: no accepted reference consumed; planning failure remains a retained outcome')
        elif not native_ok or native_result.get('required_process_failed'):errors.append('independent native coefficient/reference audit failed')
    except Exception as exc:
        native_ok=False;native_result={'error':str(exc)};errors.append('native reference audit raised: '+str(exc))
    chain=reference_chain_audit(trial,timeline,observer,refs,commits,report)
    errors+=chain['errors'];warnings+=chain['warnings']
    return dict(trial=trial.name,source=str(trial),valid_experiment=not errors and build_check['pass_integrity'],
                errors=errors,warnings=warnings,navigation_outcome=result.get('reason'),
                outcome_policy='goal/collision/navigation timeout all retained; only setup, crash or contradicted interface invalidates',
                phase_integrity=phases,reference_chain_integrity=chain,native_coefficients_pass=native_ok,
                native_reference_audit=native_result,advisory_freeze=advisory_freeze_audit(observer,refs),
                solver_outcomes=dict(Counter(r.get('reason','unknown') for r in solves)),
                inputs_sha256={name:sha(trial/name) for name in ('manifest.json','run_status.json','publisher_audit.json',
                    'dual_timeline.jsonl','observer_events.jsonl','solver_trace.jsonl','failure_trace.jsonl',
                    'trajectories.json','consumed_references.jsonl','reference_events.jsonl','mppi_diagnostics.json') if (trial/name).exists()})


def audit_batch(batch,out,reports=None,development=False):
    batch=Path(batch).resolve();plan=load(batch/'experiment_plan.json',{})
    if not development and not load(batch/'batch_complete.json',{}).get('complete'):
        raise RuntimeError('Measured batch not complete; refusing full audit during live timing')
    build=Path(plan.get('build',HERE/'builds/dual_timeline_v12_01')).resolve()
    integrity=build_integrity(build,plan)
    trials=sorted(p.parent for p in batch.glob('*/run_status.json'))
    if not trials:raise RuntimeError('No completed trial status records')
    results=[]
    for trial in trials:
        report=Path(reports)/trial.name/'summary.json' if reports else None
        if report is not None and not report.exists():report=None
        result=audit_trial(trial,integrity,report);results.append(result)
        print('AUDITED',trial.name,result['valid_experiment'],result['navigation_outcome'],flush=True)
    expected=len(plan.get('scenes',[]))*len(plan.get('delays_s',[0]))*int(plan.get('repeats',1))
    complete_count=expected==len(trials)
    summary=dict(scope='offline evidence/interface integrity, not performance or physical safety acceptance',
                 batch=str(batch),development=development,build_integrity=integrity,
                 expected_trials=expected,observed_trials=len(trials),expected_count_present=complete_count,
                 all_valid=integrity['pass_integrity'] and complete_count and all(r['valid_experiment'] for r in results),
                 outcomes=dict(Counter(r['navigation_outcome'] for r in results)),trials=results,
                 auditor_sha256=sha(Path(__file__)))
    out=Path(out);out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print('AUDIT_COMPLETE',out,'all_valid',summary['all_valid'],flush=True)
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch',type=Path,required=True);parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--reports',type=Path);parser.add_argument('--development',action='store_true')
    args=parser.parse_args();result=audit_batch(args.batch,args.out,args.reports,args.development)
    raise SystemExit(0 if result['all_valid'] else 1)
