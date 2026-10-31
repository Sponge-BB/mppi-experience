#!/usr/bin/env python3
"""Offline four-job actual-fixed admission report; defer is never success.

No ROS, compiler, model, plotting or trial reads on import. The main report
requires an attested directed signed-state prerequisite and a completed 2x2
batch, and checks recorded ROS PIDs/port before reading the large timelines.
"""
import argparse
from collections import Counter,defaultdict
from datetime import datetime,timezone
import json
from pathlib import Path

from report_domain_recovery import assert_plot_quiet,digest,read_json,read_records
from report_state_contract import (analyze_cycles,backend_scope_evidence,conjunction,
    cpp_identity,finite,integer,match_observer,reference_key,snapshot_evidence,unsigned_identity)
from report_guide_applicability import command_class,sampled_payload
from state_contract_records import record_with_validity

EPS=1e-8
SCHEMA='boundary_admission_v2_actual_fixed'
LIMITATIONS=[
    '两个seed的静态闭环描述，不是同一完整输入的因果配对或统计性能证明；请求/前端历史可以不同。',
    'defer是已耗前端时间后的失败请求，不是求解成功；仅避免optimizer之后的工作，不扣除延期、重试或前端等待。',
    'field端点使用同一冻结投影距离场/磁盘并集，world导航结果来自仿真；二者不是相同物理口径或安全认证。',
    '沿用native strict gap < -1e-8拒绝规则；[-1e-8,0)是native_tolerance，不称先天不可交付。',
    '端到端口径为静态scene obstacle appearance(result.motion_anchor+4)→首条其后发布新reference实际online_step入口；不是感知发布精确时刻。',
    '无更新时latency=null并保留删失下界；同步诊断I/O影响时序，所有等待均保留。',
]


def single(rows,event):
    matches=[r for r in rows if r.get('event')==event]
    return matches[0] if len(matches)==1 else None


def category(value):
    if not isinstance(value,dict) or not finite(value.get('physical')) or not finite(value.get('gap')):return None
    if value['physical']<=0:return 'physical_collision'
    if value['gap'] < -EPS:return 'margin_only'
    return 'native_tolerance' if value['gap']<0 else 'full_margin'


def precheck_evidence(trial,row,odom):
    if row is None:return None
    copy=dict(row,backend_frame=row.get('precheck_frame'),fixed_yaw=row.get('actual_fixed_yaw'),fixed_static=row.get('actual_fixed_static'))
    frame=snapshot_evidence(trial,copy,row)
    actual=frame.get('independent_fixed_static');raw=frame.get('independent_raw_static')
    observed=match_observer(cpp_identity(row),odom)
    return dict(raw_observer_match=observed,frame=frame,independent_actual_fixed_static=actual,
        independent_raw_static=raw,independent_classification=category(actual),
        classification_agrees=category(actual)==row.get('classification') if category(actual) is not None else None,
        native_tolerance_recorded=row.get('same_native_gap_tolerance'),
        tolerance_agrees=row.get('same_native_gap_tolerance')==EPS if finite(row.get('same_native_gap_tolerance')) else None,
        same_frozen_field_raw_and_actual=conjunction((frame.get('payload_hash_verified'),frame.get('raw_online_agreement'),frame.get('fixed_online_agreement'))),
        exact_evidence=conjunction((observed['exact_message_observed'],frame.get('payload_hash_verified'),
            frame.get('raw_online_agreement'),frame.get('fixed_online_agreement'),
            category(actual)==row.get('classification') if category(actual) is not None else None)))


def request_groups(native):
    groups=defaultdict(list)
    for row in native or []:
        pid,request=row.get('pid'),row.get('request_id')
        if integer(pid) and pid>1 and integer(request) and request>0:groups[(pid,request)].append(row)
    return dict(sorted(groups.items()))


def analyze_requests(trial,native,odom,failures):
    groups=request_groups(native)
    all_backends=[r for rows in groups.values() for r in rows if r.get('event')=='state_contract_backend_fixed']
    hashes=Counter(unsigned_identity(r.get('input_hash')) for r in all_backends)
    result=[]
    for (pid,request),rows in groups.items():
        pre=single(rows,'boundary_admission_precheck');end=single(rows,'state_contract_request_end')
        selected=single(rows,'state_contract_selected_boundary');deferred=pre.get('deferred') if pre else None
        forbidden=[r.get('event') for r in rows if r.get('event') in (
            'state_contract_backend_call','state_contract_backend_fixed','state_contract_solver_return',
            'state_contract_reference_publish','reference_publish','state_contract_backend_scope_link') or
            r.get('event')=='stage_begin' and r.get('stage')=='optimizer']
        marker=single(rows,'boundary_admission_manager_deferred')
        defer_valid=conjunction((end.get('success') is False if end and isinstance(end.get('success'),bool) else None,
            not forbidden,marker.get('optimizer_started') is False if marker else None,
            marker.get('manager_success') is False if marker else None)) if deferred is True else None
        proof=precheck_evidence(trial,pre,odom);backends=[]
        for backend in (r for r in rows if r.get('event')=='state_contract_backend_fixed'):
            input_hash=unsigned_identity(backend.get('input_hash'));scope=backend_scope_evidence(rows,backend)
            matching=[r for r in failures or [] if r.get('kind')=='solve' and r.get('label')=='normal' and unsigned_identity(r.get('input_hash'))==input_hash] if input_hash is not None else []
            unique=scope['complete'] is True and input_hash is not None and hashes[input_hash]==1 and len(matching)==1
            frame=snapshot_evidence(trial,backend,selected)
            backends.append(dict(input_hash=input_hash,backend_scope=scope,exact_failure_input_link=unique if failures is not None else None,
                failure_reason=matching[0].get('reason') if unique else None,
                recorded_solver_success=matching[0].get('success') if unique else None,
                fixed_field_classification=category(frame.get('independent_fixed_static')),frozen_frame=frame,
                same_precheck_field=(frame.get('sha256')==proof['frame'].get('sha256')) if proof and frame.get('sha256') and proof['frame'].get('sha256') else None))
        result.append(dict(pid=pid,request_id=request,request_begin_count=sum(r.get('event')=='request_begin' for r in rows),
            selected_capture_sim=selected.get('capture_sim') if selected else None,
            request_end_capture_sim=end.get('capture_sim') if end else None,
            request_success=end.get('success') if end else None,precheck_count=sum(r.get('event')=='boundary_admission_precheck' for r in rows),
            eligible=pre.get('eligible') if pre else None,would_defer=pre.get('would_defer') if pre else None,deferred=deferred,
            precheck_reason=pre.get('reason') if pre else None,precheck_schema=pre.get('boundary_admission_schema') if pre else None,
            reported_classification=pre.get('classification') if pre else None,precheck_evidence=proof,
            actual_fixed_heading_known=pre.get('actual_fixed_heading_known') if pre else None,
            exact_processed_measurement=pre.get('exact_processed_measurement') if pre else None,
            frontend_wall_ms=pre.get('frontend_wall_ms') if pre else None,frontend_ros_ms=pre.get('frontend_ros_ms') if pre else None,
            defer_contract_valid=defer_valid,defer_forbidden_children=forbidden if deferred is True else None,
            backend_records=backends,reference_publish_count=sum(r.get('event')=='state_contract_reference_publish' for r in rows),
            observed_next_request_interval_s=None,retry_policy_ros_s=.75,
            retry_interval_semantics='request-end capture to next same-pid selected capture; includes flush/timer/ESDF/frontend scheduling; never imputed as 0.75'))
    for index,row in enumerate(result):
        following=next((r for r in result[index+1:] if r['pid']==row['pid']),None)
        if following and finite(row['request_end_capture_sim']) and finite(following['selected_capture_sim']) and following['selected_capture_sim']>=row['request_end_capture_sim']:
            row['observed_next_request_interval_s']=following['selected_capture_sim']-row['request_end_capture_sim']
    known=bool(result)
    backends=[b for r in result for b in r['backend_records']]
    return dict(present=native is not None,total_requests=len(result) if native is not None else None,
        prechecks=sum(r['precheck_count']==1 for r in result) if known else None,
        eligible=sum(r['eligible'] is True for r in result) if known else None,
        would_defer=sum(r['would_defer'] is True for r in result) if known else None,
        deferred=sum(r['deferred'] is True for r in result) if known else None,
        defer_contract_violations=sum(r['deferred'] is True and r['defer_contract_valid'] is not True for r in result) if known else None,
        missing_or_ambiguous_prechecks=sum(r['precheck_count']!=1 for r in result) if known else None,
        backend_requests=sum(bool(r['backend_records']) for r in result) if known else None,backend_solves=len(backends) if known else None,
        successful_requests=sum(r['request_success'] is True and r['deferred'] is not True for r in result) if known else None,
        request_success_flags_true=sum(r['request_success'] is True for r in result) if known else None,
        exact_precheck_evidence=sum(r['precheck_evidence'] and r['precheck_evidence']['exact_evidence'] is True for r in result) if known else None,
        backend_failures_by_fixed_field_class={c:dict(Counter(b['failure_reason'] for b in backends if b['fixed_field_classification']==c and b['recorded_solver_success'] is False and b['exact_failure_input_link'] is True)) for c in ('margin_only','full_margin','native_tolerance','physical_collision',None)},
        records=result)


def end_to_end(outcome,scene,native,cycles,odom,consumed,commits,observer):
    anchor=outcome.get('motion_anchor');appearance=anchor+4. if scene=='static' and finite(anchor) else None
    answer=dict(appearance_elapsed_s=appearance,appearance_ros_time=appearance+1000. if appearance is not None else None,
        latency_s=None,right_censored=None,censor_lower_bound_s=None,request_id=None,cycle_id=None,
        definition='static scene obstacle appearance=result.motion_anchor+4 -> first postappearance newly published exact reference actual online_step entry; ALL defer/retry/frontend wait retained',
        perception_publication_time_claimed=False)
    if appearance is None:return answer
    pubs=[r for r in native or [] if r.get('event')=='state_contract_reference_publish' and finite(r.get('capture_sim')) and r['capture_sim']>=appearance+1000.]
    counts=Counter(r.get('transaction_fnv64') for r in native or [] if r.get('event')=='state_contract_reference_publish')
    analyzed=analyze_cycles(cycles,odom,consumed,commits,observer)['rows'];original={r.get('cycle_id'):r for r in cycles or []}
    candidates=[]
    for row in analyzed:
        source=original.get(row['cycle_id'],{});entry=row.get('online_step_entry_ros_time')
        if row.get('online_step_invoked') is not True or source.get('first_kernel_cycle_for_reference') is not True or row['unique_cycle_id'] is not True or not finite(entry):continue
        if row['solve']['exact_message_observed'] is not True or row['solve']['state_equals_identity'] is not True or row['reference']['complete_exact_reference_chain'] is not True:continue
        refs=[r for r in consumed or [] if reference_key(r,True)==reference_key(source)]
        if len(refs)!=1 or sampled_payload(refs[0])[0]['payload_sha256_verified'] is not True:continue
        tx=row['reference'].get('transaction_fnv64');matching=[p for p in pubs if p.get('transaction_fnv64')==tx]
        if counts[tx]==1 and len(matching)==1 and entry>=matching[0]['capture_sim']:
            candidates.append((entry,row,matching[0]))
    if candidates:
        entry,row,pub=min(candidates,key=lambda item:item[0])
        answer.update(latency_s=entry-(appearance+1000.),right_censored=False,request_id=pub.get('request_id'),cycle_id=row['cycle_id'],
            online_step_entry_ros_time=entry,new_reference_publish_capture_sim=pub['capture_sim'],kernel_code=row['kernel_code'],
            transaction_sha256=row['reference']['transaction_sha256'])
    else:
        answer['right_censored']=True
        elapsed=outcome.get('elapsed')
        if finite(elapsed) and elapsed>=appearance:answer['censor_lower_bound_s']=elapsed-appearance
    return answer


def trial_report(trial):
    trial=Path(trial);docs={name:read_json(trial/name) for name in ('manifest.json','run_status.json','trial_audit.json','result.json')}
    manifest=docs['manifest.json'];settings=manifest.get('coupling_experiment') or {};status=docs['run_status.json'];outcome=docs['result.json']
    if status.get('returncode') is None or status.get('result_written') is not True:raise ValueError('Incomplete trial: '+str(trial))
    if not isinstance(settings.get('boundary_admission'),bool):raise ValueError('Missing explicit boundary treatment')
    inputs={};hashes={name:digest(trial/name) for name in docs}
    for name in ('dual_timeline.jsonl','state_mppi_cycles.jsonl','state_odom_events.jsonl','consumed_references.jsonl','reference_events.jsonl','observer_events.jsonl','failure_trace.jsonl'):
        inputs[name],path=read_records(trial,name)
        if path is not None:hashes[path.name]=digest(path)
    cycles=inputs['state_mppi_cycles.jsonl'];requests=analyze_requests(trial,inputs['dual_timeline.jsonl'],inputs['state_odom_events.jsonl'],inputs['failure_trace.jsonl'])
    latency=end_to_end(outcome,manifest.get('scene'),*(inputs[name] for name in ('dual_timeline.jsonl','state_mppi_cycles.jsonl','state_odom_events.jsonl','consumed_references.jsonl','reference_events.jsonl','observer_events.jsonl')))
    elapsed=outcome.get('elapsed');goal=outcome.get('goal_sent')
    return dict(trial=trial.name,source=str(trial),seed=settings.get('seed'),admission=settings['boundary_admission'],
        run_status=status,audit=docs['trial_audit.json'],outcome=outcome,runtime_sha256=manifest.get('experiment_sources_sha256'),
        build=manifest.get('build'),build_manifest_sha256=manifest.get('build_manifest_sha256'),settings=settings,input_sha256=hashes,
        requests=requests,end_to_end=latency,navigation_elapsed_s=elapsed,
        navigation_since_goal_s=elapsed-goal if finite(elapsed) and finite(goal) else None,
        zero_command_reasons=dict(Counter(r.get('reason','<missing>') for r in cycles or [] if command_class(r) is True)) if cycles is not None else None,
        scope_checks=dict(static_scene=manifest.get('scene')=='static',coupled=settings.get('group')=='coupled',
            signed_policy_fixed=settings.get('signed_low_speed') is True,domain_recovery_fixed=settings.get('domain_recovery') is True,
            static_declaration=settings.get('boundary_admission_static_scene') is True))


def direction_proof(plan):
    path=plan.get('direction_proof');expected=plan.get('direction_proof_sha256')
    if not isinstance(path,str) or not path or not isinstance(expected,str):raise ValueError('Missing directed prerequisite identity')
    path=Path(path)
    if digest(path)!=expected:raise ValueError('Directed prerequisite hash changed')
    proof=read_json(path)
    if proof.get('schema')!='directed_low_speed_verdict_v1' or any(proof.get(k) is not True for k in ('verified_signed_negative_native_chain','same_fixture','same_runtime')):
        raise ValueError('Directed signed-state verification did not pass')
    return dict(path=str(path),sha256=expected,schema=proof['schema'],passed=True)


def build_report(root):
    root=Path(root).resolve();plan=read_json(root/'plan.json');complete=read_json(root/'complete.json');proof=direction_proof(plan)
    jobs=complete.get('completed')
    if complete.get('complete') is not True or not isinstance(jobs,list) or len(jobs)!=4:raise ValueError('Four completed jobs required')
    names=[job.get('trial') for job in jobs]
    if any(not isinstance(n,str) or Path(n).name!=n or n in ('.','..') for n in names) or len(set(names))!=4:raise ValueError('Invalid trial names')
    assert_plot_quiet(dict(trials=[dict(run_status=read_json(root/name/'run_status.json')) for name in names]))
    trials=sorted([trial_report(root/name) for name in names],key=lambda t:(t['seed'],t['admission']))
    if {(t['seed'],t['admission']) for t in trials}!={(11,False),(11,True),(12,False),(12,True)}:raise ValueError('Need OFF/ON for seeds 11 and 12')
    checks={}
    for seed in (11,12):
        pair=[t for t in trials if t['seed']==seed];a,b=pair
        settings=[{k:v for k,v in t['settings'].items() if k!='boundary_admission'} for t in pair]
        checks[str(seed)]=dict(runtime_full_hash_equal=a['runtime_sha256']==b['runtime_sha256'] if a['runtime_sha256'] and b['runtime_sha256'] else None,
            build_identity_equal=a['build']==b['build'] and a['build_manifest_sha256']==b['build_manifest_sha256'] if a['build'] and a['build_manifest_sha256'] and b['build'] and b['build_manifest_sha256'] else None,
            treatment_only_recorded_flag=settings[0]==settings[1] if all(settings) else None,
            scope_checks_passed=all(v is True for t in pair for v in t['scope_checks'].values()))
    return record_with_validity(dict(schema='boundary_admission_report_v1',created=datetime.now(timezone.utc).isoformat(),root=str(root),
        prerequisite=proof,plan=plan,complete=complete,pair_checks=checks,trials=trials,limitations=LIMITATIONS,
        paired_complete_native_input=False,statistical_performance_claim=False,
        conclusion='逐seed描述实际fixed预检、延期、求解与含等待的新guide接入；不把defer算成功，不以不同闭环请求宣称因果改善。'))


def show(value):return '未知' if value is None else str(value)


def report_text(report):
    lines=['Actual-fixed边界准入：两seed静态A/B（描述性）',report['conclusion']]
    for trial in report['trials']:
        r=trial['requests'];lat=trial['end_to_end']
        lines+=['','seed=%s %s: %s'%(trial['seed'],'ON' if trial['admission'] else 'OFF',trial['trial']),
            '全部请求=%s；eligible/would_defer/deferred=%s/%s/%s；backend求解=%s；成功请求=%s；defer合同违规=%s'%tuple(show(v) for v in (r['total_requests'],r['eligible'],r['would_defer'],r['deferred'],r['backend_solves'],r['successful_requests'],r['defer_contract_violations'])),
            'scene appearance→新ref实际kernel entry=%s秒；右删失=%s；删失下界=%s（等待不扣除）'%(show(lat['latency_s']),show(lat['right_censored']),show(lat['censor_lower_bound_s'])),
            'world结果=%s；导航elapsed=%s；保守碰撞=%s；零命令reason=%s'%(show(trial['outcome'].get('reason')),show(trial['navigation_elapsed_s']),show(trial['outcome'].get('collision_conservative_projection')),json.dumps(trial['zero_command_reasons'],ensure_ascii=False))]
    return '\n'.join(lines+['','配对检查='+json.dumps(report['pair_checks'],ensure_ascii=False),'','解释边界：']+LIMITATIONS)+'\n'


def plot_report(report,path):
    assert_plot_quiet(report)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(10.,4.))
    labels=['%s %s'%(t['seed'],'ON' if t['admission'] else 'OFF') for t in report['trials']]
    for offset,key,label in ((-.24,'backend_solves','Backend solves'),(0.,'deferred','Deferred requests'),(.24,'successful_requests','Successful requests')):
        points=[(i,t['requests'][key]) for i,t in enumerate(report['trials']) if t['requests'][key] is not None]
        axes[0].bar([i+offset for i,v in points],[v for i,v in points],.23,label=label)
    for i,t in enumerate(report['trials']):
        latency=t['end_to_end'];value=latency['latency_s']
        if value is not None:axes[1].scatter(i,value)
        else:axes[1].text(i,.08,'censored' if latency['right_censored'] else 'unknown',rotation=90,ha='center',transform=axes[1].get_xaxis_transform())
    axes[0].legend(fontsize=7);axes[0].set_ylabel('Recorded count (defer is not success)')
    axes[1].set_ylabel('Appearance → new guide entry (s), includes waits')
    for ax in axes:ax.set_xticks(range(4));ax.set_xticklabels(labels);ax.grid(axis='y',alpha=.2)
    fig.suptitle('Static descriptive A/B — projected field and world outcomes are different scopes',fontsize=10)
    fig.tight_layout(rect=(0,0,1,.94));fig.savefig(path,dpi=130);plt.close(fig)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--root',type=Path,required=True);parser.add_argument('--out',type=Path,required=True);parser.add_argument('--plot',action='store_true')
    args=parser.parse_args(argv);root=args.root.resolve();out=args.out.resolve()
    if out.exists() or out==root or out in root.parents:parser.error('Output must be fresh and not an input/ancestor')
    report=build_report(root);out.mkdir(parents=True,exist_ok=False)
    (out/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    (out/'实验结论.txt').write_text(report_text(report),encoding='utf-8')
    if args.plot:plot_report(report,out/'admission_counts_and_update.png')
    return 0


if __name__=='__main__':raise SystemExit(main())
