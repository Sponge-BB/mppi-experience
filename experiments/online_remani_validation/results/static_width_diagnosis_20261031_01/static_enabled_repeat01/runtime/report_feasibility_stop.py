#!/usr/bin/env python3
"""Post-run audit of two NEW paused OFF/ON/OFF feasibility-stop captures.

No solver, ROS or plotting is run at import. Do not run while timed jobs are
active. Candidate geometry and wheel reconstruction are posterior diagnostics,
never measured trajectory-delivery or publication results.
"""
import argparse
import gzip
import json
from pathlib import Path

from audit_backend_kinematics import analyze_candidate, parameters as wheel_parameters
from audit_delivery_fullcurve import frozen_frame
from audit_delivery_iterates import analyze_phase, canonical_records, conjunction, finite
from report_backend_feasibility import scene_phase
from report_domain_recovery import assert_plot_quiet, digest, read_json, read_records


HERE=Path(__file__).resolve().parent
JOBS=((11,2),(12,2))
PHASES=('delivery_nominal_060','delivery_stop_on_060','delivery_repeat_060')
CONTEXT_FIELDS=('effective_initial_states','effective_final_states','raw_seq','esdf_seq',
                'frame_now','frame_sequence','prediction_hash','sim','wheel_limit')
LIMITATIONS=[
    '两个NEW同进程暂停捕获，OFF/ON/OFF各0.6秒；不是历史157ms请求的精确重放，不作跨批速度因果比较。',
    '只改变总代价相对改善停止是否允许；没有新增恢复目标、扩大预算、改变初值/罚权/轮限制/安全余量。',
    '三臂共同新增最终返回截止时间防线；这批OFF并非旧构建逐字节原版，不能据此估计共同防线的单独收益。',
    '门内原生/投影扫描计入原截止时间；feasibility_decision_elapsed_s包含最终判定，不包括之后的canonical候选后验重建验收。',
    'failure_trace.lbfgs_wall_ms仅是L-BFGS阶段；failure_trace.wall_ms含预算外审计，不当作求解或交付耗时。',
    '所有候选的原生与冻结投影20ms结果均为求解后的审计；首次双过的迭代时间不是可实时交付时间。',
    '仅最后一个canonical候选补完整左右轮10ms/边界诊断；其他候选的原生结论来自同进程原生验收，不能冒称全部做过独立轮扫描。',
    '同首x/f/g与冻结上下文是诊断证据，不单独证明完整native地图内存及全部隐状态逐字节冻结。',
    '有诊断前normal规划不代表捕获成功；没有capture、没有触发代价停止、缺失/不完整记录全部保留，不补成成功。',
    '诊断候选不发布；本轮不证明在线交付、MPPI协同、动态安全或实车表现改善，也不作总体成功率统计。',
    '原生检查跳过前0.05秒，完整投影包含k0；抽样通过不证明连续时间或未来FSM live接收通过。',
]


def unique(rows,field,value):
    found=[r for r in rows or [] if r.get(field)==value]
    return found[0] if len(found)==1 else None


def integer(value):
    return isinstance(value,int) and not isinstance(value,bool)


def optional_json(trial,name,hashes):
    path=next((p for p in (trial/name,trial/(name+'.gz')) if p.is_file()),None)
    if path is None:return None
    hashes[path.name]=digest(path)
    opener=gzip.open if path.suffix=='.gz' else open
    with opener(path,'rt',encoding='utf-8') as stream:return json.load(stream)


def gate_evidence(rows,label):
    matches=[r for r in rows or [] if r.get('kind')=='feasibility_stop_summary' and r.get('phase')==label]
    gate=matches[0] if len(matches)==1 else None
    if gate is None:
        return dict(present=False,count=len(matches),evidence_valid=False,activated=None,
                    reason='missing or duplicate gate summary',record=None)
    expected=label==PHASES[1]
    counters=('checks','blocked_count','allowed_count','native_pass_count',
              'projected_pass_count','budget_expired_count')
    counts_valid=all(integer(gate.get(k)) and gate[k]>=0 for k in counters)
    fatal_known=isinstance(gate.get('fatal'),bool)
    if counts_valid and fatal_known:
        explained=gate['blocked_count']+gate['allowed_count']
        counts_valid=(explained<=gate['checks'] if gate['fatal'] else explained==gate['checks'])
        counts_valid=counts_valid and all(gate[k]<=gate['checks'] for k in counters[1:])
    times=('native_wall_s','projected_wall_s','gate_wall_s','feasibility_decision_elapsed_s')
    times_valid=all(finite(gate.get(k)) and gate[k]>=0. for k in times)
    variant='cost_stop_veto' if expected else 'original_cost_stop'
    valid=(gate.get('enabled') is expected and gate.get('original_budget_s')==.6 and
           gate.get('effective_variant')==variant and counts_valid and fatal_known and times_valid)
    if not expected:
        valid=valid and all(gate.get(k)==0 for k in counters)
    return dict(present=True,count=1,evidence_valid=valid,
        activated=gate['checks']>0 if counts_valid else None,
        veto_count=gate.get('blocked_count'),allowed_count=gate.get('allowed_count'),
        budget_expired_count=gate.get('budget_expired_count'),fatal=gate.get('fatal'),
        gate_wall_s=gate.get('gate_wall_s'),decision_elapsed_s=gate.get('feasibility_decision_elapsed_s'),
        record=gate,not_an_online_acceptance=True)


def phase_report(trial,label,rows,failures,capture,frame,proof,params):
    audit=analyze_phase(trial,label,rows,capture,frame,proof)
    summary=audit['solve_summary']
    native=[r for r in failures or [] if r.get('kind')=='solve' and r.get('label')==label]
    failure=native[0] if len(native)==1 else None
    returned=audit.get('recursive_return') or {}
    gate=gate_evidence(rows,label)
    failure_valid=(failure is not None and str(failure.get('input_hash'))==str(capture.get('input_hash')) and
        failure.get('budget_s')==summary.get('budget_s')==.6 and
        failure.get('success') is summary.get('native_return_success') and
        returned.get('success') is summary.get('native_return_success') and
        failure.get('reason')==summary.get('reason') and
        finite(failure.get('lbfgs_wall_ms')) and failure['lbfgs_wall_ms']>=0.)
    canonical=[r for r in rows if r.get('kind')=='canonical_candidate' and r.get('phase')==label]
    last=None
    if canonical and audit['candidates']:
        last=analyze_candidate(canonical[-1],capture,audit['candidates'][-1],params)
    valid=conjunction([audit['phase_evidence_valid'],failure_valid,gate['evidence_valid'],
                       last.get('evidence_valid') if last else None])
    return dict(phase=label,treatment='ON' if label==PHASES[1] else 'OFF',budget_s=.6,
        evidence_valid=valid,failure_record_count=len(native),failure_record_agrees=failure_valid,
        lbfgs_wall_ms=failure.get('lbfgs_wall_ms') if failure else None,
        decision_elapsed_s=gate['decision_elapsed_s'] if gate.get('present') else None,
        native_return_success=summary.get('native_return_success'),solver_reason=summary.get('reason'),
        gate=gate,iterate_audit=audit,last_canonical_wheel_audit=last,
        canonical_counts=audit['counts'],post_solver_audit_wall_s=audit['native_audit_cost'].get('post_solver_total_wall_s'),
        not_actual_delivery=True)


def trial_report(trial,job):
    hashes={};manifest=read_json(trial/'manifest.json');status=read_json(trial/'run_status.json')
    for name in ('manifest.json','run_status.json'):hashes[name]=digest(trial/name)
    rows,replay=canonical_records(trial)
    failures,failure_path=read_records(trial,'failure_trace.jsonl')
    timeline,timeline_path=read_records(trial,'dual_timeline.jsonl')
    for path in (replay,failure_path,timeline_path):
        if path is not None:hashes[path.name]=digest(path)
    basic=dict(trial=trial.name,seed=job['seed'],ordinal=job['ordinal'],run_status=status,
               captured=False,evidence_valid=False,source=str(trial),input_sha256=hashes,
               phases=[],diagnostic_candidates_published=None,navigation_metrics_valid=False)
    captures=[r for r in rows or [] if r.get('kind')=='capture']
    if len(captures)!=1:
        return dict(basic,capture_count=len(captures),reason='no or ambiguous capture; normal preplanning does not count',
                    gate_summaries=[r for r in rows or [] if r.get('kind')=='feasibility_stop_summary'])
    capture=captures[0]
    if any(r.get('schema')!='delivery_replay_v1' for r in rows):raise ValueError('Unknown replay schema')
    complete=unique(rows,'kind','complete') or {}
    done_path=trial/'diagnostic_complete.json';done=read_json(done_path) if done_path.is_file() else {}
    if done_path.is_file():hashes[done_path.name]=digest(done_path)
    frame,proof=frozen_frame(trial/'delivery_diagnostics',capture['frame'])
    if len(frame['obstacles']):raise ValueError('Static diagnostic has moving obstacles')
    params,param_proof=wheel_parameters(trial)
    phases=[phase_report(trial,label,rows,failures,capture,frame,proof,params) for label in PHASES]
    summaries=[p['iterate_audit']['solve_summary'] for p in phases]
    signatures=[p['iterate_audit']['first_signature'] for p in phases]
    same_first=(all(s.get('verified') is True for s in signatures) and
        len({(s.get('cost'),s.get('x_hash'),s.get('gradient_hash')) for s in signatures})==1)
    same_context=(all(bool(s) for s in summaries) and all(all(k in s and s[k]==summaries[0].get(k)
        for k in CONTEXT_FIELDS) for s in summaries) and all(s.get('sim')==s.get('sim_end') for s in summaries))
    observed=[r for r in rows if r.get('kind')=='native_dynamic_observation']
    observed_valid=(bool(observed) and all(r.get('matches_entry') is True for r in observed) and
        len({(r.get('query_time_nsec'),r.get('content_hash'),r.get('overlay_enabled')) for r in observed})==1)
    flag_keys=('frozen_stable','native_dynamic_observed_stable','same_first_fg','audits_complete')
    completed=all(complete.get(k) is True and done.get(k) is True for k in flag_keys)
    no_publish=(complete.get('diagnostic_candidates_published') is False and
        done.get('diagnostic_candidates_published') is False and
        timeline is not None and not any(r.get('request_id')==capture.get('request_id') and
            r.get('event') in ('reference_publish','state_contract_reference_publish') for r in timeline))
    settings=manifest.get('coupling_experiment') or {}
    request_begins=[r for r in timeline or [] if r.get('event')=='request_begin' and
                    r.get('request_id')==capture.get('request_id')]
    scope=dict(job_capture_recorded=job.get('captured') is True,
        seed=settings.get('seed')==job['seed'],ordinal=settings.get('refresh_ordinal')==job['ordinal'],
        actual_capture_ordinal=capture.get('refresh_ordinal')==job['ordinal'],
        paused=settings.get('diagnostic_pause') is True,
        phase_order=settings.get('diagnostic_phases')==list(PHASES),
        budgets=settings.get('diagnostic_budgets_s')==[.6,.6,.6],
        treatment_order=settings.get('diagnostic_treatment_order')==['OFF','ON','OFF'],
        weights_zero=settings.get('execution_weight')==0. and settings.get('handover_weight')==0.,
        seed_fix=settings.get('seed_parameterization_fix') is True,
        constraints_unchanged=settings.get('constraints_unchanged') is True,
        objective_unchanged=settings.get('objective_unchanged') is True,
        initialization_unchanged=settings.get('initialization_unchanged') is True,
        paused_before_nominal=capture.get('all_passes_paused_before_nominal') is True,
        capture_constraints_not_relaxed=capture.get('constraints_relaxed') is False)
    scope['actual_replanning_request']=len(request_begins)==1 and request_begins[0].get('fsm')==3
    status_valid=(status.get('diagnostic_complete') is True and status.get('graph_audited') is True and
        status.get('production_hashes_unchanged') is True and status.get('build_manifest_unchanged') is True and
        status.get('required_process_failed') is False and status.get('runner_error') is None)
    outcome=optional_json(trial,'result.json',hashes);events=optional_json(trial,'events.json',hashes)
    runtime=manifest.get('experiment_sources_sha256') or {}
    runtime_python={n:dict(expected=h,observed=digest(trial/'runtime'/n) if (trial/'runtime'/n).is_file() else None)
                    for n,h in runtime.items() if n.endswith('.py')}
    runtime_valid=bool(runtime_python) and all(v['expected']==v['observed'] for v in runtime_python.values())
    valid=all((same_first,same_context,observed_valid,completed,no_publish,status_valid,runtime_valid,
               all(scope.values()),all(p['evidence_valid'] is True for p in phases)))
    return dict(basic,captured=True,evidence_valid=valid,capture_count=1,capture=capture,complete=complete,
        independently_same_first_fg=same_first,same_frozen_recorded_context=same_context,
        native_dynamic_observations_valid=observed_valid,native_dynamic_observations=observed,
        completion_flags_valid=completed,no_captured_request_publication=no_publish,
        diagnostic_candidates_published=False if no_publish else None,
        status_evidence_valid=status_valid,scope_checks=scope,snapshot=proof,
        wheel_parameters=param_proof,phases=phases,runtime_python=runtime_python,
        runtime_python_verified=runtime_valid,scene_phase=scene_phase(capture,outcome,events),
        repeat_check=dict(same_input=same_first and same_context,
            off_solver_time_difference_ms=(phases[2]['lbfgs_wall_ms']-phases[0]['lbfgs_wall_ms'])
                if finite(phases[2]['lbfgs_wall_ms']) and finite(phases[0]['lbfgs_wall_ms']) else None,
            not_independent_samples=True))


def build_report(root):
    root=Path(root).resolve();plan=read_json(root/'plan.json');done=read_json(root/'complete.json')
    jobs=done.get('completed')
    if done.get('complete') is not True or not isinstance(jobs,list) or len(jobs)!=2:
        raise ValueError('Require both retained completed jobs, including no-capture jobs')
    if tuple((j.get('seed'),j.get('ordinal')) for j in jobs)!=JOBS or any(j.get('kind')!='paused' for j in jobs):
        raise ValueError('Prespecified job order differs')
    if tuple((j.get('seed'),j.get('ordinal')) for j in plan.get('diagnostic_jobs',[]))!=JOBS:
        raise ValueError('Plan job order differs')
    if plan.get('phases')!=list(PHASES) or plan.get('budgets_s')!=[.6,.6,.6]:
        raise ValueError('Plan phase/budget contract differs')
    expected=['static_stop_seed%d_refresh%d'%pair for pair in JOBS]
    if [j.get('trial') for j in jobs]!=expected:raise ValueError('Unsafe/unexpected trial names')
    statuses=[dict(run_status=read_json(root/j['trial']/'run_status.json')) for j in jobs]
    assert_plot_quiet(dict(trials=statuses))
    build=Path(plan['build']);manifest_path=build/'build_manifest.json'
    if digest(manifest_path)!=plan.get('build_manifest_sha256'):raise ValueError('Build manifest changed')
    manifest=read_json(manifest_path)
    source_ok=bool(manifest.get('isolated_source_sha256')) and all(
        (build/'source'/n).is_file() and digest(build/'source'/n)==h for n,h in manifest['isolated_source_sha256'].items())
    binary_ok=bool(manifest.get('native_binaries_sha256')) and all(
        (build/'build'/n).is_file() and digest(build/'build'/n)==h for n,h in manifest['native_binaries_sha256'].items())
    trials=[trial_report(root/j['trial'],j) for j in jobs]
    for trial in trials:
        for name,h in trial['input_sha256'].items():
            if digest(Path(trial['source'])/name)!=h:raise RuntimeError('Input changed during posterior audit')
    return dict(schema='feasibility_stop_report_v1',source=str(root),
        reporter_sha256=digest(Path(__file__)),plan_sha256=digest(root/'plan.json'),
        complete_sha256=digest(root/'complete.json'),build=str(build),
        build_manifest_sha256=digest(manifest_path),build_sources_verified=source_ok,build_binaries_verified=binary_ok,
        evidence_valid=source_ok and binary_ok and all(t['evidence_valid'] for t in trials),
        trials=trials,limitations=LIMITATIONS,online_delivery_improvement_claimed=False,
        normal_preplanning_not_counted_as_diagnostic_success=True)


def text_report(result):
    lines=['可行性停止条件：固定输入OFF/ON/OFF对照（2026-10-26）',
        '目的：检验“总代价停滞但轨迹尚不可行时，拒绝停止”能否在同一0.6秒内产生合格候选。',
        '本轮不是联合恢复算法，也不是在线轨迹交付改善证明。','']
    for trial in result['trials']:
        lines.append('{} captured={} evidence_valid={}'.format(trial['trial'],trial['captured'],trial['evidence_valid']))
        if not trial['captured']:
            lines.append('  未捕获有效输入；诊断前normal规划不算本实验成功。');continue
        lines.append('  首x/f/g一致={}，冻结上下文一致={}，capture相对障碍出现={}'.format(
            trial['independently_same_first_fg'],trial['same_frozen_recorded_context'],
            trial['scene_phase'].get('capture_minus_appearance_s')))
        for phase in trial['phases']:
            gate=phase['gate'];counts=phase['canonical_counts']['all'];last=phase['last_canonical_wheel_audit'] or {}
            lines.append('  {}：LBFGS={}ms，最终判定={}s，门耗时={}s，原生返回={}，reason={}'.format(
                phase['phase'],phase['lbfgs_wall_ms'],phase['decision_elapsed_s'],gate.get('gate_wall_s'),
                phase['native_return_success'],phase['solver_reason']))
            lines.append('    stop检查={}，veto={}，允许={}，门中预算耗尽={}，fatal={}；候选{} / native过{} / 投影过{} / 双过{}（后验）。'.format(
                (gate.get('record') or {}).get('checks'),gate.get('veto_count'),gate.get('allowed_count'),
                gate.get('budget_expired_count'),gate.get('fatal'),counts['count'],counts['native_pass'],
                counts['frozen_projected_pass'],counts['both_pass']))
            if gate.get('activated') is False and phase['treatment']=='ON':
                lines.append('    ON未触发代价停止门，不能据此估计veto收益。')
            if last:
                ratios=last['diagnostic_10ms_with_boundaries']['peaks']
                lines.append('    最后候选双轮四项峰值/最终限值：'+', '.join(
                    '{}={:.4f}'.format(k,v['ratio_to_final_limit']) for k,v in ratios.items()))
            lines.append('    canonical后验审计={}s，明确排除在求解/最终判定时间之外。'.format(phase['post_solver_audit_wall_s']))
        lines.append('  OFF复测时间差={}ms；只是顺序抖动对照，不是独立重复样本。'.format(
            trial['repeat_check']['off_solver_time_difference_ms']))
    lines+=['','边界：']+LIMITATIONS
    return '\n'.join(lines)+'\n'


def plot(result,path):
    assert_plot_quiet(result)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rows=[(t,p) for t in result['trials'] for p in t['phases']]
    if not rows:return
    fig,axes=plt.subplots(1,2,figsize=(10,3.8),constrained_layout=True)
    labels=[]
    for i,(trial,phase) in enumerate(rows):
        labels.append('s{}\n{}'.format(trial['seed'],phase['phase'].replace('delivery_','').replace('_060','')))
        solver=phase['lbfgs_wall_ms'];decision=phase['decision_elapsed_s']
        for dx,value,color,label in [(-.18,solver/1000. if finite(solver) else None,'#8c9cab','LBFGS'),
                                    (.18,decision,'#416b8e','Final decision')]:
            if finite(value):axes[0].bar(i+dx,value,.34,color=color,label=label if i==0 else None)
            else:axes[0].text(i+dx,0.,'N/A',ha='center',fontsize=7)
        counts=phase['canonical_counts']['all']
        for dx,key,color,label in [(-.18,'native_pass','#a3a3a3','Native pass'),
                                   (.18,'both_pass','#4b8a69','Native + projected')]:
            axes[1].bar(i+dx,counts[key],.34,color=color,label=label if i==0 else None)
            axes[1].text(i+dx,counts[key],str(counts[key]),ha='center',va='bottom',fontsize=7)
    axes[0].axhline(.6,color='#777777',linestyle='--',linewidth=.8)
    axes[0].set_ylabel('Seconds (posterior audit excluded)')
    axes[1].set_ylabel('Post-solver audited candidate count')
    for axis in axes:
        axis.set_xticks(range(len(rows)));axis.set_xticklabels(labels,fontsize=7)
        axis.legend(fontsize=7);axis.spines['top'].set_visible(False);axis.spines['right'].set_visible(False)
    axes[0].set_title('Solver / final decision');axes[1].set_title('Posterior feasibility, not delivery')
    fig.suptitle('Paused identical-input OFF / ON / OFF; budget 0.6 s',fontsize=11)
    fig.savefig(path,dpi=150);plt.close(fig)


def report(root,out,plot_image=False):
    root,out=Path(root).resolve(),Path(out).resolve()
    if out.exists() or out==root or out in root.parents:raise ValueError('Fresh separate report output required')
    result=build_report(root)
    out.mkdir(parents=True,exist_ok=False)
    (out/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    (out/'可行性停止对照结论.txt').write_text(text_report(result),encoding='utf-8')
    if plot_image:plot(result,out/'feasibility_stop.png')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=HERE/'results/feasibility_stop_20261026_01')
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--plot',action='store_true')
    args=parser.parse_args();answer=report(args.root,args.out,args.plot)
    print(json.dumps(dict(out=str(args.out),evidence_valid=answer['evidence_valid'],
        captured=sum(t['captured'] for t in answer['trials']))))
