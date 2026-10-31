#!/usr/bin/env python3
"""Offline delivery diagnostics; paused solves are never navigation metrics."""
import argparse
from collections import Counter,defaultdict
from datetime import datetime,timezone
import json
import math
from pathlib import Path

from report_domain_recovery import assert_plot_quiet,digest,read_json,read_records

LABELS=('delivery_nominal_060','delivery_repeat_060','delivery_long_150')
BUDGETS=(.6,.6,1.5)
LIMITATIONS=[
    '仅当前固定机械臂、静态数值SIL分支；不是原生baseline归因、Gazebo或实车安全认证。',
    '两次online使用零预测软耦合权重，与此前权重1/1批次不同，不进行跨批因果速度对比。',
    '搜索统计以wrapper最终结果为准；底层A*失败可以被采样/RRT回退挽救。采样/RRT内部细出口未记录。',
    'paused是新捕获的同进程暂停求解，不是完整历史重放，也不是导航/实时性能测量。',
    '首x/f/g一致和观测上下文一致是诊断证据，不单独证明完整native地图或所有隐状态逐字节冻结。',
    'solver时间只取failure_trace的lbfgs_wall_ms；其总wall_ms含后验逐候选重建与验收，不能当求解时间。',
    'accepted候选的完整可行性由独立canonical审计报告判定；仅native通过不等于投影终验通过或未来可发布。',
    '不把未捕获、缺日志、超时或验收失败样本删除；小样本不用于总体成功率或显著性结论。',
]


def finite(v):return isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v)


def one(rows,predicate):
    found=[r for r in rows or [] if predicate(r)]
    return found[0] if len(found)==1 else None


def hash_text(v):
    if isinstance(v,str) and v.isdigit():return str(int(v))
    if isinstance(v,int) and not isinstance(v,bool) and v>=0:return str(v)
    return None


def summarize_search(rows):
    """Separate final wrapper outcomes from explanatory A* exits."""
    if rows is None:return dict(present=False,wrapper_count=None,wrapper_reasons=None,native_reasons=None,records=[])
    exits=[r for r in rows if r.get('event')=='delivery_search_exit']
    native=[r for r in exits if r.get('solver')=='kino_astar']
    wrappers=[r for r in exits if r.get('solver')=='frontend_wrapper'];records=[]
    for w in wrappers:
        children=[n for n in native if n.get('pid')==w.get('pid') and n.get('request_id')==w.get('request_id')
                  and n.get('probe_parent_call_id')==w.get('probe_call_id')]
        success=w.get('return_code') in (1,2)
        records.append(dict(pid=w.get('pid'),request_id=w.get('request_id'),call_id=w.get('probe_call_id'),
            final_reason=w.get('exit_reason'),return_code=w.get('return_code'),frontend_success=success,
            frontend_wall_ms=w.get('wall_ms'),sampling_route=w.get('sampling_route'),
            native_attempted=w.get('native_astar_attempted'),native_skip_reason=w.get('native_astar_skip_reason'),
            native_children=children,ambiguous_native_children=len(children)>1,
            fallback_rescued=success and w.get('sampling_route')=='rrt_fallback',
            astar_success_then_sampling_failed=not success and any(n.get('return_code')==2 for n in children)))
    return dict(present=True,wrapper_count=len(wrappers),wrapper_reasons=dict(Counter(w.get('exit_reason','missing') for w in wrappers)),
        native_reasons=dict(Counter(n.get('exit_reason','missing') for n in native)),
        wrapper_successes=sum(r['frontend_success'] for r in records),
        fallback_rescues=sum(r['fallback_rescued'] for r in records),
        astar_success_then_sampling_failures=sum(r['astar_success_then_sampling_failed'] for r in records),records=records)


def summarize_paused(rows,failures,done,native_timeline=None):
    rows=rows or [];captures=[r for r in rows if r.get('kind')=='capture']
    capture=captures[0] if len(captures)==1 else None
    summaries=[r for r in rows if r.get('kind')=='solve_summary'];passes=[]
    for label,budget in zip(LABELS,BUDGETS):
        summary=one(summaries,lambda r:r.get('phase')==label)
        native=one(failures,lambda r:r.get('kind')=='solve' and r.get('label')==label)
        returned=one(rows,lambda r:r.get('kind')=='recursive_return' and r.get('phase')==label)
        expected_hash=hash_text(capture.get('input_hash')) if capture else None
        input_equal=bool(expected_hash and summary and native and
            hash_text(summary.get('input_hash'))==expected_hash==hash_text(native.get('input_hash')))
        elapsed=native.get('lbfgs_wall_ms') if native else None
        passes.append(dict(label=label,budget_s=budget,summary=summary,native_failure_frame=native,
            recursive_return=returned,input_hash_equal=input_equal,
            solver_wall_ms=elapsed if finite(elapsed) else None,
            solver_time_source='failure_trace.lbfgs_wall_ms; NOT failure_trace.wall_ms',
            complete_records=summary is not None and native is not None and returned is not None,
            budget_agrees=bool(summary and native and summary.get('budget_s')==budget==native.get('budget_s')),
            verdict_agrees=bool(summary and native and returned and
                summary.get('native_return_success') is native.get('success') and
                native.get('success') is returned.get('success') and summary.get('reason')==native.get('reason'))))
    present=[p['summary'] for p in passes]
    all_present=all(s is not None for s in present)
    first_fields=('first_cost','first_x_hash','first_gradient_hash')
    context_fields=('effective_initial_states','effective_final_states','raw_seq','esdf_seq','frame_now',
                    'frame_sequence','prediction_hash','sim','wheel_limit')
    first_equal=all_present and all(s.get('first_seen') is True for s in present) and all(
        all(field in s and s[field]==present[0].get(field) for s in present) for field in first_fields)
    context_equal=all_present and all(all(field in s and s[field]==present[0].get(field) for s in present)
        for field in context_fields) and all(s.get('sim_end')==s.get('sim') for s in present)
    done_checks={k:done.get(k) is True for k in ('complete','frozen_stable','same_first_fg','audits_complete',
                                               'native_dynamic_observed_stable')}
    no_publish=done.get('diagnostic_candidates_published') is False and not any(
        r.get('diagnostic_candidates_published') is True for r in rows)
    request=capture.get('request_id') if capture else None
    observed_no_publish=(not any(r.get('request_id')==request and r.get('event') in
        ('reference_publish','state_contract_reference_publish') for r in native_timeline)
        if native_timeline is not None and isinstance(request,int) and request>0 else None)
    valid=bool(capture and all(done_checks.values()) and no_publish and observed_no_publish is True and
        first_equal and context_equal and len(summaries)==3 and
        all(p['complete_records'] and p['input_hash_equal'] and p['verdict_agrees'] and p['budget_agrees'] for p in passes))
    return dict(capture_count=len(captures),capture=capture,passes=passes,done_checks=done_checks,
        first_x_fg_equal=first_equal,observed_context_consistent=context_equal,
        diagnostic_no_publication_declaration=no_publish,observed_no_captured_request_publication=observed_no_publish,
        diagnosis_record_contract_valid=valid,navigation_metrics_valid=False,complete_native_map_identity_proven=False,
        canonical_candidate_count=sum(r.get('kind')=='canonical_candidate' for r in rows),
        audit_errors=[r for r in rows if r.get('kind')=='audit_error'])


def _record(trial,name,hashes):
    rows,path=read_records(trial,name)
    if path is not None:hashes[path.name]=digest(path)
    return rows


def trial_report(trial,kind):
    trial=Path(trial);manifest=read_json(trial/'manifest.json');status=read_json(trial/'run_status.json')
    hashes={n:digest(trial/n) for n in ('manifest.json','run_status.json')}
    runtime=manifest.get('experiment_sources_sha256') or {};runtime_files={}
    for name,expected in runtime.items():
        if not name.endswith('.py'):continue
        path=trial/'runtime'/name
        runtime_files[name]=dict(expected=expected,observed=digest(path) if path.is_file() else None)
    runtime_verified=bool(runtime_files) and all(v['expected']==v['observed'] for v in runtime_files.values())
    native=_record(trial,'dual_timeline.jsonl',hashes);failure=_record(trial,'failure_trace.jsonl',hashes)
    answer=dict(trial=trial.name,kind=kind,source=str(trial),manifest=manifest,run_status=status,input_sha256=hashes,
        runtime_sha256=runtime,runtime_python_files=runtime_files,runtime_python_verified=runtime_verified,
        search=summarize_search(native))
    if kind=='paused':
        replay=_record(trial,'delivery_replay.jsonl',hashes)
        path=trial/'diagnostic_complete.json';done=read_json(path) if path.is_file() else {}
        if path.is_file():hashes[path.name]=digest(path)
        answer['paused']=summarize_paused(replay,failure,done,native)
    else:
        normal=[r for r in failure or [] if r.get('kind')=='solve' and r.get('label')=='normal']
        answer['native_solve_trace_present']=failure is not None
        answer['native_solves']=normal
        answer['native_solve_reasons']=dict(Counter(r.get('reason','missing') for r in normal))
        path=trial/'result.json';answer['outcome']=read_json(path) if path.is_file() else None
        if path.is_file():hashes[path.name]=digest(path)
        answer['published_reference_count']=sum(r.get('event')=='state_contract_reference_publish' for r in native or [])
    return answer


def build_report(root):
    root=Path(root).resolve();plan=read_json(root/'plan.json');complete=read_json(root/'complete.json')
    jobs=complete.get('completed')
    if complete.get('complete') is not True or not isinstance(jobs,list) or len(jobs)!=6:
        raise ValueError('Require all six retained jobs (two online plus four paused)')
    names=[j.get('trial') for j in jobs]
    if len(set(names))!=6 or any(not isinstance(n,str) or Path(n).name!=n or n in ('.','..') for n in names):
        raise ValueError('Unsafe/duplicate trial path')
    if Counter(j.get('kind') for j in jobs)!=dict(online=2,paused=4):raise ValueError('Wrong job split')
    expected={'static_capture_seed%d'%s for s in plan.get('normal_seeds',[])}
    expected.update('static_diagnostic_seed%d_refresh%d'%(j['seed'],j['ordinal'])
                    for j in plan.get('diagnostic_jobs',[]))
    if set(names)!=expected:raise ValueError('Completed jobs do not match prespecified seed/ordinal plan')
    assert_plot_quiet(dict(trials=[dict(run_status=read_json(root/n/'run_status.json')) for n in names]))
    trials=[trial_report(root/j['trial'],j['kind']) for j in jobs]
    build=Path(plan['build']);bm=build/'build_manifest.json'
    if digest(bm)!=plan.get('build_manifest_sha256'):raise ValueError('Build manifest changed')
    attested=read_json(bm)
    source_hashes=attested.get('isolated_source_sha256',{});binary_hashes=attested.get('native_binaries_sha256',{})
    source_ok=bool(source_hashes) and all((build/'source'/n).is_file() and digest(build/'source'/n)==h
        for n,h in source_hashes.items())
    binaries_ok=bool(binary_hashes) and all((build/'build'/n).is_file() and digest(build/'build'/n)==h
        for n,h in binary_hashes.items())
    same_runtime=bool(trials[0]['runtime_sha256']) and all(t['runtime_sha256']==trials[0]['runtime_sha256'] for t in trials)
    return dict(schema='delivery_stability_report_v1',created=datetime.now(timezone.utc).isoformat(),root=str(root),
        plan=plan,complete=complete,trials=trials,input_sha256={n:digest(root/n) for n in ('plan.json','complete.json')},
        integrity=dict(build_manifest_sha256=digest(bm),isolated_source_verified=source_ok,
            native_binaries_verified=binaries_ok,same_full_runtime_manifest=same_runtime,
            runtime_python_snapshots_verified=all(t['runtime_python_verified'] for t in trials)),
        diagnostic_valid_count=sum(t['paused']['diagnosis_record_contract_valid'] for t in trials if t['kind']=='paused'),
        limitations=LIMITATIONS,algorithm_fix_implemented=False,performance_improvement_claim=False)


def write_report(value,out,plot=False):
    out=Path(out);out.mkdir(parents=True,exist_ok=False)
    (out/'summary.json').write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
    lines=['REMANI轨迹交付诊断：本轮只定位，不宣称已修复','',
        '在线与暂停求解分开统计；暂停诊断候选不允许发布。','']
    for t in value['trials']:
        lines.append(t['trial'])
        if t['kind']=='online':
            lines.append('  前端最终出口：'+json.dumps(t['search']['wrapper_reasons'],ensure_ascii=False))
            lines.append('  底层A*出口：'+json.dumps(t['search']['native_reasons'],ensure_ascii=False))
            lines.append('  原生求解结果：'+json.dumps(t['native_solve_reasons'],ensure_ascii=False))
            lines.append('  发布新参考次数（含初始参考）：'+str(t['published_reference_count']))
        else:
            p=t['paused'];lines.append('  记录契约通过：'+str(p['diagnosis_record_contract_valid']))
            for a in p['passes']:
                s=a['summary'] or {};ms=a['solver_wall_ms']
                lines.append('  %s：%s；LBFGS %.2f ms'%(a['label'],s.get('reason','缺失'),ms) if ms is not None
                    else '  %s：缺少有效LBFGS计时'%a['label'])
        lines.append('')
    lines+=['边界与未完成项：']+['- '+s for s in LIMITATIONS]
    (out/'实验结论.txt').write_text('\n'.join(lines)+'\n')
    if plot:
        assert_plot_quiet(value)
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(1,2,figsize=(11,4))
        online=[t for t in value['trials'] if t['kind']=='online'];counts=Counter()
        for t in online:counts.update(t['search']['wrapper_reasons'] or {})
        axes[0].barh(list(counts),list(counts.values()),color='#4c78a8');axes[0].set_title('Final frontend exits (online)')
        axes[0].set_xlabel('Count (initial plans included)')
        paused=[t for t in value['trials'] if t['kind']=='paused']
        for j,label in enumerate(LABELS):
            xs=[];ys=[]
            for i,t in enumerate(paused):
                ms=t['paused']['passes'][j]['solver_wall_ms']
                if ms is not None:xs.append(i+(j-1)*.24);ys.append(ms)
            axes[1].bar(xs,ys,width=.22,label=label.replace('delivery_',''))
        axes[1].set_xticks(range(len(paused)));axes[1].set_xticklabels(['D'+str(i+1) for i in range(len(paused))])
        axes[1].set_title('Paused LBFGS only (not navigation)');axes[1].set_ylabel('Wall time / ms');axes[1].legend(fontsize=8)
        fig.tight_layout();fig.savefig(out/'delivery_diagnostics.png',dpi=150);plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--plot',action='store_true');a=p.parse_args()
    write_report(build_report(a.root),a.out,a.plot)
