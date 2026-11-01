#!/usr/bin/env python3
"""Post-run canonical accepted-iterate audit; never publishes or re-solves.

The native audit was performed AFTER the solver and outside its budget. A
posteriori feasible iterates do not prove a deliverable online trajectory.
"""
import argparse
from collections import Counter
import gzip
import json
import math
from pathlib import Path
import struct

import numpy as np

from audit_delivery_fullcurve import frozen_frame, quadrature_samples, scan
from recheck_delivery_diag import candidate_samples, evaluate_pose, fresh, polynomial
from report_domain_recovery import assert_plot_quiet, digest, read_json, read_records

PHASES=('delivery_nominal_060','delivery_repeat_060','delivery_long_150')
HEAD_TOLERANCE=1e-7


def loads_preserving_cpp_zero(text):
    # C++ emits IEEE negative zero as JSON integer token -0. Python's default
    # int parser loses its sign bit, invalidating exact x hashes and sometimes
    # velocity-derived atan2 headings. Preserve ONLY this token; large integer
    # identities must remain arbitrary-precision integers, never float.
    return json.loads(text,parse_int=lambda token:-0.0 if token=='-0' else int(token))


def canonical_records(directory):
    paths=[Path(directory)/'delivery_replay.jsonl',Path(directory)/'delivery_replay.jsonl.gz']
    path=next((p for p in paths if p.is_file()),None)
    if path is None:return None,None
    opener=gzip.open if path.suffix=='.gz' else open
    records=[]
    with opener(path,'rt',encoding='utf-8') as stream:
        for number,line in enumerate(stream,1):
            if not line.strip():continue
            record=loads_preserving_cpp_zero(line)
            if not isinstance(record,dict):raise ValueError('{}:{} is not an object'.format(path,number))
            records.append(record)
    return records,path


def finite(value):
    return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)


def vector_hash(values):
    """C++ resource_probe seed (NOT standard FNV offset), LE IEEE754 doubles."""
    if not isinstance(values,list) or not values or not all(finite(v) for v in values):
        return None
    result=1469598103934665603
    for value in values:
        for byte in struct.pack('<d',value):
            result=((result^byte)*1099511628211)&((1<<64)-1)
    return str(result)


def conjunction(values):
    values=list(values)
    if any(v is False for v in values):return False
    return True if values and all(v is True for v in values) else None


def canonical_candidate(record,start_time):
    if record.get('reconstructed') is not True:return None
    parts=[]
    for part in record.get('canonical_parts',[]):
        times=np.asarray(part.get('times',[]),dtype=float)
        b=np.asarray(part.get('b_rows_ascending_power',[]),dtype=float)
        gear=part.get('gear')
        if (times.ndim!=1 or not len(times) or not np.all(np.isfinite(times)) or np.any(times<=0) or
                b.ndim!=2 or b.shape[0]!=len(times)*8 or b.shape[1]<2 or not np.all(np.isfinite(b)) or gear not in (-1,1)):
            raise ValueError('invalid canonical polynomial/times/gear')
        pieces=[dict(T=float(duration),basis='descending',coeffs=b[8*i:8*(i+1),:][::-1,:].T.tolist())
            for i,duration in enumerate(times)]
        parts.append(dict(gear=int(gear),duration=float(sum(times)),pieces=pieces))
    if not parts:raise ValueError('reconstructed candidate has no parts')
    if not finite(start_time):raise ValueError('missing recorded frozen frame.now')
    return dict(schema='delivery_candidate_v1',start_time=float(start_time),parts=parts)


def matrix(record):
    rows,cols=record.get('rows'),record.get('cols')
    values=np.asarray(record.get('data_column_major',[]),dtype=float)
    if not isinstance(rows,int) or not isinstance(cols,int) or rows<=0 or cols<=0 or values.size!=rows*cols or not np.all(np.isfinite(values)):
        raise ValueError('invalid recorded effective boundary matrix')
    return values.reshape((rows,cols),order='F')


def head_evidence(candidate,summary):
    recorded=summary.get('effective_initial_states')
    if not isinstance(recorded,list) or not recorded:
        return dict(known=False,matches=None,reason='effective_initial_state_missing')
    head=matrix(recorded[0]);coeff=np.asarray(candidate['parts'][0]['pieces'][0]['coeffs'])
    if head.shape!=(coeff.shape[0],4):
        return dict(known=False,matches=None,reason='boundary_dimensions_unknown')
    reconstructed=np.column_stack([math.factorial(k)*coeff[:,7-k] for k in range(4)])
    error=float(np.max(np.abs(head-reconstructed)))
    tolerance=HEAD_TOLERANCE*max(1.,float(np.max(np.abs(head))))
    return dict(known=True,matches=error<=tolerance,max_abs_error=error,tolerance=tolerance,
        original_head=head.tolist(),canonical_head=reconstructed.tolist(),
        reason='canonical k0 position/velocity/acceleration/jerk vs native effective initial boundary')


def first_signature(summary,initial):
    if initial is None:return dict(verified=None,reason='missing_unique_initial')
    x_hash,g_hash=vector_hash(initial.get('x')),vector_hash(initial.get('g'))
    checks=[summary.get('first_seen') is True,finite(initial.get('cost')),
        initial.get('cost')==summary.get('first_cost'),x_hash is not None and x_hash==summary.get('first_x_hash'),
        g_hash is not None and g_hash==summary.get('first_gradient_hash')]
    return dict(verified=all(checks),x_hash=x_hash,gradient_hash=g_hash,cost=initial.get('cost'),
        hashing='resource_probe byte-FNV seed1469598103934665603 / little-endian IEEE754 f64')


def analyze_phase(trial,phase,rows,capture,frame,frame_proof):
    records=[r for r in rows if r.get('phase')==phase]
    summaries=[r for r in records if r.get('kind')=='solve_summary']
    ends=[r for r in records if r.get('kind')=='post_solver_audit_end']
    returns=[r for r in records if r.get('kind')=='recursive_return']
    summary=summaries[0] if len(summaries)==1 else {}
    canonical=[r for r in records if r.get('kind')=='canonical_candidate']
    initial=[r for r in canonical if r.get('stage')=='initial']
    accepted=[r for r in canonical if r.get('stage')=='accepted']
    signature=first_signature(summary,initial[0] if len(initial)==1 else None)
    audit_end=ends[0] if len(ends)==1 else {}
    native_return_matches=(len(returns)==1 and isinstance(summary.get('native_return_success'),bool) and
        returns[0].get('success')==summary['native_return_success'])
    complete=(len(summaries)==len(ends)==len(returns)==1 and native_return_matches and len(initial)==1 and
        len(canonical)==len(initial)+len(accepted) and len(accepted)==summary.get('accepted_total') and
        summary.get('accepted_truncated') is False and audit_end.get('audited_candidates')==len(canonical) and
        audit_end.get('not_part_of_lbfgs_budget') is True and not any(r.get('kind')=='audit_error' for r in records))
    iteration=[r.get('iteration') for r in accepted]
    complete=complete and all(isinstance(k,int) and k>0 for k in iteration) and iteration==sorted(set(iteration))
    exact_input=(str(summary.get('input_hash'))==str(capture.get('input_hash')) and
        all(r.get('request_id')==capture.get('request_id') for r in records)) if summary else None
    frozen=(summary.get('raw_seq')==capture.get('raw_seq') and summary.get('esdf_seq')==capture.get('esdf_seq') and
        summary.get('sim')==summary.get('sim_end')==capture.get('sim') and
        summary.get('frame_now')==capture.get('frame',{}).get('now') and summary.get('frame_sequence')==frame_proof.get('sequence')) if summary else None
    analyses=[]
    for record in canonical:
        candidate=canonical_candidate(record,capture['frame']['now'])
        head=head_evidence(candidate,summary) if candidate else dict(known=False,matches=None,reason='not_reconstructed')
        geometric=scan(candidate,frame,candidate_samples(candidate,'dense',.02),None) if candidate is not None else None
        projected=(fresh(frame,capture['frame']['now']) and geometric['static_all_sampled_clear']) if geometric else None
        native=record.get('native_IsTrajSafe_pass') if isinstance(record.get('native_IsTrajSafe_pass'),bool) else None
        audited=record.get('native_audit_wall_s');elapsed=record.get('solver_elapsed_s')
        evidence=conjunction([candidate is not None,head['matches'],native is not None,
            finite(audited) and audited>=0,finite(elapsed) and elapsed>=0,
            record.get('native_gate_skips_first_0_05_s') is True,
            record.get('future_FSM_live_acceptance_certified') is False])
        analyses.append(dict(stage=record.get('stage'),iteration=record.get('iteration'),solver_elapsed_s=elapsed,
            native_IsTrajSafe_pass=native,native_reason=record.get('native_reason'),native_audit_wall_s=audited,
            native_rejection_value=record.get('native_rejection_value'),native_rejection_time=record.get('native_rejection_time'),
            fixed_head=head,frozen_projected_20ms=geometric,frozen_projected_pass=projected,
            both_pass=conjunction([native,projected]),candidate_evidence_valid=evidence,
            candidate_x_hash=vector_hash(record.get('x')),candidate_g_hash=vector_hash(record.get('g')),
            not_an_online_delivery=True))
    statistics={}
    for stage in ('initial','accepted','all'):
        group=[r for r in analyses if stage=='all' or r['stage']==stage]
        statistics[stage]=dict(count=len(group),native_pass=sum(r['native_IsTrajSafe_pass'] is True for r in group),
            native_unknown=sum(r['native_IsTrajSafe_pass'] is None for r in group),
            frozen_projected_pass=sum(r['frozen_projected_pass'] is True for r in group),
            frozen_projected_unknown=sum(r['frozen_projected_pass'] is None for r in group),
            both_pass=sum(r['both_pass'] is True for r in group))
    both=[r for r in analyses if r['both_pass'] is True and r['candidate_evidence_valid'] is True]
    first=next(iter(both),None)
    first_native=next((r for r in analyses if r['native_IsTrajSafe_pass'] is True and r['candidate_evidence_valid'] is True),None)
    selected={}
    if analyses:
        choices={'last':len(analyses)-1}
        accepted_indices=[i for i,r in enumerate(analyses) if r['stage']=='accepted' and
            isinstance(r.get('frozen_projected_20ms'),dict) and finite(r['frozen_projected_20ms'].get('min_static_gap_m'))]
        if accepted_indices:
            choices['best_projected_gap_accepted']=max(accepted_indices,key=lambda i:analyses[i]['frozen_projected_20ms']['min_static_gap_m'])
        for label,index in choices.items():
            item=analyses[index];candidate=canonical_candidate(canonical[index],capture['frame']['now'])
            if candidate is None:
                selected[label]=dict(known=False,reason='not_reconstructed');continue
            coarse=scan(candidate,frame,quadrature_samples(candidate,8),None)
            first_pose=next(candidate_samples(candidate,'dense',.02))
            k0=evaluate_pose(frame,first_pose['x'],first_pose['y'],first_pose['yaw'],0.)
            selected[label]=dict(known=True,stage=item['stage'],iteration=item['iteration'],solver_elapsed_s=item['solver_elapsed_s'],
                native_IsTrajSafe_pass=item['native_IsTrajSafe_pass'],k0_static=k0,
                projected_20ms_min_gap_m=item['frozen_projected_20ms']['min_static_gap_m'],K8_projected_geometry=coarse,
                recorded_margin_m=frame['margin'],recorded_comfort_m=frame['comfort'],
                K8_comfort_zone_seen=coarse['worst_static']['physical']<frame['comfort'] if coarse['worst_static'] else None,
                geometry_only_not_full_cost_gradient_replay=True)
    costs=[r['native_audit_wall_s'] for r in analyses]
    total=sum(costs) if all(finite(v) and v>=0 for v in costs) else None
    valid=conjunction([complete,exact_input,frozen,signature['verified'],all(r['candidate_evidence_valid'] is True for r in analyses)])
    return dict(phase=phase,capture_identity=dict(trial=trial.name,request_id=capture.get('request_id'),phase=phase,
        input_hash=capture.get('input_hash'),initial_x_hash=signature.get('x_hash')),
        structure_complete=complete,exact_request_input=exact_input,independent_frozen_match=frozen,
        first_signature=signature,phase_evidence_valid=valid,solve_summary=summary,
        recursive_return=returns[0] if len(returns)==1 else None,post_solver_audit=audit_end,
        counts=statistics,first_both_pass=dict(stage=first['stage'],iteration=first['iteration'],
            solver_elapsed_s=first['solver_elapsed_s'],native_audit_wall_s=first['native_audit_wall_s']) if first else None,
        first_native_pass=dict(stage=first_native['stage'],iteration=first_native['iteration'],
            solver_elapsed_s=first_native['solver_elapsed_s'],native_audit_wall_s=first_native['native_audit_wall_s']) if first_native else None,
        selected_candidate_checks=selected,
        native_audit_cost=dict(sum_wall_s=total,max_candidate_wall_s=max(costs) if costs and total is not None else None,
            post_solver_total_wall_s=audit_end.get('audit_wall_s'),outside_lbfgs_budget=True),candidates=analyses,
        a_posteriori_not_real_time_delivery=True)


def analyze_trial(trial):
    rows,path=canonical_records(trial)
    status=read_json(trial/'run_status.json')
    if rows is None:
        return dict(trial=trial.name,captured=False,evidence_valid=None,reason='no_capture; never treated as pass',run_status=status)
    if any(row.get('schema')!='delivery_replay_v1' for row in rows):raise ValueError('unknown replay schema')
    captures=[r for r in rows if r.get('kind')=='capture']
    completions=[r for r in rows if r.get('kind')=='complete']
    if len(captures)!=1:raise ValueError('ambiguous capture in '+str(trial))
    capture=captures[0];completion=completions[0] if len(completions)==1 else {}
    frame,frame_proof=frozen_frame(trial/'delivery_diagnostics',capture['frame'])
    if len(frame['obstacles']):raise ValueError('static-only diagnostic contains dynamic records')
    phases=[analyze_phase(trial,phase,rows,capture,frame,frame_proof) for phase in PHASES]
    signatures=[(p['first_signature'].get('cost'),p['first_signature'].get('x_hash'),p['first_signature'].get('gradient_hash')) for p in phases]
    same_first=(all(p['first_signature']['verified'] is True for p in phases) and len(set(signatures))==1)
    hashes=[p['solve_summary'].get('prediction_hash') for p in phases]
    same_prediction=(all(isinstance(v,str) and v for v in hashes) and len(set(hashes))==1)
    flags=conjunction([completion.get(k) if isinstance(completion.get(k),bool) else None for k in (
        'frozen_stable','native_dynamic_observed_stable','same_first_fg','audits_complete')])
    status_valid=conjunction([status.get('diagnostic_complete'),status.get('graph_audited'),
        status.get('production_hashes_unchanged'),status.get('build_manifest_unchanged'),
        status.get('required_process_failed') is False,status.get('runner_error') is None])
    valid=conjunction([len(completions)==1,flags,same_first,same_prediction,status_valid,
        all(p['phase_evidence_valid'] is True for p in phases),capture.get('all_passes_paused_before_nominal') is True,
        capture.get('constraints_relaxed') is False,completion.get('diagnostic_candidates_published') is False])
    return dict(trial=trial.name,captured=True,evidence_valid=valid,capture=capture,complete=completion,
        independently_same_first_fg=same_first,same_recorded_prediction_hash=same_prediction,
        snapshot=frame_proof,replay_path=str(path),replay_sha256=digest(path),run_status=status,
        status_evidence_valid=status_valid,manifest_sha256=digest(trial/'manifest.json'),
        run_status_sha256=digest(trial/'run_status.json'),phases=phases)


def plot(report,target):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rows=[(trial,phase) for trial in report['trials'] for phase in trial.get('phases',[])]
    if not rows:return
    fig,ax=plt.subplots(figsize=(10,4.3),constrained_layout=True)
    labels=[]
    for i,(trial,phase) in enumerate(rows):
        labels.append(trial['trial'].replace('static_diagnostic_seed','s').replace('_refresh',' r')+'\n'+phase['phase'].replace('delivery_',''))
        values=phase['counts']['all']
        for dx,key,color,label in [(-.2,'count','#bdbdbd','Canonical candidates'),(0,'native_pass','#5b8bb0','Native pass'),(.2,'both_pass','#4a9a67','Native + projected pass')]:
            ax.bar(i+dx,values[key],.2,color=color,label=label if i==0 else None)
    ax.set(xticks=range(len(labels)),xticklabels=labels,ylabel='Post-solver audited count',
        title='Paused identical-input diagnostics; not online delivery or timing proof')
    ax.tick_params(axis='x',labelsize=7,rotation=30);ax.legend(fontsize=8)
    fig.savefig(target,dpi=150);plt.close(fig)


def report(root,out,plot_image=False):
    root,out=Path(root).resolve(),Path(out).resolve()
    if out.exists() or out==root or out in root.parents:raise ValueError('fresh separate output required')
    complete=read_json(root/'complete.json')
    if complete.get('complete') is not True:raise ValueError('batch is not complete')
    trials=[root/job['trial'] for job in complete.get('completed',[]) if job.get('kind')=='paused']
    if len(trials)!=4:raise ValueError('expected four predeclared paused captures')
    assert_plot_quiet(dict(trials=[dict(run_status=read_json(t/'run_status.json')) for t in trials]))
    result=dict(schema='delivery_iterate_audit_v1',source=str(root),trials=[analyze_trial(t) for t in trials],
        limitations=['Fresh paused same-process captured inputs, not exact replay of historical requests or navigation trials.',
            'Native IsTrajSafe and frozen projected gate are separate; neither certifies future live FSM acceptance.',
            'Canonical initial/accepted x is reconstructed; no mutable last line-search cache is treated as accepted.',
            'C++ JSON -0 integer tokens are preserved as negative-zero doubles; all other integer identities retain exact integer precision. Strict x/g hash verification is unchanged.',
            'Native audit took place after solver, outside budget. First feasible iteration elapsed is not an online feasible-delivery time.',
            'Frozen projected static 20ms samples use capture.frame.now; no dynamic prediction or continuous safety claim.',
            'Native gate skips first 0.05s; projected scan includes k0; comparisons retain this difference.',
            'Head equality uses native effective initial boundary after normalization, tolerance 1e-7 times max(1,head magnitude).',
            'Selected last/best-gap accepted candidates also undergo K8 projected-geometry grid scans; this is not full native 3D cost or gradient replay.',
            'Missing captures, incomplete audits and unknown fields are not successes.'])
    for trial in result['trials']:
        if trial['captured']:
            if digest(Path(trial['replay_path']))!=trial['replay_sha256'] or digest(Path(trial['snapshot']['path']))!=trial['snapshot']['sha256']:
                raise RuntimeError('captured replay/snapshot changed during audit')
    out.mkdir(parents=True,exist_ok=False)
    (out/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    lines=['REMANI固定输入：已接受迭代的后验可行性审计',
        '原生IsTrajSafe、冻结投影20ms扫描、未来FSM在线验收不是一回事。',
        '所有native候选验收均在求解结束后执行，不计入L-BFGS预算；不能把首次可行迭代时间称作实际在线交付时间。','']
    for trial in result['trials']:
        lines.append('{} captured={} evidence_valid={}'.format(trial['trial'],trial['captured'],trial['evidence_valid']))
        for phase in trial.get('phases',[]):
            lines.append('  {}: initial={} accepted={} native_pass={} projected_pass={} both_pass={} first_both={} native_audit_total={}s / solver_reason={}'.format(
                phase['phase'],phase['counts']['initial']['count'],phase['counts']['accepted']['count'],phase['counts']['all']['native_pass'],
                phase['counts']['all']['frozen_projected_pass'],phase['counts']['all']['both_pass'],phase['first_both_pass'],
                phase['native_audit_cost']['sum_wall_s'],phase['solve_summary'].get('reason')))
            lines.append('    首个native合格候选={}；首个native+projected双合格={}（均为后验审计）。'.format(
                phase['first_native_pass'],phase['first_both_pass']))
            for label,check in phase['selected_candidate_checks'].items():
                if check.get('known') is True:
                    lines.append('    {} iter={}: k0 gap={:.9f}m / 全程20ms最坏gap={:.9f}m / K8最坏gap={:.9f}m / K8违规样本={}。'.format(
                        label,check['iteration'],check['k0_static']['gap'],check['projected_20ms_min_gap_m'],
                        check['K8_projected_geometry']['min_static_gap_m'],
                        check['K8_projected_geometry']['static_categories'].get('margin_only',0)+check['K8_projected_geometry']['static_categories'].get('collision',0)))
    lines.extend(['','本报告不修改生产代码、不放宽任何限制、不发布诊断候选。完整证据、hash与逐候选最坏gap见summary.json。'])
    (out/'迭代后验复核结论.txt').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    if plot_image:plot(result,out/'iterate_feasibility.png')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--plot',action='store_true');a=p.parse_args()
    result=report(a.root,a.out,a.plot)
    print(json.dumps(dict(out=str(a.out),captured=sum(t['captured'] for t in result['trials']),valid=sum(t['evidence_valid'] is True for t in result['trials']))))
