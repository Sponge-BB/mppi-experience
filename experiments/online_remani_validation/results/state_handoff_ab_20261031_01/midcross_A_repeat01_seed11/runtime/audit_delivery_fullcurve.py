#!/usr/bin/env python3
"""Read-only full-curve audit of recorded native delivery candidates.

This reconstructs the SAME frozen disk-union/distance-field geometry, not
physical ground truth, native 3D collision replay, or continuous-time safety.
No ROS, optimizer, retiming, or acceptance policy is invoked or changed.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from recheck_delivery_diag import (EPS, candidate_samples, evaluate_pose, fresh,
    load_json, polynomial, snapshot_arrays)
from report_domain_recovery import assert_plot_quiet, digest, read_records
from report_state_contract import backend_scope_evidence, unsigned_identity


def fnv64(data):
    value = 14695981039346656037
    for byte in data:
        value = ((value ^ byte) * 1099511628211) & ((1 << 64)-1)
    return format(value, '016x')


def frozen_frame(root, frame):
    reference = frame.get('reference', frame)
    relative = reference.get('snapshot')
    if not isinstance(relative, str):
        raise ValueError('missing frozen snapshot')
    path = (root / relative).resolve()
    if root.resolve() not in path.parents:
        raise ValueError('snapshot escapes diagnostics directory')
    payload = path.read_bytes()
    if reference.get('hash_algorithm') != 'fnv1a64_json' or fnv64(payload) != reference.get('snapshot_payload_hash'):
        raise ValueError('snapshot payload hash disagreement: '+str(path))
    raw = json.loads(payload)
    if raw.get('schema') != 'delivery_snapshot_v1':
        raise ValueError('unrecognized snapshot schema')
    return snapshot_arrays(raw), dict(path=str(path), sha256=hashlib.sha256(payload).hexdigest(),
        snapshot_payload_hash=fnv64(payload), payload_hash_verified=True, sequence=raw['sequence'])


def identity(candidate, path, native):
    pid, request, scope = candidate.get('clock', {}).get('pid'), candidate.get('request_id'), candidate.get('scope_id')
    rows = [row for row in native if row.get('pid') == pid and row.get('request_id') == request]
    begins = [row for row in rows if row.get('event') == 'request_begin']
    ends = [row for row in rows if row.get('event') == 'request_end']
    fixed = [row for row in rows if row.get('event') == 'state_contract_backend_fixed']
    solves = [row for row in rows if row.get('event') == 'state_contract_solver_return']
    filename_pid = int(path.stem.split('_')[0])
    proof = backend_scope_evidence(rows, fixed[0]) if len(fixed) == 1 else {}
    valid = (isinstance(pid, int) and pid > 1 and filename_pid == pid and
        len(begins) == len(ends) == len(fixed) == len(solves) == 1 and
        solves[0].get('success') is True and proof.get('complete') is True and proof.get('scope_id') == scope and
        isinstance(candidate.get('accepted'), bool) and ends[0].get('success') == candidate['accepted'])
    if not valid:
        raise ValueError('ambiguous candidate PID/request/backend scope identity: '+str(path))
    return dict(verified=True, pid=pid, request_id=request, scope_id=scope,
        input_hash=unsigned_identity(fixed[0].get('input_hash')), backend_scope=proof,
        request_success=ends[0]['success'], solver_success=solves[0]['success'],
        initial_planning=request == 1)


def quadrature_samples(candidate, intervals):
    """Geometric evaluation at a declared per-piece grid, not cost replay."""
    elapsed = 0.
    for part_index, part in enumerate(candidate['parts']):
        within = 0.
        for piece_index, piece in enumerate(part['pieces']):
            for index in range(intervals+1):
                local = piece['T'] * index/intervals
                p, v = polynomial(piece, local)
                gear = int(part['gear'])
                yield dict(part=part_index, piece=piece_index, sample=index,
                    local_t=local, q=within+local, t=elapsed+within+local,
                    query_time=candidate['start_time']+elapsed+within+local,
                    x=float(p[0]), y=float(p[1]), vx=float(v[0]), vy=float(v[1]),
                    speed=float(np.linalg.norm(v[:2])), yaw=math.atan2(gear*float(v[1]), gear*float(v[0])), gear=gear)
            within += float(piece['T'])
        elapsed += float(part['duration'])


def acceleration(piece, t):
    c = np.asarray(piece['coeffs'], dtype=float)
    a = np.zeros(c.shape[0])
    for power in range(2, 8):
        a += power*(power-1)*t**(power-2)*c[:, 7-power]
    return a


def wheel_values(candidate, sample, parameters):
    if parameters is None:
        return None
    p = candidate['parts'][sample['part']]['pieces'][sample['piece']]
    a = acceleration(p, sample['local_t'])
    vx, vy = sample['vx'], sample['vy']
    # Native B_h=[0,-1;1,0], IsNotFeasibie speed and squared-speed floors.
    omega = (vx*float(a[1])-vy*float(a[0]))/max(1e-8, vx*vx+vy*vy)
    signed_speed = sample['gear']*max(1e-4, sample['speed'])
    half = parameters['wheel_base_m']*.5*omega
    return ((signed_speed-half)/parameters['wheel_radius_m'],
            (signed_speed+half)/parameters['wheel_radius_m'])


def wheel_parameters(trial, manifest):
    params = load_json(trial/'effective_real_params.json')
    prefix = '/remani_planner_node/mm/mobile_base_'
    keys = [('wheel_base_m','wheel_base'), ('wheel_radius_m','wheel_radius'), ('wheel_limit_rad_s','max_wheel_omega')]
    values = {name:params.get(prefix+key) for name,key in keys}
    build = Path(manifest['build'])
    attestation = load_json(build/'build_manifest.json')
    source = build/'source/poly_traj_optimizer.cpp'
    expected = attestation.get('isolated_source_sha256', {}).get('poly_traj_optimizer.cpp')
    text = source.read_text()
    markers = ['B_h_ << 0.0, -1.0,', '1.0,  0.0;', 'double feas_tol_percent_ = 0.05;',
        'double wheel_omega_left = (2.0 * singul * v_norm - mobile_base_wheel_base_ * omega)',
        'double wheel_omega_right = (2.0 * singul * v_norm + mobile_base_wheel_base_ * omega)',
        'if (v_norm < 1e-4)', 'if (vTv < 1e-8)']
    proven = expected == digest(source) and all(marker in text for marker in markers)
    valid = proven and all(isinstance(v,(int,float)) and math.isfinite(v) and v > 0 for v in values.values())
    evidence = dict(known=valid, source=str(source), source_sha256=digest(source),
        source_attestation_verified=expected == digest(source), formula_markers_verified=proven,
        parameter_source=str(trial/'effective_real_params.json'), parameter_sha256=digest(trial/'effective_real_params.json'),
        exact_parameter_keys=[prefix+key for _,key in keys], unknown_reason=None if valid else 'parameter_or_formula_provenance_unknown')
    if not valid:
        return None, evidence
    values['native_final_tolerance_fraction'] = .05
    values['native_final_limit_rad_s'] = values['wheel_limit_rad_s']*1.05
    evidence.update(values)
    return values, evidence


def scan(candidate, frame, samples, parameters):
    counts = Counter()
    minimum, first, wheel_l, wheel_r = None, None, None, None
    total, low_speed = 0, 0
    for sample in samples:
        total += 1
        low_speed += sample['speed'] < 1e-5
        value = evaluate_pose(frame, sample['x'], sample['y'], sample['yaw'], 0.)
        counts[value['category']] += 1
        if value['gap'] is not None and (minimum is None or value['gap'] < minimum['gap']):
            minimum = dict(**sample, **value)
        if first is None and value['category'] in ('invalid','collision','margin_only'):
            first = dict(**sample, **value)
        wheels = wheel_values(candidate, sample, parameters)
        if wheels is not None:
            for side, value in enumerate(wheels):
                item = dict(value_rad_s=value, abs_rad_s=abs(value), t=sample['t'], part=sample['part'], piece=sample['piece'])
                if side == 0 and (wheel_l is None or abs(value) > wheel_l['abs_rad_s']): wheel_l = item
                if side == 1 and (wheel_r is None or abs(value) > wheel_r['abs_rad_s']): wheel_r = item
    return dict(samples=total, low_speed_heading_samples=low_speed, static_categories=dict(counts),
        min_static_gap_m=minimum['gap'] if minimum else None, worst_static=minimum,
        first_static_failure=first, left_wheel_peak=wheel_l, right_wheel_peak=wheel_r,
        static_all_sampled_clear=first is None, continuous_safety_proven=False)


def audit_one(trial, path, native, parameters):
    candidate = load_json(path)
    if candidate.get('schema') != 'delivery_candidate_v1' or candidate.get('finished') is not True:
        raise ValueError('unfinished or unknown candidate')
    proof = identity(candidate, path, native)
    results, snapshots = {}, {}
    for name in ('solve_frame', 'accept_frame'):
        frame, snapshot = frozen_frame(trial/'delivery_diagnostics', candidate[name])
        if len(frame['obstacles']):
            raise ValueError('static-only full-curve experiment has dynamic records')
        snapshot.update(fresh_at_recorded_check=fresh(frame,candidate[name]['now']), recorded_now=candidate[name]['now'])
        snapshots[name] = snapshot
        results[name] = {}
        for label, samples in [('K8_projected_geometry', quadrature_samples(candidate,8)),
                ('K16_native_base_grid_geometry_only', quadrature_samples(candidate,16)),
                ('gate_20ms',candidate_samples(candidate,'dense',.02)),
                ('diagnostic_5ms',candidate_samples(candidate,'dense',.005))]:
            results[name][label] = scan(candidate, frame, samples, parameters)
    gate = results['accept_frame']['gate_20ms']
    replay_accepted = snapshots['accept_frame']['fresh_at_recorded_check'] and gate['static_all_sampled_clear']
    return dict(trial=trial.name,candidate=path.name,candidate_sha256=digest(path),identity=proof,
        duration_s=sum(float(p['duration']) for p in candidate['parts']), recorded_accepted=candidate['accepted'],
        recorded_first_failure=candidate.get('failure'), snapshots=snapshots,
        online_static_acceptance_agrees=replay_accepted == candidate['accepted'], scans=results)


def plot(report, target):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(10,4.6),constrained_layout=True)
    labels, gate, fine, coarse = [], [], [], []
    for item in report['candidates']:
        if item['identity']['initial_planning']:
            continue  # All initial candidates remain audited in the JSON/TXT.
        name = item['trial'].replace('static_coupled_seed','s').replace('_admission_',' ')
        labels.append(name+' r'+str(item['identity']['request_id']))
        scans = item['scans']['accept_frame']
        gate.append(scans['gate_20ms']['min_static_gap_m']*1000)
        fine.append(scans['diagnostic_5ms']['min_static_gap_m']*1000)
        coarse.append(scans['K8_projected_geometry']['min_static_gap_m']*1000)
    x = np.arange(len(labels))
    ax.scatter(x-.15,coarse,label='K8 projected-geometry grid',marker='^',s=40)
    ax.scatter(x,gate,label='20 ms gate sampling',marker='o',s=35)
    ax.scatter(x+.15,fine,label='5 ms diagnostic sampling',marker='x',s=40)
    ax.axhline(0,color='black',lw=.8)
    ax.set(xticks=x,xticklabels=labels,ylabel='Minimum gap = projected clearance - margin [mm]',
        title='5 replanning candidates (4 initial candidates audited, omitted here)')
    ax.tick_params(axis='x',rotation=35)
    ax.legend(fontsize=8)
    fig.savefig(target,dpi=150)
    plt.close(fig)


def report(root,out,plot_image=False):
    root,out=Path(root).resolve(),Path(out).resolve()
    if out.exists() or out == root or out in root.parents:
        raise ValueError('output must be a fresh separate directory')
    complete=load_json(root/'complete.json')
    if complete.get('complete') is not True or len(complete.get('completed',[])) != 4:
        raise ValueError('expected completed four-job batch')
    trials=[root/job['trial'] for job in complete['completed']]
    assert_plot_quiet(dict(trials=[dict(run_status=load_json(t/'run_status.json')) for t in trials]))
    audits,inputs,provenance=[],{},{}
    for trial in trials:
        manifest=load_json(trial/'manifest.json')
        native,timeline=read_records(trial,'dual_timeline.jsonl',True)
        inputs[str(timeline)]=digest(timeline)
        for filename in ('manifest.json','run_status.json','effective_real_params.json'):
            inputs[str(trial/filename)]=digest(trial/filename)
        parameters,provenance[trial.name]=wheel_parameters(trial,manifest)
        for path in sorted((trial/'delivery_diagnostics/candidates').glob('*.json')):
            item=audit_one(trial,path,native,parameters)
            audits.append(item)
            inputs[str(path)]=item['candidate_sha256']
            for snapshot in item['snapshots'].values(): inputs[snapshot['path']]=snapshot['sha256']
    if len(audits)!=9: raise ValueError('expected all 9 recorded candidates, got '+str(len(audits)))
    output=dict(schema='delivery_fullcurve_audit_v1',root=str(root),candidate_count=len(audits),
        input_sha256=inputs,wheel_model_provenance=provenance,candidates=audits,
        limitations=['No re-optimization, ROS, retiming, acceptance or production code changes.',
            'All nine saved final candidates, not all optimizer requests or accepted L-BFGS iterates; timeout/wheel-rejected candidates may not be saved.',
            'Recorded disk-union and field are not independent physical ground truth or native 3D collision replay.',
            '20 ms matches block-wise final projected gate sampling; 5 ms is diagnostic only, neither proves continuous-time clearance.',
            'K8 is added projected-geometry objective quadrature; native constrain_points_perPiece is 16 and adds extra feasibility samples. Grid geometry checks are not cost/gradient replay.',
            'Dynamic records are required empty; no dynamic objective clock equivalence is claimed.',
            'Yaw is native velocity-derived atan2(gear*vy,gear*vx), including low-speed/signed-zero behavior.',
            'Wheel peaks are sampled native-formula diagnostics, not exact extrema or all-joint feasibility certification.'])
    out.mkdir(parents=True,exist_ok=False)
    (out/'summary.json').write_text(json.dumps(output,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    lines=['REMANI已交付/拒收候选的整条轨迹复核（只读离线实验）',
        '共9候选，含4条初始规划；不是9次失败，也不覆盖超时过程中的全部迭代。',
        'gap=同记录场下完整机器人投影净距−安全余量。最坏采样gap与首次失败gap分开记录。',
        '20ms复现投影终验采样；5ms只作更密诊断，不证明连续时间安全；K8仅指新增投影代价配点。','']
    for item in audits:
        scans=item['scans']; ident=item['identity']; first=item['recorded_first_failure']
        lines.append('{} request={} / initial={} / accepted={} / exact_identity={} / online_agrees={}'.format(
            item['trial'],ident['request_id'],ident['initial_planning'],item['recorded_accepted'],ident['verified'],item['online_static_acceptance_agrees']))
        lines.append('  记录首失败gap={} m'.format(first.get('gap') if isinstance(first,dict) else None))
        for frame in ('solve_frame','accept_frame'):
            for mode in ('K8_projected_geometry','K16_native_base_grid_geometry_only','gate_20ms','diagnostic_5ms'):
                s=scans[frame][mode]; worst=s['worst_static'];l,r=s['left_wheel_peak'],s['right_wheel_peak']
                lines.append('  {} {}: min_gap={:.9f}m t={:.6f}s; margin_bad={}; sampled |left/right|={}/{}rad/s'.format(
                    frame,mode,s['min_static_gap_m'],worst['t'],s['static_categories'].get('margin_only',0),
                    '{:.6f}'.format(l['abs_rad_s']) if l else 'unknown','{:.6f}'.format(r['abs_rad_s']) if r else 'unknown'))
    special=next(a for a in audits if a['trial']=='static_coupled_seed12_admission_on' and a['identity']['request_id']==4)
    f=special['recorded_first_failure'];a=special['scans']['accept_frame']
    lines.extend(['','重点纠正：seed12 ON request4 的首失败gap={:.9f}m，不是整条轨迹最大缺口。'.format(f['gap']),
        '整条轨迹最坏采样gap：20ms={:.9f}m，5ms={:.9f}m；K8最坏gap={:.9f}m。'.format(
            a['gate_20ms']['min_static_gap_m'],a['diagnostic_5ms']['min_static_gap_m'],a['K8_projected_geometry']['min_static_gap_m']),
        '以上只说明该候选在同记录几何下的违规分布，不能单独区分优化软约束不足、模型差异或可行解是否存在。','',
        '完整来源、PID/request/scope、快照payload hash与SHA256在summary.json。旧记录未修改。'])
    (out/'整轨迹复核结论.txt').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    if plot_image: plot(output,out/'minimum_gaps.png')
    changed=[path for path,value in inputs.items() if digest(Path(path))!=value]
    if changed: raise RuntimeError('input changed while auditing: '+repr(changed))
    (out/'input_integrity.json').write_text(json.dumps(dict(all_unchanged=True,files=len(inputs)),indent=2)+'\n')
    return output


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--plot',action='store_true')
    args=parser.parse_args()
    answer=report(args.root,args.out,args.plot)
    print(json.dumps(dict(out=str(args.out),candidates=answer['candidate_count']),ensure_ascii=False))
