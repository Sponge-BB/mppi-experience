#!/usr/bin/env python3
"""Post-run static-only replay of clearance solver states, NOT references."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re

import numpy as np

import recheck_delivery_diag as replay
from report_domain_recovery import read_json, read_records, digest, candidate_records, assert_plot_quiet, show

EPS = 1e-8
NOTES = [
    '静态单 seed 诊断；新增捕获 I/O，不能证明速度或动态安全改善。',
    'OFF/ON 都包含域回退、终止负状态 fail-closed、额外扫描及原始 deadline 门；旧批次不是精确对照。',
    '重建的是求解器状态，不是发布参考；failed_last_evaluated_trial_NOT_canonical_x 永不作为恢复后的 canonical 状态。',
    'K8 是逐 piece 配点；dense 是逐 block 的 0.02s 扫描。只重放同一冻结场景下的全体 body 圆盘，不调用优化器或动态预测。',
    '几何重放不等于完整原生校验；deadline、快照新鲜度、轮速和 latest 硬门分别保留。',
    '普通 solver/failure trace 的 L-BFGS/force_stop/final_cost 是 nominal-only；重启状态仅取 clearance_restart_end，force_stop 优先。',
    '源码中 result=1 (LBFGS_STOP) 是相对代价停止，不代表硬约束满足，也不能单凭此声称梯度收敛；它不是失败原因的唯一因果证明。',
    '固定边界可行性结论只适用于本次记录请求，不解释旧批次所有拒绝；margin 违反与物理碰撞不同。',
    '缺失或格式错误保留 null；没有 canonical after 时不计算改善，不把缺失 after 当成安全或零净空。',
]


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def positive_id(value):
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


def trajectory_view(phase, now=0.):
    """Convert native ascending rows into a sampling view, never a candidate."""
    blocks = phase.get('blocks')
    if not isinstance(blocks, list) or not blocks:
        raise ValueError('missing polynomial blocks')
    parts = []
    for block in blocks:
        if not isinstance(block, dict):
            raise ValueError('polynomial block must be an object')
        times = np.asarray(block['times'], dtype=float)
        c = np.asarray(block['coeffs'], dtype=float)
        gear = block.get('gear')
        if (block.get('basis') != 'ascending_rows_per_8' or times.ndim != 1 or not len(times) or
                not np.all(np.isfinite(times)) or np.any(times <= 0) or
                c.ndim != 2 or c.shape[0] != 8 * len(times) or c.shape[1] < 2 or not np.all(np.isfinite(c)) or
                not isinstance(gear, int) or isinstance(gear, bool) or gear not in (-1, 1)):
            raise ValueError('invalid ascending block/times/gear')
        duration = float(times.sum())
        if not math.isfinite(duration) or duration / .02 > 1e6:
            raise ValueError('duration exceeds online scan bound')
        pieces = [dict(T=float(T), basis='descending', coeffs=c[8*i:8*(i+1), :].T[:, ::-1].tolist())
                  for i, T in enumerate(times)]
        parts.append(dict(gear=gear, duration=duration, pieces=pieces))
    return dict(start_time=now if finite(now) else 0., parts=parts)


def replay_phase(phase, snapshot, now):
    name = phase.get('phase') if isinstance(phase, dict) else None
    result = dict(phase=name, state_kind='canonical_x' if name == 'restored_canonical_x' else (
        'noncanonical_failed_trial' if name == 'failed_last_evaluated_trial_NOT_canonical_x' else 'nominal_or_other'),
        error=None, sampling=None)
    try:
        view = trajectory_view(phase, now)
        if snapshot is None:
            raise ValueError('missing snapshot')
        m = replay.snapshot_arrays(snapshot)
        # The helper validates queried cells; additionally fail malformed grid
        # metadata before arithmetic, rather than producing an optimistic min.
        if not all(finite(m.get(k)) for k in ('gx', 'gy', 'res')) or m['res'] <= 0:
            raise ValueError('invalid grid metadata')
        sampling = {}
        for mode in ('dense', 'K8'):
            counts = Counter(); worst = None; boundaries = {}; invalid = False
            for sample in replay.candidate_samples(view, mode, .02):
                value = replay.evaluate_pose(m, sample['x'], sample['y'], sample['yaw'], 0., False)
                counts[value['category']] += 1
                invalid |= value['category'] in ('invalid', 'unknown') or not finite(value.get('gap'))
                row = dict(sample, **value)
                if finite(value.get('gap')) and (worst is None or value['gap'] < worst['gap']):
                    worst = row
                if sample['part'] == 0 and abs(sample['q']) < 1e-12:
                    boundaries['initial'] = row
                if sample['part'] == len(view['parts']) - 1 and abs(sample['q'] - view['parts'][-1]['duration']) < 1e-10:
                    boundaries['final'] = row
            sampling[mode] = dict(valid=not invalid, samples=sum(counts.values()), counts=dict(counts),
                                  min_gap=worst['gap'] if worst and not invalid else None,
                                  min_physical=worst['physical'] if worst and not invalid else None,
                                  worst=worst, fixed_request_boundaries=boundaries)
        try:
            fresh = replay.fresh(m, now) if finite(now) else None
        except (KeyError, TypeError, ValueError):
            fresh = None
        result.update(sampling=sampling, snapshot_fresh=fresh,
                      boundary_feasibility={key: row['gap'] >= -EPS if finite(row.get('gap')) else None
                        for key, row in sampling['dense']['fixed_request_boundaries'].items()})
    except (KeyError, TypeError, ValueError, IndexError, OverflowError, AttributeError) as error:
        result['error'] = type(error).__name__ + ': ' + str(error)
    return result


def restart_status(row):
    if row is None:
        return 'unknown'
    force, code = row.get('force_stop'), row.get('lbfgs_result')
    if not isinstance(force, int) or isinstance(force, bool):
        return 'unknown'
    if force == 3:
        return 'timeout'
    if isinstance(force, int) and not isinstance(force, bool) and force != 0:
        return 'forced_stop'
    if force != 0 or not isinstance(code, int) or isinstance(code, bool):
        return 'unknown'
    return 'negative_result' if code < 0 else 'nonnegative_no_forced_stop'


def read_state_records(diagnostic):
    directory = diagnostic / 'clearance_recovery'
    if not directory.is_dir():
        return None
    rows = []
    for path in sorted(directory.glob('*.json')):
        row = dict(file=str(path.relative_to(diagnostic)), sha256=digest(path), error=None, payload=None)
        match = re.fullmatch(r'([0-9]+)_([0-9]+)\.json', path.name)
        row['pid'] = int(match.group(1)) if match and int(match.group(1)) > 1 else None
        try:
            payload = read_json(path)
            if not isinstance(payload, dict) or payload.get('schema') != 'clearance_recovery_v1':
                raise ValueError('unknown clearance solver-state schema')
            row['payload'] = payload
        except (OSError, ValueError, TypeError) as error:
            row['error'] = type(error).__name__ + ': ' + str(error)
        rows.append(row)
    return rows


def identity_proof(row, records, native):
    payload = row['payload'] or {}
    pid, request, scope = row['pid'], payload.get('request_id'), payload.get('scope_id')
    if pid is None or not positive_id(request) or not positive_id(scope):
        return None, 'missing_filename_pid_or_positive_request_scope'
    matching_records = [r for r in records if r['pid'] == pid and (r['payload'] or {}).get('request_id') == request]
    same = [r for r in native if r.get('pid') == pid and r.get('request_id') == request]
    coupling = [r for r in same if r.get('event') == 'coupling_scope']
    begins = [r for r in same if r.get('event') == 'request_begin']
    ends = [r for r in same if r.get('event') == 'request_end']
    if (len(matching_records) != 1 or len(coupling) != 1 or coupling[0].get('scope_id') != scope or
            len(begins) != 1 or len(ends) != 1):
        return None, 'ambiguous_or_mismatched_record_request_scope'
    if not finite(begins[0].get('mono')) or not finite(ends[0].get('mono')) or ends[0]['mono'] < begins[0]['mono']:
        return None, 'invalid_request_clock'
    for event in same:
        if event.get('event', '').startswith('clearance_') and (
                not finite(event.get('mono')) or not begins[0]['mono'] <= event['mono'] <= ends[0]['mono']):
            return None, 'clearance_event_outside_identified_request'
    return same, 'filename_pid+unique_request+exact_coupling_scope_id'


def only(rows, event):
    matches = [r for r in rows or [] if r.get('event') == event]
    return matches[0] if len(matches) == 1 else None


def unique_phase(phases, name):
    matches = [p for p in phases if p['phase'] == name]
    return matches[0] if len(matches) == 1 else None


def gap(phase, mode='dense'):
    return ((phase or {}).get('sampling') or {}).get(mode, {}).get('min_gap')


def compare_scan(phase, event, enabled):
    dense = ((phase or {}).get('sampling') or {}).get('dense') or {}
    valid = (event is not None and event.get('valid') is True and event.get('enabled') == int(enabled)
             and finite(event.get('min_gap')) and finite(dense.get('min_gap')))
    delta = dense['min_gap'] - event['min_gap'] if valid else None
    return dict(recorded_event=event, independent_min_gap=dense.get('min_gap'), delta=delta,
                agrees=bool(abs(delta) <= 1e-7 and dense.get('samples') == event.get('samples')) if valid else None,
                note='only a unique valid completed dense scan is comparable; no deadline replay')


def boundary_obstruction(before, after):
    a = (((before or {}).get('sampling') or {}).get('dense') or {}).get('fixed_request_boundaries', {})
    b = (((after or {}).get('sampling') or {}).get('dense') or {}).get('fixed_request_boundaries', {})
    result = []
    for endpoint in ('initial', 'final'):
        if endpoint not in a or endpoint not in b or not finite(b[endpoint].get('gap')):
            continue
        delta = max(abs(a[endpoint][k] - b[endpoint][k]) for k in ('x', 'y', 'vx', 'vy'))
        if delta <= 1e-8 and b[endpoint]['gap'] < -EPS:
            result.append(dict(endpoint=endpoint, unchanged_pv_max_abs_delta=delta, sample=b[endpoint],
                               inference='first/last request p,v are fixed in this optimizer topology; this violated endpoint cannot be moved by interior variables'))
    return result


def audit_state(row, records, native, diagnostic, candidates):
    payload = row['payload'] or {}
    events, proof = identity_proof(row, records, native)
    errors = [row['error']] if row['error'] else []
    frame = payload.get('frame') or {}
    if not isinstance(frame, dict):
        frame = {}; errors.append('malformed frame')
    try:
        snapshot = replay.resolve_snapshot(diagnostic, frame)
    except (OSError, TypeError, ValueError, KeyError) as error:
        snapshot = None; errors.append('snapshot: ' + str(error))
    phases = [replay_phase(p, snapshot, frame.get('now')) for p in payload.get('phases', [])] if isinstance(payload.get('phases', []), list) else []
    before, after = unique_phase(phases, 'nominal'), unique_phase(phases, 'restored_canonical_x')
    end = only(events, 'clearance_restart_end')
    begin = only(events, 'clearance_restart_begin')
    status = restart_status(end)
    enabled = payload.get('enabled')
    identity = events is not None and isinstance(enabled, bool)
    canonical_status = (identity and after is not None and status == 'nonnegative_no_forced_stop' and begin is not None
                        and finite(begin.get('mono')) and finite(end.get('mono')) and begin['mono'] <= end['mono'])
    pre = gap(before)
    post = gap(after) if canonical_status else None
    pre_physical = (((before or {}).get('sampling') or {}).get('dense') or {}).get('min_physical')
    post_physical = (((after or {}).get('sampling') or {}).get('dense') or {}).get('min_physical') if canonical_status else None
    improvement = post - pre if finite(pre) and finite(post) else None
    matches = [c for c in candidates if (c['pid'], c['request_id'], c['scope_id']) ==
               (row['pid'], payload.get('request_id'), payload.get('scope_id'))] if candidates is not None and identity else None
    obstruction = boundary_obstruction(before, after) if canonical_status else []
    k8_after = gap(after, 'K8') if canonical_status else None
    explanation = None
    if finite(post) and post < -EPS:
        explanation = ('K8 配点仍违反静态 margin，不只是配点之间的漏检。' if finite(k8_after) and k8_after < -EPS else
                       'K8 配点通过但 dense 仍失败，存在配点间违反。' if finite(k8_after) else '缺少可判定的 K8 净空。')
        if obstruction:
            explanation += ' 固定请求边界存在前后不变的违反节点；仅调整内部变量无法消除此边界违反（基于拓扑的推断）。'
    return dict(file=row['file'], sha256=row['sha256'], pid=row['pid'], request_id=payload.get('request_id'),
                scope_id=payload.get('scope_id'), enabled=enabled, outcome=payload.get('outcome'), errors=errors,
                identity_certified=identity, identity_proof=proof, phases=phases,
                nominal_comparison=compare_scan(before, only(events, 'clearance_nominal'), enabled) if identity else None,
                restored_comparison=compare_scan(after, only(events, 'clearance_restored'), enabled) if identity else None,
                restart_begin=begin, restart_end=end, restart_status=status,
                canonical_x_recorded=after is not None, canonical_status_certified=canonical_status,
                independent_pre_gap=pre, independent_post_gap=post, independent_canonical_gap_delta=improvement,
                independent_pre_physical=pre_physical, independent_post_physical=post_physical,
                independent_post_K8_gap=k8_after, fixed_boundary_obstructions=obstruction, failure_explanation=explanation,
                frame=frame, recorded_native_qualified=payload.get('outcome') == 'native_qualified_pending_latest_gate'
                    if identity and isinstance(payload.get('outcome'), str) else None,
                actual_latest_candidates=matches,
                latest_hard_accepted=sum(c['hard_gate_state'] == 'accepted' for c in matches) if matches is not None else None)


def summarize_trial(trial):
    documents = {n: read_json(trial / n) for n in ('manifest.json', 'run_status.json', 'trial_audit.json', 'result.json')}
    manifest, status = documents['manifest.json'], documents['run_status.json']
    settings = manifest.get('coupling_experiment') or {}
    if status.get('returncode') is None or status.get('result_written') is not True or not isinstance(settings.get('clearance_recovery'), bool):
        raise ValueError('Incomplete trial or missing explicit treatment: ' + str(trial))
    native, timeline = read_records(trial, 'dual_timeline.jsonl', required=True)
    resource, resource_path = read_records(trial, 'solver_trace.jsonl')
    diagnostic = trial / 'delivery_diagnostics'
    records = read_state_records(diagnostic)
    candidate_error = None
    try:
        candidates = candidate_records(trial)
    except (OSError, ValueError, TypeError, AttributeError) as error:
        candidates = None; candidate_error = str(error)
    audited = [audit_state(r, records, native, diagnostic, candidates) for r in records] if records is not None else None
    events = Counter(r.get('event') for r in native)
    ends = [r for r in native if r.get('event') == 'clearance_restart_end']
    counts = dict(nominal_scan_events=events['clearance_nominal'], restored_scan_events=events['clearance_restored'],
        skipped_budget_events=events['clearance_skipped'], restart_begin=events['clearance_restart_begin'], restart_end=len(ends),
        restart_status=dict(Counter(restart_status(r) for r in ends)), deadline_reject=events['clearance_deadline_reject'],
        state_files=len(audited) if audited is not None else None,
        canonical_x_records=sum(r['canonical_x_recorded'] for r in audited) if audited is not None else None,
        recorded_native_qualified=sum(r['recorded_native_qualified'] is True for r in audited) if audited is not None else None,
        outcome_counts=dict(Counter(r['outcome'] for r in audited if r['outcome'] is not None)) if audited is not None else None,
        latest_hard_accepted=sum(c['hard_gate_state'] == 'accepted' for c in candidates) if candidates is not None else None,
        latest_hard_rejected=sum(c['hard_gate_state'] == 'rejected' for c in candidates) if candidates is not None else None)
    hashes = {n: digest(trial / n) for n in documents}
    for path in (timeline, resource_path):
        if path is not None: hashes[path.name] = digest(path)
    optimizer = [r for r in resource or [] if r.get('event') == 'end' and r.get('stage') == 'optimizer' and r.get('label') == 'normal']
    return dict(trial=trial.name, source=str(trial), clearance_recovery=settings['clearance_recovery'],
                run_status=status, audit=documents['trial_audit.json'], outcome=documents['result.json'],
                settings=settings, input_sha256=hashes, counts=counts, state_records=audited,
                actual_latest_candidates=candidates, native_optimizer_success_flags=[r.get('success') for r in optimizer] if resource is not None else None,
                latest_candidate_error=candidate_error,
                ordinary_trace_solver_fields_scope='nominal-only; never used to classify restart')


def build_report(root):
    root = Path(root).resolve()
    complete, plan = read_json(root / 'complete.json'), read_json(root / 'plan.json')
    jobs = complete.get('completed')
    if complete.get('complete') is not True or not isinstance(jobs, list) or len(jobs) != 2:
        raise ValueError('Exactly two completed trials required; no active-batch analysis')
    names = [j.get('trial') for j in jobs]
    if any(not isinstance(n, str) or Path(n).name != n or n in ('.', '..') for n in names) or len(set(names)) != 2:
        raise ValueError('Invalid completed trial names')
    trials = sorted([summarize_trial(root / n) for n in names], key=lambda r: r['clearance_recovery'])
    if {r['clearance_recovery'] for r in trials} != {False, True}:
        raise ValueError('Distinct OFF/ON trials required')
    return dict(schema='clearance_recovery_report_v1', created=datetime.now(timezone.utc).isoformat(), root=str(root),
                plan=plan, complete=complete, diagnostic_only=True, performance_comparison=False, notes=NOTES, trials=trials)


def report_text(report):
    lines = ['静态净空恢复诊断：求解器状态重放，不是发布参考']
    for trial in report['trials']:
        c, result = trial['counts'], trial['outcome']
        lines += ['', '%s：结果=%s，保守投影碰撞=%s，审计有效=%s' % ('ON' if trial['clearance_recovery'] else 'OFF',
            show(result.get('reason')), show(result.get('collision_conservative_projection')), show(trial['audit'].get('valid_experiment'))),
            '重启 begin/end=%d/%d，预算跳过=%d，状态=%s；canonical=%s，记录原生资格=%s，latest 硬门接受/拒绝=%s/%s' %
            (c['restart_begin'], c['restart_end'], c['skipped_budget_events'], json.dumps(c['restart_status']),
             show(c['canonical_x_records']), show(c['recorded_native_qualified']), show(c['latest_hard_accepted']), show(c['latest_hard_rejected']))]
        for state in trial['state_records'] or []:
            if trial['clearance_recovery'] and finite(state['independent_pre_gap']) and state['independent_pre_gap'] < -EPS:
                lines += ['请求%s：dense gap 前/后=%s/%s m，K8 后=%s m，重启=%s；latest 接受=%s' %
                    (show(state['request_id']), show(state['independent_pre_gap']), show(state['independent_post_gap']),
                     show(state['independent_post_K8_gap']), state['restart_status'], show(state['latest_hard_accepted']))]
                if state['failure_explanation']: lines.append(state['failure_explanation'])
                for phase_name in ('nominal', 'restored_canonical_x'):
                    phase = unique_phase(state['phases'], phase_name)
                    bounds = (((phase or {}).get('sampling') or {}).get('dense') or {}).get('fixed_request_boundaries', {})
                    if phase is not None:
                        lines.append('%s：初/末边界 gap=%s/%s m；边界几何可行=%s。' % (phase_name,
                            show(bounds.get('initial', {}).get('gap')), show(bounds.get('final', {}).get('gap')),
                            show(phase.get('boundary_feasibility'))))
                after = unique_phase(state['phases'], 'restored_canonical_x')
                worst = (((after or {}).get('sampling') or {}).get('dense') or {}).get('worst')
                if worst:
                    lines.append('canonical dense 最差样本：t=%s s，block=%s，piece=%s，body=%s。' %
                                 tuple(show(worst.get(k)) for k in ('t', 'part', 'piece', 'body_index')))
                    lines.append('canonical 最小物理净空=%s m；正物理净空仍可能不满足 0.10m margin。' % show(worst.get('physical')))
    return '\n'.join(lines + ['', '解释边界：'] + NOTES) + '\n'


def plot_report(report, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rows = [r for t in report['trials'] if t['clearance_recovery'] for r in t['state_records'] or []
            if finite(r['independent_pre_gap']) and r['independent_pre_gap'] < -EPS]
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    for i, row in enumerate(rows):
        before = row['independent_pre_physical']
        if finite(before):
            ax.bar(i - .18, before * 100., .34, color='tab:blue', label='Before' if i == 0 else None)
            ax.text(i - .18, before * 100. + .2, '%.2f' % (before * 100.), ha='center')
        after = row['independent_post_physical']
        if finite(after):
            ax.bar(i + .18, after * 100., .34, color='tab:orange', label='After (canonical)' if i == 0 else None)
            ax.text(i + .18, after * 100. + .2, '%.2f' % (after * 100.), ha='center')
        else:
            ax.annotate('no certified canonical after', (i, (before or 0.) * 100.), xytext=(5, 8), textcoords='offset points', fontsize=8)
    ax.axhline(10., color='black', linestyle='--', linewidth=1, label='REQUIRED 10 cm')
    ax.set_xticks(range(len(rows))); ax.set_xticklabels(['request ' + str(r['request_id']) for r in rows])
    ax.set_ylabel('Full-body physical clearance (cm)'); ax.set_title('Solver candidate, not executed trajectory', fontsize=11)
    ax.set_ylim(min(ax.get_ylim()[0], 0.), max(ax.get_ylim()[1] * 1.1, 12.))
    if rows: ax.legend()
    else: ax.text(.5, .5, 'No recorded negative-gap trigger', transform=ax.transAxes, ha='center')
    fig.tight_layout(); fig.savefig(path, dpi=130); plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', required=True, type=Path); parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--plot', action='store_true', help='One PNG only after completed trials and quiet ROS process checks')
    args = parser.parse_args(argv)
    root, out = args.root.resolve(), args.out.resolve()
    if out.exists() or out == root or out in root.parents:
        parser.error('--out must be a separate fresh directory')
    result = build_report(root)
    if args.plot: assert_plot_quiet(result)
    out.mkdir(parents=True, exist_ok=False)
    (out / 'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    (out / '实验结论.txt').write_text(report_text(result), encoding='utf-8')
    if args.plot: plot_report(result, out / 'clearance_before_after.png')
    print(json.dumps(dict(out=str(out)), ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
