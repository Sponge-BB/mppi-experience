#!/usr/bin/env python3
"""Post-run fixed-input budget/scaling audit; no solver or vehicle I/O.

Primary results require an in-budget final-validation marker and an ordinary
successful solver return. Posterior accepted iterates never count as delivery.
No plotting or geometry reconstruction is performed at import.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from audit_backend_kinematics import analyze_candidate, parameters as wheel_parameters
from audit_delivery_fullcurve import frozen_frame
from audit_delivery_iterates import analyze_phase, canonical_records, finite, vector_hash
from report_domain_recovery import assert_plot_quiet, digest, read_json, read_records


HERE = Path(__file__).resolve().parent
JOBS = ((11, 1), (12, 2), (11, 2), (12, 1))
PHASES = ('budget_original_060', 'budget_reference_060', 'budget_extended_300',
          'budget_scaled_300', 'budget_repeat_060')
BUDGETS = (.6, .6, 3., 3., .6)
TREATMENTS = ('original_stop', 'feasibility_stop', 'extended_budget',
              'extended_budget_scaled', 'repeat_feasibility_stop')
SHORT = ('Original 0.6s', 'Veto 0.6s', 'Veto 3.0s', 'Scaled 3.0s', 'Repeat 0.6s')
CONTEXT_FIELDS = ('effective_initial_states', 'effective_final_states', 'raw_seq',
                  'esdf_seq', 'frame_now', 'frame_sequence', 'prediction_hash',
                  'sim', 'wheel_limit')
LIMITATIONS = [
    '四个预先指定的新暂停输入，每输入五臂；不是旧请求的精确重放，也不是总体成功率的统计估计。',
    '主指标是暂停条件下后端按时可交付，必须最终正常返回、原生与完整投影检查通过且在原预算内；不是在线发布率。',
    '所有臂共同新增最终完整投影与截止检查；原停止臂也不是生产二进制逐字节对照。',
    '固定D只改变求解器坐标，不改变物理目标、初始曲线、边界、轮限制、外形或投影安全门。',
    '原停止对照与可行性停止0.6秒分开；预算效应仅比较veto0.6与veto3秒，尺度效应仅比较相同3秒。',
    '初始及接受迭代的原生与投影重建是预算外后验审计，不把首次后验双过的时间说成实时交付时间。',
    '最终候选0/5/10厘米余量敏感性仅复用同一外形、同一地图，不改变主通过率；原生失败不会因投影放宽变成可交付。',
    '3秒是诊断上限，未改生产默认；本轮不能证明动态避障、快慢层交接或实车表现改善。',
    '未捕获、未完成及缺证据均保留在四请求分母中并标未知；未知不是失败，更不是成功。',
    '重复0.6秒仅检测顺序与截止引起的差异，不是新增独立样本；计时包括门检查与固定D准备，后验审计除外。',
    '冻结记录、首x/f/g与哈希证据不等同于完整原生地图内存逐字节导出；原生安全门仍有其自身采样范围。',
]


def unique(rows, kind, phase=None):
    found = [r for r in rows or [] if r.get('kind') == kind and
             (phase is None or r.get('phase') == phase)]
    return found[0] if len(found) == 1 else None


def integer(value):
    return isinstance(value, int) and not isinstance(value, bool)


def optional_json(path):
    return read_json(path) if Path(path).is_file() else None


def unknown_phase(label, budget, reason):
    return dict(phase=label, budget_s=budget, evidence_valid=False,
                paused_backend_deliverable=None, missing_reason=reason,
                decision_elapsed_s=None, lbfgs_wall_ms=None,
                posterior_not_delivery=True)


def gate_evidence(rows, label, budget):
    gate = unique(rows, 'feasibility_stop_summary', label)
    if gate is None:
        return dict(evidence_valid=False, reason='missing or duplicate gate summary', record=None)
    counters = ('checks', 'blocked_count', 'allowed_count', 'native_pass_count',
                'projected_pass_count', 'budget_expired_count')
    valid = all(integer(gate.get(k)) and gate[k] >= 0 for k in counters)
    fatal_known = isinstance(gate.get('fatal'), bool)
    if valid and fatal_known:
        explained = gate['blocked_count'] + gate['allowed_count']
        valid = explained <= gate['checks'] if gate['fatal'] else explained == gate['checks']
        valid = valid and all(gate[k] <= gate['checks'] for k in counters[1:])
    enabled = label != PHASES[0]
    valid = valid and fatal_known and gate.get('enabled') is enabled
    valid = valid and gate.get('original_budget_s') == budget
    valid = valid and gate.get('effective_variant') == ('cost_stop_veto' if enabled else 'original_cost_stop')
    times = ('native_wall_s', 'projected_wall_s', 'gate_wall_s', 'feasibility_decision_elapsed_s')
    valid = valid and all(finite(gate.get(k)) and gate[k] >= 0. for k in times)
    if not enabled:
        valid = valid and all(gate.get(k) == 0 for k in counters)
    return dict(evidence_valid=bool(valid), record=gate, enabled=enabled,
                activated=gate.get('checks', 0) > 0,
                decision_elapsed_s=gate.get('feasibility_decision_elapsed_s'),
                gate_wall_s=gate.get('gate_wall_s'), not_online_acceptance=True)


def primary_endpoint(marker, failure, audit, budget, phase_evidence_valid):
    """Fail closed: an audited intermediate or late candidate cannot qualify.

    A valid negative final marker may have unknown native/projected fields when
    the solver never reached validation (e.g. timeout). This is a known failure,
    not a fabricated safety result. A successful marker requires all fields.
    """
    answer = dict(evidence_valid=False, paused_backend_deliverable=None,
                  reason='missing or invalid final-validation marker', marker=marker,
                  never_counts_posterior_iterates=True)
    if not phase_evidence_valid or not isinstance(marker, dict) or not isinstance(failure, dict):
        return answer
    summary = audit.get('solve_summary') or {}
    returned = audit.get('recursive_return') or {}
    success = marker.get('success')
    elapsed = marker.get('elapsed_s')
    lbfgs_result, force_stop = failure.get('lbfgs_result'), failure.get('force_stop')
    marker_valid = (marker.get('schema') == 'delivery_replay_v1' and
        marker.get('validation_schema') == 'budget_final_validation_v1' and marker.get('valid') is True and
        isinstance(success, bool) and finite(elapsed) and elapsed >= 0. and
        marker.get('budget_s') == budget and integer(lbfgs_result) and integer(force_stop) and
        marker.get('solver_result_known') is True and marker.get('solver_result') == lbfgs_result and
        marker.get('force_stop') == force_stop and
        success is failure.get('success') is summary.get('native_return_success') is returned.get('success'))
    if not marker_valid:
        return dict(answer, reason='final marker disagrees with exact solver return/budget')
    if not success:
        return dict(answer, evidence_valid=True, paused_backend_deliverable=False,
                    reason=marker.get('reason', summary.get('reason')),
                    decision_elapsed_s=elapsed, final_safety_known=False)
    candidates = audit.get('candidates') or []
    last = candidates[-1] if candidates else {}
    final_hash = marker.get('final_x_hash')
    identity = (isinstance(final_hash, str) and bool(final_hash) and
                final_hash == last.get('candidate_x_hash'))
    positive_valid = (lbfgs_result >= 0 and force_stop == 0 and elapsed < budget and identity and
        marker.get('last_canonical_x_hash') == final_hash and
        marker.get('last_accepted_x_hash') == final_hash and marker.get('accepted_identity_match') is True and
        marker.get('cache_identity_known') is True and marker.get('cache_identity_match') is True and
        marker.get('output_constructed') is True and isinstance(marker.get('output_hash'), str) and
        bool(marker.get('output_hash')) and
        marker.get('native_known') is True and marker.get('projected_known') is True and
        marker.get('native_pass') is True and marker.get('projected_pass') is True and
        marker.get('in_budget') is True and marker.get('allowed') is True and
        last.get('native_IsTrajSafe_pass') is True and last.get('frozen_projected_pass') is True)
    if not positive_valid:
        return dict(answer, reason='successful return lacks exact final identity or in-budget dual-check proof',
                    identity_matches_last_canonical=identity)
    return dict(answer, evidence_valid=True, paused_backend_deliverable=True,
                reason='ordinary return and exact final dual-check before original deadline',
                decision_elapsed_s=elapsed, identity_matches_last_canonical=True,
                final_safety_known=True)


def margin_sensitivity(last, native_return_success):
    """Reuse the same physical minimum; changing margin does not move geometry."""
    geometry = last.get('frozen_projected_20ms') if isinstance(last, dict) else None
    worst = geometry.get('worst_static') if isinstance(geometry, dict) else None
    physical = worst.get('physical') if isinstance(worst, dict) else None
    if not finite(physical):
        return dict(known=False, reason='missing full projected final minimum', changes_primary=False)
    cases = []
    for margin in (0., .05, .10):
        gap = physical - margin
        cases.append(dict(margin_m=margin, min_gap_m=gap, sampled_projection_pass=gap >= -1e-8,
                          native_return_success=native_return_success,
                          not_counted_as_delivery=True))
    return dict(known=True, same_shape_same_field=True, min_physical_clearance_m=physical,
                cases=cases, changes_primary=False, neither_radius_nor_map_correction_removed=True)


def phase_report(trial, label, budget, rows, failures, capture, frame, proof, params):
    audit = analyze_phase(trial, label, rows, capture, frame, proof)
    summary = audit.get('solve_summary') or {}
    matches = [r for r in failures or [] if r.get('kind') == 'solve' and r.get('label') == label]
    failure = matches[0] if len(matches) == 1 else None
    gate = gate_evidence(rows, label, budget)
    failure_valid = (isinstance(failure, dict) and
        str(failure.get('input_hash')) == str(capture.get('input_hash')) and
        failure.get('budget_s') == summary.get('budget_s') == budget and
        failure.get('success') is summary.get('native_return_success') and
        failure.get('reason') == summary.get('reason') and
        finite(failure.get('lbfgs_wall_ms')) and failure['lbfgs_wall_ms'] >= 0.)
    canonical = [r for r in rows if r.get('kind') == 'canonical_candidate' and r.get('phase') == label]
    last_audit = (audit.get('candidates') or [None])[-1]
    wheel = analyze_candidate(canonical[-1], capture, last_audit, params) if canonical and last_audit else None
    phase_valid = (audit.get('phase_evidence_valid') is True and failure_valid and
                   gate['evidence_valid'] and wheel is not None and wheel.get('evidence_valid') is True)
    marker = unique(rows, 'budget_final_validation', label)
    primary = primary_endpoint(marker, failure, audit, budget, phase_valid)
    scaling_start = unique(rows, 'coordinate_scaling_start', label)
    scaling_end = unique(rows, 'coordinate_scaling_end', label)
    return dict(phase=label, budget_s=budget, evidence_valid=bool(phase_valid and primary['evidence_valid']),
        paused_backend_deliverable=primary['paused_backend_deliverable'], primary=primary,
        lbfgs_wall_ms=failure.get('lbfgs_wall_ms') if failure else None,
        decision_elapsed_s=primary.get('decision_elapsed_s', gate.get('decision_elapsed_s')),
        native_return_success=summary.get('native_return_success'), solver_reason=summary.get('reason'),
        failure_record_count=len(matches), failure_record_agrees=bool(failure_valid),
        failure_record=failure, gate=gate,
        scaling_summary=dict(start=scaling_start, end=scaling_end), iterate_audit=audit,
        canonical_counts=audit.get('counts'), last_canonical_wheel_audit=wheel,
        final_margin_sensitivity=margin_sensitivity(last_audit, summary.get('native_return_success')),
        last_canonical_x_hash=vector_hash(canonical[-1].get('x')) if canonical else None,
        last_canonical_g_hash=vector_hash(canonical[-1].get('g')) if canonical else None,
        post_solver_audit_wall_s=(audit.get('native_audit_cost') or {}).get('post_solver_total_wall_s'),
        posterior_not_delivery=True)


def repeat_comparison(phases):
    if len(phases) != len(PHASES):
        return dict(known=False, reason='incomplete phases')
    first, repeat = phases[1], phases[4]
    first_audit, repeat_audit = first.get('iterate_audit') or {}, repeat.get('iterate_audit') or {}
    left, right = first_audit.get('solve_summary') or {}, repeat_audit.get('solve_summary') or {}
    return dict(known=first.get('evidence_valid') is True and repeat.get('evidence_valid') is True,
        same_last_x_hash=first.get('last_canonical_x_hash') == repeat.get('last_canonical_x_hash'),
        same_last_g_hash=first.get('last_canonical_g_hash') == repeat.get('last_canonical_g_hash'),
        same_reason=first.get('solver_reason') == repeat.get('solver_reason'),
        first_accepted=left.get('accepted_total'), repeat_accepted=right.get('accepted_total'),
        elapsed_difference_s=(repeat['decision_elapsed_s'] - first['decision_elapsed_s'])
            if finite(first.get('decision_elapsed_s')) and finite(repeat.get('decision_elapsed_s')) else None,
        not_independent_samples=True, deadline_may_change_accepted_iterate=True)


def trial_report(trial, job):
    hashes = {}
    manifest = optional_json(trial/'manifest.json') or {}
    status = optional_json(trial/'run_status.json') or {}
    for name in ('manifest.json', 'run_status.json', 'diagnostic_complete.json'):
        if (trial/name).is_file():
            hashes[name] = digest(trial/name)
    rows, replay = canonical_records(trial)
    failures, failure_path = read_records(trial, 'failure_trace.jsonl')
    timeline, timeline_path = read_records(trial, 'dual_timeline.jsonl')
    for path in (replay, failure_path, timeline_path):
        if path is not None:
            hashes[path.name] = digest(path)
    basic = dict(trial=trial.name, seed=job['seed'], ordinal=job['ordinal'], run_status=status,
        source=str(trial), input_sha256=hashes, captured=False, evidence_valid=False,
        phases=[unknown_phase(label, budget, 'missing or incomplete captured input')
                for label, budget in zip(PHASES, BUDGETS)], navigation_metrics_valid=False)
    captures = [r for r in rows or [] if r.get('kind') == 'capture']
    if len(captures) != 1:
        return dict(basic, capture_count=len(captures),
                    reason='no/ambiguous capture; normal preplanning is not diagnostic success')
    if any(r.get('schema') != 'delivery_replay_v1' for r in rows):
        raise ValueError('Unknown replay schema')
    capture = captures[0]
    frame, proof = frozen_frame(trial/'delivery_diagnostics', capture['frame'])
    if frame['obstacles']:
        raise ValueError('Static diagnostic contains moving actors')
    params, param_proof = wheel_parameters(trial)
    phases = [phase_report(trial, label, budget, rows, failures, capture, frame, proof, params)
              for label, budget in zip(PHASES, BUDGETS)]
    signatures = [p['iterate_audit']['first_signature'] for p in phases]
    same_first = (all(s.get('verified') is True for s in signatures) and
        len({(s.get('cost'), s.get('x_hash'), s.get('gradient_hash')) for s in signatures}) == 1)
    summaries = [p['iterate_audit']['solve_summary'] for p in phases]
    same_context = (all(bool(s) for s in summaries) and all(all(k in s and s[k] == summaries[0].get(k)
        for k in CONTEXT_FIELDS) for s in summaries) and all(s.get('sim') == s.get('sim_end') for s in summaries))
    observed = [r for r in rows if r.get('kind') == 'native_dynamic_observation']
    dynamic_valid = (bool(observed) and all(r.get('matches_entry') is True for r in observed) and
        len({(r.get('query_time_nsec'), r.get('content_hash'), r.get('overlay_enabled')) for r in observed}) == 1)
    complete = unique(rows, 'complete') or {}
    done = optional_json(trial/'diagnostic_complete.json') or {}
    flags = ('frozen_stable', 'native_dynamic_observed_stable', 'same_first_fg', 'audits_complete')
    complete_valid = all(complete.get(k) is True and done.get(k) is True for k in flags)
    no_publish = (complete.get('diagnostic_candidates_published') is False and
        done.get('diagnostic_candidates_published') is False and timeline is not None and
        not any(r.get('request_id') == capture.get('request_id') and r.get('event') in
                ('reference_publish', 'state_contract_reference_publish') for r in timeline))
    settings = manifest.get('coupling_experiment') or {}
    request_begins = [r for r in timeline or [] if r.get('event') == 'request_begin' and
                      r.get('request_id') == capture.get('request_id')]
    scope = dict(recorded_capture=job.get('captured') is True,
        seed=settings.get('seed') == job['seed'], ordinal=settings.get('refresh_ordinal') == job['ordinal'],
        actual_capture_ordinal=capture.get('refresh_ordinal') == job['ordinal'],
        paused=settings.get('diagnostic_pause') is True,
        phases=settings.get('diagnostic_phases') == list(PHASES),
        budgets=settings.get('diagnostic_budgets_s') == list(BUDGETS),
        treatment_order=settings.get('diagnostic_treatment_order') == list(TREATMENTS),
        weights_zero=settings.get('execution_weight') == 0. and settings.get('handover_weight') == 0.,
        seed_fix=settings.get('seed_parameterization_fix') is True,
        constraints_unchanged=settings.get('constraints_unchanged') is True,
        objective_unchanged=settings.get('objective_unchanged') is True,
        initialization_unchanged=settings.get('initialization_unchanged') is True,
        scaling_inside_budget=settings.get('scaling_preparation_inside_budget') is True,
        scaling_fixed=settings.get('scaling_fixed_per_solve') is True,
        paused_before_nominal=capture.get('all_passes_paused_before_nominal') is True,
        limits_not_relaxed=capture.get('constraints_relaxed') is False,
        replanning_request=len(request_begins) == 1 and request_begins[0].get('fsm') == 3)
    status_valid = (status.get('diagnostic_complete') is True and status.get('graph_audited') is True and
        status.get('production_hashes_unchanged') is True and status.get('build_manifest_unchanged') is True and
        status.get('required_process_failed') is False and status.get('runner_error') is None)
    runtime = manifest.get('experiment_sources_sha256') or {}
    runtime_proof = {name: dict(expected=h, observed=digest(trial/'runtime'/name)
        if (trial/'runtime'/name).is_file() else None) for name, h in runtime.items() if name.endswith('.py')}
    runtime_valid = bool(runtime_proof) and all(v['expected'] == v['observed'] for v in runtime_proof.values())
    trial_contract_valid = all((same_first, same_context, dynamic_valid, complete_valid, no_publish,
                                status_valid, runtime_valid, all(scope.values())))
    if not trial_contract_valid:
        for phase in phases:
            phase['evidence_valid'] = False
            phase['paused_backend_deliverable'] = None
            phase['primary']['trial_contract_invalid'] = True
    return dict(basic, captured=True, capture=capture, capture_count=1, complete=complete,
        evidence_valid=trial_contract_valid and all(p['evidence_valid'] for p in phases), phases=phases,
        independently_same_first_fg=same_first, same_frozen_recorded_context=same_context,
        native_dynamic_observations_valid=dynamic_valid, completion_flags_valid=complete_valid,
        no_captured_request_publication=no_publish, scope_checks=scope, status_evidence_valid=status_valid,
        runtime_python=runtime_proof, runtime_python_verified=runtime_valid, snapshot=proof,
        wheel_parameters=param_proof, repeat_check=repeat_comparison(phases))


def aggregate(trials):
    groups = []
    for index, (label, budget) in enumerate(zip(PHASES, BUDGETS)):
        phases = [t['phases'][index] for t in trials]
        values = [p.get('paused_backend_deliverable') if p.get('evidence_valid') is True else None for p in phases]
        passed, failed = sum(v is True for v in values), sum(v is False for v in values)
        unknown = len(JOBS) - passed - failed
        groups.append(dict(phase=label, budget_s=budget, scheduled_requests=len(JOBS),
            passed=passed, failed=failed, unknown=unknown,
            confirmed_pass_fraction_of_scheduled=passed/len(JOBS),
            complete_rate_known=unknown == 0,
            success_rate=passed/len(JOBS) if unknown == 0 else None,
            lower_bound=passed/len(JOBS), upper_bound=(passed+unknown)/len(JOBS),
            normal_preplanning_excluded=True, posterior_iterates_excluded=True))
    return groups


def build_report(root, build):
    root, build = Path(root).resolve(), Path(build).resolve()
    plan = read_json(root/'plan.json')
    if tuple((j.get('seed'), j.get('ordinal')) for j in plan.get('diagnostic_jobs', [])) != JOBS:
        raise ValueError('Prespecified four-job order differs')
    if plan.get('phases') != list(PHASES) or plan.get('budgets_s') != list(BUDGETS):
        raise ValueError('Plan phase/budget contract differs')
    if plan.get('treatment_order') != list(TREATMENTS) or Path(plan.get('build', '')).resolve() != build:
        raise ValueError('Plan treatment/build differs')
    manifest_path = build/'build_manifest.json'
    if digest(manifest_path) != plan.get('build_manifest_sha256'):
        raise ValueError('Build manifest changed')
    manifest = read_json(manifest_path)
    if not manifest.get('budget_scaling_experiment') or manifest.get('budget_scaling_diagnostic_phases') != list(PHASES):
        raise ValueError('Require fresh budget/scaling build')
    source_ok = bool(manifest.get('isolated_source_sha256')) and all(
        (build/'source'/name).is_file() and digest(build/'source'/name) == h
        for name, h in manifest['isolated_source_sha256'].items())
    binary_ok = bool(manifest.get('native_binaries_sha256')) and all(
        (build/'build'/name).is_file() and digest(build/'build'/name) == h
        for name, h in manifest['native_binaries_sha256'].items())
    done = optional_json(root/'complete.json') or {}
    progress = optional_json(root/'progress.json') or []
    completed = done.get('completed', progress)
    if not isinstance(completed, list):
        raise ValueError('Invalid completed/progress list')
    expected_names = ['static_budget_seed%d_refresh%d' % pair for pair in JOBS]
    if any(j.get('trial') not in expected_names for j in completed):
        raise ValueError('Unexpected trial path')
    if len({j.get('trial') for j in completed}) != len(completed):
        raise ValueError('Duplicate completed trial')
    jobs = []
    for pair, name in zip(JOBS, expected_names):
        entries = [j for j in completed if j.get('trial') == name]
        job = entries[0] if entries else dict(seed=pair[0], ordinal=pair[1], trial=name,
                                             captured=False, kind='paused', incomplete=True)
        if (job.get('seed'), job.get('ordinal')) != pair or job.get('kind') != 'paused':
            raise ValueError('Completed trial identity differs from plan')
        jobs.append(job)
    statuses = [dict(run_status=read_json(root/j['trial']/'run_status.json')) for j in jobs
                if (root/j['trial']/'run_status.json').is_file()]
    if statuses:
        assert_plot_quiet(dict(trials=statuses))
    trials = []
    for job in jobs:
        trial = root/job['trial']
        try:
            result = trial_report(trial, job)
        except (ValueError, KeyError, OSError, TypeError) as error:
            result = dict(trial=trial.name, seed=job['seed'], ordinal=job['ordinal'], source=str(trial),
                captured=None, evidence_valid=False, run_status=optional_json(trial/'run_status.json') or {},
                input_sha256={}, audit_error=type(error).__name__+': '+str(error),
                phases=[unknown_phase(label, budget, 'trial posterior audit failed: '+str(error))
                        for label, budget in zip(PHASES, BUDGETS)])
        trials.append(result)
    for trial in trials:
        for name, h in trial['input_sha256'].items():
            if digest(Path(trial['source'])/name) != h:
                raise RuntimeError('Input changed during posterior audit')
    build_valid = source_ok and binary_ok
    if not build_valid:
        for trial in trials:
            trial['evidence_valid'] = False
            for phase in trial['phases']:
                phase['evidence_valid'] = False
                phase['paused_backend_deliverable'] = None
    complete = done.get('complete') is True and len(completed) == len(JOBS)
    return dict(schema='budget_scaling_report_v1', source=str(root), build=str(build),
        reporter_sha256=digest(Path(__file__)), plan_sha256=digest(root/'plan.json'),
        complete_sha256=digest(root/'complete.json') if (root/'complete.json').is_file() else None,
        build_manifest_sha256=digest(manifest_path), build_sources_verified=source_ok,
        build_binaries_verified=binary_ok, batch_complete=complete, scheduled_requests=len(JOBS),
        recorded_completed_requests=len(completed), captured_requests=sum(t.get('captured') is True for t in trials),
        evidence_valid=complete and build_valid and all(t['evidence_valid'] is True for t in trials),
        trials=trials, aggregate=aggregate(trials), limitations=LIMITATIONS,
        online_delivery_improvement_claimed=False, dynamic_safety_improvement_claimed=False,
        all_scheduled_requests_retained=True, posterior_candidate_passes_not_counted_as_delivery=True)


def number(value, digits=3):
    return ('{:.%df}' % digits).format(value) if finite(value) else '未知'


def text_report(result):
    lines = ['预算与固定尺度：暂停固定输入后端交付实验（2026-10-27）',
        '本报告只评价暂停条件下后端能否按时交付，不代表在线轨迹发布或实车动态避障。',
        '批次完成={}；已记录完成请求={}/4；成功捕获={}；全部证据有效={}'.format(
            result['batch_complete'], result['recorded_completed_requests'], result['captured_requests'], result['evidence_valid']),
        '', '一、主指标（最终正常返回＋原生/完整投影双过＋原预算内；分母始终为4）']
    for group, short in zip(result['aggregate'], SHORT):
        rate = '{:.1%}'.format(group['success_rate']) if group['complete_rate_known'] else '未知，区间{:.1%}～{:.1%}'.format(group['lower_bound'], group['upper_bound'])
        lines.append('{}：通过{}，失败{}，未知{}；通过率{}。'.format(
            short, group['passed'], group['failed'], group['unknown'], rate))
    lines += ['', '二、每个输入的实际结果']
    for trial in result['trials']:
        lines.append('{}：capture={}，evidence={}。'.format(trial['trial'], trial.get('captured'), trial['evidence_valid']))
        if trial.get('audit_error'):
            lines.append('  审计未完成：'+trial['audit_error'])
        for phase, short in zip(trial['phases'], SHORT):
            status = {True: '通过', False: '失败', None: '未知'}[phase.get('paused_backend_deliverable')]
            lines.append('  {}：{}；最终判定{}ms；LBFGS={}ms；reason={}'.format(
                short, status, number(phase.get('decision_elapsed_s')*1000. if finite(phase.get('decision_elapsed_s')) else None),
                number(phase.get('lbfgs_wall_ms')), phase.get('solver_reason', phase.get('missing_reason'))))
            gate = (phase.get('gate') or {}).get('record') or {}
            if gate:
                lines.append('    gate检查/拒停/允许={}/{}/{}，gate耗时={}ms；后验审计={}s（预算外）。'.format(
                    gate.get('checks'), gate.get('blocked_count'), gate.get('allowed_count'),
                    number(gate.get('gate_wall_s')*1000. if finite(gate.get('gate_wall_s')) else None),
                    number(phase.get('post_solver_audit_wall_s'))))
            wheel = phase.get('last_canonical_wheel_audit') or {}
            peaks = (wheel.get('diagnostic_10ms_with_boundaries') or {}).get('peaks') or {}
            if peaks:
                lines.append('    最终候选轮约束峰值/原生容许值：'+', '.join(
                    '{}={}'.format(name, number(value.get('ratio_to_final_limit'), 4)) for name, value in peaks.items()))
            sensitivity = phase.get('final_margin_sensitivity') or {}
            if sensitivity.get('known'):
                lines.append('    同外形投影余量0/5/10cm：最坏gap='+ '/'.join(
                    number(case['min_gap_m']*1000., 1) for case in sensitivity['cases'])+'mm；仅后验敏感性，不改主结果。')
            counts = (phase.get('canonical_counts') or {}).get('all') or {}
            if counts:
                lines.append('    后验候选{}个：native过{}、投影过{}、双过{}；均不计入交付率。'.format(
                    counts.get('count'), counts.get('native_pass'), counts.get('frozen_projected_pass'), counts.get('both_pass')))
        repeat = trial.get('repeat_check') or {}
        if repeat:
            lines.append('  0.6秒veto复测：末x相同={}，末g相同={}，接受步={}/{}，耗时差={}ms。'.format(
                repeat.get('same_last_x_hash'), repeat.get('same_last_g_hash'), repeat.get('first_accepted'),
                repeat.get('repeat_accepted'), number(repeat.get('elapsed_difference_s')*1000.
                    if finite(repeat.get('elapsed_difference_s')) else None)))
    lines += ['', '三、解释边界'] + LIMITATIONS
    return '\n'.join(lines)+'\n'


def plot(result, target):
    statuses = [t for t in result['trials'] if t.get('run_status', {}).get('monitored_node_pids')]
    if statuses:
        assert_plot_quiet(dict(trials=statuses))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.8), constrained_layout=True)
    x = np.arange(len(PHASES))
    groups = result['aggregate']
    passed = np.asarray([g['passed'] for g in groups])
    failed = np.asarray([g['failed'] for g in groups])
    unknown = np.asarray([g['unknown'] for g in groups])
    axes[0].bar(x, passed, .62, color='#4e8767', label='Pass')
    axes[0].bar(x, failed, .62, bottom=passed, color='#d0d0d0', label='Fail')
    axes[0].bar(x, unknown, .62, bottom=passed+failed, color='white', edgecolor='#666666', hatch='//', label='Unknown')
    for i, group in enumerate(groups):
        axes[0].text(i, 4.08, '{}/4'.format(group['passed']) if not group['unknown'] else '{} pass / {} ?'.format(group['passed'], group['unknown']),
                     ha='center', fontsize=8)
    axes[0].set_ylim(0., 4.55)
    axes[0].set_yticks(range(5))
    axes[0].set_ylabel('Scheduled requests (all four retained)')
    axes[0].set_title('Final in-budget backend qualification')
    axes[0].legend(fontsize=8, loc='upper center', bbox_to_anchor=(.5, -.18), ncol=3)
    for i, trial in enumerate(result['trials']):
        offset = (i-1.5)*.09
        for j, phase in enumerate(trial['phases']):
            elapsed = phase.get('decision_elapsed_s')
            if finite(elapsed):
                axes[1].plot(j+offset, elapsed, 'o', color='#47718f', markersize=4,
                    label='Final decision' if i == 0 and j == 0 else None)
    axes[1].plot(x, BUDGETS, '_', color='#aa6553', markersize=20, markeredgewidth=2, label='Budget')
    axes[1].set_ylim(bottom=0.)
    axes[1].set_ylabel('Seconds (posterior audit excluded)')
    axes[1].set_title('Actual final-decision time')
    axes[1].legend(fontsize=8)
    labels = ['Original\n0.6s', 'Veto\n0.6s', 'Veto\n3.0s', 'Scaled\n3.0s', 'Repeat\n0.6s']
    for axis in axes:
        axis.set_xticks(x)
        axis.set_xticklabels(labels, fontsize=8)
        axis.spines['top'].set_visible(False)
        axis.spines['right'].set_visible(False)
    fig.suptitle('Paused identical-input diagnostics - not online delivery', fontsize=11)
    fig.savefig(target, dpi=150)
    plt.close(fig)


def report(root, build, out, plot_image=False):
    root, build, out = Path(root).resolve(), Path(build).resolve(), Path(out).resolve()
    if out.exists() or out == root or out in root.parents or out == build or out in build.parents:
        raise ValueError('Fresh separate report output required')
    result = build_report(root, build)
    out.mkdir(parents=True, exist_ok=False)
    (out/'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    (out/'预算与尺度实验结论.txt').write_text(text_report(result), encoding='utf-8')
    if plot_image:
        plot(result, out/'budget_scaling.png')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--plot', action='store_true')
    args = parser.parse_args()
    result = report(args.root, args.build, args.out, args.plot)
    print(json.dumps(dict(out=str(args.out), evidence_valid=result['evidence_valid'],
                         captured=result['captured_requests'], aggregate=result['aggregate']), ensure_ascii=False))
