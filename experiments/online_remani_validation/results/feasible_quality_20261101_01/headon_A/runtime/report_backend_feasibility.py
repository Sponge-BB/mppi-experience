#!/usr/bin/env python3
"""Summarize four NEW paused captures and verified post-solver audits.

This report never solves, reconstructs, scans geometry or publishes. Captured
optimizer repeats are not historical exact replay or online delivery timings.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import gzip
import json
from pathlib import Path

from audit_delivery_iterates import canonical_records
from report_delivery_stability import LABELS, BUDGETS, summarize_paused
from report_domain_recovery import assert_plot_quiet, digest, read_json, read_records
from report_state_contract import finite, integer, unsigned_identity


JOBS = ((11, 1), (12, 2), (11, 2), (12, 1))
LIMITATIONS = [
    '四个预先指定的NEW同进程暂停捕获，不是历史请求的完整精确重放；各trial之间输入不同，不作配对速度因果比较。',
    'ordinal是在已有reference后第N次满足条件的0.6s后端调用，不是FSM request_id。',
    '同一trial内依次0.6/0.6/1.5s，首x/f/g和记录上下文一致；同一first f/g不能单独证明完整native地图及全部隐状态逐字节冻结。',
    '第二次0.6s用于观察墙钟预算/迭代抖动，不是独立重复样本或成功率统计。',
    '只统计failure_trace.lbfgs_wall_ms；后验native/kinematic/projected审计耗时不当作求解或在线交付时间。',
    'MPPI与simulator暂停，CPU条件不同于在线闭环；accepted x/g拷贝仍包含在L-BFGS预算内。',
    'initial(x0)与accepted迭代分开；轨迹t=0又是另一概念，不将二者混淆。',
    '原生IsTrajSafe、冻结投影20ms全程抽样、诊断10ms含边界轮速扫描是不同检查；抽样通过不证明连续时间或未来FSM live验收通过。',
    '后验发现可行候选也不是实际可交付时间；本轮不允许发布诊断候选，不是导航、实时性能或实车安全实验。',
    '保留全部四个任务；未捕获、日志缺失、身份/审计不完整记未知或无效，不补零、不挑结果。',
    'accepted记录存在4096容量限制；截断时证据无效，不能宣称所有已接受候选均不可行。',
    '捕获是否发生在障碍出现之后必须核对simulator motion_anchor与capture.sim；scene命名static不等于当时已出现障碍。',
]


def unique(rows, field, value):
    found = [r for r in rows or [] if r.get(field) == value]
    return found[0] if len(found) == 1 else None


def candidate_key(row):
    stage, iteration, value = row.get('stage'), row.get('iteration'), row.get('candidate_x_hash')
    value = unsigned_identity(value)
    if stage not in ('initial', 'accepted') or not integer(iteration) or iteration < 0 or value is None:
        return None
    if (stage == 'initial') != (iteration == 0):
        return None
    return stage, iteration, value


def read_optional_json(trial, name, hashes):
    path = next((p for p in (trial / name, trial / (name + '.gz')) if p.is_file()), None)
    if path is None:
        return None
    hashes[path.name] = digest(path)
    opener = gzip.open if path.suffix == '.gz' else open
    with opener(path, 'rt', encoding='utf-8') as stream:
        return json.load(stream)


def scene_phase(capture, outcome, events):
    sim = capture.get('sim') if isinstance(capture, dict) else None
    frame_now = capture.get('frame', {}).get('now') if isinstance(capture, dict) else None
    motion = outcome.get('motion_anchor') if isinstance(outcome, dict) else None
    anchors = [e.get('t') for e in events or [] if isinstance(e, dict) and e.get('kind') == 'motion_anchor']
    exact = finite(motion) and len(anchors) == 1 and anchors[0] == motion
    appearance = 1000. + motion + 4. if exact else None
    return dict(capture_ros_time=sim, simulator_motion_anchor_s=motion,
        capture_elapsed_s=sim - 1000. if finite(sim) else None,
        frozen_frame_ros_time=frame_now, frozen_frame_elapsed_s=frame_now - 1000. if finite(frame_now) else None,
        motion_anchor_unique_and_matches_result=exact, scene_appearance_ros_time=appearance,
        capture_at_or_after_appearance=sim >= appearance if finite(sim) and appearance is not None else None,
        capture_minus_appearance_s=sim - appearance if finite(sim) and appearance is not None else None,
        frozen_frame_at_or_after_appearance=frame_now >= appearance if finite(frame_now) and appearance is not None else None,
        frozen_frame_minus_appearance_s=frame_now - appearance if finite(frame_now) and appearance is not None else None,
        exact_perception_delivery_time_claimed=False,
        definition='static scene synthetic obstacle appears at simulator motion_anchor+4; ROS epoch=1000; not perception receipt time')


def stage_counts(candidates, stage):
    group = [r for r in candidates if r.get('stage') == stage]
    keys = ('native_IsTrajSafe_pass', 'frozen_projected_pass', 'both_pass')
    return dict(count=len(group), **{k: sum(r.get(k) is True for r in group) for k in keys},
        native_grid_wheel_pass=sum((r.get('native_10ms') or {}).get('all_wheels_sampled_pass') is True for r in group),
        diagnostic_grid_wheel_pass=sum((r.get('diagnostic_10ms_with_boundaries') or {}).get('all_wheels_sampled_pass') is True for r in group),
        unknown={k: sum(not isinstance(r.get(k), bool) for r in group) for k in keys})


def combine_phase(paused_pass, iterate_phase, kinematic_phase):
    """Join verified artifacts, never reconstruct or associate by nearest time."""
    phase = paused_pass['label']; summary = paused_pass.get('summary') or {}
    it = iterate_phase or {}; kin = kinematic_phase or {}
    it_candidates = it.get('candidates') or []; kin_candidates = kin.get('candidates') or []
    it_keys = [candidate_key(r) for r in it_candidates]
    kin_keys = [candidate_key(r) for r in kin_candidates]
    unique_candidates = (bool(it_keys) and None not in it_keys and None not in kin_keys and
        len(it_keys) == len(set(it_keys)) and len(kin_keys) == len(set(kin_keys)) and set(it_keys) == set(kin_keys))
    same_summary = bool(summary) and it.get('solve_summary') == summary
    expected_hash = unsigned_identity(summary.get('input_hash'))
    kin_hash = unsigned_identity(kin.get('input_hash'))
    identity = expected_hash is not None and kin_hash == expected_hash
    signature = (bool(it.get('first_signature')) and kin.get('first_signature') == it.get('first_signature'))
    valid = (paused_pass.get('complete_records') is True and paused_pass.get('input_hash_equal') is True and
        paused_pass.get('budget_agrees') is True and paused_pass.get('verdict_agrees') is True and
        it.get('phase_evidence_valid') is True and kin.get('phase_evidence_valid') is True and
        same_summary and identity and signature and unique_candidates)
    differences = []
    joined = []
    if unique_candidates:
        by_key = {candidate_key(r): r for r in kin_candidates}
        for candidate in it_candidates:
            k = by_key[candidate_key(candidate)]
            agreement = all(k.get(field) is candidate.get(field) for field in
                ('native_IsTrajSafe_pass', 'frozen_projected_pass', 'both_pass'))
            if not agreement:
                differences.append(candidate_key(candidate))
            joined.append(dict(stage=candidate['stage'], iteration=candidate['iteration'],
                candidate_x_hash=candidate['candidate_x_hash'], identity_and_audit_agreement=agreement,
                candidate_evidence_valid=candidate.get('candidate_evidence_valid'),
                kinematics_evidence_valid=k.get('evidence_valid'),
                solver_elapsed_s=candidate.get('solver_elapsed_s'),
                native_IsTrajSafe_pass=candidate.get('native_IsTrajSafe_pass'),
                native_reason=candidate.get('native_reason'),
                frozen_projected_pass=candidate.get('frozen_projected_pass'), both_pass=candidate.get('both_pass'),
                projected_20ms=candidate.get('frozen_projected_20ms'),
                initial_k0_values=k.get('initial_k0_values'),
                native_10ms=k.get('native_10ms'),
                diagnostic_10ms_with_boundaries=k.get('diagnostic_10ms_with_boundaries')))
    valid = valid and not differences and all(r['candidate_evidence_valid'] is True and r['kinematics_evidence_valid'] is True for r in joined)
    last = joined[-1] if joined else None
    peaks = ((last or {}).get('diagnostic_10ms_with_boundaries') or {}).get('peaks') or {}
    return dict(phase=phase, budget_s=paused_pass['budget_s'], evidence_valid=valid,
        checks=dict(complete_same_input_solver_records=paused_pass.get('complete_records') is True and paused_pass.get('input_hash_equal') is True,
            same_solver_summary=same_summary, kinematics_input_hash_matches=identity,
            same_first_signature=signature, unique_candidate_join=unique_candidates,
            audit_pass_statuses_agree=not differences if unique_candidates else None),
        solver_reason=summary.get('reason'), native_return_success=summary.get('native_return_success'),
        lbfgs_wall_ms=paused_pass.get('solver_wall_ms'), iterations=(paused_pass.get('native_failure_frame') or {}).get('iterations'),
        counts={stage: stage_counts(joined, stage) if valid else None for stage in ('initial', 'accepted')},
        candidate_join_disagreements=differences, candidates=joined,
        selected_candidate_checks=it.get('selected_candidate_checks'),
        first_both_pass=it.get('first_both_pass') if valid else None,
        first_native_pass=it.get('first_native_pass') if valid else None,
        native_audit_cost=it.get('native_audit_cost'),
        last_recorded_diagnostic_peak_ratios={name: value.get('ratio_to_final_limit') if isinstance(value, dict) else None for name, value in peaks.items()},
        not_online_delivery=True)


def repeat_check(phases):
    a = unique(phases, 'phase', LABELS[0]); b = unique(phases, 'phase', LABELS[1])
    known = a is not None and b is not None and a.get('evidence_valid') is True and b.get('evidence_valid') is True
    ta = a.get('lbfgs_wall_ms') if a else None; tb = b.get('lbfgs_wall_ms') if b else None
    return dict(valid_same_input_repeated_budget=known,
        lbfgs_wall_ms_difference_repeat_minus_nominal=tb - ta if known and finite(ta) and finite(tb) else None,
        same_solver_reason=a.get('solver_reason') == b.get('solver_reason') if known else None,
        same_native_return=a.get('native_return_success') is b.get('native_return_success') if known else None,
        accepted_count_difference=(b['counts']['accepted']['count'] - a['counts']['accepted']['count']) if known else None,
        not_independent_repetitions=True)


def trial_report(trial, job, iterate_trial, kinematic_trial):
    hashes = {}; manifest = read_json(trial / 'manifest.json'); status = read_json(trial / 'run_status.json')
    for name in ('manifest.json', 'run_status.json'):
        hashes[name] = digest(trial / name)
    rows, replay_path = canonical_records(trial)
    if replay_path is not None:
        hashes[replay_path.name] = digest(replay_path)
    native, native_path = read_records(trial, 'dual_timeline.jsonl')
    failures, failure_path = read_records(trial, 'failure_trace.jsonl')
    for path in (native_path, failure_path):
        if path is not None:
            hashes[path.name] = digest(path)
    done_path = trial / 'diagnostic_complete.json'; done = read_json(done_path) if done_path.is_file() else {}
    if done_path.is_file():
        hashes[done_path.name] = digest(done_path)
    paused = summarize_paused(rows, failures, done, native)
    capture = paused.get('capture') or {}
    replay_sha = digest(replay_path) if replay_path is not None else None
    it = iterate_trial or {}; kin = kinematic_trial or {}
    identities = dict(iterate_replay_sha256_matches=replay_sha is not None and it.get('replay_sha256') == replay_sha,
        kinematics_replay_sha256_matches=replay_sha is not None and kin.get('replay_sha256') == replay_sha,
        iterate_manifest_matches=it.get('manifest_sha256') == hashes['manifest.json'],
        kinematics_manifest_matches=kin.get('manifest_sha256') == hashes['manifest.json'],
        kinematics_run_status_matches=kin.get('run_status_sha256') == hashes['run_status.json'],
        iterate_run_status_matches=it.get('run_status_sha256') == hashes['run_status.json'],
        iterate_capture_matches=bool(capture) and it.get('capture') == capture)
    phases = [combine_phase(p, unique(it.get('phases'), 'phase', p['label']),
        unique(kin.get('phases'), 'phase', p['label'])) for p in paused['passes']]
    settings = manifest.get('coupling_experiment') or {}
    captured_begins = [r for r in native or [] if r.get('event') == 'request_begin' and r.get('request_id') == capture.get('request_id')]
    runtime = manifest.get('experiment_sources_sha256') or {}; runtime_files = {}
    for name, expected in runtime.items():
        if name.endswith('.py'):
            path = trial / 'runtime' / name
            runtime_files[name] = dict(expected=expected, observed=digest(path) if path.is_file() else None)
    outcome = read_optional_json(trial, 'result.json', hashes)
    events = read_optional_json(trial, 'events.json', hashes)
    scope = dict(static=manifest.get('scene') == 'static', seed=settings.get('seed') == job['seed'],
        ordinal=settings.get('refresh_ordinal') == job['ordinal'],
        actual_capture_ordinal=capture.get('refresh_ordinal') == job['ordinal'],
        actual_replan_fsm=len(captured_begins) == 1 and captured_begins[0].get('fsm') == 3,
        capture_weights_zero=capture.get('exec_weight') == 0. and capture.get('hand_weight') == 0.,
        paused=settings.get('diagnostic_pause') is True,
        not_online=settings.get('genuine_online_closed_loop') is False,
        signed=settings.get('signed_low_speed') is True, domain=settings.get('domain_recovery') is True,
        fix_on=settings.get('seed_parameterization_fix') is True,
        admission_off=settings.get('boundary_admission') is False,
        weights_zero=settings.get('execution_weight') == 0. and settings.get('handover_weight') == 0.,
        navigation_metrics_invalid=status.get('navigation_metrics_valid') is False)
    valid = (job.get('captured') is True and paused['diagnosis_record_contract_valid'] is True and
        it.get('evidence_valid') is True and kin.get('evidence_valid') is True and
        all(identities.values()) and all(scope.values()) and all(p['evidence_valid'] for p in phases))
    return dict(trial=trial.name, seed=job['seed'], ordinal=job['ordinal'], source=str(trial),
        run_status=status, captured=job.get('captured'), evidence_valid=valid,
        recorded_contract=paused, identities=identities, scope_checks=scope,
        scene_phase=scene_phase(capture, outcome, events), phases=phases, repeat_check=repeat_check(phases),
        runtime_sha256=runtime, runtime_python_files=runtime_files,
        runtime_python_verified=bool(runtime_files) and all(r['observed'] == r['expected'] for r in runtime_files.values()),
        build=manifest.get('build'), build_manifest_sha256=manifest.get('build_manifest_sha256'),
        kinematics_parameter_evidence=kin.get('parameter_evidence'), input_sha256=hashes,
        navigation_metrics_valid=False, diagnostic_candidates_published=False if paused['diagnostic_no_publication_declaration'] and paused['observed_no_captured_request_publication'] is True else None)


def build_report(root, iterate_audit, kinematics_audit):
    root = Path(root).resolve(); iterate_audit = Path(iterate_audit).resolve(); kinematics_audit = Path(kinematics_audit).resolve()
    plan = read_json(root / 'plan.json'); complete = read_json(root / 'complete.json')
    jobs = complete.get('completed')
    if complete.get('complete') is not True or not isinstance(jobs, list) or len(jobs) != 4:
        raise ValueError('All four predeclared paused jobs required')
    if any(j.get('kind') != 'paused' for j in jobs) or tuple((j.get('seed'), j.get('ordinal')) for j in jobs) != JOBS:
        raise ValueError('Prespecified job sequence differs')
    if tuple((j.get('seed'), j.get('ordinal')) for j in plan.get('diagnostic_jobs', [])) != JOBS:
        raise ValueError('Plan differs from prespecified four-job sequence')
    names = [j.get('trial') for j in jobs]
    if len(set(names)) != 4 or any(not isinstance(n, str) or Path(n).name != n or n in ('.', '..') for n in names):
        raise ValueError('Unsafe/duplicate trial names')
    assert_plot_quiet(dict(trials=[dict(run_status=read_json(root / n / 'run_status.json')) for n in names]))
    it = read_json(iterate_audit); kin = read_json(kinematics_audit)
    if it.get('schema') != 'delivery_iterate_audit_v1' or kin.get('schema') != 'backend_kinematics_audit_v1':
        raise ValueError('Unknown audit schema')
    if Path(it.get('source', '')).resolve() != root or Path(kin.get('source', '')).resolve() != root:
        raise ValueError('Audits belong to different batch')
    if kin.get('iterate_audit_sha256') != digest(iterate_audit):
        raise ValueError('Kinematics audit used another iterate audit')
    if any(Counter(t.get('trial') for t in a.get('trials', [])) != Counter(names) for a in (it, kin)):
        raise ValueError('Audit omitted, duplicated or added trials')
    trials = [trial_report(root / job['trial'], job, unique(it['trials'], 'trial', job['trial']),
        unique(kin['trials'], 'trial', job['trial'])) for job in jobs]
    build = Path(plan['build']).resolve(); bm = build / 'build_manifest.json'
    if digest(bm) != plan.get('build_manifest_sha256'):
        raise ValueError('Build manifest drift')
    build_manifest = read_json(bm); sources = build_manifest.get('isolated_source_sha256') or {}
    binaries = build_manifest.get('native_binaries_sha256') or {}
    integrity = dict(compiled_fix_family=build_manifest.get('seed_parameterization_experiment') is True and build_manifest.get('prepared_only') is False,
        isolated_source_verified=bool(sources) and all((build / 'source' / n).is_file() and digest(build / 'source' / n) == h for n, h in sources.items()),
        native_binaries_verified=bool(binaries) and all((build / 'build' / n).is_file() and digest(build / 'build' / n) == h for n, h in binaries.items()),
        trial_build_matches_plan=all(t['build'] and Path(t['build']).resolve() == build and t['build_manifest_sha256'] == plan['build_manifest_sha256'] for t in trials),
        same_full_runtime=bool(trials[0]['runtime_sha256']) and all(t['runtime_sha256'] == trials[0]['runtime_sha256'] for t in trials),
        runtime_python_verified=all(t['runtime_python_verified'] for t in trials))
    return dict(schema='backend_feasibility_report_v1', created=datetime.now(timezone.utc).isoformat(),
        source=str(root), plan=plan, complete=complete, trials=trials, integrity=integrity,
        all_evidence_valid=all(integrity.values()) and all(t['evidence_valid'] for t in trials),
        input_sha256=dict(plan=digest(root / 'plan.json'), complete=digest(root / 'complete.json'),
            iterate_audit=digest(iterate_audit), kinematics_audit=digest(kinematics_audit)),
        report_source_sha256=digest(Path(__file__)), limitations=LIMITATIONS,
        navigation_metrics_valid=False, real_time_delivery_claim=False,
        historical_complete_replay_claim=False, constraints_relaxed=False)


def report_text(value):
    lines = ['修正种子之后：四个NEW暂停后端约束诊断',
        '不是导航速度实验；诊断候选禁止发布。initial迭代x0与轨迹时间t=0分开解释。',
        '本批证据契约有效=' + str(value['all_evidence_valid']), '']
    for trial in value['trials']:
        lines += ['%s seed=%s eligible后端ordinal=%s captured=%s evidence_valid=%s' % (trial['trial'], trial['seed'], trial['ordinal'], trial['captured'], trial['evidence_valid']),
            '  捕获是否已在障碍出现之后=%s；相差=%s秒；frozen frame是否在场景出现之后=%s（均不是精确感知时刻）' % (trial['scene_phase']['capture_at_or_after_appearance'], trial['scene_phase']['capture_minus_appearance_s'], trial['scene_phase']['frozen_frame_at_or_after_appearance'])]
        for phase in trial['phases']:
            lines.append('  %s: valid=%s；solver=%s；LBFGS=%s ms；native return=%s' % (phase['phase'], phase['evidence_valid'], phase['solver_reason'], phase['lbfgs_wall_ms'], phase['native_return_success']))
            for stage in ('initial', 'accepted'):
                c = phase['counts'][stage]
                lines.append('    %s: %s' % (stage, '未知/无效，不补零' if c is None else '候选=%s，native过关=%s，完整投影抽样过关=%s，双过关=%s' % (c['count'], c['native_IsTrajSafe_pass'], c['frozen_projected_pass'], c['both_pass'])))
                if c is not None:
                    lines.append('      仅轮约束：native网格过关=%s；含t0/边界诊断网格过关=%s（不同检查口径）' % (c['native_grid_wheel_pass'], c['diagnostic_grid_wheel_pass']))
            lines.append('    首个双过关候选（后验）=' + json.dumps(phase['first_both_pass'], ensure_ascii=False))
            lines.append('    最后记录候选：双轮四项约束峰值/终验限值=' + json.dumps(phase['last_recorded_diagnostic_peak_ratios'], ensure_ascii=False))
        lines += ['  重复0.6s检查=' + json.dumps(trial['repeat_check'], ensure_ascii=False), '']
    return '\n'.join(lines + ['构建/runtime核验=' + json.dumps(value['integrity'], ensure_ascii=False), '', '解释边界：'] + LIMITATIONS) + '\n'


def plot_report(value, path):
    assert_plot_quiet(value)
    if value['all_evidence_valid'] is not True:
        raise ValueError('Invalid/missing evidence retained in TXT/JSON; refuse normal comparison plot')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.3))
    items = [(t, p) for t in value['trials'] for p in t['phases']]
    for dx, key, label, color in ((-.2, 'count', 'Accepted', '#b8b8b8'), (0., 'native_IsTrajSafe_pass', 'Native pass', '#4c78a8'), (.2, 'both_pass', 'Native + projected pass', '#59a14f')):
        axes[0].bar([i + dx for i in range(len(items))], [p['counts']['accepted'][key] for t, p in items], .2, label=label, color=color)
    axes[0].set_xticks(range(len(items))); axes[0].set_xticklabels(['D%s\n%s' % (i // 3 + 1, ('.6a', '.6b', '1.5')[i % 3]) for i in range(len(items))], fontsize=7)
    axes[0].set_ylabel('Accepted-iterate count (initial excluded)'); axes[0].legend(fontsize=7)
    for j, label in enumerate(LABELS):
        axes[1].bar([i + (j - 1) * .24 for i in range(4)], [t['phases'][j]['lbfgs_wall_ms'] for t in value['trials']], .23, label=('.6a', '.6b', '1.5')[j])
    axes[1].set_xticks(range(4)); axes[1].set_xticklabels(['D' + str(i + 1) for i in range(4)])
    axes[1].set_ylabel('LBFGS wall / ms (post-audits excluded)'); axes[1].legend(fontsize=8)
    for ax in axes:
        ax.grid(axis='y', alpha=.2)
    fig.suptitle('Paused same-input diagnostics — no candidate published; not online delivery', fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, .94)); fig.savefig(path, dpi=140); plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True); parser.add_argument('--iterate-audit', type=Path, required=True)
    parser.add_argument('--kinematics-audit', type=Path, required=True); parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--plot', action='store_true'); args = parser.parse_args(argv)
    root = args.root.resolve(); out = args.out.resolve()
    if out.exists() or out == root or out in root.parents:
        parser.error('Fresh separate output required')
    value = build_report(root, args.iterate_audit, args.kinematics_audit); out.mkdir(parents=True, exist_ok=False)
    (out / 'summary.json').write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    (out / '实验结论.txt').write_text(report_text(value), encoding='utf-8')
    if args.plot:
        plot_report(value, out / 'backend_feasibility.png')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
