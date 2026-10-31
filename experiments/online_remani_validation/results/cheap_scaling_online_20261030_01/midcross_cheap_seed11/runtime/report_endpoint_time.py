#!/usr/bin/env python3
"""Two-arm endpoint-time diagnosis, with no solver, ROS, or online actuation."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import numpy as np

from audit_delivery_iterates import loads_preserving_cpp_zero, matrix, vector_hash
import report_wheel_cause as wheel
from render_dual_timescale import FONT


HERE = Path(__file__).resolve().parent
GROUPS = ('original_060', 'endpoint150_060')
LABELS = ('原始时间 / 0.6秒', '每段首尾片×1.5 / 0.6秒')
SHOW = wheel.candidate.shown


def time_arrays(record, key):
    return [matrix(v).ravel() if isinstance(v, dict) else np.asarray(v, dtype=float).ravel()
            for v in record[key]]


def endpoint_mask(count, enabled):
    result = np.ones(count)
    if enabled:
        # A one-piece segment has only one distinct endpoint piece.
        result[sorted({0, count-1})] = 1.5
    return result


def treatment_proof(left, right):
    initial = [run['stages'][0] for run in (left, right)]
    parts, other = (stage['parts'] for stage in initial)
    counts = [len(p['times']) for p in parts]
    dimension = len(parts[0]['initial_state'])
    first_time = sum(dimension*(n-1) for n in counts)
    after_time = first_time+sum(counts)
    x, y = (stage['x'] for stage in initial)
    checks = dict(
        same_piece_counts=counts == [len(p['times']) for p in other],
        same_non_time_x=len(x) == len(y) and x[:first_time] == y[:first_time] and x[after_time:] == y[after_time:],
        same_gear=left['directions'] == right['directions'],
        same_limits=left['limits'] == right['limits'],
        same_requested_and_effective_pvaj=all(wheel.same_states(left, right, key) for key in
            ('requested_initial_states', 'requested_final_states', 'effective_initial_states', 'effective_final_states')),
        same_decoded_initial_pvaj=len(parts) == len(other) and all(
            a['initial_state'] == b['initial_state'] and a['final_state'] == b['final_state']
            for a, b in zip(parts, other)))
    original_times = time_arrays(left, 'requested_initial_times')
    new_times = time_arrays(right, 'requested_initial_times')
    proof = []
    for n, (a, b) in enumerate(zip(parts, other)):
        mask = endpoint_mask(len(a['times']), True)
        indices = sorted({0, len(a['times'])-1})
        proof.append(dict(segment=n, gear=a['gear'], pieces=len(mask), changed_piece_indices=indices,
            expected_factors=mask.tolist(),
            initial_canonical_times_correct=wheel.close(np.asarray(a['times'])*mask, b['times'], 1e-12),
            requested_times_correct=wheel.close(original_times[n]*mask, new_times[n], 1e-12),
            original_duration_s=float(sum(a['times'])), endpoint_duration_s=float(sum(b['times']))))
    checks['canonical_and_requested_time_mask'] = len(proof) == len(parts) and all(
        p['initial_canonical_times_correct'] and p['requested_times_correct'] for p in proof)
    for run, enabled in ((left, False), (right, True)):
        recorded = time_arrays(run, 'per_piece_time_factors')
        checks[run['label']+'_recorded_mask'] = (len(recorded) == len(counts) and all(
            wheel.close(v, endpoint_mask(n, enabled), 1e-12) for v, n in zip(recorded, counts)))
        checks[run['label']+'_enabled_flag'] = run.get('endpoint_time_fix_enabled') is enabled
        checks[run['label']+'_budget'] = wheel.close(run['budget_s'], .6, 1e-12)
    old = sum(p['original_duration_s'] for p in proof)
    new = sum(p['endpoint_duration_s'] for p in proof)
    return dict(passed=all(checks.values()), checks=checks, segments=proof,
        changed_piece_count=sum(len(p['changed_piece_indices']) for p in proof),
        total_piece_count=sum(counts), original_initial_duration_s=old,
        endpoint_initial_duration_s=new, initial_duration_increase_s=new-old,
        initial_duration_increase_fraction=new/old-1,
        hypothetical_all_piece_150_duration_s=1.5*old,
        endpoint_is_not_all_piece_time_scaling=True,
        first_signatures={r['label']:{key:r['solver'][key] for key in
            ('first_x_hash', 'first_gradient_hash', 'first_cost')} for r in (left, right)})


def analyze(run, params, k, dense, overlay):
    # Compatibility alias only for the older helper's metadata accessor. This
    # scalar describes endpoint pieces, never every piece of the trajectory.
    result = wheel.analyze_run(dict(run, time_scale=run['endpoint_time_scale']), params, k, dense, overlay)
    result.pop('time_scale')
    result['endpoint_time_scale'] = run['endpoint_time_scale']
    solver, stages = run['solver'], run['stages']
    recorded = run.get('all_recorded_stage_count', solver.get('all_recorded_stage_count'))
    # New serialization retains initial/last only. Do not fake accepted_count
    # to satisfy the older report's all-accepted-stage serialization assertion.
    count_proof = dict(all_recorded_stage_count=recorded,
        accepted_count=solver['accepted_count'], serialized_stage_count=len(stages),
        total_equals_accepted_plus_initial=recorded == solver['accepted_count']+1,
        serialized_count_matches=len(stages) == solver['recorded_stage_count'],
        retains_initial_and_last=len(stages) == (1 if solver['accepted_count'] == 0 else 2),
        initial_label=stages[0]['label'] == 'initial',
        final_label=stages[-1]['label'] == ('initial' if solver['accepted_count'] == 0 else 'accepted'))
    passed = all(value for key, value in count_proof.items() if key not in
                 ('all_recorded_stage_count', 'accepted_count', 'serialized_stage_count'))
    result['identity_checks']['stage_count'] = passed
    result['stage_count_proof'] = count_proof
    result['evidence_valid'] = all(result['identity_checks'].values()) and all(s['evidence_valid'] for s in result['stages'])
    result['intermediate_candidates_serialized'] = False
    result['endpoint_time_fix_enabled'] = run['endpoint_time_fix_enabled']
    gate = run['actual_native_gate']
    if gate['seen'] and gate['pass'] is False:
        result['first_rejection_motion'] = wheel.wheel_at(stages[-1], gate['rejection_time'], run['limits'])
    return result


def comparison_metrics(rows):
    base, treatment = rows
    final_a, final_b = (r['final'] for r in rows)
    speed_limit = final_b['native_summary']['omega_hard_limit_with_tolerance']
    alpha_limit = final_b['native_summary']['alpha_hard_limit_with_tolerance']
    speed_peaks = [max(r['final']['native_summary']['max_'+key] for key in ('left_omega', 'right_omega')) for r in rows]
    alpha_peaks = [max(r['final']['native_summary']['max_'+key] for key in ('left_alpha', 'right_alpha')) for r in rows]
    initial_alpha_pass = max(treatment['initial']['wheel_ratios_by_grid']['native'][2:]) <= 1
    final_alpha_pass = max(final_b['wheel_ratios_by_grid']['native'][2:]) <= 1
    return dict(speed_hard_limit_rad_s=speed_limit, alpha_hard_limit_rad_s2=alpha_limit,
        final_speed_peaks_rad_s=speed_peaks, final_acceleration_peaks_rad_s2=alpha_peaks,
        final_speed_peak_ratios=[v/speed_limit for v in speed_peaks],
        final_acceleration_peak_ratios=[v/alpha_limit for v in alpha_peaks],
        treatment_final_speed_all_recorded_grids_pass=all(max(values[:2]) <= 1 for values in final_b['wheel_ratios_by_grid'].values()),
        treatment_initial_acceleration_native_grid_pass=initial_alpha_pass,
        treatment_final_acceleration_native_grid_pass=final_alpha_pass,
        acceleration_violation_absent_initial_present_final=initial_alpha_pass and not final_alpha_pass,
        treatment_final_acceleration_excess_rad_s2=alpha_peaks[1]-alpha_limit,
        treatment_final_acceleration_excess_fraction=alpha_peaks[1]/alpha_limit-1,
        final_candidate_duration_increase_s=final_b['duration_s']-final_a['duration_s'],
        final_candidate_duration_ratio=final_b['duration_s']/final_a['duration_s'],
        final_candidate_duration_increase_fraction=final_b['duration_s']/final_a['duration_s']-1,
        intermediate_feasibility_known=False)


def decision(rows, valid):
    if not valid:
        return '证据核验未全部通过，暂不作因果归因，也不进入在线阶段。', False
    base, changed = rows
    a, b = (max(r['final']['wheel_ratios_by_grid']['native']) for r in rows)
    if not base['return_success'] and changed['return_success']:
        return ('本次仅首尾片加时使后端由失败变为完整原生验收通过；可进入一次新的在线旧/新对照，但本结果本身不是线上交付。'), True
    if base['return_success']:
        return ('原始0.6秒组已通过，未得到明确的失败→成功改善证据；不据本次小样本安排在线改善验证。'), False
    if b < a:
        return ('轮违规减轻（最坏轮比例%s→%s），但后端完整通过条件仍未满足，尚未改善后端可交付；本轮停止于固定输入诊断，不进入在线阶段。' %
                (SHOW(a), SHOW(b))), False
    return ('最坏轮违规未减轻（%s→%s），后端完整通过条件也未满足；本轮不进入在线阶段。' % (SHOW(a), SHOW(b))), False


def plot(rows, out):
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
    x = np.arange(4)
    ymax = max(max(r[s]['wheel_ratios_by_grid']['native']) for r in rows for s in ('initial', 'final'))*1.18
    for ax, row, label in zip(axes[0], rows, LABELS):
        for shift, stage, color, name in ((-.18, row['initial'], '#999999', '初始候选'),
                                         (.18, row['final'], '#246da0', '最后候选')):
            values = stage['wheel_ratios_by_grid']['native']
            ax.bar(x+shift, values, .34, color=color, label=name)
            for xx, value in zip(x+shift, values):
                ax.text(xx, value, '%.2f' % value, ha='center', va='bottom', fontsize=8)
        ax.axhline(1, ls='--', color='#b5372c', lw=1)
        ax.set_xticks(x)
        ax.set_xticklabels(['左轮速度', '右轮速度', '左轮加速度', '右轮加速度'], fontsize=9)
        ax.set_ylim(0, max(1.35, ymax))
        gate = row['actual_native_gate']
        reason = wheel.candidate.REASONS.get(gate['reason'], gate['reason']) if gate['seen'] else '未执行'
        ax.set_title(label+'\n实际原生门：'+reason, fontsize=10)
        ax.set_ylabel('原生10ms网格峰值 / 验收上限')
        ax.grid(axis='y', alpha=.15)
    axes[0, 1].legend(fontsize=8, loc='upper right')
    names = ['原始', '首尾片×1.5']
    values = [r['solver']['elapsed_s']*1000 for r in rows]
    axes[1, 0].bar(np.arange(2), values, color=['#999999', '#246da0'], width=.5)
    for i, value in enumerate(values):
        axes[1, 0].text(i, value, '%.1f ms' % value, ha='center', va='bottom')
    axes[1, 0].axhline(600, ls='--', lw=1, color='#b5372c', label='共同求解预算600ms')
    axes[1, 0].set_xticks(np.arange(2))
    axes[1, 0].set_xticklabels(names)
    axes[1, 0].set_ylim(0, max(max(values), 600)*1.25)
    axes[1, 0].set_ylabel('纯求解耗时 / ms（不含验收和交付）')
    axes[1, 0].legend(fontsize=8, loc='upper right')
    axes[1, 0].grid(axis='y', alpha=.15)
    for offset, stage, color, label in ((-.17, 'initial', '#999999', '初始候选'),
                                      (.17, 'final', '#246da0', '最后候选')):
        values = [r[stage]['duration_s'] for r in rows]
        axes[1, 1].bar(np.arange(2)+offset, values, .32, color=color, label=label)
        for i, value in enumerate(values):
            axes[1, 1].text(i+offset, value, '%.2f' % value, ha='center', va='bottom', fontsize=9)
    axes[1, 1].set_xticks(np.arange(2))
    axes[1, 1].set_xticklabels(names)
    axes[1, 1].set_ylabel('候选计划执行时长 / s（不是实走时间）')
    axes[1, 1].set_ylim(0, max(r[s]['duration_s'] for r in rows for s in ('initial', 'final'))*1.3)
    axes[1, 1].legend(fontsize=8, loc='upper right')
    axes[1, 1].grid(axis='y', alpha=.15)
    fig.suptitle('每个正/反向段仅首尾片加时：一次冻结输入诊断；不是在线运动或交付率', fontsize=12)
    fig.savefig(out, dpi=170)
    plt.close(fig)


def write_text(result, out):
    lines = ['首尾片时间加长：最小同输入对照', result['conclusion'],
        '本次是一次fresh seed11新捕获，不是旧wheel_cause_02输入的精确重放；两组共享本次同一冻结输入。',
        '两组求解预算均0.6秒；只把每个正/反向段的首片与末片初始时间各乘1.5，单片段只乘一次。',
        '证据核验：'+('通过' if result['evidence_valid'] else '未全部通过'), '']
    proof = result.get('treatment_proof')
    if proof:
        lines += ['实际改动%s/%s片；初始总时长%s→%s秒（+%s%%），不是全片×1.5的%s秒。' %
            (proof['changed_piece_count'], proof['total_piece_count'], SHOW(proof['original_initial_duration_s']),
             SHOW(proof['endpoint_initial_duration_s']), SHOW(100*proof['initial_duration_increase_fraction'], 2),
             SHOW(proof['hypothetical_all_piece_150_duration_s'])), '']
    for row, label in zip(result.get('runs', []), LABELS):
        initial, final, gate = row['initial'], row['final'], row['actual_native_gate']
        reason = wheel.candidate.REASONS.get(gate['reason'], gate['reason']) if gate['seen'] else '未执行'
        lines += [label,
            '后端返回成功=%s；实际完整原生门=%s；纯求解%s毫秒，真正接受%s步。' %
            (row['return_success'], reason, SHOW(row['solver']['elapsed_s']*1000, 1), row['solver']['accepted_count']),
            '初始→最后最坏轮比例%s→%s；最后左速/右速/左加速/右加速：%s。' %
            (SHOW(max(initial['wheel_ratios_by_grid']['native'])), SHOW(max(final['wheel_ratios_by_grid']['native'])),
             '/'.join(SHOW(x) for x in final['wheel_ratios_by_grid']['native'])),
            '候选计划时长%s→%s秒；路径长%s→%s米；最后底盘/机械臂最小硬间隙%s/%s米。' %
            (SHOW(initial['duration_s']), SHOW(final['duration_s']),
             SHOW(initial['geometry']['path_length_m']), SHOW(final['geometry']['path_length_m']),
             SHOW(final['native_summary']['min_base_hard_gap_m'], 4), SHOW(final['native_summary']['min_arm_hard_gap_m'], 4)),
            '最后四轮代价合计%s，低速代价%s；优化采样最低原始速度%s m/s，触发1e-4截断的点%s个。' %
            (SHOW(sum(final['base_feasibility_breakdown']['costs'][key] for key in wheel.COST_CHANNELS[2:]), 2),
             SHOW(final['base_feasibility_breakdown']['costs']['dense_min_speed'], 2),
             SHOW(final['base_feasibility_breakdown']['minimum_raw_speed_m_s'], 5),
             final['base_feasibility_breakdown']['nodes_below_floor']), '']
    if result.get('runs'):
        treatment = result['runs'][1]
        first, last, metrics = treatment['initial'], treatment['final'], result['comparison_metrics']
        native_a, native_b = first['native_summary'], last['native_summary']
        alpha_key = max(('left_alpha', 'right_alpha'), key=lambda key: native_b['max_'+key])
        peak = last['wheel_peak_proof']['grids']['native'][alpha_key]
        value, point = peak['independent'], peak['recorded']
        components = value['wheel_components'][alpha_key]
        side = '左' if alpha_key.startswith('left') else '右'
        lines += ['本次剩余问题',
            '首尾加时组的初值轮加速度已合限：左%s、右%s rad/s²，验收上限%s；但初值左轮速%s rad/s仍超限。' %
            (SHOW(native_a['max_left_alpha']), SHOW(native_a['max_right_alpha']), SHOW(metrics['alpha_hard_limit_rad_s2'], 4),
             SHOW(native_a['max_left_omega'])),
            '优化后轮速左%s、右%s rad/s，已在原生/粗采样/低速加密三个已检网格合限（上限%s）；但%s轮加速度升至%s，超限%s rad/s²（%s%%）。' %
            (SHOW(native_b['max_left_omega']), SHOW(native_b['max_right_omega']), SHOW(metrics['speed_hard_limit_rad_s']), side,
             SHOW(point['wheel_value'] if point['wheel_value'] >= 0 else -point['wheel_value']),
             SHOW(metrics['treatment_final_acceleration_excess_rad_s2'], 5),
             SHOW(100*metrics['treatment_final_acceleration_excess_fraction'], 2)),
            '这说明加时减轻了初始化负担，但最后候选出现初值没有的加速度违规；不能简单归结为首尾加时仍不够，也不能只凭总cost下降认定可行。',
            '剩余最大加速度峰：第%s段第%s片（均从1计），t=%s秒，位置(%s,%s)米，原始速度%s m/s。' %
            (point['segment_index']+1, point['piece_index']+1, SHOW(point['t']), SHOW(point['x'], 4), SHOW(point['y'], 4),
             SHOW(value['raw_speed_m_s'], 5)),
            '%s轮加速度%s=平动%s+转动%s rad/s²；两项同号叠加、大小接近。角加速度alpha=%s−(%s)=%s rad/s²，未触发1e-4速度截断。' %
            (side, SHOW(point['wheel_value'], 5), SHOW(components['linear'], 5), SHOW(components['turning'], 5),
             SHOW(value['angular_alpha_jerk_term'], 5), SHOW(value['angular_alpha_correction_term'], 5), SHOW(value['alpha'], 5)),
            '实际原生门更早在t=%s秒以%s拒绝；这是首次越界，不是上述全轨迹最大峰。' %
            (SHOW(treatment['actual_native_gate']['rejection_time']), SHOW(treatment['actual_native_gate']['rejection_value'], 5)),
            '末候选计划时长相对原始组增加%s秒，为%s倍（+%s%%）；不把它说成实走用时或有效交付耗时。' %
            (SHOW(metrics['final_candidate_duration_increase_s']), SHOW(metrics['final_candidate_duration_ratio'], 5),
             SHOW(100*metrics['final_candidate_duration_increase_fraction'], 2)), '',
            '解释边界',
            '轮违规是否减轻与完整原生验收是否通过分别报告；首拒绝变成碰撞、或只降低峰值但仍拒绝，都不算后端成功。',
            '实际首尾时间和非时间x、P/V/A/J、gear与轮限制均核对；时间变化后仍由原MINCO重新生成，并非只改显示时间。',
            '保留边界重新生成会改变段内几何；本次初始曲线同片归一时间采样最大位置变化%s米。' %
                SHOW(result['initial_spatial_difference']['maximum_sampled_normalized_piece_position_difference_m'], 4),
            '首尾片增加50%只是诊断扰动，不是所有片延长50%，也不是上线建议；优化后的时长仍可变化。',
            '仅保存初始/最后候选；真正接受步数来自完整求解计数，不把未序列化的中间候选算作验收失败或交付。',
            '本次不能推断中间曾出现完全可行候选；剩余违规作为后续有针对性分析的线索，本轮不新增策略或实验。',
            '0.6秒只约束求解，不含重建、原生门或最新环境验收；尚未证明线上发布、MPPI采用、到达或安全改善。',
            '一条固定输入的数值ROS SIL，不是统计结论、连续时间安全证明、Gazebo或实车结果。']
    (out/'实验结论.txt').write_text('\n'.join(lines)+'\n')


def report(batch, out):
    batch, out = Path(batch).resolve(), Path(out).resolve()
    if out.exists():
        raise FileExistsError('Fresh report output required: '+str(out))
    plan, complete = wheel.load(batch/'plan.json'), wheel.load(batch/'complete.json')
    if complete.get('complete') is not True or len(plan.get('trials', [])) != 1:
        raise ValueError('The one predeclared capture must finish before reporting')
    trial = batch/plan['trials'][0]['directory']
    path = trial/'candidate_audit/endpoint_time_replay.jsonl'
    inputs = [batch/'plan.json', batch/'complete.json', trial/'manifest.json', trial/'run_status.json']
    result = dict(schema='endpoint_time_report_v1', source=str(batch),
        reporter_sha256=wheel.sha(Path(__file__)), helper_sha256=wheel.sha(HERE/'report_wheel_cause.py'),
        fresh_capture_not_historical_exact_replay=True, evidence_valid=False, runs=[],
        diagnostic_candidates_published=False, navigation_metrics_valid=False,
        online_stage_recommended=False, online_stage_executed=False)
    if not path.is_file():
        result['conclusion'] = '本次未捕获完整两组记录；保留该结果，不换输入补样本，也不进入在线阶段。'
    else:
        records = [loads_preserving_cpp_zero(line) for line in path.read_text().splitlines() if line.strip()]
        if any(r.get('schema') != 'endpoint_time_replay_v1' for r in records):
            raise ValueError('Unknown endpoint-time schema')
        runs = [r for r in records if r.get('kind') == 'run']
        ends = [r for r in records if r.get('kind') == 'complete']
        if [r['label'] for r in runs] != list(GROUPS) or len(ends) != 1 or records[-1] != ends[0]:
            raise ValueError('Expected the prescribed two arms and completion record')
        if any(r.get('capture_error') for r in runs):
            raise ValueError('Incomplete capture; no causal comparison is valid')
        end = ends[0]
        build = Path(plan['build'])
        if wheel.sha(build/'build_manifest.json') != plan['build_manifest_sha256']:
            raise ValueError('Build identity changed since the plan was frozen')
        params, k, dense, parameter_proof = wheel.parameters(trial, build)
        overlay = runs[0]['frozen_checks']['overlay_begin']
        rows = [analyze(r, params, k, dense, overlay) for r in runs]
        time_proof = treatment_proof(*runs)
        case_path, failure_path = trial/'candidate_audit/case.json', trial/'failure_trace.jsonl'
        case = wheel.load(case_path)
        failures = [loads_preserving_cpp_zero(line) for line in failure_path.read_text().splitlines() if line.strip()]
        matching = [r for r in failures if r.get('kind') == 'solve' and r.get('label') == 'normal' and
                    str(r.get('input_hash')) == str(case['input_hash'])]
        failure = matching[0] if len(matching) == 1 else {}
        signature = dict(capture_input_hash=case['input_hash'] == runs[0]['input_hash'],
            first_x=vector_hash(case['stages'][0]['x']) == runs[0]['solver']['first_x_hash'],
            first_f=case['stages'][0]['cost'] == runs[0]['solver']['first_cost'] == failure.get('first_cost'),
            first_g=str(failure.get('first_gradient_hash')) == runs[0]['solver']['first_gradient_hash'],
            effective_pvaj=all(wheel.same_states(case, runs[0], key) for key in
                ('effective_initial_states', 'effective_final_states')),
            original_times=all(wheel.close(a, b, 1e-12) for a, b in zip(
                time_arrays(case, 'initial_times'), time_arrays(runs[0], 'requested_initial_times'))))
        checks = {key:end.get(key) is True for key in (
            'all_captures_complete', 'environment_stable', 'capture_original_same_first_x_f_g',
            'retime_only_initial_endpoint_time_coordinates', 'original_solver_pin_owned',
            'original_solver_entry_exit_equal')}
        checks.update(not_published=end.get('diagnostic_candidates_published') is False,
            limits_not_relaxed=end.get('constraints_relaxed') is False,
            actual_67_spheres=len(wheel.load(trial/'native_collision_3d_body.json')['body']) == 67,
            capture_signature=all(signature.values()),
            original_failure=case['reason'] == failure.get('reason') and failure.get('success') is False)
        valid = all(checks.values()) and time_proof['passed'] and all(r['evidence_valid'] for r in rows)
        conclusion, online = decision(rows, valid)
        result.update(evidence_valid=valid, conclusion=conclusion, online_stage_recommended=online,
            runs=rows, capture_checks=checks, capture_original_signature=signature,
            treatment_proof=time_proof, completion=end, parameter_evidence=parameter_proof,
            captured_request_id=case['request_id'], captured_input_hash=case['input_hash'],
            initial_spatial_difference=wheel.initial_spatial_difference(*runs),
            comparison_metrics=comparison_metrics(rows),
            derivative_hash_scope='first gradient hash recorded, gradient vector not independently recomputed',
            wheel_violation_reduced=max(rows[1]['final']['wheel_ratios_by_grid']['native']) <
                max(rows[0]['final']['wheel_ratios_by_grid']['native']),
            full_native_return_improved=not rows[0]['return_success'] and rows[1]['return_success'])
        result['postprocessing_note'] = ('Reporting only: full gates and inputs unchanged; added initial-to-final acceleration reintroduction, '
            'remaining peak decomposition, and candidate-duration comparisons. Reporter and helper hashes identify this analysis version.')
        inputs += [path, case_path, failure_path, trial/'parameters.yaml',
                   trial/'native_collision_3d_body.json', build/'build_manifest.json']
        if valid:
            out.mkdir(parents=True)
            font = FontProperties(fname=FONT)
            plt.rcParams.update({'font.family':font.get_name(), 'axes.unicode_minus':False,
                                 'font.size':9, 'savefig.facecolor':'white'})
            plot(rows, out/'首尾片时间对照.png')
            result['figures'] = ['首尾片时间对照.png']
    result['input_sha256'] = {str(p):wheel.sha(p) for p in inputs}
    out.mkdir(parents=True, exist_ok=True)
    write_text(result, out)
    (out/'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    if any(wheel.sha(name) != digest for name, digest in result['input_sha256'].items()):
        raise RuntimeError('Source input changed while reporting')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch', type=Path, default=HERE/'results/endpoint_time_20261030_01')
    parser.add_argument('--out', type=Path, default=HERE/'results/endpoint_time_report_20261030_01')
    args = parser.parse_args()
    result = report(args.batch, args.out)
    print(json.dumps({key:result[key] for key in ('evidence_valid', 'online_stage_recommended', 'conclusion')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
