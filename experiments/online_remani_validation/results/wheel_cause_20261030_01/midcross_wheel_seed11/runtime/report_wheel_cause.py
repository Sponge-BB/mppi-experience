#!/usr/bin/env python3
"""Report one frozen-input wheel diagnosis; no ROS, solver, or publication.

All figures use recorded canonical candidates. Native return gates, posterior
checks, and online delivery are deliberately kept separate.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import numpy as np
import yaml

from audit_delivery_iterates import loads_preserving_cpp_zero, matrix, vector_hash
from backend_base_feasibility_breakdown import (CHANNELS as COST_CHANNELS,
    piece_nodes, point_costs, source_contract)
import report_candidate_audit as candidate
from render_dual_timescale import FONT


GROUPS = ('original_060', 'original_120', 'time150_060')
LABELS = ('原始时间 / 0.6秒', '原始时间 / 1.2秒', '初始时间×1.5 / 0.6秒')
WHEELS = ('left_omega', 'right_omega', 'left_alpha', 'right_alpha')
WHEEL_LABELS = ('左轮速度', '右轮速度', '左轮加速度', '右轮加速度')
GRID_KEYS = {'native': 'summary', 'K16': 'optimizer_summary',
             'dense': 'feasibility_optimizer_summary'}
HERE = Path(__file__).resolve().parent


def load(path):
    return loads_preserving_cpp_zero(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def close(left, right, tolerance=1e-9):
    a, b = np.asarray(left, dtype=float), np.asarray(right, dtype=float)
    return bool(a.shape == b.shape and np.all(np.isfinite(a)) and
                np.all(np.isfinite(b)) and np.allclose(a, b, rtol=tolerance, atol=tolerance))


def states(record, key):
    return [matrix(value) for value in record[key]]


def same_states(left, right, key):
    a, b = states(left, key), states(right, key)
    return len(a) == len(b) and all(np.array_equal(x, y) for x, y in zip(a, b))


def pvaj(coefficients, t):
    c = np.asarray(coefficients, dtype=float)
    result = []
    for order in range(4):
        factors = np.asarray([math.factorial(i)//math.factorial(i-order)
                              for i in range(order, 8)], dtype=float)
        result.append(np.polynomial.polynomial.polyval(t, c[order:]*factors[:, None]))
    return np.column_stack(result)


def locate(stage, time):
    """Use native strict-> ownership: exact knots belong to the preceding piece."""
    local = float(time)
    parts = stage['parts']
    n = 0
    while n+1 < len(parts) and local > sum(parts[n]['times']):
        local -= sum(parts[n]['times'])
        n += 1
    part = parts[n]
    k = 0
    while k+1 < len(part['times']) and local > part['times'][k]:
        local -= part['times'][k]
        k += 1
    if local < -1e-8 or local > part['times'][k]+1e-8:
        raise ValueError('Recorded sample lies outside its canonical trajectory')
    return n, k, local


def wheel_at(stage, time, limits):
    n, k, local = locate(stage, time)
    part = stage['parts'][n]
    c = np.asarray(part['b_rows_ascending_power'], dtype=float)[8*k:8*(k+1)]
    state = pvaj(c, local)
    v, a, j = (state[:2, order] for order in (1, 2, 3))
    raw_speed = float(np.linalg.norm(v))
    v2, speed = max(1e-8, float(v@v)), max(1e-4, raw_speed)
    cross, dot = float(v[0]*a[1]-v[1]*a[0]), float(a@v)
    term1 = float(v[0]*j[1]-v[1]*j[0])/v2
    term2 = 2*cross*dot/(v2*v2)
    omega, alpha = cross/v2, term1-term2
    b, r, gear = limits['wheel_base'], limits['wheel_radius'], part['gear']
    wheels = [(gear*speed-b*omega/2)/r, (gear*speed+b*omega/2)/r,
              (gear*dot/speed-b*alpha/2)/r, (gear*dot/speed+b*alpha/2)/r]
    return dict(raw_speed_m_s=raw_speed, below_velocity_floor=raw_speed < 1e-4,
        omega=omega, alpha=alpha, angular_alpha_jerk_term=term1,
        angular_alpha_correction_term=term2, signed_tangential_acceleration=gear*dot/speed,
        velocity=v.tolist(), acceleration=a.tolist(), jerk=j.tolist(),
        segment_index=n, piece_index=k, piece_local_t=local, **dict(zip(WHEELS, wheels)))


def boundary_checks(stage):
    errors = []
    for part in stage['parts']:
        times = np.asarray(part['times'], dtype=float)
        c = np.asarray(part['b_rows_ascending_power'], dtype=float)
        if (times.ndim != 1 or not len(times) or np.any(times <= 0) or
                not np.all(np.isfinite(times)) or c.shape != (8*len(times), 8) or
                not np.all(np.isfinite(c)) or part['gear'] not in (-1, 1)):
            raise ValueError('Invalid native canonical polynomial/times/gear')
        for side, actual in (('initial', pvaj(c[:8], 0.)),
                             ('final', pvaj(c[-8:], times[-1]))):
            intended = np.asarray(part[side+'_state'], dtype=float)
            recorded = np.asarray(part['actual_'+side+'_pvaj'], dtype=float)
            error = float(np.max(np.abs(actual-intended)))
            errors.append(dict(side=side, max_abs_error=error,
                passed=close(actual, intended, 1e-7) and close(actual, recorded, 1e-8)))
    return dict(passed=all(e['passed'] for e in errors),
        maximum_abs_error=max(e['max_abs_error'] for e in errors), parts=errors)


def coordinate_checks(stage, run, non_singular_speed):
    """Check decoded x against recorded times, waypoints, and effective PVAJ."""
    parts, x = stage['parts'], np.asarray(stage['x'], dtype=float)
    dim = len(parts[0]['initial_state'])
    counts = [len(part['times']) for part in parts]
    at, points = 0, []
    for count in counts:
        size = dim*(count-1)
        points.append(x[at:at+size].reshape((dim, count-1), order='F'))
        at += size
    times_match = True
    for part, count in zip(parts, counts):
        virtual = x[at:at+count]
        real = np.asarray([(.5*v+1.)*v+1. if v > 0 else 1./((.5*v-1.)*v+1.) for v in virtual])
        times_match = times_match and close(real, part['times'], 1e-12)
        at += count
    gear_points = x[at:at+dim*(len(parts)-1)].reshape((dim, len(parts)-1), order='F')
    angles = x[at+dim*(len(parts)-1):]
    shape_match = len(angles) == len(parts)-1
    boundary_match, waypoint_match = shape_match, True
    effective_ini, effective_fin = states(run, 'effective_initial_states'), states(run, 'effective_final_states')
    for n, part in enumerate(parts):
        initial, final = effective_ini[n].copy(), effective_fin[n].copy()
        if n and shape_match:
            initial[:, 0] = gear_points[:, n-1]
            initial[:2, 1] = -non_singular_speed*np.asarray([math.cos(angles[n-1]), math.sin(angles[n-1])])
        if n+1 < len(parts) and shape_match:
            final[:, 0] = gear_points[:, n]
            final[:2, 1] = non_singular_speed*np.asarray([math.cos(angles[n]), math.sin(angles[n])])
        boundary_match = boundary_match and close(initial, part['initial_state'], 1e-12) and close(final, part['final_state'], 1e-12)
        c = np.asarray(part['b_rows_ascending_power'], dtype=float)
        for k in range(counts[n]-1):
            waypoint_match = waypoint_match and close(pvaj(c[8*k:8*(k+1)], part['times'][k])[:, 0], points[n][:, k], 1e-7)
    checks = dict(real_times_match_x=times_match, decoded_effective_pvaj_match=boundary_match,
                  inner_waypoints_match_x=waypoint_match,
                  gear_signs_match=[p['gear'] for p in parts] == run['directions'])
    return dict(passed=all(checks.values()), checks=checks)


def ratios(summary):
    return [summary['max_'+key]/summary['omega_hard_limit_with_tolerance' if key.endswith('omega')
            else 'alpha_hard_limit_with_tolerance'] for key in WHEELS]


def peak_checks(stage, limits):
    proof, all_valid = {}, True
    for grid, summary_key in GRID_KEYS.items():
        summary = stage['audit'][summary_key]
        locations = stage['wheel_peak_locations'][grid]
        grid_proof = {}
        for key in WHEELS:
            location = locations[key]
            value = wheel_at(stage, location['t'], limits)
            valid = (close(value[key], location['wheel_value'], 1e-7) and
                     close(abs(location['wheel_value']), summary['max_'+key], 1e-7) and
                     value['segment_index'] == location['segment_index'] and
                     value['piece_index'] == location['piece_index'])
            grid_proof[key] = dict(recorded=location, independent=value, passed=valid)
            all_valid = all_valid and valid
        proof[grid] = grid_proof
    return dict(passed=bool(all_valid), grids=proof)


def parameters(trial, build):
    loaded = yaml.safe_load((trial/'parameters.yaml').read_text())
    prefix = '/remani_planner_node/'
    get = lambda key: float(loaded[prefix+key])
    omega, alpha, radius = (get('mm/'+key) for key in
        ('mobile_base_max_wheel_omega', 'mobile_base_max_wheel_alpha', 'mobile_base_wheel_radius'))
    values = [get('optimization/weight_base_feasibility'), omega*radius,
              get('mm/mobile_base_non_singul_vel'), get('mm/mobile_base_wheel_base'),
              radius, omega, alpha]
    k = int(get('optimization/constrain_points_perPiece'))
    dense = int(get('optimization/dense_sample_resolution'))
    proof = source_contract(build)
    manifest = load(build/'build_manifest.json')
    config = build/'source/mm_config.cpp'
    if sha(config) != manifest['isolated_source_sha256']['mm_config.cpp']:
        raise ValueError('MMConfig source differs from the attested build')
    text = config.read_text()
    if not all(token in text for token in (
            'mobile_base_max_vel_ = mobile_base_max_wheel_omega_ * mobile_base_wheel_radius_;',
            'mobile_base_max_acc_ = mobile_base_max_wheel_alpha_ * mobile_base_wheel_radius_;')):
        raise ValueError('Unknown planner linear-limit derivation')
    return values, k, dense, dict(base_feasibility_source=proof,
        planner_max_velocity_m_s=omega*radius, planner_max_acceleration_m_s2=alpha*radius,
        time_weight=get('optimization/weight_time'), time_cost='weight_time * sum(real_piece_times)',
        time_is_positive_but_not_hard_upper_bounded=True,
        mm_config_source_sha256=sha(config), parameter_sha256=sha(trial/'parameters.yaml'))


def cost_breakdown(stage, params, k, dense):
    totals = np.zeros(len(COST_CHANNELS))
    minimum, below, count = math.inf, 0, 0
    for part in stage['parts']:
        c = np.asarray(part['b_rows_ascending_power'], dtype=float)
        for piece, duration in enumerate(part['times']):
            nodes = piece_nodes(c[8*piece:8*(piece+1)], duration, k, dense)
            for kind in ('primary', 'dense'):
                v, a, j, weights = nodes[kind]
                if not len(v):
                    continue
                speeds = np.linalg.norm(v, axis=1)
                minimum = min(minimum, float(np.min(speeds)))
                below += int(np.sum(speeds < 1e-4))
                count += len(v)
                totals += np.sum(weights[:, None]*point_costs(v, a, j, params,
                    part['gear'], kind == 'dense'), axis=0)
    recorded = stage['cost_components']['base_feasibility']
    error = abs(float(sum(totals))-recorded)/max(1., abs(recorded), abs(float(sum(totals))))
    return dict(costs=dict(zip(COST_CHANNELS, totals.tolist())), sum=float(sum(totals)),
        recorded_base_feasibility=recorded, normalized_sum_error=error, passed=error <= 1e-7,
        actual_objective_nodes=count, minimum_raw_speed_m_s=minimum, nodes_below_floor=below,
        scope='independent six cost branches on original primary/triggered-dense quadrature; not a safety gate')


def initial_comparison(a, b, scale):
    x, y = a['stages'][0]['x'], b['stages'][0]['x']
    parts = a['stages'][0]['parts']
    dim = len(parts[0]['initial_state'])
    pieces = [len(part['times']) for part in parts]
    start, count = sum(dim*(n-1) for n in pieces), sum(pieces)
    same_non_time = (len(x) == len(y) and x[:start] == y[:start] and x[start+count:] == y[start+count:])
    same_boundaries = (len(parts) == len(b['stages'][0]['parts']) and all(
        left['initial_state'] == right['initial_state'] and left['final_state'] == right['final_state']
        for left, right in zip(parts, b['stages'][0]['parts'])))
    time_scale = len(parts) == len(b['stages'][0]['parts']) and all(
        close(np.asarray(left['times'])*scale, right['times'], 1e-12)
        for left, right in zip(parts, b['stages'][0]['parts']))
    requested = states(a, 'requested_initial_times'), states(b, 'requested_initial_times')
    requested_scale = len(requested[0]) == len(requested[1]) and all(
        close(left*scale, right, 1e-12) for left, right in zip(*requested))
    same_first = all(a['solver'][key] == b['solver'][key]
        for key in ('first_x_hash', 'first_cost', 'first_gradient_hash'))
    equal_effective = all(same_states(a, b, key) for key in
        ('effective_initial_states', 'effective_final_states', 'requested_initial_states', 'requested_final_states'))
    checks = dict(non_time_x_equal=same_non_time, initial_decoded_pvaj_equal=same_boundaries,
        initial_real_times_scale_correct=time_scale, requested_real_times_scale_correct=requested_scale,
        effective_and_requested_boundaries_equal=equal_effective, directions_equal=a['directions'] == b['directions'],
        limits_equal=a['limits'] == b['limits'], original_first_x_f_g_equal=same_first if scale == 1 else None,
        original_input_hash_equal=a['input_hash'] == b['input_hash'] if scale == 1 else None)
    return dict(passed=all(v for v in checks.values() if v is not None), checks=checks,
        first_x_hashes=[a['solver']['first_x_hash'], b['solver']['first_x_hash']],
        first_gradient_hashes=[a['solver']['first_gradient_hash'], b['solver']['first_gradient_hash']],
        first_costs=[a['solver']['first_cost'], b['solver']['first_cost']],
        gradient_proof='recorded first-gradient byte hash; gradient vector itself is not dumped')


def analyze_run(run, params, k, dense, overlay):
    stages = run['stages']
    if not stages or stages[0]['label'] != 'initial':
        raise ValueError('Missing unique canonical initial candidate')
    solver, frozen, limits = run['solver'], run['frozen_checks'], run['limits']
    identities = dict(first_x_hash=vector_hash(stages[0]['x']) == solver['first_x_hash'],
        first_cost=stages[0]['cost'] == solver['first_cost'],
        gradient_hash_recorded=str(solver['first_gradient_hash']) not in ('0', '', 'None'),
        stage_count=len(stages) == solver['recorded_stage_count'] == solver['accepted_count']+1,
        solver_finished=solver['finished_hook_seen'] is True,
        limits_match_parameters=close([limits['wheel_base'], limits['wheel_radius'],
            limits['wheel_limit'], limits['alpha_limit']], params[3:]) and
            limits['quadrature_points'] == k and limits['dense_resolution'] == dense,
        map_stable=frozen['stable'] is True and frozen['solver_entry_exit_key_equal'] is True and
            frozen['evaluate_overlay_changed_count'] == 0 and all(frozen[key] == overlay
                for key in ('overlay_begin', 'overlay_end', 'solver_entry', 'solver_exit')),
        final_saved_identity=frozen['last_matches_saved_candidate'] is True,
        not_published=run['diagnostic_candidates_published'] is False)
    analyzed = []
    for stage in stages:
        bounds = boundary_checks(stage)
        coordinates = coordinate_checks(stage, run, params[2])
        peaks = peak_checks(stage, limits)
        costs = cost_breakdown(stage, params, k, dense)
        analyzed.append(dict(index=stage['index'], label=stage['label'], iteration=stage['iteration'],
            cost=stage['cost'], cost_components=stage['cost_components'], elapsed_s=stage['elapsed_s'],
            native_IsTrajSafe_pass=stage['native_IsTrajSafe_pass'], native_reason=stage['native_reason'],
            native_reject_time=stage['native_reject_time'],
            duration_s=sum(sum(p['times']) for p in stage['parts']),
            wheel_ratios_by_grid={grid:ratios(stage['audit'][key]) for grid, key in GRID_KEYS.items()},
            native_summary=stage['audit']['summary'], boundary_proof=bounds,
            canonical_coordinate_proof=coordinates,
            wheel_peak_proof=peaks, base_feasibility_breakdown=costs,
            evidence_valid=bounds['passed'] and coordinates['passed'] and peaks['passed'] and costs['passed'] and
                stage['cost_components_recorded'] is True))
    last, gate = stages[-1], run['actual_native_gate']
    gate_agrees = (gate['pass'] == last['native_IsTrajSafe_pass'] and gate['reason'] == last['native_reason'] and
                  close(gate['rejection_value'], last['native_rejection_value'], 1e-7) and
                  close(gate['rejection_time'], last['native_reject_time'], 1e-9)) if gate['seen'] else None
    identities['actual_gate_matches_canonical_final'] = gate_agrees is not False
    identities['successful_return_requires_actual_gate'] = (not run['return_success'] or
                                                           (gate['seen'] and gate['pass']))
    return dict(label=run['label'], input_hash=run['input_hash'], budget_s=run['budget_s'],
        time_scale=run['time_scale'], return_success=run['return_success'], solver=solver,
        actual_native_gate=gate, actual_return_diagnosis=run['actual_return_diagnosis'],
        identity_checks=identities, evidence_valid=all(identities.values()) and all(s['evidence_valid'] for s in analyzed),
        stages=analyzed, initial=analyzed[0], final=analyzed[-1],
        posterior_intermediate_gate_passes_counted_as_delivery=False)


def interpretation(rows, valid):
    if not valid:
        return '证据核验未全部通过；本批保留结果，但不作预算或时间初始化的因果归因。'
    base, longer, timed = [r['return_success'] for r in rows]
    if base:
        return '暂停后的原始0.6秒组已通过，本次未稳定复现原失败；不能将另外两组的结果称为修复收益。'
    if longer and timed:
        return '延长预算和调整时间初始化均可解此输入；两种因素都有帮助，不能选定唯一主因。'
    if longer:
        return '仅延长预算组正常返回并通过原生门，支持当前输入受0.6秒计算预算限制；不代表线上1.2秒可用。'
    if timed:
        return '仅时间初始化组正常返回并通过原生门，支持当前输入对时间初始化敏感；不证明初始空间路径本身无解。'
    return '两种小范围调整均未交出原生合格结果；不能据此宣称几何无解或继续增加预算必然有效。'


def plot_peaks(rows, out):
    fig, axes = plt.subplots(1, 3, figsize=(12, 4.1), sharey=True, constrained_layout=True)
    x = np.arange(4)
    for ax, row, label in zip(axes, rows, LABELS):
        for at, stage, color, name in ((-.18, row['initial'], '#999999', '初始'),
                                       (.18, row['final'], '#246da0', '最后')):
            values = stage['wheel_ratios_by_grid']['native']
            ax.bar(x+at, values, .34, color=color, label=name)
            for xx, value in zip(x+at, values):
                ax.text(xx, value, '%.2f' % value, ha='center', va='bottom', fontsize=8)
        ax.axhline(1, color='#b5372c', ls='--', lw=1, label='含5%容差的验收上限')
        ax.set_xticks(x, ['左速度', '右速度', '左加速度', '右加速度'], rotation=20)
        ax.set_title(label+'\n最后原生门：'+candidate.REASONS.get(
            row['actual_native_gate']['reason'], row['actual_native_gate']['reason'] or '未执行'), fontsize=10)
        ax.grid(axis='y', alpha=.15)
    axes[0].set_ylabel('原生10ms网格峰值 / 验收上限')
    axes[0].legend(fontsize=8)
    fig.suptitle('同一冻结输入：四轮约束初末对比（>1为超限；不是在线交付率）', fontsize=12)
    fig.savefig(out, dpi=170)
    plt.close(fig)


def plot_curves(runs, out):
    fig, axes = plt.subplots(2, 3, figsize=(13, 6), sharey='row', constrained_layout=True)
    for col, (run, label) in enumerate(zip(runs, LABELS)):
        for row, kind in enumerate(('omega', 'alpha')):
            ax = axes[row, col]
            for stage, style, name in ((run['stages'][0], '--', '初始'), (run['stages'][-1], '-', '最后')):
                data = {r['t']:r for r in candidate.samples(stage)+candidate.samples(stage, 'start_samples')}
                data = [data[t] for t in sorted(data)]
                for side, color, side_name in (('left', '#246da0', '左轮'), ('right', '#d07c26', '右轮')):
                    limit = stage['audit']['summary']['omega_hard_limit_with_tolerance' if kind == 'omega'
                                                      else 'alpha_hard_limit_with_tolerance']
                    ax.plot(candidate.array(data, 't'), candidate.array(data, 'wheel_'+side+'_'+kind)/limit,
                            ls=style, color=color, lw=1, alpha=.85, label=name+side_name)
            for sign in (-1, 1):
                ax.axhline(sign, color='#b5372c', ls=':', lw=.9)
            ax.axvspan(0, .05, color='#999999', alpha=.2)
            ax.grid(alpha=.15)
            if row == 0:
                ax.set_title(label, fontsize=10)
            if col == 0:
                ax.set_ylabel(('轮速' if kind == 'omega' else '轮加速度')+' / 验收上限')
            if row == 1:
                ax.set_xlabel('候选轨迹相对时间 / s')
    axes[0, 0].legend(fontsize=8, ncol=2)
    for row in range(2):
        maximum = max(max(abs(y) for y in line.get_ydata()) for ax in axes[row]
                      for line in ax.lines if len(line.get_ydata()))
        if maximum > 20:
            for ax in axes[row]:
                ax.set_yscale('symlog', linthresh=1.)
            axes[row, 0].set_ylabel(axes[row, 0].get_ylabel()+'（对称对数）')
    fig.suptitle('三组初末轮曲线；虚线=初始，实线=最后；灰区0–0.05s不属原生验收', fontsize=12)
    fig.savefig(out, dpi=170)
    plt.close(fig)


def write_text(result, out):
    shown = candidate.shown
    rows = result.get('runs', [])
    lines = ['轮约束失败主因：一次固定输入三组诊断', result['conclusion'],
        '同一次新鲜pin=1轮约束失败后暂停仿真，三组用同一冻结环境；不是三次在线导航。',
        '只比较原始时间/0.6秒、原始时间/1.2秒、初始时间×1.5/0.6秒；轮限制、外形与边界不放宽。',
        '证据核验：'+('通过' if result.get('evidence_valid') else '未全部通过'), '']
    for row, label in zip(rows, LABELS):
        initial, final, gate = row['initial'], row['final'], row['actual_native_gate']
        reason = candidate.REASONS.get(gate['reason'], gate['reason']) if gate['seen'] else '没有执行'
        lines += [label,
            '正常返回并通过原生门=%s；实际原生门=%s；求解%.1f毫秒，真正接受%s步。' %
            (row['return_success'], reason, row['solver']['elapsed_s']*1000, row['solver']['accepted_count']),
            '初始/最后原生重查=%s/%s；候选总时长%s→%s秒；最大轮约束比例%s→%s。' %
            (initial['native_IsTrajSafe_pass'], final['native_IsTrajSafe_pass'],
             shown(initial['duration_s']), shown(final['duration_s']),
             shown(max(initial['wheel_ratios_by_grid']['native'])), shown(max(final['wheel_ratios_by_grid']['native']))),
            '最后四轮比例（左速度/右速度/左加速度/右加速度）：'+
            '/'.join(shown(v) for v in final['wheel_ratios_by_grid']['native'])+'。',
            '最后峰值比例：原生密网格%s，粗采样%s，原始低速加密网格%s。' %
            tuple(shown(max(final['wheel_ratios_by_grid'][key])) for key in ('native', 'K16', 'dense'))]
        costs = final['base_feasibility_breakdown']
        peak_key = WHEELS[int(np.argmax(final['wheel_ratios_by_grid']['native']))]
        peak = final['wheel_peak_proof']['grids']['native'][peak_key]
        lines += ['最后最坏轮峰发生于%s秒，原始底盘速度%s m/s；优化实际采样最低速度%s m/s，低于1e-4的点%s个。' %
            (shown(peak['recorded']['t']), shown(peak['independent']['raw_speed_m_s'], 5),
             shown(costs['minimum_raw_speed_m_s'], 5), costs['nodes_below_floor']),
            '底盘运动代价%s→%s；最后六分支中低速惩罚%s、四轮惩罚合计%s，独立求和误差%s。' %
            (shown(initial['base_feasibility_breakdown']['sum'], 2), shown(costs['sum'], 2),
             shown(costs['costs']['dense_min_speed'], 2),
             shown(sum(costs['costs'][name] for name in COST_CHANNELS[2:]), 2), shown(costs['normalized_sum_error'], 9)), '']
    if rows:
        lines += ['怎样解释',
            '首x/f/g匹配只要求原始时间两组；×1.5组核对非时间x、边界与真实时长比例。g只记录首梯度哈希，未独立重算梯度向量。',
            '保留P/V/A/J重新生成MINCO会改变段内空间曲线；×1.5不是完全相同路径的慢放，也不保证轮峰值单调下降。',
            '1.5只约束初始时间扰动，优化后的时间仍可变化；没有新增时长硬上限。',
            '低速公式含1/|v|²及1/|v|⁴，换向处须看两侧；有截断不等于物理奇异性已解决。未触发截断也不能直接归咎于该截断。',
            '“左轮首次失败”不代表只有左轮失败；四轮均完整扫描。运动代价还包含低速惩罚，不等同于轮加速度代价。']
        source = result['parameter_evidence']
        lines += ['后端max_vel/max_acc由轮参数计算，为%s m/s、%s m/s²；不能直接套用控制器limits文件。' %
                  (shown(source['planner_max_velocity_m_s']), shown(source['planner_max_acceleration_m_s2'])),
            '时间代价为%s×总时长，会与软约束竞争；这只是源码支持的线索，不能单独证明本轮失败主因。' % shown(source['time_weight']),
            '0.6/1.2秒是纯求解预算；验收、重建和事后审计不在其中。未验证线上发布、最新环境验收或MPPI采用。',
            '中间候选只做轮运动摘要，未执行完整原生门；未检查不计为失败，更不能算交付成功。',
            '一次固定输入的数值ROS SIL，不是统计成功率、连续时间安全证明、Gazebo或实车结果。']
    (out/'实验结论.txt').write_text('\n'.join(lines)+'\n')


def report(batch, out):
    batch, out = Path(batch).resolve(), Path(out).resolve()
    if out.exists():
        raise FileExistsError('Use a fresh report directory: '+str(out))
    if batch == out or out in batch.parents:
        raise ValueError('Report must not overwrite input')
    complete, plan = load(batch/'complete.json'), load(batch/'plan.json')
    if complete.get('complete') is not True or len(plan['trials']) != 1:
        raise ValueError('The one predeclared capture batch must be finished')
    trial = batch/plan['trials'][0]['directory']
    path = trial/'candidate_audit/wheel_cause_replay.jsonl'
    inputs = [batch/'complete.json', batch/'plan.json', trial/'run_status.json', trial/'manifest.json']
    result = dict(schema='wheel_cause_report_v1', source=str(batch),
        navigation_metrics_valid=False, diagnostic_candidates_published=False,
        online_performance_report=False, runs=[], evidence_valid=False,
        reporter_sha256=sha(Path(__file__)),
        helper_sha256={name:sha(HERE/name) for name in
            ('audit_delivery_iterates.py', 'backend_base_feasibility_breakdown.py', 'report_candidate_audit.py')})
    if not path.is_file():
        result['conclusion'] = '本次未产生完整三组诊断记录；保留本次结果，没有更换输入补样本。'
        result['run_status'] = load(trial/'run_status.json')
    else:
        records = [loads_preserving_cpp_zero(line) for line in path.read_text().splitlines() if line.strip()]
        if any(r.get('schema') != 'wheel_cause_replay_v1' for r in records):
            raise ValueError('Unknown wheel-cause replay schema')
        runs = [r for r in records if r.get('kind') == 'run']
        ends = [r for r in records if r.get('kind') == 'complete']
        if len(ends) != 1 or [r['label'] for r in runs] != list(GROUPS) or records[-1] != ends[0]:
            raise ValueError('Expected exactly the predeclared three runs and final completion record')
        if any(r.get('capture_error') for r in runs):
            raise ValueError('A captured replay is incomplete; retain it without causal interpretation')
        end = ends[0]
        build = Path(plan['build'])
        if sha(build/'build_manifest.json') != plan['build_manifest_sha256']:
            raise ValueError('Build identity changed after the capture plan')
        case_path = trial/'candidate_audit/case.json'
        case = load(case_path)
        params, k, dense, parameter_evidence = parameters(trial, build)
        overlay = runs[0]['frozen_checks']['overlay_begin']
        rows = [analyze_run(r, params, k, dense, overlay) for r in runs]
        budget_pair, time_pair = initial_comparison(runs[0], runs[1], 1.), initial_comparison(runs[0], runs[2], 1.5)
        failure_path = trial/'failure_trace.jsonl'
        failures = [loads_preserving_cpp_zero(line) for line in failure_path.read_text().splitlines() if line.strip()]
        original_failures = [row for row in failures if row.get('kind') == 'solve' and
            row.get('label') == 'normal' and str(row.get('input_hash')) == str(case['input_hash'])]
        original_failure = original_failures[0] if len(original_failures) == 1 else {}
        original_signature = dict(
            input_hash=case['input_hash'] == runs[0]['input_hash'],
            initial_x=vector_hash(case['stages'][0]['x']) == runs[0]['solver']['first_x_hash'],
            initial_cost=case['stages'][0]['cost'] == runs[0]['solver']['first_cost'],
            original_failure_gradient=str(original_failure.get('first_gradient_hash')) == runs[0]['solver']['first_gradient_hash'],
            original_failure_cost=original_failure.get('first_cost') == runs[0]['solver']['first_cost'],
            original_failure_reason=original_failure.get('reason') == case['reason'] and original_failure.get('success') is False,
            effective_initial=same_states(case, runs[0], 'effective_initial_states'),
            effective_final=same_states(case, runs[0], 'effective_final_states'))
        capture_checks = dict(
            all_captures_complete=end.get('all_captures_complete') is True,
            environment_stable=end.get('environment_stable') is True,
            first_x_f_g=end.get('original_budgets_same_first_x_f_g') is True,
            retime_coordinates=end.get('retime_only_initial_time_coordinates') is True,
            original_pin=end.get('original_solver_pin_owned') is True,
            original_map=end.get('original_solver_entry_exit_equal') is True,
            limits_not_relaxed=end.get('constraints_relaxed') is False,
            not_published=end.get('diagnostic_candidates_published') is False,
            settings=all(close(r['budget_s'], budget) and close(r['time_scale'], scale)
                for r, budget, scale in zip(runs, (.6, 1.2, .6), (1., 1., 1.5))),
            actual_67_spheres=len(load(trial/'native_collision_3d_body.json')['body']) == 67,
            original_signature=all(original_signature.values()))
        valid = all(capture_checks.values()) and budget_pair['passed'] and time_pair['passed'] and all(r['evidence_valid'] for r in rows)
        result.update(runs=rows, evidence_valid=valid, capture_checks=capture_checks,
            original_capture_signature=original_signature, budget_pair=budget_pair, time_initialization_pair=time_pair,
            original_failed_solve=original_failure,
            completion=end, parameter_evidence=parameter_evidence,
            captured_original_reason=case['reason'], captured_request_id=case['request_id'],
            conclusion=interpretation(rows, valid),
            limitations=['No online delivery, latest-map acceptance, or controller consumption is certified.',
                'Canonical posterior checks are outside the solver budget; intermediate null native gates are not failures.',
                'Regenerating with fixed PVAJ and altered T also changes the between-waypoint spatial curve.',
                'Initial 1.5x time perturbation is bounded, but the ordinary positive time coordinates have no hard upper bound.',
                'Full gradients are not dumped; only their original first-evaluation hashes are compared.',
                'Finite native and objective grids do not certify continuous-time extrema.'])
        inputs += [path, case_path, failure_path, trial/'parameters.yaml', trial/'native_collision_3d_body.json', build/'build_manifest.json']
        # Do not draw causal-comparison graphics if input/provenance checks fail.
        if valid:
            out.mkdir(parents=True)
            font = FontProperties(fname=FONT)
            plt.rcParams.update({'font.family':font.get_name(), 'axes.unicode_minus':False,
                                 'font.size':9, 'savefig.facecolor':'white'})
            plot_peaks(rows, out/'四轮违规对比.png')
            plot_curves(runs, out/'三组初末轮曲线.png')
            result['figures'] = ['四轮违规对比.png', '三组初末轮曲线.png']
        result['visual_scope'] = 'Recorded initial/final wheel curves only; no simulated robot movement or invented intermediate animation.'
    result['input_sha256'] = {str(p):sha(p) for p in inputs}
    out.mkdir(parents=True, exist_ok=True)
    write_text(result, out)
    (out/'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    for name, expected in result['input_sha256'].items():
        if sha(name) != expected:
            raise RuntimeError('Input changed while reporting: '+name)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch', type=Path, default=HERE/'results/wheel_cause_20261030_01')
    parser.add_argument('--out', type=Path, default=HERE/'results/wheel_cause_report_20261030_01')
    args = parser.parse_args()
    result = report(args.batch, args.out)
    print(json.dumps(dict(out=str(args.out), evidence_valid=result['evidence_valid'],
                         conclusion=result['conclusion']), ensure_ascii=False))


if __name__ == '__main__':
    main()
