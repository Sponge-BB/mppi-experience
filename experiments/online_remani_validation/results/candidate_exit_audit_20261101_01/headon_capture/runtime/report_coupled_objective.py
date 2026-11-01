#!/usr/bin/env python3
"""Evidence-only report for online objective-coupling ablations.

Reads plain or losslessly archived records.  No ROS, optimizer, controller or
renderer is started.  Failed navigation and undelivered references are retained.
"""
import argparse
from collections import Counter, defaultdict
import csv
import gzip
import hashlib
import json
import math
import re
from pathlib import Path

import numpy as np


GROUPS = ('native', 'engineering', 'execution', 'handover', 'coupled')
TITLES = {'native': '串联基准', 'engineering': '附加投影资格/软代价',
          'execution': '可执行性项', 'handover': '交接项', 'coupled': '两项耦合'}
FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'


def read(path, default=None):
    path = Path(path)
    if path.exists():
        with path.open() as stream:
            return json.load(stream)
    archive = path.with_name(path.name + '.gz')
    if archive.exists():
        with gzip.open(archive, 'rt') as stream:
            return json.load(stream)
    return default


def lines(path):
    path = Path(path)
    if path.exists():
        stream = path.open()
    elif path.with_name(path.name + '.gz').exists():
        stream = gzip.open(path.with_name(path.name + '.gz'), 'rt')
    else:
        return []
    with stream:
        return [json.loads(line) for line in stream if line.strip()]


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def percentile(values, percentile_value):
    values = [v for v in values if finite(v)]
    return float(np.percentile(values, percentile_value)) if values else None


def ratio(numerator, denominator):
    return float(numerator) / denominator if denominator else None


def solver_intervals(resource):
    """Native solve IDs and clocks, never observer-tail times."""
    active = {}
    intervals = []
    for row in resource:
        if row.get('stage') != 'optimizer':
            continue
        key = str(row.get('id'))
        if row.get('event') == 'begin':
            active[key] = row
        elif row.get('event') == 'end' and key in active:
            begin = active.pop(key)
            intervals.append(dict(id=key, begin=begin['mono'], end=row['mono']))
    return intervals


def coupling_metrics(native, resource, request_rows):
    """Report observed scopes and actual cost use, not merely enabled flags.

    Cost-event call IDs are preferred. If absent, assign by native monotonic
    optimizer intervals. Unmatched cost events are reported, NOT silently used
    to claim coverage. Missing fields remain unknown rather than zero.
    """
    requests = {r['request_id']: r for r in request_rows if r['request_id'] > 1}
    # A normally terminated run may cut off its final request. Its completed
    # optimizer calls and coupling snapshots still belong in the coverage.
    for row in native:
        if row.get('event') == 'request_begin' and row.get('request_id', 0) > 1:
            requests.setdefault(row['request_id'], dict(request_id=row['request_id'], optimizer_calls=[]))
    scope = [r for r in native if r.get('event') == 'coupling_scope' and r.get('request_id') in requests]
    scope_valid = [r for r in scope if r.get('valid') is True or r.get('snapshot_valid') is True]
    scope_known = [r for r in scope if isinstance(r.get('valid', r.get('snapshot_valid')), bool)]
    intervals = solver_intervals(resource)
    calls = {str(call['resource_optimizer_id']) for request in requests.values()
             for call in request.get('optimizer_calls', [])}
    request_spans = {}
    for row in native:
        ident = row.get('request_id')
        if ident not in requests:
            continue
        if row.get('event') == 'request_begin':
            request_spans[ident] = [row['mono'], float('inf')]
        elif row.get('event') == 'request_end' and ident in request_spans:
            request_spans[ident][1] = row['mono']
    for interval in intervals:
        if any(begin - .001 <= interval['begin'] <= interval['end'] <= finish + .001
               for begin, finish in request_spans.values()):
            calls.add(interval['id'])
    matched = defaultdict(list)
    unmatched = []
    events = [r for r in native if r.get('event') in ('coupling_cost', 'coupling_objective')
              and r.get('request_id') in requests]
    for row in events:
        ident = row.get('resource_optimizer_id', row.get('optimizer_id', row.get('solve_id')))
        scope_row = next((v for v in scope if v.get('scope_id') == row.get('scope_id')
                          and v.get('scope_id') is not None), None)
        if ident is None and scope_row is not None and finite(scope_row.get('mono')):
            candidates = [v for v in intervals if v['begin'] - .001 <= scope_row['mono'] <= v['end'] + .001]
            if len(candidates) == 1:
                ident = candidates[0]['id']
        if ident is None and finite(row.get('mono')):
            candidates = [v for v in intervals if v['begin'] - .001 <= row['mono'] <= v['end'] + .001]
            if len(candidates) == 1:
                ident = candidates[0]['id']
        if str(ident) in calls:
            matched[str(ident)].append(row)
        else:
            unmatched.append(row)
    exec_keys = ('execution_cost', 'exec_cost', 'max_execution_cost', 'max_exec_cost')
    hand_keys = ('delivery_cost', 'handover_cost', 'hand_cost', 'max_handover_cost', 'max_hand_cost')

    def cost(rows, keys):
        return [r[k] for r in rows for k in keys if finite(r.get(k))]

    execution = {key for key, rows in matched.items() if any(v > 1e-12 for v in cost(rows, exec_keys))
                 or any(finite(r.get('nonzero_execution_calls')) and r['nonzero_execution_calls'] > 0 for r in rows)}
    handover = {key for key, rows in matched.items() if any(v > 1e-12 for v in cost(rows, hand_keys))
                or any(finite(r.get('nonzero_handover_calls')) and r['nonzero_handover_calls'] > 0 for r in rows)}
    any_active = execution | handover | {
        key for key, records in matched.items()
        if any(finite(r.get('active_callback_count')) and r['active_callback_count'] > 0 for r in records)}
    final = {}
    for row in events:
        if row.get('phase') == 'final' and row.get('scope_id') is not None:
            final[row['scope_id']] = row
    callbacks = sum(r.get('callback_count', 0) for r in final.values() if finite(r.get('callback_count')))
    active_callbacks = sum(r.get('active_callback_count', 0) for r in final.values() if finite(r.get('active_callback_count')))
    # Term costs are only sampled at initial/final callbacks. A single enabled
    # term can additionally be inferred from an intermediate active callback;
    # two enabled terms cannot be disambiguated without per-term counters.
    inferred_execution, inferred_handover = set(execution), set(handover)
    mixed_ambiguous = set()
    for scope_row in scope:
        cost_rows = [r for r in events if r.get('scope_id') == scope_row.get('scope_id')]
        active = any(finite(r.get('active_callback_count')) and r['active_callback_count'] > 0 for r in cost_rows)
        ids = [key for key, records in matched.items() if any(r in records for r in cost_rows)]
        ew, hw = scope_row.get('exec_weight'), scope_row.get('hand_weight')
        if active and finite(ew) and finite(hw):
            if ew > 0 and hw == 0:
                inferred_execution.update(ids)
            elif hw > 0 and ew == 0:
                inferred_handover.update(ids)
            elif ew > 0 and hw > 0:
                mixed_ambiguous.update(key for key in ids if key not in execution or key not in handover)
    return dict(scope_events=len(scope), valid_snapshot_scopes=len(scope_valid),
                unknown_snapshot_scopes=len(scope) - len(scope_known),
                valid_snapshot_scope_rate=ratio(len(scope_valid), len(scope)),
                snapshot_reasons=dict(Counter(r.get('reason', 'unspecified') for r in scope)),
                refresh_optimizer_calls=len(calls), cost_recorded_optimizer_calls=len(matched),
                cost_recording_coverage=ratio(len(matched), len(calls)),
                execution_nonzero_optimizer_calls=len(execution),
                handover_nonzero_optimizer_calls=len(handover),
                any_nonzero_optimizer_calls=len(any_active),
                execution_nonzero_coverage=ratio(len(execution), len(calls)),
                handover_nonzero_coverage=ratio(len(handover), len(calls)),
                execution_initial_final_nonzero_coverage=ratio(len(execution), len(calls)),
                delivery_initial_final_nonzero_coverage=ratio(len(handover), len(calls)),
                execution_inferred_any_callback_coverage=ratio(len(inferred_execution), len(calls)),
                delivery_inferred_any_callback_coverage=ratio(len(inferred_handover), len(calls)),
                mixed_term_callback_ambiguous_calls=len(mixed_ambiguous),
                any_nonzero_coverage=ratio(len(any_active), len(calls)),
                final_scope_events=len(final),
                final_active_scope_events=sum(r.get('active') is True for r in final.values()),
                objective_callback_count=callbacks, active_objective_callback_count=active_callbacks,
                active_objective_callback_rate=ratio(active_callbacks, callbacks),
                unmatched_cost_events=len(unmatched),
                note='refresh solves only; deduplicated optimizer IDs; enabled weights do not establish actual coupling')


def measured_advance(state, consumed, end, distance=.05):
    """First 5 cm translation after reference consumption, not goal progress."""
    if consumed is None or not len(state) or not {'t', 'x', 'y'}.issubset(state.dtype.names):
        return None
    selected = state[(state['t'] >= consumed) & (state['t'] <= end)]
    if len(selected) < 2:
        return None
    displacement = np.hypot(selected['x'] - selected['x'][0], selected['y'] - selected['y'][0])
    indices = np.flatnonzero(displacement >= distance)
    return float(selected['t'][indices[0]] - consumed) if len(indices) else None


def cpu_metrics(trial):
    values = defaultdict(list)
    for row in lines(trial / 'cpu_samples.jsonl'):
        for process in row.get('processes', []):
            if process.get('name') in ('remani_planner_node', 'base_mppi') and finite(process.get('cpu_pct')):
                values[process['name']].append(process['cpu_pct'])
    return {name + '_cpu_pct_p95': percentile(values[name], 95)
            for name in ('remani_planner_node', 'base_mppi')}


def measure(trial):
    trial = Path(trial)
    manifest = read(trial / 'manifest.json', {})
    settings = manifest.get('coupling_experiment')
    if not settings or settings.get('group') not in GROUPS:
        raise ValueError('Not a named objective-coupling trial (old A-D groups forbidden): ' + str(trial))
    if settings.get('legacy_rolling') or settings.get('legacy_prefix') or settings.get('diagnostic_pause'):
        raise ValueError('Legacy prefix/rolling or paused diagnostic cannot enter this online comparison')
    result = read(trial / 'result.json', {})
    status = read(trial / 'run_status.json', {})
    audit = read(trial / 'trial_audit.json', {})
    summary = read(trial / 'analysis/summary.json', {})
    native = lines(trial / 'dual_timeline.jsonl')
    resource = lines(trial / 'solver_trace.jsonl')
    rows = summary.get('requests', [])
    refreshes = [r for r in rows if r['request_id'] > 1]
    solves = [call for request in refreshes for call in request.get('optimizer_calls', [])]
    latency = summary.get('latency', {})
    blocked = latency.get('first_native_blocked_sim_s')
    # A first published candidate may never be consumed. Keep later actually
    # consumed candidates, instead of declaring the whole trial undelivered.
    chains = [r for r in summary.get('reference_chains', [])
              if blocked is not None and r.get('publish_sim_s', -float('inf')) >= blocked]
    consumes = [r['consume_sim_s'] for r in chains if finite(r.get('consume_sim_s'))]
    consumed = min(consumes) if consumes else None
    state_path = trial / 'state.csv'
    state = np.atleast_1d(np.genfromtxt(state_path, names=True, delimiter=',')) if state_path.exists() else np.empty(0)
    end = summary.get('end_sim_s', result.get('elapsed'))
    phases = summary.get('stop_by_phase', {}).get('phases', {})
    navigation = [v for key, v in phases.items() if key != 'terminal_settling']
    nearly_stopped = sum(v.get('measured_nearly_zero_motion_s', 0.) for v in navigation) if phases else None
    command_zero = sum(sum(v.get('zero_command_duration_s', {}).values()) for v in navigation) if phases else None
    c = coupling_metrics(native, resource, rows)
    refresh_ids = {r.get('request_id') for r in native if r.get('event') == 'request_begin' and r.get('request_id', 0) > 1}
    optimizer_entered = {r.get('request_id') for r in native if r.get('event') == 'stage_begin'
                         and r.get('stage') == 'optimizer' and r.get('request_id') in refresh_ids}
    optimizer_entered.update(r['request_id'] for r in refreshes if r.get('optimizer_calls'))
    preprocessing = [r for r in refreshes if r.get('success') is not True and not r.get('search_results')
                     and not r.get('optimizer_calls') and r['request_id'] not in optimizer_entered]
    stdout = (trial / 'launch_stdout.log').read_text(errors='replace') if (trial / 'launch_stdout.log').exists() else ''
    final_state = result.get('final_state', [])
    outcome = result.get('reason', 'invalid_or_incomplete_run')
    result_row = dict(path=str(trial), group=settings['group'], seed=settings.get('seed'), scene=manifest.get('scene'),
                      valid_experiment=audit.get('valid_experiment') is True,
                      audit_errors=audit.get('errors', []), outcome=outcome,
                      goal=outcome == 'goal', collision=result.get('collision_conservative_projection'),
                      min_clearance_m=result.get('min_clearance'),
                      final_goal_distance_m=float(np.linalg.norm(np.asarray(final_state[:2]) - [3., 0.])) if len(final_state) >= 2 else None,
                      elapsed_navigation_s=end - summary['anchor_sim_s'] if finite(end) and finite(summary.get('anchor_sim_s')) else None,
                      actual_nearly_stopped_s=nearly_stopped, command_zero_s=command_zero,
                      blocked_to_consume_s=consumed - blocked if consumed is not None else None,
                      delivery_status='not_triggered' if blocked is None else 'delivered' if consumed is not None else 'no_delivery',
                      blocked_to_end_lower_bound_s=end - blocked if blocked is not None and consumed is None and finite(end) else None,
                      consume_to_5cm_displacement_s=measured_advance(state, consumed, end) if finite(end) else None,
                      displacement_note='measured 5 cm from first post-consumption pose; not proof of forward task progress',
                      refresh_requests=len(refreshes), accepted_refresh_requests=sum(r.get('success') is True for r in refreshes),
                      request_outcomes=dict(Counter(r.get('outcome_class', 'unknown') for r in refreshes)),
                      request_wall_ms=[r['request_wall_ms'] for r in refreshes if finite(r.get('request_wall_ms'))],
                      optimizer_wall_ms=[s['wall_ms'] for s in solves if finite(s.get('wall_ms'))],
                      optimizer_thread_cpu_ms=[s['thread_cpu_ms'] for s in solves if finite(s.get('thread_cpu_ms'))],
                      optimizer_runqueue_wait_ms=[s['runqueue_wait_ms'] for s in solves if finite(s.get('runqueue_wait_ms'))],
                      optimizer_outcomes=dict(Counter(s.get('reason', 'unknown') for s in solves)),
                      unfinished_native_requests=len(summary.get('unmatched', {}).get('requests', [])),
                      fixed_arm_max_error=result.get('arm_max_error'), execution_weight=settings.get('execution_weight'),
                      handover_weight=settings.get('handover_weight'), engineering=settings.get('engineering'),
                      runner_error=status.get('runner_error'), coupling=c, **cpu_metrics(trial))
    result_row.update(request_total=sum(r.get('event') == 'request_begin' for r in native),
                      refresh_request_total=len(refresh_ids), refresh_optimizer_entered_requests=len(optimizer_entered),
                      preprocessing_failure_no_search_or_optimizer=len(preprocessing),
                      exact_target_warning_requests=sum(bool(r.get('target_selection_reject_source_ros')) for r in refreshes),
                      exact_target_warning_lines_all=stdout.count('No planning-layer-free local target found'),
                      exact_target_type0_warning_lines_all=len(re.findall(r'No planning-layer-free local target found[^\n]*\btype\s*[:=]?\s*0\b', stdout)))
    result_row.update(scope_events=c['scope_events'], valid_snapshot_scopes=c['valid_snapshot_scopes'],
                      valid_snapshot_scope_rate=c['valid_snapshot_scope_rate'],
                      refresh_optimizer_calls=c['refresh_optimizer_calls'],
                      cost_recording_coverage=c['cost_recording_coverage'],
                      any_nonzero_optimizer_coverage=c['any_nonzero_coverage'],
                      execution_nonzero_optimizer_coverage=c['execution_nonzero_coverage'],
                      delivery_nonzero_optimizer_coverage=c['handover_nonzero_coverage'],
                      objective_callback_count=c['objective_callback_count'],
                      active_objective_callback_count=c['active_objective_callback_count'],
                      active_objective_callback_rate=c['active_objective_callback_rate'],
                      execution_initial_final_nonzero_coverage=c['execution_initial_final_nonzero_coverage'],
                      delivery_initial_final_nonzero_coverage=c['delivery_initial_final_nonzero_coverage'],
                      execution_inferred_any_callback_coverage=c['execution_inferred_any_callback_coverage'],
                      delivery_inferred_any_callback_coverage=c['delivery_inferred_any_callback_coverage'],
                      stop_errors=status.get('stop_errors', []))
    result_row['request_wall_ms_p95'] = percentile(result_row['request_wall_ms'], 95)
    result_row['optimizer_wall_ms_p95'] = percentile(result_row['optimizer_wall_ms'], 95)
    return result_row


def csv_write(path, rows):
    columns = list(dict.fromkeys(k for row in rows for k in row))
    with Path(path).open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: json.dumps(value, ensure_ascii=False) if isinstance(value, (dict, list)) else value
                             for key, value in row.items()})


def aggregate(rows):
    output = {}
    for group in GROUPS:
        data = [r for r in rows if r['group'] == group]
        if not data:
            continue
        valid = [r for r in data if r['valid_experiment']]
        c = [r['coupling'] for r in valid]
        denominator = sum(r['refresh_optimizer_calls'] for r in c)
        scope_denominator = sum(r['scope_events'] for r in c)
        callbacks = sum(r['objective_callback_count'] for r in c)
        active_callbacks = sum(r['active_objective_callback_count'] for r in c)
        output[group] = dict(trials=len(data), valid_trials=len(valid),
            invalid_trials=len(data) - len(valid), goals=sum(r['goal'] for r in valid),
            collisions=sum(r['collision'] is True for r in valid),
            navigation_timeouts=sum(r['outcome'] == 'timeout' for r in valid),
            no_delivery_trials=sum(r['delivery_status'] == 'no_delivery' for r in valid),
            untriggered_trials=sum(r['delivery_status'] == 'not_triggered' for r in valid),
            delivered_trials=sum(r['delivery_status'] == 'delivered' for r in valid),
            request_total=sum(r['request_total'] for r in valid),
            refresh_request_total=sum(r['refresh_request_total'] for r in valid),
            refresh_optimizer_entered_requests=sum(r['refresh_optimizer_entered_requests'] for r in valid),
            preprocessing_failure_no_search_or_optimizer=sum(r['preprocessing_failure_no_search_or_optimizer'] for r in valid),
            exact_target_warning_requests=sum(r['exact_target_warning_requests'] for r in valid),
            exact_target_warning_lines_all=sum(r['exact_target_warning_lines_all'] for r in valid),
            blocked_to_consume_s_p50=percentile([r['blocked_to_consume_s'] for r in valid], 50),
            blocked_to_consume_s_p95=percentile([r['blocked_to_consume_s'] for r in valid], 95),
            actual_nearly_stopped_s_p50=percentile([r['actual_nearly_stopped_s'] for r in valid], 50),
            request_wall_ms_p95=percentile([v for r in valid for v in r['request_wall_ms']], 95),
            optimizer_wall_ms_p95=percentile([v for r in valid for v in r['optimizer_wall_ms']], 95),
            scope_events=scope_denominator, valid_snapshot_scopes=sum(r['valid_snapshot_scopes'] for r in c),
            valid_snapshot_scope_rate=ratio(sum(r['valid_snapshot_scopes'] for r in c), scope_denominator),
            unknown_snapshot_scopes=sum(r['unknown_snapshot_scopes'] for r in c),
            refresh_optimizer_calls=denominator,
            cost_recorded_optimizer_calls=sum(r['cost_recorded_optimizer_calls'] for r in c),
            any_nonzero_optimizer_calls=sum(r['any_nonzero_optimizer_calls'] for r in c),
            execution_nonzero_coverage=ratio(sum(r['execution_nonzero_optimizer_calls'] for r in c), denominator),
            handover_nonzero_coverage=ratio(sum(r['handover_nonzero_optimizer_calls'] for r in c), denominator),
            any_nonzero_coverage=ratio(sum(r['any_nonzero_optimizer_calls'] for r in c), denominator),
            cost_recording_coverage=ratio(sum(r['cost_recorded_optimizer_calls'] for r in c), denominator),
            objective_callback_count=callbacks, active_objective_callback_count=active_callbacks,
            active_objective_callback_rate=ratio(active_callbacks, callbacks),
            remani_cpu_trial_p95_pct_median=percentile([r['remani_planner_node_cpu_pct_p95'] for r in valid], 50),
            mppi_cpu_trial_p95_pct_median=percentile([r['base_mppi_cpu_pct_p95'] for r in valid], 50))
    return output


def plot(rows, totals, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    font = FontProperties(fname=FONT) if Path(FONT).exists() else FontProperties()
    groups = [g for g in GROUPS if g in totals]
    x = np.arange(len(groups))
    labels = ['投影补项' if g == 'engineering' else TITLES[g] for g in groups]
    figure, axes = plt.subplots(1, 3, figsize=(11.6, 3.6))
    panels = [('blocked_to_consume_s_p50', '受阻→消费新参考 / s'),
              ('actual_nearly_stopped_s_p50', '导航近静止时间 / s'),
              ('any_nonzero_coverage', '非零耦合代价求解覆盖率')]
    for axis, (key, title) in zip(axes, panels):
        values = [totals[g][key] for g in groups]
        axis.bar(x, [v if v is not None else float('nan') for v in values], color='#437ba6', width=.6)
        axis.set_xticks(x)
        axis.set_xticklabels(labels, fontproperties=font, rotation=20)
        axis.set_title(title, fontproperties=font, fontsize=10)
        for name in ('top', 'right'):
            axis.spines[name].set_visible(False)
        for index, value in enumerate(values):
            if value is None:
                axis.text(index, .02, 'N/A', transform=axis.get_xaxis_transform(), ha='center', fontsize=8)
        if key.endswith('coverage'):
            axis.set_ylim(0., 1.12)
        axis.grid(axis='y', alpha=.15)
    figure.suptitle('在线数值仿真初筛：未交付保留，缺失值不记作 0', fontproperties=font, fontsize=11)
    figure.tight_layout()
    figure.savefig(out / '对比结果.png', dpi=150)
    plt.close(figure)


def report(batches, out, allow_incomplete=False, plots=True):
    rows = []
    plans = []
    paths, identities = set(), set()
    for batch in batches:
        batch = Path(batch).resolve()
        plan = read(batch / 'experiment_plan.json', {})
        if not allow_incomplete and not read(batch / 'batch_complete.json', {}).get('complete'):
            raise RuntimeError('Incomplete batch; use --allow-incomplete for explicit provisional reporting')
        if plan.get('paired') or plan.get('legacy_rolling') or plan.get('legacy_prefix'):
            raise ValueError('Paused / legacy factorial batch cannot be pooled into coupling-objective report')
        plans.append(plan)
        expected = {(scene, group, seed) for scene in plan.get('scenes', [])
                    for group in plan.get('groups', []) for seed in plan.get('seeds', [])}
        batch_identities = set()
        for path in sorted(batch.glob('*/run_status.json')):
            row = measure(path.parent)
            key = (row['scene'], row['group'], row['seed'])
            if key in batch_identities:
                raise ValueError('Duplicate trial identity inside batch: ' + str(key))
            batch_identities.add(key)
            if path.parent.resolve() in paths:
                continue
            if key in identities:
                raise ValueError('Repeated scene/group/seed would rerun or duplicate a comparison: ' + str(key))
            paths.add(path.parent.resolve())
            identities.add(key)
            rows.append(row)
        if expected and (batch_identities - expected or (not allow_incomplete and batch_identities != expected)):
            raise ValueError('Observed trial identities do not match the complete original plan')
    if not rows:
        raise RuntimeError('No completed status records found')
    # Pooling groups from different binaries/weights would confound the
    # objective comparison. Separate pilots remain separate reports.
    for key in ('build_manifest_sha256', 'execution_weight', 'handover_weight'):
        values = {json.dumps(plan.get(key), sort_keys=True) for plan in plans}
        if len(values) > 1:
            raise ValueError('Cannot pool batches with different ' + key)
    out = Path(out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    totals = aggregate(rows)
    csv_write(out / '逐次结果.csv', rows)
    csv_write(out / '分组汇总.csv', [dict(group=group, **values) for group, values in totals.items()])
    text = ['双时间尺度耦合优化：在线数值仿真验证', '',
            '范围：固定机械臂，原生 REMANI 在线规划，MPPI 唯一底盘执行者；非 Gazebo / 实车。',
            'native=当前串联基准；engineering=附加投影资格/软代价；execution/handover=单项；coupled=两项。',
            '所有组禁用旧 rolling 子目标与 MPPI 前缀初始化；不得与旧 A-D 实验混用。', '',
            '工程组边界：原生 3D、自碰撞、轮速和长时动态保守层全部保留，MPPI 原 V12 评分不改；',
            '未修复末端主动停车，也不宣称两层全部动态可行性等价。动态优化补项使用请求时刻的名义时间；',
            '发布前附加资格按 candidate.start_time 和最新输入复核，实际执行仍逐周期检查完整外形与制动。', '',
            '有效性与结果分开：碰撞、导航超时、无新参考交付均保留；接口/进程异常单列。',
            '消费参考不等于已恢复前进；另报消费后首个 5 cm 实测位移（不是到目标进展）。',
            '开启权重不等于代价生效；非零覆盖率按真实优化调用去重统计。',
            '分项 initial_final 非零率只观察首末回调：不能把首末为 0 解读为中间从未生效。',
            'any_callback 来自完整活跃回调计数；单项开启时可据此推断该项生效，双项同时开启时不能区分中间是哪一项。',
            '前处理失败（无搜索/无优化）与精确目标警告另列；不能将所有未进入优化的失败武断称为目标失败或优化超时。',
            '时延仅在有交付样本上计算分位数，同时单列未交付数量，缺失不记为 0。', '',]
    for group, result in totals.items():
        text.append('%s：试次 %d，有效 %d，到达 %d，碰撞 %d，导航超时 %d；已交付 %d，未交付 %d，未触发 %d。' % (
            TITLES[group], result['trials'], result['valid_trials'], result['goals'], result['collisions'],
            result['navigation_timeouts'], result['delivered_trials'], result['no_delivery_trials'], result['untriggered_trials']))
        percentage = lambda value: 'N/A' if value is None else '%.1f%%' % (100. * value)
        text.append('  有效快照 %d/%d（%s）；非零耦合求解 %d/%d（%s）；活跃目标函数回调 %d/%d（%s）。' % (
            result['valid_snapshot_scopes'], result['scope_events'], percentage(result['valid_snapshot_scope_rate']),
            result['any_nonzero_optimizer_calls'], result['refresh_optimizer_calls'], percentage(result['any_nonzero_coverage']),
            result['active_objective_callback_count'], result['objective_callback_count'], percentage(result['active_objective_callback_rate'])))
        text.append('  请求总数 %d；重规划请求 %d，其中进入优化 %d；无搜索/无优化前处理失败 %d；精确目标警告请求 %d。' % (
            result['request_total'], result['refresh_request_total'], result['refresh_optimizer_entered_requests'],
            result['preprocessing_failure_no_search_or_optimizer'], result['exact_target_warning_requests']))
    text += ['', '解释边界：少量种子仅是机制验证，不能据此宣称统计显著或实车安全。',
             '若耦合非零覆盖率不足，应先判定机制未被充分激发，不能把结果相近解释成方法无效。',
             '分组 CPU 项为逐次 CPU p95 的中位数，不冒充全组采样的 p95。',
             '计算资源数据为观察结果；CPU 百分比或未归因时间不能单独证明因果竞争。']
    (out / '结果说明.txt').write_text('\n'.join(text) + '\n')
    data = dict(scope=text[2], provisional=allow_incomplete, plans=plans, trials=rows, groups=totals,
                all_integrity_valid=all(r['valid_experiment'] for r in rows),
                reporter_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (out / 'summary.json').write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    if plots:
        plot(rows, totals, out)
    print('REPORTED', len(rows), 'online objective-coupling trials', out, flush=True)
    return data


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch', type=Path, action='append', required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--allow-incomplete', action='store_true')
    parser.add_argument('--no-plots', action='store_true')
    args = parser.parse_args()
    report(args.batch, args.out, args.allow_incomplete, not args.no_plots)
