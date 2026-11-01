#!/usr/bin/env python3
"""Post-run report for the 160 versus 67 sphere online SIL comparison.

Reads only recorded trials. Failed solver calls remain in timing statistics;
unfinished calls and invalid runs are explicit, never silently treated as zero.
"""
import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from render_dual_timescale import FONT
from render_native_collision_3d import Trial
from report_execution_timing import measured_stops
from report_long_goal_crossing import (panel, sampled_gap, scene_actors, settings,
                                       terminal_settling_start)
from report_sim_crop_bypass import precontact_pairs


LABELS = {'legacy': '旧模型160球', 'b47': '候选B共67球'}
SCENES = {'empty': '空旷', 'static': '静态障碍', 'midcross': '中段横穿',
          'headon': '迎面障碍', 'multi': '多个动态障碍'}


def finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def stats(values):
    values = [float(v) for v in values if finite(v)]
    if not values:
        return dict(n=0, p50=None, p95=None, maximum=None, mean=None)
    return dict(n=len(values), p50=float(np.percentile(values, 50)),
                p95=float(np.percentile(values, 95)), maximum=max(values),
                mean=float(np.mean(values)))


def shown(value, decimals=1):
    return ('%.*f' % (decimals, value)) if finite(value) else '无记录'


def decrease(old, new):
    return 100.*(old-new)/old if finite(old) and finite(new) and old > 0 else None


def request_metrics(summary, end):
    """Keep every started request; only finish times determine completion."""
    started = [r for r in summary.get('requests', [])
               if finite(r.get('start_sim_s')) and r['start_sim_s'] < end]
    # The existing analyzer keeps unclosed request_begin events separately,
    # not in summary.requests. Their elapsed cost is unknown, never zero.
    known_ids = {r.get('request_id') for r in started}
    for unmatched in summary.get('unmatched', {}).get('requests', []):
        event = unmatched.get('event', {})
        if unmatched.get('reason') != 'unclosed_begin' or event.get('request_id') in known_ids:
            continue
        stamp = event.get('sim')
        if finite(stamp):
            stamp -= float(summary.get('clock_origin_ros_s', 1000.))
            if stamp < end:
                started.append(dict(request_id=event.get('request_id'), start_sim_s=stamp,
                                    start_mono=event.get('mono'), outcome_class='unfinished',
                                    source='unmatched request_begin'))
                known_ids.add(event.get('request_id'))
    started.sort(key=lambda row: row['start_sim_s'])
    completed = [r for r in started if finite(r.get('end_sim_s')) and r['end_sim_s'] <= end]
    pending = [r for r in started if r not in completed]
    optimizer = [call for r in completed for call in r.get('optimizer_calls', [])]
    refresh = [r for r in completed if r.get('request_id', 0) > 1]
    latency = summary.get('latency', {})
    blocked = latency.get('first_native_blocked_sim_s')
    consumed = latency.get('first_refresh_consume_sim_s')
    valid_wait = (finite(blocked) and finite(consumed) and blocked <= consumed <= end)
    return dict(available='requests' in summary, started=len(started), completed=len(completed), pending=len(pending),
        accepted=sum(r.get('success') is True for r in completed),
        failed=sum(r.get('success') is False for r in completed),
        unknown_outcome=sum(not isinstance(r.get('success'), bool) for r in completed),
        refresh_completed=len(refresh),
        refresh_accepted=sum(r.get('success') is True for r in refresh),
        completed_request_wall_ms=stats(r.get('request_wall_ms') for r in completed),
        completed_refresh_wall_ms=stats(r.get('request_wall_ms') for r in refresh),
        frontend_wall_ms=stats(r.get('stage_wall_ms', {}).get('search') for r in completed),
        backend_wall_ms=stats(c.get('wall_ms') for c in optimizer),
        backend_thread_cpu_ms=stats(c.get('thread_cpu_ms') for c in optimizer),
        backend_runqueue_wait_ms=stats(c.get('runqueue_wait_ms') for c in optimizer),
        backend_outcomes=dict(Counter(c.get('reason', 'unknown') for c in optimizer)),
        outcomes=dict(Counter(r.get('outcome_class', 'unknown') for r in completed)),
        blocked_to_first_consumed_refresh_s=consumed-blocked if valid_wait else None,
        first_native_blocked_t=blocked,
        first_refresh_consume_t=consumed if finite(consumed) and consumed <= end else None,
        waiting_for_refresh_at_end=bool(finite(blocked) and blocked < end and not valid_wait),
        observed_refresh_wait_lower_bound_s=end-blocked if finite(blocked) and blocked < end and not valid_wait else None,
        request_rows=started, incomplete_request_ids=[r.get('request_id') for r in pending],
        limitations='search includes A* / initial polynomial work, not pure graph-search CPU; request/frontend/backend overlap and must not be summed; all completed failures included')


def cycle_metrics(pairs, deadline_s=.2):
    invoked = [(d, c, t) for d, c, t in pairs
               if c.get('online_step_invoked') is True and d is not None]
    rows = [d for d, _, _ in invoked]
    # Publication intervals include failed/no-feasible cycles and post-solve
    # bookkeeping. They are not the callback duration or simulated time step.
    publications = [float(c['command_publish_after_monotonic']) for _, c, _ in pairs
                    if finite(c.get('command_publish_after_monotonic'))]
    intervals = [1000.*(b-a) for a, b in zip(publications, publications[1:]) if b >= a]
    deadline = [c for _, c, _ in invoked if c.get('reason') == 'deadline_miss']
    safe_late = [d for d, c, _ in invoked if c.get('reason') == 'deadline_miss'
                and c.get('validation_code') == 0 and d.get('validated_proposal') is not None
                and sum(abs(v) for v in d['validated_proposal'][:2]) > 1e-9]
    validation_cycles = [c for _, c, _ in invoked
                         if c.get('validation_invoked') is True or c.get('validation_attempts')]
    attempt_rows = [a for c in validation_cycles for a in c.get('validation_attempts', [])]
    def attempt_sum(cycle, key):
        attempts = cycle.get('validation_attempts', [])
        values = [a.get(key) for a in attempts]
        # Missing timer fields are unknown, not zero time. Report incomplete
        # cycles separately instead of summing only the fast recorded attempts.
        return sum(values) if values and all(finite(v) and v >= 0 for v in values) else None
    validation = dict(
        cycles=len(validation_cycles), attempts=len(attempt_rows),
        cycle_wall_ms=stats(attempt_sum(c, 'wall_ms') for c in validation_cycles),
        cycle_main_thread_cpu_ms=stats(attempt_sum(c, 'thread_cpu_ms') for c in validation_cycles),
        per_attempt_wall_ms=stats(a.get('wall_ms') for a in attempt_rows),
        per_attempt_main_thread_cpu_ms=stats(a.get('thread_cpu_ms') for a in attempt_rows),
        missing_or_incomplete_timer_cycles=sum(attempt_sum(c, 'wall_ms') is None for c in validation_cycles),
        retry_cycles=sum(len(c.get('validation_attempts', [])) > 1 for c in validation_cycles),
        extra_retry_attempts=sum(max(0, len(c.get('validation_attempts', []))-1) for c in validation_cycles),
        final_code_counts=dict(Counter(str(c.get('validation_code')) for c in validation_cycles)),
        scope='sum of actual execution-validation call timers per solver cycle, including all retries; not callback minus kernel; thread CPU excludes other threads')
    phase = {}
    for name in ('freepath', 'monitoring', 'intervention', 'unknown'):
        selected = []
        for row in rows:
            decision = (row.get('local_phase') or {}).get('decision', [])
            current = ('intervention' if decision[6] > .5 else
                       'monitoring' if any(len(o) >= 6 and o[5] > .5 for o in row.get('observed_obstacles', [])) else
                       'freepath') if len(decision) >= 7 else 'unknown'
            if current == name:
                selected.append(row)
        phase[name] = dict(kernel_ms=stats(r.get('kernel_ms') for r in selected),
                           callback_ms=stats(r.get('callback_ms') for r in selected))
    return dict(published_cycles=len(pairs), invoked_solver_cycles=len(invoked),
        kernel_ms=stats(r.get('kernel_ms') for r in rows),
        callback_ms=stats(r.get('callback_ms') for r in rows),
        validation=validation, solver_phase=phase, publication_interval_ms=stats(intervals),
        publication_intervals_over_lease=sum(v > 250. for v in intervals),
        callback_over_deadline=sum(finite(r.get('callback_ms')) and r['callback_ms'] > deadline_s*1000. for r in rows),
        deadline_labels=len(deadline), validated_nonzero_discarded_late=len(safe_late),
        reason_counts=dict(Counter(c.get('reason', 'unknown') for _, c, _ in pairs)),
        missing_diagnostic_invocations=sum(c.get('online_step_invoked') is True and d is None for d, c, _ in pairs),
        kernel_failures=sum(c.get('online_step_invoked') is True and c.get('online_step_succeeded') is False for _, c, _ in pairs),
        scope='invoked solver cycles, including failures; update intervals use all consecutive actual command publications after goal; monotonic wall-clock milliseconds')


def cpu_metrics(samples, start_mono, end_mono, status):
    pids = status.get('monitored_node_pids', {})
    samples = [r for r in samples if finite(r.get('mono')) and start_mono <= r['mono'] <= end_mono]
    result = {}
    for node in ('remani_planner_node', 'base_mppi'):
        values = [p['cpu_pct'] for r in samples for p in r.get('processes', [])
                  if (p.get('pid') == pids.get('/'+node) or p.get('name') == node) and finite(p.get('cpu_pct'))]
        result[node] = stats(values)
    result['system_busy_pct'] = stats(c['busy_pct'] for r in samples for c in r.get('cores', [])
                                     if c.get('core') == 'cpu' and finite(c.get('busy_pct')))
    result['scope'] = 'process total CPU percent; 100% is one logical core; sampled mean/P95, filtered by monotonic command-publication interval; not optimizer-only CPU'
    return result


def loop_metrics(record, ids):
    columns = record.get('columns', [])
    if not {'cycle', 'stage', 'mono', 'thread_cpu_s'}.issubset(columns):
        return dict(available=False)
    rows = [dict(zip(columns, r)) for r in record.get('rows', [])]
    per_cycle = defaultdict(dict)
    for row in rows:
        if row['cycle'] in ids:
            per_cycle[row['cycle']][row['stage']] = row
    wall, cpu = [], []
    for stages in per_cycle.values():
        if 'cycle_start' in stages and 'sleep_enter' in stages:
            a, b = stages['cycle_start'], stages['sleep_enter']
            wall.append(1000.*(b['mono']-a['mono']))
            cpu.append(1000.*(b['thread_cpu_s']-a['thread_cpu_s']))
    return dict(available=bool(wall), active_loop_wall_ms=stats(wall),
                active_loop_main_thread_cpu_ms=stats(cpu),
                scope='cycle_start to sleep_enter, including publication/logging; excludes deliberate sleep and other Python/ROS threads')


def summarize(source):
    trial = Trial(source, 'native3d')
    if trial.status.get('result_written') is not True:
        raise ValueError('trial must finish before reporting')
    _, goal, bounds = settings(trial.manifest)
    cycles = trial.reader.jsonl('state_mppi_cycles.jsonl')
    pairs, alignment = precontact_pairs(trial.diagnostics, cycles, trial.end)
    sent = float(trial.result.get('goal_sent') or trial.anchor)
    pairs = [p for p in pairs if p[2] >= sent]
    deadline = float(trial.manifest.get('execution_timing_pair', {}).get('callback_deadline_s', .2))
    solver = cycle_metrics(pairs, deadline)
    publication_monotonic = [c['command_publish_after_monotonic'] for _, c, _ in pairs
                            if finite(c.get('command_publish_after_monotonic'))]
    cpu = cpu_metrics(trial.reader.jsonl('cpu_samples.jsonl'), min(publication_monotonic, default=math.inf),
                      max(publication_monotonic, default=-math.inf), trial.status)
    loop = loop_metrics(trial.reader.json('loop_stages.json', {}), {c['cycle_id'] for _, c, _ in pairs})
    remani = request_metrics(trial.reader.json('analysis/summary.json', {}), trial.end)
    physical = [sampled_gap(np.array([r['x'], r['y'], r['yaw']]), trial.body3d, trial.resolution,
                           scene_actors(trial.manifest, trial.result, float(r['t'])), bounds)
                for r in trial.state if r['t'] <= trial.end]
    monitor = trial.result.get('native3d_truth_monitor', {})
    minimum = min(physical + ([monitor['min_physical_sample_gap']] if finite(monitor.get('min_physical_sample_gap')) else []))
    limits = trial.reader.json('limits.json', {}, required=True)
    settling = (terminal_settling_start(trial.state, limits, goal, trial.end)
                if trial.result.get('reason') == 'goal' else None)
    stopped = measured_stops(trial.state, trial.anchor, settling if settling is not None else trial.end)
    valid = bool(trial.audit.get('valid_experiment') is True and
                 not trial.status.get('required_process_failed') and not trial.status.get('runner_error'))
    result = dict(trial=str(trial.source), scene=trial.manifest['scene'], seed=trial.parameters.get('seed'),
        total_spheres=len(trial.body3d), fixed_spheres=int(np.sum(trial.body3d[:, 6] == 0)),
        arm_spheres=int(np.sum(trial.body3d[:, 6] != 0)), valid_experiment=valid,
        audit_valid=trial.audit.get('valid_experiment'), required_process_failed=trial.status.get('required_process_failed'),
        runner_error=trial.status.get('runner_error'), termination=trial.result.get('reason'),
        reached_goal=trial.result.get('reason') == 'goal', observed_duration_s=trial.end-trial.anchor,
        goal_sent_t=sent, anchor_t=trial.anchor, end_t=trial.end,
        model_sample_contact=minimum < 0 or bool(monitor.get('sample_collision')),
        minimum_own_model_sample_gap_m=minimum, measured_stops=stopped,
        mppi=solver, loop=loop, cpu=cpu, remani=remani, alignment=alignment,
        consumed_references=len([r for r in trial.refs if r['t'] <= trial.end]),
        actual_reference_settings=trial.manifest.get('long_goal_crossing_experiment'),
        collision_scope='own recorded sphere model versus finite walls/cylinders; different sphere geometry means these gaps are not a common mesh-level accuracy test',
        scope='fixed-arm differential-drive full REMANI + MPPI online numerical ROS SIL; not Gazebo or hardware',
        statistical_success_rate_claim=False)
    trial.verify_unchanged()
    result['input_sha256'] = dict(trial.reader.hashes)
    return result


def trial_entries(plan):
    """Small plan contract; explicit trial paths prevent history-directory scans."""
    entries = plan.get('trials', [])
    if not entries:
        raise ValueError('plan.trials must explicitly list measured trials')
    seen = set()
    for entry in entries:
        if entry['group'] not in LABELS:
            raise ValueError('unknown sphere comparison group')
        key = entry['scene'], entry['group']
        if key in seen:
            raise ValueError('duplicate scene/group in single-seed comparison')
        seen.add(key)
        path = entry.get('directory', entry.get('path', entry.get('trial', entry.get('source'))))
        relative = Path(path)
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError('trial path must remain under batch directory')
        yield dict(entry, directory=relative.as_posix())


def comparisons(rows):
    paired = defaultdict(dict)
    for row in rows:
        paired[row['scene_key']][row['group']] = row
    results = []
    for scene, groups in paired.items():
        if len(groups) != 2:
            continue
        old, new = groups['legacy'], groups['b47']
        comparisons = {}
        for name, path in (
                ('kernel_p50', ('mppi', 'kernel_ms', 'p50')),
                ('kernel_p95', ('mppi', 'kernel_ms', 'p95')),
                ('callback_p50', ('mppi', 'callback_ms', 'p50')),
                ('callback_p95', ('mppi', 'callback_ms', 'p95')),
                ('intervention_kernel_p95', ('mppi', 'solver_phase', 'intervention', 'kernel_ms', 'p95')),
                ('intervention_callback_p95', ('mppi', 'solver_phase', 'intervention', 'callback_ms', 'p95')),
                ('publication_interval_p95', ('mppi', 'publication_interval_ms', 'p95')),
                ('remani_all_request_p95', ('remani', 'completed_request_wall_ms', 'p95')),
                ('remani_frontend_p95', ('remani', 'frontend_wall_ms', 'p95')),
                ('remani_backend_p95', ('remani', 'backend_wall_ms', 'p95'))):
            before, after = old, new
            for key in path:
                before, after = before[key], after[key]
            comparisons[name] = dict(old=before, new=after, decrease_pct=decrease(before, after))
        results.append(dict(scene=scene, both_valid=old['valid_experiment'] and new['valid_experiment'],
                            metrics=comparisons))
    return results


def draw_chart(rows, out):
    scenes = list(dict.fromkeys(r['scene_key'] for r in rows))
    paired = {(r['scene_key'], r['group']): r for r in rows}
    image = Image.new('RGB', (1240, 880), 'white')
    draw = ImageDraw.Draw(image)
    title, small = ImageFont.truetype(FONT, 22), ImageFont.truetype(FONT, 16)
    draw.text((24, 12), '碰撞球简化：完整REMANI＋MPPI在线闭环耗时对照', font=title, fill='#333333')
    draw.text((24, 47), '灰：旧160球　蓝：新67球　数值为P95；失败请求也统计，异常组标 *；每组仅一次', font=small, fill='#555555')
    specs = [('MPPI内核 / ms', ('mppi', 'kernel_ms', 'p95')),
             ('MPPI求解＋验收 / ms', ('mppi', 'callback_ms', 'p95')),
             ('实际指令更新间隔 / ms', ('mppi', 'publication_interval_ms', 'p95')),
             ('REMANI全部完成请求 / ms', ('remani', 'completed_request_wall_ms', 'p95'))]
    for index, (name, path) in enumerate(specs):
        x, y = 24+(index % 2)*620, 100+(index // 2)*360
        draw.text((x, y), name, font=small, fill='#333333')
        values = []
        for row in rows:
            value = row
            for key in path:
                value = value[key]
            if finite(value):
                values.append(value)
        maximum = max(values+[1.])
        for j, scene in enumerate(scenes):
            yy = y+36+j*55
            draw.text((x, yy+6), SCENES.get(scene, scene), font=small, fill='#333333')
            for k, group in enumerate(LABELS):
                row = paired.get((scene, group))
                if row is None:
                    continue
                value = row
                for key in path:
                    value = value[key]
                width = 345.*value/maximum if finite(value) else 0.
                top = yy+k*21
                if width > 0:
                    draw.rectangle((x+115, top, x+115+width, top+15), fill='#999999' if k == 0 else '#2874a6')
                draw.text((x+122+width, top-4), shown(value)+( '' if row['valid_experiment'] else ' *'), font=small, fill='#333333')
    draw.text((24, 826), '数值ROS闭环，不是实车/Gazebo。不同闭环会走不同路线；不是同输入纯查询基准。', font=small, fill='#666666')
    draw.text((24, 852), '指令更新间隔含两轮之间的处理；不能与“内核”或“求解＋验收”混为一谈。', font=small, fill='#666666')
    image.save(out)


def render_pair(old, new, out, scene, max_frames=100):
    trials = [Trial(old, 'native3d'), Trial(new, 'native3d')]
    for trial in trials:
        settings(trial.manifest)
        # The recorded detector diagnostics remain the only CV display source.
        trial.native_validity = []
    duration = max(t.end-t.anchor for t in trials)
    ages = np.linspace(0., duration, min(max_frames, max(2, math.ceil(duration/.4)+1)))
    frame_ms = max(20, 10*round(1000.*float(ages[1]-ages[0])/2./10.))
    font, small = ImageFont.truetype(FONT, 18), ImageFont.truetype(FONT, 14)
    frames = []
    for age in ages:
        image = Image.new('RGB', (1160, 482), 'white')
        draw = ImageDraw.Draw(image)
        draw.text((16, 8), '%s　%.1f s（各自实测起步对齐）' % (SCENES.get(scene, scene), age), font=font, fill='#333333')
        draw.text((16, 34), '橙：REMANI参考　蓝：实际机器人投影　灰：记录搜索子集　绿：获准展开　紫虚线：CV', font=small, fill='#555555')
        for index, trial in enumerate(trials):
            t = min(trial.anchor+age, trial.end)
            draw.text((index*580+16, 64), list(LABELS.values())[index], font=font, fill='#333333')
            row = panel(image, trial, t, (index*580+16, 98, (index+1)*580-16, 402))
            draw = ImageDraw.Draw(image)
            draw.text((index*580+16, 413), '%s　v=%+.2f m/s' % (trial.status_at(t), row['v']), font=small, fill='#555555')
            if trial.status.get('required_process_failed'):
                draw.text((index*580+16, 435), '进程异常退出：本组无效，画面停在实际终点', font=small, fill='#a44b33')
        draw.text((16, 459), '固定臂ROS数值闭环，非实车/Gazebo；完整球投影，不用点机器人；只回放实录。', font=small, fill='#777777')
        frames.append(image.convert('P', palette=Image.ADAPTIVE, colors=96))
    frames[0].save(out, save_all=True, append_images=frames[1:],
                   duration=[frame_ms]*(len(frames)-1)+[max(700, frame_ms)], loop=0, disposal=2, optimize=True)
    for trial in trials:
        trial.verify_unchanged()


def narrative(result):
    lines = ['本轮做了什么',
        '重新跑完整REMANI＋MPPI在线数值闭环；旧模型160球，对比候选B的67球。不是Gazebo或实车。',
        '每个场景每组只跑一次。它能观察这次计算时间和运动表现，不能当成总体成功率。',
        '球的位置、大小和数量一起改变，不是只删球；闭环运动不同，所以不是同输入的纯计算基准。', '', '计算时间对比']
    for comparison in result['comparisons']:
        lines.append(SCENES.get(comparison['scene'], comparison['scene'])+('：两组有效。' if comparison['both_valid'] else '：含无效组，只列事实，不作正常性能结论。'))
        for name, label in [('kernel_p50', 'MPPI内核P50'), ('kernel_p95', 'MPPI内核P95'),
                            ('callback_p50', '求解＋验收P50'), ('callback_p95', '求解＋验收P95'),
                            ('intervention_kernel_p95', '进入避障阶段内核P95'),
                            ('intervention_callback_p95', '进入避障阶段求解＋验收P95'),
                            ('publication_interval_p95', '实际指令更新间隔P95'),
                            ('remani_all_request_p95', 'REMANI全部完成请求P95')]:
            item = comparison['metrics'][name]
            change = ('降低%s%%' % shown(item['decrease_pct']) if finite(item['decrease_pct']) and item['decrease_pct'] >= 0 else
                      '增加%s%%' % shown(-item['decrease_pct']) if finite(item['decrease_pct']) else '无法计算变化')
            lines.append('  %s：%s → %s毫秒，%s。' % (label, shown(item['old']), shown(item['new']), change))
    lines += ['', '各组实际情况']
    for row in result['trials']:
        m, r = row['mppi'], row['remani']
        lines += ['%s / %s：%s；结束=%s；观察%s秒。' %
            (SCENES.get(row['scene_key'], row['scene_key']), LABELS[row['group']],
             '有效' if row['valid_experiment'] else '无效/进程异常', row['termination'], shown(row['observed_duration_s'])),
            '  到达目标=%s；自身球模型采样接触=%s；最长实测停车%s秒。' %
            ('是' if row['reached_goal'] else '否', '有' if row['model_sample_contact'] else '无', shown(row['measured_stops']['longest_s'])),
            '  调用MPPI %d次；超期标签%d次；已验收非零动作因超期清零%d次；实际更新间隔超过250毫秒%d次。' %
            (m['invoked_solver_cycles'], m['deadline_labels'], m['validated_nonzero_discarded_late'], m['publication_intervals_over_lease']),
            '  单轮执行验收累计耗时P50/P95：%s/%s毫秒；共验收%d次，其中%d轮发生重试，额外重试%d次。' %
            (shown(m['validation']['cycle_wall_ms']['p50']), shown(m['validation']['cycle_wall_ms']['p95']),
             m['validation']['attempts'], m['validation']['retry_cycles'], m['validation']['extra_retry_attempts']),
            '  REMANI启动%d次，完成%d次：成功%d、失败%d；结束时还有%d次未完成。重规划完成%d次、成功%d次。' %
            (r['started'], r['completed'], r['accepted'], r['failed'], r['pending'], r['refresh_completed'], r['refresh_accepted']),
            '  发现参考受阻到实际收到第一条新参考：%s秒。' % shown(r['blocked_to_first_consumed_refresh_s']),
            '  REMANI / MPPI进程CPU采样P95：%s%% / %s%%（100%%代表占满一核）。' %
            (shown(row['cpu']['remani_planner_node']['p95']), shown(row['cpu']['base_mppi']['p95']))]
        if r['waiting_for_refresh_at_end']:
            lines.append('  截至结束仍未收到新参考，已等%s秒；不是“零延迟”。' % shown(r['observed_refresh_wait_lower_bound_s']))
        for request in r['request_rows']:
            stage = request.get('stage_wall_ms', {})
            backends = request.get('optimizer_calls', [])
            lines.append('  请求%s：%s，前端%s毫秒，后端[%s]毫秒，整次%s毫秒。' %
                (request.get('request_id'), request.get('outcome_class', '未完成/未知'), shown(stage.get('search')),
                 ', '.join(shown(c.get('wall_ms')) for c in backends) or '未进入/无记录', shown(request.get('request_wall_ms'))))
    lines += ['', '怎样看这些数字',
        'P50是中位数，P95表示约95%的记录不超过这个值。',
        'MPPI内核只计核心求解；求解＋验收还包含候选检查；更新间隔还会受日志、发布、调度等影响。',
        '执行验收耗时由每次验收调用直接计时，包含候选重试；不是拿“回调减内核”猜出来的。',
        '空旷跟随、监视障碍、真正介入避障的工作量不同；报告同时列介入避障阶段耗时，缺少该阶段时标“无记录”。',
        'REMANI失败请求和成功请求都计时；前端字段含搜索和初始曲线处理，不是纯A*时间。各段有包含关系，不可直接相加。',
        '没有拿成功那一次代替整段等待。整次规划耗时与等待新参考的时间分别给出。',
        '两套模型外形不同，采样间隙不能当成共同URDF真值安全结论；本轮主要比较计算和在线交付。',
        'comparison.png是简明耗时图；静态和迎面/多障碍GIF仅回放实录，不重跑控制器。', '']
    return '\n'.join(lines)


def report_batch(batch, out, gif_scenes=('static', 'headon')):
    batch, out = Path(batch).resolve(), Path(out).resolve()
    if not (batch/'complete.json').is_file():
        raise ValueError('finish all timed trials before reporting')
    if out == batch or batch in out.parents:
        raise ValueError('report output must be outside measured batch')
    plan = json.loads((batch/'plan.json').read_text())
    entries = list(trial_entries(plan))
    rows, paths = [], {}
    for entry in entries:
        source = batch/entry['directory']
        row = summarize(source)
        row.update(group=entry['group'], scene_key=entry['scene'])
        expected = 160 if entry['group'] == 'legacy' else 67
        if row['total_spheres'] != expected:
            raise ValueError('recorded sphere count differs from group')
        rows.append(row)
        paths[entry['scene'], entry['group']] = source
    out.mkdir(parents=True, exist_ok=True)
    result = dict(plan=plan, trials=rows, comparisons=comparisons(rows),
        scope='new complete REMANI + MPPI online ROS numerical SIL, fixed arm; not Gazebo or hardware',
        statistical_success_rate_claim=False)
    draw_chart(rows, out/'comparison.png')
    for scene in gif_scenes:
        if (scene, 'legacy') in paths and (scene, 'b47') in paths:
            render_pair(paths[scene, 'legacy'], paths[scene, 'b47'], out/(scene+'.gif'), scene)
    (out/'report.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    (out/'实验总结.txt').write_text(narrative(result))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch', required=True, type=Path)
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--gif-scenes', nargs='*', default=['static', 'headon'])
    args = parser.parse_args()
    result = report_batch(args.batch, args.out, args.gif_scenes)
    print(json.dumps(dict(out=str(args.out), trials=len(result['trials']),
                         invalid=sum(not r['valid_experiment'] for r in result['trials'])), ensure_ascii=False))


if __name__ == '__main__':
    main()
