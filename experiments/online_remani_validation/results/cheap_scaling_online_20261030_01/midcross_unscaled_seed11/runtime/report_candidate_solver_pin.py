#!/usr/bin/env python3
"""Report one fresh online pair testing per-solve dynamic-query snapshots."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from render_coupled_objective import SourceReader
from render_dual_timescale import FONT
from render_native_collision_3d import Trial
from report_long_goal_crossing import panel, settings
from report_sphere_scenarios import finite, shown, stats, summarize as summarize_trial
from run_candidate_solver_pin import GROUPS, check_pair_inputs

LABELS = {'pin0': '求解时动态地图持续更新', 'pin1': '单次求解固定动态地图'}


def content(key):
    if not isinstance(key, dict):
        raise ValueError('Runtime overlay key must be a recorded object')
    return {k: v for k, v in key.items() if k != 'frozen'}


def pin_observations(rows, enabled):
    # This SIL uses the ordinary local-owner refresh branch; every completed
    # 0.6-second call must prove its requested and actually acquired pin.
    calls = [r for r in rows if r.get('event') == 'end' and r.get('stage') == 'optimizer'
             and finite(r.get('budget_s')) and abs(r['budget_s']-.6) < 1e-9]
    proofs = []
    for row in calls:
        begin, end = row.get('solver_overlay_entry_key'), row.get('solver_overlay_return_key')
        fields = all(key in row for key in ('solver_overlay_freeze_requested', 'solver_overlay_pin_owned',
                     'overlay_evaluate_complete_count', 'overlay_evaluate_changed_count'))
        keys = isinstance(begin, dict) and isinstance(end, dict)
        expected = fields and row['solver_overlay_freeze_requested'] == int(enabled) \
            and row['solver_overlay_pin_owned'] == int(enabled)
        stable = keys and content(begin) == content(end)
        pinned = (keys and begin.get('frozen') is True and end.get('frozen') is True
                  and stable and row.get('overlay_evaluate_changed_count') == 0)
        passed = bool(expected and keys and (pinned if enabled else
                     begin.get('frozen') is False and end.get('frozen') is False))
        proofs.append(dict(optimizer_id=row['id'], passed=passed, requested=row.get('solver_overlay_freeze_requested'),
            acquired=row.get('solver_overlay_pin_owned'), entry=begin, returned=end,
            completed_evaluations=row.get('overlay_evaluate_complete_count'),
            evaluations_with_changed_overlay=row.get('overlay_evaluate_changed_count'),
            content_stable=stable, lbfgs_wall_ms=row.get('lbfgs_wall_ms'),
            native_validation_ms=row.get('native_validation_ms'),
            optimizer_success=row.get('success'), force_stop=row.get('force_stop')))
    return dict(eligible_calls=len(calls), runtime_proof_pass=bool(proofs) and all(p['passed'] for p in proofs),
        evaluations=sum(p['completed_evaluations'] for p in proofs if finite(p['completed_evaluations'])),
        evaluations_with_changed_overlay=sum(p['evaluations_with_changed_overlay'] for p in proofs
                                            if finite(p['evaluations_with_changed_overlay'])),
        calls_with_changed_entry_return=sum(not p['content_stable'] for p in proofs), calls=proofs,
        scope='ordinary completed 0.6s refresh solver calls; return key observed before function-scope pin release')


def summarize(source, group):
    result = summarize_trial(source)
    reader = SourceReader(source)
    trace = reader.jsonl('solver_trace.jsonl')
    origin = reader.json('analysis/summary.json', {}, required=True).get('clock_origin_ros_s', 1000.)
    trace = [r for r in trace if not finite(r.get('sim')) or r['sim']-origin <= result['end_t']]
    proof = pin_observations(trace, group == 'pin1')
    refresh = [r for r in result['remani']['request_rows'] if r.get('request_id', 0) > 1
               and finite(r.get('end_sim_s')) and r['end_sim_s'] <= result['end_t']]
    by_id = {r['id']: r for r in trace if r.get('event') == 'end' and r.get('stage') == 'optimizer'}
    for request in refresh:
        for call in request.get('optimizer_calls', []):
            measured = by_id.get(call.get('resource_optimizer_id'), {})
            for key in ('lbfgs_wall_ms', 'native_validation_ms', 'force_stop'):
                if key in measured:
                    call[key] = measured[key]
    result.update(group=group, pin_runtime=proof, refresh_requests=refresh,
        refresh_failure_reasons=dict(Counter(c.get('reason', 'unrecorded') for r in refresh
             for c in r.get('optimizer_calls', []) if c.get('success') is False)),
        refresh_lbfgs_wall_ms=stats(c.get('lbfgs_wall_ms') for r in refresh for c in r.get('optimizer_calls', [])),
        refresh_native_validation_ms=stats(c.get('native_validation_ms') for r in refresh for c in r.get('optimizer_calls', [])),
        refresh_live_acceptance_ms=stats(r.get('stage_wall_ms', {}).get('live_acceptance') for r in refresh))
    reader.verify()
    result['input_sha256'].update(reader.hashes)
    return result


def render_pair(paths, out):
    trials = [Trial(path, 'native3d') for path in paths]
    for trial in trials:
        settings(trial.manifest)
        trial.native_validity = []
    if not np.array_equal(trials[0].body3d, trials[1].body3d):
        raise ValueError('Pair geometry differs')
    duration = max(t.end-t.anchor for t in trials)
    ages = np.linspace(0., duration, min(110, max(2, math.ceil(duration/.4)+1)))
    frame_ms = max(20, 10*round(1000.*float(ages[1]-ages[0])/2./10.))
    font, small = ImageFont.truetype(FONT, 18), ImageFont.truetype(FONT, 14)
    frames = []
    for age in ages:
        image = Image.new('RGB', (1160, 490), 'white')
        draw = ImageDraw.Draw(image)
        draw.text((16, 8), '中段横穿：求解期间固定动态地图对照　%.1f秒（起步对齐，2倍速）' % age,
                  font=font, fill='#333333')
        draw.text((16, 35), '橙：REMANI参考　蓝：实走路线/机器人投影　灰：MPPI搜索子集　绿：获准展开　紫：CV',
                  font=small, fill='#555555')
        for i, trial in enumerate(trials):
            t = min(trial.anchor+age, trial.end)
            draw.text((i*580+16, 66), LABELS[GROUPS[i]], font=font, fill='#333333')
            row = panel(image, trial, t, (i*580+16, 100, (i+1)*580-16, 400))
            draw = ImageDraw.Draw(image)
            draw.text((i*580+16, 411), '%s　v=%+.2f m/s' % (trial.status_at(t), row['v']),
                      font=small, fill='#444444')
            if t >= trial.end:
                draw.text((i*580+16, 435), '本组结束：'+str(trial.result.get('reason')), font=small, fill='#666666')
        draw.text((16, 468), '真实在线数值ROS闭环记录，非实车/Gazebo。蓝色为原生球模型投影；搜索轨迹每秒记录一次。',
                  font=small, fill='#777777')
        frames.append(image.convert('P', palette=Image.ADAPTIVE, colors=96))
    frames[0].save(out, save_all=True, append_images=frames[1:],
                  duration=[frame_ms]*(len(frames)-1)+[max(700, frame_ms)], loop=0, disposal=2, optimize=True)
    frames[-1].convert('RGB').save(out.with_suffix('.png'))
    for trial in trials:
        trial.verify_unchanged()
    return dict(frames=len(frames), speedup=2., genuine_online_motion=True,
                reference_and_sphere_projection=True, no_paused_candidate_replay=True)


def write_text(rows, out, pair_checks):
    lines = ['动态查询地图固定：一次在线对照',
        '这次只改一件事：同一次REMANI优化和原生验收期间，固定当前动态查询快照。求解结束后释放。',
        'MPPI和机器人继续运动，感知数据继续接收；发布前最新环境检查仍执行。',
        '同一构建、67球、6米目标、中段横穿、seed11，两组各一次数值ROS闭环。不是Gazebo或实车。',
        '参数/代码/场景对齐：'+('通过' if pair_checks['passed'] else '未通过'), '']
    for row in rows:
        p, n = row['pin_runtime'], row['remani']
        lines += [LABELS[row['group']],
            '运行有效=%s；到达目标=%s；自身球模型采样接触=%s。' %
            (row['valid_experiment'], row['reached_goal'], row['model_sample_contact']),
            '受阻到MPPI收到新参考：%s秒；重规划成功/完成请求=%s/%s。' %
            (shown(n['blocked_to_first_consumed_refresh_s'], 3), n['refresh_accepted'], n['refresh_completed']),
            '运行时证据核验=%s；普通0.6秒求解%s次。完整评估%s次，其中%s次评估期间动态查询快照改变。' %
            (p['runtime_proof_pass'], p['eligible_calls'], p['evaluations'], p['evaluations_with_changed_overlay']),
            '求解首尾快照改变%s次；每次实际取得固定权限和首尾快照记录见summary.json。' %
            p['calls_with_changed_entry_return'],
            'MPPI求解加验收P95=%s毫秒；最长停车%s秒。' %
            (shown(row['mppi']['callback_ms']['p95']), shown(row['measured_stops']['longest_s'], 2)),
            '失败优化器原因：'+json.dumps(row['refresh_failure_reasons'], ensure_ascii=False)]
        if n['waiting_for_refresh_at_end']:
            lines.append('结束时仍在等新参考，已观察等待至少%s秒；不能记成零耗时。' %
                         shown(n['observed_refresh_wait_lower_bound_s'], 3))
        for request in row['refresh_requests']:
            stages = request.get('stage_wall_ms', {})
            calls = request.get('optimizer_calls', [])
            lines.append('  请求%s：%s；搜索/初始化阶段%s毫秒，纯求解[%s]，原生验收[%s]，最新验收%s，整次%s毫秒。' %
                (request['request_id'], request.get('outcome_class'), shown(stages.get('search')),
                 ','.join(shown(c.get('lbfgs_wall_ms')) for c in calls) or '未进入',
                 ','.join(shown(c.get('native_validation_ms')) for c in calls) or '未进入',
                 shown(stages.get('live_acceptance')), shown(request.get('request_wall_ms'))))
        lines.append('')
    a, b = rows
    lines += ['怎样解释',
        '运行时计数用于确认开关实际生效；不能只凭launch参数说已经固定了动态地图。',
        '两次闭环后续状态和规划输入会分化，单次结果只能说明该场景现象，不能给出总体成功率或严格的同输入因果改善比例。',
        '固定地图针对的是一次求解中题目持续变化的问题；轮约束残差、初始化、搜索和最新环境验收仍可能导致失败。',
        '搜索、优化器、请求的计时有包含关系，不把各列直接相加；缺失字段记为无记录。',
        'midcross.gif回放两次实际在线运动，和候选诊断GIF展示的优化步骤不同。', '']
    if all(row['pin_runtime']['runtime_proof_pass'] for row in rows):
        ta, tb = (row['remani']['blocked_to_first_consumed_refresh_s'] for row in rows)
        if finite(ta) and finite(tb):
            lines.insert(1, '本次受阻到收到新参考：持续更新%s秒，单次固定%s秒。' % (shown(ta, 3), shown(tb, 3)))
    (out/'实验结论.txt').write_text('\n'.join(lines)+'\n')


def report(batch, out):
    batch, out = Path(batch).resolve(), Path(out).resolve()
    plan = json.loads((batch/'plan.json').read_text())
    if json.loads((batch/'complete.json').read_text()).get('complete') is not True:
        raise ValueError('Finish both prespecified runs before reporting')
    jobs = plan['trials']
    if len(jobs) != 2 or [j['group'] for j in jobs] != list(GROUPS):
        raise ValueError('Expected the prescribed pin0/pin1 pair')
    paths = [batch/j['directory'] for j in jobs]
    pair_checks = check_pair_inputs(*paths)
    rows = [summarize(path, job['group']) for path, job in zip(paths, jobs)]
    out.mkdir(parents=True, exist_ok=True)
    gif = render_pair(paths, out/'midcross.gif')
    write_text(rows, out, pair_checks)
    result = dict(schema='candidate_solver_pin_report_v1', plan=plan, trials=rows,
        pair_checks=pair_checks, gif=gif,
        runtime_pin_proof_pass=all(r['pin_runtime']['runtime_proof_pass'] for r in rows),
        scope='fresh unpaused numerical ROS SIL pair; one seed; no statistical success-rate claim')
    (out/'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    here = Path(__file__).resolve().parent
    parser.add_argument('--batch', type=Path, default=here/'results/candidate_solver_pin_20261030_01')
    parser.add_argument('--out', type=Path, default=here/'results/candidate_solver_pin_report_20261030_01')
    args = parser.parse_args()
    report(args.batch, args.out)
