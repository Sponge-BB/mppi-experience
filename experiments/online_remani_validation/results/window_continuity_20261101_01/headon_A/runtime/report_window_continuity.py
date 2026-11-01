#!/usr/bin/env python3
"""Read the saved-input continuity check and two unpaused online SIL runs.

This script only reads completed records. Frozen-input optimizer progress and
online reference delivery are reported separately; neither means that the
two-timescale optimization problem has been completely solved.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from render_coupled_objective import sha
from render_dual_timescale import FONT
from report_delivery_feedback import load_trial
from report_long_goal_crossing import panel
from report_static_agility import show
from report_time_indexed_collision import TERMINATIONS


HERE = Path(__file__).resolve().parent
DEFAULT = HERE / 'results/window_continuity_20261101_01'
LABELS = ('A：原动态代价', 'B：固定预测窗积分')


def rows_at(path):
    if not path.is_file():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def first(value, *keys):
    return next((value[k] for k in keys if k in value), None)


def read_replay(path):
    records = rows_at(path)
    exact = [r for r in records if r.get('kind') == 'original_exact_check']
    scale = [r for r in records if r.get('kind') == 'common_scale']
    continuations = [r for r in records if r.get('kind') == 'continuation']
    branches = []
    for r in continuations:
        continuous = r.get('continuous')
        if continuous is None:
            continuous = r.get('variant') in ('B', 'fixed', 'continuous')
        gate = first(r, 'final_gate', 'verdict') or {}
        branches.append(dict(label=LABELS[bool(continuous)], continuous=bool(continuous),
            solver_exit=first(r, 'result', 'lbfgs_result'), force_stop=r.get('force_stop'),
            budget_s=first(r, 'budget_s', 'remaining_allowance_s'),
            solver_wall_s=r.get('solver_wall_s'), accepted_steps=r.get('accepted_steps'),
            native_pass=first(gate, 'native_pass', 'pass'),
            native_reason=first(gate, 'native_reason', 'reason'),
            native_rejection_time=first(gate, 'native_rejection_time', 'rejection_time'),
            initial_x_hash=r.get('initial_x_hash'), final_x_hash=r.get('final_x_hash'),
            initial_cost=r.get('initial_cost'), same_common_scale=r.get('same_common_scale'),
            published=r.get('published'), raw=r))
    check = exact[0] if len(exact) == 1 else None
    original_exact = (all(check.get(k) is True for k in ('cost_exact', 'x_exact', 'gradient_exact'))
                      if check else None)
    paired = len(branches) == 2 and {r['continuous'] for r in branches} == {False, True}
    same_x = (branches[0]['initial_x_hash'] == branches[1]['initial_x_hash'] and
              branches[0]['initial_x_hash'] is not None) if paired else None
    same_budget = (branches[0]['budget_s'] is not None and branches[1]['budget_s'] is not None and
                   abs(branches[0]['budget_s']-branches[1]['budget_s']) < 1e-12) if paired else None
    common_scale = all(r['same_common_scale'] is True for r in branches) if paired else None
    return dict(path=str(path.resolve()), sha256=sha(path) if path.is_file() else None,
        original_exact_check=check, original_exact=original_exact,
        common_scale=scale[0] if len(scale) == 1 else None,
        original_scale_available=False,
        comparison=dict(two_branches=paired, same_initial_x=same_x, equal_budget=same_budget,
                        same_common_scale=common_scale,
                        valid=original_exact is True and paired and same_x is True and
                              same_budget is True and common_scale is True),
        finite_difference=[r for r in records if r.get('kind') in
                           ('finite_difference', 'native_central_difference')],
        boundary_line=[r for r in records if r.get('kind') in
                       ('boundary_line', 'saved_line_search_boundary_sweep')],
        continuations=branches, raw_records=records,
        scope='Saved complete native input; original cost/gradient verification, then common-new-scale A/B; no publication or online performance inference.')


def online(source, label):
    row, trial = load_trial(source, label)
    timeline = trial.reader.jsonl('dual_timeline.jsonl')
    requests = [r for r in row['remani']['requests'] if r.get('request_id', 0) > 1]
    publications = [r for r in timeline if r.get('event') == 'reference_publish'
                    and r.get('request_id', 0) > 1 and r.get('sim', 0.) <= 1000.+trial.end+1e-6]
    refs = [r for r in trial.refs if r.get('revision', 0) > 1 and r['t'] <= trial.end]
    native = [r for r in requests if any(c.get('success') is True for c in r.get('optimizer_calls', []))]
    events = trial.reader.json('events.json', [])
    if isinstance(events, dict):
        events = events.get('events', events.get('rows', []))
    gaps = [r for r in events if isinstance(r, dict) and r.get('kind') == 'plant_loop_gap']
    pause = [r for r in timeline if r.get('event') in
             ('candidate_exit_audit_begin', 'candidate_exit_audit_end')]
    row['delivery_chain_counts'] = dict(refresh_requests=len(requests),
        requests_with_native_optimizer_pass=len(native),
        refresh_reference_publications=len(publications),
        refresh_references_consumed=len(refs),
        native_pass_request_ids=[r['request_id'] for r in native],
        publication_request_ids=[r['request_id'] for r in publications],
        consumed_revisions=[r.get('revision') for r in refs],
        definition='Native optimizer success precedes latest-CV publication; consumption is recorded independently. Intermediate checks are not counted as deliveries.')
    row['failure_reason_counts'] = dict(Counter(c.get('reason', 'unrecorded')
        for r in requests for c in r.get('optimizer_calls', []) if c.get('success') is not True))
    row['online_pause_audit'] = dict(explicit_diagnostic_pause_events=pause,
        plant_loop_gap_events=gaps,
        maximum_plant_loop_gap_s=max([float(r.get('dt', 0.)) for r in gaps] or [0.]),
        diagnostic_pause_detected=bool(pause),
        interpretation='No SIGSTOP audit is requested in these online runs. Logged loop gaps still describe actual scheduling interruptions; absence of this threshold event does not imply zero jitter.')
    row['source_sha256'] = trial.reader.hashes
    row['physical_sphere_count'] = len(trial.body3d)
    trial.verify_unchanged()
    return row, trial


def verdict(branch):
    if branch['native_pass'] is True:
        return '通过原生检查'
    if branch['native_pass'] is False:
        return '未通过：'+str(branch['native_reason'])
    return '检查未记录'


def replay_conclusion(replay):
    if not replay['comparison']['valid']:
        return '同输入A/B的一致性尚未全部确认，不据此宣称优化修复有效。'
    a = next(r for r in replay['continuations'] if not r['continuous'])
    b = next(r for r in replay['continuations'] if r['continuous'])
    changed = b['initial_x_hash'] != b['final_x_hash']
    if b['native_pass'] is True and a['native_pass'] is not True:
        return '该完整冻结输入上，改动版通过原生检查，原版未通过；这证明本输入的后端可行性改善，在线交付仍需另看。'
    if b['native_pass'] is not True and changed:
        return '改动版接受了后续更新，但最终仍未通过原生检查；连续性改善不能等同于候选已可交付。'
    if b['native_pass'] is not True:
        return '改动版最终仍未通过原生检查；当前修复不足以解决该输入的可用轨迹交付。'
    return '两分支均通过原生检查；这次输入不能证明只有改动版才能取得可行候选。'


def conclusions(replay, rows):
    notes = [replay_conclusion(replay)]
    if len(rows) != 2:
        return notes+['尚无完整的正常在线A/B结果，线上收益未验证。']
    a, b = rows
    da, db = [r['control_delivery']['first_refresh_to_new_reference_validated_command_wall_ms'] for r in rows]
    if any(r['online_pause_audit']['diagnostic_pause_detected'] for r in rows):
        notes.append('检测到在线诊断暂停，交付与避障性能对照受干扰。')
    elif db is None:
        notes.append('本轮改动版没有产生新参考的有效控制，在线交付问题仍未解决。')
    elif da is None:
        notes.append('本轮改动版交付了有效新控制，原版没有；只说明本次闭环结果，不等于总体稳定交付。')
    else:
        notes.append('首次重规划到有效新控制：A %.3f秒，B %.3f秒；差值%+.3f秒。' %
                     (da/1000., db/1000., (db-da)/1000.))
    if b['physical_sample_collision']:
        notes.append('改动版发生物理球采样接触，尚不能认为避障问题已解决。')
    notes.append('本轮处理慢层动态代价的数值连续性，并检查能否及时交给快层；尚未证明双时间尺度协同优化完成。')
    return notes


def narrative(data):
    replay, rows = data['replay'], data['rows']
    lines = ['动态预测窗口连续性：实验结果', '',
        '这轮只针对一个问题：优化器微调轨迹段时间时，原动态软代价在3秒预测边界突然加回或删掉，导致线搜索失败、候选不能交付。',
        '改动：动态软代价改为在固定CV有效时间窗内积分，保留原三维球、静态/自身/轮约束、常速和半速硬检查及最新CV发布检查。',
        '先用上一轮保存的完整输入复核和比较，再新跑迎面场景A/B。上轮1.22秒暂停补算扰乱闭环，不能作为在线性能基线。', '',
        '本轮结论']+data['conclusions']+['', '同输入验证']
    check = replay['original_exact_check']
    if check:
        lines.append('原始候选的变量、代价、梯度精确重建：%s/%s/%s。' % tuple(
            '一致' if check.get(k) is True else '不一致或未记录' for k in ('x_exact', 'cost_exact', 'gradient_exact')))
    else:
        lines.append('尚无原代价/梯度精确复核记录。')
    lines += ['原运行的坐标缩放未保存；本次A/B共同使用在同一起点重新估计的一套缩放。因此是同输入、同新缩放对照，不声称完整复现原求解过程。',
              '共同缩放估计放在两分支等额求解预算之外；求解耗时与最终原生复核耗时分别记录。']
    for branch in replay['continuations']:
        lines.append('%s：预算%s毫秒，求解%s毫秒，退出码%s，接受%d次更新；%s。' % (
            branch['label'], show(1000*branch['budget_s'], 3) if branch['budget_s'] is not None else '未记录',
            show(1000*branch['solver_wall_s'], 3) if branch['solver_wall_s'] is not None else '未记录',
            branch['solver_exit'], branch['accepted_steps'] or 0, verdict(branch)))
    lines.append('梯度差分记录%d条，预测边界扫线记录%d条；数值详情保留在summary.json。' %
                 (len(replay['finite_difference']), len(replay['boundary_line'])))
    for r in replay['finite_difference']:
        lines.append('%s真实MINCO目标函数的中心差分：检查%d个底盘路点/时间变量，最大绝对误差%s，最大相对误差%s。' % (
            LABELS[bool(r.get('continuous'))], len(r.get('samples', [])),
            '%.6g' % r['max_absolute_error'] if r.get('max_absolute_error') is not None else '未记录',
            '%.6g' % r['max_relative_error_max1'] if r.get('max_relative_error_max1') is not None else '未记录'))
    lines.append('相对误差分母为max(1,解析梯度绝对值,差分值绝对值)；本轮没有预先规定判定阈值，不把事后选阈值写成通过率。')
    for r in replay['boundary_line']:
        samples = r.get('samples', [])
        jumps = [abs(b['cost']-a['cost']) for a, b in zip(samples, samples[1:])]
        lines.append('%s沿原被拒线搜索方向跨边界采样%d点，相邻代价最大变化%s；这是完整重建MINCO后的实际目标值。' % (
            LABELS[bool(r.get('continuous'))], len(samples), show(max(jumps), 6) if jumps else '未记录'))
        for sample in samples:
            lines.append('  alpha=%s，代价%s，梯度沿方向分量%s，原半速j10采样时刻%s秒。' % (
                show(sample.get('alpha'), 6), show(sample.get('cost'), 9),
                show(sample.get('gradient_dot_direction'), 6),
                show(sample.get('first_piece_half_speed_sample_j10_dt'), 9)))
    lines += ['', '正常在线闭环（同一迎面场景，每组一次）']
    for row in rows:
        chain, cd = row['delivery_chain_counts'], row['control_delivery']
        delay = cd['first_refresh_to_new_reference_validated_command_wall_ms']
        lines += ['', row['label'],
            '结果：%s；%s物理球采样接触，最小采样间隙%s厘米；非目标低速停留%s秒。' % (
                '到达目标' if row['reached_goal'] else TERMINATIONS.get(row['termination'], str(row['termination'])),
                '发生' if row['physical_sample_collision'] else '未发现',
                show(100*row['minimum_physical_sample_gap_m'], 3), show(row['non_goal_low_speed']['total_s'])),
            '重规划%d次 → 原生优化验收通过%d次 → 最新CV验收后发布%d次 → MPPI实际采用%d条新参考。' % (
                chain['refresh_requests'], chain['requests_with_native_optimizer_pass'],
                chain['refresh_reference_publications'], chain['refresh_references_consumed']),
            '首次重规划开始到新参考首条有效控制：'+(show(delay/1000, 3)+'秒。' if delay is not None else '未交付。'),
            'MPPI内核P95/最大%s/%s毫秒；整循环P95/最大%s/%s毫秒。' % (
                show(row['kernel']['p95_ms'], 1), show(row['kernel']['max_ms'], 1),
                show(row['callback']['p95_ms'], 1), show(row['callback']['max_ms'], 1)),
            '原生优化失败原因：'+json.dumps(row['failure_reason_counts'], ensure_ascii=False)+'。',
            '最新CV发布拒绝%d次；诊断暂停事件%d条；仿真大步长记录%d条，最大%s秒。' % (
                len(row['latest_rejections']), len(row['online_pause_audit']['explicit_diagnostic_pause_events']),
                len(row['online_pause_audit']['plant_loop_gap_events']), show(row['online_pause_audit']['maximum_plant_loop_gap_s']))]
        for r in row['remani']['requests']:
            if r.get('request_id', 0) <= 1:
                continue
            lines.append('  请求%d：%s，耗时%s毫秒；优化原因%s；发布拒绝%s。' % (
                r['request_id'], r.get('outcome_class'), show(r.get('request_wall_ms'), 1),
                [c.get('reason') for c in r.get('optimizer_calls', [])], r.get('acceptance_rejections', [])))
        ad = row.get('first_adoption')
        if ad and ad.get('adoption_error'):
            error, following = ad['adoption_error'], ad['following_2s']
            lines.append('首次采用残差：位置%s厘米，朝向%s度，速度%+.3f米/秒；采用后已记录%s秒内，低速停留%s秒，倒车%s秒。' % (
                show(100*error['position_m'], 2), show(error['yaw_abs_deg'], 2), error['signed_speed_mps'],
                show(following['duration_s']), show(following['measured_non_goal_low_speed']['total_s']),
                show(following['measured_reverse_duration_s'])))
        else:
            lines.append('没有可关联的新参考采用，交接残差和采用后2秒指标为空，不按0统计。')
    lines += ['', '解释范围',
        '同输入回放用于判断数值和候选可行性；正常在线A/B用于判断交付与运动。相同种子不代表异步闭环输入完全相同。',
        '原生验收通过、最新CV发布通过、MPPI实际采用是三个环节，不能把单次优化成功当成新控制已经生效。',
        'GIF显示真实记录的完整67球投影、REMANI参考、获准的MPPI展开和有记录的搜索子集；不补画未记录搜索。',
        '物理球采样接触与预测模型碰撞是不同量；此处物理评估仍是离散采样，不是连续网格接触证明。',
        '固定机械臂、差速底盘、完整REMANI+MPPI ROS数值闭环，非Gazebo或实车。生产部署未改。', '']
    return '\n'.join(lines)


def chart(data, output):
    rows = data['rows']
    image = Image.new('RGB', (1240, 760), 'white')
    draw = ImageDraw.Draw(image)
    title, normal, small = [ImageFont.truetype(FONT, n) for n in (23, 18, 15)]
    draw.text((20, 14), '固定动态预测窗口：后端能否算出并及时交付可用轨迹', font=title, fill='#333333')
    draw.text((20, 52), '先看同输入原生可行性，再看无暂停在线交付；少量对照不代表总体成功率。', font=small, fill='#666666')
    draw.text((20, 91), '同输入回放（两边使用共同新缩放）', font=normal, fill='#444444')
    for i, branch in enumerate(data['replay']['continuations']):
        draw.text((20, 128+31*i), '%s：接受%s步，退出%s；%s' % (
            branch['label'], branch['accepted_steps'], branch['solver_exit'], verdict(branch)), font=small, fill='#555555')
    finite = data['replay']['finite_difference']
    if finite:
        draw.text((20, 192), '真实MINCO梯度最大相对误差：'+ '；'.join(
            ('B' if r.get('continuous') else 'A')+' '+('%.3g' % r['max_relative_error_max1'])
            for r in finite if r.get('max_relative_error_max1') is not None), font=small, fill='#666666')
    for i, row in enumerate(rows):
        y = 230+182*i
        draw.line((20, y-14, 1220, y-14), fill='#dddddd')
        draw.text((20, y), row['label'], font=normal, fill=('#2874a6', '#c47723')[i])
        delay = row['control_delivery']['first_refresh_to_new_reference_validated_command_wall_ms']
        c = row['delivery_chain_counts']
        draw.text((20, y+33), '首次重规划→有效新控制：'+(show(delay/1000, 3)+' s' if delay is not None else '未交付'), font=normal, fill='#444444')
        draw.text((20, y+67), '请求 %d → 原生通过 %d → 发布 %d → 采用 %d；最小物理间隙 %s cm' % (
            c['refresh_requests'], c['requests_with_native_optimizer_pass'], c['refresh_reference_publications'],
            c['refresh_references_consumed'], show(100*row['minimum_physical_sample_gap_m'], 2)), font=small, fill='#555555')
        draw.text((20, y+96), 'MPPI整循环P95 %s ms；非目标低速 %s s；%s；%s' % (
            show(row['callback']['p95_ms'], 1), show(row['non_goal_low_speed']['total_s']),
            '到达目标' if row['reached_goal'] else TERMINATIONS.get(row['termination'], str(row['termination'])),
            '发生物理采样接触' if row['physical_sample_collision'] else '未发现物理采样接触'), font=small, fill='#555555')
    draw.line((20, 615, 1220, 615), fill='#dddddd')
    for i, line in enumerate(data['conclusions'][:3]):
        draw.text((20, 634+28*i), line, font=small, fill='#555555')
    draw.text((20, 731), '完整67球、固定臂ROS数值闭环。原生硬检查保留；双时间尺度协同优化仍需继续验证。', font=small, fill='#777777')
    image.save(output)


def animation(rows, trials, output, max_frames):
    duration = max(trial.end-trial.anchor for trial in trials)
    ages = np.linspace(0., duration, min(max_frames, max(2, math.ceil(duration/.4)+1)))
    title, small = [ImageFont.truetype(FONT, n) for n in (19, 14)]
    frames = []
    for age in ages:
        image = Image.new('RGB', (1160, 510), 'white')
        draw = ImageDraw.Draw(image)
        draw.text((18, 8), '迎面对照：起步后 %.1f s' % age, font=title, fill='#333333')
        draw.text((18, 38), '蓝：机器人/实走　橙：REMANI参考　灰：已记录搜索　绿：获准展开　紫：CV预测', font=small, fill='#555555')
        for i, (row, trial) in enumerate(zip(rows, trials)):
            left, t = i*580+18, min(trial.anchor+float(age), trial.end)
            draw.text((left, 69), row['label'], font=title, fill='#333333')
            state = panel(image, trial, t, (left, 104, (i+1)*580-18, 410))
            draw = ImageDraw.Draw(image)
            if t >= trial.end-1e-9:
                status = '到达目标' if row['reached_goal'] else TERMINATIONS.get(row['termination'], str(row['termination']))
            else:
                status = trial.status_at(t)
                status = '控制有效' if status == 'ok' else status
            draw.text((left, 422), '%s　v=%+.2f　ω=%+.2f' % (status, state['v'], state['w']), font=small, fill='#555555')
        draw.text((18, 461), '2倍实录；完整67球投影；绿色为获准展开，灰色只显示真实保存的搜索子集。', font=small, fill='#777777')
        draw.text((18, 484), '完整REMANI＋底盘MPPI，固定臂ROS数值闭环；非Gazebo/实车。', font=small, fill='#777777')
        frames.append(image.convert('P', palette=Image.ADAPTIVE, colors=96))
    durations = [max(30, 10*round(float(dt)*500/10)) for dt in np.diff(ages)]+[1000]
    frames[0].save(output, save_all=True, append_images=frames[1:], duration=durations,
                   loop=0, disposal=2, optimize=True)
    return dict(frames=len(frames), playback_speedup=2., sha256=sha(output),
        full_physical_sphere_projection=True, sphere_counts=[len(t.body3d) for t in trials],
        actual_records_only=True, full_search_recorded=False,
        green_semantics='Recorded execution-authorized local rollout; gray is recorded search subset.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=DEFAULT)
    parser.add_argument('--replay', type=Path)
    parser.add_argument('--baseline', type=Path)
    parser.add_argument('--changed', type=Path)
    parser.add_argument('--max-frames', type=int, default=90)
    args = parser.parse_args()
    if args.max_frames < 2:
        raise ValueError('at least two animation frames required')
    replay = read_replay(args.replay or args.root/'replay/result.jsonl')
    pair = [online(args.baseline or args.root/'headon_A', LABELS[0]),
            online(args.changed or args.root/'headon_B', LABELS[1])]
    rows, trials = zip(*pair)
    data = dict(replay=replay, rows=list(rows), conclusions=conclusions(replay, rows),
        previous_paused_run_performance_baseline_valid=False,
        previous_pause_catchup_s=1.220230378996348,
        scope='Saved-input native optimizer check plus fresh unpaused fixed-arm REMANI+MPPI numerical ROS SIL A/B; no hardware.')
    output = args.root/'report'
    output.mkdir(parents=True, exist_ok=True)
    data['artifacts'] = dict(headon=animation(rows, trials, output/'迎面对照.gif', args.max_frames))
    chart(data, output/'对照数据.png')
    (output/'实验结果.txt').write_text(narrative(data))
    for trial in trials:
        trial.verify_unchanged()
    data['report_script_sha256'] = sha(Path(__file__))
    (output/'summary.json').write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')
    print('WINDOW_CONTINUITY_REPORT', output, flush=True)


if __name__ == '__main__':
    main()
