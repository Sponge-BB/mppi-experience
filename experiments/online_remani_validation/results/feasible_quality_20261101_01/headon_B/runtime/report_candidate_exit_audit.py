#!/usr/bin/env python3
"""Report one frozen native exit audit and an optional actual online recovery.

The capture run pauses owned numerical processes. Its full-run timing is not
an online performance benchmark. No diagnostic time is subtracted to invent
an online request-to-command delay.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from render_coupled_objective import sha
from render_dual_timescale import FONT
from report_delivery_feedback import load_trial as load_online
from report_frontend_rejoin import keyframes
from report_long_goal_crossing import panel
from report_static_agility import show
from report_time_indexed_collision import TERMINATIONS, inspect


HERE = Path(__file__).resolve().parent
DEFAULT = HERE/'results/candidate_exit_audit_20261101_01'


def native_text(verdict):
    if not verdict or not verdict.get('available'):
        return '未得到可重建候选'
    if verdict.get('native_pass'):
        return '通过完整原生检查'
    reason = verdict.get('reason')
    if reason == 'planning_collision':
        # Native MMConfig::checkcollision/checkPlanningCollision defines
        # 0 car-env, 1 arm-env, 2 car-arm, 3 arm-arm (mm_config.cpp:1293).
        kind = {0: '底盘与环境', 1: '机械臂与环境', 2: '底盘与机械臂', 3: '机械臂连杆间'}.get(
            verdict.get('rejection_value'), '类型%s的' % verdict.get('rejection_value'))
        return '%s碰撞，候选t=%s秒' % (kind, show(verdict.get('rejection_time'), 3))
    return str(reason)


def match_text(value):
    return '一致' if value is True else '不一致' if value is False else '未记录'


def conclusion(capture):
    if capture is None:
        return '没有取得预定首个动态请求的完整审计，不能判断退出时是否丢弃可用候选。'
    if not capture.get('stable'):
        return '审计期间冻结输入不一致，不能用该结果判断候选可行性。'
    verdict = capture['verdict']
    if capture['original_lbfgs_result'] == -1009:
        if verdict.get('available') and verdict.get('native_pass'):
            if capture['original_force_stop'] != 0:
                return '捕获到-1009且候选通过原生检查，但同时有强制停止标记；须结合原停止处理分支，不能仅凭退出码认定候选被丢弃。'
            return '捕获到线搜索达到上限（-1009），但实际最后接受的候选通过完整原生检查；默认按退出码拒绝，会丢掉这条候选。最新CV发布检查仍需独立验证。'
        if not verdict.get('available'):
            return '捕获到线搜索达到上限（-1009），但没有取得可复核的实际接受候选，尚不能判断是否丢弃可用轨迹。'
        return '捕获到线搜索达到上限（-1009），实际最后接受的候选仍未通过原生检查；不能仅放过退出码就交付。'
    return '本轮首个动态请求没有以-1009退出；只能判断本次候选的原生可行性，尚不能回答-1009是否丢弃可用轨迹。'


def mechanism(source):
    row, trial = inspect(source, '首个动态请求退出复核')
    captures = trial.reader.jsonl('exit_audit/capture.jsonl')
    if len(captures) > 1:
        raise ValueError('expected one predetermined first-refresh audit, not selected failures')
    capture = captures[0] if captures else None
    timeline = trial.reader.jsonl('dual_timeline.jsonl')
    endings = [r for r in timeline if r['event'] == 'candidate_exit_audit_end']
    continuation_consistency = None
    if capture and capture['continuation_triggered']:
        branches = capture['continuations']
        original_weight = next((r for r in branches if r['factor'] == 1.), None)
        weighted = next((r for r in branches if r['factor'] == 10.), None)
        continuation_consistency = dict(
            accepted_evaluation_matches_actual_x=capture.get('accepted_evaluation') is not None and
                capture['accepted_evaluation']['x_hash'] == capture['accepted_x_hash'],
            same_initial_x=all(r.get('initial_x_matches') is True and
                r['initial_x_hash'] == capture['accepted_x_hash'] for r in branches),
            equal_original_remaining_budget=all(abs(r['remaining_allowance_s']-
                capture['original_remaining_budget_s']) < 1e-12 for r in branches),
            weight1_original_cost_matches=original_weight.get('weight1_original_cost_matches') if original_weight else None,
            weight1_original_gradient_matches=original_weight.get('weight1_original_gradient_matches') if original_weight else None)
        continuation_consistency['comparison_valid'] = (capture['stable'] is True and len(branches) == 2
            and all(v is True for v in continuation_consistency.values()))
        continuation_consistency['weight10_initial_cost_and_gradient_changed'] = (
            weighted is not None and original_weight is not None
            and weighted.get('initial_evaluation_available') is True
            and original_weight.get('initial_evaluation_available') is True
            and weighted['initial_cost'] != original_weight['initial_cost']
            and weighted['initial_gradient_hash'] != original_weight['initial_gradient_hash'])
    capture_info = dict(source=str(source), capture=capture,
        conclusion=conclusion(capture),
        original_solver_exit_code=capture['original_lbfgs_result'] if capture else None,
        accepted_iterate_native_verdict=capture.get('verdict') if capture else None,
        accepted_evaluation_matches_actual_x=(capture.get('accepted_evaluation') is not None and
            capture['accepted_evaluation']['x_hash'] == capture['accepted_x_hash']) if capture else None,
        same_input_stable=capture.get('stable') if capture else None,
        conditional_continuations=(capture['continuations'] or None) if capture else None,
        continuation_consistency=continuation_consistency,
        diagnostic_wall_ms=endings[-1]['diagnostic_wall_ms'] if endings else None,
        request_to_new_validated_control_ms=None, stop_benefit_s=None,
        online_navigation_timing_valid=False,
        capture_run_navigation=dict(reached_goal=row['reached_goal'], termination=row['termination'],
            physical_sample_collision=row['physical_sample_collision'],
            minimum_physical_sample_gap_m=row['minimum_physical_sample_gap_m'],
            actual_path_length_m=row['actual_path_length_m'],
            scope='Actual capture-run motion, affected by diagnostics; not an online-performance comparison.'),
        source_sha256=trial.reader.hashes)
    trial.verify_unchanged()
    return capture_info, trial


def brief(data):
    mechanism = data['mechanism']
    capture, online = mechanism['capture'], data['online_recovery']
    boundary = data.get('prediction_window_boundary_check')
    lines = ['REMANI候选退出复核：实验结果', '']
    if boundary:
        lines += ['本轮找到一处具体问题：改变轨迹段时间时，动态预测软罚在3秒预测窗边界突然加回或删掉，目标函数不连续。',
                  '这处缺陷来自我们此前加入的时间索引动态代价，位于当前隔离实验实现；应先修正它，不能笼统归咎于原生REMANI框架或CPU。',
                  '同一失败记录的最后试探恰好跨过该边界并发生代价跳升，形成可复查的线搜索失败线索；修复收益尚未验证。']
    lines += [mechanism['conclusion'], '']
    if capture:
        lines += ['固定选择首个动态重规划请求%d，在知道求解结果前确定，不挑失败案例。' % capture['request_id'],
            '原求解器退出码%s，退出前优化已用%s毫秒，原1.2秒预算还剩%s毫秒。' % (
                capture['original_lbfgs_result'], show(1000*capture['original_elapsed_s'], 1),
                show(1000*capture['original_remaining_budget_s'], 1)),
            '实际最后接受的候选：'+native_text(capture['verdict'])+'。',
            '接受向量与保存的代价/梯度对应%s；冻结输入保持%s；ESDF前后CRC为%s/%s。' % (
                '一致' if mechanism['accepted_evaluation_matches_actual_x'] else '未确认一致',
                '一致' if mechanism['same_input_stable'] else '不一致',
                capture['esdf_crc32'], capture['esdf_crc32_after'])]
        independent = data.get('independent_dynamic_check')
        if independent:
            lines.append('同候选、同冻结CV的独立动态复核：')
            for check in independent['checks']:
                lines.append('  %s：首次相交在候选t=%s秒，预测有效窗口内最小模型间隙%s厘米。' % (
                    '常速' if check['execution_scale'] == 1. else '半速',
                    show(check['first_collision']['local_t_s'], 3) if check['first_collision'] else '无',
                    show(100*check['minimum']['minimum_model_gap_m'], 3) if check['minimum'] else '无有效点对'))
            if all(check['first_collision'] for check in independent['checks']):
                lines.append('本次常速也不安全，不是只有半速检查才造成拒绝。这里是动态模型间隙，不是实车物理接触。')
        if capture['continuation_triggered']:
            lines += ['', '同候选继续优化诊断（未发布，不能当在线对照）：']
            consistency = mechanism['continuation_consistency']
            lines.append('原权重首个输入的代价与梯度复核、两分支相同起点和剩余预算：'+
                         ('一致。' if consistency['comparison_valid'] else '没有全部确认一致，不据此比较修复效果。'))
            for branch in capture['continuations']:
                lines.append('权重%s：同一起点、原剩余预算%s毫秒；求解%s毫秒，退出码%s；%s。' % (
                    show(branch['factor'], 0), show(1000*branch['remaining_allowance_s'], 1),
                    show(1000*branch['solver_wall_s'], 1), branch['lbfgs_result'], native_text(branch['verdict'])))
                lines.append('  初始接受向量精确匹配：'+match_text(branch.get('initial_x_matches'))+'。')
                if branch['factor'] == 1.:
                    lines.append('  原权重首个输入的代价精确匹配：%s；梯度精确匹配：%s。' % (
                        match_text(branch.get('weight1_original_cost_matches')),
                        match_text(branch.get('weight1_original_gradient_matches'))))
            lines.append('权重10只加强原失败段的环境碰撞软罚。两分支的加权目标不同，不用代价高低直接判断改善。')
            if consistency['weight10_initial_cost_and_gradient_changed']:
                lines.append('10倍权重确实改变了同一初始向量上的代价和梯度，开关已生效；不能把最终无变化归因于罚权没有接入。')
            if all(r['solver_wall_s'] < r['remaining_allowance_s'] and r['lbfgs_result'] == -1009 and r['force_stop'] == 0
                   for r in capture['continuations']):
                lines.append('两分支都在预算用完之前因线搜索达到上限而停止，这次不是剩余求解时间耗尽。')
        else:
            lines.append('没有触发“同候选权重1/10继续优化”条件，分支效果为空，不能称已验证。')
    lines += ['', '暂停诊断额外用时%s毫秒。该次运行包含暂停、复核与可能的分支求解，不能报告在线交付速度改善。' %
              show(mechanism['diagnostic_wall_ms'], 1)]
    navigation = mechanism['capture_run_navigation']
    lines.append('诊断实录最终%s；%s物理球采样接触，最小间隙%s厘米。%s' % (
        '到达目标' if navigation['reached_goal'] else TERMINATIONS.get(navigation['termination'], navigation['termination']),
        '发生' if navigation['physical_sample_collision'] else '未发现',
        show(100*navigation['minimum_physical_sample_gap_m'], 3),
        '这次不是成功避障；诊断暂停影响时序，不用于评价正常在线避障性能。' if navigation['physical_sample_collision'] else
        '此处只保留实际终止情况，不作在线性能对比。'))
    if boundary:
        critical = boundary['critical_branch']
        lines += ['', '进一步定位到的开关：',
            '第一段时间仅从%s秒变到%s秒，就让一个半速采样从预测%s秒（窗外）移到%s秒（窗内），恢复该处动态罚项。' % (
                show(critical['accepted_T_s'], 6), show(critical['last_trial_T_s'], 6),
                show(critical['accepted_raw_dt_s'], 6), show(critical['last_trial_raw_dt_s'], 6)),
            '全部%d个分支采样中仅%d个切换有效性；总代价%s→%s，增加%s，而局部梯度原本预计变化%s。' % (
                boundary['checked_branch_samples'], boundary['validity_mask_changes'],
                show(boundary['accepted_total_cost'], 3), show(boundary['last_trial_total_cost'], 3),
                show(boundary['total_cost_delta'], 3), show(boundary['accepted_gradient_dot_delta'], 3)),
            '新加回罚项的局部估算约%s，与代价增加相近；这是固定已接受几何的估算，不是完整重算最后试探的全部目标。' %
                show(boundary['approximate_reintroduced_sample_penalty_integral'], 3),
            '尚未逐次重建整段线搜索历史，不能称所有梯度都错误，也不能把所有交付失败都归因于这一处。']
    branches = capture['continuations'] if capture else []
    if (capture and capture['stable'] and capture['original_lbfgs_result'] == -1009 and
            not capture['verdict']['native_pass'] and mechanism['continuation_consistency'] and
            mechanism['continuation_consistency']['comparison_valid'] and
            all(r['lbfgs_result'] == -1009 and r['final_x_hash'] == r['initial_x_hash'] for r in branches)):
        lines += ['', '这次锁定了什么：',
            '最后接受的候选确实不能交付，不只是退出码将一条安全轨迹丢掉。',
            '原权重重启和失败段碰撞权重乘10的最终优化变量都没有变化，仍停在原接受候选，碰撞没有消除。',
            ('下一核心是先处理预测窗边界软代价的连续性，再用本次冻结输入验证能否接受下降步并通过原生验收。'
             if boundary else '下一核心是查清线搜索为什么不能接受更新，并核对梯度和数值计算是否一致；当前尚未确认梯度错误。'),
            '本次结果不支持单纯放过负返回码、继续加预算或加罚权就能解决交付问题。']
    if online:
        delay = online['control_delivery']['first_refresh_to_new_reference_validated_command_wall_ms']
        lines += ['', '另一次真正在线恢复运行：',
            '首次重规划请求到新参考首条有效控制：'+(show(delay/1000, 3)+'秒。' if delay is not None else '未交付。'),
            '结果：%s；最小物理采样间隙%s厘米；非目标低速停留%s秒。' % (
                '到达目标' if online['reached_goal'] else TERMINATIONS.get(online['termination'], online['termination']),
                show(100*online['minimum_physical_sample_gap_m'], 2), show(online['non_goal_low_speed']['total_s'], 3)),
            '退出恢复事件：'+json.dumps(online['exit_recovery_events'], ensure_ascii=False)]
    else:
        lines += ['本轮没有另跑在线恢复版本，在线恢复效果为空。']
    lines += ['', '原生检查通过、最新CV发布检查通过、机器人实际采用，是三个不同环节。诊断候选没有发布。',
        '复核使用同一进程内冻结的ESDF、CV、起终状态和反馈；没有重建简化碰撞模型。',
        'GIF显示实录和完整67球投影。二维投影只用于观看，碰撞仍按原三维球检查。',
        '固定机械臂的完整REMANI+底盘MPPI数值ROS实验，非Gazebo或实车。']
    return '\n'.join(lines)+'\n'


def chart(data, output):
    capture, online = data['mechanism']['capture'], data['online_recovery']
    image = Image.new('RGB', (1240, 570), 'white')
    draw = ImageDraw.Draw(image)
    title, text, small = [ImageFont.truetype(FONT, n) for n in (23, 18, 15)]
    draw.text((20, 14), 'REMANI退出复核：新增动态代价在预测窗边界跳变' if data.get('prediction_window_boundary_check') else
              'REMANI退出复核：是退出码丢候选，还是候选本身不可用？', font=title, fill='#333333')
    draw.text((20, 52), '问题来自新增实验动态代价；当前只定位，修复收益未验证。诊断暂停耗时不当在线性能。'
              if data.get('prediction_window_boundary_check') else
              '预先固定首个动态请求；冻结原生模型复核。诊断暂停耗时不当在线性能。', font=small, fill='#666666')
    if capture:
        draw.text((20, 96), '原求解退出码：%s；退出前已用 %s ms；剩余 %s ms' % (
            capture['original_lbfgs_result'], show(1000*capture['original_elapsed_s'], 1),
            show(1000*capture['original_remaining_budget_s'], 1)), font=text, fill='#444444')
        draw.text((20, 132), '实际最后接受的候选：'+native_text(capture['verdict']), font=text, fill='#2874a6')
        draw.text((20, 167), '冻结输入：%s；候选与代价记录：%s' % (
            '一致' if capture['stable'] else '不一致',
            '一致' if data['mechanism']['accepted_evaluation_matches_actual_x'] else '未确认一致'), font=small, fill='#555555')
        independent = data.get('independent_dynamic_check')
        if independent:
            minima = {r['execution_scale']: r['minimum'] for r in independent['checks']}
            if minima.get(1.) and minima.get(.5):
                draw.text((20, 188), '常速/半速最小动态模型间隙：%s / %s cm；两者均有相交' % (
                    show(100*minima[1.]['minimum_model_gap_m'], 2),
                    show(100*minima[.5]['minimum_model_gap_m'], 2)), font=small, fill='#a34c3c')
        draw.line((20, 208, 1220, 208), fill='#dddddd')
        draw.text((20, 225), '同起点继续优化：仅在原生环境碰撞且有剩余预算时触发', font=text, fill='#444444')
        if capture['continuation_triggered']:
            for index, branch in enumerate(capture['continuations']):
                draw.text((20, 265+37*index), '权重%s：%s；求解%s ms；退出码%s' % (
                    show(branch['factor'], 0), native_text(branch['verdict']),
                    show(1000*branch['solver_wall_s'], 1), branch['lbfgs_result']), font=small, fill='#555555')
        else:
            draw.text((20, 269), '本次未触发；不将空结果写成成功或失败。', font=text, fill='#777777')
    else:
        draw.text((20, 110), '没有取得预定请求的完整审计，无法判断。', font=text, fill='#777777')
    draw.line((20, 359, 1220, 359), fill='#dddddd')
    if online:
        delay = online['control_delivery']['first_refresh_to_new_reference_validated_command_wall_ms']
        draw.text((20, 380), '独立在线恢复：首次请求→有效新控制 '+(show(delay/1000, 3)+' s' if delay is not None else '未交付'), font=text, fill='#c47723')
        draw.text((20, 418), '是否到达：%s；最小物理间隙 %s cm；非目标低速 %s s' % (
            '是' if online['reached_goal'] else '否', show(100*online['minimum_physical_sample_gap_m'], 2),
            show(online['non_goal_low_speed']['total_s'], 3)), font=small, fill='#555555')
    else:
        draw.text((20, 380), '独立在线恢复：本轮未运行，效果尚未验证。', font=text, fill='#777777')
        nav = data['mechanism']['capture_run_navigation']
        draw.text((20, 418), '诊断实录：%s；最小物理间隙 %s cm（含暂停，不作在线性能对比）' % (
            '发生采样接触并结束' if nav['physical_sample_collision'] else '未见采样接触',
            show(100*nav['minimum_physical_sample_gap_m'], 3)), font=small, fill='#a34c3c' if nav['physical_sample_collision'] else '#777777')
    boundary = data.get('prediction_window_boundary_check')
    if boundary:
        draw.text((20, 453), '预测窗边界仅切换1个采样罚项，代价 %s→%s；修复收益未验证。' % (
            show(boundary['accepted_total_cost'], 2), show(boundary['last_trial_total_cost'], 2)), font=small, fill='#a34c3c')
    draw.text((20, 484), '原生检查通过 ≠ 最新CV验收通过 ≠ 实际采用；冻结诊断候选没有发布。', font=small, fill='#666666')
    draw.text((20, 520), '67球三维检查，固定机械臂，完整REMANI+MPPI数值ROS实验。', font=small, fill='#777777')
    image.save(output)


def single_gif(trial, output, diagnostic, max_frames):
    duration = trial.end-trial.anchor
    ages = np.linspace(0., duration, min(max_frames, max(2, math.ceil(duration/.4)+1)))
    title, small = [ImageFont.truetype(FONT, n) for n in (20, 14)]
    frames = []
    for age in ages:
        image = Image.new('RGB', (1040, 600), 'white')
        draw = ImageDraw.Draw(image)
        time = min(trial.anchor+float(age), trial.end)
        draw.text((20, 12), ('退出复核实录（含诊断暂停）' if diagnostic else '候选恢复在线实录')+'　起步后 %.1f s' % age,
                  font=title, fill='#333333')
        draw.text((20, 46), '蓝：机器人及实走轨迹　橙：REMANI参考　灰：已记录搜索　绿：获准展开　紫：CV预测', font=small, fill='#555555')
        state = panel(image, trial, time, (20, 80, 1020, 490))
        draw = ImageDraw.Draw(image)
        reason = trial.result.get('reason')
        result = ('到达目标' if reason == 'goal' else '结束：'+TERMINATIONS.get(reason, str(reason))) if time >= trial.end-1e-9 else trial.status_at(time)
        draw.text((20, 505), '%s　v=%+.2f m/s　ω=%+.2f rad/s' % (result, state['v'], state['w']), font=small, fill='#555555')
        draw.text((20, 539), '本次含额外暂停复核，不作为实时避障性能对比。' if diagnostic else '独立在线运行；保留所有停止与失败标记。', font=small, fill='#777777')
        draw.text((20, 568), '实录2倍播放；完整67球投影；固定臂数值ROS实验，非Gazebo/实车。', font=small, fill='#777777')
        frames.append(image.convert('P', palette=Image.ADAPTIVE, colors=96))
    durations = [max(30, 10*round(float(dt)*500/10)) for dt in np.diff(ages)]+[1000]
    frames[0].save(output, save_all=True, append_images=frames[1:], duration=durations,
                   loop=0, disposal=2, optimize=True)
    return dict(frames=len(frames), playback_speedup=2., sha256=sha(output),
                diagnostic_pause=diagnostic, full_sphere_projection=True, actual_records_only=True,
                full_search_recorded=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=DEFAULT)
    parser.add_argument('--capture', type=Path)
    parser.add_argument('--online', type=Path)
    parser.add_argument('--out', type=Path)
    parser.add_argument('--max-frames', type=int, default=100)
    args = parser.parse_args()
    capture, capture_trial = mechanism(args.capture or args.root/'headon_capture')
    online, online_trial = None, None
    if args.online:
        online, online_trial = load_online(args.online, '独立在线候选恢复')
        timeline = online_trial.reader.jsonl('dual_timeline.jsonl')
        online['exit_recovery_events'] = [r for r in timeline if r['event'].startswith('candidate_exit_recovery')]
        online['source_sha256'] = online_trial.reader.hashes
    data = dict(mechanism=capture, online_recovery=online,
                scope='One predetermined paused native candidate mechanism audit; optional separate online recovery.')
    output = args.out or args.root/'report'
    output.mkdir(parents=True, exist_ok=True)
    independent_path = output/'动态碰撞复核.json'
    data['independent_dynamic_check'] = json.loads(independent_path.read_text(encoding='utf8')) if independent_path.is_file() else None
    data['independent_dynamic_check_sha256'] = sha(independent_path) if independent_path.is_file() else None
    boundary_path = output/'预测窗口切换核对.json'
    data['prediction_window_boundary_check'] = json.loads(boundary_path.read_text(encoding='utf8')) if boundary_path.is_file() else None
    data['prediction_window_boundary_check_sha256'] = sha(boundary_path) if boundary_path.is_file() else None
    chart(data, output/'候选退出复核.png')
    animation = output/('在线恢复实录.gif' if online_trial else '退出复核实录.gif')
    data['animation'] = single_gif(online_trial or capture_trial, animation, not bool(online_trial), args.max_frames)
    keyframes(animation, output/'关键帧.png')
    (output/'实验结果.txt').write_text(brief(data), encoding='utf8')
    (output/'summary.json').write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    capture_trial.verify_unchanged()
    if online_trial:
        online_trial.verify_unchanged()
    print('CANDIDATE_EXIT_AUDIT_REPORT', output, flush=True)


if __name__ == '__main__':
    main()
