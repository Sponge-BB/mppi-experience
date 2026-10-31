#!/usr/bin/env python3
"""Post-run, evidence-only report for paired dynamic-computation experiments.

No ROS, controller replay, reconstructed candidates or optimization is run.
Run only after the complete timed batch has stopped. All input records remain
untouched. These are fixed-arm numerical ROS SIL cases, not Gazebo/hardware.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from native_realtime_runtime import deadline_zero_pattern
from render_dual_timescale import FONT
from render_native_collision_3d import Trial as NativeTrial, draw_panel
from report_native_realtime import percentiles, summarize_trial as summarize_parent

GROUPS = ('baseline', 'fast')
LABELS = {'baseline': '修改前', 'fast': '省重复计算后'}
SCENES = {'empty': '空旷', 'static': '静态障碍', 'crossing': '横穿障碍',
          'dynamic_head_on_center': '迎面障碍'}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class Trial(NativeTrial):
    def __init__(self, source, group):
        if group not in GROUPS:
            raise ValueError('Expected baseline or fast panel')
        super().__init__(source, 'native3d')
        enabled = self.manifest.get('native_dynamic_fast', {}).get('enabled')
        if not isinstance(enabled, bool) or enabled != (group == 'fast'):
            raise ValueError('native_dynamic_fast.enabled differs from requested group')
        if self.manifest.get('native_realtime', {}).get('demand') is not True:
            raise ValueError('Both groups must enable the same free-path demand policy')
        if self.status.get('result_written') is not True:
            raise ValueError('Finish timed trial before reporting')
        self.group, self.title = group, LABELS[group]


def phase_of(row):
    decision = (row.get('local_phase') or {}).get('decision', [])
    if len(decision) < 7:
        return 'unknown'
    if decision[6] > .5:
        return 'intervention'
    moving = any(len(o) >= 6 and o[5] > .5 for o in row.get('observed_obstacles', []))
    return 'monitoring' if moving else 'freepath'


def timing_by_phase(rows):
    solved = [r for r in rows if r.get('reference_points', 0) > 0 and r.get('code') == 0]
    return {name: dict(cycles=len(selected), kernel_ms=percentiles(selected, 'kernel_ms'),
                      callback_ms=percentiles(selected, 'callback_ms'))
            for name in ('overall', 'freepath', 'monitoring', 'intervention', 'unknown')
            for selected in [[r for r in solved if name == 'overall' or phase_of(r) == name]]}


def deadline_zero_duration(trial):
    """Integrate issued zero commands until the next publication, not compute time.

    Actual command-publication timestamps are preferred. Diagnostic start times
    are not command timestamps, so their fallback durations are labelled estimates.
    """
    cycles = trial.reader.jsonl('state_mppi_cycles.jsonl')
    measured = []
    for row in cycles:
        t = row.get('command_publish_after_ros_time')
        if t is not None and math.isfinite(t):
            measured.append(dict(t=float(t)-1000., reason=row.get('reason'),
                zero=abs(row.get('command_v', 0.))+abs(row.get('command_w', 0.)) <= 1e-9))
    exact = bool(measured)
    if not measured:
        measured = [dict(t=float(r['t']), reason=r.get('reason'),
            zero=abs(r.get('applied_v', 0.))+abs(r.get('applied_w', 0.)) <= 1e-9)
            for r in trial.diagnostics]
    measured.sort(key=lambda r: r['t'])
    duration, count = 0., 0
    for index, row in enumerate(measured):
        if row['reason'] != 'deadline_miss' or not row['zero'] or row['t'] >= trial.end:
            continue
        count += 1
        next_t = measured[index+1]['t'] if index+1 < len(measured) else trial.end
        duration += max(0., min(next_t, trial.end)-max(row['t'], float(trial.state['t'][0])))
    return dict(cycles=count, issued_zero_duration_s=duration,
        publication_timestamp_based=exact,
        duration_is_estimate=not exact,
        basis='issued zero until next recorded command publication' if exact else
              'diagnostic cycle-start intervals; publication timing unavailable',
        validated_nonzero_action_discarded_cycles=sum(deadline_zero_pattern(r) for r in trial.diagnostics),
        not_a_sum_of_callback_times=True)


def delivery_metrics(trial):
    summary = trial.reader.json('analysis/summary.json', {})
    if not summary:
        return dict(available=False, completed_refresh_requests=None,
                    accepted_refresh_requests=None, consumed_refresh_references=None,
                    first_blocked_to_consumed_s=None, request_wall_ms=None,
                    optimizer_wall_ms=None)
    requests = [r for r in summary.get('requests', []) if r.get('request_id', 0) > 1]
    chains = [r for r in summary.get('reference_chains', [])
              if r.get('request_id', 0) > 1 and r.get('consume_sim_s') is not None]
    blocked = summary.get('latency', {}).get('first_native_blocked_sim_s')
    after = [r['consume_sim_s'] for r in chains if blocked is not None and r['consume_sim_s'] >= blocked]
    return dict(available=True, completed_refresh_requests=len(requests),
        accepted_refresh_requests=sum(r.get('success') is True for r in requests),
        consumed_refresh_references=len(chains),
        first_blocked_to_consumed_s=min(after)-blocked if after else None,
        request_wall_ms=percentiles(requests, 'request_wall_ms'),
        optimizer_wall_ms=percentiles([c for r in requests for c in r.get('optimizer_calls', [])], 'wall_ms'),
        unfinished_requests=summary.get('unmatched', {}).get('requests', []),
        initial_plan_excluded=True, source='analysis/summary.json')


def summarize_trial(source, group):
    trial = Trial(source, group)  # Explicitly validate the new switch first.
    # Both branches use DEMAND=1. Never pass old "original" to its validator.
    value = summarize_parent(source, 'demand')
    value.update(group=group, label=LABELS[group], scene=trial.manifest['scene'],
        phase_timing=timing_by_phase(trial.diagnostics), deadline_zero=deadline_zero_duration(trial),
        remani_delivery=delivery_metrics(trial),
        native_dynamic_fast=trial.manifest['native_dynamic_fast'],
        navigation_success_claim=bool(value['reached_goal'] and not value['native3d_physical_sample_collision']))
    trial.verify_unchanged()
    value['input_sha256'].update(trial.reader.hashes)
    return value


def render_pair(baseline, fast, out, max_frames=120):
    trials = [Trial(baseline, 'baseline'), Trial(fast, 'fast')]
    if len({t.manifest['scene'] for t in trials}) != 1 or len({t.parameters['seed'] for t in trials}) != 1:
        raise ValueError('Same scene and seed required')
    if not np.array_equal(trials[0].body3d, trials[1].body3d) or trials[0].resolution != trials[1].resolution:
        raise ValueError('Native collision geometry changed between panels')
    out = Path(out).resolve()
    if any(out == t.source or t.source in out.parents for t in trials):
        raise ValueError('Cannot write visualizations inside measured trials')
    if max_frames < 2:
        raise ValueError('At least two GIF frames required')
    out.mkdir(parents=True, exist_ok=False)
    by_motion = all(t.result.get('motion_anchor') is not None for t in trials)
    if not by_motion:
        for t in trials:
            t.anchor = float(t.result.get('goal_sent') or t.state['t'][0])
    duration = max(t.end-t.anchor for t in trials)
    ages = np.linspace(0., duration, min(max_frames, max(2, int(math.ceil(duration/.3))+1)))
    frame_ms = max(20, 10*round(1000*float(ages[1]-ages[0])/2./10))
    font, small = ImageFont.truetype(FONT, 18), ImageFont.truetype(FONT, 14)
    frames = []
    for age in ages:
        image = Image.new('RGB', (1160, 482), 'white')
        draw = ImageDraw.Draw(image)
        draw.text((16, 8), '%s　%.1f s　%s' % (SCENES.get(trials[0].manifest['scene'], trials[0].manifest['scene']),
            age, '实测起步对齐' if by_motion else '双方发目标时间对齐'), font=font, fill='#333333')
        draw.text((16, 34), '橙：真实REMANI参考　蓝：实际机器人投影　灰：真实MPPI候选子集　绿：获准展开', font=small, fill='#555555')
        for index, trial in enumerate(trials):
            t = min(trial.anchor+age, trial.end)
            draw.text((index*580+16, 64), trial.title, font=font, fill='#333333')
            row = draw_panel(image, trial, t, (index*580+16, 98, (index+1)*580-16, 404))
            draw = ImageDraw.Draw(image)
            draw.text((index*580+16, 414), '%s　v=%+.2f m/s' % (trial.status_at(t), row['v']), font=small, fill='#555555')
            if trial.result.get('motion_anchor') is None:
                draw.text((index*580+16, 434), '未起步，不能评价动态避障成功', font=small, fill='#a44b33')
        draw.text((16, 452), '固定臂差速数值闭环，非Gazebo/实车；三维球模型判碰撞，不以俯视重叠代替。', font=small, fill='#777777')
        frames.append(image.convert('P', palette=Image.ADAPTIVE, colors=96))
    frames[0].save(out/'comparison.gif', save_all=True, append_images=frames[1:],
        duration=[frame_ms]*(len(frames)-1)+[max(500, frame_ms)], loop=0, disposal=2, optimize=True)
    frames[-1].convert('RGB').save(out/'paths.png')
    for t in trials:
        t.verify_unchanged()
    record = dict(scene=trials[0].manifest['scene'], frames=len(frames),
        alignment='each actual motion anchor' if by_motion else 'both goal publication times',
        source_sha256={t.group: t.reader.hashes for t in trials},
        actual_robot_projection=True, actual_remani_references=True,
        actual_recorded_search_subset_only=True, no_controller_reexecution=True,
        no_future_actor_or_route_fabrication=True)
    (out/'artifact_integrity.json').write_text(json.dumps(record, ensure_ascii=False, indent=2)+'\n')
    return record


def show(value, digits=2):
    return '无记录' if value is None else ('%.*f' % (digits, value))


def replay_summary(record):
    rows = record.get('records', [])
    checks = [c for r in rows for c in r.get('checks', [])]
    kinds = ('same_inputs', 'core_output_exact', 'candidate_classifications_exact',
             'candidate_traces_exact_except_declared_pruned_cost')
    return dict(sample_count=len(rows), repeats=record.get('repeats'),
        checks_all_passed=bool(checks) and all(c.get('passed') is True for c in checks),
        checks={kind: all(c.get('checks', {}).get(kind) is True for c in checks)
                if checks else None for kind in kinds},
        unprofiled_selected_input_timing=record.get('aggregate_sample_timing'),
        libraries=record.get('libraries'),
        historical_warm_state_reproduced=False,
        not_online_callback_timing=True,
        not_navigation_evidence=True)


def timing_chart(rows, out):
    """Small readable chart, using only actual online callback wall times."""
    image = Image.new('RGB', (1160, 660), 'white')
    draw = ImageDraw.Draw(image)
    font, small = ImageFont.truetype(FONT, 20), ImageFont.truetype(FONT, 16)
    draw.text((25, 12), '在线闭环：计算与超期停车', font=font, fill='#333333')
    colors = {'baseline': '#9b9b9b', 'fast': '#2874a6'}
    scenes = list(dict.fromkeys(r['scene'] for r in rows))
    metrics = [('完整回调 P95 / 毫秒', lambda r: (r['phase_timing']['overall']['callback_ms'] or {}).get('p95')),
               ('避让介入回调 P95 / 毫秒', lambda r: (r['phase_timing']['intervention']['callback_ms'] or {}).get('p95')),
               ('超期清零持续时间 / 秒', lambda r: r['deadline_zero']['issued_zero_duration_s'])]
    for panel, (title, measure) in enumerate(metrics):
        x0, y0 = 25+panel*380, 72
        draw.text((x0, y0), title, font=small, fill='#333333')
        values = [measure(r) for r in rows]
        maximum = max([v for v in values if v is not None]+[1.])
        for index, scene in enumerate(scenes):
            y = y0+45+index*155
            draw.text((x0, y), SCENES.get(scene, scene), font=small, fill='#333333')
            for j, group in enumerate(GROUPS):
                row = next(r for r in rows if r['scene'] == scene and r['group'] == group)
                value, yy = measure(row), y+30+j*40
                width = 245*value/maximum if value is not None else 0.
                if width > 0:
                    draw.rectangle((x0, yy, x0+width, yy+19), fill=colors[group])
                draw.text((x0+width+5, yy-2), show(value), font=small, fill='#333333')
    draw.text((25, 595), '灰：修改前　蓝：省重复计算后。单种子配对复现，不代表统计成功率。', font=small, fill='#555555')
    draw.text((25, 622), '回调包含规划与检查；不是MPPI内核时间。无介入数据时标“无记录”，不写成零。', font=small, fill='#555555')
    image.save(out)


def narrative(result):
    replay = result['replay']['record']
    replay_brief = result['replay']['summary']
    text = ['本轮做了什么',
        '先把保存的同一份输入交给两个版本计算，再分别重新跑在线闭环。不是只播放旧运动轨迹。',
        '修改只省去重复或已经无用的计算；机器人碰撞球、预测时间、搜索数量、安全门槛没有因此放宽。', '',
        '同输入回放', '回放验收结果：%s。具体输入身份、控制比较和重复计时见 report.json 内的原始回放结果。' %
        ('通过' if replay.get('passed') is True else '未通过或记录不完整'),
        '回放没有执行机器人动作；它不等于闭环避障通过。性能分析开关的耗时不拿来当正式在线耗时。', '',
        '重新跑在线闭环的结果']
    insert = text.index('重新跑在线闭环的结果')
    details = ['实际选取%d份输入，每份每版本正式重复次数：%s。' %
               (replay_brief['sample_count'], replay_brief['repeats'] if replay_brief['repeats'] is not None else '无记录')]
    timing = replay_brief['unprofiled_selected_input_timing'] or {}
    for name, label in (('off', '修改前'), ('on', '省重复计算后')):
        kernel = timing.get(name, {}).get('kernel_ms', {})
        if kernel:
            details.append('%s在这些固定输入中的内核耗时P50/P95为%s/%s毫秒。这里只统计回放调用，不是整个ROS回调。' %
                           (label, show(kernel.get('p50')), show(kernel.get('p95'))))
    checks = replay_brief['checks']
    if checks.get('core_output_exact') is not None:
        details.append('所选输入的动作、选中控制和验证输出%s；被提前淘汰路线的代价允许不同，不拿它当有效动作。' %
                       ('保持一致' if checks['core_output_exact'] else '未保持一致，不能当等价提速验收'))
    text[insert:insert] = details+['']
    for row in result.get('startup_trials', []):
        overall = row['phase_timing']['overall']
        text.append('空旷起步，%s：最大位移%s米，完整回调P50/P95为%s/%s毫秒，超期清零%d轮。' %
            (row['label'], show(row['max_measured_displacement_m']),
             show((overall['callback_ms'] or {}).get('p50')), show((overall['callback_ms'] or {}).get('p95')),
             row['deadline_zero']['cycles']))
    for row in result['trials']:
        phase = row['phase_timing']
        overall, intervention = phase['overall'], phase['intervention']
        text += ['%s，%s：%s；%s。' % (SCENES.get(row['scene'], row['scene']), row['label'],
            '到达目标' if row['reached_goal'] else '因%s结束，未到达目标' % row['termination'],
            '检测到三维采样接触' if row['native3d_physical_sample_collision'] else '本次没有检测到三维采样接触'),
            '  成功返回的计算共%d轮；MPPI内核P50/P95为%s/%s毫秒，完整回调为%s/%s毫秒。' %
            (overall['cycles'], show((overall['kernel_ms'] or {}).get('p50')),
             show((overall['kernel_ms'] or {}).get('p95')), show((overall['callback_ms'] or {}).get('p50')),
             show((overall['callback_ms'] or {}).get('p95'))),
            '  避让介入%d轮，完整回调P50/P95为%s/%s毫秒。超期清零%d轮，持续约%s秒%s。' %
            (intervention['cycles'], show((intervention['callback_ms'] or {}).get('p50')),
             show((intervention['callback_ms'] or {}).get('p95')), row['deadline_zero']['cycles'],
             show(row['deadline_zero']['issued_zero_duration_s']),
             '（只有周期起点记录，时间为估算）' if row['deadline_zero']['duration_is_estimate'] else '')]
        delivery = row['remani_delivery']
        if delivery['available']:
            text.append('  REMANI已结束的重规划请求%d次，其中%d次被接受，%d条新参考被MPPI实际使用。' %
                (delivery['completed_refresh_requests'], delivery['accepted_refresh_requests'], delivery['consumed_refresh_references']))
        else:
            text.append('  REMANI重规划明细缺失，不填成零。')
        if row['declared_dynamic_scene'] and not row['dynamic_exposure_valid']:
            text.append('  这一场没有确认真实动态障碍输入，不能据此评价动态避障。')
    text += ['', '怎么看这些结果',
        '一轮计算变快，不等于已经能绕开障碍。要一起看有没有真正执行、有没有到达、有没有接触。',
        '“未介入”分段表示控制器没有进入避让分支，不保证场地里完全没有障碍。各分段和总体耗时完整保存在JSON。',
        '这里的“完整回调”沿用日志callback_ms：包含规划、最终检查及下发前处理，不包含指令下发后的记录和画图，也不是整条感知到执行链路的延迟。',
        '超期清零时间按一次零指令发出后到下一条指令的间隔累加，不是把计算耗时相加。',
        '本轮只有每场一对种子11，用来复现问题和观察变化，不能说明总体成功率，更不能证明实车安全。',
        '这是固定机械臂的ROS数值闭环，不是Gazebo，也没有包含真实雷达、定位和驱动误差。',
        'GIF只画记录里真实的REMANI参考、MPPI候选子集、获准轨迹和机器人投影，没有补画“应该走”的路线。']
    return '\n'.join(text)+'\n'


def report_batch(batch, replay_path, out, max_frames=120, render=True):
    batch, replay_path, out = Path(batch).resolve(), Path(replay_path).resolve(), Path(out).resolve()
    if not (batch/'complete.json').is_file():
        raise ValueError('All timed trials must finish before reporting')
    if out == batch or batch in out.parents:
        raise ValueError('Report must remain outside measured batch')
    plan = json.loads((batch/'plan.json').read_text())
    if plan.get('groups') != list(GROUPS):
        raise ValueError('Expected paired baseline/fast batch, not old original/demand')
    replay = json.loads(replay_path.read_text())
    if replay.get('passed') is not True:
        raise ValueError('Replay did not pass; do not hide failed gate')
    if plan.get('replay_gate_sha256') and sha(replay_path) != plan['replay_gate_sha256']:
        raise ValueError('Replay file differs from actual online gate')
    provenance = dict(replay_gate_sha256=sha(replay_path),
                      libraries=replay.get('libraries'), binary_identity_verified=False)
    if plan.get('build'):
        build_manifest_path = Path(plan['build'])/'build_manifest.json'
        if not build_manifest_path.is_file():
            raise ValueError('Actual build manifest unavailable for replay identity verification')
        build_manifest = json.loads(build_manifest_path.read_text())
        if plan.get('build_manifest_sha256') != sha(build_manifest_path):
            raise ValueError('Build manifest differs from online batch')
        libraries = replay.get('libraries', {})
        if libraries.get('child', {}).get('sha256') != build_manifest.get('mppi_library_sha256'):
            raise ValueError('Replay child library differs from online build')
        if libraries.get('parent', {}).get('sha256') != build_manifest.get('native_dynamic_fast_parent_mppi_sha256'):
            raise ValueError('Replay parent library differs from declared baseline')
        provenance.update(binary_identity_verified=True, build_manifest_sha256=sha(build_manifest_path))
    rows = [summarize_trial(batch/('%s_%s_seed%d' % (scene, group, plan['seed'])), group)
            for scene in plan['scenes'] for group in GROUPS]
    startup = []
    if plan.get('precheck'):
        precheck = Path(plan['precheck'])
        if not (precheck/'complete.json').is_file():
            raise ValueError('Declared empty precheck no longer has complete records')
        precheck_plan = json.loads((precheck/'plan.json').read_text())
        if precheck_plan.get('build_manifest_sha256') != plan.get('build_manifest_sha256'):
            raise ValueError('Empty precheck used a different build')
        startup = [summarize_trial(precheck/('empty_%s_seed%d' % (g, plan['seed'])), g) for g in GROUPS]
    out.mkdir(parents=True, exist_ok=False)
    result = dict(schema='native_dynamic_fast_report_v1', plan=plan, trials=rows,
        replay=dict(path=str(replay_path), sha256=sha(replay_path), record=replay,
                    summary=replay_summary(replay), provenance=provenance), startup_trials=startup,
        superiority_claim=False, statistical_success_rate_claim=False,
        scope='fixed-arm differential-drive ROS numerical SIL; not Gazebo/hardware',
        timing_basis='unprofiled online diagnostics; successful return0 active-reference cycles',
        phase_basis='decision[6] intervention; otherwise moving proposals=monitoring, no moving=freepath; missing=unknown',
        artifacts=[])
    timing_chart(rows, out/'online_comparison.png')
    if render:
        for scene in plan['scenes']:
            if scene == 'empty':
                continue
            result['artifacts'].append(render_pair(
                batch/('%s_baseline_seed%d' % (scene, plan['seed'])),
                batch/('%s_fast_seed%d' % (scene, plan['seed'])), out/'gifs'/scene, max_frames))
    (out/'实验总结.txt').write_text(narrative(result))
    (out/'report.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    readme = ['# 省重复计算：同输入与新闭环对照', '',
        '先看[直白总结](实验总结.txt)和[在线对比图](online_comparison.png)。完整数据见[report.json](report.json)。', '',
        '修改前与修改后都使用已有空旷提速、160个三维碰撞球与相同安全规则。每场一对种子11，仅故障复现。', '',
        '| 场景 | 组别 | 结束原因 | 三维采样接触 | 完整回调P95/ms | 超期清零次数 |',
        '|---|---|---|---|---:|---:|']
    for row in rows:
        readme.append('| %s | %s | %s | %s | %s | %d |' %
            (SCENES.get(row['scene'], row['scene']), row['label'], row['termination'],
             '有' if row['native3d_physical_sample_collision'] else '未检测到',
             show((row['callback_ms'] or {}).get('p95')), row['deadline_zero']['cycles']))
    if render:
        readme += ['', '关键GIF：']+['- [%s](gifs/%s/comparison.gif)' % (SCENES.get(s, s), s)
                                    for s in plan['scenes'] if s != 'empty']
    readme += ['', '仅固定臂ROS数值闭环，非Gazebo或实车。无接触不等于成功，未实际遇到动态障碍不能评价避障。']
    (out/'README.md').write_text('\n'.join(readme)+'\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch', type=Path, required=True)
    parser.add_argument('--replay', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--max-frames', type=int, default=120)
    parser.add_argument('--no-gifs', action='store_true')
    args = parser.parse_args()
    report_batch(args.batch, args.replay, args.out, args.max_frames, not args.no_gifs)


if __name__ == '__main__':
    main()
