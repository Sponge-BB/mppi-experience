#!/usr/bin/env python3
"""Post-run comparison of one recorded self-crop trial and one SIL bypass run.

Only reads completed records. Never starts ROS, replays a controller, or uses
unused CropBox YAML parameters to infer which filter actually ran.
"""
import argparse
from collections import Counter, defaultdict, deque
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from render_dual_timescale import FONT
from render_native_collision_3d import Trial, draw_panel
from report_execution_timing import (measured_stops, summarize_trial,
                                    validated_actions)
from report_native_dynamic_fast import show, timing_by_phase


HERE = Path(__file__).resolve().parent
BASELINE = HERE / 'results/execution_timing_diagnosis_20261028_01/online_probe'
BYPASS = HERE / 'results/sim_crop_bypass_20261029_01/head_on_bypass_seed11'
OUTPUT = HERE / 'results/sim_crop_bypass_report_20261029_01'
LABELS = ('原自车过滤', '仿真旁路自车过滤')


def precontact_pairs(diagnostics, cycles, end):
    """Join the two sequential logs; gate by publication, not solve-start time."""
    pending = defaultdict(deque)
    for diagnostic in diagnostics:
        pending[(round(float(diagnostic['t']), 7), diagnostic['reason'], diagnostic['validation_code'])].append(diagnostic)
    pairs, removed, missing, missing_precontact = [], 0, 0, 0
    for cycle in cycles:
        key = (round(float(cycle['solve_ros_time'])-1000., 7), cycle['reason'], cycle['validation_code'])
        diagnostic = pending[key].popleft() if pending[key] else None
        missing += diagnostic is None
        published = float(cycle['command_publish_after_ros_time'])-1000.
        if published >= end-1e-7:
            removed += 1
            continue
        missing_precontact += diagnostic is None
        pairs.append((diagnostic, cycle, published))
    return pairs, dict(excluded_contact_or_later_rows=removed,
        missing_diagnostics=missing, missing_precontact_diagnostics=missing_precontact,
        unmatched_diagnostics=sum(len(values) for values in pending.values()),
        matched_by='solve timestamp, reason, validation code, ordered within duplicate keys; cycle publication controls cutoff')


def window_metrics(pairs):
    rows = [d for d, _, _ in pairs if d is not None and d.get('reference_points', 0) > 0]
    deadlines = [(d, c) for d, c, _ in pairs if c['reason'] == 'deadline_miss']
    safe_discarded = [(d, c) for d, c in deadlines if c.get('validation_code') == 0
        and d is not None and d.get('validated_proposal') is not None
        and sum(abs(v) for v in d['validated_proposal'][:2]) > 1e-9]
    attempts = [a for _, c, _ in pairs for a in c.get('validation_attempts', [])]
    first_brake = next(((d, c, t) for d, c, t in pairs if c.get('validation_code') == 5), None)
    brake_example = None
    if first_brake is not None:
        diagnostic, cycle, published = first_brake
        values = diagnostic.get('validation', []) if diagnostic else []
        brake_example = dict(cycle=cycle['cycle_id'], command_publish_t=published,
            reason=cycle['reason'], selected_motion_min_gap_m=values[1] if len(values) > 1 else None,
            fallback_braking_min_gap_m=values[2] if len(values) > 2 else None)
    return dict(command_cycles=len(pairs), matched_diagnostics=sum(d is not None for d, _, _ in pairs),
        cycles_with_reference=len(rows), reason_counts=dict(Counter(c['reason'] for _, c, _ in pairs)),
        phase_timing=timing_by_phase(rows), validated_actions=validated_actions(rows),
        validation_attempt_counts=dict(Counter(str(a['code']) for a in attempts)),
        cycles_with_rollout_rejection_code3=sum(any(a['code'] == 3 for a in c.get('validation_attempts', [])) for _, c, _ in pairs),
        cycles_with_braking_rejection_code5=sum(any(a['code'] == 5 for a in c.get('validation_attempts', [])) for _, c, _ in pairs),
        first_final_braking_rejection=brake_example,
        first_no_feasible_command_t=next((t for _, c, t in pairs if c['reason'] == 'no_feasible_candidate'), None),
        final_validation_code_counts=dict(Counter(str(c.get('validation_code')) for _, c, _ in pairs)),
        deadline=dict(label_count=len(deadlines),
            validated_nonzero_proposals_cleared=len(safe_discarded),
            already_failed_validation=sum(c.get('validation_code', -1) > 0 for _, c in deadlines),
            no_successful_validation_or_no_nonzero_proposal=len(deadlines)-len(safe_discarded),
            final_validation_code_counts=dict(Counter(str(c.get('validation_code')) for _, c in deadlines))),
        semantics='code 3: selected-motion clearance/recovery rejected; code 5: fallback braking rejected; candidate attempts are not cycle counts')


def summarize(source, label):
    # Reuse native truth and measured-stop definitions; replace diagnostics-based
    # statistics, because the historical helper includes rows published at contact.
    result = summarize_trial(source, 'matched')
    trial = Trial(source, 'native3d')
    cycles = trial.reader.jsonl('state_mppi_cycles.jsonl')
    pairs, alignment = precontact_pairs(trial.diagnostics, cycles, trial.end)
    goal = float(trial.result.get('goal_sent', trial.anchor))
    actor = trial.result.get('native_realtime_evidence', {}).get('first_truth_actor_t')
    active_start = float(actor) if actor is not None else trial.anchor
    after_goal = [pair for pair in pairs if pair[2] >= goal]
    active = [pair for pair in pairs if pair[2] >= active_start]
    gaps = [dict(cycle=a[1]['cycle_id'], next_cycle=b[1]['cycle_id'],
                 gap_s=float(b[1]['cycle_start_monotonic'])-float(a[1]['command_publish_after_monotonic']),
                 previous_publish_t=a[2], next_publish_t=b[2], next_reason=b[1]['reason'])
            for a, b in zip(after_goal, after_goal[1:])]
    launch_path = trial.reader.locate('experiment.launch', required=True)
    launch = ET.parse(launch_path).getroot()
    node_names = [node.get('name') for node in launch.findall('node')]
    passthrough = next(node for node in launch.findall('node') if node.get('name') == 'passthrough_z')
    output = next(remap.get('to') for remap in passthrough.findall('remap') if remap.get('from') == '~output')
    result.update(group='bypass' if label == LABELS[1] else 'baseline', label=label,
        diagnostic_statistics_scope='Only command publications strictly before recorded termination/contact; contact-time and later commands excluded',
        excluded_contact_or_later_rows=alignment['excluded_contact_or_later_rows'],
        diagnostic_alignment=alignment, after_goal=window_metrics(after_goal),
        actor_window=window_metrics(active), largest_postpublish_to_next_start_gap=max(gaps, key=lambda r: r['gap_s']) if gaps else None,
        measured_stop=measured_stops(trial.state, active_start, trial.end),
        actual_filter=dict(crop_node_present='crop_box_filter' in node_names,
            passthrough_output=output, manifest_adjustment=trial.manifest.get('simulation_perception_adjustment'),
            inferred_from='recorded launch and manifest, not retained inactive YAML parameters'))
    # Remove the helper's unfiltered versions instead of exposing contradictory totals.
    for key in ('phase_timing', 'validated_actions', 'deadline_miss_cycles', 'reason_counts', 'deadline_zero'):
        result.pop(key, None)
    current_refs = [r for r in trial.refs if r['t'] < trial.end-1e-7]
    prior = [r for r in current_refs if r['t'] <= active_start]
    initial = prior[-1] if prior else None
    refreshed = [r for r in current_refs if r['t'] > active_start
                 and (initial is None or r.get('sha256') != initial.get('sha256'))]
    result['observed_reference_updates'] = dict(consumed_references=len(current_refs),
        after_actor_new_reference_count=len(refreshed),
        first_after_actor_new_reference_t=min((float(r['t']) for r in refreshed), default=None),
        no_new_reference_is_not_proof_optimizer_never_ran=True)
    analysis = trial.reader.json('analysis/summary.json', {})
    refresh = [r for r in analysis.get('requests', []) if r.get('request_id', 0) > 1
               and r.get('start_sim_s', math.inf) < trial.end-1e-7]
    result['remani_refresh_requests'] = dict(count=len(refresh),
        outcomes=dict(Counter(r.get('outcome_class', 'unknown') for r in refresh)),
        optimizer_calls=sum(len(r.get('optimizer_calls', [])) for r in refresh),
        requests=[dict(request_id=r['request_id'], start_sim_s=r.get('start_sim_s'),
            outcome=r.get('outcome_class'), optimizer_calls=len(r.get('optimizer_calls', []))) for r in refresh])
    trial.verify_unchanged()
    result['input_sha256'].update(trial.reader.hashes)
    return result, trial


def render(trials, out, max_frames=120):
    duration = max(trial.end-trial.anchor for trial in trials)
    ages = np.linspace(0., duration, min(max_frames, max(2, math.ceil(duration/.3)+1)))
    frame_ms = max(20, 10*round(1000*float(ages[1]-ages[0])/2./10))
    font, small = ImageFont.truetype(FONT, 18), ImageFont.truetype(FONT, 14)
    frames = []
    for age in ages:
        image = Image.new('RGB', (1160, 482), 'white')
        draw = ImageDraw.Draw(image)
        draw.text((16, 8), '迎面场景　%.1f s　起步对齐' % age, font=font, fill='#333333')
        draw.text((16, 34), '橙：REMANI参考　蓝：实际整机投影　灰：真实MPPI搜索子集　绿：获准展开', font=small, fill='#555555')
        for index, trial in enumerate(trials):
            t = min(trial.anchor+age, trial.end)
            draw.text((index*580+16, 64), LABELS[index], font=font, fill='#333333')
            row = draw_panel(image, trial, t, (index*580+16, 98, (index+1)*580-16, 404))
            draw = ImageDraw.Draw(image)
            draw.text((index*580+16, 414), '%s　v=%+.2f m/s' % (trial.status_at(t), row['v']), font=small, fill='#555555')
        draw.text((16, 452), '固定臂差速ROS数值闭环，非Gazebo/实车；历史单次对照，仅定位机制，不代表成功率。', font=small, fill='#777777')
        frames.append(image.convert('P', palette=Image.ADAPTIVE, colors=96))
    frames[0].save(out/'comparison.gif', save_all=True, append_images=frames[1:],
        duration=[frame_ms]*(len(frames)-1)+[max(500, frame_ms)], loop=0, disposal=2, optimize=True)
    for trial in trials:
        trial.verify_unchanged()
    return dict(frames=len(frames), alignment='each recorded actual motion anchor; freeze at actual termination',
        actual_robot_projection=True, actual_remani_reference=True,
        actual_recorded_mppi_search_subset=True, no_controller_reexecution=True)


def narrative(result):
    new = result['trials'][1]
    text = ['先说结论',
        '旁路自车误删后，本次迎面测试仍发生三维采样接触，说明仅修这层过滤还不够，不能宣称避障已经成功。',
        '新组没有出现“通过验收的非零动作因超期被清零”；剩余记录先出现后备刹停拒绝，随后出现找不到可行候选，不能简单归因为200毫秒不够。',
        '新组REMANI的%d次重规划请求都在选局部目标时被拒绝，尚未进入优化器；期间没有交付新参考。' % new['remani_refresh_requests']['count'],
        '', '本轮改了什么',
        '只在新数值仿真中取消自车CropBox，让高度过滤后的环境点直接进入感知。仿真没有机器人自身回波，因此不需要这层删点。',
        '实车代码不动；仍为完整REMANI＋MPPI、固定机械臂、200/250毫秒、seed11，保留分段计时。', '', '结果']
    for row in result['trials']:
        active = row['actor_window']
        deadline = active['deadline']
        timing = active['phase_timing']['overall']['callback_ms'] or {}
        gap = row['largest_postpublish_to_next_start_gap']
        attempts = active['validation_attempt_counts']
        text.extend([row['label'],
            '结束：%s；三维采样物理接触：%s；最小物理采样间隙%s米。' %
            (row['termination'], '有' if row['native3d_physical_sample_collision'] else '无', show(row['minimum_recorded_3d_sample_gap_m'], 4)),
            '障碍出现后实际累计停车%s秒，最长连续停车%s秒。' %
            (show(row['measured_stop']['total_s']), show(row['measured_stop']['longest_s'])),
            '障碍出现后求解及验收段P95为%s毫秒；超期标签%d次，其中通过验收的非零动作被超期清零%d次，已验收失败%d次。' %
            (show(timing.get('p95')), deadline['label_count'], deadline['validated_nonzero_proposals_cleared'], deadline['already_failed_validation']),
            '候选验收：展开检查拒绝(code3)%d次，后备刹停拒绝(code5)%d次；这是候选次数，不是停车次数。' %
            (attempts.get('3', 0), attempts.get('5', 0)),
            '出现过后备刹停拒绝的控制轮数%d轮；找不到可行候选%d轮，首次在实验时间%s秒。' %
            (active['cycles_with_braking_rejection_code5'], active['reason_counts'].get('no_feasible_candidate', 0),
             show(active['first_no_feasible_command_t'])),
            '障碍出现后收到新REMANI参考%d条；发目标后两轮之间最大空档%s毫秒。' %
            (row['observed_reference_updates']['after_actor_new_reference_count'], show(1000*gap['gap_s'] if gap else None)),
            '已剔除接触/结束时刻及之后发布的%d条控制记录。' % row['excluded_contact_or_later_rows'], ''])
    endpoint = result.get('endpoint_perception_audit', {})
    if endpoint:
        snapshot = endpoint['recorded_snapshot']
        text += ['终点再核对感知',
            '新组终点用最后记录的动态快照复核，整机最小硬检查间隙为%s米，已经是负值，说明这一时刻感知模型已经看到了危险。' %
            show(snapshot['minimum_dynamic_hard_gap_m'], 4),
            '因此不能把本次最终失败继续说成“障碍完全没看见”。但点云仍经过量程、体素与输出抽样，不代表所有感知外形问题都已消除。',
            '复核来源：perception_endpoint_audit.json，仅检查记录终点，不反推较早时刻每轮候选是否可行。', '']
    text += ['怎样看',
        '本轮首先看取消误删后能否正常避让，再看剩余停车是候选不可行、后备刹停拒绝，还是通过验收后被超期清零，不能混为同一种“算得慢”。',
        '求解及验收段耗时不包含两轮之间的全部操作；最大空档单独统计。实际停车按测量速度统计，不把零指令时间当成停车时间。',
        '旧组是之前完成的单次记录，新组是本次重新运行的在线数值闭环，不是同输入回放，也不是同期重复统计。',
        '仅凭这两次不能声称成功率提升，或证明点云误删是碰撞唯一原因；未再出现长空档也不等于已修复。',
        'GIF显示真实机器人外形投影、REMANI参考与记录下来的MPPI搜索子集，投影重叠不代替三维接触判断。']
    return '\n'.join(text)+'\n'


def report(baseline=BASELINE, bypass=BYPASS, out=OUTPUT, make_gif=True):
    rows, trials = zip(*(summarize(path, label) for path, label in zip((baseline, bypass), LABELS)))
    if rows[0]['actual_filter']['crop_node_present'] is not True or rows[1]['actual_filter']['crop_node_present'] is not False:
        raise ValueError('Recorded launches do not show crop-versus-bypass comparison')
    if rows[1]['actual_filter']['passthrough_output'] != '/velodyne_points':
        raise ValueError('Bypass launch does not route height-filtered cloud to the planner input')
    if trials[0].manifest['scene'] != trials[1].manifest['scene'] or trials[0].parameters['seed'] != trials[1].parameters['seed']:
        raise ValueError('Scene and seed must match')
    if not np.array_equal(trials[0].body3d, trials[1].body3d):
        raise ValueError('Recorded robot collision geometry differs')
    out = Path(out).resolve()
    if any(out == trial.source or trial.source in out.parents for trial in trials):
        raise ValueError('Report output must stay outside the measured trial directories')
    out.mkdir(parents=True, exist_ok=True)
    result = dict(trials=rows, changed_factor='simulation self-crop bypass only',
        scope='fixed-arm complete REMANI+MPPI ROS numerical SIL, not Gazebo or hardware',
        historical_single_trial_comparison=True, statistical_success_rate_claim=False)
    endpoint = out/'perception_endpoint_audit.json'
    if endpoint.is_file():
        audit = json.loads(endpoint.read_text())
        if Path(audit['trial']).resolve() != Path(bypass).resolve():
            raise ValueError('Endpoint audit belongs to a different bypass trial')
        result['endpoint_perception_audit'] = audit
    if make_gif:
        result['visualization'] = render(trials, out)
    (out/'report.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    (out/'实验总结.txt').write_text(narrative(result))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, default=BASELINE)
    parser.add_argument('--bypass', type=Path, default=BYPASS)
    parser.add_argument('--out', type=Path, default=OUTPUT)
    parser.add_argument('--no-gif', action='store_true')
    args = parser.parse_args()
    value = report(args.baseline, args.bypass, args.out, not args.no_gif)
    print(json.dumps(dict(output=str(args.out.resolve()),
        termination=[row['termination'] for row in value['trials']]), ensure_ascii=False))


if __name__ == '__main__':
    main()
