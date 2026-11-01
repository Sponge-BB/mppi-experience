#!/usr/bin/env python3
"""Report one completed collision-repair A/B; no controller or optimizer replay."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

from render_dual_timescale import FONT
from report_delivery_feedback import load_trial as load_feedback_trial
from report_frontend_rejoin import keyframes
from report_static_agility import gif, show
from report_time_indexed_collision import OUTCOMES, TERMINATIONS
from replay_temporal_collision import Candidate, digest, seconds, temporal_gap


HERE = Path(__file__).resolve().parent
DEFAULT = HERE/'results/candidate_collision_repair_20261031_01'


def dynamic_check(candidate, plan, scale):
    """Same frozen CV/body and candidate clock, matching .01 s hard samples.

    This diagnoses the dynamic branch only. Its positive gap is not the full
    native static/self/wheel acceptance result.
    """
    overlay = plan['overlay']
    snapshot = overlay.get('source_snapshot')
    if not snapshot or not overlay.get('source_exact'):
        return dict(execution_scale=scale, available=False,
                    reason='exact_frozen_CV_source_missing')
    # A rejected final trajectory may retain start_epoch=0 because it has not
    # been assigned a publication timestamp. The native optimizer hard check
    # uses the request's frozen desired start epoch for before/after alike.
    epoch = plan['requested_epoch']
    end = min(candidate.duration, max(0., seconds(snapshot)+overlay['maximum_prediction_s']-epoch)*scale)
    samples = []
    skipped_history, skipped_future = 0, 0
    local_t = .05
    for unused in range(5, int(math.floor(candidate.duration/.01))):
        query = epoch+local_t/scale
        if query < seconds(snapshot)-1e-6:
            skipped_history += 1
        elif query > seconds(snapshot)+overlay['maximum_prediction_s']+1e-6:
            skipped_future += 1
        else:
            samples.append(temporal_gap(candidate, local_t, query, snapshot, overlay, 'native_cv_uncertainty'))
        local_t += .01
    valid = [r for r in samples if r.get('query_valid')]
    finite = [r for r in valid if r.get('minimum_model_gap_m') is not None]
    return dict(execution_scale=scale, available=True, epoch=epoch,
        epoch_source='planning.requested_epoch (native hard-check clock)',
        captured_candidate_start_epoch=candidate.record.get('start_epoch'),
        source_sha256=digest(snapshot), candidate_sha256=candidate.geometry_hash(),
        hard_sample_step_s=.01, hard_sample_begin_s=.05,
        evaluated_local_end_s=end, valid_sample_count=len(valid),
        skipped_history_count=skipped_history, skipped_future_count=skipped_future,
        invalid_sample_count=len(samples)-len(valid),
        minimum=min(finite, key=lambda r: r['minimum_model_gap_m']) if finite else None,
        first_collision=next((r for r in valid if r.get('model_collision')), None),
        scope='Unchanged candidate and same frozen CV. Dynamic hard sphere margin only, not full native acceptance.')


def load_trial(source, label):
    row, trial = load_feedback_trial(source, label)
    timeline = trial.reader.jsonl('dual_timeline.jsonl')
    captures = trial.reader.jsonl('temporal_collision.jsonl')
    plans = {r['plan_ordinal']: r for r in captures if r['stage'] == 'planning'}
    events = [r for r in timeline if 'repair' in r.get('event', '')]
    row['repair_events'] = events
    row['repair_event_counts'] = dict(Counter(r['event'] for r in events))
    row['repair_summary'] = dict(
        diagnosed_collision_count=sum(r['event'] == 'candidate_collision_repair_diagnosis' for r in events),
        triggered_count=sum(r['event'] == 'candidate_collision_repair_begin' for r in events),
        full_native_pass_count=sum(r['event'] == 'candidate_collision_repair_end' and r.get('native_pass') is True for r in events),
        repair_end_count=sum(r['event'] == 'candidate_collision_repair_end' for r in events),
        original_budget_reset_count=sum(r['event'] == 'candidate_collision_repair_end' and
            abs(r['original_budget_start_s']-r['final_budget_start_s']) > 1e-9 for r in events))
    row['repair_candidate_checks'] = []
    for capture in captures:
        stage = capture.get('stage', '')
        if 'repair' not in stage and stage != 'final_candidate':
            continue
        plan = plans.get(capture['plan_ordinal'])
        if not plan or not capture.get('candidate'):
            continue
        candidate = Candidate(capture['candidate'], plan)
        row['repair_candidate_checks'].append(dict(stage=stage,
            request_id=capture['request_id'], plan_ordinal=capture['plan_ordinal'],
            native_success=capture.get('native_success'), native_reason=capture.get('native_reason'),
            capture_metadata={k: v for k, v in capture.items()
                              if k not in ('candidate', 'overlay')},
            checks=[dynamic_check(candidate, plan, scale) for scale in (1., .5)]))
    row['repair_settings'] = {k: v for k, v in trial.manifest.items()
                            if 'repair' in k or k == 'delivery_feedback'}
    row['solver_end_records'] = [r for r in trial.reader.jsonl('solver_trace.jsonl')
                                if r.get('event') == 'end' and r.get('stage') == 'optimizer']
    row['input_sha256'] = trial.reader.hashes
    trial.verify_unchanged()
    return row, trial


def gap_text(check):
    value = check.get('minimum')
    return show(100*value['minimum_model_gap_m'], 2)+'厘米' if value else '无有效点对'


def compact_event(event):
    # Keep the actual recorded field names rather than infer an unlogged gate.
    return json.dumps({k: v for k, v in event.items()
                       if k not in ('sim_time', 'mono', 'wall_time', 'thread')}, ensure_ascii=False)


def narrative(rows):
    lines = ['候选碰撞修复：迎面场景A/B实验结果', '',
        '核心问题：优化停下后候选仍碰撞，能否在同次请求剩余预算内修好并更早交付。',
        'A保持上一轮开启交付状态软反馈的版本；B在最终碰撞检查失败后，从当前候选继续修复一次。',
        '两组总优化预算均为1.2秒，碰撞边界与验收不放宽。每组只跑一次，同次记录失败、求解、交付和运动。', '']
    for row in rows:
        cmd = row['control_delivery']['first_refresh_to_new_reference_validated_command_wall_ms']
        lines += [row['label'],
            '结果：%s；%s物理球采样接触，最小间隙%s厘米；实走%s米。' % (
                '到达目标' if row['reached_goal'] else TERMINATIONS.get(row['termination'], row['termination']),
                '发生' if row['physical_sample_collision'] else '未发现',
                show(100*row['minimum_physical_sample_gap_m'], 2), show(row['actual_path_length_m'], 3)),
            '首次重规划请求到新参考首条有效控制：%s；交付成功那次请求：%s。' % (
                show(cmd/1000, 3)+'秒' if cmd is not None else '未交付',
                show(row['delivery']['first_adopted_request_wall_ms']/1000, 3)+'秒'
                if row['delivery']['first_adopted_request_wall_ms'] is not None else '未交付'),
            '非目标低速停留%s秒；MPPI完整循环P95/最大%s/%s毫秒，超过200毫秒%d次。' % (
                show(row['non_goal_low_speed']['total_s'], 3), show(row['callback']['p95_ms'], 1),
                show(row['callback']['max_ms'], 1), row['callback']['over_200_ms'])]
        for request in row['remani']['requests']:
            if request['request_id'] <= 1:
                continue
            value = request.get('request_wall_ms')
            lines.append('请求%d：%s，耗时%s秒；优化/硬检查原因%s；发布拒绝%s。' % (
                request['request_id'], OUTCOMES.get(request.get('outcome_class'), request.get('outcome_class')),
                show(value/1000, 3) if value is not None else '未结束',
                [c.get('reason') for c in request.get('optimizer_calls', [])], request.get('acceptance_rejections', [])))
        lines.append('修复事件计数：'+json.dumps(row['repair_event_counts'], ensure_ascii=False))
        for event in row['repair_events']:
            if event['event'] == 'candidate_collision_repair_diagnosis':
                lines.append('请求%d碰撞诊断：原求解器返回%s，碰撞位置t=%s秒，目标曲线段%d（从1计数）；已用%s毫秒，剩余%s毫秒；完整决策梯度范数%s。' % (
                    event['request_id'], event.get('original_lbfgs_result'),
                    show(event.get('original_rejection_time'), 3), event.get('target_piece', -1)+1,
                    show(1000*event['elapsed_s'], 1), show(event.get('budget_remaining_ms'), 1),
                    show(event.get('objective_gradient_norm'), 4)))
            elif event['event'] == 'candidate_collision_repair_end':
                lines.append('请求%d修复：完整硬验收%s；停止原因%s，求解器返回%s，最终拒绝位置t=%s秒；额外求解%s毫秒，修复含检查%s毫秒；原预算起点%s。' % (
                    event['request_id'], '通过' if event.get('native_pass') else '未通过',
                    event.get('repair_reason'), event.get('repair_lbfgs_result'),
                    show(event.get('final_rejection_time'), 3),
                    show(event.get('repair_solver_wall_ms'), 1), show(event.get('repair_wall_ms'), 1),
                    '保持' if event.get('original_budget_start_s') == event.get('final_budget_start_s') else '发生变化')))
        for capture in row['repair_candidate_checks']:
            if 'repair' not in capture['stage']:
                continue
            lines.append('请求%d %s：同冻结CV复算，常速最小间隙%s，半速%s；原生结果%s/%s。' % (
                capture['request_id'], capture['stage'], gap_text(capture['checks'][0]),
                gap_text(capture['checks'][1]), capture['native_success'], capture['native_reason']))
        lines.append('')
    lines += ['如何判断',
        '先看B是否真正触发修复、修复后是否通过完整硬验收，再看首次请求到实际有效控制是否缩短。',
        '同冻结CV间隙用于对比修复前后；常速与半速都列出。它不替代静态、自碰撞、轮速和最新发布验收。',
        '修复时失败段环境碰撞软权重乘10，前后加权软代价不属于同一目标，不能单凭代价升降判断改善。',
        '没有新轨迹交付时，耗时写“未交付”，不按0统计；全部失败与终止保留。',
        '同场景同种子在线运行仍会有调度与输入时序差异，因此这一组定位机制，不估计总体成功率。',
        '固定机械臂、完整REMANI与底盘MPPI的ROS数值闭环，非Gazebo或实车。',
        'GIF保留完整碰撞球投影；绿色为获准局部展开，不是全部MPPI搜索。详细修复事件与停止原因见summary.json。']
    return '\n'.join(lines)+'\n'


def chart(rows, output):
    image = Image.new('RGB', (1240, 580), 'white')
    draw = ImageDraw.Draw(image)
    title, text, small = [ImageFont.truetype(FONT, n) for n in (23, 18, 15)]
    colors = ('#2874a6', '#c47723')
    draw.text((20, 15), '候选碰撞修复：相同预算下能否更早交付', font=title, fill='#333333')
    draw.text((20, 53), '同一迎面场景A/B各一次；首请求到有效新控制包含失败和重试。', font=small, fill='#666666')
    for index, row in enumerate(rows):
        top = 98+205*index
        draw.line((20, top-10, 1220, top-10), fill='#dddddd')
        draw.text((20, top), row['label'], font=text, fill=colors[index])
        value = row['control_delivery']['first_refresh_to_new_reference_validated_command_wall_ms']
        result = '到达目标' if row['reached_goal'] else TERMINATIONS.get(row['termination'], row['termination'])
        draw.text((20, top+34), '首次请求→有效新控制：%s；%s' % (
            show(value/1000, 3)+' s' if value is not None else '未交付', result), font=text, fill='#444444')
        requests = [r for r in row['remani']['requests'] if r['request_id'] > 1]
        outcomes = Counter(r.get('outcome_class') for r in requests)
        draw.text((20, top+68), '重规划结果：'+'；'.join('%s%d次' % (OUTCOMES.get(k, k), v)
            for k, v in outcomes.items()), font=small, fill='#555555')
        draw.text((20, top+99), '实走 %.3f m；最小物理间隙 %.2f cm；非目标低速 %.3f s' % (
            row['actual_path_length_m'], 100*row['minimum_physical_sample_gap_m'],
            row['non_goal_low_speed']['total_s']), font=small, fill='#555555')
        draw.text((20, top+130), 'MPPI完整循环P95/最大 %s/%s ms；超过200 ms：%d次' % (
            show(row['callback']['p95_ms'], 1), show(row['callback']['max_ms'], 1),
            row['callback']['over_200_ms']), font=small, fill='#555555')
        draw.text((20, top+159), '最终碰撞诊断 %d次；修复触发 %d次；修复后完整硬验收通过 %d次' % (
            row['repair_summary']['diagnosed_collision_count'], row['repair_summary']['triggered_count'],
            row['repair_summary']['full_native_pass_count']), font=small, fill='#555555')
    draw.text((20, 536), '完整硬验收、最新CV验收和真实采用分别记录；同冻结CV的间隙不是物理接触结果。', font=small, fill='#666666')
    draw.text((20, 562), '固定臂ROS数值闭环；完整保留失败；单次对照不代表总体成功率。', font=small, fill='#777777')
    image.save(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=DEFAULT)
    parser.add_argument('--baseline', type=Path)
    parser.add_argument('--changed', type=Path)
    parser.add_argument('--out', type=Path)
    parser.add_argument('--max-frames', type=int, default=100)
    args = parser.parse_args()
    pairs = [load_trial(args.baseline or args.root/'headon_A', 'A：当前版本，碰撞后不修复'),
             load_trial(args.changed or args.root/'headon_B', 'B：同请求剩余预算内修复一次')]
    rows, trials = zip(*pairs)
    output = args.out or args.root/'report'
    output.mkdir(parents=True, exist_ok=True)
    chart(rows, output/'碰撞修复与交付对照.png')
    animation = output/'迎面动态对照.gif'
    artifacts = gif([dict(row, termination=TERMINATIONS.get(row['termination'], row['termination']))
                     for row in rows], trials, animation, args.max_frames)
    keyframes(animation, output/'迎面动态关键帧.png')
    (output/'实验结果.txt').write_text(narrative(rows), encoding='utf8')
    (output/'summary.json').write_text(json.dumps(dict(rows=rows, artifacts=artifacts,
        scope='One recorded fixed-arm numerical ROS SIL head-on A/B; no controller or optimizer replay.',
        dynamic_gap_scope='Same polynomial, frozen CV and 67 native spheres, .01 s samples from .05 s; dynamic branch only.'),
        ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    for trial in trials:
        trial.verify_unchanged()
    print('CANDIDATE_COLLISION_REPAIR_REPORT', output, flush=True)


if __name__ == '__main__':
    main()
