#!/usr/bin/env python3
"""Read first-feasible/selected checkpoints and the completed online pair.

No native objective or solver is replayed. The strongest comparison is the
first-safe and selected accepted incumbents from one actual B solve, using
recorded full original-objective snapshots. Asynchronous A/B is kept separate.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from render_coupled_objective import sha
from render_dual_timescale import FONT
from replay_temporal_collision import digest
from report_fast_refresh_feedback import animation
from report_frontend_rejoin import keyframes
from report_half_speed_contract import add_common_motion_window
from report_prefix_handoff import curve_summary, load_online as load_prefix_online, normalized_geometry
from report_static_agility import show
from report_static_width_diagnosis import sample_curve
from report_time_indexed_collision import OUTCOMES, TERMINATIONS


HERE = Path(__file__).resolve().parent
DEFAULT = HERE/'results/feasible_quality_20261101_01'
LABELS = ('A：首个可行候选即返回', 'B：原预算内继续优化')
OBJECTIVE_FIELDS = ('smooth', 'base_obstacle', 'arm_obstacle', 'self_collision',
    'base_feasibility', 'arm_feasibility', 'other_integral', 'time', 'geometry', 'prediction')


def read_evidence(root):
    path = root/'replay/初值与返回曲线核对.json'
    return dict(path=str(path.resolve()), sha256=sha(path), records=json.loads(path.read_text())) if path.is_file() else None


def objective_consistency(snapshot, reported_cost):
    if not snapshot or not snapshot.get('valid'):
        return dict(available=False)
    total = sum(snapshot[name] for name in OBJECTIVE_FIELDS)
    return dict(available=True, snapshot_total_matches_reported_cost=snapshot['total'] == reported_cost,
        recorded_component_sum=total, component_sum_minus_recorded_total=total-snapshot['total'],
        scope='Arithmetic check of recorded native components, not map-objective or gradient replay.')


def checkpoint_geometry(candidate, record, plan, head, target, stage):
    if candidate is None:
        return None
    wrapper = dict(stage=stage, success=True, capture_mono=record['capture_mono'],
        capture_sim=record['capture_sim'], resource_replan_id=record['plan_ordinal'], parts=candidate['parts'])
    value = curve_summary(wrapper, head, target)
    points = sample_curve(wrapper)
    value.update(world_y_range_m=[float(np.min(points[:, 1])), float(np.max(points[:, 1]))],
        maximum_absolute_world_y_m=float(np.max(np.abs(points[:, 1]))),
        candidate_epoch=candidate.get('start_epoch'), planning_requested_epoch=plan['requested_epoch'],
        candidate_semantics='Independently rebuilt actual accepted native-feasible incumbent at its recorded checkpoint; publication/latest acceptance and execution are separate.')
    return value


def quality_analysis(row, trial):
    captures = trial.reader.jsonl('temporal_collision.jsonl')
    timeline = trial.reader.jsonl('dual_timeline.jsonl')
    plans = {r['plan_ordinal']: r for r in captures if r.get('stage') == 'planning'}
    requests = {r['request_id']: r for r in timeline if r.get('event') == 'request_begin'}
    publications = [r for r in timeline if r.get('event') == 'reference_publish']
    finals = [r for r in captures if r.get('stage') == 'final_candidate']
    checkpoints = []
    for record in captures:
        if record.get('stage') != 'feasible_quality_polish' or record.get('request_id', 0) <= 1:
            continue
        plan = plans[record['plan_ordinal']]
        request = requests[record['request_id']]
        head = dict(x=request['start_x'], y=request['start_y'], yaw=request['start_yaw'])
        feedback = [f for f in row['feedback']['scopes'] if f['scope']['request_id'] == record['request_id']
                    and f.get('target')]
        target = feedback[0]['target'].get('predicted') if len(feedback) == 1 else None
        first = checkpoint_geometry(record.get('first_feasible_candidate'), record, plan, head, target, 'first_native_feasible')
        selected = checkpoint_geometry(record.get('selected_candidate'), record, plan, head, target, 'selected_native_feasible')
        native_matches = [r for r in finals if r['plan_ordinal'] == record['plan_ordinal'] and selected
            and digest(normalized_geometry(r['candidate']['parts'])) == selected['geometry_sha256']]
        published = [r for r in publications if r['request_id'] == record['request_id']]
        consumed = [r for r in trial.refs if any(abs(r['stamp']-p['guide_start']) < 1e-7 for p in published)]
        consumed_matches = []
        if selected:
            wrapper = dict(stage='selected', parts=selected['full_polynomial_parts'])
            for ref in consumed:
                predicted = sample_curve(wrapper, ref['times'])
                measured = np.asarray(ref['reference'], dtype=float)[:, :2]
                consumed_matches.append(dict(revision=ref['revision'], reference_sha256=ref.get('sha256'),
                    maximum_xy_polynomial_error_m=float(np.max(np.linalg.norm(predicted-measured, axis=1)))))
        initial_matches = [a['initial'] for a in row['prefix_handoff']['attempts']
            if a['request_id'] == record['request_id']]
        first_cost, best_cost = record.get('first_cost'), record.get('best_cost')
        checkpoints.append(dict(request_id=record['request_id'], plan_ordinal=record['plan_ordinal'],
            polish_enabled=record['polish_enabled'], safe_checkpoint_available=record['safe_checkpoint_available'],
            original_budget_s=record.get('original_budget_s'),
            first_feasible_elapsed_s=record.get('first_feasible_elapsed_s'),
            selected_elapsed_s=record.get('selected_elapsed_s'),
            first_feasible_iteration=record.get('first_feasible_iteration'), selected_iteration=record.get('selected_iteration'),
            remaining_at_first_feasible_s=record['original_budget_s']-record['first_feasible_elapsed_s']
                if record.get('original_budget_s') is not None and record.get('first_feasible_elapsed_s') is not None else None,
            solver_result=record.get('solver_result'), improvements=record.get('improvements'),
            native_checks=record.get('native_checks'), first_cost=first_cost, best_cost=best_cost,
            cost_decrease=first_cost-best_cost if first_cost is not None and best_cost is not None else None,
            initial_objective=record.get('initial_objective'),
            first_feasible_objective=record.get('first_feasible_objective'), selected_objective=record.get('selected_objective'),
            first_objective_consistency=objective_consistency(record.get('first_feasible_objective'), first_cost),
            selected_objective_consistency=objective_consistency(record.get('selected_objective'), best_cost),
            initial_geometry=initial_matches[0] if len(initial_matches) == 1 else None,
            first_feasible_geometry=first, selected_geometry=selected,
            selected_matches_final_native_candidates=[dict(native_success=r.get('native_success'),
                native_reason=r.get('native_reason')) for r in native_matches],
            reference_publications=len(published), consumed_reference_matches=consumed_matches,
            native_to_latest_rejection=[r for r in row['latest_rejections'] if r['request_id'] == record['request_id']],
            raw_record=record,
            scope='Within one recorded solve, checkpoints use its unchanged original objective and deadline. Native-certified incumbent is not automatically latest-CV acceptable or actually consumed.'))
    return dict(checkpoints=checkpoints, captured_refresh_solve_count=len(checkpoints),
        native_checkpoint_solve_count=sum(r['safe_checkpoint_available'] for r in checkpoints),
        solve_count_with_later_cost_improvement=sum((r.get('improvements') or 0) > 0 for r in checkpoints),
        total_later_checked_improvements=sum(r.get('improvements') or 0 for r in checkpoints),
        first_refresh_checkpoint=next((r for r in checkpoints if r['safe_checkpoint_available']), None),
        selection='First chronological refresh solve with a native-feasible checkpoint, before inspecting its improvement or publication outcome. All refresh records retained.')


def load_online(source, label):
    row, trial = load_prefix_online(source, label)
    row['quality'] = quality_analysis(row, trial)
    row['quality_settings'] = {k: v for k, v in trial.manifest.items() if 'quality' in k}
    row['source_sha256'] = trial.reader.hashes
    trial.verify_unchanged()
    return row, trial


def conclusions(rows):
    b = rows[1]
    item = b['quality']['first_refresh_checkpoint']
    notes = []
    if item is None:
        notes.append('B没有记录到可行检查点，未能检验“首次可行之后继续改善”的效果。')
    elif not (item.get('improvements') or 0):
        notes.append('B首次可行之后，没有保存到成本更低且原生检查通过的新候选；本次没有取得后续质量改善。')
    else:
        first, selected = item['first_feasible_geometry'], item['selected_geometry']
        notes.append('同一B求解中，首次可行→最终选择：原目标值%s→%s，后续检查通过并改善%d次。' % (
            show(item['first_cost'], 3), show(item['best_cost'], 3), item['improvements']))
        notes.append('同次候选几何长度%s→%s米，曲线时长%s→%s秒，最大|world y|%s→%s米。' % (
            show(first['full_curve_path_length_m'], 3), show(selected['full_curve_path_length_m'], 3),
            show(first['duration_s'], 3), show(selected['duration_s'], 3),
            show(first['maximum_absolute_world_y_m'], 3), show(selected['maximum_absolute_world_y_m'], 3)))
        if selected['full_curve_path_length_m'] >= first['full_curve_path_length_m'] or selected['duration_s'] >= first['duration_s']:
            notes.append('原总代价降低，没有同时保证路线和时长都缩短；不能把成本改善直接写成绕远、慢行已经解决。')
    if item and item.get('remaining_at_first_feasible_s') is not None:
        notes.append('B首次原生可行时，原预算还剩%s毫秒；没有重置或额外增加求解预算。' % show(1000*item['remaining_at_first_feasible_s'], 1))
    totals = [r['stage_timeline']['hint_send_to_first_new_validated_command_wall_s'] for r in rows]
    notes.append('首次获准介入反馈→实际有效新控制：A %s，B %s。' % tuple(
        show(value, 3)+'秒' if value is not None else '未交付' for value in totals))
    for row in rows:
        notes.append(row['label']+'：'+('到达目标' if row['reached_goal'] else TERMINATIONS.get(row['termination'], str(row['termination'])))+
            '，'+('检测到' if row['physical_sample_collision'] else '未发现')+'物理球采样接触。')
    notes.append('B同次求解检查点比较用于判断继续优化的直接效果；A/B在线输入时序可能不同，不能称为同输入配对或总体成功率。')
    return notes


def narrative(data):
    lines = ['首个可行候选后继续优化：实验结果', '',
        '核心问题：上轮短前缀使新轨迹更早交付，但首个可行候选就退出，轨迹仍过宽、过长、偏慢。',
        'A遇到首个原生可行候选即返回；B保存该候选，在原1.2秒预算内继续优化原目标，只保留成本更低且原生检查通过的已接受候选。没有增加预算、改代价、放宽约束或更换时间映射。',
        '两组都开启现有MPPI短前缀及快层反馈；先比较同一B求解的首次可行和最终选择，再看正常在线交付和实走。', '', '结论']+data['conclusions']
    for row in data['rows']:
        chain, quality, motion = row['delivery_chain_counts'], row['quality'], row['motion_effects']
        lines += ['', row['label'],
            '请求→原生通过→最新CV发布→实际采用：%d→%d→%d→%d；最新动态拒绝%d次。前缀实际使用%d/%d次。' % (
                chain['refresh_requests'], chain['requests_with_native_optimizer_pass'],
                chain['refresh_reference_publications'], chain['refresh_references_consumed'], len(row['latest_rejections']),
                row['prefix_handoff']['prefix_used_count'], row['prefix_handoff']['prefix_attempt_count']),
            '实走%s米，起步到结束%s秒，平均|v|%s米/秒；最小物理采样间隙%s厘米。MPPI完整循环P95/最大%s/%s毫秒。' % (
                show(row['actual_path_length_m'], 3), show(row['motion_duration_s'], 3),
                show(motion['time_weighted_mean_abs_body_speed_mps'], 3), show(100*row['minimum_physical_sample_gap_m'], 3),
                show(row['callback']['p95_ms'], 1), show(row['callback']['max_ms'], 1))]
        for item in quality['checkpoints']:
            lines.append('请求%d：首次可行耗时%s毫秒（第%s次接受更新），最终选中耗时%s毫秒（第%s次），改善%s次，原生复核%s次；求解器退出码%s。' % (
                item['request_id'], show(1000*item['first_feasible_elapsed_s'], 1) if item['first_feasible_elapsed_s'] is not None else '未获得',
                item.get('first_feasible_iteration'), show(1000*item['selected_elapsed_s'], 1) if item['selected_elapsed_s'] is not None else '未获得',
                item.get('selected_iteration'), item.get('improvements'), item.get('native_checks'), item.get('solver_result')))
            for name, objective in [('初值', item['initial_objective']), ('首次可行', item['first_feasible_objective']), ('最终选择', item['selected_objective'])]:
                if objective:
                    lines.append('  %s原目标：总代价%s；平滑%s、时间%s、底盘可行性%s、底盘障碍%s、动态/其余%s。' % (
                        name, show(objective['total'], 3), show(objective['smooth'], 3), show(objective['time'], 3),
                        show(objective['base_feasibility'], 3), show(objective['base_obstacle'], 3),
                        show(sum(objective[k] for k in ('arm_obstacle', 'self_collision', 'arm_feasibility',
                            'other_integral', 'geometry', 'prediction')), 3)))
            first, selected = item['first_feasible_geometry'], item['selected_geometry']
            if first and selected:
                lines.append('  同次首次可行→最终：长度%s→%s米、曲线时长%s→%s秒、最大|y|%s→%s米；实际采用版本%s。' % (
                    show(first['full_curve_path_length_m'], 3), show(selected['full_curve_path_length_m'], 3),
                    show(first['duration_s'], 3), show(selected['duration_s'], 3),
                    show(first['maximum_absolute_world_y_m'], 3), show(selected['maximum_absolute_world_y_m'], 3),
                    [r['revision'] for r in item['consumed_reference_matches']]))
        for request in row['remani']['requests']:
            if request.get('request_id', 0) > 1:
                lines.append('请求%d：%s，%s毫秒；原生%s；发布拒绝%s。' % (
                    request['request_id'], OUTCOMES.get(request.get('outcome_class'), request.get('outcome_class')),
                    show(request.get('request_wall_ms'), 1), [c.get('reason') for c in request.get('optimizer_calls', [])],
                    request.get('acceptance_rejections', [])))
        execution = row['execution_evidence']
        lines.append('有效控制展开＋制动唯一核对%d/%d条；共同%s秒内非目标近停%s秒；终点位置合格后到结束%s秒。' % (
            execution['successful_exact_selected_and_brake_matches'], execution['validated_command_count'],
            show(row['common_motion_window']['duration_s'], 3), show(row['common_motion_window']['non_goal_low_speed']['total_s'], 3),
            show(motion['first_goal_position_entry_to_end_s'], 3)))
    lines += ['', '解释范围：上轮轻量回放只含轮/底盘可行性、平滑和时间成本，没有完整ESDF或全梯度，不冒充全目标重放。本轮快照来自在线原生完整目标，未重算地图。',
        '轮速参数按本轮parameters.yaml和manifest override核对，不能把原部署effective_real_params直接当本轮加载值。',
        '检查点原生可行不等于最新CV验收通过，更不等于新控制生效。若没有新的成本更低且原生可行候选，保留原可行检查点；不接受未验收的最后一次试探。',
        'GIF只画真实参考、实际获准MPPI展开和67球投影，未记录搜索候选。固定臂差速底盘完整ROS数值闭环，非Gazebo/实车；生产部署未替换。', '']
    return '\n'.join(lines)


def chart(data, output):
    rows = data['rows']
    image = Image.new('RGB', (1180, 590), 'white')
    draw = ImageDraw.Draw(image)
    title, text, small = [ImageFont.truetype(FONT, n) for n in (23, 18, 15)]
    colors = ('#2874a6', '#c47723')
    draw.text((20, 14), '可行后继续优化：候选质量与真正交付', font=title, fill='#333333')
    draw.text((20, 51), '前两栏：同一B求解，首=首次可行、选=最终选择；第三栏：在线A/B，不是同输入。', font=small, fill='#666666')
    item = rows[1]['quality']['first_refresh_checkpoint']
    first, selected = (item['first_feasible_geometry'], item['selected_geometry']) if item else (None, None)
    metrics = [('B同次候选曲线时长 / s', [r['duration_s'] if r else None for r in (first, selected)], ('首', '选')),
        ('B同次候选曲线长度 / m', [r['full_curve_path_length_m'] if r else None for r in (first, selected)], ('首', '选')),
        ('介入 → 有效新控制 / s', [r['stage_timeline']['hint_send_to_first_new_validated_command_wall_s'] for r in rows], ('A', 'B'))]
    for index, (name, values, labels) in enumerate(metrics):
        left, right = 55+index*390, 327+index*390
        upper = max([float(v) for v in values if v is not None]+[.1])*1.25
        draw.text((left-28, 103), name, font=small, fill='#333333')
        for fraction in (0., .5, 1.):
            x = left+fraction*(right-left)
            draw.line((x, 143, x, 299), fill='#e5e5e5')
            draw.text((x-8, 307), show(upper*fraction, 2), font=small, fill='#777777')
        for group, value in enumerate(values):
            y = 160+group*74
            draw.text((left-27, y+7), labels[group], font=text, fill=colors[group])
            if value is None:
                draw.text((left+5, y+8), '未获得/未交付', font=small, fill='#555555')
            else:
                x = left+float(value)/upper*(right-left)
                if value > 0.:
                    draw.rectangle((left, y, x, y+35), fill=colors[group])
                draw.text((x+5, y+8), show(value, 3), font=small, fill='#444444')
    draw.line((20, 352, 1160, 352), fill='#dddddd')
    for group, row in enumerate(rows):
        y, chain = 371+group*82, row['delivery_chain_counts']
        chain_text = '→'.join(str(chain[k]) for k in ('refresh_requests', 'requests_with_native_optimizer_pass',
            'refresh_reference_publications', 'refresh_references_consumed'))
        draw.text((20, y), 'AB'[group]+'　请求→原生→发布→采用：'+chain_text+
            '；实走 %s m / %s s' % (show(row['actual_path_length_m'], 3), show(row['motion_duration_s'], 3)), font=text, fill=colors[group])
        draw.text((20, y+32), '最小物理采样间隙 %s cm；%s接触；循环P95 %s ms。' % (
            show(100*row['minimum_physical_sample_gap_m'], 3), '发现' if row['physical_sample_collision'] else '未发现',
            show(row['callback']['p95_ms'], 1)), font=small, fill='#555555')
    if item:
        draw.text((20, 537), 'B同次原目标 %s → %s；后续改善 %s 次；成本降低不自动保证路线更短或实际交付更早。' % (
            show(item['first_cost'], 3), show(item['best_cost'], 3), item['improvements']), font=small, fill='#666666')
    draw.text((20, 563), '原1.2秒预算与完整验收保留；正常不暂停ROS数值闭环，失败与未交付均保留。', font=small, fill='#666666')
    image.save(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=DEFAULT)
    parser.add_argument('--max-frames', type=int, default=70)
    args = parser.parse_args()
    pairs = [load_online(args.root/'headon_A', LABELS[0]), load_online(args.root/'headon_B', LABELS[1])]
    rows, trials = zip(*pairs)
    add_common_motion_window(rows, trials)
    data = dict(schema='feasible_quality_report_v1', rows=list(rows), conclusions=conclusions(rows),
        previous_partial_cost_audit=read_evidence(args.root),
        same_native_binaries=rows[0]['binary_identity']['native'] == rows[1]['binary_identity']['native'],
        same_mppi_library=rows[0]['binary_identity']['mppi'] == rows[1]['binary_identity']['mppi'],
        scope='One normal unpaused fixed-arm numerical ROS SIL pair, first-feasible exit vs original-budget quality polishing. Within-B accepted safe checkpoints compared; A/B are asynchronous, not frozen-input paired solves.')
    output = args.root/'report'
    output.mkdir(parents=True, exist_ok=True)
    data['artifacts'] = dict(headon=animation(rows, trials, output/'迎面对照.gif', args.max_frames))
    keyframes(output/'迎面对照.gif', output/'迎面关键帧.png')
    chart(data, output/'对照数据.png')
    (output/'实验结果.txt').write_text(narrative(data))
    for trial in trials:
        trial.verify_unchanged()
    data['report_script_sha256'] = sha(Path(__file__))
    (output/'summary.json').write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print('FEASIBLE_QUALITY_REPORT', output, flush=True)


if __name__ == '__main__':
    main()
