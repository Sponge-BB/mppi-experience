#!/usr/bin/env python3
"""Read paired handoff-feedback SIL runs; distinguish trial outcomes from probes.

Only reports recorded observations. A frozen spatial soft objective is not a
time-alignment method or a committed MPPI prefix. Early feasibility-gate failures
are reported separately from final native-gate records and actual delivery.
"""
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from render_coupled_objective import sha
from render_dual_timescale import FONT
from report_frontend_rejoin import delivery_metrics, keyframes
from report_static_agility import gif, narrative as motion_narrative, show, summarize


SOURCE_NAMES = dict(esdf_only='ESDF本层', swept_only='预测扫掠层', both='ESDF与扫掠均有',
                    neither='底盘球未见冲突', missing='没有诊断')
OUTCOME_NAMES = dict(accepted='交付成功', live_acceptance_reject='发布验收拒绝',
    optimizer_reject_or_timeout='优化未通过', search_failure='搜索失败')


def pose_source(pose):
    if not pose:
        return 'missing'
    a, b = pose.get('esdf_alone_collision'), pose.get('swept_alone_collision')
    if a and b:
        return 'both'
    if a:
        return 'esdf_only'
    if b:
        return 'swept_only'
    return 'neither'


def reject_record(row):
    record = dict(row)
    record['start_source'] = pose_source(row.get('start'))
    record['failure_source'] = pose_source(row.get('failure'))
    for field in ('start', 'failure'):
        pose = row.get(field, {})
        swept = pose.get('swept', {})
        sequence = swept.get('frozen_sequence') if swept.get('frozen') else swept.get('sequence')
        record[field+'_snapshot_sequence_matches_swept'] = (
            sequence == row.get('cv_snapshot_sequence') if swept.get('ready') else None)
    return record


def inspect(source, label):
    row, trial = summarize(source, label)
    row['handoff_settings'] = trial.manifest.get('handoff_alignment', {})
    timeline = trial.reader.jsonl('dual_timeline.jsonl')
    solvers = [r for r in trial.reader.jsonl('solver_trace.jsonl')
               if r.get('event') == 'end' and r.get('stage') == 'optimizer']
    scopes = [r for r in timeline if r['event'] == 'coupling_scope']
    all_costs = [r for r in timeline if r['event'] == 'coupling_cost']
    costs = [r for r in all_costs if r.get('phase') == 'final']
    scope_keys = [(r['request_id'], r['scope_id']) for r in scopes]
    cost_keys = [(r['request_id'], r['scope_id']) for r in costs]
    if len(set(scope_keys)) != len(scope_keys) or len(set(cost_keys)) != len(cost_keys):
        raise ValueError('duplicate optimizer scope/final cost; counters would double count')
    rejects = [reject_record(r) for r in timeline if r['event'] == 'handoff_native_planning_reject']
    stages = ('first_native_gate_failure', 'final_native_gate')
    if any(r.get('stage') not in stages for r in rejects):
        raise ValueError('unknown native rejection diagnostic stage')
    requests = []
    for request in row['remani']['requests']:
        identifier = request['request_id']
        selected_scopes = [r for r in scopes if r['request_id'] == identifier]
        selected_costs = [r for r in costs if r['request_id'] == identifier]
        selected_rejects = [r for r in rejects if r['request_id'] == identifier]
        optimizer_ids = {c['resource_optimizer_id'] for c in request.get('optimizer_calls', [])}
        selected_solvers = [r for r in solvers if r['id'] in optimizer_ids]
        requests.append(dict(request_id=identifier, actual_outcome=request.get('outcome_class'),
            request_wall_ms=request.get('request_wall_ms'),
            optimizer_reasons=[c.get('reason') for c in request.get('optimizer_calls', [])],
            acceptance_rejections=request.get('acceptance_rejections', []),
            scope_count=len(selected_scopes), eligible_scope_count=sum(bool(r['valid']) for r in selected_scopes),
            active_scope_count=sum(bool(r.get('active')) for r in selected_costs),
            callback_count=sum(r.get('callback_count', 0) for r in selected_costs),
            active_callback_count=sum(r.get('active_callback_count', 0) for r in selected_costs),
            scope_reasons=dict(Counter(r.get('reason') for r in selected_scopes)),
            budget_stop_records=[r for r in selected_solvers if r.get('force_stop') == 3],
            first_native_gate_failure=[r for r in selected_rejects if r['stage'] == stages[0]],
            final_native_gate=[r for r in selected_rejects if r['stage'] == stages[1]]))
    row['feedback'] = dict(scopes=scopes, final_costs=costs, all_recorded_costs=all_costs,
        inputs=[r for r in timeline if r['event'] == 'handoff_feedback_input'],
        total_scopes=len(scopes), eligible_scopes=sum(bool(r['valid']) for r in scopes),
        active_scopes=sum(bool(r.get('active')) for r in costs),
        final_cost_records=len(costs), active_callbacks=sum(r.get('active_callback_count', 0) for r in costs),
        counting='Count unique optimizer scopes and final cost records only; never sum initial and final callback counters.')
    row['native_collision_diagnosis'] = dict(requests=requests, records=rejects,
        stage_counts=dict(Counter(r['stage'] for r in rejects)),
        first_gate_start_sources=dict(Counter(r['start_source'] for r in rejects if r['stage'] == stages[0])),
        final_gate_start_sources=dict(Counter(r['start_source'] for r in rejects if r['stage'] == stages[1])),
        diagnostic_total_wall_ms=sum(r.get('diagnostic_wall_ms', 0.) for r in rejects),
        probe_errors=[r for r in timeline if r['event'] == 'handoff_native_planning_probe_error'],
        scope='Read-only base-sphere diagnosis, not full-body or physical truth. First gate is an early candidate; final gate is a return-check event. Actual request outcome and publication remain authoritative.')
    row['delivery'] = delivery_metrics(row, trial)
    analysis = trial.reader.json('analysis/summary.json')
    row['native_wall_accounting'] = analysis.get('native_wall_accounting') if analysis else None
    row['input_sha256'] = dict(trial.reader.hashes)
    trial.verify_unchanged()
    return row, trial


def gate_text(records):
    if not records:
        return '无记录'
    return '；'.join('%s / t=%s s' % (SOURCE_NAMES[r['start_source']], show(r.get('failure_t'), 2))
                    for r in records)


def chart(rows, destination):
    count = max(len(r['native_collision_diagnosis']['requests']) for r in rows)
    height = 325+count*72
    image = Image.new('RGB', (1320, height), 'white')
    draw = ImageDraw.Draw(image)
    title, normal, small = [ImageFont.truetype(FONT, size) for size in (22, 17, 14)]
    draw.text((20, 12), '空间软反馈：是否参与优化，候选为什么没交付', font=title, fill='#333333')
    draw.text((20, 47), '这是两次新在线闭环；早期迭代失败不等于最终失败，最终结果看“整次结果”。', font=small, fill='#666666')
    for index, row in enumerate(rows):
        left = 20+index*650
        feedback = row['feedback']
        draw.text((left, 82), row['label'], font=normal, fill='#333333')
        draw.text((left, 112), '实走 %.2f m / %.2f s；最小采样间隙 %.1f cm' % (
            row['actual_path_length_m'], row['motion_duration_s'], 100*row['minimum_physical_sample_gap_m']), font=small, fill='#555555')
        draw.text((left, 139), '反馈参与：%d/%d 次优化；有效代价计算 %d 次' % (
            feedback['active_scopes'], feedback['total_scopes'], feedback['active_callbacks']), font=small, fill='#555555')
        for x, text in ((0, '请求 / 整次结果'), (198, '早期失败：起点来源'), (410, '最终硬检查：起点来源')):
            draw.text((left+x, 178), text, font=small, fill='#777777')
        for number, request in enumerate(row['native_collision_diagnosis']['requests']):
            top = 209+number*72
            draw.line(((left, top-8), (left+630, top-8)), fill='#dddddd')
            status = OUTCOME_NAMES.get(request['actual_outcome'], request['actual_outcome'] or '未完成')
            draw.text((left, top), 'q%d  %s' % (request['request_id'], status), font=small,
                      fill='#258446' if request['actual_outcome'] == 'accepted' else '#b55828')
            draw.text((left, top+24), '请求 %s ms / 反馈%d次' % (
                show(request['request_wall_ms'], 1), request['active_scope_count']), font=small, fill='#555555')
            draw.text((left+198, top), gate_text(request['first_native_gate_failure']), font=small, fill='#555555')
            draw.text((left+410, top), gate_text(request['final_native_gate']), font=small, fill='#555555')
        blocked = next((r for r in row['native_collision_diagnosis']['records']
                        if r['stage'] == 'final_native_gate' and r['start_source'] == 'swept_only'), None)
        if blocked:
            start, swept = blocked['start'], blocked['start']['swept']
            draw.text((left, height-148), 'q%d起点球：ESDF %s m / 扫掠 %s m / 硬半径 %s m' % (
                blocked['request_id'], show(start.get('precise_esdf_distance_m')),
                show(swept.get('distance_after_point_radius_m')), show(start.get('hard_radius_m'))), font=small, fill='#b55828')
            draw.text((left, height-124), 'ESDF距离足够，扫掠距离已小于硬半径；不是实测碰撞。', font=small, fill='#b55828')
    draw.text((20, height-85), '起点来源只说明被记录底盘球的ESDF层/预测扫掠层；t是首次失败处的候选局部时间，不是计算耗时。', font=small, fill='#666666')
    draw.text((20, height-56), '“无记录”不等于检查通过；可能在轮速、数值检查或发布验收等其他环节拒绝。', font=small, fill='#666666')
    draw.text((20, height-27), '固定机械臂、完整REMANI＋MPPI ROS数值闭环；非Gazebo/实车。软空间反馈不是时间对齐。', font=small, fill='#666666')
    image.save(destination)


def narrative(rows, notes, counterfactual):
    lines = ['MPPI短路径空间反馈：在线实验结果', '',
        '一、这次改什么',
        '各组都保留上轮降低前端转向评分的设置。主对照是原组关闭反馈，候选组打开执行空间反馈和交付状态软匹配；另保留只开执行项的中间试次。',
        '每次REMANI请求冻结一份已检查的MPPI短期预测，把它作为位置和朝向的软偏好加入优化。',
        '交付状态项取冻结预测中约0.65秒后的位置、朝向和速度作空间匹配目标，不强制REMANI在对应绝对时刻到达它。',
        '这不是把MPPI轨迹硬接成前缀，不是按未来位置改起点，也没有实现时间对齐；碰撞检查、轮速限制和发布验收保持不变。', '',
        '二、在线结果']
    for row in rows:
        feedback, delivery = row['feedback'], row['delivery']
        lines += ['', row['label']+'：'+('到达目标。' if row['reached_goal'] else '未到达目标：'+row['termination']),
            '执行项权重%s，交付状态项权重%s。' % (
                show(row['handoff_settings'].get('execution_weight')), show(row['handoff_settings'].get('handover_weight'))),
            '实走%s米，运动%s秒；最小物理球采样间隙%s厘米，%s采样碰撞；非目标停顿累计%s秒。' % (
                show(row['actual_path_length_m']), show(row['motion_duration_s']),
                show(100*row['minimum_physical_sample_gap_m']), '发生' if row['physical_sample_collision'] else '未发现',
                show(row['non_goal_low_speed']['total_s'])),
            'MPPI内核P95/最大%s/%s毫秒；整回调P95/最大%s/%s毫秒。' % (
                show(row['kernel']['p95_ms']), show(row['kernel']['max_ms']),
                show(row['callback']['p95_ms']), show(row['callback']['max_ms'])),
            '控制结果计数：'+json.dumps(row['reason_counts'], ensure_ascii=False),
            '反馈：%d个优化作用域，%d个开启且输入合格，%d个实际产生非零反馈代价；合计%d次代价回调产生非零反馈。' % (
                feedback['total_scopes'], feedback['eligible_scopes'], feedback['active_scopes'], feedback['active_callbacks']),
            '首次重规划请求开始到MPPI采用%s毫秒；实际交付那次请求本身计算%s毫秒。前者包含失败重试，二者不能混为单次求解时间。' % (
                show(delivery['first_refresh_start_to_consumed_reference_ms']), show(delivery['first_adopted_request_wall_ms']))]
        accounting = row.get('native_wall_accounting')
        if accounting:
            lines += ['另一墙钟口径：REMANI判定参考受阻到发布%s秒，其中各次请求合计%s秒，已记录的重试/ESDF等待%s秒。' % (
                show(accounting['total_wall_s']), show(accounting['request_union_wall_s']),
                show(accounting['closed_explicit_wait_union_wall_s']))]
        for request in row['native_collision_diagnosis']['requests']:
            lines += ['请求%d：%s；反馈实际参与%d个优化，%d次有效代价回调。' % (
                request['request_id'], OUTCOME_NAMES.get(request['actual_outcome'], request['actual_outcome'] or '未完成'),
                request['active_scope_count'], request['active_callback_count']),
                '  输入判定：'+json.dumps(request['scope_reasons'], ensure_ascii=False)+'；优化日志原因：'+str(request['optimizer_reasons'])+
                '；发布验收拒绝：'+str(request['acceptance_rejections'])]
            for stop in request['budget_stop_records']:
                lines += ['  本次触发计算预算停止：L-BFGS实际%s毫秒，预算%s秒；最终是否可交付仍需看保留候选的安全检查结果。' % (
                    show(stop.get('lbfgs_wall_ms')), show(stop.get('budget_s')))]
            for stage, label in (('first_native_gate_failure', '早期候选检查失败'), ('final_native_gate', '后端返回前硬检查失败')):
                for record in request[stage]:
                    start, failure = record['start'], record['failure']
                    lines += ['  %s：候选失败处t=%s秒；起点来源=%s，起点底盘最小检查间隙%s厘米；失败处来源=%s，间隙%s厘米。' % (
                        label, show(record['failure_t']), SOURCE_NAMES[record['start_source']],
                        show(100*start['base_minimum_hard_gap_m']) if start.get('base_minimum_hard_gap_m') is not None else '未知',
                        SOURCE_NAMES[record['failure_source']],
                        show(100*failure['base_minimum_hard_gap_m']) if failure.get('base_minimum_hard_gap_m') is not None else '未知')]
                    swept = start.get('swept', {})
                    lines += ['    起点记录球%d：硬半径%s米，ESDF距离%s米，扫掠距离%s米；重建距离与原查询差%s米。' % (
                        start.get('reported_sphere_index', -1), show(start.get('hard_radius_m'), 6),
                        show(start.get('precise_esdf_distance_m'), 6), show(swept.get('distance_after_point_radius_m'), 6),
                        show(start.get('reconstructed_distance_difference_m'), 9)),
                        '    扫掠序列%s（冻结序列%s），诊断CV快照序列%s；是否同序列：%s。' % (
                        swept.get('sequence'), swept.get('frozen_sequence'), record.get('cv_snapshot_sequence'),
                        '是' if record['start_snapshot_sequence_matches_swept'] else '否/未确认')]
        lines += ['本组碰撞来源探针累计耗时%s毫秒。' % show(row['native_collision_diagnosis']['diagnostic_total_wall_ms'])]
        if row['native_collision_diagnosis']['probe_errors']:
            lines += ['诊断探针错误：'+json.dumps(row['native_collision_diagnosis']['probe_errors'], ensure_ascii=False)]
    lines += ['', '三、结论']+list(notes or ['结论待根据实际数据填写，不自动宣布反馈有效。'])
    lines += ['', '四、不能混淆的几点',
        'first_native_gate_failure只记录本请求早期候选首次检查失败，优化随后可能修复；final_native_gate是后端返回前检查失败记录。是否真正没交付仍看请求结果和实际参考采用，不能只数失败事件。',
        '探针只解释底盘碰撞球。ESDF本层不等于已确认是静态障碍物；“预测扫掠”是把预测占据区域合并后的空间层，不能当作障碍物在当前时刻真的撞到了机器人。',
        '探针中的当前/未来CV比较只是诊断替代查询，不改变实际验收。扫掠点没有保留每个点的预测时间戳；缺少一致快照时不能据此断言时间错配。',
        '空间反馈按同挡位位置/朝向软匹配，不要求同一个时刻到达同一个位置，因此本轮不是完整的双时间尺度时间耦合方案。',
        '反馈计数只读取每个优化作用域的final累计值，避免把initial和final重复相加。',
        '分项成本只记录initial/final，不包含每次中间回调；因此不能把总非零次数直接全归到某一个分项。控制耗时统计为障碍出现后的有效记录，与整段运动的P95口径不同。',
        '三组为新跑的固定机械臂、完整REMANI+底盘MPPI数值ROS闭环，不是Gazebo或实车；物理采样间隙与规划检查间隙不同。',
        '有限次数只能验证本次现象，不宣称总体成功率；未发现采样碰撞不等于连续时间安全证明。']
    if counterfactual:
        lines += ['', '五、上一轮旧候选的反事实复算（不是本轮新在线轨迹）',
            '目录：'+str(counterfactual),
            '同输入时间映射对照.png展示上一轮被拒候选的不同时间起点/执行速度假设；候选没有实际执行，不能当成改后闭环成功证据。']
    return '\n'.join(lines)+'\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--treatment', type=Path, required=True)
    parser.add_argument('--execution-only', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--counterfactual-report', type=Path)
    parser.add_argument('--note', action='append', default=[])
    parser.add_argument('--no-gif', action='store_true')
    parser.add_argument('--max-frames', type=int, default=100)
    args = parser.parse_args()
    pairs = [inspect(args.baseline, '原条件：反馈关闭'), inspect(args.treatment, '候选：执行＋交付状态软匹配')]
    if args.execution_only:
        pairs.append(inspect(args.execution_only, '中间试次：只开执行空间反馈'))
    rows, trials = map(list, zip(*pairs))
    if trials[0].manifest['scenario'] != trials[1].manifest['scenario']:
        raise ValueError('different paired scenarios')
    if not np.array_equal(trials[0].body3d, trials[1].body3d):
        raise ValueError('different paired collision geometry')
    output = args.output.resolve()
    if any(output == t.source or t.source in output.parents for t in trials):
        raise ValueError('report must be outside trial directories')
    output.mkdir(parents=True, exist_ok=True)
    chart(rows[:2], output/'反馈参与与候选拒绝.png')
    rendering = None
    if not args.no_gif:
        rendering = gif(rows[:2], trials[:2], output/'迎面空间反馈对照.gif', args.max_frames)
        keyframes(output/'迎面空间反馈对照.gif', output/'迎面对照关键帧.png')
    for trial in trials:
        trial.verify_unchanged()
    counterfactual = None
    if args.counterfactual_report:
        counterfactual = dict(path=str(args.counterfactual_report.resolve()), files={p.name: sha(p)
            for p in args.counterfactual_report.iterdir() if p.is_file()})
    (output/'summary.json').write_text(json.dumps(dict(trials=rows, rendering=rendering,
        notes=args.note, old_candidate_counterfactual=counterfactual,
        scope='Fresh paired numerical ROS SIL; spatial soft feedback is not temporal alignment.'),
        ensure_ascii=False, indent=2)+'\n')
    (output/'实验结果.txt').write_text(narrative(rows, args.note, args.counterfactual_report))
    (output/'运动详细数据.txt').write_text(motion_narrative(rows).replace(
        '静态绕行响应对照', '迎面动态障碍物运动数据', 1))
    print(json.dumps(dict(output=str(output), trials=[dict(label=r['label'], reached_goal=r['reached_goal'],
        delivery=r['delivery'], feedback_active_scopes=r['feedback']['active_scopes'],
        rejection_stage_counts=r['native_collision_diagnosis']['stage_counts']) for r in rows]), ensure_ascii=False))


if __name__ == '__main__':
    main()
