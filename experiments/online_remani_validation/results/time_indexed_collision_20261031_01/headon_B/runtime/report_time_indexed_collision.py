#!/usr/bin/env python3
"""Report recorded A/B delivery, motion and computational costs, with two GIFs."""
import argparse
from collections import Counter
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

from render_dual_timescale import FONT
from report_frontend_rejoin import delivery_metrics, keyframes
from report_static_agility import gif, show, summarize

HERE = Path(__file__).resolve().parent
DEFAULT = HERE / 'results/time_indexed_collision_20261031_01'
OUTCOMES = dict(accepted='交付成功', live_acceptance_reject='发布验收拒绝',
    optimizer_reject_or_timeout='优化或硬检查未通过', search_failure='搜索失败')


def outcomes_text(values):
    return '；'.join('%s %d次' % (OUTCOMES.get(k, k), v) for k, v in values.items()) or '无重规划记录'


def inspect(source, label):
    row, trial = summarize(source, label)
    row['delivery'] = delivery_metrics(row, trial)
    timeline = trial.reader.jsonl('dual_timeline.jsonl')
    row['time_query'] = [r for r in timeline if r['event'].startswith('time_indexed_collision_')]
    row['temporal_capture'] = [r for r in timeline if r['event'] == 'temporal_collision_capture']
    analysis = trial.reader.json('analysis/summary.json')
    row['wall_accounting'] = analysis.get('native_wall_accounting') if analysis else None
    row['settings'] = trial.manifest.get('time_indexed_collision', {})
    solvers = trial.reader.jsonl('solver_trace.jsonl')
    row['budget_stops'] = [r for r in solvers if r.get('event') == 'end'
                           and r.get('stage') == 'optimizer' and r.get('force_stop') == 3]
    row['latest_rejections'] = [r for r in timeline if r['event'] == 'latest_prediction_reject_detail']
    row['replanning_outcomes'] = dict(Counter(r.get('outcome_class') for r in row['remani']['requests']
                                            if r.get('request_id', 0) > 1))
    row['capture_serialization_ms'] = sum(r.get('serialization_wall_ms', 0.) for r in row['temporal_capture'])
    trial.verify_unchanged()
    return row, trial


def comparison_chart(rows, output):
    image = Image.new('RGB', (1260, 180 + 160 * len(rows)), 'white')
    draw = ImageDraw.Draw(image)
    title, normal, small = [ImageFont.truetype(FONT, n) for n in (23, 18, 15)]
    draw.text((20, 14), '动态碰撞按时间检查：能否更快交付新参考', font=title, fill='#333333')
    draw.text((20, 52), '主指标是实际采用新参考；单次请求计算成功与最终交付分开统计。', font=small, fill='#666666')
    for i, row in enumerate(rows):
        top = 92 + 160 * i
        draw.line((20, top-10, 1235, top-10), fill='#dddddd')
        color = '#2874a6' if not row['settings'].get('enabled') else '#c47723'
        draw.text((20, top), row['label'], font=normal, fill=color)
        delay = row['delivery']['first_refresh_start_to_consumed_reference_ms']
        success = row['delivery']['first_adopted_request_wall_ms']
        draw.text((20, top+32), '首次请求→采用：%s ms；交付那次请求：%s ms' % (
            show(delay, 1), show(success, 1)), font=normal, fill='#444444')
        draw.text((20, top+64), '重规划结果：'+outcomes_text(row['replanning_outcomes']), font=small, fill='#555555')
        draw.text((20, top+91), '实走 %.2f m / %.2f s；最小物理采样间隙 %.1f cm；MPPI回调最大 %s ms' % (
            row['actual_path_length_m'], row['motion_duration_s'], 100*row['minimum_physical_sample_gap_m'],
            show(row['callback']['max_ms'], 1)), font=small, fill='#555555')
        draw.text((20, top+118), '目标：%s；采样碰撞：%s；非目标停顿：%.3f s' % (
            '到达' if row['reached_goal'] else row['termination'],
            '有' if row['physical_sample_collision'] else '未发现', row['non_goal_low_speed']['total_s']), font=small, fill='#555555')
    draw.text((20, image.height-44), '固定机械臂完整 REMANI+MPPI ROS数值闭环。A/B为有限次在线对照，不能据此估计总体成功率。', font=small, fill='#777777')
    image.save(output)


def narrative(rows, notes):
    lines = ['动态碰撞时间化实验结果', '',
        '这轮要回答：把障碍物未来位置合成一片占据区域，是否造成不必要拒绝，拖慢新轨迹交付。',
        'A沿用当前查询；B让后端动态代价、原生硬检查及发布前规划检查按请求计划时间查询。',
        '两组均关闭新增软反馈。球、膨胀、轮约束、预算、前端、MPPI及最后最新CV验收保持一致。',
        '计划时间不保证等于最后启用时间；超出原预测窗时仍退回原扫掠查询。', '']
    if notes:
        lines += ['先看结论'] + list(notes) + ['']
    for row in rows:
        d, w = row['delivery'], row['wall_accounting'] or {}
        lines += [row['label'],
            '结果：%s；实走%s米，运动%s秒，非目标停顿%s秒。' % (
                '到达目标' if row['reached_goal'] else row['termination'], show(row['actual_path_length_m']),
                show(row['motion_duration_s']), show(row['non_goal_low_speed']['total_s'])),
            '最小物理球采样间隙%s厘米，%s采样碰撞。' % (
                show(100*row['minimum_physical_sample_gap_m']), '发生' if row['physical_sample_collision'] else '未发现'),
            '首次重规划请求到MPPI采用%s毫秒；交付那次请求计算%s毫秒。前者包含失败和重试。' % (
                show(d['first_refresh_start_to_consumed_reference_ms']), show(d['first_adopted_request_wall_ms'])),
            '受阻到发布墙钟%s秒；各次请求合计%s秒，已记录等待%s秒。' % (
                show(w.get('total_wall_s')), show(w.get('request_union_wall_s')), show(w.get('closed_explicit_wait_union_wall_s'))),
            'MPPI整回调P95/最大%s/%s毫秒；内核P95/最大%s/%s毫秒。' % (
                show(row['callback']['p95_ms']), show(row['callback']['max_ms']),
                show(row['kernel']['p95_ms']), show(row['kernel']['max_ms'])),
            '控制结果计数：'+json.dumps(row['reason_counts'], ensure_ascii=False),
            '预算停止次数：%d；取证文件序列化累计%s毫秒（发生在请求决策后）。' % (
                len(row['budget_stops']), show(row['capture_serialization_ms']))]
        for request in row['remani']['requests']:
            lines.append('请求%d：%s，墙钟%s毫秒；优化原因%s；发布拒绝%s。' % (
                request['request_id'], OUTCOMES.get(request.get('outcome_class'), request.get('outcome_class')), show(request.get('request_wall_ms')),
                [c.get('reason') for c in request.get('optimizer_calls', [])], request.get('acceptance_rejections', [])))
        for rejection in row['latest_rejections']:
            lines.append('最新预测拒绝：请求%d，分支%s，候选局部时间%s秒，执行比例%s，候选年龄%s秒。' % (
                rejection['request_id'], rejection.get('branch'), show(rejection.get('local_t')),
                show(rejection.get('execution_scale')), show(rejection.get('candidate_age'))))
        for scope in [r for r in row['time_query'] if r['event'] == 'time_indexed_collision_summary']:
            lines.append('时间查询统计：请求%d，实际启用%s，按时查询%d次，扫掠回退%d次。' % (
                scope['request_id'], scope.get('active'), scope.get('indexed_queries', 0), scope.get('swept_fallback_queries', 0)))
        lines.append('')
    lines += ['数据解释',
        'GIF显示完整固定臂碰撞球投影、REMANI参考和获准的MPPI局部展开；绿色并非所有搜索采样。',
        '上轮handoff_native_planning_reject探针读取原扫掠层，对B不能直接当作实际拒绝来源；B应看本轮完整候选复算。',
        'MPPI耗时统计窗口为障碍出现后；交付耗时以实际采用记录关联请求，不把优化返回成功当作交付。',
        '物理球采样间隙、按预测模型检查间隙是不同数据；模型碰撞不等于已发生物理接触。',
        '全部为新跑的固定机械臂ROS数值闭环，生产部署未变。两场景各一对试次用于定位问题，不证明总体优越性。', '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=DEFAULT)
    parser.add_argument('--note', action='append', default=[])
    parser.add_argument('--max-frames', type=int, default=100)
    args = parser.parse_args()
    output = args.root / 'report'
    output.mkdir(exist_ok=True)
    rows, artifacts = [], {}
    for scene, name in [('headon', '迎面动态'), ('static', '静态挡路')]:
        pair = [inspect(args.root / (scene+'_'+group), name+' '+label)
                for group, label in [('A', 'A：原查询'), ('B', 'B：按时间查询')]]
        selected, trials = zip(*pair)
        rows.extend(selected)
        animation = output / (name+'对照.gif')
        artifacts[scene] = gif(selected, trials, animation, args.max_frames)
        keyframes(animation, output / (name+'关键帧.png'))
    comparison_chart(rows, output / '交付与运动对照.png')
    (output / '实验结果.txt').write_text(narrative(rows, args.note))
    (output / 'summary.json').write_text(json.dumps(dict(rows=rows, notes=args.note,
        artifacts=artifacts, scope='Recorded fixed-arm numerical ROS SIL; no new controller replay'),
        ensure_ascii=False, indent=2) + '\n')
    print('TIME_INDEXED_COLLISION_REPORT', output, flush=True)


if __name__ == '__main__':
    main()
