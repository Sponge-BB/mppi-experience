#!/usr/bin/env python3
"""Report one finished short-prefix fallback search-cost A/B experiment.

Both slow planners use the same 4.5 s CV window. Only eligible B fallback
searches temporarily remove the gear-switch penalty. Recorded final-native
adoption and actual animation are reused from the mature midcross report.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

import report_delivery_midcross as midcross
from render_coupled_objective import sha
from render_dual_timescale import FONT
from report_frontend_rejoin import keyframes
from report_half_speed_contract import add_common_motion_window
from report_static_agility import crossing_y, show
from report_static_width_diagnosis import geometry, sample_curve
from report_time_indexed_collision import TERMINATIONS


HERE = Path(__file__).resolve().parent
DEFAULT = HERE / 'results/short_prefix_rejoin_20261101_01'
LABELS = ('A：原回退；慢层4.5s/最新3s', 'B：换向提示；慢层4.5s/最新3s')


def sign(value):
    return 1 if value > 0 else -1 if value < 0 else 0


def path_summary(points, request, stage, gears=None):
    points = np.asarray(points, dtype=float).reshape((-1, 2))
    if not len(points):
        return None
    return dict(stage=stage, point_count=len(points), start_xy=points[0].tolist(), end_xy=points[-1].tolist(),
        sampled_path_length_m=float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1))),
        minimum_x_m=float(np.min(points[:, 0])), maximum_x_m=float(np.max(points[:, 0])),
        minimum_y_m=float(np.min(points[:, 1])), maximum_y_m=float(np.max(points[:, 1])),
        maximum_absolute_y_m=float(np.max(np.abs(points[:, 1]))),
        world_x_backtrack_from_request_start_m=max(0., request['start_x'] - float(np.min(points[:, 0]))),
        y_at_all_x3_crossings_m=crossing_y(points, 3.), gears=gears,
        scope='Geometric extent and sampled path length. Negative-world-x movement is not itself reverse gear or proof of a loop; gears and actual animation are reported separately.')


def selected_chain(record, request, hint_gear):
    nodes = record.get('nodes', [])
    if not nodes:
        return dict(record=record, available=False)
    if any(not isinstance(node.get('state'), list) or len(node['state']) < 3 for node in nodes):
        raise ValueError('short_prefix_selected_nodes state schema changed')
    points = np.asarray([node['state'][:2] for node in nodes], dtype=float)
    actual_start = record.get('actual_search_start_state')
    start = (np.asarray(actual_start[:2], dtype=float) if actual_start is not None else
             np.array([request['start_x'], request['start_y']]))
    reversed_order = bool(np.linalg.norm(points[-1] - start) < np.linalg.norm(points[0] - start))
    ordered = list(reversed(nodes)) if reversed_order else nodes
    directions = [int(node['singul']) for node in ordered if int(node['singul']) != 0]
    compact = []
    for direction in directions:
        if not compact or compact[-1] != direction:
            compact.append(direction)
    return dict(record=record, available=True, reversed_record_order_for_start_to_end_analysis=reversed_order,
        selected_direction_sequence=compact, selected_direction_change_count=max(0, len(compact) - 1),
        includes_hint_direction=hint_gear in directions if hint_gear in (-1, 1) else None,
        geometry=path_summary([node['state'][:2] for node in ordered], request, 'selected_astar_chain_before_resampling', compact),
        scope='Selected A* chain before resampling, not all expanded search nodes; analytic shot coverage follows the recorded shot_success field.')


def intervention_analysis(row, trial):
    timeline = trial.reader.jsonl('dual_timeline.jsonl')
    trace = trial.reader.jsonl('static_width_trace.jsonl')
    captures = trial.reader.jsonl('temporal_collision.jsonl')
    scopes = [r for r in timeline if r.get('event') == 'short_prefix_rejoin_scope']
    selections = [r for r in timeline if r.get('event') == 'short_prefix_selected_nodes']
    requests = {r['request_id']: r for r in timeline if r.get('event') == 'request_begin'}
    prefixes = [r for r in timeline if r.get('event') == 'collab_prefix']
    finals = [r for r in captures if r.get('stage') == 'final_candidate']
    calls = row['deferred_quality']['calls']
    ids = sorted(set(r['request_id'] for r in scopes if r['request_id'] > 1))
    records = []
    for request_id in ids:
        request = requests[request_id]
        here = [r for r in scopes if r['request_id'] == request_id]
        selected = [r for r in selections if r['request_id'] == request_id]
        hint_gears = [r['hint_gear'] for r in here if r.get('hint_valid')]
        hint_gear = hint_gears[0] if hint_gears and len(set(hint_gears)) == 1 else None
        raw = [r for r in trace if r.get('stage') == 'native_search_raw' and r['request_id'] == request_id]
        initial = [r for r in trace if r.get('stage') == 'minco_before_optimizer' and r['request_id'] == request_id]
        returned = [r for r in finals if r['request_id'] == request_id]
        matched_calls = [c for c in calls if c['request_id'] == request_id]
        records.append(dict(request_id=request_id, request_start={k: request[k] for k in ('start_x', 'start_y', 'start_yaw')},
            scopes=here, prefix_events=[r for r in prefixes if r['request_id'] == request_id],
            selected_chains=[selected_chain(r, request, hint_gear) for r in selected],
            search_resampled_paths=[dict(geometry=path_summary(geometry(r), request, 'native_search_resampled_output', r.get('gears')),
                capture_mono=r['capture_mono'], semantics=r.get('semantics')) for r in raw],
            initial_curves=[dict(geometry=path_summary(sample_curve(r), request, 'minco_before_optimizer',
                [p['gear'] for p in r['parts']]), capture_mono=r['capture_mono']) for r in initial],
            final_curves=[dict(native_success=r['native_success'], native_reason=r['native_reason'],
                plan_ordinal=r['plan_ordinal'], geometry=path_summary(sample_curve(dict(parts=r['candidate']['parts'])),
                    request, 'actual_final_native_candidate', [p['gear'] for p in r['candidate']['parts']])) for r in returned],
            actual_adoption=[dict(plan_ordinal=c['plan_ordinal'], actually_adopted=c['selected_curve_actually_adopted'],
                consumed_revisions=c['consumed_revisions'], actual_final_adoption=c['actual_final_adoption']) for c in matched_calls]))
    return dict(scope_records=scopes, selected_node_records=selections, requests=records,
        hint_valid_scope_count=sum(r.get('hint_valid') is True for r in scopes),
        intervention_active_scope_count=sum(r.get('active') is True for r in scopes),
        switch_penalty_evaluations=sum(r.get('switch_penalty_evaluations', 0) for r in scopes),
        zeroed_switch_penalty_evaluations=sum(r.get('zeroed_switch_penalty_evaluations', 0) for r in scopes),
        selected_chains_with_direction_change=sum(c.get('selected_direction_change_count', 0) > 0
            for r in records for c in r['selected_chains']),
        actual_refresh_delivery_allowances_s=row['delivery_window']['actual_refresh_allowances_s'],
        all_actual_planning_maxima_s=sorted(set(r['event']['planning_max_prediction_s']
            for r in row['delivery_window']['refresh_events'])),
        policy='Eligible short-prefix fallback search only: uniformly set gear-switch coefficient to zero within that search and restore afterward; not a first-switch-only credit.',
        scope='Actual hint eligibility, search cost use, selected chain, resampled output, MINCO initializer and final gate are distinct records. A requested intervention is not counted as a selected maneuver or delivered improvement.')


def load_online(source, label):
    row, trial = midcross.load_online(source, label)
    row['short_prefix_rejoin'] = intervention_analysis(row, trial)
    row['source_sha256'] = trial.reader.hashes
    trial.verify_unchanged()
    return row, trial


def conclusions(data):
    notes = [data['initial_pair_contract']['interpretation']]
    for row in data['rows']:
        effect = row['short_prefix_rejoin']
        notes.append(row['label'] + '：' + ('到达目标' if row['reached_goal'] else
            TERMINATIONS.get(row['termination'], str(row['termination']))) + '，' +
            ('发现' if row['physical_sample_collision'] else '未发现') + '物理球采样接触。')
        notes.append('  实走 %s 米、起步到结束 %s 秒；首次避障反馈→首条实际有效新控制 %s 秒，包含失败和重试。' % (
            show(row['actual_path_length_m'], 3), show(row['motion_duration_s'], 3),
            show(row['stage_timeline']['hint_send_to_first_new_validated_command_wall_s'], 3)))
        notes.append('  反向提示有效 %d 次，实际启用本次搜索换向代价调整 %d 次；换向成本计算 %d 次，其中置零 %d 次；选中链含换向 %d 条。' % (
            effect['hint_valid_scope_count'], effect['intervention_active_scope_count'],
            effect['switch_penalty_evaluations'], effect['zeroed_switch_penalty_evaluations'],
            effect['selected_chains_with_direction_change']))
        chain = row['delivery_chain_counts']
        notes.append('  请求→原生通过→发布→采用：%d→%d→%d→%d；最新动态拒绝 %d 次。' % (
            chain['refresh_requests'], chain['requests_with_native_optimizer_pass'],
            chain['refresh_reference_publications'], chain['refresh_references_consumed'], len(row['latest_rejections'])))
        notes.append('  实际慢层预测窗口 %s 秒；最新/MPPI 全局仍为 3 秒；MPPI 完整循环 P95/最大 %s/%s ms。' % (
            effect['all_actual_planning_maxima_s'], show(row['callback']['p95_ms'], 1), show(row['callback']['max_ms'], 1)))
    b = data['rows'][1]['short_prefix_rejoin']
    if not b['intervention_active_scope_count']:
        notes.append('B 没有实际命中该退化分支，本次不能判断换向代价调整对带圈初值是否有效。')
    elif not b['zeroed_switch_penalty_evaluations']:
        notes.append('B 虽启用提示，但未记录换向成本实际置零；不能把结果写成搜索行为已被该调整改变。')
    else:
        notes.append('B 提示条件和换向成本置零均被实际使用；接下来依据选中链、初值、最终可行候选与实走结果判断收益。')
    notes.append('本次每组一次路径中段横穿，首次参考相同，后续感知、位置和触发时序仍各自在线产生；不外推总体成功率。')
    return notes


def narrative(data):
    lines = ['短前缀退化后的换向衔接：实验结果', '',
        '当前关键任务：短前缀因为制动/换向失效后，让前端有机会沿新鲜 MPPI 提示选择合理换向，减少初值先接出大圈。',
        '双方慢层预测均为 4.5 秒，最新验收与 MPPI 为 3 秒。A 维持原搜索代价；B 只有短前缀且新鲜预测未来 1 秒明确反向时，本次回退搜索的换向系数从 10 临时置 0，退出后恢复。',
        '这次调整作用于整次符合条件的搜索，不是只优惠首次换向；倒退、转向代价和原有最终检查保留。', '',
        '主要结果'] + data['conclusions']
    for row in data['rows']:
        lines += ['', row['label']]
        for item in row['short_prefix_rejoin']['requests']:
            lines.append('q%d：' % item['request_id'])
            for scope in item['scopes']:
                lines.append('  提示有效 %s、实际启用 %s，原因 %s；当前档位 %s、提示档位 %s，提示反向时间 %s 秒。' % (
                    scope.get('hint_valid'), scope.get('active'), scope.get('reason'),
                    scope.get('current_gear'), scope.get('hint_gear'), show(scope.get('suggested_switch_dt_s'), 3)))
            for chain in item['selected_chains']:
                if chain['available']:
                    lines.append('  选中 A* 节点链档位 %s，换向 %d 次，包含提示档位 %s；这是重采样前选中链，并非全部扩展节点。' % (
                        chain['selected_direction_sequence'], chain['selected_direction_change_count'], chain['includes_hint_direction']))
            for field, name in (('search_resampled_paths', '搜索重采样路径'), ('initial_curves', 'MINCO 初值'), ('final_curves', '最终候选')):
                for result in item[field]:
                    g = result['geometry']
                    if g is None:
                        continue
                    lines.append('  %s：长度 %s 米，世界 x 最小 %s 米，y 范围 [%s,%s] 米，档位 %s。%s' % (
                        name, show(g['sampled_path_length_m'], 3), show(g['minimum_x_m'], 3),
                        show(g['minimum_y_m'], 3), show(g['maximum_y_m'], 3), g['gears'],
                        '原生通过 %s / %s。' % (result['native_success'], result['native_reason']) if field == 'final_curves' else ''))
            for adopted in item['actual_adoption']:
                lines.append('  计划序号 %d：实际有效采用 %s、消费版本 %s；未采用候选不计交付收益。' % (
                    adopted['plan_ordinal'], adopted['actually_adopted'], adopted['consumed_revisions']))
        evidence = row['execution_evidence']
        lines.append('有效控制与获准展开/制动唯一匹配 %d/%d 条；非零但未经有效发布 %d 条。' % (
            evidence['successful_exact_selected_and_brake_matches'], evidence['validated_command_count'],
            evidence['nonzero_without_validated_publication_count']))
    lines += ['', '口径',
        '世界 x 回退只是几何现象，不自动等同倒档或大圈；真实档位和 GIF 同时核对。',
        '换向代价计算/置零次数是搜索候选边的计算次数，不是机器人实际换挡次数；选中节点不含 analytic shot 插值，也不冒充全部搜索展开。',
        '原生最终通过、最新发布、参考消费与有效控制分别绑定；预算内无保底不自动等于最后没有可行轨迹。',
        '4.5 秒仍是慢层冻结副本的 CV 外推假设；本轮没有继续延长它。1.2 秒内部求解预算与下游最终验收耗时分别统计。',
        '固定机械臂、完整 REMANI＋底盘 MPPI 的 ROS 数值闭环，非 Gazebo、非实车。GIF 用真实参考、获准 MPPI 展开、CV 和 67 球投影；未记录的 MPPI 搜索候选不补画。', '']
    return '\n'.join(lines)


def chart(data, output):
    image = Image.new('RGB', (1180, 610), 'white')
    draw = ImageDraw.Draw(image)
    title, text, small = [ImageFont.truetype(FONT, n) for n in (23, 18, 15)]
    rows, colors = data['rows'], ('#2874a6', '#c47723')
    draw.text((20, 14), '短前缀退化后：换向提示是否改善实际路线', font=title, fill='#333333')
    draw.text((20, 52), data['initial_pair_contract']['interpretation'], font=small, fill='#555555')
    metrics = [('实走路程 / m', [r['actual_path_length_m'] for r in rows]),
        ('起步到结束 / s', [r['motion_duration_s'] for r in rows]),
        ('首次反馈→首条有效新控 / s', [r['stage_timeline']['hint_send_to_first_new_validated_command_wall_s'] for r in rows])]
    for column, (name, values) in enumerate(metrics):
        left, right = 55 + column * 390, 325 + column * 390
        upper = max([v for v in values if v is not None] + [.1]) * 1.25
        draw.text((left - 28, 96), name, font=small, fill='#333333')
        for fraction in (0., .5, 1.):
            x = left + fraction * (right - left)
            draw.line((x, 131, x, 275), fill='#e5e5e5')
            draw.text((x - 8, 284), show(upper * fraction, 2), font=small, fill='#777777')
        for group, value in enumerate(values):
            y = 146 + group * 69
            draw.text((left - 27, y + 7), 'AB'[group], font=text, fill=colors[group])
            if value is None:
                draw.text((left + 4, y + 7), '未交付', font=small, fill='#555555')
            else:
                x = left + value / upper * (right - left)
                draw.rectangle((left, y, x, y + 32), fill=colors[group])
                draw.text((x + 4, y + 7), show(value, 3), font=small, fill='#444444')
    draw.line((20, 321, 1160, 321), fill='#dddddd')
    for group, row in enumerate(rows):
        y = 340 + group * 93
        effect, chain = row['short_prefix_rejoin'], row['delivery_chain_counts']
        draw.text((20, y), 'AB'[group] + '　提示有效 %d；调整命中 %d；置零计算 %d；选中链换向 %d。' % (
            effect['hint_valid_scope_count'], effect['intervention_active_scope_count'],
            effect['zeroed_switch_penalty_evaluations'], effect['selected_chains_with_direction_change']), font=text, fill=colors[group])
        draw.text((20, y + 29), '请求→原生→发布→采用：%d→%d→%d→%d；最新拒绝 %d 次；%s物理球采样接触。' % (
            chain['refresh_requests'], chain['requests_with_native_optimizer_pass'], chain['refresh_reference_publications'],
            chain['refresh_references_consumed'], len(row['latest_rejections']), '发现' if row['physical_sample_collision'] else '未发现'),
            font=small, fill='#555555')
        draw.text((20, y + 56), 'MPPI 完整循环 P95/最大 %s/%s ms。' % (
            show(row['callback']['p95_ms'], 1), show(row['callback']['max_ms'], 1)), font=small, fill='#555555')
    draw.text((20, 542), '双方慢层 4.5s；最新验收/MPPI 3s。B 仅提示触发时临时调整整次搜索的换向代价。', font=small, fill='#666666')
    draw.text((20, 570), '每组一次中段横穿，在线输入非完全相同。有效交付统计包含失败和重试，几何回退不自动等于大圈。', font=small, fill='#666666')
    image.save(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=DEFAULT)
    parser.add_argument('--max-frames', type=int, default=70)
    parser.add_argument('--skip-gif', action='store_true')
    args = parser.parse_args()
    pairs = [load_online(args.root / 'midcross_A', LABELS[0]), load_online(args.root / 'midcross_B', LABELS[1])]
    rows, trials = zip(*pairs)
    add_common_motion_window(rows, trials)
    data = dict(schema='short_prefix_rejoin_report_v1', rows=list(rows),
        initial_pair_contract=midcross.delivery.initial_pair_contract(rows),
        same_native_binaries=rows[0]['binary_identity']['native'] == rows[1]['binary_identity']['native'],
        same_mppi_library=rows[0]['binary_identity']['mppi'] == rows[1]['binary_identity']['mppi'],
        scope='One unpaused midpoint crossing pair; both slow CV 4.5s, latest/MPPI 3s. Only eligible B short-prefix fallback searches temporarily remove the uniform gear-switch coefficient. Actual gates and final-adoption binding retained.')
    data['conclusions'] = conclusions(data)
    print('SHORT_PREFIX_REJOIN_CORE', json.dumps(data['conclusions'], ensure_ascii=False), flush=True)
    output = args.root / 'report'
    output.mkdir(parents=True, exist_ok=True)
    gif_name, keyframe_name = '短前缀衔接对照.gif', '短前缀衔接关键帧.png'
    if args.skip_gif:
        previous = json.loads((output / 'summary.json').read_text())
        artifact = dict(previous['artifacts']['midcross'])
        if sha(output / gif_name) != artifact['sha256'] or not (output / keyframe_name).is_file():
            raise ValueError('Existing GIF/keyframes missing or changed.')
        artifact['reused_without_rerender'] = True
        data['artifacts'] = dict(midcross=artifact)
    else:
        data['artifacts'] = dict(midcross=midcross.animation(rows, trials, output / gif_name, args.max_frames))
        keyframes(output / gif_name, output / keyframe_name)
    chart(data, output / '对照数据.png')
    (output / '实验结果.txt').write_text(narrative(data))
    (output / '本轮结论.txt').write_text('\n'.join(data['conclusions']) + '\n')
    for trial in trials:
        trial.verify_unchanged()
    data['report_script_sha256'] = sha(Path(__file__))
    data['inherited_midcross_report_sha256'] = sha(Path(midcross.__file__))
    (output / 'summary.json').write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    print('SHORT_PREFIX_REJOIN_REPORT', output, flush=True)


if __name__ == '__main__':
    main()
