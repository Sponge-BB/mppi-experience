#!/usr/bin/env python3
"""Read a completed late/early collision-repair experiment; no ROS or solves.

Both branches use the continuous fixed-window objective and one-shot repair.
Only the point at which ordinary optimization yields to repair is changed.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

from render_coupled_objective import sha
from render_dual_timescale import FONT
from report_candidate_collision_repair import dynamic_check
from report_frontend_rejoin import keyframes
from report_static_agility import show
from report_time_indexed_collision import OUTCOMES, TERMINATIONS
from report_window_continuity import animation, online, rows_at
from replay_temporal_collision import Candidate


HERE = Path(__file__).resolve().parent
DEFAULT = HERE/'results/early_collision_repair_20261101_01'
LABELS = ('A：最终失败后修复', 'B：提前进入修复')


def first(record, *keys):
    return next((record[k] for k in keys if k in record), None)


def boolean(value):
    if value in (True, 1, 'true', '1'):
        return True
    if value in (False, 0, 'false', '0'):
        return False
    return None


def read_replay(path):
    records = rows_at(path)
    branches = []
    for record in records:
        if record.get('kind') not in ('continuation', 'early_collision_repair_continuation',
                                     'repair_continuation', 'feasibility_continuation'):
            continue
        early = boolean(first(record, 'early', 'early_repair', 'early_collision_repair', 'early_yield_enabled'))
        if early is None:
            variant = first(record, 'variant', 'mode', 'branch')
            early = {'B': True, 'early': True, 'A': False, 'late': False}.get(variant)
        if early is None:
            raise ValueError('replay branch has no explicit late/early identity')
        gate = first(record, 'final_gate', 'verdict') or {}
        stages = record.get('stages', [])
        branches.append(dict(label=LABELS[early], early=early,
            budget_s=first(record, 'budget_s', 'remaining_allowance_s'),
            solver_wall_s=first(record, 'solver_wall_s', 'total_solver_wall_s')
                          if first(record, 'solver_wall_s', 'total_solver_wall_s') is not None else
                          sum(stage['solver_wall_s'] for stage in stages) if stages else None,
            total_through_final_gate_s=record.get('total_through_final_gate_s'),
            solver_exit=first(record, 'result', 'lbfgs_result') if not stages else stages[-1].get('result'),
            accepted_steps=record.get('accepted_steps') if not stages else
                           sum(stage['accepted_steps'] for stage in stages),
            initial_x_hash=record.get('initial_x_hash'),
            final_x_hash=record.get('final_x_hash') if not stages else stages[-1].get('final_x_hash'),
            same_common_scale=record.get('same_common_scale'),
            published=record.get('published'),
            native_pass=first(gate, 'native_pass', 'pass'),
            native_reason=first(gate, 'native_reason', 'reason'),
            native_rejection_time=first(gate, 'native_rejection_time', 'rejection_time'),
            native_pass_by_original_deadline=record.get('native_pass_by_original_deadline'),
            within_original_remaining_budget=record.get('within_original_remaining_budget'),
            shared_start_unchanged=record.get('shared_start_unchanged'),
            repair_attempted=record.get('repair_attempted'),
            early_yield_triggered=record.get('early_yield_triggered'),
            repair=first(record, 'repair', 'repair_summary'), raw=record))
    branches.sort(key=lambda r: r['early'])
    paired = len(branches) == 2 and [r['early'] for r in branches] == [False, True]
    same_x = (paired and branches[0]['initial_x_hash'] is not None and
              branches[0]['initial_x_hash'] == branches[1]['initial_x_hash'])
    budgets = [r['budget_s'] for r in branches]
    equal_budget = (paired and all(v is not None for v in budgets) and
                    abs(budgets[0]-budgets[1]) < 1e-12)
    common_scale = paired and all(r['same_common_scale'] is True for r in branches)
    exact_records = [r for r in records if r.get('kind') == 'original_exact_check']
    exact = len(exact_records) == 1 and all(exact_records[0].get(k) is True
                for k in ('x_exact', 'cost_exact', 'gradient_exact'))
    return dict(path=str(path.resolve()), sha256=sha(path) if path.is_file() else None,
        branches=branches, comparison=dict(two_branches=paired, same_initial_x=same_x,
            equal_budget=equal_budget, same_common_scale=common_scale,
            original_exact=exact, valid=exact and paired and same_x and equal_budget and common_scale),
        raw_records=records,
        scope='Same complete frozen native input, continuous fixed-window objective, same new coordinate scale and allowance; no control publication.')


def load_online(source, label):
    row, trial = online(source, label)
    timeline = trial.reader.jsonl('dual_timeline.jsonl')
    selected = [r for r in timeline if r.get('event') == 'early_collision_repair_yield'
                or r.get('event', '').startswith('candidate_collision_repair_')]
    yields = [r for r in selected if r['event'] == 'early_collision_repair_yield']
    begins = [r for r in selected if r['event'] == 'candidate_collision_repair_begin']
    ends = [r for r in selected if r['event'] == 'candidate_collision_repair_end']
    resets = [r for r in ends if r.get('original_budget_start_s') is not None and
              r.get('final_budget_start_s') is not None and
              abs(r['original_budget_start_s']-r['final_budget_start_s']) > 1e-9]
    unrecorded = [r for r in ends if r.get('original_budget_start_s') is None or
                  r.get('final_budget_start_s') is None]
    begin_requests = Counter(r.get('request_id') for r in begins)
    row['repair_events'] = selected
    row['repair_summary'] = dict(yield_count=len(yields), begin_count=len(begins),
        end_count=len(ends), native_pass_count=sum(boolean(r.get('native_pass')) is True for r in ends),
        budget_reset_count=len(resets), budget_start_unrecorded_count=len(unrecorded),
        maximum_repair_count_per_request=max(begin_requests.values(), default=0),
        shared_budget_values_s=sorted(set(float(r['shared_budget_s']) for r in selected
                                         if r.get('shared_budget_s') is not None)),
        begin_remaining_ms=[r.get('budget_remaining_ms') for r in begins],
        scope='Repair is a candidate feasibility attempt; native pass is not latest-CV publication or MPPI consumption.')
    row['repair_candidate_dynamic_checks'] = []
    captures = trial.reader.jsonl('temporal_collision.jsonl')
    plans = {r['plan_ordinal']: r for r in captures if r.get('stage') == 'planning'}
    for record in captures:
        if record.get('stage') not in ('before_repair', 'after_repair') or not record.get('candidate'):
            continue
        plan = plans.get(record['plan_ordinal'])
        if plan is None:
            continue
        candidate = Candidate(record['candidate'], plan)
        row['repair_candidate_dynamic_checks'].append(dict(stage=record['stage'],
            request_id=record['request_id'], plan_ordinal=record['plan_ordinal'],
            native_success=record.get('native_success'), native_reason=record.get('native_reason'),
            checks=[dynamic_check(candidate, plan, scale) for scale in (1., .5)]))
    row['repair_settings'] = {k: v for k, v in trial.manifest.items()
                             if 'repair' in k or 'window' in k}
    row['source_sha256'] = trial.reader.hashes
    trial.verify_unchanged()
    return row, trial


def delay(row):
    value = row['control_delivery']['first_refresh_to_new_reference_validated_command_wall_ms']
    return value/1000. if value is not None else None


def conclusions(replay, rows):
    notes = []
    if not replay['comparison']['valid']:
        notes.append('同输入分支的一致性没有全部确认，不据此判断提前修复的因果效果。')
    else:
        a, b = replay['branches']
        if b['native_pass_by_original_deadline'] is True and a['native_pass_by_original_deadline'] is not True:
            notes.append('同一输入、相同剩余预算下，提前修复通过完整原生验收，晚修复未通过；本输入的候选可行性得到改善。')
        elif b['native_pass'] is not True:
            notes.append('同输入的提前修复最终仍未通过完整原生验收，尚未解决该候选的可交付性。')
            if b['early_yield_triggered'] is True and b['repair_attempted'] is True:
                notes.append('提前交接给修复留下了更多计算时间，但增加修复更新仍没有消除碰撞，不能仅用“修复开始太晚”解释这个输入的失败。')
        elif b['native_pass_by_original_deadline'] is not True:
            notes.append('同输入的提前修复最终候选安全，但没有在原期限内满足可交付条件，不能算按时交付成功。')
        else:
            notes.append('同输入两分支均通过完整原生验收；仅从通过与否不能证明提前修复更好。')
    if len(rows) != 2:
        return notes+['缺少完整在线A/B，在线交付收益未验证。']
    a, b = rows
    if b['repair_summary']['yield_count'] == 0:
        notes.append('在线B没有提前修复交接事件，不能把A/B差异归因于提前修复。')
    da, db = delay(a), delay(b)
    if da is not None and db is not None:
        notes.append('首次重规划到新参考首条有效控制：A %.3f秒，B %.3f秒，差值%+.3f秒。' % (da, db, db-da))
    elif db is None:
        notes.append('在线B没有交付新参考的有效控制。')
    else:
        notes.append('在线B交付了新参考的有效控制，A没有；这是本轮单次闭环结果。')
    if any(r['online_pause_audit']['diagnostic_pause_detected'] for r in rows):
        notes.append('在线记录出现诊断暂停，性能比较受到干扰。')
    if any(r['repair_summary']['budget_reset_count'] for r in rows):
        notes.append('检测到修复预算起点改变，不能按相同原始预算比较。')
    if b['physical_sample_collision']:
        notes.append('在线B发生物理球采样接触，尚不能认为避障改善成立。')
    notes.append('这轮检查慢层能否更早产出快层可用的新参考；不代表已经实现完整的双时间尺度耦合优化。')
    return notes


def gap_text(check):
    minimum = check.get('minimum')
    return show(100*minimum['minimum_model_gap_m'], 2)+'厘米' if minimum else '无有效样本'


def narrative(data):
    rows, replay = data['rows'], data['replay']
    lines = ['提前修复碰撞候选：实验结果', '',
        '要解决的问题：REMANI在优化中已经发现候选碰撞，却继续普通优化，最后仍可能交不出轨迹。',
        'A：保留最终碰撞失败后修复。B：首个已接受、轮约束合格、完整原生检查仍有环境碰撞的候选，提前转入一次修复。',
        '两版都用连续固定预测窗目标、失败段环境软权重×10，合用原1.2秒预算；修复不重新计时。', '',
        '结论']+data['conclusions']+['', '同输入回放']
    lines.append('同起点/等预算/同坐标缩放：%s/%s/%s。' % tuple(
        '已确认' if replay['comparison'][k] else '未全部确认'
        for k in ('same_initial_x', 'equal_budget', 'same_common_scale')))
    for branch in replay['branches']:
        lines.append('%s：可用%s毫秒，实际求解%s毫秒，接受%s步；完整原生验收%s，原因%s。' % (
            branch['label'], show(branch['budget_s']*1000, 2) if branch['budget_s'] is not None else '未记录',
            show(branch['solver_wall_s']*1000, 2) if branch['solver_wall_s'] is not None else '未记录',
            branch['accepted_steps'] if branch['accepted_steps'] is not None else '未记录',
            '通过' if branch['native_pass'] is True else '未通过' if branch['native_pass'] is False else '未记录',
            branch['native_reason']))
        lines.append('含最终重建和验收%s毫秒；原期限内可交付%s；预算起点%s，提前交接%s，修复%s。' % (
            show(branch['total_through_final_gate_s']*1000, 2) if branch['total_through_final_gate_s'] is not None else '未记录',
            '是' if branch['native_pass_by_original_deadline'] is True else '否',
            '保持' if branch['shared_start_unchanged'] is True else '未确认保持',
            '触发' if branch['early_yield_triggered'] is True else '未触发',
            '触发' if branch['repair_attempted'] is True else '未触发'))
        for stage in branch['raw'].get('stages', []):
            trace = stage.get('accepted_trace', [])
            lines.append('  %s：求解%s毫秒，接受%d步；本阶段自身目标%s→%s。' % (
                '普通优化' if stage['name'] == 'normal_objective' else '碰撞修复',
                show(1000*stage['solver_wall_s'], 2), stage['accepted_steps'],
                show(stage.get('initial_cost'), 3), show(trace[-1].get('cost'), 3) if trace else '未记录'))
    lines.append('不同阶段碰撞权重不同，总代价值不能跨阶段比较；两分支最终都碰撞时，不能只把失败解释为检查耗时导致超期。')
    lines += ['', '正常在线迎面对照（每版一次，不暂停）']
    for row in rows:
        chain, repair = row['delivery_chain_counts'], row['repair_summary']
        status = '到达目标' if row['reached_goal'] else TERMINATIONS.get(row['termination'], str(row['termination']))
        lines += [row['label']+'：'+status+'。',
            '首次请求到有效新控制%s；请求→原生通过→发布→MPPI采用：%d→%d→%d→%d。' % (
                show(delay(row), 3)+'秒' if delay(row) is not None else '未交付',
                chain['refresh_requests'], chain['requests_with_native_optimizer_pass'],
                chain['refresh_reference_publications'], chain['refresh_references_consumed']),
            '提前交接%d次，修复开始/结束%d/%d次，修复后完整原生通过%d次；预算起点重置%d次，未记录%d次。' % (
                repair['yield_count'], repair['begin_count'], repair['end_count'], repair['native_pass_count'],
                repair['budget_reset_count'], repair['budget_start_unrecorded_count']),
            '实走%s米，最小物理采样间隙%s厘米，%s采样接触；全程非目标低速%s秒；MPPI整循环P95/最大%s/%s毫秒。' % (
                show(row['actual_path_length_m'], 3), show(100*row['minimum_physical_sample_gap_m'], 2),
                '发生' if row['physical_sample_collision'] else '未发现', show(row['non_goal_low_speed']['total_s']),
                show(row['callback']['p95_ms'], 1), show(row['callback']['max_ms'], 1))]
        adoption = row.get('first_adoption')
        if adoption and adoption.get('adoption_error'):
            error, after = adoption['adoption_error'], adoption['following_2s']
            lines.append('首次采用时偏差%s厘米/%s度；采用后实际记录%s秒内，低速%s秒、倒车%s秒。' % (
                show(100*error['position_m'], 2), show(error['yaw_abs_deg'], 2),
                show(after['duration_s']), show(after['measured_non_goal_low_speed']['total_s']),
                show(after['measured_reverse_duration_s'])))
        else:
            lines.append('未关联到有效新参考采用，采用偏差和随后2秒指标留空。')
        for request in row['remani']['requests']:
            if request.get('request_id', 0) > 1:
                lines.append('请求%d：%s，%s毫秒；优化/硬验收%s；发布拒绝%s。' % (
                    request['request_id'], OUTCOMES.get(request.get('outcome_class'), request.get('outcome_class')),
                    show(request.get('request_wall_ms'), 1),
                    [c.get('reason') for c in request.get('optimizer_calls', [])], request.get('acceptance_rejections', [])))
        for event in row['repair_events']:
            if event['event'] == 'candidate_collision_repair_end':
                lines.append('修复请求%d：耗时%s毫秒，接受%s步，%s；最终原因%s。' % (
                    event['request_id'], show(event.get('repair_solver_wall_ms'), 1), event.get('accepted_steps'),
                    '验收通过' if boolean(event.get('native_pass')) is True else '验收未通过', event.get('repair_reason')))
        for record in row['repair_candidate_dynamic_checks']:
            lines.append('请求%d %s：同冻结CV复算，常速间隙%s、半速%s。' % (
                record['request_id'], '修复前' if record['stage'] == 'before_repair' else '修复后',
                gap_text(record['checks'][0]), gap_text(record['checks'][1])))
        lines.append('')
    lines += ['范围说明',
        '半速动态检查属于此前新增的实验执行合同，不是原生REMANI固有要求；本轮两版保持一致，只比较修复时机。',
        '同冻结CV间隙只解释动态预测分支，不代替静态、自碰撞、轮约束和最新CV发布验收，也不是物理接触结果。',
        '同输入回放用于定位修复机制；在线同种子仍可能有异步输入差异，一组结果不代表总体稳定性。',
        '固定臂、差速底盘、完整REMANI＋MPPI的ROS数值闭环，非Gazebo/实车。GIF保留67球投影和真实记录搜索子集。',
        '完整事件、修复前后候选、采用残差及文件校验值保存在summary.json。', '']
    return '\n'.join(lines)


def chart(data, output):
    rows = data['rows']
    image = Image.new('RGB', (1180, 580), 'white')
    draw = ImageDraw.Draw(image)
    title, normal, small = [ImageFont.truetype(FONT, n) for n in (23, 18, 15)]
    colors = ('#2874a6', '#c47723')
    draw.text((20, 14), '相同预算：晚修复与提前修复', font=title, fill='#333333')
    draw.text((20, 51), 'A：最终碰撞失败才修复　B：已接受候选确认环境碰撞后提前修复；在线每组一次。', font=small, fill='#666666')
    values = [delay(row) for row in rows]
    limit = max(1., math.ceil(max([v for v in values if v is not None] or [1.])*1.15))
    draw.text((35, 103), '首次请求 → 有效新控制 / 秒', font=normal, fill='#333333')
    left, right = 70, 500
    for tick in range(int(limit)+1):
        x = left+tick/limit*(right-left)
        draw.line((x, 145, x, 310), fill='#e7e7e7')
        draw.text((x-5, 318), str(tick), font=small, fill='#777777')
    for i, value in enumerate(values):
        y = 164+76*i
        draw.text((35, y+7), 'AB'[i], font=normal, fill=colors[i])
        if value is None:
            draw.text((left+10, y+7), '未交付', font=small, fill='#555555')
        else:
            x = left+value/limit*(right-left)
            draw.rectangle((left, y, x, y+34), fill=colors[i])
            draw.text((x+7, y+7), show(value, 3), font=small, fill='#444444')
    draw.text((625, 103), '有效请求链 / 次', font=normal, fill='#333333')
    keys = ('refresh_requests', 'requests_with_native_optimizer_pass',
            'refresh_reference_publications', 'refresh_references_consumed')
    names, shades = ('请求', '原生', '发布', '采用'), ('#cccccc', '#8ba8bd', '#729b7f', '#2874a6')
    chains = [r['delivery_chain_counts'] for r in rows]
    maximum = max(1, max(c[k] for c in chains for k in keys))
    for i, chain in enumerate(chains):
        top = 149+82*i
        draw.text((598, top+17), 'AB'[i], font=normal, fill=colors[i])
        for j, key in enumerate(keys):
            y, x = top+17*j, 675+chain[key]/maximum*365
            draw.text((625, y), names[j], font=small, fill='#666666')
            if chain[key]:
                draw.rectangle((675, y+2, x, y+13), fill=shades[j])
            draw.text((x+7, y-1), str(chain[key]), font=small, fill='#555555')
    draw.line((20, 366, 1160, 366), fill='#dddddd')
    for i, row in enumerate(rows):
        top = 386+64*i
        repair = row['repair_summary']
        result = '到达' if row['reached_goal'] else TERMINATIONS.get(row['termination'], str(row['termination']))
        draw.text((20, top), '%s　%s；提前交接%d次；修复%d次，通过%d次；预算重置%d次' % (
            'AB'[i], result, repair['yield_count'], repair['begin_count'], repair['native_pass_count'],
            repair['budget_reset_count']), font=normal, fill=colors[i])
        draw.text((20, top+28), '实走 %.3f m；最小物理采样间隙 %.2f cm；MPPI整循环 P95 %s ms' % (
            row['actual_path_length_m'], 100*row['minimum_physical_sample_gap_m'],
            show(row['callback']['p95_ms'], 1)), font=small, fill='#555555')
    draw.text((20, 548), '原生通过 ≠ 最新CV发布 ≠ MPPI采用；完整67球投影，固定臂ROS数值闭环。', font=small, fill='#777777')
    image.save(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=DEFAULT)
    parser.add_argument('--replay', type=Path)
    parser.add_argument('--baseline', type=Path)
    parser.add_argument('--changed', type=Path)
    parser.add_argument('--max-frames', type=int, default=70)
    args = parser.parse_args()
    if args.max_frames < 2:
        raise ValueError('at least two animation frames required')
    replay = read_replay(args.replay or args.root/'replay/result.jsonl')
    loaded = [load_online(args.baseline or args.root/'headon_A', LABELS[0]),
              load_online(args.changed or args.root/'headon_B', LABELS[1])]
    rows, trials = zip(*loaded)
    data = dict(replay=replay, rows=list(rows), conclusions=conclusions(replay, rows),
        scope='Same-input late/early repair check plus fresh unpaused full REMANI+MPPI fixed-arm numerical ROS SIL A/B; no hardware.',
        contract='Both continuous fixed-window objective and environmental failed-piece weight x10 repair enabled; only yield timing differs. Half-speed check is an added experimental contract, not native REMANI requirement.')
    output = args.root/'report'
    output.mkdir(parents=True, exist_ok=True)
    data['artifacts'] = dict(headon=animation(rows, trials, output/'迎面对照.gif', args.max_frames))
    keyframes(output/'迎面对照.gif', output/'迎面关键帧.png')
    chart(data, output/'对照数据.png')
    (output/'实验结果.txt').write_text(narrative(data), encoding='utf8')
    for trial in trials:
        trial.verify_unchanged()
    data['report_script_sha256'] = sha(Path(__file__))
    (output/'summary.json').write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print('EARLY_COLLISION_REPAIR_REPORT', output, flush=True)


if __name__ == '__main__':
    main()
