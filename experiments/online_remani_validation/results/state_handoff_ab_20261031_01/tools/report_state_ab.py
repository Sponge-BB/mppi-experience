#!/usr/bin/env python3
"""Read the completed eight-run state-reception A/B batch; never run control."""
import argparse
from collections import Counter
import importlib.util
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve()
ROOT = HERE.parents[1]
EXPERIMENT = HERE.parents[3]
sys.path.insert(0, str(EXPERIMENT))

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from render_coupled_objective import sha
from render_dual_timescale import FONT
from report_long_goal_crossing import panel

SPEC = importlib.util.spec_from_file_location('state_ab_followup_reader',
    EXPERIMENT/'results/delivery_handoff_followup_20261031_01/tools/report_followup.py')
FOLLOWUP = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(FOLLOWUP)

SCENES = {'static': '静态挡路', 'midcross': '中段横穿'}
GROUPS = {'A': 'A 默认队列接收', 'B': 'B 独立接收、主线程消费'}
STAGES = ('before_solve', 'after_solve_before_refresh', 'after_solve_refreshed',
          'acceptance_entry', 'publication_entry')


def show(value, digits=3):
    return '无记录' if value is None else ('%.*f' % (digits, value))


def angle(value):
    return float(math.atan2(math.sin(value), math.cos(value)))


def actual_at(state, sim):
    if sim is None:
        return None
    t = float(sim)-1000.
    index = int(np.argmin(np.abs(state['t']-t)))
    row = state[index]
    return dict(sim_s=float(row['t']), sample_minus_event_s=float(row['t']-t),
        x=float(row['x']), y=float(row['y']), yaw=float(row['yaw']),
        v=float(row['v']), w=float(row['w']))


def event_state(event, state):
    """Applied raw odometry is distinct from regularized optimizer velocity."""
    if event is None:
        return None
    out = dict(event)
    sim = event.get('capture_sim', event.get('sim'))
    stamp = event.get('applied_stamp')
    out['computed_applied_age_s'] = None if sim is None or stamp is None else float(sim-stamp)
    age = event.get('applied_age_s')
    out['reported_age_consistent'] = (None if age is None or out['computed_applied_age_s'] is None else
        abs(float(age)-out['computed_applied_age_s']) < 1e-6)
    received_stamp = event.get('latest_received_stamp')
    out['latest_received_message_age_s'] = (None if sim is None or received_stamp is None or received_stamp == 0 else
        float(sim-received_stamp))
    receipt = event.get('receiver_receipt_ros')
    out['receiver_to_event_ros_s'] = (None if sim is None or receipt is None or receipt == 0 else float(sim-receipt))
    receipt_mono = event.get('receiver_receipt_mono')
    capture_mono = event.get('capture_mono', event.get('mono'))
    out['receiver_to_event_wall_s'] = (None if capture_mono is None or receipt_mono is None or receipt_mono == 0 else
        float(capture_mono-receipt_mono))
    truth = actual_at(state, sim)
    out['nearest_recorded_actual'] = truth
    if truth is not None and all(event.get(k) is not None for k in ('applied_x', 'applied_y', 'applied_yaw')):
        out['applied_to_actual_distance_m'] = float(np.hypot(
            truth['x']-event['applied_x'], truth['y']-event['applied_y']))
        out['applied_to_actual_yaw_error_rad'] = angle(truth['yaw']-event['applied_yaw'])
        out['raw_body_v_error_m_s'] = (None if event.get('applied_body_vx') is None else
            float(truth['v']-event['applied_body_vx']))
        out['raw_w_error_rad_s'] = (None if event.get('applied_w') is None else
            float(truth['w']-event['applied_w']))
    return out


def handoff_states(timeline, state):
    events = [event for event in timeline if event.get('event') == 'handoff_state']
    ids = sorted({event['request_id'] for event in events if 'request_id' in event})
    requests = []
    for request_id in ids:
        related = [event for event in events if event.get('request_id') == request_id]
        by_stage = {}
        for stage in STAGES:
            items = [event for event in related if event.get('stage') == stage]
            by_stage[stage] = event_state(items[-1], state) if items else None
        before = by_stage['before_solve']
        refreshed = by_stage['after_solve_refreshed']
        frozen_fields = ('selected_boundary_x', 'selected_boundary_y', 'selected_boundary_yaw')
        checked = [event for event in by_stage.values() if event is not None
                   and all(event.get(key) is not None for key in frozen_fields)]
        frozen = None
        max_boundary_distance = max_boundary_yaw = None
        if before and all(before.get(key) is not None for key in frozen_fields) and checked:
            max_boundary_distance = max(float(np.hypot(event['selected_boundary_x']-before['selected_boundary_x'],
                event['selected_boundary_y']-before['selected_boundary_y'])) for event in checked)
            max_boundary_yaw = max(abs(angle(event['selected_boundary_yaw']-before['selected_boundary_yaw']))
                for event in checked)
            frozen = max_boundary_distance < 1e-9 and max_boundary_yaw < 1e-9
        def growth(key):
            if before is None or refreshed is None or before.get(key) is None or refreshed.get(key) is None:
                return None
            return int(refreshed[key])-int(before[key])
        requests.append(dict(request_id=request_id, stages=by_stage,
            stage_counts=dict(Counter(event.get('stage') for event in related)),
            independent_odom=bool(before.get('independent_odom')) if before else None,
            selected_boundary_unchanged=frozen, max_selected_boundary_distance_m=max_boundary_distance,
            max_selected_boundary_yaw_difference_rad=max_boundary_yaw,
            received_sequence_growth_during_solve=growth('latest_received_sequence'),
            consumed_sequence_growth_at_refresh=growth('consumed_sequence'),
            definition='Freeze optimization boundary throughout solve; receive mailbox may advance independently. '
                'after_solve_refreshed describes main-thread consumption, not concurrent FSM mutation.'))
    return requests


def adopted_projection(trial):
    """Measure recorded first adoption; do not invoke MPPI or create actions."""
    metadata = trial.reader.json('native3d_event_input.json', {})
    path = trial.reader.locate('native3d_event_input.npz')
    if path is None or metadata.get('event') != 'first_replan_reference_adoption':
        return dict(available=False, recorded_event=metadata.get('event'))
    if metadata.get('npz_sha256') != sha(path):
        raise ValueError('Adoption snapshot hash differs: '+str(path))
    with np.load(path, allow_pickle=False) as data:
        measured = data['state'].copy()
        reference = data['reference'].copy()
        times = data['reference_times'].copy()
    if len(times) != len(reference) or len(reference) < 2 or reference.shape[1] < 4:
        raise ValueError('Incomplete recorded adoption reference: '+str(path))
    if np.any(np.diff(times) < 0):
        raise ValueError('Nonmonotone native reference times')
    debug = metadata['cusp_debug']
    begin, end = int(debug[3]), int(debug[4])
    if not 0 <= begin < end < len(reference):
        raise ValueError('Invalid measured-phase bounds')
    best = None
    for index in range(begin, end):
        a, b = reference[index, :2], reference[index+1, :2]
        d = b-a
        length2 = float(d@d)
        if length2 < 1e-18:
            continue
        fraction = float(np.clip((measured[:2]-a)@d/length2, 0., 1.))
        point = a+fraction*d
        distance2 = float((measured[:2]-point)@(measured[:2]-point))
        if best is None or distance2 < best[0]:
            best = (distance2, index, fraction, point)
    if best is None:
        raise ValueError('No nonzero segment in actual adopted phase')
    distance2, index, fraction, point = best
    ref_yaw = angle(float(reference[index, 2])+fraction*angle(float(reference[index+1, 2]-reference[index, 2])))
    ref_v = float(reference[index, 3]+fraction*(reference[index+1, 3]-reference[index, 3]))
    projected_time = float(times[index]+fraction*(times[index+1]-times[index]))
    dt = float(times[index+1]-times[index])
    ref_w = angle(float(reference[index+1, 2]-reference[index, 2]))/dt if dt > 1e-9 else None
    age = float(metadata['solve_ros_time']-metadata['guide_start'])
    return dict(available=True, event=metadata['event'], npz_sha256=sha(path),
        epoch=metadata.get('epoch'), revision=metadata.get('revision'),
        first_solve_ros_time=metadata['solve_ros_time'], measured_state=measured.tolist(),
        phase_begin=begin, phase_end=end, nearest_segment=index, fraction=fraction,
        reference_start_distance_m=float(np.linalg.norm(measured[:2]-reference[0, :2])),
        projected_distance_m=float(np.sqrt(distance2)), projected_xy=point.tolist(),
        projected_yaw_rad=ref_yaw, measured_minus_projected_yaw_rad=angle(float(measured[2])-ref_yaw),
        projected_signed_speed_m_s=ref_v,
        measured_minus_projected_speed_m_s=float(measured[3]-ref_v),
        projected_local_angular_rate_rad_s=ref_w,
        measured_minus_projected_angular_rate_rad_s=None if ref_w is None else float(measured[4]-ref_w),
        projected_reference_time_s=projected_time, original_clock_age_s=age,
        geometric_progress_minus_original_clock_s=projected_time-age,
        semantics='Actual first adopted solve input; no-hint nearest segment within measured gear phase. '
            'Native polynomial speed and local yaw rate are descriptive reference quantities. '
            'MPPI uses geometric progress; differences alone do not establish a command discontinuity or unsafe handoff. '
            'No controller rerun; sampler/RNG history is not reconstructed.')


def first_replan(row):
    return next((episode for episode in row['delivery_episodes'] if episode.get('publish_mono') is not None), {})


def first_handoff(row):
    request_id = first_replan(row).get('publish_request_id')
    return next((item for item in row['handoff_state_requests'] if item['request_id'] == request_id), {})


def stage_age(row, stage):
    event = first_handoff(row).get('stages', {}).get(stage)
    return event.get('applied_age_s') if event else None


def low_speed_relationship(row, timeline):
    triggers = [float(event['sim'])-1000. for event in timeline if event.get('event') == 'refresh_trigger']
    first_trigger = min(triggers) if triggers else None
    adoption = row['adoption_projection']
    adopted_t = float(adoption['first_solve_ros_time'])-1000. if adoption.get('available') else None
    for interval in row['measured_low_speed']['intervals']:
        start, end = interval['start_sim_s'], interval['end_sim_s']
        interval['relative_to_first_refresh'] = ('no_refresh_trigger' if first_trigger is None else
            'before_first_refresh' if end <= first_trigger else
            'after_first_refresh' if start >= first_trigger else 'straddles_first_refresh')
        interval['overlaps_adoption_window_250ms'] = (None if adopted_t is None else
            start <= adopted_t+.25 and end >= adopted_t-.25)
    return dict(first_refresh_sim_s=first_trigger, first_adoption_sim_s=adopted_t,
        low_speed_intervals_before_first_refresh=sum(item['relative_to_first_refresh']=='before_first_refresh'
            for item in row['measured_low_speed']['intervals']),
        low_speed_intervals_overlapping_optimizer=sum(bool(item['overlapping_optimizer_request_ids'])
            for item in row['measured_low_speed']['intervals']),
        low_speed_intervals_near_first_adoption=sum(item['overlaps_adoption_window_250ms'] is True
            for item in row['measured_low_speed']['intervals']),
        note='Temporal overlap only; no causal attribution to backend waiting or state age.')


def summarize(path, job):
    group = job['group']
    repeat = int(job.get('repeat', job.get('repeat_index', 0)))
    if not repeat:
        repeat = int(path.name.split('_repeat')[1].split('_')[0])
    row, trial = FOLLOWUP.summarize(path, '%s%d' % (group, repeat), True)
    timeline = trial.reader.jsonl('dual_timeline.jsonl')
    states = handoff_states(timeline, trial.state)
    lifecycle = []
    for request in row['requests']:
        related = [event for event in timeline if event.get('request_id') == request['request_id']]
        initialization = next((event for event in related if event.get('event') == 'stage_end'
            and event.get('stage') == 'initialization'), None)
        manager_end = next((event for event in related if event.get('event') == 'stage_end'
            and event.get('stage') == 'manager_replan'), None)
        lifecycle.append(dict(request_id=request['request_id'],
            optimizer_entered=request.get('optimizer_begin_mono') is not None,
            optimizer_final_pass=request['backend_final_pass'],
            initialization_end=initialization, manager_end=manager_end,
            acceptance_rejection=request.get('latest_rejection'),
            acceptance_reject_details=request.get('latest_reject_details', []),
            published=request.get('publish_mono') is not None))
    row.update(scene=job['scene'], group=group, repeat=repeat,
        algorithm_configuration='common early feasible delivery/cusp phase/scaling; odometry reception and postsolve main-thread refresh A/B',
        handoff_state_requests=states, adoption_projection=adopted_projection(trial),
        planning_request_count=len(lifecycle),
        optimizer_invocations=sum(item['optimizer_entered'] for item in lifecycle),
        failed_before_optimizer=sum(not item['optimizer_entered'] and item['manager_end'] is not None
            and item['manager_end'].get('success') is False for item in lifecycle),
        request_lifecycle=lifecycle)
    # The reused parser's historical backend_calls counts manager requests,
    # including a frontend failure. Keep it for provenance, but never use it
    # as the number of optimizer invocations in the new narrative.
    row['backend_calls_legacy_semantics'] = 'number of manager planning requests; includes requests that never entered optimizer'
    row['low_speed_relationship'] = low_speed_relationship(row, timeline)
    expected = group == 'B'
    switches = {item['independent_odom'] for item in states if item['independent_odom'] is not None}
    if not states or switches != {expected}:
        raise ValueError('Missing or inconsistent state-reception switch in '+str(path))
    row['stage_state_ages_s'] = {stage: stage_age(row, stage) for stage in STAGES}
    row['selected_boundaries_unchanged'] = all(item['selected_boundary_unchanged'] is True for item in states)
    row['state_age_consistency_failures'] = sum(event.get('reported_age_consistent') is False
        for item in states for event in item['stages'].values() if event)
    row['input_sha256'] = dict(trial.reader.hashes)
    trial.verify_unchanged()
    return row, trial


def make_chart(rows, path):
    image = Image.new('RGB', (1260, 716), 'white')
    draw = ImageDraw.Draw(image)
    title, font, small = [ImageFont.truetype(FONT, size) for size in (22, 16, 14)]
    draw.text((20, 12), '状态接收 A/B：同求解预算、碰撞模型与轮约束', font=title, fill='#333333')
    draw.text((20, 46), 'A 默认队列；B 独立接收、主线程消费。每组每场景两次；按实际记录统计，不重算控制。', font=font, fill='#555555')
    columns = [(210, '验收状态龄 / s'), (410, '触发→采用 / s'), (620, '采用投影误差 / cm'),
               (850, '速度差 / m/s'), (1050, '低速累计 / 最长 s')]
    for x, text in columns:
        draw.text((x, 100), text, font=small, fill='#444444')
    for index, row in enumerate(rows):
        y = 141+index*56
        if index in (0, 4):
            draw.line((20, y-13, 1238, y-13), fill='#dddddd')
        color = '#4e7c9d' if row['group'] == 'B' else '#777777'
        draw.text((20, y), SCENES[row['scene']]+' '+row['label'], font=font, fill=color)
        adoption = row['adoption_projection']
        values = [show(stage_age(row, 'acceptance_entry')),
            show(first_replan(row).get('trigger_to_first_kernel_wall_s')),
            show(100*adoption['projected_distance_m'], 2) if adoption.get('available') else '无记录',
            show(adoption.get('measured_minus_projected_speed_m_s')),
            show(row['measured_low_speed']['cumulative_s'])+' / '+show(row['measured_low_speed']['longest_s'])]
        for (x, _), value in zip(columns, values):
            draw.text((x, y), value, font=font, fill=color)
        outcome = ('到达' if row['reached_goal'] else '未到达：'+row['termination'])
        draw.text((20, y+25), '后端%d/%d，规划请求%d；发布/采用%d/%d；%s；最小物理球采样间隙%.3fm' % (
            row['backend_final_passes'], row['optimizer_invocations'], row['planning_request_count'], row['reference_publications'],
            row['references_actually_adopted'], outcome, row['minimum_physical_sample_gap_m']), font=small, fill='#666666')
    draw.text((20, 600), '状态龄0：与当前仿真时钟同帧，不表示零传输延迟。后端分母只含真正进入优化的请求。', font=small, fill='#777777')
    draw.text((20, 626), '采用误差：同换向阶段最近线段；速度差=实际速度−该处原生参考速度，不能单独判定控制跳变。', font=small, fill='#777777')
    draw.text((20, 653), '低速：起步后、距目标>0.1m，|v|<0.02m/s且|ω|<0.05rad/s。状态龄按消息时间戳计算。', font=small, fill='#777777')
    draw.text((20, 680), '固定机械臂，67球。完整REMANI＋底盘MPPI ROS数值闭环，非Gazebo/实车。', font=small, fill='#777777')
    image.save(path)


def make_gif(rows, trials, path, scene):
    """Preselected repeat 1 only, actual measured records and actual termination."""
    duration = max(trial.end-trial.anchor for trial in trials)
    ages = np.linspace(0., duration, min(90, max(2, math.ceil(duration/.5)+1)))
    adopted = [row['adoption_projection'].get('first_solve_ros_time') for row in rows]
    extras = [float(t)-1000.-trial.anchor for t, trial in zip(adopted, trials) if t is not None]
    ages = np.unique(np.r_[ages, extras])
    title, small = [ImageFont.truetype(FONT, size) for size in (19, 14)]
    frames = []
    for trial in trials:
        trial.native_validity = []
    for age in ages:
        image = Image.new('RGB', (1160, 510), 'white')
        draw = ImageDraw.Draw(image)
        draw.text((18, 9), '%s A/B：起步后 %.1fs（预选第1次重复）' % (SCENES[scene], age), font=title, fill='#333333')
        draw.text((18, 39), '蓝：实际机器人/轨迹　橙：REMANI参考　绿：获准局部展开　紫：CV预测', font=small, fill='#555555')
        for index, (row, trial, adopt_time) in enumerate(zip(rows, trials, adopted)):
            left = index*580+18
            t = min(trial.anchor+float(age), trial.end)
            draw.text((left, 69), GROUPS[row['group']], font=title, fill='#333333')
            state = panel(image, trial, t, (left, 107, (index+1)*580-18, 414))
            draw = ImageDraw.Draw(image)
            if t >= trial.end-1e-9:
                status = '已到达目标' if row['reached_goal'] else '记录结束：'+str(row['termination'])
            else:
                status = trial.status_at(t)
                if status == 'ok':
                    status = '局部控制有效'
            adopted_now = adopt_time is not None and t >= float(adopt_time)-1000.
            draw.text((left, 423), '%s　v=%+.2f m/s　新参考%s' % (
                status, state['v'], '已采用' if adopted_now else '待采用'), font=small, fill='#555555')
        draw.text((18, 461), '两组各自实际起步对齐；67球投影，结束后不续画未来；未记录搜索线，不生成示意搜索。', font=small, fill='#777777')
        draw.text((18, 484), 'ROS数值闭环，非Gazebo/实车；2倍播放；停顿与失败画面保留。', font=small, fill='#777777')
        frames.append(image.convert('P', palette=Image.ADAPTIVE, colors=96))
    durations = [max(30, 10*round(float(dt)*500./10.)) for dt in np.diff(ages)]+[1000]
    frames[0].save(path, save_all=True, append_images=frames[1:], duration=durations,
        loop=0, disposal=2, optimize=True)
    for trial in trials:
        trial.verify_unchanged()
    return dict(scene=scene, repeat=1, frames=len(frames), speedup=2.,
        actual_records_only=True, controller_reexecuted=False, record_search=False,
        trials=[row['trial'] for row in rows], sha256=sha(path))


def narrative(rows):
    lines = ['状态接收与轨迹交接 A/B 实验',
        '要检查的问题：REMANI计算期间位置缓存是否变旧；修复状态接收后，交付和衔接是否改善。',
        'A：默认队列接收里程计。B：独立线程只收消息；求解结束由主线程消费，再做验收和发布。',
        '两组每次求解起点都固定。没有在求解过程中不断修改边界；不是把整个规划器改成并行。',
        '两组保持提前可行交付、换向段关联、廉价缩放、1.2秒预算、67球和轮约束；每场景每组两次。',
        '完整REMANI＋底盘MPPI，固定机械臂，ROS数值闭环；不是Gazebo或实车。','']
    for row in rows:
        episode = first_replan(row)
        adoption = row['adoption_projection']
        handoff = first_handoff(row)
        lines += [SCENES[row['scene']]+' '+row['label']+'：'+('到达目标。' if row['reached_goal'] else '未到目标，原因：'+row['termination']+'。'),
            '规划请求%d次；后端通过%d/%d，另%d次未进入后端；发布%d条，MPPI实际采用%d条，获准控制覆盖%d条；下游拒收%d次。' % (
                row['planning_request_count'], row['backend_final_passes'], row['optimizer_invocations'],
                row['failed_before_optimizer'], row['reference_publications'],
                row['references_actually_adopted'], row['references_with_validated_command'], sum(row['latest_rejection_counts'].values())),
            '首次重规划触发到采用%s秒，到首条获准指令%s秒。' % (
                show(episode.get('trigger_to_first_kernel_wall_s')), show(episode.get('trigger_to_validated_command_wall_s'))),
            '首次发布重规划的状态龄：求解前%s秒；返回尚未刷新%s秒；主线程刷新后%s秒；验收入口%s秒；发布入口%s秒。' % tuple(
                show(stage_age(row, stage)) for stage in STAGES),
            '本次求解固定边界是否始终保持：%s。所有请求的边界均固定：%s。' % (
                '是' if handoff.get('selected_boundary_unchanged') else '未确认或发生变化',
                '是' if row['selected_boundaries_unchanged'] else '未确认或发生变化'),
            '求解期间接收序号增加%s，刷新后消费序号增加%s。' % (
                str(handoff.get('received_sequence_growth_during_solve', '无记录')),
                str(handoff.get('consumed_sequence_growth_at_refresh', '无记录')))]
        if adoption.get('available'):
            lines += ['首次实际采用时：距同换向段参考%.2f厘米，朝向差%.2f度；实际速度%+.3f米/秒，投影处参考速度%+.3f米/秒，差%+.3f米/秒。' % (
                adoption['projected_distance_m']*100., adoption['measured_minus_projected_yaw_rad']*180./math.pi,
                adoption['measured_state'][3], adoption['projected_signed_speed_m_s'], adoption['measured_minus_projected_speed_m_s']),
                '距原始新轨迹起点%.2f厘米。这是起点距离，不能当作当前横向偏差。' % (adoption['reference_start_distance_m']*100.)]
        else:
            lines.append('没有首次实际采用的完整NPZ，采用位姿/速度误差不填0。')
        low = row['measured_low_speed']
        relation = row['low_speed_relationship']
        lines += ['非目标低速累计%s秒，最长%s秒；%d段在首次重规划触发之前，%d段与优化时段重叠，%d段靠近首次采用前后0.25秒。' % (
            show(low['cumulative_s']), show(low['longest_s']), relation['low_speed_intervals_before_first_refresh'],
            relation['low_speed_intervals_overlapping_optimizer'], relation['low_speed_intervals_near_first_adoption']),
            '实走%.2f米，运动%.2f秒；最小物理球采样间隙%.3f米，%s采样碰撞。' % (
                row['measured_travel_m'], row['motion_duration_s'], row['minimum_physical_sample_gap_m'],
                '发生' if row['physical_sample_collision'] else '未发现'), '']
        for item in row['handoff_state_requests']:
            lines.append('  请求%d%s：刷新/验收/发布状态龄%s / %s / %s秒；边界%s。' % (
                item['request_id'], '（初始规划）' if item['request_id']==1 else '',
                *[show((item['stages'].get(stage) or {}).get('applied_age_s')) for stage in
                  ('after_solve_refreshed', 'acceptance_entry', 'publication_entry')],
                '保持固定' if item['selected_boundary_unchanged'] else '未确认或变化'))
        for item in row['request_lifecycle']:
            if not item['optimizer_entered']:
                lines.append('  请求%d：没有进入后端优化；初始化结束%s，规划返回%s。不能算成后端优化不收敛。' % (
                    item['request_id'], str((item['initialization_end'] or {}).get('success', '无记录')),
                    str((item['manager_end'] or {}).get('success', '无记录'))))
            rejected = item['acceptance_rejection']
            if rejected:
                details = item['acceptance_reject_details']
                lines.append('  请求%d：后端%s，但下游拒收，原因%s%s；没有发布和采用该条候选。' % (
                    item['request_id'], '最终通过' if item['optimizer_final_pass'] else '未最终通过',
                    str(rejected.get('reason', '无记录')),
                    '，详细分支'+', '.join(str(event.get('branch')) for event in details) if details else ''))
        lines.append('')
    lines += ['如何判断：',
        '先看B的刷新/验收/发布状态龄是否降低、序号是否增长、求解起点是否固定。这只验证状态接收机制。',
        '再看实际采用、交付时间、交接误差、低速和安全是否共同改善。若仅状态更新更快，不能说双时间尺度优化已解决。',
        '低速与求解或交接在时间上重叠，不足以证明因果；发生在触发前的低速不能归为等待此次REMANI。',
        '采用误差按实际新参考、实测状态和当前换向阶段求最近线段；MPPI按几何进度跟随，速度差本身不等于控制跳变。',
        '位置缓存使用原始里程计时间戳和速度；processed_vx/vy可能经过低速正则化，不冒充实测。',
        'B的真实消息接收时刻看receiver_receipt_ros/mono；旧state_contract接收时刻仍可能是主线程消费时刻。A消息槽序号为0是正常现象。',
        '没有增加交接拒收阈值。状态更新变新不代表接续问题已经解决；机械臂仍在默认队列且固定，本轮不能外推活动机械臂。',
        '消息时间戳年龄与接收后等待时间不同，SIL还存在时钟调度差；保持原始正负值，不人为截为0。',
        'B状态龄为0表示与当前SIL发布tick同时间戳，不是承诺实车零延迟；本轮初始规划另有一次约20毫秒。',
        '每组每场景两次，用于机制与现象检查，不据此宣称总体成功率。GIF预先固定第1次重复，不挑最好看的。',
        '碰撞结论限于原生物理球和有限高障碍/地图边界的离散采样与在线子步监测，不是完整网格连续碰撞证明。',
        '所有控制来自实际在线记录；本报告没有重新运行MPPI，也不补画缺失搜索线。','']
    return '\n'.join(lines)


def report(batch, out=None, gifs=True):
    batch = Path(batch).resolve()
    out = Path(out).resolve() if out else batch/'report'
    if not (batch/'complete.json').is_file():
        raise ValueError('The timed eight-run batch must be complete before rendering')
    plan = json.loads((batch/'plan.json').read_text())
    jobs = plan['jobs']
    if len(jobs) != 8:
        raise ValueError('Expected all eight original declared jobs')
    normalized = []
    for original in jobs:
        job = dict(original)
        if 'group' not in job:
            job['group'] = 'B' if job.get('independent_odom') is True else 'A'
        job.setdefault('repeat', int(job['trial'].split('_repeat')[1].split('_')[0]))
        normalized.append(job)
    declared = {(job['scene'], job['group'], int(job['repeat'])) for job in normalized}
    expected = {(scene, group, repeat) for scene in SCENES for group in GROUPS for repeat in (1, 2)}
    if declared != expected:
        raise ValueError('Declared groups/scenes/repetitions differ from original A/B plan')
    rows, trials = [], {}
    for job in normalized:
        row, trial = summarize(batch/job['trial'], job)
        rows.append(row)
        trials[(row['scene'], row['group'], row['repeat'])] = trial
    rows.sort(key=lambda row: (list(SCENES).index(row['scene']), row['repeat'], row['group']))
    out.mkdir(parents=True, exist_ok=True)
    make_chart(rows, out/'状态交接AB对照.png')
    visualizations = []
    if gifs:
        for scene in SCENES:
            pair = [next(row for row in rows if (row['scene'], row['group'], row['repeat']) == (scene, group, 1))
                    for group in GROUPS]
            selected = [trials[(scene, group, 1)] for group in GROUPS]
            visualizations.append(make_gif(pair, selected, out/(SCENES[scene]+'AB.gif'), scene))
    elif (out/'summary.json').exists():
        previous = json.loads((out/'summary.json').read_text())
        for item in previous.get('visualizations', []):
            path = out/(SCENES[item['scene']]+'AB.gif')
            if not path.exists() or sha(path) != item['sha256']:
                raise ValueError('Previously rendered GIF missing or changed: '+str(path))
            visualizations.append(item)
    for trial in trials.values():
        trial.verify_unchanged()
    summary = dict(scope='eight fresh complete numerical ROS SIL runs, fixed arm; no hardware/Gazebo',
        group_difference='A default callback queue; B independent immutable-message receive mailbox consumed on main thread after solve',
        no_solve_boundary_mutation=True, no_predictive_mppi_boundary_feedback=True,
        no_new_handoff_rejection_threshold=True, arm_fixed_and_default_queue=True,
        plan_sha256=sha(batch/'plan.json'), report_sha256=sha(HERE),
        reused_reader_sha256=sha(Path(FOLLOWUP.__file__)), trials=rows, visualizations=visualizations,
        caution='Low-speed overlap and reference speed differences are observations, not independent causal proof.')
    (out/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    (out/'实验结果.txt').write_text(narrative(rows)+'\n')
    print(json.dumps(dict(report=str(out), trials=[dict(scene=row['scene'], label=row['label'],
        goal=row['reached_goal'], acceptance_age=stage_age(row, 'acceptance_entry'),
        trigger_to_adoption=first_replan(row).get('trigger_to_first_kernel_wall_s'),
        adoption_error_m=row['adoption_projection'].get('projected_distance_m'),
        backend_pass=row['backend_final_passes'], optimizer_invocations=row['optimizer_invocations'],
        planning_requests=row['planning_request_count'],
        publications=row['reference_publications'], adoptions=row['references_actually_adopted'],
        longest_low_speed_s=row['measured_low_speed']['longest_s']) for row in rows]), ensure_ascii=False))
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch', type=Path, default=ROOT)
    parser.add_argument('--out', type=Path)
    parser.add_argument('--no-gifs', action='store_true', help='Do not render GIFs; preserve and verify existing GIF metadata when present')
    args = parser.parse_args()
    report(args.batch, args.out, not args.no_gifs)
