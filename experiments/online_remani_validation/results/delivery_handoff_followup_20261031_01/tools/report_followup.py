#!/usr/bin/env python3
"""Read completed follow-up trials; never run ROS or generate controller actions."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve()
EXPERIMENT = HERE.parents[3]
sys.path.insert(0, str(EXPERIMENT))

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from render_coupled_objective import SourceReader, sha
from render_dual_timescale import FONT
from render_native_collision_3d import Trial
from report_cheap_scaling_online import delivery_episodes, first
from report_delivery_next import request_chain, enrich_episodes
from report_long_goal_crossing import panel, sampled_gap, scene_actors, settings
from report_sphere_scenarios import stats


def number(value, digits=3):
    return '无记录' if value is None else ('%.*f' % (digits, value))


def delta(end, start):
    return None if end is None or start is None else float(end-start)


def candidate_delivery(requests, timeline, observer, commits, cycles):
    """Match source transaction hashes, not nearest event timestamps."""
    receipts = {str(row['transaction_fnv64']): row for row in observer
                if row.get('event') == 'reference_observer_receipt' and 'transaction_fnv64' in row}
    result = []
    for request in requests:
        request_id = request['request_id']
        pub = first(timeline, lambda r: r.get('event') == 'reference_publish' and r.get('request_id') == request_id)
        receipt = receipts.get(str(pub.get('transaction_fnv64'))) if pub else None
        hashes = receipt.get('message_hashes') if receipt else None
        commit = first(commits, lambda r: hashes is not None and r.get('event') == 'replace'
                       and r.get('message_hashes') == hashes)
        related = [row for row in cycles if commit is not None and row.get('guide_epoch') == commit.get('epoch')
                   and row.get('guide_revision') == commit.get('revision')]
        use = first(related, lambda row: row.get('first_kernel_cycle_for_reference')
                    and row.get('online_step_entry_monotonic') is not None)
        command = first(related, lambda row: row.get('first_validated_command_cycle_for_reference')
                        and row.get('command_publish_after_monotonic') is not None)
        kernel_mono = use.get('online_step_entry_monotonic') if use else None
        command_mono = command.get('command_publish_after_monotonic') if command else None
        pub_mono = pub['mono'] if pub else None
        qualified = request.get('qualified_mono')
        backend_end = request.get('optimizer_end_mono')
        result.append(dict(request_id=request_id, backend_final_pass=request['backend_final_pass'],
            optimizer_end_mono=backend_end, qualified_mono=qualified,
            qualified_clock_semantics=request.get('qualified_clock_semantics'),
            latest_accept_mono=request.get('latest_accept_mono'),
            latest_rejection=request.get('latest_rejection'),
            latest_reject_details=request.get('latest_reject_details'),
            reject_detail_state_note='Legacy measured_* fields are planner-cached state, not certified acceptance-time measured state.',
            publish_mono=pub_mono, publish_sim=pub.get('sim') if pub else None,
            guide_start=pub.get('guide_start') if pub else None,
            published_reference_age_sim_s=request.get('publish_reference_age_sim_s'),
            reference_revision=commit.get('revision') if commit else None,
            reference_epoch=commit.get('epoch') if commit else None,
            commit_mono=commit.get('mono') if commit else None,
            first_kernel_mono=kernel_mono,
            first_kernel_sim_s=float(use['online_step_entry_ros_time'])-1000.
                if use and use.get('online_step_entry_ros_time') is not None else None,
            first_validated_command_mono=command_mono,
            first_validated_command_sim_s=float(command['command_publish_after_ros_time'])-1000.
                if command and command.get('command_publish_after_ros_time') is not None else None,
            qualified_to_publish_wall_s=delta(pub_mono, qualified),
            backend_end_to_publish_wall_s=delta(pub_mono, backend_end),
            publish_to_first_kernel_wall_s=delta(kernel_mono, pub_mono),
            publish_to_first_validated_command_wall_s=delta(command_mono, pub_mono),
            qualified_to_first_kernel_wall_s=delta(kernel_mono, qualified),
            qualified_to_first_validated_command_wall_s=delta(command_mono, qualified),
            backend_end_to_first_kernel_wall_s=delta(kernel_mono, backend_end),
            backend_end_to_first_validated_command_wall_s=delta(command_mono, backend_end),
            transaction_fnv64=pub.get('transaction_fnv64') if pub else None,
            actual_source_matching='publication transaction hash -> observer message hashes -> committed epoch/revision -> kernel/command cycle'))
    return result


def state_cache_evidence(request_id, timeline, state):
    events = [row for row in timeline if row.get('request_id')==request_id]
    cached = first(events, lambda row: row.get('event')=='state_contract_desired_boundary')
    accepted = first(events, lambda row: row.get('event') in ('acceptance_pass','acceptance_reject'))
    if cached is None or accepted is None:
        return None
    stamp = cached.get('raw_odom_stamp')
    sim = float(accepted['sim'])
    index = int(np.argmin(np.abs(state['t']-(sim-1000.))))
    actual = state[index]
    boundary_p = cached.get('planner_p')
    boundary_yaw = cached.get('planner_yaw')
    raw_x,raw_y,raw_yaw = [cached.get(name) for name in ('raw_odom_x','raw_odom_y','raw_odom_yaw')]
    displacement = (float(np.hypot(actual['x']-raw_x,actual['y']-raw_y))
                    if raw_x is not None and raw_y is not None else None)
    yaw_error = (float(math.atan2(math.sin(actual['yaw']-raw_yaw),math.cos(actual['yaw']-raw_yaw)))
                 if raw_yaw is not None else None)
    return dict(boundary_capture_sim=cached.get('capture_sim'),boundary_capture_mono=cached.get('capture_mono'),
        raw_odom_stamp=stamp,acceptance_event=accepted['event'],acceptance_sim=sim,
        raw_odom_age_at_acceptance_s=None if stamp is None else sim-float(stamp),
        raw_cached_odom_xy=[raw_x,raw_y],raw_cached_odom_yaw=raw_yaw,
        desired_boundary_p=boundary_p,desired_boundary_yaw=boundary_yaw,
        boundary_state_contract_semantics=cached.get('state_contract_semantics'),
        nearest_actual_state_sim_s=float(actual['t']),state_sample_minus_acceptance_s=float(actual['t'])-(sim-1000.),
        actual_xy=[float(actual['x']),float(actual['y'])],actual_yaw=float(actual['yaw']),
        cached_to_acceptance_sample_distance_m=displacement,cached_to_acceptance_sample_yaw_difference_rad=yaw_error,
        semantics='raw cached odometry with its original stamp compared with nearest recorded SIL state at acceptance; desired planner boundary is listed separately and may be regularized/predicted depending on branch; not proof of erroneous safety rejection')


def low_speed(state, anchor, end, goal):
    """Integrate left-sample low-speed flags to the next recorded timestamp."""
    times = np.asarray(state['t'])
    distance = np.hypot(state['x']-goal[0], state['y']-goal[1])
    mask = ((times >= anchor) & (times <= end) & (distance > .1)
            & (np.abs(state['v']) < .02) & (np.abs(state['w']) < .05))
    indices = np.flatnonzero(mask)
    groups = np.split(indices, np.flatnonzero(np.diff(indices) != 1)+1) if len(indices) else []
    intervals = [dict(start_sim_s=float(times[g[0]]), end_sim_s=float(min(times[min(g[-1]+1,len(times)-1)],end)),
                      start_after_motion_s=float(times[g[0]]-anchor),
                      end_after_motion_s=float(min(times[min(g[-1]+1,len(times)-1)],end)-anchor),
                      duration_s=float(max(0.,min(times[min(g[-1]+1,len(times)-1)],end)-times[g[0]])), samples=int(len(g)))
                 for g in groups if len(g)]
    return dict(cumulative_s=sum(row['duration_s'] for row in intervals),
        longest_s=max([row['duration_s'] for row in intervals]+[0.]), intervals=intervals,
        qualifying_samples=int(np.sum(mask)),
        definition='after measured motion_anchor, measured distance to goal > 0.1 m, |v| < 0.02 m/s, |w| < 0.05 rad/s; each qualifying left sample contributes the recorded interval until the next sample, clipped at actual end; contiguous intervals are merged. This is the same left-sample integration convention used for the previous report.')


def capture_description(reader):
    metadata = reader.json('native3d_event_input.json', {})
    path = reader.locate('native3d_event_input.npz')
    value = dict(metadata=metadata, available=path is not None)
    if path is not None:
        with np.load(path, allow_pickle=False) as record:
            value['arrays'] = {name: dict(shape=list(record[name].shape), dtype=str(record[name].dtype)) for name in record.files}
            # Timing arrays are small recorded data. Large fields are described,
            # never expanded into the report or substituted by an analytic map.
            value['timing_values'] = {name: record[name].tolist() for name in record.files
                if any(part in name.lower() for part in ('guide_start', 'ref_time')) and record[name].size < 5000}
    return value


def summarize(source, label, repeated):
    source = Path(source).resolve()
    trial = Trial(source, 'native3d')
    reader = trial.reader
    if not trial.status.get('result_written'):
        raise ValueError('Timed trial not finished: '+str(source))
    declaration = trial.manifest.get('delivery_next_online', {})
    if declaration.get('early_exit') is not True or declaration.get('common_cusp_phase_association') is not True:
        raise ValueError('This is not the declared current early-exit/cusp-correction configuration: '+str(source))
    timeline = reader.jsonl('dual_timeline.jsonl')
    solver = reader.jsonl('solver_trace.jsonl')
    observer = reader.jsonl('observer_events.jsonl')
    commits = reader.jsonl('reference_events.jsonl')
    cycles = reader.jsonl('state_mppi_cycles.jsonl')
    requests = request_chain(timeline, solver)
    candidates = candidate_delivery(requests, timeline, observer, commits, cycles)
    episodes = enrich_episodes(delivery_episodes(timeline, observer, commits, cycles), requests)
    _, goal, bounds = settings(trial.manifest)
    independent_gaps = [sampled_gap(np.asarray([row['x'],row['y'],row['yaw']]), trial.body3d, trial.resolution,
                        scene_actors(trial.manifest,trial.result,float(row['t'])),bounds)
                        for row in trial.state if float(row['t']) <= trial.end]
    monitor = trial.result.get('native3d_truth_monitor', {})
    substep_gap = monitor.get('min_physical_sample_gap')
    minimum_gap = float(substep_gap) if substep_gap is not None else float(min(independent_gaps))
    final = trial.result['final_state']
    native_selected = [request for request in requests if (request.get('early_summary') or {}).get('selected')]
    opt_metrics = [row['optimizer_metrics'] for row in requests if row.get('optimizer_metrics')]
    capture = capture_description(reader)
    for candidate in candidates:
        candidate['planner_state_cache_evidence'] = state_cache_evidence(candidate['request_id'],timeline,trial.state)
    low = low_speed(trial.state,trial.anchor,trial.end,goal)
    for interval in low['intervals']:
        start,end = interval['start_sim_s'],interval['end_sim_s']
        recorded_cycles = [row for row in cycles if row.get('command_publish_after_ros_time') is not None
                           and start <= float(row['command_publish_after_ros_time'])-1000. <= end]
        interval['published_cycle_reason_counts'] = dict(Counter(row.get('reason','unknown') for row in recorded_cycles))
        interval['published_cycle_count'] = len(recorded_cycles)
        interval['validated_command_cycle_count'] = sum(row.get('validated_command_published') is True for row in recorded_cycles)
        interval['overlapping_optimizer_request_ids'] = []
        for request in requests:
            request_events = [row for row in timeline if row.get('request_id')==request['request_id']]
            begin = first(request_events,lambda row:row.get('event')=='stage_begin' and row.get('stage')=='optimizer')
            finish = first(request_events,lambda row:row.get('event')=='stage_end' and row.get('stage')=='optimizer')
            if begin and finish and float(begin['sim'])-1000. <= end and float(finish['sim'])-1000. >= start:
                interval['overlapping_optimizer_request_ids'].append(request['request_id'])
    out = dict(trial=source.name, source=str(source), label=label, is_new_repetition=repeated,
        algorithm_configuration='current B: cheap scaling + measured cusp phase + full-native feasible early exit',
        termination=trial.result['reason'], reached_goal=trial.result['reason']=='goal',
        interface_valid=trial.audit.get('valid_experiment') is True,
        runner_error=trial.status.get('runner_error'),
        motion_anchor_sim_s=trial.anchor, end_sim_s=trial.end, motion_duration_s=trial.end-trial.anchor,
        final_xy=final[:2], final_yaw_rad=final[2], final_v_m_s=final[3], final_w_rad_s=final[4],
        final_goal_distance_m=float(np.hypot(final[0]-goal[0],final[1]-goal[1])),
        measured_travel_m=float(np.sum(np.hypot(np.diff(trial.state['x']),np.diff(trial.state['y'])))),
        backend_calls=len(requests), backend_final_passes=sum(row['backend_final_pass'] for row in requests),
        early_selected_calls=len(native_selected), reference_publications=sum(row['publish_mono'] is not None for row in candidates),
        references_actually_adopted=sum(row['first_kernel_mono'] is not None for row in candidates),
        references_with_validated_command=sum(row['first_validated_command_mono'] is not None for row in candidates),
        latest_rejection_counts=dict(Counter(row.get('reason','unknown') for row in timeline if row.get('event')=='acceptance_reject')),
        latest_reject_detail_counts=dict(Counter(row.get('branch','unknown') for row in timeline if row.get('event')=='latest_prediction_reject_detail')),
        delivery_episodes=episodes, candidates=candidates, requests=requests,
        full_optimizer_wall_ms=stats(row.get('wall_ms') for row in opt_metrics),
        budgeted_internal_ms=stats(row['early_summary'].get('elapsed_ms') for row in requests if row.get('early_summary')),
        measured_low_speed=low,
        independent_state_sample_min_gap_m=float(min(independent_gaps)),
        minimum_physical_sample_gap_m=float(minimum_gap),
        minimum_physical_sample_gap_source='online <=5ms truth monitor' if substep_gap is not None else 'independent recorded-state samples; substep minimum missing',
        physical_sample_collision=bool(minimum_gap<0 or min(independent_gaps)<0 or monitor.get('sample_collision')),
        online_substep_truth_monitor=monitor,
        truth_scope='native physical 67 spheres vs recorded finite-height cylinder/walls; state sample audit + <=5ms integration monitor; not full mesh or continuous proof',
        event_capture=capture)
    trial.verify_unchanged()
    out['input_sha256'] = dict(reader.hashes)
    return out, trial


def first_refresh(row):
    return next((event for event in row['delivery_episodes'] if event.get('publish_mono') is not None), {})


def chart(rows, path):
    image = Image.new('RGB',(1160,585),'white')
    draw = ImageDraw.Draw(image)
    title, font, small = [ImageFont.truetype(FONT,size) for size in (22,17,14)]
    draw.text((22,12),'同一新版配置：中段横穿重复验证',font=title,fill='#333333')
    draw.text((22,49),'上次B单次仅作参照；本轮复测1 / 2使用相同算法，不是算法改进对照。',font=font,fill='#555555')
    metrics = [('触发重规划 → MPPI实际采用 / s',lambda row:first_refresh(row).get('trigger_to_first_kernel_wall_s')),
               ('全程非目标低速：累计 / 最长连续 / s',lambda row:row['measured_low_speed']['cumulative_s'])]
    for column,(heading,getter) in enumerate(metrics):
        left = 22+column*575
        draw.text((left,105),heading,font=font,fill='#333333')
        maximum = max([getter(row) for row in rows if getter(row) is not None]+[.001])
        for index,row in enumerate(rows):
            y = 156+index*83
            value = getter(row)
            draw.text((left,y),row['label'],font=font,fill='#555555')
            width = (0 if value is None else value/maximum*300)
            draw.rectangle((left+102,y+3,left+102+width,y+23),fill='#aaaaaa' if not row['is_new_repetition'] else '#4e7c9d')
            text = number(value)
            if column: text += ' / '+number(row['measured_low_speed']['longest_s'])
            draw.text((left+108+width,y),text,font=small,fill='#444444')
    y = 421
    for row in rows:
        text = '%s：%s；后端通过%d/%d，发布/采用%d/%d；物理采样最小间隙%.3fm；拒收%d次' % (
            row['label'],'到达目标' if row['reached_goal'] else '未到目标',row['backend_final_passes'],row['backend_calls'],
            row['reference_publications'],row['references_actually_adopted'],row['minimum_physical_sample_gap_m'],sum(row['latest_rejection_counts'].values()))
        draw.text((22,y),text,font=small,fill='#555555');y += 30
    draw.text((22,548),'低速判据：起步后、距目标>0.1m，|v|<0.02m/s且|ω|<0.05rad/s；ROS数值闭环，非实车/Gazebo。',font=small,fill='#777777')
    image.save(path)


def render_repetitions(rows, trials, path, max_frames=72):
    duration = max(trial.end-trial.anchor for trial in trials)
    ages = np.linspace(0.,duration,min(max_frames,max(2,math.ceil(duration/.5)+1)))
    # Include each first refreshed-reference adoption instant in the actual
    # frame grid, so the label cannot be skipped by uniform decimation.
    adopted = []
    for row in rows:
        candidate = next((item for item in row['candidates'] if item['request_id']>1 and item['first_kernel_sim_s'] is not None),{})
        t = candidate.get('first_kernel_sim_s')
        adopted.append(t)
    ages = np.unique(np.r_[ages,[t-trial.anchor for t,trial in zip(adopted,trials) if t is not None]])
    title,small = [ImageFont.truetype(FONT,size) for size in (19,14)]
    for trial in trials: trial.native_validity = []
    search_counts = [len(trial.search.entries) for trial in trials]
    legend = '蓝：机器人/实走　橙：REMANI参考　绿：获准展开　紫：CV预测'
    if any(search_counts): legend += '　灰细线：已记录搜索子集'
    frames = []
    for age in ages:
        image = Image.new('RGB',(1160,510),'white')
        draw = ImageDraw.Draw(image)
        draw.text((18,9),'中段横穿：起步后 %.1fs（两次各自起步对齐）' % age,font=title,fill='#333333')
        draw.text((18,39),legend,font=small,fill='#555555')
        for index,(trial,row,t_adopt) in enumerate(zip(trials,rows,adopted)):
            x = index*580+18
            t = min(trial.anchor+float(age),trial.end)
            draw.text((x,69),row['label'],font=title,fill='#333333')
            if t_adopt is None:
                label = '未记录到重规划参考采用'
            else:
                label = '新参考%s：起步后%.2fs' % ('已采用' if t>=t_adopt else '待采用',t_adopt-trial.anchor)
            draw.text((x+104,72),label,font=small,fill='#555555')
            measured = panel(image,trial,t,(x,108,(index+1)*580-18,414))
            draw = ImageDraw.Draw(image)
            status = trial.status_at(t)
            if t>=trial.end-1e-9:
                status = '已到达目标' if trial.result['reason']=='goal' else '实际记录结束：'+trial.result['reason']
            elif status=='ok': status='局部控制有效'
            draw.text((x,424),'%s　v=%+.2f m/s　ω=%+.2f rad/s' % (status,measured['v'],measured['w']),font=small,fill='#555555')
        draw.text((18,462),'同一新版配置复测，不是A/B改进；固定臂67球投影，完整REMANI＋MPPI实际记录回放。',font=small,fill='#777777')
        draw.text((18,484),'ROS数值闭环，非实车/Gazebo；没有重算控制动作，短暂低速和失败均保留。',font=small,fill='#777777')
        frames.append(image.convert('P',palette=Image.ADAPTIVE,colors=96))
    delays = [max(30,10*round(float(gap)*500./10.)) for gap in np.diff(ages)]+[1000]
    frames[0].save(path,save_all=True,append_images=frames[1:],loop=0,disposal=2,optimize=True,duration=delays)
    for trial in trials: trial.verify_unchanged()
    return dict(frames=len(frames),speedup=2.,adoption_sim_times=adopted,
                no_controller_reexecution=True,search_frames_per_trial=search_counts,
                search_displayed=bool(any(search_counts)),real_search_subset_only=True,actual_robot_sphere_projection=True)


def narrative(rows):
    lines = ['轨迹交付复测：当前新版在中段横穿中是否能重复交付',
        '本轮没有更改算法。延用廉价变量缩放、MPPI实测换向段关联、候选完整验收通过后提前退出。',
        '原始初始化、67球、轮约束4.1/4.25、1.2秒内部预算。固定机械臂，完整REMANI＋MPPI ROS数值闭环，不是实车或Gazebo。',
        '两次复测同场景、同种子，闭环时间和后续输入仍会分化。上次B仅作参照，不能说是新算法对比。','']
    for row in rows:
        episode = first_refresh(row)
        low = row['measured_low_speed']
        lines += [row['label']+'：'+('到达目标。' if row['reached_goal'] else '未到目标，结束原因'+row['termination']+'。'),
            '后端完整验收通过%d/%d；发布%d条，MPPI实际采用%d条，产生获准指令%d条。最新动态拒收%d次。' % (
                row['backend_final_passes'],row['backend_calls'],row['reference_publications'],row['references_actually_adopted'],
                row['references_with_validated_command'],sum(row['latest_rejection_counts'].values())),
            '首次重规划触发到MPPI采用%s秒，到首条获准指令%s秒。' % (
                number(episode.get('trigger_to_first_kernel_wall_s')),number(episode.get('trigger_to_validated_command_wall_s'))),
            '实际运动%.2f秒、走%.2f米；距目标%.3f米。最小物理球采样间隙%.3f米；%s采样碰撞。' % (
                row['motion_duration_s'],row['measured_travel_m'],row['final_goal_distance_m'],row['minimum_physical_sample_gap_m'],
                '发生' if row['physical_sample_collision'] else '未发现'),
            '全程非目标低速累计%.3f秒，最长连续%.3f秒。低速不等于完全停车，也不自动等于等待后端。' % (low['cumulative_s'],low['longest_s'])]
        for candidate in row['candidates']:
            lines += ['请求%d：后端%s；通过原生检查后到发布%s秒，发布到内核采用%s秒，到获准指令%s秒。' % (
                candidate['request_id'],'通过' if candidate['backend_final_pass'] else '未通过',
                number(candidate['qualified_to_publish_wall_s']),number(candidate['publish_to_first_kernel_wall_s']),
                number(candidate['publish_to_first_validated_command_wall_s']))]
            if candidate['latest_rejection']:
                lines.append('该请求被下游拒收：'+str(candidate['latest_rejection'].get('reason'))+'；详细分支'+
                    ', '.join(str(item.get('branch')) for item in candidate['latest_reject_details'])+'。')
        lines.append('')
    lines += ['读数说明：时间链使用实际单调墙钟，场景与GIF使用仿真时间。后端最终通过以最终原生检查后的结束事件确认，提前选中时另给出检查精确完成时刻。',
        '候选到控制的关联依靠原始事务哈希、消息哈希、参考版本和控制周期，不用最近时间猜测。缺失事件显示“无记录”，不当作0。',
        '旧latest_prediction_reject_detail中的measured_*为规划缓存状态，不能称为验收瞬间的实测位置。',
        '碰撞结论限于原生球与有限高障碍/边界的离散采样及在线子步监测。低速沿用上轮左采样区间积分：某样本满足判据，就计入它到下一样本的时长；连续区间合并。剔除起步前及距目标0.1米以内的驻停。',
        '这里只验证当前场景的重复表现，不据三次运行宣称所有动态场景稳定，也不把低速现象一概归因于REMANI。','']
    return '\n'.join(lines)


def report(batch, out=None, make_gif=True):
    batch = Path(batch).resolve()
    out = Path(out).resolve() if out else batch/'report'
    plan_path = batch/'plan.json'
    plan = json.loads(plan_path.read_text())
    # A complete timed batch is required; analysis never competes with running
    # planners or silently omits an unfinished/failed job.
    if not (batch/'complete.json').is_file():
        raise ValueError('Wait for complete.json before generating analysis/GIFs')
    rows, trials = [], []
    old = EXPERIMENT/'results/delivery_next_20261031_01/midcross_early_exit_seed11'
    old_row, _ = summarize(old,'上次B单次',False)
    rows.append(old_row)
    for index,job in enumerate(plan['jobs']):
        if job['scene']!='midcross' or job['early_exit'] is not True:
            raise ValueError('Only declared current-B middle-crossing repetitions belong in this report')
        row, trial = summarize(batch/job['trial'],'复测%d' % (index+1),True)
        rows.append(row); trials.append(trial)
    if len(trials)!=2:
        raise ValueError('Expected exactly the two declared repetitions, without replacements')
    out.mkdir(parents=True,exist_ok=True)
    chart(rows,out/'交付复测对照.png')
    visualization = render_repetitions(rows[1:],trials,out/'中段横穿两次复测.gif') if make_gif else None
    for trial in trials: trial.verify_unchanged()
    summary = dict(scope='same-current-B configuration repeated numerical ROS SIL; fixed arm; not hardware/Gazebo',
        algorithm_changed_in_this_batch=False, old_single_run_is_reference_only=True,
        plan_sha256=sha(plan_path), report_script_sha256=sha(HERE), trials=rows,
        visualization=visualization,
        old_measured_detail_caution='measured_* in legacy rejection diagnostics is planner-cached, not acceptance-instant fresh measurement')
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    (out/'实验结果.txt').write_text(narrative(rows))
    print(json.dumps(dict(report=str(out),trials=[dict(label=row['label'],goal=row['reached_goal'],
        backend_pass=row['backend_final_passes'],backend_calls=row['backend_calls'],
        publications=row['reference_publications'],adoptions=row['references_actually_adopted'],
        latest_rejects=sum(row['latest_rejection_counts'].values()),
        first_refresh_to_adopt_s=first_refresh(row).get('trigger_to_first_kernel_wall_s'),
        low_speed=row['measured_low_speed']['longest_s'],gap=row['minimum_physical_sample_gap_m']) for row in rows]),ensure_ascii=False))
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch',type=Path,default=HERE.parents[1])
    parser.add_argument('--out',type=Path)
    parser.add_argument('--no-gif',action='store_true')
    args=parser.parse_args()
    report(args.batch,args.out,not args.no_gif)
