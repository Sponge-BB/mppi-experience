#!/usr/bin/env python3
"""Report the normal online midpoint-crossing delivery-window A/B pair.

Reuse the final-native / consumed-polynomial / validated-command binding from
report_delivery_window. Extra early-detour observations use existing recorded
state and the existing first approved dynamic-intervention hint, with no new
onset threshold, online probe or controller replay.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

import report_delivery_window as delivery
from render_coupled_objective import sha
from render_dual_timescale import FONT
from report_frontend_rejoin import keyframes
from report_half_speed_contract import add_common_motion_window
from report_long_goal_crossing import panel, settings
from report_static_agility import show
from report_time_indexed_collision import TERMINATIONS


HERE = Path(__file__).resolve().parent
DEFAULT = HERE / 'results/delivery_midcross_20261101_01'
LABELS = delivery.LABELS


def state_at(trial, stamp):
    state = trial.state
    if stamp is None or not len(state) or stamp < state['t'][0] or stamp > state['t'][-1]:
        return None
    return dict(t=float(stamp), x=float(np.interp(stamp, state['t'], state['x'])),
        y=float(np.interp(stamp, state['t'], state['y'])),
        v=float(np.interp(stamp, state['t'], state['v'])),
        w=float(np.interp(stamp, state['t'], state['w'])))


def first_plane_crossing(trial, x, begin):
    for previous, current in zip(trial.state, trial.state[1:]):
        if current['t'] < begin or previous['x'] >= x or current['x'] < x:
            continue
        span = float(current['x'] - previous['x'])
        if span <= 0.:
            continue
        fraction = (x - float(previous['x'])) / span
        stamp = float(previous['t']) + fraction * float(current['t'] - previous['t'])
        return state_at(trial, stamp)
    return None


def maximum_lateral_sample(trial, begin, end, center_y):
    if end < begin:
        return None
    states = [state_at(trial, begin), state_at(trial, end)]
    states += [dict(t=float(r['t']), x=float(r['x']), y=float(r['y']))
               for r in trial.state if begin <= r['t'] <= end]
    states = [r for r in states if r is not None and begin <= r['t'] <= end]
    return max(states, key=lambda r: abs(r['y'] - center_y)) if states else None


def crossing_observations(row, trial):
    scene = trial.manifest['scenario']
    _, goal, _ = settings(trial.manifest)
    actors = [actor for actor in scene['actors'] if abs(float(actor['vx'])) < 1e-12
              and abs(float(actor['vy'])) > 1e-12]
    if len(actors) != 1:
        raise ValueError('midcross report requires one recorded constant-x crossing actor')
    actor = actors[0]
    anchor_name = 'motion_anchor' if trial.manifest.get('scenario_start') == 'motion' else 'goal_sent'
    anchor = trial.result.get(anchor_name)
    if anchor is None:
        raise ValueError('missing recorded scenario anchor: ' + anchor_name)
    appearance = float(anchor) + float(scene['appearance_s'])
    center_y = float(goal[1])
    crossing_x = float(actor['x'])
    center_crossing = appearance + (center_y - float(actor['y'])) / float(actor['vy'])
    robot_crossing = first_plane_crossing(trial, crossing_x, trial.anchor)
    hint = row['fast_refresh']['first_actual_approved_avoidance_hint']
    hint_stamp = (hint.get('actual_command_publish_after_ros_time') if hint else None)
    hint_elapsed = hint_stamp - 1000. if hint_stamp is not None else None
    pose = state_at(trial, hint_elapsed)
    first_adopted = row['deferred_quality']['first_actually_adopted']
    adoption_time = first_adopted['first_validated_command_elapsed_s'] if first_adopted else None
    hint_observation = None
    if pose is not None:
        age = hint_elapsed - appearance
        actor_y = float(actor['y']) + max(0., age) * float(actor['vy'])
        hint_observation = dict(actual_validated_command_elapsed_s=hint_elapsed,
            state=pose, existing_hint=hint,
            remaining_longitudinal_distance_to_crossing_line_m=crossing_x - pose['x'],
            actor_present=age >= 0., actor_y_m=actor_y if age >= 0. else None,
            actor_distance_to_centerline_m=abs(actor_y - center_y) if age >= 0. else None,
            time_until_actor_center_crossing_s=center_crossing - hint_elapsed,
            first_new_reference_actual_control_elapsed_s=adoption_time,
            precedes_first_new_reference_execution=(hint_elapsed < adoption_time) if adoption_time is not None else None,
            scope='Existing first approved dynamic-intervention hint and actual measured position. This is not an independently detected visible-detour onset.')
    end = robot_crossing['t'] if robot_crossing is not None else trial.end
    maximum = maximum_lateral_sample(trial, appearance, end, center_y)
    before = maximum_lateral_sample(trial, appearance,
        min(end, adoption_time) if adoption_time is not None else end, center_y)
    after = maximum_lateral_sample(trial, max(appearance, adoption_time), end,
        center_y) if adoption_time is not None else None
    return dict(actor_definition=actor, scenario_anchor_name=anchor_name,
        scenario_anchor_elapsed_s=float(anchor), actor_appearance_elapsed_s=appearance,
        actor_center_crossing_elapsed_s=center_crossing,
        first_observed_truth_actor_elapsed_s=trial.result.get('native_realtime_evidence', {}).get('first_truth_actor_t'),
        crossing_line_x_m=crossing_x, original_route_centerline_y_m=center_y,
        robot_first_crossing_plane=robot_crossing,
        first_approved_dynamic_intervention=hint_observation,
        actual_maximum_lateral_offset_before_crossing_m=abs(maximum['y'] - center_y) if maximum else None,
        actual_maximum_lateral_offset_sample=maximum,
        first_new_reference_actual_control_elapsed_s=adoption_time,
        actual_maximum_lateral_offset_before_adoption_m=abs(before['y'] - center_y) if before else None,
        actual_maximum_lateral_offset_after_adoption_m=abs(after['y'] - center_y) if after else None,
        before_adoption_maximum_sample=before, after_adoption_maximum_sample=after,
        lateral_measurement_begin_s=appearance, lateral_measurement_end_s=end,
        robot_reached_crossing_line=robot_crossing is not None,
        scope='Read-only actual state observations, no added detour threshold. Actor coordinates are prescribed recorded scene truth, not a CV accuracy measurement. Negative remaining time/distance means the corresponding crossing has already occurred.')


def load_online(source, label):
    row, trial = delivery.load_online(source, label)
    row['midcross_observations'] = crossing_observations(row, trial)
    row['source_sha256'] = trial.reader.hashes
    trial.verify_unchanged()
    return row, trial


def crossing_notes(rows):
    notes = []
    for row in rows:
        record = row['midcross_observations']
        hint = record['first_approved_dynamic_intervention']
        if hint is None:
            notes.append(row['label'] + '：未关联到可核实的首次获准动态介入时刻，不补造起绕距离。')
        else:
            notes.append(row['label'] + '：首次获准动态介入时，机器人距穿行线还剩 %s 米；障碍距原路线中心线 %s 米，障碍中心穿越还需 %s 秒。' % (
                show(hint['remaining_longitudinal_distance_to_crossing_line_m'], 3),
                show(hint['actor_distance_to_centerline_m'], 3), show(hint['time_until_actor_center_crossing_s'], 3)))
            notes.append('  该介入发生于首条新参考实际采用%s；首条新参考有效控制时刻为 %s 秒。' % (
                '之前' if hint['precedes_first_new_reference_execution'] is True else
                '之后' if hint['precedes_first_new_reference_execution'] is False else '之前（本轮未见新参考采用）',
                show(record['first_new_reference_actual_control_elapsed_s'], 3)))
        notes.append('  障碍出现到机器人第一次通过穿行线%s，实际最大横向偏移 %s 米。' % (
            '期间' if record['robot_reached_crossing_line'] else '（本轮尚未通过，统计至结束）',
            show(record['actual_maximum_lateral_offset_before_crossing_m'], 3)))
        notes.append('  在上述统计窗口内，首条新参考采用前 / 后最大横偏为 %s / %s 米。' % (
            show(record['actual_maximum_lateral_offset_before_adoption_m'], 3),
            show(record['actual_maximum_lateral_offset_after_adoption_m'], 3)))
    notes.append('首次获准动态介入不等于肉眼可见的明显起绕；以上距离和横偏描述提前程度，是否绕得不必要还要结合 GIF、间隙和交付结果判断。')
    notes.append('新参考真正采用前的运动差异不能直接归因于慢层延长预测窗口；采用前后已分别列出。')
    return notes


def narrative(data):
    text = delivery.narrative(data).replace('慢层覆盖交付后近期执行时段：实验结果',
        '路径中段横穿：交付窗口实验结果').replace('每组一次迎面场景', '每组一次路径中段横穿场景')
    notes = ['提前绕行的现有记录观察'] + data['crossing_conclusions']
    return text.replace('\n口径\n', '\n' + '\n'.join(notes) + '\n\n口径\n')


def animation(rows, trials, output, max_frames):
    """Same mature recorded panels; scene title/labels describe midcross."""
    duration = max(trial.end - trial.anchor for trial in trials)
    ages = np.linspace(0., duration, min(max_frames, max(2, math.ceil(duration / .4) + 1)))
    title, small = [ImageFont.truetype(FONT, n) for n in (19, 14)]
    frames = []
    for age in ages:
        image = Image.new('RGB', (1160, 510), 'white')
        draw = ImageDraw.Draw(image)
        draw.text((18, 8), '路径中段横穿：起步后 %.1f s' % age, font=title, fill='#333333')
        draw.text((18, 38), '蓝：机器人/实走　橙：REMANI参考　绿：获准MPPI展开　紫：现有CV预测（最新上限3s）', font=small, fill='#555555')
        for i, (row, trial) in enumerate(zip(rows, trials)):
            left, stamp = i * 580 + 18, min(trial.anchor + float(age), trial.end)
            draw.text((left, 69), row['label'], font=title, fill='#333333')
            state = panel(image, trial, stamp, (left, 104, (i + 1) * 580 - 18, 410))
            draw = ImageDraw.Draw(image)
            if stamp >= trial.end - 1e-9:
                status = '到达目标' if row['reached_goal'] else TERMINATIONS.get(row['termination'], str(row['termination']))
            else:
                status = trial.status_at(stamp)
                status = '控制有效' if status == 'ok' else status
            draw.text((left, 422), '%s　v=%+.2f　ω=%+.2f' % (status, state['v'], state['w']), font=small, fill='#555555')
        draw.text((18, 461), '2倍实录；完整67球投影；本轮未记录搜索候选，不补画搜索过程。', font=small, fill='#777777')
        draw.text((18, 484), '完整REMANI＋底盘MPPI，固定臂ROS数值闭环；非Gazebo/实车。', font=small, fill='#777777')
        frames.append(image.convert('P', palette=Image.ADAPTIVE, colors=96))
    durations = [max(30, 10 * round(float(dt) * 500 / 10)) for dt in np.diff(ages)] + [1000]
    frames[0].save(output, save_all=True, append_images=frames[1:], duration=durations,
        loop=0, disposal=2, optimize=True)
    return dict(frames=len(frames), playback_speedup=2., sha256=sha(output),
        full_physical_sphere_projection=True, sphere_counts=[len(t.body3d) for t in trials],
        actual_records_only=True, full_search_recorded=False, no_search_recorded=True,
        green_semantics='Recorded execution-authorized local rollout; no search candidates recorded or fabricated.',
        cv_visualization_semantics='Existing execution-side CV display with global/latest 3s limit; no fabricated slow-copy 4.5s observed prediction line.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=DEFAULT)
    parser.add_argument('--max-frames', type=int, default=70)
    parser.add_argument('--skip-gif', action='store_true')
    args = parser.parse_args()
    pairs = [load_online(args.root / 'midcross_A', LABELS[0]), load_online(args.root / 'midcross_B', LABELS[1])]
    rows, trials = zip(*pairs)
    add_common_motion_window(rows, trials)
    data = dict(schema='delivery_midcross_report_v1', rows=list(rows),
        initial_pair_contract=delivery.initial_pair_contract(rows), old_candidate_replay=None,
        execution_record_resolution=delivery.execution_resolution(args.root),
        same_native_binaries=rows[0]['binary_identity']['native'] == rows[1]['binary_identity']['native'],
        same_mppi_library=rows[0]['binary_identity']['mppi'] == rows[1]['binary_identity']['mppi'],
        scope='One unpaused midpoint-crossing fixed-arm ROS numerical pair. Only B refresh uses slow-copy 4.5s CV continuation. Both global/latest/MPPI limits stay 3s; actual final-native adoption binding is reused.')
    data['conclusions'] = delivery.conclusions(data)
    data['crossing_conclusions'] = crossing_notes(rows)
    print('DELIVERY_MIDCROSS_CORE', json.dumps(dict(conclusions=data['conclusions'],
        crossing_conclusions=data['crossing_conclusions']), ensure_ascii=False), flush=True)
    output = args.root / 'report'
    output.mkdir(parents=True, exist_ok=True)
    gif_name, keyframe_name = '中段横穿对照.gif', '中段横穿关键帧.png'
    if args.skip_gif:
        previous = json.loads((output / 'summary.json').read_text())
        artifact = dict(previous['artifacts']['midcross'])
        if sha(output / gif_name) != artifact['sha256'] or not (output / keyframe_name).is_file():
            raise ValueError('Existing GIF/keyframes missing or changed.')
        artifact['reused_without_rerender'] = True
        data['artifacts'] = dict(midcross=artifact)
    else:
        data['artifacts'] = dict(midcross=animation(rows, trials, output / gif_name, args.max_frames))
        keyframes(output / gif_name, output / keyframe_name)
    delivery.chart(data, output / '对照数据.png')
    (output / '实验结果.txt').write_text(narrative(data))
    (output / '本轮结论.txt').write_text('\n'.join(data['conclusions'] + [''] + data['crossing_conclusions']) + '\n')
    for trial in trials:
        trial.verify_unchanged()
    data['report_script_sha256'] = sha(Path(__file__))
    data['inherited_delivery_report_sha256'] = sha(Path(delivery.__file__))
    (output / 'summary.json').write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    print('DELIVERY_MIDCROSS_REPORT', output, flush=True)


if __name__ == '__main__':
    main()
