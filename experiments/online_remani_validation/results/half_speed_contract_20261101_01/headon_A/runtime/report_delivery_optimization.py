#!/usr/bin/env python3
"""Read completed delivery OFF/ON records; no planning/controller rerun."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from render_coupled_objective import SourceReader
from render_dual_timescale import FONT
from render_native_collision_3d import Trial
from report_long_goal_crossing import panel, settings
from report_sphere_scenarios import finite, shown, stats, summarize as base_summary
from run_delivery_optimization import GROUPS, SCENES, check_pair_inputs

LABELS = {'baseline': '优化前', 'optimized': '交付优化后'}
NAMES = {'static': '静态障碍', 'midcross': '路径中段横穿'}


def delivery_metrics(analysis, resources, end):
    """Backend acceptance is never counted as publication or consumption."""
    origin = float(analysis.get('clock_origin_ros_s', 1000.))
    requests = [r for r in analysis.get('requests', [])
                if finite(r.get('end_sim_s')) and r['end_sim_s'] <= end]
    by_id = {c['resource_optimizer_id']: r for r in requests
             for c in r.get('optimizer_calls', []) if 'resource_optimizer_id' in c}
    optimizer = [r for r in resources if r.get('event') == 'end' and r.get('stage') == 'optimizer'
                 and finite(r.get('sim')) and r['sim']-origin <= end]
    # Header enum DONT_STOP, STOP_FOR_REBOUND, STOP_FOR_ERROR,
    # STOP_FOR_TIMEOUT -> 3. This records WHY solving stopped, not whether the
    # rebuilt candidate later passed native checks or reached the controller.
    timeouts = [r for r in optimizer if r.get('force_stop') == 3]
    chains = [c for c in analysis.get('reference_chains', [])
              if finite(c.get('publish_sim_s')) and c['publish_sim_s'] <= end]
    salvages = []
    for row in optimizer:
        if not row.get('salvage_attempt'):
            continue
        request = by_id.get(row.get('id'))
        native_accepted = bool(row.get('salvage_native_result')) and row.get('success') is True
        # A failed optimizer in a later-successful request must not receive
        # credit for another candidate's eventual publication.
        linked = [c for c in chains if native_accepted and request is not None
                  and c.get('request_id') == request['request_id']]
        consumed = [c for c in linked if finite(c.get('consume_sim_s')) and c['consume_sim_s'] <= end]
        salvages.append(dict(resource_optimizer_id=row.get('id'), request_id=request.get('request_id') if request else None,
            candidate_available=row.get('candidate_available'), accepted_steps=row.get('candidate_accepted_steps'),
            source_initial=row.get('candidate_source_initial'), rebuilt=row.get('salvage_rebuilt'),
            native_result=row.get('salvage_native_result'), optimizer_success=row.get('success'),
            rebuild_ms=row.get('salvage_rebuild_ms'), validation_ms=row.get('salvage_validation_ms'),
            lbfgs_wall_ms=row.get('lbfgs_wall_ms'), optimizer_with_validation_ms=row.get('wall_ms'),
            request_success=request.get('success') if request else None,
            acceptance_rejections=request.get('acceptance_rejections', []) if request else [],
            published=bool(linked), consumed=bool(consumed), reference_chains=linked))
    latency = analysis.get('latency', {})
    def interval(begin, finish):
        a, b = latency.get(begin), latency.get(finish)
        return b-a if finite(a) and finite(b) and a <= b <= end else None
    return dict(
        first_blocked_to_publish_s=interval('first_native_blocked_sim_s', 'first_refresh_publish_sim_s'),
        first_trigger_to_publish_s=interval('first_refresh_trigger_sim_s', 'first_refresh_publish_sim_s'),
        first_trigger_to_consume_s=interval('first_refresh_trigger_sim_s', 'first_refresh_consume_sim_s'),
        first_blocked_to_consume_s=interval('first_native_blocked_sim_s', 'first_refresh_consume_sim_s'),
        salvage_attempts=len(salvages), salvage_native_pass=sum(bool(r['native_result']) for r in salvages),
        salvage_request_success=sum(bool(r['native_result']) and r['optimizer_success'] is True
                                    and r['request_success'] is True for r in salvages),
        salvage_published=sum(r['published'] for r in salvages), salvage_consumed=sum(r['consumed'] for r in salvages),
        salvage_validation_ms=stats(r['validation_ms'] for r in salvages), salvages=salvages,
        lbfgs_wall_ms=stats(r.get('lbfgs_wall_ms') for r in optimizer),
        native_validation_ms=stats(r.get('native_validation_ms') for r in optimizer),
        optimizer_timeout_events=len(timeouts),
        optimizer_timeout_final_backend_accepted=sum(r.get('success') is True for r in timeouts),
        optimizer_timeout_final_backend_failed=sum(r.get('success') is False for r in timeouts),
        optimizer_timeout_final_backend_unknown=sum(not isinstance(r.get('success'), bool) for r in timeouts),
        timeout_semantics='force_stop=3 is solver budget exhaustion; success=true may still deliver a checked candidate',
        live_acceptance_wall_ms=stats(r.get('stage_wall_ms', {}).get('live_acceptance') for r in requests),
        optimizer_rows=optimizer, reference_chains=chains,
        native_wall_accounting=analysis.get('native_wall_accounting'),
        limitations='native pass means backend candidate only; linked native publication and MPPI consumption counted separately; native_validation_ms covers every entered final native gate, failed and successful')


def summarize(source, scene, group):
    row = base_summary(source)
    reader = SourceReader(source)
    analysis = reader.json('analysis/summary.json', {}, required=True)
    row.update(scene_key=scene, group=group,
        delivery=delivery_metrics(analysis, reader.jsonl('solver_trace.jsonl'), row['end_t']))
    optimizer = {r['id']: r for r in row['delivery']['optimizer_rows'] if 'id' in r}
    for request in row['remani']['request_rows']:
        for call in request.get('optimizer_calls', []):
            measured = optimizer.get(call.get('resource_optimizer_id'), {})
            for key in ('lbfgs_wall_ms', 'native_validation_ms', 'force_stop',
                        'salvage_attempt', 'salvage_native_result'):
                if key in measured:
                    call[key] = measured[key]
    if row['total_spheres'] != 67:
        raise ValueError('Expected recorded B67 geometry')
    reader.verify()
    row['input_sha256'].update(reader.hashes)
    return row


def chart(rows, out):
    image = Image.new('RGB', (1060, 690), 'white')
    draw = ImageDraw.Draw(image)
    title, small = ImageFont.truetype(FONT, 21), ImageFont.truetype(FONT, 16)
    draw.text((22, 12), '轨迹交付优化：同场景在线对照', font=title, fill='#333333')
    draw.text((22, 45), '灰：优化前　蓝：优化后；每组仅跑一次。无新轨迹不等于零等待。', font=small, fill='#555555')
    specs = [('受阻 → MPPI收到新参考 / 秒', lambda r: r['delivery']['first_blocked_to_consume_s']),
             ('完成的重规划请求中位数 / 毫秒', lambda r: r['remani']['completed_refresh_wall_ms']['p50']),
             ('REMANI优化器中位数 / 毫秒', lambda r: r['remani']['backend_wall_ms']['p50']),
             ('MPPI求解＋验收P95 / 毫秒', lambda r: r['mppi']['callback_ms']['p95'])]
    lookup = {(r['scene_key'], r['group']): r for r in rows}
    for index, (label, get) in enumerate(specs):
        x, y = 22+(index % 2)*525, 95+(index // 2)*250
        draw.text((x, y), label, font=small, fill='#333333')
        maximum = max([get(r) for r in rows if finite(get(r))]+[1.])
        for j, scene in enumerate(SCENES):
            top = y+40+j*82
            draw.text((x, top+12), NAMES[scene], font=small, fill='#444444')
            for k, group in enumerate(GROUPS):
                row = lookup[scene, group]
                value = get(row)
                width = 245.*value/maximum if finite(value) else 0.
                yy = top+k*28
                if width:
                    draw.rectangle((x+130, yy, x+130+width, yy+19), fill='#999999' if k == 0 else '#2874a6')
                text = shown(value, 2) if finite(value) else '未交付/无记录'
                draw.text((x+136+width, yy), text, font=small, fill='#333333')
    draw.text((22, 610), 'REMANI优化器含原生验收；搜索另计。所有完成的失败请求保留，不只统计成功那次。', font=small, fill='#666666')
    draw.text((22, 638), '固定机械臂、67球、完整REMANI＋MPPI数值ROS闭环；不是Gazebo/实车。', font=small, fill='#666666')
    image.save(out)


def render_pair(before, after, out, scene, max_frames=100):
    trials = [Trial(p, 'native3d') for p in (before, after)]
    for trial in trials:
        settings(trial.manifest)
        trial.native_validity = []
    if not np.array_equal(trials[0].body3d, trials[1].body3d):
        raise ValueError('Cannot visualize as same-geometry comparison')
    duration = max(t.end-t.anchor for t in trials)
    ages = np.linspace(0., duration, min(max_frames, max(2, math.ceil(duration/.4)+1)))
    frame_ms = max(20, 10*round(1000.*float(ages[1]-ages[0])/2./10.))
    font, small = ImageFont.truetype(FONT, 18), ImageFont.truetype(FONT, 14)
    frames = []
    for age in ages:
        image = Image.new('RGB', (1160, 490), 'white')
        draw = ImageDraw.Draw(image)
        draw.text((16, 8), '%s　%.1f秒（各自起步对齐，2倍速）' % (NAMES[scene], age), font=font, fill='#333333')
        draw.text((16, 35), '橙：REMANI参考　蓝：实走路线/机器人投影　灰细线：MPPI搜索子集　绿：获准展开　紫：CV', font=small, fill='#555555')
        for index, trial in enumerate(trials):
            t = min(trial.anchor+age, trial.end)
            draw.text((index*580+16, 66), LABELS[GROUPS[index]], font=font, fill='#333333')
            row = panel(image, trial, t, (index*580+16, 100, (index+1)*580-16, 400))
            draw = ImageDraw.Draw(image)
            draw.text((index*580+16, 411), '%s　v=%+.2f m/s' % (trial.status_at(t), row['v']), font=small, fill='#444444')
            if t >= trial.end:
                draw.text((index*580+16, 435), '本组结束：'+str(trial.result.get('reason')), font=small, fill='#666666')
        draw.text((16, 468), '固定臂ROS数值闭环；非实车/Gazebo。仅回放实录；搜索每秒记录一次，不等于只每秒计算一次。', font=small, fill='#777777')
        frames.append(image.convert('P', palette=Image.ADAPTIVE, colors=96))
    frames[0].save(out, save_all=True, append_images=frames[1:],
        duration=[frame_ms]*(len(frames)-1)+[max(700, frame_ms)], loop=0, disposal=2, optimize=True)
    frames[-1].convert('RGB').save(out.with_suffix('.png'))
    for trial in trials:
        trial.verify_unchanged()
    return dict(frames=len(frames), speedup=2., actual_sphere_projection=True,
                recorded_search_frames=[len(t.search.entries) for t in trials],
                actual_reference_and_motion=True, no_controller_rerun=True)


def narrative(rows):
    lines = ['这次改了什么',
        '两组均使用67球和原生三维检查。优化组只开启：等价的预测点距离查询加速；优化超时后重建最后接受的候选，通过原生和最新环境检查才交付。',
        '没有放宽轮速、缩碰撞球或取消安全检查；优化预算仍为首次1.5秒、重规划0.6秒。',
        '静态和中段横穿各跑一组优化前/后，共4次完整在线数值ROS闭环。不是Gazebo或实车。',
        '', '实际结果']
    for scene in SCENES:
        pair = {r['group']: r for r in rows if r['scene_key'] == scene}
        a, b = [pair[g] for g in GROUPS]
        lines += [NAMES[scene]+'：',
            '  从发现参考受阻，到MPPI收到新参考：%s → %s秒。' %
            (shown(a['delivery']['first_blocked_to_consume_s'], 3), shown(b['delivery']['first_blocked_to_consume_s'], 3)),
            '  已完成重规划中，成功次数/完成次数：%d/%d → %d/%d。' %
            (a['remani']['refresh_accepted'], a['remani']['refresh_completed'], b['remani']['refresh_accepted'], b['remani']['refresh_completed'])]
        for r in (a, b):
            d, m, n = r['delivery'], r['mppi'], r['remani']
            lines += ['  %s：%s；到达目标=%s；自身球模型采样接触=%s；最长停车%s秒。' %
                (LABELS[r['group']], '记录有效' if r['valid_experiment'] else '记录无效',
                 '是' if r['reached_goal'] else '否', '有' if r['model_sample_contact'] else '无', shown(r['measured_stops']['longest_s'], 2)),
                '    超时候选验收尝试%d次；原生验收通过%d次；最终发布%d次；MPPI实际收到%d次。' %
                (d['salvage_attempts'], d['salvage_native_pass'], d['salvage_published'], d['salvage_consumed']),
                '    求解触及预算%d次：候选最终通过后端验收%d次，后端最终失败%d次。触及预算不再一律等同交付失败。' %
                (d['optimizer_timeout_events'], d['optimizer_timeout_final_backend_accepted'], d['optimizer_timeout_final_backend_failed']),
                '    纯L-BFGS求解中位数%s毫秒；实际进入的原生验收中位数%s毫秒（n=%d，包含正常和超时候选）。' %
                (shown(d['lbfgs_wall_ms']['p50']), shown(d['native_validation_ms']['p50']), d['native_validation_ms']['n']),
                '    MPPI求解＋验收P95=%s毫秒；超期标签%d次。' % (shown(m['callback_ms']['p95']), m['deadline_labels'])]
            if n['waiting_for_refresh_at_end']:
                lines.append('    结束时仍未收到新参考，至少已等%s秒；不能记成零耗时。' % shown(n['observed_refresh_wait_lower_bound_s'], 2))
            for req in n['request_rows']:
                calls = req.get('optimizer_calls', [])
                lines.append('    请求%s：%s；搜索%s毫秒，优化器[%s]毫秒，最新环境验收%s毫秒，整次%s毫秒。' %
                    (req.get('request_id'), req.get('outcome_class', '未完成'),
                     shown(req.get('stage_wall_ms', {}).get('search')), ', '.join(shown(c.get('wall_ms')) for c in calls) or '未进入',
                     shown(req.get('stage_wall_ms', {}).get('live_acceptance')), shown(req.get('request_wall_ms'))))
                for call in calls:
                    lines.append('      优化器内部：纯求解%s毫秒，原生验收%s毫秒，触及预算=%s，最终后端通过=%s。' %
                        (shown(call.get('lbfgs_wall_ms')), shown(call.get('native_validation_ms')),
                         call.get('force_stop') == 3 if 'force_stop' in call else '未记录', call.get('success')))
            for event in d['salvages']:
                lines.append('    超时恢复请求%s：候选重建%s毫秒，原生验收%s毫秒，通过=%s，发布=%s，收到=%s。' %
                    (event['request_id'], shown(event['rebuild_ms']), shown(event['validation_ms']),
                     event['native_result'], event['published'], event['consumed']))
    lines += ['', '该怎样理解',
        '主指标是“发现旧参考受阻，到MPPI真正拿到新参考”，不是只看成功那次优化花多久。',
        '“优化器”计时包含它内部的原生验收；搜索、初始化和整次请求存在包含关系，不可把所有字段直接相加。',
        '超时候选原生验收通过不等于已经交付，报告分别统计了真正发布和MPPI收到。',
        '两次在线闭环可能走不同路线，得到的优化输入也会不同。只能说明这次场景结果，不能把改善全部归因于某一个开关。',
        '每组一次，不据此声称总体成功率。若某组失败、碰撞或仍卡顿，照实保留，不能只凭GIF说问题已解决。',
        'static.gif和midcross.gif为这4次运行的真实记录：机器人用完整球模型投影，显示REMANI参考和记录到的MPPI搜索轨迹。', '']
    return '\n'.join(lines)


def report(batch, out):
    batch, out = Path(batch).resolve(), Path(out).resolve()
    if not (batch/'complete.json').is_file():
        raise ValueError('Finish every timed trial before reporting')
    if out == batch or batch in out.parents:
        raise ValueError('Report must be outside measured batch')
    plan = json.loads((batch/'plan.json').read_text())
    entries = plan['trials']
    if {(e['scene'], e['group']) for e in entries} != {(s, g) for s in SCENES for g in GROUPS} or len(entries) != 4:
        raise ValueError('Expected exactly the four prespecified online trials')
    paths = {}
    for entry in entries:
        path = Path(entry['directory'])
        if path.is_absolute() or '..' in path.parts:
            raise ValueError('Trial path must stay inside batch')
        paths[entry['scene'], entry['group']] = batch/path
    pairs = {s: check_pair_inputs(*(paths[s, g] for g in GROUPS)) for s in SCENES}
    rows = [summarize(paths[e['scene'], e['group']], e['scene'], e['group']) for e in entries]
    out.mkdir(parents=True, exist_ok=True)
    chart(rows, out/'交付耗时对照.png')
    gifs = {s: render_pair(*(paths[s, g] for g in GROUPS), out/(s+'.gif'), s) for s in SCENES}
    result = dict(plan=plan, trials=rows, pair_checks=pairs, gifs=gifs,
        scope='fresh unpaused numerical ROS SIL; not Gazebo/hardware; one seed per scene/group')
    (out/'report.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    (out/'实验总结.txt').write_text(narrative(rows))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch', required=True, type=Path)
    parser.add_argument('--out', required=True, type=Path)
    args = parser.parse_args()
    result = report(args.batch, args.out)
    print(json.dumps(dict(out=str(args.out), trials=len(result['trials']),
                         invalid=sum(not r['valid_experiment'] for r in result['trials'])), ensure_ascii=False))
