#!/usr/bin/env python3
"""Plain-language report of one paused native67 same-input budget probe."""
import argparse
import gzip
import json
import math
from pathlib import Path

from run_coupled_objective import sha
from run_native67_probe import PHASES


def records(path):
    path = Path(path)
    if not path.exists():
        path = Path(str(path)+'.gz')
    if not path.exists():
        return []
    with (gzip.open(path, 'rt') if path.suffix == '.gz' else path.open()) as stream:
        return [json.loads(line) for line in stream if line.strip()]


def one(rows, **match):
    found = [row for row in rows if all(row.get(key) == value for key, value in match.items())]
    return found[0] if len(found) == 1 else {}


def finite(value):
    return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)


def shown(value, places=1):
    return ('%.*f' % (places, value)) if finite(value) else '未记录'


def summarize_phase(label, budget, rows, failure_rows):
    solve = one(rows, kind='solve_summary', phase=label)
    timing = one(rows, kind='native67_timing', phase=label)
    failure = one(failure_rows, kind='solve', label=label)
    returned = one(rows, kind='recursive_return', phase=label)
    canonical = [r for r in rows if r.get('kind') == 'canonical_candidate' and r.get('phase') == label]
    candidates = [dict(stage=r.get('stage'), iteration=r.get('iteration'),
        solver_elapsed_s=r.get('solver_elapsed_s'), cost=r.get('cost'),
        native_pass=r.get('native_IsTrajSafe_pass'), native_reason=r.get('native_reason'),
        native_rejection_value=r.get('native_rejection_value'),
        native_rejection_time=r.get('native_rejection_time'),
        residual=r.get('full_grid_residual_audit')) for r in canonical]
    audits = [r for r in rows if r.get('kind') == 'post_solver_audit_end' and r.get('phase') == label]
    profile = timing.get('profile_wall_ms_inclusive') or {}
    cost_ms, obstacle_ms = profile.get('cost_callback'), profile.get('obstacle_cost')
    valid = (bool(solve) and bool(timing) and bool(failure) and bool(returned) and
        solve.get('budget_s') == failure.get('budget_s') == budget and
        str(solve.get('input_hash')) == str(failure.get('input_hash')) and
        solve.get('native_return_success') is failure.get('success') and
        solve.get('native_return_success') is returned.get('success') and
        solve.get('reason') == failure.get('reason') and
        len(canonical) >= 2 and all(r.get('reconstructed') is True for r in canonical) and
        sum(r.get('stage') == 'solver_return_x' for r in canonical) == 1)
    return dict(phase=label, budget_s=budget, valid=valid,
        success=solve.get('native_return_success'), reason=solve.get('reason'),
        lbfgs_wall_ms=timing.get('solver_wall_ms'), lbfgs_thread_cpu_ms=timing.get('solver_thread_cpu_ms'),
        accepted_iterations=timing.get('accepted_iterations'),
        legacy_iterations_field_is_completed_callbacks=failure.get('iterations'),
        evaluate_entries=timing.get('evaluate_entries'), evaluate_completed=timing.get('evaluate_completed'),
        evaluate_early_or_exception=timing.get('evaluate_early_or_exception'),
        line_search_trial_evaluations=timing.get('line_search_trial_evaluations'),
        lbfgs_result=failure.get('lbfgs_result'), force_stop=failure.get('force_stop'),
        backend_decision_wall_ms=timing.get('backend_decision_wall_ms'),
        backend_decision_thread_cpu_ms=timing.get('backend_decision_thread_cpu_ms'),
        cost_callback_ms=cost_ms, callback_thread_cpu_ms=timing.get('callback_thread_cpu_ms'),
        obstacle_cost_ms=obstacle_ms,
        obstacle_share_of_cost_callback=(100*obstacle_ms/cost_ms
            if finite(obstacle_ms) and finite(cost_ms) and cost_ms > 0 else None),
        first_cost=solve.get('first_cost'), final_cost=failure.get('final_cost'),
        first_signature={key: solve.get(key) for key in
            ('first_seen', 'first_cost', 'first_x_hash', 'first_gradient_hash')},
        solve_summary=solve, candidates=candidates,
        profile=profile, failure_record=failure, native67_timing=timing,
        post_solver_audit_s=sum(a['audit_wall_s'] for a in audits if finite(a.get('audit_wall_s'))),
        audit_count=len(audits),
        timing_note='failure wall/thread CPU includes posterior reconstruction; only LBFGS and explicit decision timers are solver measurements')


def summarize(batch):
    batch = Path(batch).resolve()
    plan = json.loads((batch/'plan.json').read_text())
    complete = json.loads((batch/'complete.json').read_text()) if (batch/'complete.json').exists() else {}
    job = plan['trials'][0]
    trial = batch/job['directory']
    manifest = json.loads((trial/'manifest.json').read_text())
    status = json.loads((trial/'run_status.json').read_text())
    rows = records(trial/'delivery_replay.jsonl')
    failures = records(trial/'failure_trace.jsonl')
    timeline = records(trial/'dual_timeline.jsonl')
    capture = one(rows, kind='capture')
    done = json.loads((trial/'diagnostic_complete.json').read_text()) if (trial/'diagnostic_complete.json').exists() else {}
    end = one(rows, kind='complete')
    phases = [summarize_phase(label, budget, rows, failures)
              for label, budget in zip(PHASES, (.6, 3.))]
    context_keys = ('effective_initial_states', 'effective_final_states', 'raw_seq', 'esdf_seq',
                    'frame_now', 'frame_sequence', 'prediction_hash', 'sim', 'wheel_limit', 'input_hash')
    ss = [p['solve_summary'] for p in phases]
    observations = [r for r in rows if r.get('kind') == 'native_dynamic_observation']
    published = [r for r in timeline if r.get('request_id') == capture.get('request_id')
                 and r.get('event') in ('reference_publish', 'state_contract_reference_publish')]
    body_path = trial/'native_collision_3d_body.json'
    body = json.loads(body_path.read_text()).get('body', []) if body_path.exists() else []
    same_input = dict(
        identical_first_x_cost_gradient=all(p['first_signature'].get('first_seen') is True for p in phases)
            and phases[0]['first_signature'] == phases[1]['first_signature'],
        identical_recorded_context=all(ss) and all(key in ss[0] and ss[0][key] == ss[1].get(key) for key in context_keys),
        simulation_time_frozen=all(s.get('sim') == s.get('sim_end') and finite(s.get('sim')) for s in ss),
        native_dynamic_observations_match=bool(observations) and all(r.get('matches_entry') is True for r in observations),
        nonempty_dynamic_snapshot=bool(capture.get('native_dynamic_count', 0) > 0),
        no_candidate_publication=bool(capture) and not published
            and done.get('diagnostic_candidates_published') is False
            and end.get('diagnostic_candidates_published') is False,
        actual_67_spheres=len(body) == 67,
        ordinal=bool(capture) and capture.get('refresh_ordinal') == job['ordinal'],
        completed_consistent=all(done.get(k) is True and end.get(k) is True for k in
            ('frozen_stable', 'same_first_fg', 'audits_complete', 'native_dynamic_observed_stable')))
    intact = (complete.get('complete') is True and status.get('diagnostic_complete') is True and
        status.get('graph_audited') is True and status.get('runner_error') is None and
        status.get('required_process_failed') is False and not status.get('cleanup_errors') and
        status.get('production_hashes_unchanged') is True and status.get('build_manifest_unchanged') is True)
    sources = {name: sha(trial/name) for name in ('manifest.json', 'run_status.json', 'coupling_parameters.json')}
    for name in ('delivery_replay.jsonl', 'failure_trace.jsonl', 'dual_timeline.jsonl'):
        path = trial/name
        if not path.exists():
            path = trial/(name+'.gz')
        if path.exists():
            sources[path.name] = sha(path)
    return dict(schema='native67_probe_report_v1', source=str(batch), trial=str(trial),
        valid=bool(intact and all(same_input.values()) and all(p['valid'] for p in phases)),
        captured=bool(capture), run_status=status, same_input_checks=same_input,
        capture=capture, phases=phases, source_sha256=sources,
        navigation_metrics_valid=False, diagnostic_candidates_published=False if same_input['no_candidate_publication'] else None,
        limitations=plan['limitations'], capture_predicate=plan['capture_predicate'])


REASONS = {'ok': '通过原生验收', 'budget_timeout': '预算到时',
    'left_wheel_speed': '左轮速度超限', 'right_wheel_speed': '右轮速度超限',
    'left_wheel_acceleration': '左轮加速度超限', 'right_wheel_acceleration': '右轮加速度超限',
    'base_collision': '底盘碰撞', 'arm_collision': '机械臂碰撞'}


def conclusion(report):
    if not report['captured']:
        return '没有捕获到预定的动态重规划输入，本轮不能回答预算是否够用；未换输入补成功。'
    if not report['valid']:
        return '同输入或记录核验未完整通过，本轮不能据此判断预算效果。'
    a, b = report['phases']
    if a['success']:
        return '0.6秒组已经通过：这个输入没有复现交付失败，不能用它证明延长预算能解决此前超时。'
    if b['success']:
        return '同一输入在0.6秒未通过，3秒通过：本次请求受预算限制，增加预算确实帮助原生后端得到合格轨迹。'
    if b['reason'] == 'budget_timeout':
        return '同一输入给到3秒仍超时：本次延长预算仍未拿到合格返回，下一步应看目标函数评估负担与迭代收敛进展。'
    return ('同一输入给到3秒仍未通过，最后卡在“'+REASONS.get(b['reason'], str(b['reason']))+
            '”：延长预算没有直接解决这个请求的约束或验收问题。')


def write_text(report, out):
    if not report['captured']:
        lines = ['本次诊断未完成，不能评价0.6秒和3秒预算的效果。',
            '没有记录到两组同输入求解结果；不能把这次运行算作算法失败或预算不足的证据。']
        if report['run_status'].get('required_process_failed'):
            lines += ['规划进程在准备诊断输入时异常退出，属于实验记录代码的问题。',
                '请以原始launch_stdout.log和后续独立修正构建记录为准；本目录保留，不覆盖。']
        else:
            lines += ['未到达预先指定的捕获条件，保留这次结果，没有另选一个输入补成功。']
        lines += ['没有有效计时数据，因此不生成预算对照图或避障GIF。']
        (out/'实验结论.txt').write_text('\n'.join(lines)+'\n')
        return
    lines = ['本轮要弄清楚什么',
        '动态障碍出现后，REMANI慢，是0.6秒来不及算出合格轨迹，还是算完了仍过不了原生约束。', '',
        '怎么测', '最新67球、6米目标、中段横穿场景；预先固定seed11，取第一个符合条件的普通动态重规划输入。',
        '冻结这一次的机器人和障碍输入，分别给0.6秒、3秒。轮速、初值、碰撞规则、停止规则保持相同。',
        '候选不会发布。这是一次后端诊断，不能直接说在线避障改善了。', '',
        '结果', conclusion(report), '同输入及记录核验：'+('通过' if report['valid'] else '未完整通过')]
    for p in report['phases']:
        lines += ['', '%s秒预算：%s' % (shown(p['budget_s']), REASONS.get(p['reason'], str(p['reason']))),
            'L-BFGS计算 %s ms（其中线程CPU %s ms），实际接受 %s 步。' %
                (shown(p['lbfgs_wall_ms']), shown(p['lbfgs_thread_cpu_ms']), p['accepted_iterations']),
            '进入目标函数 %s 次，完成 %s 次，提前退出 %s 次；线搜索试探 %s 次。' %
                (p['evaluate_entries'], p['evaluate_completed'], p['evaluate_early_or_exception'], p['line_search_trial_evaluations']),
            '后端至原生最终判定 %s ms；预算只管L-BFGS，不包含后续原生验收。' % shown(p['backend_decision_wall_ms']),
            '碰撞代价累计 %s ms，占目标函数评估时间 %s%%。这些计时存在包含关系，不能加起来当总耗时。' %
                (shown(p['obstacle_cost_ms']), shown(p['obstacle_share_of_cost_callback'])),
            '总代价从 %s 降到 %s。' % (shown(p['first_cost'], 2), shown(p['final_cost'], 2))]
        for c in p['candidates']:
            lines.append('%s候选：原生检查%s，原因 %s。' %
                ('初始' if c['stage'] == 'initial' else '求解器返回' if c['stage'] == 'solver_return_x' else '最后接受步',
                 '通过' if c['native_pass'] is True else '未通过' if c['native_pass'] is False else '未记录',
                 REASONS.get(c['native_reason'], str(c['native_reason']))))
            residual = c['residual'] or {}
            if c['stage'] == 'solver_return_x' and residual:
                lines += ['返回曲线的完整采样检查：底盘最小硬间隙 %s m，机械臂 %s m（负值是越界）。' %
                    (shown(residual.get('min_base_hard_gap_m'), 3), shown(residual.get('min_arm_hard_gap_m'), 3)),
                    '左右轮最大速度 %s / %s rad/s，允许 %s；最大加速度 %s / %s rad/s²，允许 %s。' %
                    tuple(shown(residual.get(k), 3) for k in ('max_left_omega', 'max_right_omega',
                        'omega_hard_limit_with_tolerance', 'max_left_alpha', 'max_right_alpha', 'alpha_hard_limit_with_tolerance'))]
    lines += ['', '结论边界',
        '这里只回答这一个新请求的后端问题，不是昨天失败请求的完整重放，也不能估计整体成功率。',
        '候选重建和事后验收的耗时没有算进L-BFGS；原生采样通过也不能保证未来动态执行安全。',
        '分项计时本身有少量开销，两臂一致启用；本轮耗时不当作生产版关闭计时后的实测速度。',
        '冻结后暂时不含MPPI运动和并行负载变化，在线交付仍需单独验证。旧日志iterations字段其实是完成的目标函数次数，不当作接受步数。',
        '图中展示真实求解记录；本轮没有移动过程对照，不制作避障效果GIF。']
    (out/'实验结论.txt').write_text('\n'.join(lines)+'\n')


def draw(report, out):
    from PIL import Image, ImageDraw, ImageFont
    from render_dual_timescale import FONT
    im = Image.new('RGB', (1120, 625), 'white')
    d = ImageDraw.Draw(im)
    title, font, small = [ImageFont.truetype(FONT, s) for s in (26, 21, 18)]
    d.text((25, 18), '67球：同一个动态重规划输入，增加预算是否有效', font=title, fill='#222222')
    d.text((25, 60), '只比较0.6秒与3秒；保持初值、原生约束和停止规则相同', font=small, fill='#555555')
    colors = ['#7c858d', '#2f79b5']
    maximum = max([p['lbfgs_wall_ms'] for p in report['phases'] if finite(p['lbfgs_wall_ms'])]+[600.])
    for i, p in enumerate(report['phases']):
        y = 115+i*160
        d.text((25, y), '%s秒预算' % shown(p['budget_s']), font=font, fill='#222222')
        value = p['lbfgs_wall_ms']
        if finite(value):
            width = int(630*value/maximum)
            d.rectangle((170, y, 170+width, y+30), fill=colors[i])
            d.text((180+width, y+2), shown(value)+' ms', font=small, fill='#222222')
        d.text((170, y+42), '接受 %s步 / 目标函数 %s次 / 碰撞代价 %s ms' %
               (p['accepted_iterations'], p['evaluate_entries'], shown(p['obstacle_cost_ms'])), font=small, fill='#444444')
        d.text((170, y+74), '返回：'+REASONS.get(p['reason'], str(p['reason'])), font=font, fill='#333333')
        d.text((170, y+107), '总代价 '+shown(p['first_cost'], 2)+' → '+shown(p['final_cost'], 2), font=small, fill='#555555')
    d.line((25, 448, 1095, 448), fill='#cccccc')
    text = conclusion(report)
    for n in range(0, len(text), 43):
        d.text((25, 465+(n//43)*28), text[n:n+43], font=small, fill='#333333')
    d.text((25, 558), '暂停诊断；不是在线交付时间。碰撞代价属于目标函数的一部分，不能与总时间相加。', font=small, fill='#666666')
    d.text((25, 589), '一个预定输入，不代表全部动态场景；候选未发布。', font=small, fill='#666666')
    im.save(out/'同输入预算对照.png')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--batch', type=Path, required=True)
    p.add_argument('--out', type=Path)
    args = p.parse_args()
    out = args.out or args.batch/'report'
    report = summarize(args.batch)
    out.mkdir(parents=True, exist_ok=False)
    (out/'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    write_text(report, out)
    if report['captured'] and any(finite(p['lbfgs_wall_ms']) for p in report['phases']):
        draw(report, out)
    print('NATIVE67_REPORT', out, 'valid='+str(report['valid']))


if __name__ == '__main__':
    main()
