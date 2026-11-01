#!/usr/bin/env python3
"""Concise same-input delivery report. No paused candidate is called delivered."""
import argparse
import json
from pathlib import Path

from report_native67_probe import finite, one, records, shown, summarize_phase
from run_coupled_objective import sha
from run_delivery_same_input import PHASES


def phase_report(label, mode, rows, failures, resources):
    original = summarize_phase(label, .6, rows, failures)
    resource = one(resources, event='end', stage='optimizer', label=label)
    resource_fields = ('candidate_mode', 'candidate_available', 'candidate_accepted_steps',
        'candidate_source_initial', 'salvage_attempt', 'salvage_rebuilt', 'salvage_rebuild_ms',
        'salvage_native_result', 'salvage_validation_ms', 'native_validation_ms', 'lbfgs_wall_ms',
        'lbfgs_result', 'force_stop', 'success', 'input_hash')
    metrics = {key: original.get(key) for key in ('phase', 'budget_s', 'success', 'reason',
        'lbfgs_wall_ms', 'lbfgs_thread_cpu_ms', 'accepted_iterations', 'evaluate_entries',
        'evaluate_completed', 'evaluate_early_or_exception', 'line_search_trial_evaluations',
        'lbfgs_result', 'force_stop', 'backend_decision_wall_ms', 'backend_decision_thread_cpu_ms',
        'edt_query_ms', 'edt_share_of_callback', 'obstacle_cost_ms', 'profile', 'post_solver_audit_s')}
    metrics.update(mode=mode, resource={key: resource.get(key) for key in resource_fields},
        first_signature=original['first_signature'], candidates=original['candidates'],
        context={key: original['solve_summary'].get(key) for key in
            ('effective_initial_states', 'effective_final_states', 'raw_seq', 'esdf_seq',
             'frame_now', 'frame_sequence', 'prediction_hash', 'sim', 'sim_end', 'wheel_limit', 'input_hash')},
        checks=dict(original_diagnostic_record_valid=original['valid'],
            matching_resource=bool(resource) and resource.get('success') is original['success']
                and str(resource.get('input_hash')) == str(original['solve_summary'].get('input_hash')),
            candidate_mode=resource.get('candidate_mode') == mode),
        native_candidate_not_live_delivery=True)
    return metrics


def summarize(batch):
    batch = Path(batch).resolve()
    plan = json.loads((batch/'plan.json').read_text())
    job = plan['trials'][0]
    trial = batch/job['directory']
    def metadata(name):
        path = trial/name
        return json.loads(path.read_text()) if path.exists() else {}
    complete_path = batch/'complete.json'
    complete = json.loads(complete_path.read_text()) if complete_path.exists() else {}
    status = metadata('run_status.json')
    rows = records(trial/'delivery_replay.jsonl')
    failures = records(trial/'failure_trace.jsonl')
    resources = records(trial/'solver_trace.jsonl')
    timeline = records(trial/'dual_timeline.jsonl')
    phases = [phase_report(label, mode, rows, failures, resources) for mode, label in enumerate(PHASES)]
    capture = one(rows, kind='capture')
    done = metadata('diagnostic_complete.json')
    end = one(rows, kind='complete')
    contexts = [phase['context'] for phase in phases]
    context_keys = ('effective_initial_states', 'effective_final_states', 'raw_seq', 'esdf_seq',
        'frame_now', 'frame_sequence', 'prediction_hash', 'sim', 'wheel_limit', 'input_hash')
    observations = [r for r in rows if r.get('kind') == 'native_dynamic_observation']
    published = [r for r in timeline if r.get('request_id') == capture.get('request_id') and
        r.get('event') in ('reference_publish', 'state_contract_reference_publish')]
    checks = dict(
        same_first_x_cost_gradient=all(p['first_signature'].get('first_seen') is True for p in phases)
            and phases[0]['first_signature'] == phases[1]['first_signature'],
        same_context=bool(capture) and all(contexts[0].get(k) is not None
            and contexts[0][k] == contexts[1].get(k) for k in context_keys),
        frozen_simulation=all(finite(c.get('sim')) and c['sim'] == c.get('sim_end') for c in contexts),
        native_dynamic_frozen=bool(observations) and all(r.get('matches_entry') is True for r in observations),
        valid_nonempty_dynamic=capture.get('native_dynamic_count', 0) > 0,
        ordinal=capture.get('refresh_ordinal') == job['ordinal'],
        body67=len(metadata('native_collision_3d_body.json').get('body', [])) == 67,
        never_published=bool(capture) and not published and
            done.get('diagnostic_candidates_published') is False and end.get('diagnostic_candidates_published') is False,
        completion=all(done.get(k) is True and end.get(k) is True for k in
            ('frozen_stable', 'same_first_fg', 'audits_complete', 'native_dynamic_observed_stable')),
        lifecycle=complete.get('complete') is True and status.get('diagnostic_complete') is True and
            status.get('graph_audited') is True and status.get('runner_error') is None and
            not status.get('cleanup_errors') and status.get('required_process_failed') is False and
            status.get('production_hashes_unchanged') is True and status.get('build_manifest_unchanged') is True,
        manifest=metadata('manifest.json').get('delivery_same_input_experiment') is True and
            plan.get('diagnostic_phases') == list(PHASES) and plan.get('budgets_s') == [.6, .6],
        frozen_switch_source=metadata('manifest.json').get('delivery_same_input', {}).get(
            'switch_source_proof', {}).get('valid') is True)
    sources = {}
    for name in ('manifest.json', 'run_status.json', 'coupling_parameters.json', 'delivery_replay.jsonl',
                 'failure_trace.jsonl', 'solver_trace.jsonl', 'dual_timeline.jsonl'):
        path = trial/name
        if not path.exists():
            path = Path(str(path)+'.gz')
        if path.exists():
            sources[path.name] = sha(path)
    return dict(schema='delivery_same_input_report_v1', source=str(batch), trial=str(trial),
        captured=bool(capture), valid=all(checks.values()) and all(all(p['checks'].values()) for p in phases),
        checks=checks, phases=phases, run_status=status,
        capture_request_id=capture.get('request_id'), input_hash=capture.get('input_hash'),
        capture_predicate=plan.get('capture_predicate'), source_sha256=sources,
        navigation_metrics_valid=False, diagnostic_candidates_published=False if checks['never_published'] else None,
        limitations=plan.get('limitations', []))


def conclusion(report):
    if not report['captured']:
        return '没有达到预先指定的捕获条件，保留结果，没有另选输入。本轮没有有效同输入对照。'
    if not report['valid']:
        return '记录或同输入核验未全部通过，暂不能把两组差异作为已确认的优化效果。'
    old, new = report['phases']
    if old['success'] and new['success']:
        return '两组都通过原生后端验收；本次只能比较计算时间，不能说修好了一个原本失败的请求。'
    if not old['success'] and new['success']:
        return '同一个输入、相同0.6秒优化预算，原组失败，改进组通过原生后端验收。尚不等于在线已安全交付。'
    if old['success'] and not new['success']:
        return '同一个输入原组通过，改进组失败，保留这次退化；不能把本轮说成改善。'
    return '两组都没有通过原生后端验收，本轮没有解决这个输入的轨迹交付问题。'


def write_text(report, out):
    lines = ['同输入轨迹交付对照', '', '怎么测',
        '67球、中段横穿、6米目标，seed11，固定第一个合格普通重规划输入。',
        '两组优化预算都是0.6秒。原组维持旧逻辑；改进组启用等价距离查询加速和超时候选重建验收。',
        '机器人和障碍输入暂停不变；候选不发布。本报告不代表在线运动效果，在线GIF另看闭环实验。',
        '', '结果', conclusion(report), '同输入与记录核验：'+('通过' if report['valid'] else '未完整通过')]
    if report['captured']:
        for phase in report['phases']:
            r = phase['resource']
            lines += ['', ('原组' if phase['mode'] == 0 else '改进组')+'：',
                '优化器耗时 %s ms，接受 %s 步，完整目标函数评估 %s 次。' %
                    (shown(phase['lbfgs_wall_ms']), phase['accepted_iterations'], phase['evaluate_completed']),
                '从优化开始到后端作出最终判断 %s ms；原生验收 %s ms。' %
                    (shown(phase['backend_decision_wall_ms']), shown(r.get('native_validation_ms'))),
                '后端最终%s，原因 %s；原始停止标志 %s 不直接代表最终失败。' %
                    ('通过' if phase['success'] is True else '未通过', phase['reason'], phase['force_stop']),
                '距离查询累计 %s ms，占目标函数回调 %s%%（子项不可与总时间相加）。' %
                    (shown(phase['edt_query_ms']), shown(phase['edt_share_of_callback'])),
                '超时候选恢复：%s；重建 %s ms，恢复后的原生检查 %s。' %
                    ('尝试过' if r.get('salvage_attempt') == 1 else '未尝试', shown(r.get('salvage_rebuild_ms')),
                     '通过' if r.get('salvage_native_result') == 1 else '未通过' if r.get('salvage_native_result') == 0 else '未执行')]
            for c in phase['candidates']:
                title = {'initial': '初始解', 'last_accepted': '最后接受点',
                         'solver_return_x': '求解器返回x（不保证是恢复所用x）'}.get(c['stage'], c['stage'])
                lines.append('事后补查 %s：%s。' % (title,
                    '通过原生验收' if c['native_pass'] is True else '未通过：'+str(c['native_reason'])))
    failures = [k for k, good in report['checks'].items() if not good]
    failures += [p['phase']+'/'+k for p in report['phases'] for k, good in p['checks'].items() if not good]
    if failures:
        lines += ['', '未通过的核验：'+', '.join(failures)]
    lines += ['', '不要混淆',
        '0.6秒只限制继续优化；候选重建和原生验收另占实际时间，不能把最终后端判断说成严格0.6秒。',
        '诊断析构时还会重建初始/接受/返回候选补查，RESOURCE整段wall包含这些补查；上面的后端判断计时不包含它们。',
        '求解器返回x与保存的最后接受x分别记录；恢复效果以salvage_native_result及真实backend success为准。',
        '这次同时打开两项改动，不能单独归因其中某一项；只测一个输入，不估计整体成功率。',
        '候选开关有运行时0/1记录；距离查询开关由冻结构建源码核对，不宣称记录了每次查询分支。',
        '仍要看在线 request→发布→MPPI消费以及最小间隙，才能判断真正交付和运动效果。']
    (out/'简要结论.txt').write_text('\n'.join(lines)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch', type=Path, required=True)
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    out = args.out or args.batch/'report'
    report = summarize(args.batch)
    out.mkdir(parents=True, exist_ok=False)
    (out/'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    write_text(report, out)
    print('DELIVERY_SAME_INPUT_REPORT', out, 'valid='+str(report['valid']))


if __name__ == '__main__':
    main()
