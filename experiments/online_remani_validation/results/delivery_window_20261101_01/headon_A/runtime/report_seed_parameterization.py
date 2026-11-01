#!/usr/bin/env python3
"""Offline static OFF/ON seed-parameterization report; no online work on import."""
import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import json
from pathlib import Path

from report_boundary_admission import end_to_end
from report_delivery_stability import summarize_search
from report_domain_recovery import assert_plot_quiet, digest, read_json, read_records
from report_state_contract import (analyze_cycles, backend_scope_evidence, finite,
    integer, reference_key, unsigned_identity)
from report_guide_applicability import sampled_payload


LIMITATIONS = [
    '两个seed的静态原生ROS数值SIL，固定机械臂；不是Gazebo、实车或统计显著性证明。',
    '两臂同binary、同runtime和相同记录参数；实时闭环会分叉，不声称每个native规划请求输入相同。',
    'initial按原生FSM GEN_NEW_TRAJ=2识别并单列；REPLAN_TRAJ=3才计入重规划分母，不按request_id=1猜测。',
    'A*成功不等于最终前端成功；flat种子、机械臂采样/RRT和后端求解是不同环节。',
    '种子事件没有公开wrapper probe id，只能在同pid/request恰有一个wrapper和一个事件时关联；多wrapper一律歧义。valid只表示记录的非空段和有限正总时长，不是碰撞/轮速可行性证明。',
    '后端失败仅在唯一input_hash和完整backend scope关联成立时归因；重复或缺日志记未知。',
    '发布、MPPI实际online_step采用、验证命令是不同层级，采用不等于持续推进或安全认证。',
    '端到端为scene appearance=result.motion_anchor+4至首条其后新参考的实际kernel entry，包含全部失败/重试等待；不是精确感知发布时刻。',
    '只修种子参数化，不放宽外形、余量、轮速、求解预算或最终硬检查；单次速度变化不能证明因果收益。',
]

SHARED_MANIFEST_FIELDS = ('mode', 'scene', 'duration', 'scenario_start', 'port',
    'parameter_overrides', 'experiment_profile', 'reference_source', 'mppi_revision',
    'perception_mode', 'native_binaries_sha256', 'mppi_library_sha256',
    'native_safety_chain', 'local_execution_owner', 'shared_collision',
    'shared_collision_source_sha256', 'record_search', 'local_prediction_s',
    'reference_delay_s', 'scenario', 'planner_variant', 'progress_contract',
    'shared_collision_runtime_enabled')


def one(rows, event):
    found = [r for r in rows or [] if r.get('event') == event]
    return found[0] if len(found) == 1 else None


def phase_for(rows):
    begin = one(rows, 'request_begin')
    if begin is None:
        return 'unknown'
    return {2: 'initial', 3: 'replan'}.get(begin.get('fsm'), 'unknown')


def seed_evidence(rows, wrappers, expected_fix=None):
    events = [r for r in rows if r.get('event') == 'seed_parameterization']
    unique = len(events) == 1 and len(wrappers) == 1
    result = []
    for event in events:
        fields = (event.get('seed_parameterization_schema') == 'seed_parameterization_v1' and
            all(isinstance(event.get(k), bool) for k in ('enabled', 'applied', 'valid')) and
            all(integer(event.get(k)) and event[k] >= 0 for k in ('input_points', 'output_segments')) and
            finite(event.get('signed_start_speed')) and integer(event.get('first_gear')) and
            event['first_gear'] in (-1, 0, 1) and isinstance(event.get('reason'), str) and bool(event['reason']) and
            event.get('probe_call_id_available') is False)
        flag = event.get('enabled') is expected_fix if isinstance(expected_fix, bool) else None
        status = (fields and not (event.get('applied') is True and event.get('enabled') is not True) and
            not (event.get('valid') is True and event.get('output_segments') == 0))
        wrapper = wrappers[0] if unique else None
        children = (wrapper.get('native_children') or []) if wrapper else []
        native_success = len(children) == 1 and children[0].get('return_code') in (1, 2)
        certified = status and unique and flag is True
        result.append(dict(record=event, schema_fields_valid=fields, switch_matches_manifest=flag,
            applied_enabled_consistent=status, unique_request_wrapper_association=unique,
            association_scope='unique same pid/request event and wrapper only; no probe call ID',
            linked_wrapper_call_id=wrapper.get('call_id') if wrapper else None,
            evidence_valid=certified,
            certified_applied=certified and event.get('applied') is True,
            repaired_positive_seed=certified and event.get('enabled') is True and event.get('applied') is True and event.get('valid') is True and event.get('reason') == 'ok',
            astar_success_with_invalid_seed=certified and native_success and event.get('valid') is False,
            empty_output_seed=certified and event.get('output_segments') == 0))
    return result


def adoption_evidence(native, cycles, odom, consumed, commits, observer):
    """Exact reference/odom proof; never timestamp-nearest association."""
    pubs = [r for r in native or [] if r.get('event') == 'state_contract_reference_publish']
    counts = Counter(unsigned_identity(r.get('transaction_fnv64')) for r in pubs)
    analyzed = analyze_cycles(cycles, odom, consumed, commits, observer)['rows']
    originals = defaultdict(list)
    for row in cycles or []:
        originals[row.get('cycle_id')].append(row)
    result = []
    for pub in pubs:
        tx = unsigned_identity(pub.get('transaction_fnv64'))
        matches = []
        for row in analyzed:
            source = originals[row['cycle_id']]
            if len(source) != 1 or source[0].get('first_kernel_cycle_for_reference') is not True:
                continue
            if (row.get('online_step_invoked') is not True or row['unique_cycle_id'] is not True or
                row['solve'].get('exact_message_observed') is not True or
                row['solve'].get('state_equals_identity') is not True or
                row['reference'].get('complete_exact_reference_chain') is not True):
                continue
            entry = row.get('online_step_entry_ros_time')
            if (tx is None or counts[tx] != 1 or
                unsigned_identity(row['reference'].get('transaction_fnv64')) != tx or
                not finite(entry) or not finite(pub.get('capture_sim')) or entry < pub['capture_sim']):
                continue
            refs = [r for r in consumed or [] if reference_key(r, True) == reference_key(source[0])]
            if len(refs) != 1 or sampled_payload(refs[0])[0]['payload_sha256_verified'] is not True:
                continue
            matches.append(dict(cycle_id=row['cycle_id'], online_step_entry_ros_time=entry,
                kernel_code=source[0].get('kernel_code'),
                actual_validated_command=row.get('actual_validated_command'),
                complete_exact_python_chain=row.get('complete_exact_python_chain')))
        adopted = matches[0] if len(matches) == 1 else None
        result.append(dict(pid=pub.get('pid'), request_id=pub.get('request_id'), transaction_fnv64=tx,
            publish_capture_sim=pub.get('capture_sim'), unique_publication=tx is not None and counts[tx] == 1,
            exact_adopted=adopted is not None, adoption_matches=len(matches), adoption=adopted))
    return result


def summarize_requests(native, failures, adoptions=None, expected_fix=None):
    groups = defaultdict(list)
    for row in native or []:
        if integer(row.get('pid')) and row['pid'] > 1 and integer(row.get('request_id')) and row['request_id'] > 0:
            groups[(row['pid'], row['request_id'])].append(row)
    all_fixed = [r for rows in groups.values() for r in rows if r.get('event') == 'state_contract_backend_fixed']
    hash_counts = Counter(unsigned_identity(r.get('input_hash')) for r in all_fixed)
    records = []
    for (pid, request), rows in sorted(groups.items()):
        phase = phase_for(rows)
        search = summarize_search(rows)
        backends = []
        for fixed in (r for r in rows if r.get('event') == 'state_contract_backend_fixed'):
            value = unsigned_identity(fixed.get('input_hash'))
            scope = backend_scope_evidence(rows, fixed)
            matches = [r for r in failures or [] if r.get('kind') == 'solve' and r.get('label') == 'normal'
                and unsigned_identity(r.get('input_hash')) == value] if value is not None else []
            exact = scope.get('complete') is True and value is not None and hash_counts[value] == 1 and len(matches) == 1
            failure = matches[0] if exact else None
            backends.append(dict(input_hash=value, backend_scope=scope, exact_failure_link=exact,
                reason=failure.get('reason') if failure else None,
                success=failure.get('success') if failure else None,
                lbfgs_wall_ms=failure.get('lbfgs_wall_ms') if failure else None,
                budget_s=failure.get('budget_s') if failure else None))
        wrappers = search.get('records') or []
        terminal = [r for r in rows if r.get('event') == 'acceptance_reject']
        pubs = [r for r in rows if r.get('event') == 'state_contract_reference_publish']
        adopted = [r for r in adoptions or [] if r.get('pid') == pid and r.get('request_id') == request and r.get('exact_adopted') is True]
        fix = seed_evidence(rows, wrappers, expected_fix)
        end = one(rows, 'state_contract_request_end')
        begin = one(rows, 'request_begin')
        records.append(dict(pid=pid, request_id=request, phase=phase,
            request_begin_count=sum(r.get('event') == 'request_begin' for r in rows),
            fsm=begin.get('fsm') if begin else None,
            request_success=end.get('success') if end else None,
            search=search, wrapper_count=len(wrappers),
            frontend_success=wrappers[0]['frontend_success'] if len(wrappers) == 1 else None,
            astar_success_then_sampling_failed=any(r['astar_success_then_sampling_failed'] for r in wrappers),
            optimizer_stage_begin_count=sum(r.get('event') == 'stage_begin' and r.get('stage') == 'optimizer' for r in rows),
            backend_calls=len(backends), backend_records=backends,
            terminal_rejection_count=len(terminal), terminal_rejections=terminal,
            reference_publish_count=len(pubs), exactly_adopted_reference_count=len(adopted),
            seed_parameterization_records=fix))

    def counts(subset):
        if native is None:
            return dict(requests=None, wrapper_failures=None, optimizer_entered_requests=None,
                backend_reason_counts=None, terminal_rejections=None, published_requests=None, exactly_adopted_requests=None)
        backends = [b for r in subset for b in r['backend_records']]
        seeds = [e for r in subset for e in r['seed_parameterization_records']]
        entered = sum(r['optimizer_stage_begin_count'] > 0 for r in subset)
        return dict(requests=len(subset), wrapper_calls=sum(r['wrapper_count'] for r in subset),
            wrapper_successes=sum(r['frontend_success'] is True for r in subset),
            wrapper_failures=sum(r['frontend_success'] is False for r in subset),
            astar_success_then_sampling_failures=sum(r['astar_success_then_sampling_failed'] for r in subset),
            optimizer_entered_requests=entered,
            optimizer_entry_fraction=entered / len(subset) if subset else None,
            backend_calls=len(backends), backend_reason_counts=dict(Counter(b['reason'] for b in backends if b['exact_failure_link'])),
            unknown_backend_links=sum(not b['exact_failure_link'] for b in backends),
            terminal_rejections=sum(r['terminal_rejection_count'] for r in subset),
            published_requests=sum(r['reference_publish_count'] > 0 for r in subset),
            published_references=sum(r['reference_publish_count'] for r in subset),
            exactly_adopted_requests=sum(r['exactly_adopted_reference_count'] > 0 for r in subset),
            exactly_adopted_references=sum(r['exactly_adopted_reference_count'] for r in subset),
            seed_events=len(seeds), certified_seed_events=sum(e['evidence_valid'] for e in seeds),
            certified_fix_applied_events=sum(e['certified_applied'] for e in seeds),
            repaired_positive_seeds=sum(e['repaired_positive_seed'] for e in seeds),
            astar_success_with_invalid_seed=sum(e['astar_success_with_invalid_seed'] for e in seeds),
            empty_output_seeds=sum(e['empty_output_seed'] for e in seeds),
            seed_event_reasons=dict(Counter(e['record'].get('reason') for e in seeds)),
            seed_switch_observed=all(e['switch_matches_manifest'] is True for e in seeds) if seeds else None)
    return dict(present=native is not None, all=counts(records),
        initial=counts([r for r in records if r['phase'] == 'initial']),
        replan=counts([r for r in records if r['phase'] == 'replan']),
        unknown_phase=counts([r for r in records if r['phase'] == 'unknown']), records=records)


def trial_report(trial):
    trial = Path(trial)
    names = ('manifest.json', 'run_status.json', 'trial_audit.json', 'result.json')
    docs = {name: read_json(trial / name) for name in names}
    manifest = docs['manifest.json']; settings = manifest.get('coupling_experiment') or {}
    status = docs['run_status.json']; outcome = docs['result.json']
    if status.get('returncode') is None or status.get('result_written') is not True:
        raise ValueError('Incomplete trial: ' + str(trial))
    if not isinstance(settings.get('seed_parameterization_fix'), bool):
        raise ValueError('Missing explicit seed parameterization treatment')
    inputs = {}; hashes = {name: digest(trial / name) for name in names}
    for name in ('dual_timeline.jsonl', 'failure_trace.jsonl', 'state_mppi_cycles.jsonl',
                 'state_odom_events.jsonl', 'consumed_references.jsonl', 'reference_events.jsonl', 'observer_events.jsonl'):
        inputs[name], path = read_records(trial, name)
        if path is not None:
            hashes[path.name] = digest(path)
    native = inputs['dual_timeline.jsonl']
    chain_inputs = [inputs[name] for name in ('state_mppi_cycles.jsonl', 'state_odom_events.jsonl',
        'consumed_references.jsonl', 'reference_events.jsonl', 'observer_events.jsonl')]
    adoptions = adoption_evidence(native, *chain_inputs)
    runtime = manifest.get('experiment_sources_sha256') or {}
    runtime_files = {}
    for name, expected in runtime.items():
        if not name.endswith('.py'):
            continue
        path = trial / 'runtime' / name
        runtime_files[name] = dict(expected=expected, observed=digest(path) if path.is_file() else None)
    requests = summarize_requests(native, inputs['failure_trace.jsonl'], adoptions, settings['seed_parameterization_fix'])
    return dict(trial=trial.name, source=str(trial), seed=settings.get('seed'),
        fix=settings['seed_parameterization_fix'], settings=settings,
        run_status=status, audit=docs['trial_audit.json'], outcome=outcome,
        interface_valid=docs['trial_audit.json'].get('valid_experiment') is True,
        runtime_sha256=runtime, runtime_python_files=runtime_files,
        runtime_python_verified=bool(runtime_files) and all(r['observed'] == r['expected'] for r in runtime_files.values()),
        build=manifest.get('build'), build_manifest_sha256=manifest.get('build_manifest_sha256'),
        shared_manifest={k: manifest.get(k) for k in SHARED_MANIFEST_FIELDS},
        requests=requests, adoptions=adoptions,
        end_to_end=end_to_end(outcome, manifest.get('scene'), native, *chain_inputs),
        input_sha256=hashes,
        scope_checks=dict(static_scene=manifest.get('scene') == 'static', coupled=settings.get('group') == 'coupled',
            signed_low_speed=settings.get('signed_low_speed') is True,
            domain_recovery=settings.get('domain_recovery') is True,
            admission_disabled=settings.get('boundary_admission') is False,
            clearance_recovery_disabled=settings.get('clearance_recovery', False) is False,
            diagnostic_pause_disabled=settings.get('diagnostic_pause') is False,
            weights_zero=settings.get('execution_weight') == 0. and settings.get('handover_weight') == 0.,
            genuine_online=settings.get('genuine_online_closed_loop') is True,
            actual_seed_switch_observed=requests['all'].get('seed_switch_observed') is True,
            paused_replay_trace_absent=not any((trial / n).exists() for n in ('delivery_replay.jsonl', 'delivery_replay.jsonl.gz', 'diagnostic_complete.json'))))


def pair_checks(trials):
    answer = {}
    for seed in (11, 12):
        pair = sorted([t for t in trials if t['seed'] == seed], key=lambda t: t['fix'])
        if len(pair) != 2 or [t['fix'] for t in pair] != [False, True]:
            raise ValueError('Need exactly OFF/ON for seed ' + str(seed))
        a, b = pair
        settings = [{k: v for k, v in t['settings'].items() if k != 'seed_parameterization_fix'} for t in pair]
        answer[str(seed)] = dict(runtime_full_hash_equal=bool(a['runtime_sha256']) and a['runtime_sha256'] == b['runtime_sha256'],
            build_identity_equal=bool(a['build'] and a['build_manifest_sha256']) and a['build'] == b['build'] and a['build_manifest_sha256'] == b['build_manifest_sha256'],
            treatment_only_recorded_flag=bool(settings[0]) and settings[0] == settings[1],
            runtime_python_snapshots_verified=all(t['runtime_python_verified'] for t in pair),
            shared_safety_runtime_configuration_equal=bool(a.get('shared_manifest')) and a.get('shared_manifest') == b.get('shared_manifest'),
            scope_checks_passed=all(v is True for t in pair for v in t['scope_checks'].values()))
    return answer


def build_report(root):
    root = Path(root).resolve()
    plan = read_json(root / 'plan.json'); complete = read_json(root / 'complete.json')
    jobs = complete.get('completed')
    if complete.get('complete') is not True or not isinstance(jobs, list) or len(jobs) != 4:
        raise ValueError('Need all four prespecified retained jobs')
    names = [j.get('trial') for j in jobs]
    if any(not isinstance(n, str) or Path(n).name != n or n in ('.', '..') for n in names) or len(set(names)) != 4:
        raise ValueError('Unsafe or duplicate trial path')
    if {(j.get('seed'), j.get('fix')) for j in jobs} != {(11, False), (11, True), (12, False), (12, True)}:
        raise ValueError('Wrong seed/treatment set')
    assert_plot_quiet(dict(trials=[dict(run_status=read_json(root / n / 'run_status.json')) for n in names]))
    trials = sorted([trial_report(root / n) for n in names], key=lambda t: (t['seed'], t['fix']))
    for job in jobs:
        t = next(t for t in trials if t['trial'] == job['trial'])
        if t['seed'] != job['seed'] or t['fix'] is not job['fix']:
            raise ValueError('Completed job disagrees with captured treatment')
        t['completion_valid'] = job.get('valid')
        t['valid_for_descriptive_comparison'] = t['interface_valid'] is True and job.get('valid') is True
    build = Path(plan['build']); bm = build / 'build_manifest.json'
    if digest(bm) != plan.get('build_manifest_sha256'):
        raise ValueError('Build manifest changed')
    manifest = read_json(bm)
    if manifest.get('seed_parameterization_experiment') is not True or manifest.get('prepared_only') is not False:
        raise ValueError('Require compiled seed parameterization build family')
    if any(Path(t['build']).resolve() != build.resolve() or t['build_manifest_sha256'] != plan['build_manifest_sha256'] for t in trials):
        raise ValueError('Trial build differs from prespecified plan')
    sources = manifest.get('isolated_source_sha256') or {}
    binaries = manifest.get('native_binaries_sha256') or {}
    integrity = dict(isolated_source_verified=bool(sources) and all((build / 'source' / n).is_file() and digest(build / 'source' / n) == h for n, h in sources.items()),
        native_binaries_verified=bool(binaries) and all((build / 'build' / n).is_file() and digest(build / 'build' / n) == h for n, h in binaries.items()))
    checks = pair_checks(trials)
    valid = (all(t['valid_for_descriptive_comparison'] for t in trials) and
        all(v is True for p in checks.values() for v in p.values()) and all(integrity.values()))
    return dict(schema='seed_parameterization_report_v1', created=datetime.now(timezone.utc).isoformat(),
        root=str(root), plan=plan, complete=complete, trials=trials, pair_checks=checks,
        descriptive_comparison_valid=valid,
        integrity=integrity, input_sha256={n: digest(root / n) for n in ('plan.json', 'complete.json')},
        report_source_sha256=digest(Path(__file__)), limitations=LIMITATIONS,
        complete_native_input_pair=False, statistical_performance_claim=False,
        conclusion='先核查修正实际触发、前端空种子与后端进入，再描述完整参考交付；不把一次更快或最终到达当稳定交付证明。')


def report_text(value):
    lines = ['短倒车种子参数化：静态OFF/ON（描述性）', value['conclusion'], '']
    for trial in value['trials']:
        r = trial['requests']; lat = trial['end_to_end']
        lines += ['seed=%s %s: %s' % (trial['seed'], 'ON' if trial['fix'] else 'OFF', trial['trial']),
            'interface valid=%s；complete job valid=%s；可用作描述性对比=%s' % (trial['interface_valid'], trial['completion_valid'], trial['valid_for_descriptive_comparison']),
            'initial请求=%s；replan请求=%s；unknown phase=%s' % (r['initial']['requests'], r['replan']['requests'], r['unknown_phase']['requests']),
            'replan前端成功/失败=%s/%s；A*成功后采样失败=%s；进入后端=%s' % (r['replan'].get('wrapper_successes'), r['replan'].get('wrapper_failures'), r['replan'].get('astar_success_then_sampling_failures'), r['replan'].get('optimizer_entered_requests')),
            'replan种子事件/可信关联=%s/%s；修正尝试=%s；修后非空正时长种子=%s；A*成功但种子无效=%s；空输出=%s' % (r['replan'].get('seed_events'), r['replan'].get('certified_seed_events'), r['replan'].get('certified_fix_applied_events'), r['replan'].get('repaired_positive_seeds'), r['replan'].get('astar_success_with_invalid_seed'), r['replan'].get('empty_output_seeds')),
            'replan种子reason=' + json.dumps(r['replan'].get('seed_event_reasons'), ensure_ascii=False),
            'replan后端结果=%s；unknown link=%s；终验拒绝=%s；发布/精确采用请求=%s/%s' % (json.dumps(r['replan']['backend_reason_counts'], ensure_ascii=False), r['replan'].get('unknown_backend_links'), r['replan']['terminal_rejections'], r['replan']['published_requests'], r['replan']['exactly_adopted_requests']),
            'appearance→实际采用=%s秒；右删失=%s；删失下界=%s秒（所有等待保留）' % (lat['latency_s'], lat['right_censored'], lat['censor_lower_bound_s']),
            'world结果=%s；保守投影碰撞=%s' % (trial['outcome'].get('reason'), trial['outcome'].get('collision_conservative_projection')), '']
    return '\n'.join(lines + ['本批描述性对比契约有效=' + str(value['descriptive_comparison_valid']),
        '配对检查=' + json.dumps(value['pair_checks'], ensure_ascii=False),
        '构建完整性=' + json.dumps(value['integrity'], ensure_ascii=False), '', '解释边界：'] + LIMITATIONS) + '\n'


def plot_report(value, path):
    assert_plot_quiet(value)
    if value['descriptive_comparison_valid'] is not True:
        raise ValueError('Invalid interface/pair/build retained in TXT/JSON; no comparison plot')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    labels = ['%s %s' % (t['seed'], 'ON' if t['fix'] else 'OFF') for t in value['trials']]
    for offset, key, label in ((-.24, 'requests', 'Replan requests'), (0, 'optimizer_entered_requests', 'Optimizer entered'), (.24, 'exactly_adopted_requests', 'New guide used')):
        points = [(i, t['requests']['replan'][key]) for i, t in enumerate(value['trials']) if t['requests']['replan'][key] is not None]
        axes[0].bar([i + offset for i, v in points], [v for i, v in points], .23, label=label)
    for i, trial in enumerate(value['trials']):
        lat = trial['end_to_end']
        if lat['latency_s'] is not None:
            axes[1].scatter(i, lat['latency_s'], color='#4c78a8')
        else:
            axes[1].text(i, .08, 'censored' if lat['right_censored'] else 'unknown', rotation=90, ha='center', transform=axes[1].get_xaxis_transform())
    axes[0].set_ylabel('Recorded replan count (initial excluded)'); axes[0].legend(fontsize=7)
    axes[1].set_ylabel('Appearance → new guide entry (s), all waits')
    for ax in axes:
        ax.set_xticks(range(4)); ax.set_xticklabels(labels); ax.grid(axis='y', alpha=.2)
    fig.suptitle('Static seed-parameterization A/B — descriptive, not matched native inputs', fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, .94)); fig.savefig(path, dpi=140); plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True); parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--plot', action='store_true'); args = parser.parse_args(argv)
    root = args.root.resolve(); out = args.out.resolve()
    if out.exists() or out == root or out in root.parents:
        parser.error('Output must be fresh and not an input/ancestor')
    value = build_report(root); out.mkdir(parents=True, exist_ok=False)
    (out / 'summary.json').write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    (out / '实验结论.txt').write_text(report_text(value), encoding='utf-8')
    if args.plot:
        plot_report(value, out / 'seed_delivery.png')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
