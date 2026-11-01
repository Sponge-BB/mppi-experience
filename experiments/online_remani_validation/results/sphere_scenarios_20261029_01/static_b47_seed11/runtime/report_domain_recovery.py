#!/usr/bin/env python3
"""Post-run, evidence-only report of the two single-seed domain SIL trials.

No ROS, model invocation, geometry replay, or plotting is done on import.  The
optional plot requires a completed batch and inactive recorded ROS processes.
"""
import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import gzip
import hashlib
import json
import os
from pathlib import Path
import socket


# Verified against FORCE_STOP_OPTIMIZE_TYPE in the native optimizer header.
FORCE_STOP = {0: 'DONT_STOP', 1: 'STOP_FOR_REBOUND', 2: 'STOP_FOR_ERROR', 3: 'STOP_FOR_TIMEOUT'}
LIMITATIONS = [
    '仅静态场景、单一 seed 的两次诊断 SIL；不能据此建立统计结论。',
    '候选/失败捕获增加 I/O；不是严格速度基准，也不证明动态安全改善。',
    'OFF/ON 均包含“终止负求解器状态在使用可变回调缓存前 fail-closed”的共同修复；旧诊断批次不是精确 OFF 对照。',
    '拒绝 trial、拒绝后的 Wolfe 接受步、实际成功求解、最终硬门接受是不同层级；非负 L-BFGS 状态不等于成功，TIMEOUT 优先。',
    'domain_terminal_reject 不涵盖提前返回的 TIMEOUT/ERROR，不能代替负求解器状态总数。',
    '缺失数据记为 null/未知，绝不补零；状态关联不使用最近时间匹配。',
]


def read_json(path):
    with Path(path).open(encoding='utf-8') as stream:
        return json.load(stream)


def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def read_records(directory, name, required=False):
    """One logical JSONL input: prefer plain over its archived duplicate."""
    paths = [Path(directory) / name, Path(directory) / (name + '.gz')]
    path = next((p for p in paths if p.is_file()), None)
    if path is None:
        if required:
            raise ValueError('Missing required timeline: ' + str(paths[0]))
        return None, None
    opener = gzip.open if path.suffix == '.gz' else open
    rows = []
    with opener(path, 'rt', encoding='utf-8') as stream:
        for number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            row = json.loads(line)
            if not isinstance(row, dict):
                raise ValueError('%s:%d: record must be an object' % (path, number))
            rows.append(row)
    return rows, path


def integer(value):
    return isinstance(value, int) and not isinstance(value, bool)


def counter_sum(rows, key):
    values = [r.get(key) for r in rows]
    return sum(values) if all(integer(v) and v >= 0 for v in values) else None


def histogram(rows, key):
    return dict(sorted(Counter(str(r[key]) for r in rows if key in r and r[key] is not None).items()))


def resource_scopes(rows):
    """Validate exact same-thread scope IDs and strict nesting, not proximity.

    IDs are thread-local, so a (pid,id) collision across tids remains visible
    in all_begins and cannot be hidden by a dictionary overwrite.
    """
    stacks = defaultdict(list)
    seen, bad_threads, completed, all_begins, last_mono = set(), set(), [], [], {}
    for row in rows or []:
        thread = (row.get('pid'), row.get('tid'))
        key = thread + (row.get('id'), row.get('stage'))
        mono = row.get('mono')
        if not isinstance(mono, (int, float)) or (thread in last_mono and mono < last_mono[thread]):
            bad_threads.add(thread)
        else:
            last_mono[thread] = mono
        if row.get('event') == 'begin':
            all_begins.append(row)
            if key in seen or any(v is None for v in key):
                bad_threads.add(thread)
            seen.add(key)
            parent = stacks[thread][-1]['key'] if stacks[thread] else None
            stacks[thread].append(dict(key=key, parent=parent, begin=row))
        elif row.get('event') == 'end':
            if not stacks[thread] or stacks[thread][-1]['key'] != key:
                bad_threads.add(thread)
                continue
            scope = stacks[thread].pop()
            scope['end'] = row
            begin = scope['begin']
            if (begin.get('label') != row.get('label') or
                    not isinstance(begin.get('mono'), (int, float)) or
                    not isinstance(row.get('mono'), (int, float)) or row['mono'] < begin['mono']):
                bad_threads.add(thread)
            completed.append(scope)
    bad_threads.update(thread for thread, stack in stacks.items() if stack)
    return [s for s in completed if s['key'][:2] not in bad_threads], all_begins, sorted(map(str, bad_threads))


def certify_status(row, native, scopes, begins):
    """Prove request -> exact replan ID -> unique nested optimizer identity.

    Extra input-hash equality checks the coupling solve identity. Any duplicate,
    incomplete nesting, missing field, or ambiguous thread fails closed.
    """
    result = dict(row, certification='status-not-certified')
    pid, request = row.get('pid'), row.get('request_id')

    def fail(reason):
        result['certification_reason'] = reason
        return result

    if pid is None or request is None:
        return fail('missing_pid_or_request_id')
    same = [r for r in native if r.get('pid') == pid and r.get('request_id') == request]
    request_begin = [r for r in same if r.get('event') == 'request_begin']
    request_end = [r for r in same if r.get('event') == 'request_end']
    if len(request_begin) != 1 or len(request_end) != 1:
        return fail('missing_or_duplicate_request_identity')
    if sum(r.get('event') == 'domain_solver_status' for r in same) != 1:
        return fail('ambiguous_status_per_request')
    links = [r for r in same if r.get('event') == 'resource_link']
    coupling = [r for r in same if r.get('event') == 'coupling_scope']
    if len(links) != 1 or len(coupling) != 1:
        return fail('missing_or_duplicate_resource_link_or_coupling_scope')
    replan_id = links[0].get('resource_replan_id')
    matching_begins = [r for r in begins if r.get('stage') == 'replan' and
                       r.get('pid') == pid and r.get('id') == replan_id]
    replans = [s for s in scopes if s['begin'].get('stage') == 'replan' and
               s['begin'].get('pid') == pid and s['begin'].get('id') == replan_id]
    if replan_id is None or len(matching_begins) != 1 or len(replans) != 1:
        return fail('ambiguous_or_incomplete_exact_replan_id')
    optimizers = [s for s in scopes if s['parent'] == replans[0]['key'] and
                  s['begin'].get('stage') == 'optimizer' and s['begin'].get('label') == 'normal']
    # Count begins too: an unfinished second optimizer is not silently ignored.
    replan = replans[0]
    all_children = [r for r in begins if r.get('stage') == 'optimizer' and r.get('label') == 'normal' and
                    (r.get('pid'), r.get('tid')) == replan['key'][:2] and
                    replan['begin']['mono'] <= r.get('mono', float('-inf')) <= replan['end']['mono']]
    if len(optimizers) != 1 or len(all_children) != 1:
        return fail('ambiguous_or_incomplete_optimizer_child')
    scope, identity = optimizers[0], coupling[0]
    end = scope['end']
    if (identity.get('input_hash') is None or identity.get('input_hash') != end.get('input_hash') or
            not integer(row.get('lbfgs_result')) or row['lbfgs_result'] != end.get('lbfgs_result')):
        return fail('missing_or_disagreeing_input_hash_or_lbfgs_result')
    if not integer(end.get('force_stop')) or not isinstance(end.get('success'), bool):
        return fail('missing_force_stop_or_actual_success')
    timestamps = [request_begin[0].get('mono'), request_end[0].get('mono'), row.get('mono'), identity.get('mono')]
    if (not all(isinstance(t, (int, float)) for t in timestamps) or
            not (timestamps[0] <= replan['begin']['mono'] <= scope['begin']['mono'] <= timestamps[3] <=
                 timestamps[2] <= scope['end']['mono'] <= replan['end']['mono'] <= timestamps[1])):
        return fail('identity_chain_clock_or_containment_disagreement')
    result.update(certification='certified',
                  certification_reason='explicit_resource_link_id+balanced_same_thread_nesting+unique_solve+exact_input_hash',
                  resource_replan_id=replan_id, resource_optimizer_id=end['id'], tid=end['tid'],
                  coupling_scope_id=identity.get('scope_id'), force_stop=end['force_stop'],
                  force_stop_name=FORCE_STOP.get(end['force_stop'], 'UNKNOWN'),
                  recorded_optimizer_success=end['success'],
                  actual_successful_solve=bool(end['success'] and end['force_stop'] == 0 and row['lbfgs_result'] >= 0))
    return result


def candidate_records(trial):
    directory = trial / 'delivery_diagnostics/candidates'
    if not directory.is_dir():
        return None
    result = []
    for path in sorted(directory.glob('*.json')):
        candidate = read_json(path)
        decision = candidate.get('accepted')
        online = candidate.get('online_accept')
        if 'accepted' not in candidate:
            decision = online.get('accepted') if isinstance(online, dict) else online
        elif isinstance(online, bool) and online != decision:
            decision = None
        finished = candidate.get('finished')
        state = ('accepted' if decision is True else 'rejected') if finished is True and isinstance(decision, bool) else (
            'unfinished' if finished is False else 'unknown')
        if candidate.get('schema') != 'delivery_candidate_v1':
            state = 'unknown'
        clock = candidate.get('clock') or {}
        result.append(dict(file=str(path.relative_to(trial)), sha256=digest(path), schema=candidate.get('schema'),
                           pid=clock.get('pid'), request_id=candidate.get('request_id'), scope_id=candidate.get('scope_id'),
                           finished=finished, recorded_accepted=decision, online_accept=online,
                           failure=candidate.get('failure'), hard_gate_state=state))
    return result


def summarize_trial(trial):
    names = ('manifest.json', 'run_status.json', 'trial_audit.json', 'result.json')
    documents = {name: read_json(trial / name) for name in names}
    manifest, status, audit, outcome = [documents[n] for n in names]
    if status.get('returncode') is None or status.get('result_written') is not True:
        raise ValueError('Trial is not finished: ' + str(trial))
    settings = manifest.get('coupling_experiment') or {}
    if not isinstance(settings.get('domain_recovery'), bool):
        raise ValueError('Missing explicit OFF/ON treatment in ' + str(trial))
    native, timeline_path = read_records(trial, 'dual_timeline.jsonl', required=True)
    if not native:
        raise ValueError('Empty required timeline: ' + str(trial))
    resource, resource_path = read_records(trial, 'solver_trace.jsonl')
    failures, failure_path = read_records(trial, 'failure_trace.jsonl')
    scopes, begins, bad_threads = resource_scopes(resource)
    statuses = [certify_status(r, native, scopes, begins) for r in native if r.get('event') == 'domain_solver_status']
    candidates = candidate_records(trial)
    for row in statuses:
        matches = None
        if row['certification'] == 'certified' and row.get('coupling_scope_id') is not None and candidates is not None:
            matches = [c for c in candidates if (c['pid'], c['request_id'], c['scope_id']) ==
                       (row['pid'], row['request_id'], row['coupling_scope_id'])]
        row['matched_hard_candidate_count'] = len(matches) if matches is not None else None
        row['matched_hard_accept_count'] = sum(c['hard_gate_state'] == 'accepted' for c in matches) if matches is not None else None
    events = Counter(r.get('event') for r in native)
    optimizer_ends = [r for r in resource or [] if r.get('event') == 'end' and
                      r.get('stage') == 'optimizer' and r.get('label') == 'normal']
    optimizer_begins = [r for r in resource or [] if r.get('event') == 'begin' and
                        r.get('stage') == 'optimizer' and r.get('label') == 'normal']
    trace_complete = (resource is not None and not bad_threads and len(optimizer_begins) == len(optimizer_ends)
                      and len(statuses) <= len(optimizer_ends))
    known_solves = [r for r in optimizer_ends if isinstance(r.get('success'), bool) and
                   integer(r.get('force_stop')) and integer(r.get('lbfgs_result'))]
    successful = [r for r in known_solves if r['success'] and r['force_stop'] == 0 and r['lbfgs_result'] >= 0]
    recovered = [r for r in statuses if r.get('actual_successful_solve') and
                 integer(r.get('rejected_trials')) and r['rejected_trials'] > 0]
    counts = dict(domain_solver_status=len(statuses), rejected_trials=counter_sum(statuses, 'rejected_trials'),
                  accepted_steps_after_rejection=counter_sum(statuses, 'accepted_steps_after_rejection'),
                  statuses_with_rejected_trial=sum(integer(r.get('rejected_trials')) and r['rejected_trials'] > 0 for r in statuses),
                  enabled=histogram(statuses, 'enabled'), lbfgs_result=histogram(statuses, 'lbfgs_result'),
                  missing_status_fields={k: sum(k not in r or r[k] is None for r in statuses)
                                         for k in ('enabled', 'rejected_trials', 'accepted_steps_after_rejection', 'lbfgs_result')},
                  domain_initial_reject=events['domain_initial_reject'], domain_terminal_reject=events['domain_terminal_reject'],
                  coupling_objective_error=events['coupling_objective_error'],
                  certified_statuses=sum(r['certification'] == 'certified' for r in statuses),
                  status_not_certified=sum(r['certification'] != 'certified' for r in statuses),
                  certified_successful_solves_after_rejection=len(recovered),
                  certified_recovered_hard_accepts=sum(r['matched_hard_accept_count'] for r in recovered)
                    if all(r.get('matched_hard_accept_count') is not None for r in recovered) else None)
    trace = dict(present=resource is not None, optimizer_end_count=len(optimizer_ends) if resource is not None else None,
                 optimizer_begin_count=len(optimizer_begins) if resource is not None else None,
                 scopes_complete=trace_complete,
                 successful_optimizer_solves=len(successful) if trace_complete and len(known_solves) == len(optimizer_ends) else None,
                 known_successful_optimizer_solves=len(successful) if resource is not None else None,
                 missing_success_or_force_stop_or_status=len(optimizer_ends) - len(known_solves) if resource is not None else None,
                 force_stop=histogram(optimizer_ends, 'force_stop') if resource is not None else None,
                 nonnegative_but_timeout=sum(integer(r.get('lbfgs_result')) and r['lbfgs_result'] >= 0 and r.get('force_stop') == 3
                                            for r in optimizer_ends) if resource is not None else None,
                 invalid_scope_threads=bad_threads,
                 rows=[{k: r.get(k) for k in ('pid', 'tid', 'id', 'success', 'lbfgs_result', 'force_stop')} for r in optimizer_ends])
    gate = dict(present=candidates is not None, observed_count=len(candidates) if candidates is not None else None,
                **{key: sum(c['hard_gate_state'] == key for c in candidates) if candidates is not None else None
                   for key in ('accepted', 'rejected', 'unfinished', 'unknown')}, records=candidates)
    hashes = {name: digest(trial / name) for name in names}
    for path in (timeline_path, resource_path, failure_path):
        if path is not None:
            hashes[path.name] = digest(path)
    return dict(trial=trial.name, source=str(trial), domain_recovery=settings['domain_recovery'],
                scene=manifest.get('scene'), seed=settings.get('seed'), build=manifest.get('build'),
                duration_s=manifest.get('duration'), coupling_experiment=settings,
                build_manifest_sha256=manifest.get('build_manifest_sha256'),
                common_terminal_negative_fail_closed=settings.get('terminal_solver_failure_fail_closed'),
                run_status=status, audit=audit, outcome=outcome, input_sha256=hashes,
                domain=counts, solver_status_records=statuses, optimizer_trace=trace, hard_candidates=gate,
                failure_reason_counts=histogram([r for r in failures or [] if r.get('kind') == 'solve' and r.get('label') == 'normal'], 'reason')
                                      if failures is not None else None,
                failure_events=[r for r in native if r.get('event') in
                                ('domain_initial_reject', 'domain_terminal_reject', 'coupling_objective_error')])


def build_report(root):
    root = Path(root).resolve()
    complete, plan = read_json(root / 'complete.json'), read_json(root / 'plan.json')
    jobs = complete.get('completed')
    if complete.get('complete') is not True or not isinstance(jobs, list) or len(jobs) != 2:
        raise ValueError('Exactly two completed trials are required; no report during a live/incomplete batch')
    trials = []
    for job in jobs:
        name = job.get('trial')
        if not isinstance(name, str) or Path(name).name != name or name in ('.', '..'):
            raise ValueError('Invalid trial name in complete.json')
        trials.append(summarize_trial(root / name))
    if {t['domain_recovery'] for t in trials} != {False, True}:
        raise ValueError('A distinct OFF and ON trial are required')
    trials.sort(key=lambda t: t['domain_recovery'])
    on = trials[1]['domain']
    exercised = True if on['rejected_trials'] is not None and on['rejected_trials'] > 0 else (
        False if on['rejected_trials'] == 0 and on['domain_initial_reject'] == 0 else None)
    if exercised is False:
        conclusion = 'ON 闭环试验未激发域拒绝/回退，不能据此证明成功域恢复；单元测试仅验证缩步机制。'
    elif on['certified_successful_solves_after_rejection'] > 0:
        conclusion = '观测到经身份认证的拒绝后成功求解；最终硬门接受另计，仍不能推广为速度或动态安全改善。'
    else:
        conclusion = '尚无经身份认证的拒绝后成功求解证据；计数器增长或非负返回值本身不是域恢复成功。'
    common = all(t['common_terminal_negative_fail_closed'] is True for t in trials)
    pair_checks = {}
    for field in ('scene', 'seed', 'duration_s', 'build', 'build_manifest_sha256'):
        values = [t[field] for t in trials]
        pair_checks[field] = None if any(v is None for v in values) else values[0] == values[1]
    off_settings, on_settings = [dict(t['coupling_experiment']) for t in trials]
    off_settings.pop('domain_recovery', None); on_settings.pop('domain_recovery', None)
    pair_checks['other_recorded_coupling_settings_equal'] = off_settings == on_settings
    return dict(schema='domain_recovery_report_v1', created=datetime.now(timezone.utc).isoformat(), root=str(root),
                diagnostic_only=True, performance_comparison=False, plan=plan,
                complete=complete, root_input_sha256={n: digest(root / n) for n in ('plan.json', 'complete.json')},
                pair_consistency=pair_checks,
                common_pair_fail_closed_recorded=common, domain_recovery_exercised=exercised,
                conclusion=conclusion, limitations=LIMITATIONS, trials=trials)


def show(value):
    return '未知' if value is None else str(value)


def report_text(report):
    lines = ['域内回退：单 seed 静态诊断（非性能/动态安全结论）', report['conclusion'],
             '共同负状态 fail-closed 修复已记录：' + show(report['common_pair_fail_closed_recorded'])]
    for trial in report['trials']:
        domain, trace, gate, outcome = [trial[k] for k in ('domain', 'optimizer_trace', 'hard_candidates', 'outcome')]
        lines += ['', '%s：%s' % ('ON' if trial['domain_recovery'] else 'OFF', trial['trial']),
                  '结果=%s；保守碰撞=%s；审计有效=%s' % (show(outcome.get('reason')),
                    show(outcome.get('collision_conservative_projection')), show(trial['audit'].get('valid_experiment'))),
                  '状态记录=%d；拒绝 trial=%s；拒绝后接受步=%s；L-BFGS=%s' %
                    (domain['domain_solver_status'], show(domain['rejected_trials']), show(domain['accepted_steps_after_rejection']), json.dumps(domain['lbfgs_result'])),
                  '初始拒绝=%d；终止拒绝=%d；目标错误=%d；未认证状态=%d' %
                    (domain['domain_initial_reject'], domain['domain_terminal_reject'], domain['coupling_objective_error'], domain['status_not_certified']),
                  '实际成功求解=%s/%s；force_stop=%s；非负但 TIMEOUT=%s' %
                    (show(trace['successful_optimizer_solves']), show(trace['optimizer_end_count']), json.dumps(trace['force_stop']), show(trace['nonnegative_but_timeout'])),
                  '最终硬门：接受=%s，拒绝=%s，未完成=%s，未知=%s；认证拒绝后成功求解=%d，随后硬门接受=%s' %
                    (show(gate['accepted']), show(gate['rejected']), show(gate['unfinished']), show(gate['unknown']),
                     domain['certified_successful_solves_after_rejection'], show(domain['certified_recovered_hard_accepts']))]
    lines += ['', '解释边界：'] + LIMITATIONS
    return '\n'.join(lines) + '\n'


def assert_plot_quiet(report):
    """Fail closed on even possible PID reuse; never signal/stop any process."""
    ports = set()
    for trial in report['trials']:
        pids = trial['run_status'].get('monitored_node_pids')
        if not isinstance(pids, dict) or not pids:
            raise ValueError('Recorded process IDs are required before plotting')
        for pid in pids.values():
            if not integer(pid) or pid <= 1:
                raise ValueError('Invalid recorded process ID')
            try:
                os.kill(pid, 0)
            except ProcessLookupError:
                continue
            raw = (Path('/proc') / str(pid) / 'stat').read_text()
            if raw[raw.rfind(')') + 2:].split()[0] != 'Z':
                raise RuntimeError('Recorded ROS process may still be live; no plot: ' + str(pid))
        ports.add(11441)
    for port in ports:
        with socket.socket() as sock:
            sock.settimeout(.5)
            if sock.connect_ex(('127.0.0.1', port)) == 0:
                raise RuntimeError('Isolated ROS master is active; no plot')


def plot_report(report, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7.8, 4.2))
    labels = ['Rejected trials', 'Successful solves', 'Hard accepted candidates']
    for offset, trial in ((-.18, report['trials'][0]), (.18, report['trials'][1])):
        values = [trial['domain']['rejected_trials'], trial['optimizer_trace']['successful_optimizer_solves'], trial['hard_candidates']['accepted']]
        for index, value in enumerate(values):
            if value is None:
                ax.text(index + offset, .1, 'unknown', rotation=90, ha='center')
        bars = ax.bar([i + offset for i in range(3)], [v if v is not None else 0 for v in values], .34,
                      label='ON' if trial['domain_recovery'] else 'OFF')
        for bar, value in zip(bars, values):
            if value is not None:
                ax.text(bar.get_x() + bar.get_width()/2, value + .04, str(value), ha='center', fontsize=9)
    ax.set_xticks(range(3)); ax.set_xticklabels(labels); ax.set_ylabel('Recorded count'); ax.legend()
    ax.set_title('Single-seed static diagnostic: not speed or safety evidence', fontsize=11)
    ax.set_ylim(0, max(ax.get_ylim()[1] * 1.18, 1.))
    fig.tight_layout(); fig.savefig(path, dpi=130); plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True, help='Separate fresh output directory; no source files are modified')
    parser.add_argument('--plot', action='store_true', help='One small PNG, only after the two trials have stopped')
    args = parser.parse_args(argv)
    root, out = args.root.resolve(), args.out.resolve()
    if out.exists() or out == root or out in root.parents:
        parser.error('--out must be a separate fresh directory, never an input directory or its ancestor')
    report = build_report(root)
    if args.plot:
        assert_plot_quiet(report)
    out.mkdir(parents=True, exist_ok=False)
    (out / 'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    (out / '实验结论.txt').write_text(report_text(report), encoding='utf-8')
    if args.plot:
        plot_report(report, out / 'diagnostic_counts.png')
    print(json.dumps(dict(out=str(out), conclusion=report['conclusion']), ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
