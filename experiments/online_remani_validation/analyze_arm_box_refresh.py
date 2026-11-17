#!/usr/bin/env python3
"""Offline mechanism and closed-loop comparison of arm-box refresh trials."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np

from report_arm_box_navigation import load_trial, read_json, safe, sha


def jsonl(path):
    if not path.is_file():
        return []
    with path.open() as stream:
        return [json.loads(line) for line in stream if line.strip()]


def truth(value):
    return value is True or value in (1, 'true', '1')


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def scan(case):
    """Support case/mppi and case/legacy_seed11/mppi without unrelated data."""
    found = []
    for path in sorted(case.iterdir()):
        if not path.is_dir():
            continue
        candidates = [path] if (path/'result.json').is_file() else sorted(path.iterdir())
        found += [item for item in candidates if item.is_dir()
                  and (item/'result.json').is_file() and (item/'state.csv').is_file()
                  and (item/'active_arm_config.json').is_file()]
    return found


def ages(event):
    """Use recorded source ages; absent source timestamps remain unknown."""
    names = ('seed', 'cloud', 'dynamic', 'odom', 'joint')
    result = {}
    now = event.get('request_time')
    for name in names:
        value = next((event[key] for key in (name+'_age_s', name+'_age') if key in event), None)
        stamp_keys = ('seed_stamp',) if name == 'seed' else ('seed_'+name+'_stamp', name+'_stamp')
        stamp = next((event[key] for key in stamp_keys if key in event), None)
        if value is None and now is not None and stamp is not None and stamp > 0:
            value = now-stamp
        result[name+'_age_s'] = value
    known = [value for value in result.values() if value is not None]
    result['all_sources_observed'] = len(known) == len(names)
    result['known_sources_within_original_age_bounds'] = all(
        value is None or -.05 <= value <= (.35 if name in ('cloud', 'dynamic') else .25)
        for name in names for value in [result[name+'_age_s']]) if known else None
    return result


def zero_duration(state, begin, end):
    t = state['t']
    dt = np.maximum(0., np.minimum(t[1:], end)-np.maximum(t[:-1], begin))
    stationary = (np.abs(state['v'][:-1]) < .01) & (np.abs(state['w'][:-1]) < .03)
    return float(np.sum(dt[stationary]))


def solve_summary(rows):
    times = [row['wall_ms'] for row in rows if isinstance(row.get('wall_ms'), (int, float))]
    failed = [row for row in rows if row.get('success') is False]
    return dict(solve_count=len(rows), successful_count=sum(row.get('success') is True for row in rows),
        failed_count=len(failed), reason_counts=dict(Counter(row.get('reason', 'unknown') for row in rows)),
        failed_reason_counts=dict(Counter(row.get('reason', 'unknown') for row in failed)),
        failed_without_named_rejection_count=sum(row.get('reason') in (None, 'ok') for row in failed),
        total_wall_ms=float(sum(times)), median_wall_ms=float(np.median(times)) if times else None,
        maximum_wall_ms=max(times) if times else None)


def scope_solves(scope, solves):
    key = scope.get('input_hash')
    candidates = [row for row in solves if key is not None and str(row.get('input_hash')) == str(key)]
    if len(candidates) > 1:
        # Same hash may recur. A source-time match disambiguates repeated work;
        # otherwise retain the ambiguity rather than guess which solve ran.
        exact = [row for row in candidates if abs(row.get('sim', -1e9)-scope.get('sim', 1e9)) < 1e-6]
        candidates = exact if len(exact) == 1 else candidates
    state = ('scope_input_hash_missing' if key is None else 'solve_input_unknown' if not candidates
             else 'ambiguous_repeated_input_hash' if len(candidates) > 1 else 'matched')
    fields = ('input_hash', 'label', 'sim', 'sim_end', 'reason', 'success', 'wall_ms',
              'rejection_value', 'rejection_time', 'iterations', 'lbfgs_result', 'force_stop',
              'budget_s', 'raw_seq', 'esdf_seq', 'candidate_hash', 'candidate_duration_s')
    return dict(scope_id=scope.get('scope_id'), input_hash=None if key is None else str(key),
        match_state=state, solve_records=[{field: row[field] for field in fields if field in row}
                                         for row in candidates],
        rejection_time_semantics='Native rejection sample time along candidate trajectory, not elapsed solve wall time')


def refresh_context(timeline, begin, initial):
    if initial:
        return dict(kind='initial', trigger=None, retry_since_trigger=False), None
    position = next((i for i, row in enumerate(timeline) if row is begin), len(timeline))
    before = timeline[:position]
    trigger_index = next((i for i in range(len(before)-1, -1, -1)
                          if before[i]['event'] == 'refresh_trigger'), None)
    if trigger_index is None:
        return dict(kind='unknown', trigger=None, retry_since_trigger=None), None
    trigger = before[trigger_index]
    retry = any(row['event'] == 'request_begin' for row in before[trigger_index+1:])
    required = truth(trigger.get('blocked')) or truth(trigger.get('continue_partial'))
    optional = truth(trigger.get('fast_feedback_trigger')) and not required
    kind = ('required' if required else 'optional_origin_retry_need_unrecorded' if optional and retry
            else 'optional' if optional else 'unknown')
    inputs = [row for row in before if row['event'] == 'arm_box_refresh_input'
              and truth(row.get('valid')) and row.get('seed_hash') is not None
              and str(row['seed_hash']) == str(trigger.get('seed_hash'))
              and row.get('seed_epoch') == trigger.get('seed_epoch')]
    return dict(kind=kind, trigger={key: trigger[key] for key in
        ('sim', 'source', 'blocked', 'continue_partial', 'fast_feedback_trigger', 'episode',
         'seed_stamp', 'seed_epoch', 'seed_hash') if key in trigger}, retry_since_trigger=retry), inputs[-1] if inputs else None


def seed_identity(context, admission, prefixes, scopes):
    if context['kind'] == 'initial':
        return dict(strict_optional_freeze_required=False, state='initial_request_excluded')
    required = context['kind'] == 'optional'
    records = ([dict(source='admission', **admission)] if admission else [])
    records += [dict(source='prefix', **row) for row in prefixes if truth(row.get('used'))]
    records += [dict(source='coupling', **row) for row in scopes]
    fields = [{key: row.get(key) for key in ('source', 'scope_id', 'seed_hash', 'seed_epoch')} for row in records]
    complete = ({row['source'] for row in fields} >= {'admission', 'prefix', 'coupling'}
                and all(row['seed_hash'] is not None and row['seed_epoch'] is not None for row in fields))
    hashes = {str(row['seed_hash']) for row in fields if row['seed_hash'] is not None}
    epochs = [row['seed_epoch'] for row in fields if row['seed_epoch'] is not None]
    same = len(hashes) == 1 and bool(epochs) and max(epochs)-min(epochs) <= 1e-9
    return dict(strict_optional_freeze_required=required,
        state=('unknown_missing_identity_evidence' if not complete else 'consistent' if same else 'mismatch')
            if required else 'not_required_for_required_or_unclassified_path',
        observed_hashes_match=len(hashes) == 1 if hashes else None,
        observed_epochs_match=max(epochs)-min(epochs) <= 1e-9 if epochs else None,
        evidence=fields)


def early_delivery(rows):
    checks = [row for row in rows if row['event'] == 'early_delivery_check']
    summaries = [row for row in rows if row['event'] == 'early_delivery_summary']
    return dict(domain_solver_status=[{key: value for key, value in row.items()
        if key not in ('mono', 'pid', 'request_id')} for row in rows if row['event'] == 'domain_solver_status'],
        summaries=[{key: value for key, value in row.items() if key not in ('mono', 'pid', 'request_id')}
                   for row in summaries],
        check_reason_counts=dict(Counter(row.get('reason', 'unknown') for row in checks)),
        native_reason_counts=dict(Counter(row.get('native_reason', 'unknown') for row in checks)),
        final_check_reason=checks[-1].get('reason') if checks else None,
        no_deliverable_candidate_within_budget=any(not truth(row.get('selected'))
            and row.get('elapsed_ms', 0) >= 1000*row.get('budget_s', float('inf')) for row in summaries),
        native_gate_internal_reason_unknown=any(row.get('reason') == 'native_gate_reject'
            and row.get('native_reason') in (None, 'not_validated', 'unknown') for row in checks))


def refresh_kind_summary(requests):
    result = {}
    for kind in sorted({row['refresh_context']['kind'] for row in requests}):
        rows = [row for row in requests if row['refresh_context']['kind'] == kind]
        solves = [record for row in rows for scope in row['optimizer_scope_solves']
                  if scope['match_state'] == 'matched' for record in scope['solve_records']]
        result[kind] = dict(request_count=len(rows), prefix_used_count=sum(row['prefix_used'] for row in rows),
            arm_target_callback_count=sum(row['arm_target_callbacks_available'] for row in rows),
            manager_success_count=sum(any(item.get('success') is True for item in row['manager_results']) for row in rows),
            manager_failed_count=sum(any(item.get('success') is False for item in row['manager_results']) for row in rows),
            reference_published_count=sum(row['reference_published'] for row in rows),
            coupled_delivery_count=sum(row['complete_coupled_delivery_evidence'] for row in rows),
            no_candidate_within_budget_count=sum(row['early_delivery']['no_deliverable_candidate_within_budget'] for row in rows),
            solver_summary=solve_summary(solves))
    return result


def mechanisms(timeline, replacements, solves=()):
    ids = sorted({row.get('request_id', 0) for row in timeline if row.get('request_id', 0) > 0})
    requests = []
    keep = ('event', 'sim', 'request_id', 'scope_id', 'input_hash', 'seed_hash', 'seed_epoch', 'reason', 'used', 'valid', 'active',
            'active_arm_coupling', 'callback_count', 'active_callback_count',
            'execution_targets', 'delivery_targets', 'delivery_cost', 'total_coupling_cost',
            'seed_stamp', 'seed_age_s', 'reference_generation', 'reference_revision',
            'prediction_valid', 'prediction_reason', 'request_time')
    for request_id in ids:
        rows = [r for r in timeline if r.get('request_id') == request_id]
        begin = next((r for r in rows if r['event'] == 'request_begin'), {})
        prefix = [r for r in rows if r['event'] == 'collab_prefix']
        scopes = [r for r in rows if r['event'] == 'coupling_scope']
        costs = [r for r in rows if r['event'] == 'coupling_cost']
        publications = [r for r in rows if r['event'] == 'reference_publish']
        active_scopes = {r.get('scope_id') for r in scopes
                         if truth(r.get('valid')) and truth(r.get('active_arm_coupling'))}
        target_costs = [r for r in costs if r.get('scope_id') in active_scopes
                        and r.get('callback_count', 0) > 0 and r.get('delivery_targets', 0) > 0]
        received = [r for r in replacements if any(
            -.05 <= r.get('t', -1e9)-(pub['sim']-1000.) <= .5 for pub in publications)]
        prefix_used = any(truth(r.get('used')) for r in prefix)
        callbacks = any(r.get('active_callback_count', 0) > 0 for r in target_costs)
        linked = [scope_solves(scope, solves) for scope in scopes]
        context, admission = refresh_context(timeline, begin, request_id == ids[0])
        requests.append(dict(request_id=request_id, request_time_s=begin.get('sim', 1000.)-1000.,
            initial_request=request_id == ids[0], prefix_used=prefix_used,
            refresh_context=context, frozen_seed_identity=seed_identity(context, admission, prefix, scopes),
            early_delivery=early_delivery(rows),
            manager_results=[{key: row[key] for key in ('sim', 'success') if key in row}
                             for row in rows if row['event'] == 'manager_result'],
            optimizer_scope_solves=linked,
            solver_input_evidence_state=('no_optimizer_scope_recorded_unknown' if not scopes
                else 'matched' if all(item['match_state'] == 'matched' for item in linked)
                else 'scope_solve_input_unknown_or_ambiguous'),
            arm_coupling_valid=bool(active_scopes), arm_target_callbacks_available=bool(target_costs),
            arm_nonzero_cost_callbacks_observed=callbacks,
            reference_published=bool(publications), reference_replacement_received=bool(received),
            complete_coupled_delivery_evidence=bool(prefix_used and target_costs
                                                   and publications and received),
            evidence=[{key: row[key] for key in keep if key in row} for row in rows
                      if row['event'] in ('collab_prefix', 'coupling_scope', 'coupling_cost', 'reference_publish')],
            seed_inputs=[dict(event=row['event'], sim=row.get('sim'), **ages(row)) for row in rows
                         if row['event'] in ('handoff_feedback_input', 'arm_box_refresh_input', 'coupling_scope')]))
    return requests


def analyze(path, case):
    trial = load_trial(path)
    state, result, runtime = trial['state'], trial['result'], trial['runtime']
    timeline = jsonl(path/'dual_timeline.jsonl')
    cycles = jsonl(path/'active_arm_cycles.jsonl')
    solves = [row for row in jsonl(path/'failure_trace.jsonl') if row.get('kind') == 'solve']
    reference_events = jsonl(path/'active_arm_reference_events.jsonl')
    replacements = [r for r in reference_events if r.get('event') == 'replace']
    trajectories = read_json(path/'trajectories.json', [])
    refs, seen = [], set()
    for row in trajectories:
        if row.get('action') != 1 or row.get('id') != 1:
            continue
        key = row.get('canonical_message_sha256', row.get('message_sha256'))
        if key is None:
            key = digest({k: row.get(k) for k in ('stamp', 'singul', 'polynomials')})
        if key not in seen:
            refs.append(row)
            seen.add(key)
    first = refs[0] if refs else None
    appearance = runtime.get('appearance_time')
    active = np.zeros(len(state['t']), dtype=bool) if appearance is None else state['t'] >= appearance
    rear = np.min(trial['body'][:, 47:, 0]-trial['body'][:, 47:, 3], axis=1)
    passed = active & (rear > trial['center'][0]+.5*trial['size'][0])
    crossing = float(state['t'][np.flatnonzero(passed)[0]]) if passed.any() else None
    end = float(state['t'][-1])
    encounter_end = crossing if crossing is not None else end
    requests = mechanisms(timeline, replacements, solves)
    after = [r for r in requests if not r['initial_request']]
    gate = [r for r in timeline if r['event'] in ('arm_box_refresh_pending', 'arm_box_refresh_input',
                                                'optional_refresh_defer', 'optional_refresh_cancel')]
    input_events = [dict(event=r['event'], sim=r.get('sim'), request_id=r.get('request_id'),
        kind=r.get('kind'), valid=r.get('valid'), reason=r.get('reason'), **ages(r),
        recorded_fields={key: value for key, value in r.items() if not isinstance(value, (dict, list))}) for r in gate]
    status = read_json(path/'run_status.json', {})
    run_valid = bool(status and status.get('result_written') and status.get('graph_audited')
                     and not status.get('stop_reason') and not status.get('required_process_failure'))
    info = dict(label=str(path.relative_to(case)), seed=trial['config'].get('seed'),
        run_valid_for_comparison=run_valid,
        result=result.get('reason'), goal_reached=trial['metrics']['goal_reached'],
        collision_physical=result.get('collision_physical'), joint_limit_violation=result.get('joint_limit_violation'),
        duration_from_motion_anchor_s=end-trial['origin'], appearance_time_s=appearance,
        first_all_arm_spheres_past_box_time_s=crossing, final_all_arm_spheres_past_box=bool(passed[-1]),
        arm_passage_verified=bool(crossing is not None and passed[-1]),
        maximum_lateral_offset_m=trial['metrics']['maximum_lateral_offset_m'],
        maximum_yaw_change_rad=trial['metrics']['maximum_heading_change_rad'],
        maximum_arm_joint_departure_from_initial_rad=trial['metrics']['maximum_joint_change_rad'],
        maximum_arm_joint_departure_from_appearance_rad=float(np.max(np.abs(trial['q'][active]
            -np.asarray(runtime['q'])))) if active.any() and runtime.get('q') is not None else None,
        arm_joint_path_l2_rad=trial['metrics']['arm_joint_path_l2_rad'],
        minimum_arm_box_recorded_gap_m=trial['metrics']['minimum_arm_box_sampled_gap_m'],
        minimum_arm_box_audit_gap_m=result.get('min_arm_box_gap_m'),
        minimum_base_box_audit_gap_m=result.get('min_base_box_gap_m'),
        minimum_external_audit_gap_m=result.get('min_external_gap_m'),
        zero_speed_after_motion_anchor_s=zero_duration(state, trial['origin'], end),
        zero_speed_during_encounter_s=None if appearance is None else zero_duration(state, appearance, encounter_end),
        encounter_duration_s=None if appearance is None else encounter_end-appearance,
        zero_speed_definition='Left-state interval integral: |v|<0.01 m/s AND |w|<0.03 rad/s; encounter ends at first complete arm passage or final sample',
        received_reference_bundle_count=len(refs),
        received_reference_bundles_after_appearance=sum(r['t'] >= appearance for r in refs) if appearance is not None else None,
        coupled_deliveries_after_initial=sum(r['complete_coupled_delivery_evidence'] for r in after),
        strict_optional_coupled_deliveries=sum(row['refresh_context']['kind'] == 'optional'
            and row['frozen_seed_identity']['state'] == 'consistent'
            and row['complete_coupled_delivery_evidence'] for row in after),
        refresh_kind_summary=refresh_kind_summary(requests),
        solver_summary=solve_summary(solves),
        failure_trace_file_present=(path/'failure_trace.jsonl').is_file(),
        manager_result_counts=dict(Counter(str(row.get('success', 'unknown')) for row in timeline
                                           if row['event'] == 'manager_result')),
        unknown_scope_solve_inputs=sum(item['match_state'] != 'matched' for request in requests
                                      for item in request['optimizer_scope_solves']),
        requests=requests, gate_event_kind_counts=dict(Counter(r.get('kind', r['event']) for r in gate)),
        gate_source_age_evidence=input_events,
        first_reference_content_sha256=None if first is None else digest({key: first.get(key) for key in ('singul', 'polynomials')}),
        first_reference_canonical_message_sha256=None if first is None else first.get('canonical_message_sha256'),
        first_reference_duration_s=None if first is None else sum(p['duration'] for p in first['polynomials']),
        appearance_base_pose=runtime.get('base_pose'), appearance_q=runtime.get('q'),
        actual_box_center=trial['center'].tolist(), actual_box_size=trial['size'].tolist(),
        appearance_front_gap_m=runtime.get('front_gap_m'),
        cycle_reason_counts=dict(Counter(r.get('reason', 'unknown') for r in cycles)),
        simulation_status=status,
        source_sha256={name: sha(path/name) for name in ('state.csv', 'result.json', 'parameters.yaml',
            'active_arm_config.json', 'scenario.json', 'box_runtime.json', 'dual_timeline.jsonl',
            'active_arm_cycles.jsonl', 'active_arm_reference_events.jsonl', 'trajectories.json',
            'failure_trace.jsonl') if (path/name).is_file()})
    trial['refresh_analysis'] = info
    return trial


def comparability(trials):
    pairs = []
    for i, a in enumerate(trials):
        for b in trials[i+1:]:
            x, y = a['refresh_analysis'], b['refresh_analysis']
            if x['seed'] != y['seed']:
                continue
            difference = lambda key: None if x[key] is None or y[key] is None else (np.asarray(y[key])-np.asarray(x[key])).tolist()
            pairs.append(dict(a=x['label'], b=y['label'], seed=x['seed'],
                both_runs_valid_for_comparison=x['run_valid_for_comparison'] and y['run_valid_for_comparison'],
                identical_initial_reference_content=x['first_reference_content_sha256'] == y['first_reference_content_sha256']
                    if x['first_reference_content_sha256'] and y['first_reference_content_sha256'] else None,
                initial_reference_duration_difference_s=difference('first_reference_duration_s'),
                box_center_difference_m=difference('actual_box_center'),
                appearance_base_pose_difference=difference('appearance_base_pose'),
                appearance_q_difference_rad=difference('appearance_q'),
                delta_maximum_lateral_offset_m=difference('maximum_lateral_offset_m'),
                delta_maximum_yaw_change_rad=difference('maximum_yaw_change_rad'),
                delta_encounter_zero_speed_s=difference('zero_speed_during_encounter_s'),
                interpretation='Observed paired closed-loop outcomes; different initial reference/appearance states prevent an exact single-factor causal claim'))
    return pairs


def plot(trials, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(11., 7.), constrained_layout=True)
    for trial in trials:
        info, state = trial['refresh_analysis'], trial['state']
        label = info['label']
        origin = info['appearance_time_s'] if info['appearance_time_s'] is not None else trial['origin']
        t = state['t']-origin
        line, = axes[0, 0].plot(t, state['v'], label=label)
        color = line.get_color()
        axes[0, 1].plot(t, state['y']-state['y'][0], color=color, label=label)
        axes[1, 0].plot(t, trial['arm_gap'], color=color, label=label)
        axes[1, 1].plot(t, np.rad2deg(np.unwrap(state['yaw'])-state['yaw'][0]), color=color, label=label)
        for request in info['requests']:
            if request['initial_request']:
                continue
            marker = 'o' if request['complete_coupled_delivery_evidence'] else 'x'
            axes[0, 1].plot(request['request_time_s']-origin, 0., marker, color=color, ms=6)
        for ref in trial['refs']+trial['origin']:
            if ref >= origin:
                for ax in axes.flat:
                    ax.axvline(ref-origin, color=color, lw=.8, ls=':', alpha=.45)
    settings = [('Actual base speed', 'v [m/s]'), ('Base lateral offset', 'y [m]'),
                ('Physical arm-box gap', 'gap [m]'), ('Base heading change', 'yaw [deg]')]
    for ax, (title, ylabel) in zip(axes.flat, settings):
        ax.set(title=title, xlabel='Time from obstacle appearance [s]', ylabel=ylabel)
        ax.axvline(0., color='gray', ls='--', lw=.6)
        ax.grid(alpha=.2)
        ax.legend(fontsize=7)
    axes[1, 0].axhline(0., color='red', lw=.7)
    fig.suptitle('Recorded closed-loop comparison; dotted: received reference; o/x: coupled/other refresh')
    fig.savefig(out/'refresh_comparison.png', dpi=130)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', type=Path, required=True)
    parser.add_argument('--out', type=Path)
    parser.add_argument('--no-plot', action='store_true')
    args = parser.parse_args()
    case = args.case.resolve()
    out = args.out.resolve() if args.out else case/'refresh_analysis'
    paths = scan(case)
    if not paths:
        raise ValueError('No completed arm-box trials: '+str(case))
    trials, unavailable = [], []
    for path in paths:
        try:
            trials.append(analyze(path, case))
        except (ValueError, KeyError, OSError) as error:
            unavailable.append(dict(label=str(path.relative_to(case)), error=str(error),
                result=read_json(path/'result.json'), run_status=read_json(path/'run_status.json')))
    if not trials:
        raise ValueError('No trials with usable recorded geometry: '+str(unavailable))
    out.mkdir(parents=True, exist_ok=True)
    summaries = [trial['refresh_analysis']['solver_summary'] for trial in trials]
    totals = {key: sum(row[key] for row in summaries) for key in
              ('solve_count', 'successful_count', 'failed_count', 'failed_without_named_rejection_count', 'total_wall_ms')}
    for key in ('reason_counts', 'failed_reason_counts'):
        counts = Counter()
        for row in summaries:
            counts.update(row[key])
        totals[key] = dict(counts)
    report = dict(schema='arm_box_refresh_analysis_v1', case=str(case), hardware_validation=False,
        trials=[trial['refresh_analysis'] for trial in trials], unavailable_trials=unavailable,
        paired_comparisons=comparability(trials), solver_totals=totals,
        mechanism_definition='Same request_id must contain prefix used, valid active-arm coupling scope, callbacks with available delivery targets, reference publication and a received replace event within [-.05,.5] seconds. Strict optional delivery additionally requires identical admission/prefix/coupling seed_hash and seed_epoch.',
        mechanism_limits='Replacement receipt is associated by time, not transport hash. Nonzero active callback counts implement cost>1e-12, so a perfect zero-cost match alone is not proven inactive. Missing ages remain unknown; freshness is not inferred from receipt time.',
        solver_evidence_limits='Only failure_trace kind=solve rows are counted. Scope linkage uses exact input_hash (and source time if repeated). A raw reason=ok with success=false remains a failed solve with no named rejection; it is not relabeled as success or assigned a guessed cause.',
        scope='Recorded numerical ROS closed-loop outcomes and sampled physical spheres only; offline mechanism evidence does not certify global causality or real-robot behavior',
        analyzer_sha256=sha(Path(__file__)))
    (out/'refresh_analysis.json').write_text(json.dumps(safe(report), ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    if not args.no_plot:
        plot(trials, out)
    print(str(out/'refresh_analysis.json'))


if __name__ == '__main__':
    main()
