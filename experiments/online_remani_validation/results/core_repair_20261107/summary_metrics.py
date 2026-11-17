#!/usr/bin/env python3
"""Read completed trial records; no ROS, controls, source edits or benchmarks."""
import csv
import datetime
import json
from collections import Counter
from pathlib import Path

OUT = Path(__file__).resolve().parent
RUNS = {
    'failure_capture': 'historical_failure_diagnostic',
    'empty_mid360_s11': 'preclock',
    'moving_ideal_s11': 'preclock',
    'empty_mid360_fixed_clock_s11': 'postclock_pretracker',
    'blind_enter_mppi_s11': 'postclock_pretracker',
    'blind_enter_native_s11': 'postclock_pretracker',
    'blind_enter_tracker_fixed_s11': 'postclock_tracker_fixed',
    'blind_enter_native_tracker_fixed_s11': 'postclock_tracker_fixed',
}


def records(path):
    if path.exists():
        with path.open() as stream:
            for line in stream:
                if line.strip():
                    yield json.loads(line)


def scene_record(row, end):
    elapsed = row.get('elapsed_s')
    return elapsed is not None and -1e-8 <= elapsed <= end+1e-8


def metrics(name, stage):
    directory = OUT/name
    runner = json.loads((directory/'runner.json').read_text())
    result = json.loads((directory/'result.json').read_text())
    experiment = json.loads((directory/'experiment.json').read_text())
    start, end = runner['start_ros_time'], result['elapsed_s']
    events = [r for r in records(directory/'navigation_events.jsonl') if scene_record(r, end)]
    statuses = [r for r in records(directory/'mppi_status.jsonl') if scene_record(r, end)]
    snapshots = [r for r in records(directory/'snapshot_headers.jsonl') if scene_record(r, end)]
    goal = runner.get('goal_sent_ros_time')
    finishes = [r['elapsed_s'] for r in events if r['kind'] == 'finish' and r['data'] is True
                and goal is not None and r['ros_time'] >= goal]
    runner_finish = runner.get('finish_after_goal_ros_time')
    if runner_finish is not None and goal is not None and goal <= runner_finish <= start+end+1e-8:
        finishes.append(runner_finish-start)
    counts = Counter(r['status']['state'] for r in statuses)
    delivered = [r for r in statuses if r['status']['state'] == 'command_delivered']
    faults = [r for r in events if r['kind'] == 'tracking_fault' and r['data'] is True]
    map_ages = []
    for r in statuses:
        diagnostic = r['status'].get('candidate_diagnostics', {})
        h = diagnostic.get('snapshot_header', [])
        if len(h) == 32 and 'plan_time' in diagnostic and 'solve_ms' in diagnostic:
            map_ages.append((diagnostic['plan_time']-h[3], diagnostic['plan_time']-start,
                             r['status']['state'], int(h[2])))
    maximum = max(map_ages, default=None)
    seen = False; first_dynamic = None; loss = None; intervals = []; opened = None
    for r in snapshots:
        h = r['header']; elapsed = r['elapsed_s']
        if h[19] == 1 and h[13] > 0:
            seen = True
            if first_dynamic is None:
                first_dynamic = dict(receive_elapsed_s=elapsed, observation_source_elapsed_s=h[18]-start,
                                     dynamic_points=int(h[13]))
        if seen and h[19] != 1:
            if loss is None:
                loss = dict(receive_elapsed_s=elapsed, last_observation_source_elapsed_s=h[18]-start,
                    source_age_at_receive_s=start+elapsed-h[18], snapshot_sequence=int(h[2]),
                    map_valid=int(h[4]), dynamic_valid=int(h[19]), dynamic_points=int(h[13]))
            if opened is None:
                opened = elapsed
        elif opened is not None:
            intervals.append(dict(start_s=opened, end_s=elapsed, recovered=True)); opened = None
    if opened is not None:
        intervals.append(dict(start_s=opened, end_s=end, recovered=False))
    visibility = []
    with (directory/'sensor_visibility.csv').open() as stream:
        for row in csv.DictReader(stream):
            if row.get('t') and -1e-8 <= float(row['t']) <= end+1e-8:
                visibility.append(dict(source_elapsed_s=float(row['t']),
                    publish_elapsed_s=float(row['publish_t']),
                    actor_present=row['actor_present'] == 'True',
                    visible_points=int(row['actor_visible_points'])))
    first_visible = next((r for r in visibility if r['actor_present'] and r['visible_points'] > 0), None)
    invisible = next((r for r in visibility if first_visible is not None
        and r['source_elapsed_s'] > first_visible['source_elapsed_s']
        and r['actor_present'] and r['visible_points'] == 0), None)
    after_loss = None
    if loss is not None:
        boundary = loss['receive_elapsed_s']
        after = [r for r in delivered if r['elapsed_s'] >= boundary]
        trajectory = [r for r in events if r['kind'] == 'trajectory'
            and r['elapsed_s'] >= boundary and r['data']['action'] == 1 and r['data']['pieces']]
        visible_at_loss = [r for r in visibility if r['publish_elapsed_s'] <= boundary]
        after_loss = dict(mppi_delivered_count=len(after),
            first_delivery_elapsed_s=after[0]['elapsed_s'] if after else None,
            added_trajectory_message_count=len(trajectory),
            new_trajectory_id1_count=sum(r['data']['trajectory_id'] == 1 for r in trajectory),
            first_new_trajectory_elapsed_s=trajectory[0]['elapsed_s'] if trajectory else None,
            latest_sensor_sample_at_first_loss=visible_at_loss[-1] if visible_at_loss else None,
            loss_receive_minus_geometric_invisibility_source_s=None if invisible is None
                else boundary-invisible['source_elapsed_s'])
    violation = result.get('first_violation') or {}
    return dict(stage=stage, group=experiment['group'], scene=experiment['scene'],
        sensor_model=experiment['sensor_model'], duration_s=end,
        runner_reason=runner['runner_reason'], result_reason=result['reason'],
        duration_completed=bool(result.get('completed')), actual_goal_finish=bool(finishes),
        first_goal_finish_elapsed_s=min(finishes) if finishes else None,
        collision=bool(result['collision_physical']),
        collision_elapsed_s=violation.get('t') if result['collision_physical'] else None,
        max_arm_change_deg=result.get('max_arm_change_deg'), base_path_length_m=result.get('base_path_length_m'),
        max_base_displacement_m=result.get('max_base_displacement_m'),
        final_goal_position_error_m=result.get('final_goal_position_error_m'),
        peak_graph_rss_mib=runner.get('peak_graph_rss_mib'),
        tracking_fault_true_messages=len(faults), first_tracking_fault_true_elapsed_s=faults[0]['elapsed_s'] if faults else None,
        mppi_cycle_status_count=len(statuses), mppi_delivered_count=len(delivered),
        status_counts=dict(sorted(counts.items())),
        planning_snapshot_count=len(map_ages), planning_snapshot_max_age_s=maximum[0] if maximum else None,
        maximum_map_age_context=None if maximum is None else dict(plan_elapsed_s=maximum[1], status=maximum[2], sequence=maximum[3]),
        first_valid_dynamic_observation=first_dynamic, first_dynamic_source_invalid_after_observation=loss,
        dynamic_invalid_intervals=intervals, first_geometrically_visible_actor=first_visible,
        first_geometrically_invisible_after_visible=invisible, after_first_dynamic_loss=after_loss)


def main():
    complete = {}; unfinished = []
    for name, stage in RUNS.items():
        if all((OUT/name/f).exists() for f in ('runner.json', 'result.json', 'experiment.json')):
            complete[name] = metrics(name, stage)
        else:
            unfinished.append(name)
    report = dict(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        definitions={
            'window': 'Scene0 <= record.elapsed_s <= result.elapsed_s; warmup excluded. Native has no MPPI statuses.',
            'goal_finish': 'True finish event after the sent goal, cross-checked with runner finish timestamp. Duration completed alone is not arrival.',
            'planning_map_age': 'Exact diagnostic.plan_time minus its snapshot_header[3]; only records with solve_ms, not receive-time approximation.',
            'dynamic_loss': 'First header[19]!=1 after an actually valid nonempty dynamic observation (header[19]==1,header[13]>0). Initial empty observations are excluded. nd0 when invalid is expected.',
            'dynamic_source': 'header[18] is source stamp; invalid snapshots can retain last source while map stamp advances.',
            'geometric_invisibility': 'First actor-present sensor sample with0 visible points after at least one visible sample; sensor source t and publication time are separately reported.',
            'post_loss': 'Messages received after first invalid snapshot. New trajectory means ADD/action1 with pieces; counts do not imply execution.',
            'scope': 'Recorded numerical closed-loop trial metrics, not hardware validation; geometric model omits scan/noise/robot occlusion.'},
        runs=complete, skipped_unfinished=unfinished,
        related_figure='tracker_repair_comparison.png')
    (OUT/'summary_metrics.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    for name, r in complete.items():
        loss = r['first_dynamic_source_invalid_after_observation']
        invisible = r['first_geometrically_invisible_after_visible']
        print(name, json.dumps(dict(finish=r['actual_goal_finish'], collision_s=r['collision_elapsed_s'],
            arm_deg=r['max_arm_change_deg'], path_m=r['base_path_length_m'], rss_mib=r['peak_graph_rss_mib'],
            fault_true=r['tracking_fault_true_messages'], max_map_age_s=r['planning_snapshot_max_age_s'],
            loss_s=None if loss is None else loss['receive_elapsed_s'],
            invisible_s=None if invisible is None else invisible['source_elapsed_s'],
            after_loss=r['after_first_dynamic_loss'])))


if __name__ == '__main__':
    main()
