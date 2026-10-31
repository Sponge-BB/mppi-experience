#!/usr/bin/env python3
"""Summarize existing timing probes; never starts ROS or changes parameters."""
from collections import Counter
import gzip
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'results/execution_timing_diagnosis_20261028_01'
OLD = ROOT / 'results/execution_timing_pair_20261028_01/dynamic_head_on_center_matched_seed11'
NEW = OUT / 'online_probe'


def jsonl(path):
    opener = gzip.open if path.suffix == '.gz' else open
    with opener(path, 'rt') as stream:
        return [json.loads(line) for line in stream if line.strip()]


def before_contact_cycles(trial):
    result = json.loads((trial / 'result.json').read_text())
    return [r for r in jsonl(trial / 'state_mppi_cycles.jsonl')
            if r['command_publish_after_ros_time'] < 1000 + result['elapsed'] - 1e-7]


def gaps(cycles):
    return [dict(cycle=a['cycle_id'], next_cycle=b['cycle_id'],
                 publish_mono=a['command_publish_after_monotonic'],
                 next_start_mono=b['cycle_start_monotonic'],
                 gap_s=b['cycle_start_monotonic'] - a['command_publish_after_monotonic'],
                 next_reason=b['reason']) for a, b in zip(cycles, cycles[1:])]


def main():
    import numpy as np
    from sys import path as module_path
    module_path.insert(0, str(ROOT))
    from report_execution_timing import measured_stops
    original = before_contact_cycles(OLD)
    gap = max(gaps(original), key=lambda r: r['gap_s'])
    samples = jsonl(OLD / 'cpu_samples.jsonl.gz')
    # Enclose the whole gap: a sample's start is NOT its /proc-read timestamp.
    left = max((s for s in samples if s['mono'] + s['sample_ms'] / 1000 < gap['publish_mono']),
               key=lambda s: s['mono'])
    right = min((s for s in samples if s['mono'] > gap['next_start_mono']), key=lambda s: s['mono'])
    def main_thread(sample):
        process = next(p for p in sample['processes'] if p['name'] == 'base_mppi')
        return next(t for t in process['threads'] if t['tid'] == process['pid'])
    lthread, rthread = main_thread(left), main_thread(right)
    hz = os.sysconf('SC_CLK_TCK')
    cpu_ms = (rthread['ticks'] - lthread['ticks']) / hz * 1000
    schedule_ms = (rthread['wait_ns'] - lthread['wait_ns']) / 1e6
    cpu = dict(enclosing_start_sample_mono=left['mono'],
               enclosing_start_sample_complete_mono=left['mono'] + left['sample_ms'] / 1000,
               enclosing_end_sample_start_mono=right['mono'],
               cpu_counter_delta_ms=cpu_ms, cpu_conservative_upper_bound_ms=cpu_ms + 2000 / hz,
               runnable_wait_delta_ms=schedule_ms,
               note='Two per-mode tick quantization allowances; envelope, NOT exact CPU within the gap.')
    old_state = np.genfromtxt(OLD / 'state.csv', delimiter=',', names=True)
    cycle = next(r for r in original if r['cycle_id'] == gap['cycle'])
    next_cycle = next(r for r in original if r['cycle_id'] == gap['next_cycle'])
    plant = old_state[(old_state['t'] >= cycle['command_publish_after_ros_time'] - 1000) &
                      (old_state['t'] <= next_cycle['solve_ros_time'] - 1000)]
    fresh = before_contact_cycles(NEW)
    result = json.loads((NEW / 'result.json').read_text())
    fresh = [r for r in fresh if r['solve_ros_time'] >= 1000 + result['goal_sent']]
    active = [r for r in fresh if r['solve_ros_time'] >= 1000 + result['native_realtime_evidence']['first_truth_actor_t']]
    probe = json.loads((NEW / 'loop_stages.json').read_text())
    stages = [r for r in probe['rows'] if 1000 + result['goal_sent'] <= r[4] < 1000 + result['elapsed'] - 1e-7]
    spans = [dict(cycle=a[0], stage=b[1], previous_stage=a[1],
                  wall_ms=(b[2] - a[2]) * 1000, cpu_ms=(b[3] - a[3]) * 1000)
             for a, b in zip(stages, stages[1:]) if a[0] == b[0]]
    deadlines = [r for r in active if r['reason'] == 'deadline_miss']
    diagnostics = json.load(gzip.open(NEW / 'mppi_diagnostics.json.gz', 'rt'))
    braking = next(r for r in diagnostics if r['t'] < result['elapsed'] - 1e-7 and
                   r['reason'] == 'local_validation_failed' and r['validation_code'] == 5)
    states = np.genfromtxt(NEW / 'state.csv', delimiter=',', names=True)
    report = dict(scope='Read-only summary: original completed trial plus ONE instrumented same-parameter SIL rerun; NOT Gazebo/hardware or a speed comparison.',
        original_gap=gap, original_cpu_envelope=cpu,
        original_plant_during_gap=dict(rows=len(plant), max_record_dt_s=float(np.diff(plant['t']).max())),
        rerun=dict(result=result, status=json.loads((NEW / 'run_status.json').read_text()),
                   largest_after_goal_postpublish_gap=max(gaps(fresh), key=lambda r: r['gap_s']),
                   largest_precontact_stage_spans=sorted(spans, key=lambda r: r['wall_ms'], reverse=True)[:12],
                   actor_window_reasons=dict(Counter(r['reason'] for r in active)),
                   deadline_cycles=[dict(cycle=r['cycle_id'], validation_code=r['validation_code'],
                                         attempts=r['validation_attempts'], published=r['validated_command_published'])
                                    for r in deadlines],
                   all_deadline_cycles_already_failed_braking=bool(deadlines) and all(r['validation_code'] == 5 for r in deadlines),
                   first_braking_rejection=dict(solve_t=braking['t'], callback_ms=braking['callback_ms'],
                       last_attempt_selected_min_gap_m=braking['validation'][1],
                       last_attempt_brake_min_gap_m=braking['validation'][2],
                       attempts=braking['validation_attempts']),
                   measured_stops=measured_stops(states, result['native_realtime_evidence']['first_truth_actor_t'], result['elapsed'])),
        limits=['The old rare 2.819 s stall did not recur; exact blocking line remains unproven.',
                'Watchdog stacks collected while stopping after collision must not be assigned to the earlier driving stall.',
                'The three deadline labels overwrite an already failed validation; they are not three proven safe actions discarded only by time.',
                'Positive full-rollout clearance and negative fallback clearance refer to DIFFERENT future motions; this alone does not prove the brake check is unnecessary.',
                'Perception reconstruction is in perception_audit.json; do not equate point deletion with proof of the sole collision cause.'])
    (OUT / 'timing_audit.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(dict(original_gap=gap, original_cpu_envelope=cpu,
                          rerun_largest_gap=report['rerun']['largest_after_goal_postpublish_gap'],
                          rerun_reasons=report['rerun']['actor_window_reasons'],
                          first_braking_rejection=report['rerun']['first_braking_rejection'],
                          measured_stops=report['rerun']['measured_stops']), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
