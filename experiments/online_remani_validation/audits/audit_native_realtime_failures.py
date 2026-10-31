#!/usr/bin/env python3
"""Read-only evidence for residual gear-cusp stall and head-on deadline stall.

Recomputes only the recorded geometric projection, not a controller/simulation.
No safety authority, new seed, optimizer invocation or modification of trials.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from report_coupled_objective import read, lines


def project(reference, state):
    """BaseMPPI::project hint=-1, preserving strict first minimum and zero skip."""
    points = reference[:, :2]
    lengths = np.hypot(*(np.diff(points, axis=0).T))
    arc = np.concatenate(([0.], np.cumsum(lengths)))
    best, index = 1e9, 0
    for i in range(len(points)-1):
        length = arc[i+1]-arc[i]
        if length < 1e-9:
            continue
        delta = points[i+1]-points[i]
        fraction = np.clip(np.dot(state[:2]-points[i], delta)/(length*length), 0., 1.)
        residual = state[:2]-(points[i]+fraction*delta)
        distance = float(np.dot(residual, residual))
        if distance < best:
            best, index = distance, i
    return index, float(np.sqrt(best))


def audit_static(trial):
    result = read(trial/'result.json')
    refs = {r['sha256']: np.asarray(r['reference']) for r in lines(trial/'consumed_references.jsonl')}
    diagnostics = read(trial/'mppi_diagnostics.json')
    by_time = {round(r['t']+1000, 8): r for r in diagnostics}
    cycles = [r for r in lines(trial/'state_mppi_cycles.jsonl')
        if r.get('online_step_succeeded') and r['solve_ros_time']-1000 >= result['elapsed']-20]
    projections, errors, commands = [], [], []
    for cycle in cycles:
        row = by_time[round(cycle['solve_ros_time'], 8)]
        reference = refs[cycle['reference_array_sha256']]
        index, distance = project(reference, np.asarray(cycle['solve_state']))
        projections.append(index)
        errors.append(abs(distance-row['local_phase']['decision'][7]))
        commands.append(row)
    reference = refs[cycles[-1]['reference_array_sha256']]
    state = np.genfromtxt(trial/'state.csv', delimiter=',', names=True)
    tail = state[state['t'] >= result['elapsed']-20]
    final = np.asarray(result['final_state'])
    nearest = int(np.argmin(np.linalg.norm(reference[:, :2]-final[:2], axis=1)))
    return dict(trial=str(trial), final_reference_endpoint=reference[-1].tolist(),
        final_state=final.tolist(), final_endpoint_error_m=float(np.linalg.norm(reference[-1, :2]-final[:2])),
        nearest_reference_index=nearest, nearest_reference_pose=reference[nearest].tolist(),
        nearest_reference_distance_m=float(np.linalg.norm(reference[nearest, :2]-final[:2])),
        last20s_cycles=len(cycles), all_authorized=all(r['authorized'] for r in commands),
        all_validation_passed=all(r['validation_code'] == 0 for r in commands),
        deadline_misses=sum(r['reason'] == 'deadline_miss' for r in commands),
        reasons=dict(Counter(r['reason'] for r in commands)),
        projection_index_counts=dict(Counter(projections)),
        projection_index_switches=sum(a != b for a, b in zip(projections, projections[1:])),
        projection_distance_vs_logged_max_error_m=max(errors),
        net_displacement_m=float(np.hypot(tail['x'][-1]-tail['x'][0], tail['y'][-1]-tail['y'][0])),
        command_sign_counts=dict(forward=sum(r['applied_v'] > 1e-6 for r in commands),
            reverse=sum(r['applied_v'] < -1e-6 for r in commands),
            zero_linear=sum(abs(r['applied_v']) <= 1e-6 for r in commands)),
        direction_switch_points=[dict(index=i, reference=reference[i].tolist(), next_reference=reference[i+1].tolist())
            for i in range(len(reference)-1) if reference[i, 3]*reference[i+1, 3] < 0],
        conclusion='near-duplicate reverse/forward cusp; global nearest-space association switches motion phase and checked REJOIN outputs counteracting actions; not deadline or missing global endpoint')


def audit_head_on(trial):
    result = read(trial/'result.json')
    rows = read(trial/'mppi_diagnostics.json')
    first_dynamic = result['native_realtime_evidence']['first_observed_moving_t']
    active = [r for r in rows if r.get('reference_points', 0) > 0 and r['t'] >= first_dynamic]
    last_ok = max((r for r in active if r['reason'] == 'ok' and r.get('authorized')), key=lambda r:r['t'])
    later = [r for r in active if r['t'] > last_ok['t']]
    first_deadline = next(r for r in later if r['reason'] == 'deadline_miss')
    block = []
    for row in later:
        if row['reason'] != 'deadline_miss':
            break
        block.append(row)
    keys = ('t', 'reason', 'nominal_v', 'nominal_w', 'applied_v', 'applied_w',
            'kernel_ms', 'callback_ms', 'validation_code', 'validation', 'local_phase')
    state = np.genfromtxt(trial/'state.csv', delimiter=',', names=True)
    late = state[state['t'] >= 20]
    return dict(trial=str(trial), termination=result['reason'], collision_time_s=result['elapsed'],
        physical_sample_gap_m=result['native3d_truth_monitor']['min_physical_sample_gap'],
        last_authorized={k:last_ok[k] for k in keys}, first_deadline={k:first_deadline[k] for k in keys},
        consecutive_deadline_cycles=len(block), deadline_block_validation_passed=sum(r['validation_code'] == 0 for r in block),
        deadline_block_kernel_p50_ms=float(np.median([r['kernel_ms'] for r in block])),
        deadline_block_kernel_max_ms=max(r['kernel_ms'] for r in block),
        deadline_block_callback_p50_ms=float(np.median([r['callback_ms'] for r in block])),
        later_reason_counts=dict(Counter(r['reason'] for r in later)),
        after20s_max_abs_linear_speed=float(np.max(np.abs(late['v']))),
        after20s_net_displacement_m=float(np.hypot(late['x'][-1]-late['x'][0], late['y'][-1]-late['y'][0])),
        conclusion='intervention candidates initially pass finite rollout/brake checks but miss deadline; robot stops while incoming actor continues, later all candidates become infeasible; no claim that removing timeout alone guarantees avoidance')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if args.out.exists():
        raise FileExistsError('Fresh audit output required')
    result = dict(static=audit_static(args.batch/'static_demand_seed11'),
        head_on=audit_head_on(args.batch/'dynamic_head_on_center_demand_seed11'),
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        scope='read-only recorded-input geometric association and event arithmetic; not a new closed-loop run')
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2)+'\n')
    print('static', {k:result['static'][k] for k in ('last20s_cycles', 'projection_index_counts',
        'projection_index_switches', 'projection_distance_vs_logged_max_error_m', 'net_displacement_m')})
    print('head_on', {k:result['head_on'][k] for k in ('consecutive_deadline_cycles',
        'deadline_block_validation_passed', 'deadline_block_kernel_p50_ms', 'collision_time_s')})


if __name__ == '__main__':
    main()
