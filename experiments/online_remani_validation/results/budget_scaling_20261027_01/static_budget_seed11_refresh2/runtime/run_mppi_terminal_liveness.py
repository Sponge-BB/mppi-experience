#!/usr/bin/env python3
"""Fixed-reference MPPI endpoint liveness experiment; no ROS or robot I/O.

Runs the actual kernel AND execution/braking validator with projected body
disks. Ideal free field and synchronous state are deliberate counterfactuals,
not a replay of the original sensor stream or an online REMANI experiment.
"""
import argparse
import collections
import ctypes as ct
import csv
import json
import math
import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import numpy as np

from dynamic_scenarios import body_clearance
from run_mppi_rejoin_diagnosis import Kernel, World, DP, ptr, extract_plant, sha, wrap, write_json

HERE = Path(__file__).resolve().parent
SOURCE = HERE/'results/collab_factorial_20261022/formal_extra_01/dynamic_head_on_center_D_seed04'
CASES = ('recorded_history', 'inside_history', 'mirrored_history',
         'outside_legacy', 'inside_legacy', 'far_live', 'overshoot_history',
         'alignment_history', 'blocked_history')
SWITCH = 'MPPI_TERMINAL_PHASE_FIX'
CRITERIA = dict(distance_m=.05, yaw_rad=.1, linear_speed_mps=.05,
                angular_speed_rps=.05, dwell_s=1., duration_s=20., dt_s=.05,
                substep_s=.005, decision_deadline_ms=50., margin_m=.1,
                prime_state_advanced=False)


def fixture(source, case):
    records = [json.loads(line) for line in (source/'consumed_references.jsonl').read_text().splitlines() if line]
    reference = np.ascontiguousarray(records[-1]['reference'], dtype=float)
    state = np.asarray(json.loads((source/'result.json').read_text())['final_state'], dtype=float)
    if case.startswith('inside_'):
        state[1] = .24
    if case == 'mirrored_history':
        reference[:, 1:3] *= -1.
        state[1:3] *= -1.
        state[4] *= -1.
    if case == 'overshoot_history':
        state = np.array([reference[-1, 0]+.27, reference[-1, 1], reference[-1, 2], 0., 0.])
    if case == 'alignment_history':
        state = np.array([reference[-1, 0]-.01, reference[-1, 1],
                          wrap(reference[-1, 2]+math.pi/2), 0., 0.])
    body = np.ascontiguousarray(np.atleast_2d(np.loadtxt(source/'geometry.csv', delimiter=',')))
    limits = json.loads((source/'limits.json').read_text())
    return state, reference, body, limits


def actors_for(case, state, body, t):
    if case == 'far_live':
        return [(4.+.1*t, 3.5, .2, 1, .1, 0.)]
    if case == 'blocked_history':
        # Positive physical gap .05m, but below the unchanged .10m margin.
        # Place against the highest world-y disk: no disk can overlap it.
        co, si = math.cos(state[2]), math.sin(state[2])
        centers = state[:2]+body[:, :2]@np.array([[co, si], [-si, co]])
        i = int(np.argmax(centers[:, 1]+body[:, 2]))
        return [(centers[i, 0]+.001*t, centers[i, 1]+body[i, 2]+.2+.05,
                 .2, 1, .001, 0.)]
    return []


def observations(actors):
    return np.ascontiguousarray([[x-.04*vx, y-.04*vy, vx, vy, r, 1.]
                                for x, y, r, _, vx, vy in actors], dtype=float).reshape(-1, 6)


def run_case(build, source, out, case, enabled, seed=4, duration=20.):
    out.mkdir(parents=True, exist_ok=False)
    state, reference, body, limits = fixture(source, case)
    initial = state.copy()
    field = np.full((World.ny, World.nx), 10., dtype=float)
    integrate, plant_hash = extract_plant()
    previous = os.environ.get(SWITCH)
    os.environ[SWITCH] = '1' if enabled else '0'
    try:
        kernel = Kernel(build, body, limits, seed)
    finally:
        if previous is None:
            os.environ.pop(SWITCH, None)
        else:
            os.environ[SWITCH] = previous
    kernel.lib.online_dynamic_strategy_debug.argtypes = [ct.c_void_p, DP]
    plant = SimpleNamespace(s=state, command=np.zeros(2), command_time=0., command_lease=.075,
                            limits=limits, lock=threading.Lock(), scene='terminal_free_field', t=0.)
    metadata = dict(case=case, enabled=enabled, seed=seed, criteria=CRITERIA,
        source=str(source), build=str(build), source_record_hash=sha(source/'consumed_references.jsonl'),
        reference_sha256=__import__('hashlib').sha256(reference.tobytes()).hexdigest(),
        library_sha256=sha(build/'build/libonline_mppi.so'), initial_state=initial.tolist(),
        geometry=body.tolist(), geometry_sha256=sha(source/'geometry.csv'), limits=limits,
        plant_method_sha256=plant_hash, runner_sha256=sha(Path(__file__)),
        inputs_sha256={str(path): sha(path) for path in
            (source/'result.json', source/'limits.json', HERE/'run_mppi_rejoin_diagnosis.py',
             HERE/'simulator.py', HERE/'dynamic_scenarios.py')},
        duration_s=duration, branch_priming=case not in ('outside_legacy', 'inside_legacy'),
        scope=__doc__, collision_contract='Unchanged actual kernel clearance and publication/braking validator; '
        'constant 10m field, full projected disks and original limits; no shared-objective claim.')
    write_json(out/'metadata.json', metadata)
    np.savetxt(out/'reference.csv', reference, delimiter=',', header='x,y,yaw,v', comments='')
    rows, decisions, trace = [], [], []
    counts, validations = collections.Counter(), collections.Counter()
    dwell, reached = 0., None
    initial_actors = actors_for(case, initial, body, 0.)
    initial_gap = body_clearance(initial, body, initial_actors)
    minimum_gap = initial_gap
    start_wall = time.monotonic()
    try:
        if metadata['branch_priming']:
            prime = kernel.solve_validate(state, reference, np.array([[4., 3.5, .1, 0., .2, 1.]]), field)
            # No physical advance while arming the historical branch. Feedback
            # resets actual issued action to zero before the measured trial.
            kernel.lib.online_feedback(kernel.handle, 0., 0.)
            write_json(out/'priming.json', {k: v.tolist() if isinstance(v, np.ndarray) else v
                                           for k, v in prime.items()})
        for k in range(int(math.ceil(duration/.05))):
            t = k*.05
            actors = actors_for(case, initial, body, t)
            solved = kernel.solve_validate(plant.s, reference, observations(actors), field)
            cmd = solved['command']
            dynamic = np.zeros(4)
            kernel.lib.online_dynamic_strategy_debug(kernel.handle, ptr(dynamic))
            distance = float(np.linalg.norm(plant.s[:2]-reference[-1, :2]))
            yaw = abs(wrap(plant.s[2]-reference[-1, 2]))
            counts[solved['reason']] += 1
            counts['active_zero'] += int(solved['active_zero'])
            for attempt in solved['attempts']:
                validations[str(attempt['code'])] += 1
            bounds = max(abs(cmd[0])/limits['vmax'], abs(cmd[1])/limits['wmax'],
                         abs(cmd[0]-.5*limits['track']*cmd[1])/limits['wheel_v'],
                         abs(cmd[0]+.5*limits['track']*cmd[1])/limits['wheel_v'])
            rows.append([t, *plant.s, *cmd, distance, yaw, solved['callback_ms'],
                         solved['kernel_ms'], solved['reason'], int(solved['active_zero']),
                         *dynamic, bounds])
            decisions.append(dict(t=t, **{key: value.tolist() if isinstance(value, np.ndarray) else value
                              for key, value in solved.items() if key != 'route'}))
            if k % 5 == 0:
                search, total, export_ms = kernel.search(max_candidates=12)
                trace.append(dict(t=t, total=total, rows=search.tolist(), export_ms=export_ms))
            plant.command, plant.command_time = cmd, t
            for j in range(10):
                plant.t = t+(j+1)*.005
                integrate(plant, .005)
                minimum_gap = min(minimum_gap, body_clearance(plant.s, body,
                    actors_for(case, initial, body, plant.t)))
                endpoint = (np.linalg.norm(plant.s[:2]-reference[-1, :2]) <= .05
                            and abs(wrap(plant.s[2]-reference[-1, 2])) <= .1
                            and abs(plant.s[3]) <= .05 and abs(plant.s[4]) <= .05)
                dwell = dwell+.005 if endpoint else 0.
                if reached is None and dwell >= 1.-1e-9:
                    reached = plant.t-dwell
            if reached is not None or minimum_gap < 0.:
                break
    finally:
        kernel.close()
    with (out/'state.csv').open('w') as f:
        writer = csv.writer(f)
        writer.writerow(('t', 'x', 'y', 'yaw', 'v', 'w', 'cmd_v', 'cmd_w', 'goal_distance',
                         'goal_yaw_error', 'callback_ms', 'kernel_ms', 'reason', 'active_zero',
                         'dynamic_using', 'dynamic_active', 'lateral_needed', 'had_dynamic', 'bound_ratio'))
        writer.writerows(rows)
    for name, values in [('decisions', decisions), ('search', trace)]:
        with (out/(name+'.jsonl')).open('w') as f:
            for value in values:
                f.write(json.dumps(value, allow_nan=False)+'\n')
    result = dict(case=case, enabled=enabled, seed=seed, reached=reached is not None,
        settled_start_s=reached, duration_s=len(rows)*.05, initial_gap_m=initial_gap,
        min_physical_gap_m=minimum_gap, final_state=plant.s.tolist(),
        initial_goal_distance_m=float(np.linalg.norm(initial[:2]-reference[-1, :2])),
        final_goal_distance_m=float(np.linalg.norm(plant.s[:2]-reference[-1, :2])),
        final_yaw_error_rad=abs(wrap(plant.s[2]-reference[-1, 2])),
        active_zero_count=counts['active_zero'], reason_counts=dict(counts), validator_counts=dict(validations),
        deadline_misses=counts['deadline_miss'], max_command_bound_ratio=max(r[-1] for r in rows),
        callback_p95_ms=float(np.percentile([r[10] for r in rows], 95)),
        kernel_p95_ms=float(np.percentile([r[11] for r in rows], 95)),
        max_command_magnitude=float(max(max(abs(r[6]), abs(r[7])) for r in rows)),
        collision=minimum_gap < 0., actual_wall_s=time.monotonic()-start_wall,
        library_sha256=metadata['library_sha256'], reference_sha256=metadata['reference_sha256'])
    final_endpoint = (result['final_goal_distance_m'] <= .05 and result['final_yaw_error_rad'] <= .1
                      and abs(plant.s[3]) <= .05 and abs(plant.s[4]) <= .05)
    result['final_endpoint_valid'] = bool(final_endpoint)
    result['negative_safety_rejection_valid'] = bool(case == 'blocked_history'
        and 0. <= initial_gap < limits['margin']
        and all(r[12] == 'no_feasible_candidate' or
                (r[12] == 'local_validation_failed' and all(a['code'] == 3 for a in d['attempts'])
                 and len(d['attempts']) > 0) for r, d in zip(rows, decisions)))
    result['passed'] = bool(minimum_gap >= 0. and result['max_command_bound_ratio'] <= 1.+1e-8
        and result['deadline_misses'] == 0 and (result['max_command_magnitude'] < 1e-9
        and result['negative_safety_rejection_valid'] if case == 'blocked_history'
        else result['reached'] and final_endpoint))
    for path, digest in metadata['inputs_sha256'].items():
        if sha(Path(path)) != digest:
            raise RuntimeError('Frozen input changed during run: '+path)
    if sha(Path(__file__)) != metadata['runner_sha256'] or sha(build/'build/libonline_mppi.so') != metadata['library_sha256']:
        raise RuntimeError('Runner/library changed during run')
    write_json(out/'result.json', result)
    print('RESULT', case, 'ON' if enabled else 'OFF', json.dumps(result), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--source', type=Path, default=SOURCE)
    parser.add_argument('--cases', nargs='+', choices=CASES, default=list(CASES))
    parser.add_argument('--mode', choices=('off', 'on', 'paired'), default='paired')
    parser.add_argument('--duration', type=float, default=20.)
    args = parser.parse_args()
    if not 0 < args.duration <= 60:
        parser.error('duration must be in (0,60]s')
    manifest = json.loads((args.build/'build_manifest.json').read_text())
    if manifest.get('prepared_only') or sha(args.build/'build/libonline_mppi.so') != manifest['mppi_library_sha256']:
        raise ValueError('Unfinished or changed build')
    if args.mode != 'off' and not manifest.get('mppi_terminal_liveness_experiment'):
        raise ValueError('ON requires a compiled switch-bearing build')
    args.out.mkdir(parents=True, exist_ok=False)
    # Save the predeclared protocol before any outcomes are measured.
    write_json(args.out/'protocol.json', dict(cases=args.cases, mode=args.mode, criteria=CRITERIA,
        duration_s=args.duration, scope=__doc__, runner_sha256=sha(Path(__file__))))
    modes = [False, True] if args.mode == 'paired' else [args.mode == 'on']
    results = []
    for i, case in enumerate(args.cases):
        # Alternate OFF/ON ordering; each case/controller starts fresh.
        for enabled in (modes if i % 2 == 0 else list(reversed(modes))):
            results.append(run_case(args.build.resolve(), args.source.resolve(),
                args.out/(case+('_on' if enabled else '_off')), case, enabled, duration=args.duration))
    write_json(args.out/'summary.json', dict(results=results, scope=__doc__))


if __name__ == '__main__':
    main()
