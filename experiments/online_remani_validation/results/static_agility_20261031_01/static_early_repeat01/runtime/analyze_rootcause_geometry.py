#!/usr/bin/env python3
"""Small offline MINCO sensitivity study from frozen canonical records.

No optimizer, ROS, collisions, or actuation. The linear equations are checked
against recorded native coefficients before counterfactual curves are used.
"""
import copy
import hashlib
import json
import math
from pathlib import Path

import numpy as np

import report_wheel_cause as wheel
from backend_base_feasibility_breakdown import piece_nodes, point_costs, CHANNELS

HERE = Path(__file__).resolve().parent
RESULTS = HERE / 'results'
RAW = RESULTS / 'endpoint_time_20261030_01/midcross_wheel_seed11/candidate_audit/endpoint_time_replay.jsonl'


def basis(t, order):
    return np.asarray([0. if k < order else math.factorial(k)/math.factorial(k-order)*t**(k-order)
                       for k in range(8)])


def generate(times, inner, head, tail):
    """Native MinSnap 8N linear equations, solved independently by NumPy."""
    n = len(times)
    a = np.zeros((8*n, 8*n))
    b = np.zeros((8*n, head.shape[0]))
    for d in range(4):
        a[d, :8] = basis(0., d)
        b[d] = head[:, d]
    for i, duration in enumerate(times[:-1]):
        for offset, d in enumerate((4, 5, 6)):
            row = 8*i+4+offset
            a[row, 8*i:8*i+8] = basis(duration, d)
            a[row, 8*i+8:8*i+16] = -basis(0., d)
        a[8*i+7, 8*i:8*i+8] = basis(duration, 0)
        b[8*i+7] = inner[:, i]
        for d in range(4):
            row = 8*i+8+d
            a[row, 8*i:8*i+8] = basis(duration, d)
            a[row, 8*i+8:8*i+16] = -basis(0., d)
    for d in range(4):
        a[8*n-4+d, -8:] = basis(times[-1], d)
        b[8*n-4+d] = tail[:, d]
    return np.linalg.solve(a, b)


def data(part):
    c = np.asarray(part['b_rows_ascending_power'])
    ts = np.asarray(part['times'])
    inner = np.column_stack([wheel.pvaj(c[8*i:8*i+8], ts[i])[:, 0]
                             for i in range(len(ts)-1)])
    return dict(times=ts, inner=inner,
                head=np.asarray(part['initial_state']), tail=np.asarray(part['final_state']))


def rebuilt_part(part, fields):
    result = copy.deepcopy(part)
    result['times'] = fields['times'].tolist()
    result['initial_state'] = fields['head'].tolist()
    result['final_state'] = fields['tail'].tolist()
    result['b_rows_ascending_power'] = generate(**fields).tolist()
    return result


def first_metrics(part, limits, sample_t=.29):
    stage = dict(parts=[part])
    times = np.arange(.05, sum(part['times']), .001)
    rows = [wheel.wheel_at(stage, float(t), limits) for t in times]
    peak = max(range(len(rows)), key=lambda i: abs(rows[i]['left_alpha']))
    params = [100., .07*4.1, .05, .37, .07, 4.1, 4.25]
    costs = np.zeros(6)
    c = np.asarray(part['b_rows_ascending_power'])
    for k, t in enumerate(part['times']):
        nodes = piece_nodes(c[8*k:8*k+8], t, 16, 16)
        for kind in ('primary', 'dense'):
            v, a, j, weight = nodes[kind]
            if len(v):
                costs += np.sum(weight[:, None]*point_costs(v, a, j, params,
                                                          part['gear'], kind == 'dense'), axis=0)
    return dict(at_029=wheel.wheel_at(stage, sample_t, limits),
                first_segment_left_alpha_peak_1ms=rows[peak]['left_alpha'],
                first_segment_left_alpha_peak_t=float(times[peak]),
                first_segment_duration_s=sum(part['times']),
                base_feasibility_costs=dict(zip(CHANNELS, costs.tolist())))


def native_wheel_grid(stage, limits):
    maxima = np.zeros(4)
    locations = {}
    minimum_speed = float('inf')
    minimum_speed_location = None
    t = .05
    total = sum(sum(p['times']) for p in stage['parts'])
    for unused in range(5, math.floor(total/.01)):
        q = wheel.wheel_at(stage, float(t), limits)
        values = np.asarray([abs(q[k]) for k in wheel.WHEELS])
        for k in np.flatnonzero(values > maxima):
            locations[wheel.WHEELS[k]] = dict(t=float(t), **q)
        maxima = np.maximum(maxima, values)
        if q['raw_speed_m_s'] < minimum_speed:
            minimum_speed, minimum_speed_location = q['raw_speed_m_s'], dict(t=float(t), **q)
        t += .01
    bounds = np.asarray([limits['wheel_limit']]*2+[limits['alpha_limit']]*2)*(1+limits['feas_tol'])
    return dict(maxima=dict(zip(wheel.WHEELS, maxima.tolist())),
                ratios=(maxima/bounds).tolist(), wheel_only_pass=bool(np.all(maxima <= bounds)),
                locations=locations, minimum_speed=minimum_speed,
                minimum_speed_location=minimum_speed_location,
                full_native_collision_or_delivery_verdict=False)


def main():
    runs = [wheel.loads_preserving_cpp_zero(line) for line in RAW.read_text().splitlines()]
    runs = [r for r in runs if 'stages' in r]
    r = next(r for r in runs if r['label'] == 'endpoint150_060')
    initial, final = (r['stages'][i]['parts'][0] for i in (0, -1))
    a, b = data(initial), data(final)
    coefficient_proof = []
    for run in runs:
        for st in run['stages']:
            for k, p in enumerate(st['parts']):
                generated = generate(**data(p))
                recorded = np.asarray(p['b_rows_ascending_power'])
                error = float(np.max(np.abs(generated-recorded)))
                normalized = error/max(1., float(np.max(np.abs(recorded))))
                coefficient_proof.append(dict(run=run['label'], stage=st['label'], segment=k,
                    max_absolute_coefficient_error=error, normalized_error=normalized,
                    passed=normalized < 1e-9))
    assert all(p['passed'] for p in coefficient_proof), coefficient_proof
    changes = ('times', 'inner', 'tail_position', 'tail_velocity')
    trials = []
    for name in ('initial', 'final')+changes:
        fields = copy.deepcopy(b if name == 'final' else a)
        if name in ('times', 'inner'):
            fields[name] = b[name].copy()
        if name.startswith('tail_'):
            column = 0 if name == 'tail_position' else 1
            fields['tail'][:, column] = b['tail'][:, column]
        p = rebuilt_part(initial, fields)
        trials.append(dict(changed=name, **first_metrics(p, r['limits'])))
    direction = a['head'][:2, 1]/np.linalg.norm(a['head'][:2, 1])
    perpendicular = np.asarray([-direction[1], direction[0]])
    movement = {}
    for key, delta in [('inner', b['inner'][:2, 0]-a['inner'][:2, 0]),
                       ('tail_position', b['tail'][:2, 0]-a['tail'][:2, 0])]:
        movement[key] = dict(delta_m=delta.tolist(), length_m=float(np.linalg.norm(delta)),
            along_initial_heading_m=float(delta@direction), perpendicular_m=float(delta@perpendicular))
    movement['tail_heading_delta_rad'] = float(math.atan2(b['tail'][1, 1], b['tail'][0, 1])-
                                             math.atan2(a['tail'][1, 1], a['tail'][0, 1]))
    scale_trials = []
    for stage_name, st in [('initial', r['stages'][0]), ('final', r['stages'][-1])]:
        rebuilt = dict(parts=[])
        for p in st['parts']:
            fields = data(p)
            fields['times'] *= 1.5
            rebuilt['parts'].append(rebuilt_part(p, fields))
        scale_trials.append(dict(stage=stage_name, all_piece_time_factor=1.5,
            total_duration_s=sum(sum(p['times']) for p in rebuilt['parts']),
            **native_wheel_grid(rebuilt, r['limits'])))
    out = RESULTS/'root_cause_20261030_01'
    out.mkdir(exist_ok=True)
    result = dict(scope='offline first-segment MINCO variable replacement; no optimizer or collisions',
        source=str(RAW), source_sha256=hashlib.sha256(RAW.read_bytes()).hexdigest(),
        coefficient_proof=coefficient_proof, variable_movements=movement,
        one_at_a_time_first_segment_trials=trials,
        all_piece_time_counterfactuals=scale_trials)
    (out/'offline_geometry_analysis.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps(dict(coefficient_checks=all(x['passed'] for x in coefficient_proof),
        movements=movement, trials=[dict(changed=t['changed'], alpha029=t['at_029']['left_alpha'],
          alpha029_yaw=t['at_029']['alpha'], tangential=t['at_029']['signed_tangential_acceleration'],
          peak=t['first_segment_left_alpha_peak_1ms'], costs=t['base_feasibility_costs']) for t in trials],
        scale_trials=scale_trials), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
