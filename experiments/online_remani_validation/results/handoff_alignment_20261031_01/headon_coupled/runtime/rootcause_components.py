#!/usr/bin/env python3
"""Current frozen candidate audit using an already compiled native MINCO probe.

Only the smooth, time and base-feasibility components are replayed.  The probe
binds a synthetic empty projected field because its ignored fourth component
requires a field.  This is not a native 3-D collision or online solver replay.
"""
import argparse
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from audit_delivery_iterates import loads_preserving_cpp_zero, matrix
from backend_component_probe import Probe, compare_canonical, extract, function
from backend_base_feasibility_breakdown import CHANNELS, piece_nodes, point_costs, state_values

HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[1]
OLD_BUILD = HERE/'builds/feasibility_stop_20261026_01'
CURRENT_BUILD = HERE/'builds/endpoint_time_20261030_01'
PROBE_DIR = HERE/'results/backend_component_probe_build_20261026_02'
TRIAL = HERE/'results/endpoint_time_20261030_01/midcross_wheel_seed11'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def attest():
    old_manifest = json.loads((OLD_BUILD/'build_manifest.json').read_text())
    current_manifest = json.loads((CURRENT_BUILD/'build_manifest.json').read_text())
    compiled = json.loads((PROBE_DIR/'manifest.json').read_text())
    for directory, manifest in ((OLD_BUILD, old_manifest), (CURRENT_BUILD, current_manifest)):
        source = directory/'source/poly_traj_optimizer.cpp'
        if sha(source) != manifest['isolated_source_sha256']['poly_traj_optimizer.cpp']:
            raise ValueError('Optimizer source hash changed: '+str(source))
    if sha(PROBE_DIR/'backend_component_probe.so') != compiled['library_sha256']:
        raise ValueError('Existing probe library hash changed')
    old = (OLD_BUILD/'source/poly_traj_optimizer.cpp').read_text()
    current = (CURRENT_BUILD/'source/poly_traj_optimizer.cpp').read_text()
    bodies = {}
    for name in ('smoothedL1', 'smoothedLog', 'feasibilityGradCostCar'):
        a, b = function(old, name), function(current, name)
        if a != b: raise ValueError('Probe body mismatch: '+name)
        bodies[name] = hashlib.sha256(b.encode()).hexdigest()
    for name in ('VirtualT2RealT', 'VirtualTGradCost', 'addPVAJGradCost2CT'):
        needle = 'void PolyTrajOptimizer::'+name+'('
        a, b = extract(old, needle, True), extract(current, needle, True)
        if a != b: raise ValueError('Probe template mismatch: '+name)
        bodies[name] = hashlib.sha256(b.encode()).hexdigest()
    minco = WORKSPACE/'src/mm_remani_planner/traj_utils/include/traj_utils/poly_traj_utils.hpp'
    rel = str(minco.relative_to(WORKSPACE))
    if sha(minco) != compiled['sources_sha256']['poly_traj_utils.hpp'] or sha(minco) != current_manifest['production_sources_sha256'][rel]:
        raise ValueError('MINCO source changed')
    return dict(library=str(PROBE_DIR/'backend_component_probe.so'), library_sha256=compiled['library_sha256'],
        source_functions_sha256=bodies, current_build_manifest_sha256=sha(CURRENT_BUILD/'build_manifest.json'),
        probe_manifest_sha256=sha(PROBE_DIR/'manifest.json'), no_new_compilation=True)


def context(run, loaded):
    first = run['stages'][0]
    prefix = '/remani_planner_node/'
    limits = run['limits']
    pieces = np.asarray([len(p['times']) for p in first['parts']], dtype=np.int32)
    dim = 8
    count = int(sum(pieces))
    np_count = int(sum(dim*(pieces-1)))
    nx = len(first['x'])
    # The recorded live limits override the prelaunch dump (omega=3.28 there).
    params = np.asarray([loaded[prefix+'optimization/weight_base_feasibility'],
        limits['wheel_limit']*limits['wheel_radius'], loaded[prefix+'mm/mobile_base_non_singul_vel'],
        limits['wheel_base'], limits['wheel_radius'], limits['wheel_limit'], limits['alpha_limit'],
        loaded[prefix+'optimization/weight_time'], limits['quadrature_points'], limits['dense_resolution'],
        int(loaded[prefix+'optimization/mobile_base_opt_gear'])], dtype=float)
    free = [i for i in range(np_count) if i%8 < 2]
    free += list(range(np_count, np_count+count))
    free += [np_count+count+8*t+d for t in range(len(pieces)-1) for d in (0, 1)]
    free += list(range(np_count+count+8*(len(pieces)-1), nx))
    # Broad positive, static field, used only by the ignored projected component.
    snapshot = np.asarray([1, 1, 1, 2, 2, -1000, -1000, 2000, .1, .12, 1,
                           1, 0, .35, 3, 1, 0, 0, .01, 1000, 1000, 1000, 1000], dtype=float)
    return SimpleNamespace(pieces=pieces, gears=np.asarray(run['directions'], dtype=np.int32),
        ini=np.concatenate([matrix(v).ravel(order='F') for v in run['effective_initial_states']]),
        fin=np.concatenate([matrix(v).ravel(order='F') for v in run['effective_final_states']]),
        params=params, snapshot=snapshot, epoch=1., nx=nx, free_indices=np.asarray(sorted(free), dtype=int),
        metadata=dict(real_3d_map_replayed=False, projected_component_ignored=True),
        groups=dict(point_xy=[i for i in range(np_count) if i%8 < 2],
            time=list(range(np_count, np_count+count)),
            gear_xy=[np_count+count+8*t+d for t in range(len(pieces)-1) for d in (0, 1)],
            gear_angle=list(range(np_count+count+8*(len(pieces)-1), nx))))


def channels(value, ctx):
    total = np.zeros(6)
    for coeff, times, gear in zip(value['b_parts'], value['times_parts'], ctx.gears):
        for piece, duration in enumerate(times):
            nodes = piece_nodes(coeff[piece*8:(piece+1)*8], duration, int(ctx.params[8]), int(ctx.params[9]))
            for kind in ('primary', 'dense'):
                v, a, j, weights = nodes[kind]
                if len(v): total += np.sum(weights[:, None]*point_costs(v, a, j, ctx.params, int(gear), kind == 'dense'), axis=0)
    return dict(zip(CHANNELS, total.tolist()))


def numeric_gradient(probe, ctx, x, multiplier):
    gradients = {name: np.zeros(len(x)) for name in ('smooth', 'time', 'wheel', 'low_speed', 'wheel_only')}
    for index in ctx.free_indices:
        h = multiplier*max(1., abs(x[index]))
        values = []
        for factor in (-2., -1., 1., 2.):
            xx = x.copy(); xx[index] += factor*h
            v = probe.evaluate(xx, ctx); branch = channels(v, ctx)
            costs = {k: v['costs'][k] for k in ('smooth', 'time', 'wheel')}
            costs['low_speed'] = branch['dense_min_speed']
            costs['wheel_only'] = costs['wheel']-costs['low_speed']
            values.append(costs)
        for name in gradients:
            gradients[name][index] = (8*(values[2][name]-values[1][name])-(values[3][name]-values[0][name]))/(12*h)
    return gradients


def wheel_state(value, ctx, t):
    remaining = float(t)
    for block, (coeff, times, gear) in enumerate(zip(value['b_parts'], value['times_parts'], ctx.gears)):
        for piece, duration in enumerate(times):
            if remaining <= duration or (block+1 == len(ctx.pieces) and piece+1 == len(times)):
                v, a, j = [s[0] for s in state_values(coeff[piece*8:(piece+1)*8], [remaining])]
                speed = max(1e-4, float(np.linalg.norm(v))); v2 = max(1e-8, float(v@v))
                va = float(v@a); cross = float(v[0]*a[1]-v[1]*a[0]); jc = float(v[0]*j[1]-v[1]*j[0])
                w = cross/v2; alpha = jc/v2-2*cross*va/v2**2
                r, b = ctx.params[4], ctx.params[3]
                return np.asarray([(gear*speed-b*w/2)/r, (gear*speed+b*w/2)/r,
                    (gear*va/speed-b*alpha/2)/r, (gear*va/speed+b*alpha/2)/r])
            remaining -= duration
    raise ValueError('Time outside candidate')


def hard_peak(value, ctx):
    duration = float(sum(sum(t) for t in value['times_parts']))
    grid = np.arange(.05, int(np.floor(duration/.01))*.01, .01)
    values = np.asarray([wheel_state(value, ctx, t) for t in grid])
    limits = np.asarray([ctx.params[5]]*2+[ctx.params[6]]*2)*1.05
    peaks = np.max(abs(values), axis=0)
    return dict(maxima=peaks.tolist(), ratios=(peaks/limits).tolist(),
        times=grid[np.argmax(abs(values), axis=0)].tolist(), wheel_gate_pass=bool(np.all(peaks <= limits)),
        native_collision_gate_replayed=False)


def cos(a, b):
    n = float(np.linalg.norm(a)*np.linalg.norm(b))
    return None if n < 1e-20 else float(a@b/n)


def encode_time(t):
    t = np.asarray(t)
    result = np.empty_like(t)
    mask = t > 1
    result[mask] = np.sqrt(2*t[mask]-1)-1
    result[~mask] = 1-np.sqrt(2/t[~mask]-1)
    return result


def run_audit(trial, output):
    evidence = attest()
    raw = trial/'candidate_audit/endpoint_time_replay.jsonl'
    records = [loads_preserving_cpp_zero(l) for l in raw.open()]
    completion = next(r for r in records if r['kind'] == 'complete')
    if not all(completion[k] for k in ('all_captures_complete', 'environment_stable', 'capture_original_same_first_x_f_g')):
        raise ValueError('Original frozen capture failed')
    loaded = json.loads((trial/'effective_real_params.json').read_text())
    probe = Probe(evidence['library'])
    report = dict(schema='rootcause_component_audit_v1', trial=str(trial), raw_sha256=sha(raw),
        attestation=evidence, stages=[], full_native_3d_replay=False, online_solver_run=False,
        source_sha256=sha(__file__))
    for run in (r for r in records if r['kind'] == 'run'):
        ctx = context(run, loaded)
        for stage in run['stages']:
            x = np.asarray(stage['x'], dtype=float)
            value = probe.evaluate(x, ctx)
            expected = dict(canonical_parts=[dict(times=p['times'], b_rows_ascending_power=p['b_rows_ascending_power']) for p in stage['parts']], x=stage['x'])
            agreement = compare_canonical(value, expected)
            branch = channels(value, ctx)
            component_errors = {k: abs(value['costs'][k]-stage['cost_components'][{'wheel':'base_feasibility'}.get(k, k)])/max(1., abs(value['costs'][k])) for k in ('smooth', 'time', 'wheel')}
            low_agreement = abs(sum(branch.values())-value['costs']['wheel'])/max(1., abs(value['costs']['wheel']))
            if not agreement['verified'] or max(component_errors.values()) > 1e-7 or low_agreement > 1e-7:
                raise ValueError('Native reconstruction/cost mismatch')
            numeric = numeric_gradient(probe, ctx, x, 3e-6)
            check = numeric_gradient(probe, ctx, x, 8e-7)
            gradients = {k: np.asarray(value['gradients'][k]) for k in ('smooth', 'time', 'wheel')}
            gradients.update(low_speed=numeric['low_speed'], wheel_only=numeric['wheel_only'])
            errors = {k: float(np.max(abs(gradients[k][ctx.free_indices]-numeric[k][ctx.free_indices])/np.maximum(1., np.maximum(abs(gradients[k][ctx.free_indices]), abs(numeric[k][ctx.free_indices]))))) for k in ('smooth', 'time', 'wheel')}
            gradient_details = {}
            for name in ('smooth', 'time', 'wheel'):
                indices = sorted(ctx.free_indices, key=lambda i: abs(gradients[name][i]-numeric[name][i])/max(1., abs(gradients[name][i]), abs(numeric[name][i])), reverse=True)[:10]
                gradient_details[name] = [dict(index=int(i), analytic=float(gradients[name][i]),
                    numeric_primary=float(numeric[name][i]), numeric_second=float(check[name][i]),
                    normalized_error=float(abs(gradients[name][i]-numeric[name][i])/max(1., abs(gradients[name][i]), abs(numeric[name][i])))) for i in indices]
            low_convergence = float(np.max(abs(numeric['low_speed'][ctx.free_indices]-check['low_speed'][ctx.free_indices])/np.maximum(1., np.maximum(abs(numeric['low_speed'][ctx.free_indices]), abs(check['low_speed'][ctx.free_indices])))))
            self_per_second = stage['cost_components']['self_collision']/value['costs']['time']
            gradients['time_plus_fixed_arm_self'] = gradients['time']*(1+self_per_second)
            gradients['known_full_local'] = gradients['smooth']+gradients['wheel']+gradients['time_plus_fixed_arm_self']
            peak = hard_peak(value, ctx)
            # Follow the final endpoint treatment's left-acceleration peak.
            peak_time = peak['times'][2]
            sign = float(np.sign(wheel_state(value, ctx, peak_time)[2]))
            peak_grad = np.zeros(len(x))
            for index in ctx.free_indices:
                h = 8e-7*max(1., abs(x[index])); xx = x.copy(); xx[index] += h
                yy = x.copy(); yy[index] -= h
                peak_grad[index] = sign*(wheel_state(probe.evaluate(xx, ctx), ctx, peak_time)[2]-wheel_state(probe.evaluate(yy, ctx), ctx, peak_time)[2])/(2*h)
            directions = []
            for name, grad in gradients.items():
                norm = float(np.linalg.norm(grad)); direction = -grad/max(norm, 1e-30)
                item = dict(name=name, gradient_norm=norm, cosine_with_left_alpha_peak_gradient=cos(grad, peak_grad),
                    left_alpha_peak_derivative_along_unit_descent=float(peak_grad@direction), groups={k:float(np.linalg.norm(grad[idx])) for k,idx in ctx.groups.items()}, steps=[])
                for step in (1e-5, 1e-4):
                    changed = probe.evaluate(x+step*direction, ctx); bc = channels(changed, ctx)
                    item['steps'].append(dict(step=step, smooth=changed['costs']['smooth'],
                        base_feasibility=changed['costs']['wheel'], low_speed=bc['dense_min_speed'],
                        wheel_without_low=changed['costs']['wheel']-bc['dense_min_speed'],
                        time=changed['costs']['time'], left_alpha_abs_at_original_peak=abs(float(wheel_state(changed, ctx, peak_time)[2]))))
                    if run['label'] == 'endpoint150_060' and stage['label'] == 'accepted':
                        item['steps'][-1]['all_native_motion_peaks'] = hard_peak(changed, ctx)
                directions.append(item)
            coordinate_names = {idx:name for name,indices in ctx.groups.items() for idx in indices}
            top = sorted(ctx.free_indices, key=lambda i: abs(peak_grad[i]), reverse=True)[:10]
            item = dict(run=run['label'], stage=stage['label'], iteration=stage['iteration'],
                component_costs=value['costs'], native_recorded_components=stage['cost_components'],
                base_channels=branch, canonical_agreement=agreement, component_errors=component_errors,
                low_sum_error=low_agreement, native_gradient_vs_fd=errors, gradient_fd_details=gradient_details,
                low_gradient_fd_two_step_error=low_convergence,
                fixed_arm_self_per_second=self_per_second, native_peak=peak, peak_gradient_time_s=peak_time,
                top_left_alpha_coordinates=[dict(index=int(i),group=coordinate_names[i], derivative=float(peak_grad[i])) for i in top],
                component_gradients={k:v.tolist() for k,v in gradients.items()}, directions=directions,
                limited_scope='All component gradients use the captured fixed PVAJ, gears and native quadrature; self term is inferred constant per second on this fixed-arm input. Native collision is not replayed.')
            report['stages'].append(item)
            print(run['label'], stage['label'], 'gradient_errors', errors, 'low_fd_error', low_convergence, flush=True)
            if run['label'] == 'endpoint150_060' and stage['label'] == 'accepted':
                scans = []
                first_time = sum(8*(ctx.pieces-1)); current_times = np.concatenate(value['times_parts'])
                for factor in (1., 1.03, 1.1, 1.25, 1.5):
                    xx = x.copy(); xx[first_time:first_time+len(current_times)] = encode_time(current_times*factor)
                    vv = probe.evaluate(xx, ctx); bc = channels(vv, ctx)
                    scans.append(dict(factor=factor, duration_s=float(sum(sum(t) for t in vv['times_parts'])),
                        costs={**vv['costs'], **bc}, native_motion_peak=hard_peak(vv, ctx)))
                report['final_time_scan'] = scans
    report['evidence_valid'] = all(s['canonical_agreement']['verified'] and max(s['component_errors'].values()) < 1e-7 and s['low_sum_error'] < 1e-7 for s in report['stages'])
    output.mkdir(parents=True, exist_ok=False)
    (output/'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(output/'summary.json', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--trial', type=Path, default=TRIAL)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    run_audit(args.trial.resolve(), args.out.resolve())
