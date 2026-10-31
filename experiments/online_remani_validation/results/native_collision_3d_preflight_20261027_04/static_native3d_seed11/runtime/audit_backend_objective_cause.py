#!/usr/bin/env python3
"""Posterior attribution of frozen backend objectives; never runs a planner.

The unreplayed remainder is NOT a reconstruction of the native 3-D map cost.
Neither finite differences nor soft-cost reduction certify a usable trajectory.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from audit_delivery_fullcurve import frozen_frame
from audit_delivery_iterates import canonical_candidate, canonical_records, vector_hash
from recheck_delivery_diag import evaluate_pose
from report_domain_recovery import digest, read_json

HERE = Path(__file__).resolve().parent
PHASES = ('delivery_nominal_060', 'delivery_stop_on_060', 'delivery_repeat_060')
COMPONENTS = ('smooth', 'time', 'wheel', 'projected_static')
STEPS = (1e-5, 3e-6, 1e-6)
FINE_STEPS = (1e-7, 3e-8, 1e-8)
FD_TOLERANCE = 1e-4


def bilinear(frame, centers):
    centers = np.asarray(centers, dtype=float)
    grid = (centers - [frame['gx'], frame['gy']]) / frame['res']
    if (not np.all(np.isfinite(grid)) or np.any(grid < 0) or
            np.any(grid[:, 0] >= frame['nx']-1) or np.any(grid[:, 1] >= frame['ny']-1)):
        raise ValueError('Outside recorded projected map')
    ij = np.floor(grid).astype(int)
    a, b = (grid-ij).T
    ix, iy = ij.T
    field = frame['field']
    q00, q10, q01, q11 = field[iy, ix], field[iy, ix+1], field[iy+1, ix], field[iy+1, ix+1]
    d = (1-b)*((1-a)*q00+a*q10) + b*((1-a)*q01+a*q11)
    dx = ((1-b)*(q10-q00)+b*(q11-q01))/frame['res']
    dy = ((1-a)*(q01-q00)+a*(q11-q10))/frame['res']
    return d, np.column_stack((dx, dy)), ij


def projected_quadrature(candidate, frame):
    """Independent Python K8 cost and local-map coverage, not native 3-D cost."""
    body = frame['body']; count = len(body)
    costs = np.zeros(count); min_gaps = np.full(count, np.inf)
    active_count = unsafe_count = zero_unsafe = 0
    min_gradient = math.inf
    cells = []; worst = None; duration = 0.; knots=[]
    for part_index, part in enumerate(candidate['parts']):
        for piece_index, piece in enumerate(part['pieces']):
            c = np.asarray(piece['coeffs'], dtype=float)[:, ::-1]
            T = float(piece['T']); duration += T
            for k in range(9):
                t = T*k/8.
                p = np.array([np.polynomial.polynomial.polyval(t, row) for row in c])
                v = np.array([np.polynomial.polynomial.polyval(t, np.polynomial.polynomial.polyder(row)) for row in c])
                yaw = math.atan2(part['gear']*v[1], part['gear']*v[0])
                co, si = math.cos(yaw), math.sin(yaw)
                centers = np.column_stack((p[0]+co*body[:, 0]-si*body[:, 1],
                                           p[1]+si*body[:, 0]+co*body[:, 1]))
                d, grad, ij = bilinear(frame, centers)
                physical = d-body[:, 2]; gap = physical-frame['margin']
                errors = np.maximum(0., frame['comfort']-physical)
                costs += (.5 if k in (0, 8) else 1.)*T/8.*frame['weight']*errors**3
                min_gaps = np.minimum(min_gaps, gap)
                unsafe = gap < -1e-8; norms = np.linalg.norm(grad, axis=1)
                unsafe_count += int(np.sum(unsafe)); active_count += int(np.sum(errors > 0.))
                zero_unsafe += int(np.sum(unsafe & (norms <= 1e-12)))
                if np.any(unsafe): min_gradient = min(min_gradient, float(np.min(norms[unsafe])))
                bi = int(np.argmin(gap))
                if worst is None or gap[bi] < worst['gap_m']:
                    worst = dict(part=part_index, piece=piece_index, sample=k, body=bi,
                        local_t=t, gap_m=float(gap[bi]), physical_m=float(physical[bi]),
                        field_gradient=grad[bi].tolist(), position=p[:2].tolist(), yaw=yaw)
                if k in (0,8):
                    knots.append(dict(part=part_index,piece=piece_index,endpoint=k,
                        min_gap_m=float(np.min(gap)),per_disk_gap_m=gap.tolist(),
                        position=p[:2].tolist(),yaw=yaw))
                cells.extend(ij.ravel().tolist())
    return dict(cost=float(np.sum(costs)), per_disk_integrated_cost=costs.tolist(),
        per_disk_K8_min_gap_m=min_gaps.tolist(), duration_s=duration,
        mean_cost_per_second=float(np.sum(costs)/duration), active_disk_nodes=active_count,
        unsafe_disk_nodes=unsafe_count, zero_map_gradient_unsafe_nodes=zero_unsafe,
        min_unsafe_map_gradient_norm=None if math.isinf(min_gradient) else min_gradient,
        worst=worst, piece_endpoints_including_duplicate_knots=knots,grid_cell_signature=cells,
        scope='same conservative planar projection, not independent physical ground truth')


def relative_error(a, b):
    return float(np.max(np.abs(a-b)/np.maximum(1., np.maximum(np.abs(a), np.abs(b)))))


def cosine(a, b):
    a, b = np.asarray(a), np.asarray(b)
    denominator = np.linalg.norm(a)*np.linalg.norm(b)
    return float(np.dot(a, b)/denominator) if denominator > 1e-20 else None


def finite_difference(probe, context, x, baseline, steps=STEPS):
    free = [int(i) for i in baseline['free_indices']]
    rows = []
    for multiplier in steps:
        numeric = {name:np.zeros(len(x)) for name in COMPONENTS}
        for index in free:
            h = multiplier*max(1., abs(x[index])); axis = np.zeros(len(x)); axis[index] = h
            values = [probe.evaluate(x+offset*axis, context)['costs'] for offset in (-2., -1., 1., 2.)]
            for name in COMPONENTS:
                numeric[name][index] = (8.*(values[2][name]-values[1][name])-
                                         (values[3][name]-values[0][name]))/(12.*h)
        parts = {}
        for name in COMPONENTS:
            analytic = np.asarray(baseline['gradients'][name], dtype=float)
            errors = np.abs(analytic[free]-numeric[name][free]) / np.maximum(
                1., np.maximum(np.abs(analytic[free]), np.abs(numeric[name][free])))
            worst = int(np.argmax(errors))
            parts[name] = dict(max_normalized_error=float(errors[worst]),
                worst_coordinate=free[worst], analytic=analytic.tolist(), numeric=numeric[name].tolist(),
                max_absolute_error=float(np.max(np.abs(analytic[free]-numeric[name][free]))),
                passed=bool(np.max(errors) <= FD_TOLERANCE))
        rows.append(dict(multiplier=multiplier, components=parts))
    primary = rows[1]
    return dict(steps=list(steps), primary_step=steps[1], tolerance=FD_TOLERANCE,
        free_coordinates=free, differences=rows,
        primary_all_pass=all(v['passed'] for v in primary['components'].values()),
        not_a_complete_native_objective_gradient_audit=True)


def compare_canonical(record, result):
    expected = record['canonical_parts']
    if len(expected) != len(result['b_parts']): raise ValueError('MINCO block count mismatch')
    errors = []; times = []; bits = []
    for old, b, t in zip(expected, result['b_parts'], result['times_parts']):
        target = np.asarray(old['b_rows_ascending_power'], dtype=float)
        b = np.asarray(b, dtype=float); rt = np.asarray(t, dtype=float)
        if b.shape != target.shape or rt.shape != np.asarray(old['times']).shape:
            raise ValueError('Canonical coefficient/time shape mismatch')
        errors.append(relative_error(target, b))
        times.append(relative_error(np.asarray(old['times']), rt))
        bits.append(bool(np.array_equal(target.view(np.uint64), b.view(np.uint64)) and
            np.array_equal(np.asarray(old['times'],dtype=float).view(np.uint64),rt.view(np.uint64))))
    maximum = max(errors+times)
    return dict(coefficient_max_normalized_error=max(errors), time_max_normalized_error=max(times),
        all_coefficients_and_times_bitwise_equal=all(bits), tolerance=1e-8, passed=maximum <= 1e-8)


def gradient_attribution(record, values):
    total = np.asarray(record['g'], dtype=float)
    parts = {name:np.asarray(values['gradients'][name], dtype=float) for name in COMPONENTS}
    parts['unreplayed_native_remainder'] = total-sum(parts.values())
    groups = values.get('coord_groups', {'all_free':values['free_indices']})
    result = {}
    for group, indices in groups.items():
        selected = {name:v[indices] for name, v in parts.items()}
        result[group] = dict(norms={name:float(np.linalg.norm(v)) for name, v in selected.items()},
            total_norm=float(np.linalg.norm(total[indices])),
            cosine_with_projected={name:cosine(v, selected['projected_static']) for name, v in selected.items()},
            total_descent_projected_derivative=float(-np.dot(total[indices], selected['projected_static'])))
    return result, {name:v.tolist() for name, v in parts.items()}


def build_report(root, prior_path, out, probe, probe_manifest):
    prior = read_json(prior_path)
    if prior.get('evidence_valid') is not True: raise ValueError('Prior audit invalid')
    if prior.get('source') != str(root): raise ValueError('Prior audit belongs to a different batch')
    report = dict(schema='backend_objective_cause_v1', source=str(root),
        prior_report=str(prior_path), prior_report_sha256=digest(prior_path),
        analysis_source_sha256=digest(Path(__file__)), probe=probe_manifest, trials=[],
        changes_to_production=False, new_ros_launches=0, full_native_collision_replay=False,
        usable_trajectory_improvement_claimed=False)
    from backend_component_probe import Context
    from backend_base_feasibility_breakdown import breakdown
    for old in prior['trials']:
        trial = root/old['trial']; rows, raw_path = canonical_records(trial)
        if digest(raw_path) != old['input_sha256'][raw_path.name]: raise ValueError('Raw capture changed')
        capture = next(r for r in rows if r['kind']=='capture')
        frame, proof = frozen_frame(trial/'delivery_diagnostics', capture['frame'])
        if len(frame['obstacles']): raise ValueError('This protocol is static only')
        cases = []; test_rows = []
        for phase in PHASES:
            context = Context.from_trial(trial, phase)
            old_phase = next(p for p in old['phases'] if p['phase']==phase)
            old_candidates = old_phase['iterate_audit']['candidates']
            records = [r for r in rows if r.get('kind')=='canonical_candidate' and r.get('phase')==phase]
            if len(records) != len(old_candidates): raise ValueError('Candidate count changed')
            values = []; last_record = None; last_grad = None
            for index, (record, old_record) in enumerate(zip(records, old_candidates)):
                if (vector_hash(record['x']) != old_record['candidate_x_hash'] or
                        vector_hash(record['g']) != old_record['candidate_g_hash']):
                    raise ValueError('Canonical x/g hash mismatch')
                value = probe.evaluate(np.asarray(record['x'], dtype=float), context)
                value['coord_groups'] = context.coord_groups
                agreement = compare_canonical(record, value)
                candidate = canonical_candidate(record, capture['frame']['now'])
                geometry = projected_quadrature(candidate, frame)
                projection_error = abs(geometry['cost']-value['costs']['projected_static'])/max(1., abs(geometry['cost']))
                if not agreement['passed'] or projection_error > 1e-8:
                    raise ValueError('Polynomial or independent projection reconstruction mismatch')
                attrib, gradients = gradient_attribution(record, value)
                base_parts=breakdown(value,context)
                if not base_parts['sum_agrees_with_native_probe']:
                    raise ValueError('Independent base-feasibility subcosts disagree')
                base_parts.pop('source_proof')
                costs = dict(value['costs'])
                costs['unreplayed_native_remainder'] = record['cost']-sum(costs.values())
                free = value['free_indices']
                head = next(iter(candidate['parts']))['pieces'][0]
                last = candidate['parts'][-1]['pieces'][-1]
                from recheck_delivery_diag import polynomial
                endpoints = []
                for poly, at, gear in ((head,0.,candidate['parts'][0]['gear']),
                                       (last,last['T'],candidate['parts'][-1]['gear'])):
                    p, v = polynomial(poly, at)
                    endpoints.append(evaluate_pose(frame,p[0],p[1],math.atan2(gear*v[1],gear*v[0]),0.))
                row = dict(stage=record['stage'], iteration=record['iteration'], x_hash=vector_hash(record['x']),
                    g_hash=vector_hash(record['g']), total_cost=record['cost'], costs=costs,
                    canonical_agreement=agreement, projected_cost_relative_error=projection_error,
                    projection={k:v for k,v in geometry.items() if k!='grid_cell_signature'},
                    prior_projected_20ms=old_record['frozen_projected_20ms'], endpoints=endpoints,
                    gradients=gradients, gradient_attribution=attrib,
                    base_feasibility_breakdown=base_parts,
                    native_first_failure_reason=record['native_reason'],
                    native_first_failure_time=record['native_rejection_time'],
                    native_collision_after_failure_unknown=True)
                if last_record is not None:
                    delta = np.asarray(record['x'])-np.asarray(last_record['x'])
                    row['step_from_previous'] = dict(
                        total_cost_change=float(record['cost']-last_record['cost']),
                        component_cost_changes={k:float(costs[k]-values[-1]['costs'][k]) for k in costs},
                        previous_gradient_dot_dx={k:float(np.dot(last_grad[k],delta)) for k in last_grad},
                        free_step_norm=float(np.linalg.norm(delta[free])))
                values.append(row); last_record=record; last_grad=gradients
                chosen = ((phase==PHASES[0] and index==0) or
                          (phase in PHASES[:2] and index==len(records)-1))
                if chosen:
                    fd = finite_difference(probe, context, np.asarray(record['x'], dtype=float), value)
                    fine = finite_difference(probe, context, np.asarray(record['x'], dtype=float), value,FINE_STEPS)
                    test_rows.append(dict(phase=phase, stage=record['stage'], iteration=record['iteration'],
                        x_hash=row['x_hash'], finite_difference=fd,fine_step_diagnostic=fine))
            cases.append(dict(phase=phase, count=len(values), candidates=values,
                measured_solver_reason=old_phase['solver_reason'],
                measured_decision_elapsed_s=old_phase['decision_elapsed_s'],
                last_canonical_wheel_audit=old_phase['last_canonical_wheel_audit']))
        report['trials'].append(dict(trial=trial.name, seed=old['seed'],
            input_sha256={raw_path.name:digest(raw_path),'manifest.json':digest(trial/'manifest.json'),
                'parameters.yaml':digest(trial/'parameters.yaml'),
                'effective_real_params.json':digest(trial/'effective_real_params.json')},
            snapshot=proof, phases=cases, finite_difference_states=test_rows))
    report['all_selected_component_gradients_pass'] = all(s['finite_difference']['primary_all_pass']
        for t in report['trials'] for s in t['finite_difference_states'])
    report['all_selected_fine_component_gradients_pass'] = all(s['fine_step_diagnostic']['primary_all_pass']
        for t in report['trials'] for s in t['finite_difference_states'])
    report['canonical_count'] = sum(p['count'] for t in report['trials'] for p in t['phases'])
    return report


def write_text(report):
    lines=['后端有效轨迹交付：分项目标主因探查（离线，不改生产）','']
    for trial in report['trials']:
        lines.append('seed{}'.format(trial['seed']))
        for phase in trial['phases'][:2]:
            a,b=phase['candidates'][0],phase['candidates'][-1]
            lines.append('  {}: {}个候选；总f {:.6f} -> {:.6f}'.format(phase['phase'],phase['count'],a['total_cost'],b['total_cost']))
            for component in (*COMPONENTS,'unreplayed_native_remainder'):
                lines.append('    {}: {:.6f} -> {:.6f}'.format(component,a['costs'][component],b['costs'][component]))
            lines.append('    wheel键=底盘可行性积分，含低速项，不是仅轮约束。末候选六子项：'+
                json.dumps(b['base_feasibility_breakdown']['costs'],ensure_ascii=False))
            lines.append('    K8 gap {:.3f} -> {:.3f} mm；零地图梯度违规节点 {} -> {}'.format(
                min(a['projection']['per_disk_K8_min_gap_m'])*1000,min(b['projection']['per_disk_K8_min_gap_m'])*1000,
                a['projection']['zero_map_gradient_unsafe_nodes'],b['projection']['zero_map_gradient_unsafe_nodes']))
        for state in trial['finite_difference_states']:
            primary=state['finite_difference']['differences'][1]['components']
            lines.append('  FD {} k{}: {}'.format(state['phase'],state['iteration'],
                '; '.join('{}={:.3g} {}'.format(k,v['max_normalized_error'],'PASS' if v['passed'] else 'FAIL') for k,v in primary.items())))
            fine=state['fine_step_diagnostic']['differences'][1]['components']
            lines.append('    追加细步3e-8: '+ '; '.join('{}={:.3g} {}'.format(
                k,v['max_normalized_error'],'PASS' if v['passed'] else 'FAIL') for k,v in fine.items()))
    lines.extend(['','边界：原生3D地图未dump；未重放余项不是单独的碰撞项。',
        '原生首轮拒绝后的碰撞检查未知；2D保守投影不等同实车物理碰撞。',
        '分项大小/梯度夹角不能单独证明因果；本轮不证明联合可行性修复或在线交付提升。'])
    return '\n'.join(lines)+'\n'


def plot(report, target):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2,2,figsize=(10,6.5),constrained_layout=True)
    colors={'smooth':'#4c78a8','wheel':'#f58518','projected_static':'#e45756',
            'time':'#72b7b2','unreplayed_native_remainder':'#999999'}
    for col,trial in enumerate(report['trials']):
        phase=trial['phases'][1]; rows=phase['candidates']; x=[r['iteration'] for r in rows]
        ax=axes[0,col]
        for name,color in colors.items():
            ax.plot(x,[r['costs'][name] for r in rows],label=name,color=color,lw=1.4)
        ax.set(title='seed {} | unchanged objective, continued solve'.format(trial['seed']),
            xlabel='Accepted iteration (0 = initial)',ylabel='Cost [native objective units]')
        ax.set_yscale('symlog',linthresh=1.)
        ax.grid(alpha=.2); ax.legend(fontsize=7)
        ax=axes[1,col]
        for bi in range(3):
            ax.plot(x,[r['projection']['per_disk_K8_min_gap_m'][bi]*1000 for r in rows],
                label='base {}'.format(bi) if bi<2 else 'fixed-arm projection',lw=1.4)
        ax.axhline(0.,color='black',lw=.8,label='Required margin satisfied')
        ax.set(xlabel='Accepted iteration',ylabel='Min K8 gap = clearance - 100 mm [mm]')
        ax.grid(alpha=.2); ax.legend(fontsize=7)
    fig.savefig(target,dpi=150); plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,default=HERE/'results/feasibility_stop_20261026_01')
    parser.add_argument('--prior',type=Path,default=HERE/'results/feasibility_stop_report_20261026_01/summary.json')
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--probe-build',type=Path,help='Reuse a hash-verified successful offline probe build')
    args=parser.parse_args(); out=args.out.resolve()
    if out.exists(): raise ValueError('Require fresh output directory; never overwrite previous evidence')
    out.mkdir(parents=True)
    protocol=dict(scope='offline posterior component attribution, no ROS or solver',
        inputs=str(args.input.resolve()),prior=str(args.prior.resolve()),steps=STEPS,
        fine_steps=FINE_STEPS,fine_primary_step=FINE_STEPS[1],
        primary_step=STEPS[1],tolerance=FD_TOLERANCE,seeds=[11,12],phases=PHASES,
        fd_selection='each seed: nominal initial, nominal last, ON last',
        script_sha256=digest(Path(__file__)),probe_python_sha256=digest(HERE/'backend_component_probe.py'),
        base_decomposition_sha256=digest(HERE/'backend_base_feasibility_breakdown.py'))
    (out/'protocol.json').write_text(json.dumps(protocol,indent=2,ensure_ascii=False)+'\n')
    from backend_component_probe import build_probe, Probe, source_material
    if args.probe_build:
        build=args.probe_build.resolve(); manifest=read_json(build/'manifest.json')
        library=build/'backend_component_probe.so'
        if (manifest.get('returncode')!=0 or digest(library)!=manifest.get('library_sha256') or
                digest(HERE/'backend_component_probe.py')!=manifest.get('probe_python_sha256') or
                digest(build/'backend_component_probe.cpp')!=manifest.get('generated_cpp_sha256')):
            raise ValueError('Reused probe source/library changed')
        _, current=source_material()
        if current['sources_sha256']!=manifest['sources_sha256']:
            raise ValueError('Probe source attestation differs')
        manifest=dict(manifest,reused_build=str(build),reused_manifest_sha256=digest(build/'manifest.json'))
    else:
        library, manifest=build_probe(out/'probe')
    report=build_report(args.input.resolve(),args.prior.resolve(),out,Probe(library),manifest)
    (out/'summary.json').write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    (out/'分项目标审计.txt').write_text(write_text(report),encoding='utf-8')
    plot(report,out/'objective_components.png')
    print(json.dumps(dict(out=str(out),count=report['canonical_count'],
        all_selected_component_gradients_pass=report['all_selected_component_gradients_pass'],
        all_selected_fine_component_gradients_pass=report['all_selected_fine_component_gradients_pass']),ensure_ascii=False))


if __name__=='__main__': main()
