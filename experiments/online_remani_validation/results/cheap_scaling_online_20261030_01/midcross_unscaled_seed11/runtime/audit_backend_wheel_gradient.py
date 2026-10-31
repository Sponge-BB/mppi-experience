#!/usr/bin/env python3
"""Numerical audit of the actual frozen native car-feasibility function.

No ROS, replanning, retiming or controller changes. The C++ function bodies are
extracted byte-for-byte from the attested build, not transcribed from a paper.
Finite differences test the implemented cost, not physical model correctness.
"""
import argparse
from collections import Counter
import ctypes
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess

import numpy as np

from audit_backend_kinematics import (CHANNELS, derivative, loaded_values,
    parameters as wheel_parameters, wheel_values)
from audit_delivery_iterates import canonical_candidate, canonical_records, vector_hash
from report_domain_recovery import digest, read_json


BASE = Path(__file__).resolve().parent
PARAMETER_NAMES = ('weight', 'max_velocity', 'non_singular_velocity',
                   'wheel_base', 'wheel_radius', 'max_wheel_speed',
                   'max_wheel_acceleration')
STEPS = (2e-5, 6e-6, 2e-6)
TOLERANCE = 5e-5


def function(text, name):
    """Extract one complete out-of-class bool member; braces in these bodies
    are plain C++ code (no string/comment braces). Reject ambiguous functions.
    """
    needle = 'bool PolyTrajOptimizer::' + name + '('
    if text.count(needle) != 1:
        raise ValueError('Expected unique function: ' + name)
    start = text.index(needle)
    opening = text.index('{', start)
    depth = 0
    for end in range(opening, len(text)):
        depth += (text[end] == '{') - (text[end] == '}')
        if depth == 0:
            return text[start:end + 1]
    raise ValueError('Unclosed function: ' + name)


def source_contract(text):
    wheel = function(text, 'feasibilityGradCostCar')
    smooth = function(text, 'smoothedL1')
    if text.count('B_h_ << 0.0, -1.0,\n            1.0,  0.0;') != 1:
        raise ValueError('Unknown B matrix initialization')
    snippets = (
        'gradv_2d.setZero(); grada_2d.setZero(); gradj_2d.setZero();',
        'gradViolaVt = alpha * gradv.transpose() * acc;',
        'gdT(i) += omg * (cost_mm_feasible / K + step * gradViolaVt);',
        'gradViolaAt = alpha * grada.transpose() * jer;',
        'gradViolaJt = alpha * gradj.transpose() * snap;',
    )
    if not all(s in text for s in snippets):
        raise ValueError('Unknown gradient initialization/time-chain contract')
    return wheel, smooth, dict(
        wheel_function_sha256=hashlib.sha256(wheel.encode()).hexdigest(),
        smooth_function_sha256=hashlib.sha256(smooth.encode()).hexdigest(),
        B=[[0., -1.], [1., 0.]],
        B_orientation='a^T B v = vx*ay-vy*ax = planar cross(v,a)',
        gradients_are_accumulators=True,
        callers_zero_gradients_before_each_call=True,
        fixed_coefficient_time_chain_source_snippets=list(snippets[1:]))


def cpp_probe(text):
    wheel, smooth, unused = source_contract(text)
    # Only shell class/type declarations and a no-op profiling timer are added.
    # Both numerical function bodies and their member names remain unchanged.
    return '''#include <Eigen/Core>
#include <vector>
namespace failure_probe { enum { BASE_FEAS=0 }; struct Timer { explicit Timer(int){} }; }
class PolyTrajOptimizer {
public:
  Eigen::Matrix2d B_h_;
  std::vector<int> singul_container_;
  double wei_feas_, max_vel_, non_singul_v_, mobile_base_wheel_base_,
    mobile_base_wheel_radius_, max_wheel_omega_, max_wheel_alpha_;
  bool smoothedL1(const double&, const double&, double&, double&);
  bool feasibilityGradCostCar(const Eigen::Vector2d&, const Eigen::Vector2d&,
    const Eigen::Vector2d&, int, Eigen::Vector2d&, Eigen::Vector2d&,
    Eigen::Vector2d&, double&, bool);
};
''' + smooth + '\n' + wheel + '''
extern "C" int native_wheel_probe(const double *p, int gear, int dense,
                                  const double *x, double *out) {
  if(gear!=1 && gear!=-1)return -1;
  PolyTrajOptimizer o;
  o.B_h_ << 0.,-1.,1.,0.;
  o.singul_container_={gear};
  o.wei_feas_=p[0]; o.max_vel_=p[1]; o.non_singul_v_=p[2];
  o.mobile_base_wheel_base_=p[3]; o.mobile_base_wheel_radius_=p[4];
  o.max_wheel_omega_=p[5]; o.max_wheel_alpha_=p[6];
  Eigen::Vector2d gv=Eigen::Vector2d::Zero(),ga=gv,gj=gv;
  double cost=0.;
  bool active=o.feasibilityGradCostCar(Eigen::Vector2d(x[0],x[1]),
    Eigen::Vector2d(x[2],x[3]),Eigen::Vector2d(x[4],x[5]),0,gv,ga,gj,cost,dense!=0);
  out[0]=cost; out[1]=gv[0];out[2]=gv[1];out[3]=ga[0];out[4]=ga[1];
  out[5]=gj[0];out[6]=gj[1];
  return active?1:0;
}
'''


class Probe:
    def __init__(self, library):
        self.library = ctypes.CDLL(str(Path(library).resolve()))
        self.call = self.library.native_wheel_probe
        pointer = ctypes.POINTER(ctypes.c_double)
        self.call.argtypes = (pointer, ctypes.c_int, ctypes.c_int, pointer, pointer)
        self.call.restype = ctypes.c_int

    def __call__(self, params, state, gear=1, dense=False):
        p = np.ascontiguousarray(params, dtype=np.float64)
        x = np.ascontiguousarray(state, dtype=np.float64)
        if p.shape != (7,) or x.shape != (6,) or not np.all(np.isfinite(p)) or not np.all(np.isfinite(x)):
            raise ValueError('Invalid probe parameters/state')
        if gear not in (-1, 1):
            raise ValueError('Invalid gear')
        out = np.zeros(7)
        pointer = ctypes.POINTER(ctypes.c_double)
        code = self.call(p.ctypes.data_as(pointer), gear, int(dense),
                         x.ctypes.data_as(pointer), out.ctypes.data_as(pointer))
        if code < 0 or not np.all(np.isfinite(out)):
            raise ValueError('Native probe failed or returned nonfinite')
        return float(out[0]), out[1:], bool(code)


def finite_difference(probe, params, state, gear, dense):
    state = np.asarray(state, dtype=float)
    cost, analytic, active = probe(params, state, gear, dense)
    norm = float(np.linalg.norm(state[:2]))
    category = ('clamp_boundary_nondifferentiable' if abs(norm - 1e-4) <= 1e-12
                else 'clamp_interior' if norm < 1e-4 else 'ordinary')
    # Scale v perturbations down at low speed to avoid crossing the clamp except
    # for explicitly identified boundary tests. Steps were specified a priori.
    velocity_scale = max(norm, 1e-5)
    scales = np.array([velocity_scale] * 2 +
                      [max(np.linalg.norm(state[2:4]), .01)] * 2 +
                      [max(np.linalg.norm(state[4:6]), .01)] * 2)
    differences = []
    for multiplier in STEPS:
        h = multiplier * scales
        central = np.zeros(6)
        five = np.zeros(6)
        for i in range(6):
            axis = np.zeros(6); axis[i] = h[i]
            pm = [probe(params, state + k * axis, gear, dense)[0]
                  for k in (-2., -1., 1., 2.)]
            central[i] = (pm[2] - pm[1]) / (2. * h[i])
            five[i] = five_point(pm, h[i])
        denominator = np.maximum(1., np.maximum(np.abs(analytic), np.abs(five)))
        differences.append(dict(multiplier=multiplier, h=h.tolist(),
            centered=central.tolist(), five_point=five.tolist(),
            normalized_error=(np.abs(analytic-five)/denominator).tolist(),
            max_normalized_error=float(np.max(np.abs(analytic-five)/denominator))))
    # The middle step is the prespecified primary result. Other steps diagnose
    # convergence/cancellation, never select an outcome-favorable step.
    primary = differences[1]
    return dict(state_v_a_j=state.tolist(), gear=gear, dense_sample=bool(dense),
        cost=cost, active=active, analytic=analytic.tolist(), category=category,
        velocity_norm=norm, finite_differences=differences,
        primary_max_normalized_error=primary['max_normalized_error'],
        tolerance=TOLERANCE,
        primary_pass=None if category=='clamp_boundary_nondifferentiable'
                     else primary['max_normalized_error'] <= TOLERANCE)


def five_point(values, h):
    # Pair opposite samples before multiplication. The algebraically equivalent
    # unpaired sum manufactures nonzero derivatives for a constant large cost.
    return (8.*(values[2]-values[1])-(values[3]-values[0]))/(12.*h)


def native_parameters(trial):
    wheel, evidence = wheel_parameters(trial)
    loaded = (trial/'parameters.yaml').read_text()
    prefix = '/remani_planner_node/'
    keys = [prefix+'optimization/weight_base_feasibility',
            prefix+'mm/mobile_base_non_singul_vel',
            prefix+'optimization/constrain_points_perPiece',
            prefix+'optimization/dense_sample_resolution']
    values = loaded_values(loaded, keys)
    exported = read_json(trial/'effective_real_params.json')
    if any(exported.get(k) != values[k] for k in keys):
        raise ValueError('Loaded/exported feasibility parameter mismatch')
    config = BASE.parents[1]/'src/mm_remani_planner/mm_config/src/mm_config.cpp'
    text = config.read_text()
    expected = read_json(Path(evidence['source']).parent.parent/'build_manifest.json')
    attested = expected['production_sources_sha256'].get(str(config.relative_to(BASE.parents[1])))
    if digest(config) != attested or ('mobile_base_max_vel_ = mobile_base_max_wheel_omega_ * mobile_base_wheel_radius_;' not in text):
        raise ValueError('Unattested maximum linear velocity derivation')
    params = [values[keys[0]], wheel['wheel_speed_limit_rad_s']*wheel['wheel_radius_m'],
              values[keys[1]], wheel['wheel_base_m'], wheel['wheel_radius_m'],
              wheel['wheel_speed_limit_rad_s'], wheel['wheel_acceleration_limit_rad_s2']]
    return params, wheel, dict(evidence, feasibility_values=dict(zip(PARAMETER_NAMES, params)),
        maximum_velocity_derivation='attested MMConfig wheel_speed_limit * wheel_radius',
        mm_config_sha256=digest(config), quadrature_intervals=int(values[keys[2]]),
        dense_resolution=int(values[keys[3]]))


def basis_derivative(coefficients, t, order):
    return np.array([np.polynomial.polynomial.polyval(t,
        np.polynomial.polynomial.polyder(c, order)) for c in coefficients.T])


def quadrature_nodes(piece, intervals, dense_resolution):
    """Actual native primary and low-speed-triggered dense evaluation times.

    Dense trigger is based on smoothedLog(x,.001): x>-mu. Dense branch uses
    primary node velocity; dense_sample=true adds minimum speed penalty.
    """
    c = np.asarray(piece['coeffs'], dtype=float)[:2, ::-1].T
    T = float(piece['T']); step = T/intervals
    time = 0.
    for j in range(intervals + 1):
        yield time, False
        vel = basis_derivative(c, time, 1)
        if .01 - float(vel @ vel) > -.001:
            begin = time if j == 0 else time - step/2.
            count = dense_resolution//2 if j in (0, intervals) else dense_resolution
            dense_time = begin
            for unused in range(count + 1):
                yield dense_time, True
                dense_time += step/dense_resolution
        time += step


def select_candidate_indices(candidates):
    if not candidates:
        return []
    indices = [0]
    accepted = [i for i, c in enumerate(candidates) if c['stage']=='accepted']
    if accepted:
        indices += [accepted[-1], max(accepted, key=lambda i:max(
            candidates[i]['diagnostic_10ms_with_boundaries']['peaks'][k]['ratio_to_final_limit']
            for k in CHANNELS))]
    return sorted(set(indices))


def time_chain(probe, params, piece, gear, intervals):
    """Primary feasibility quadrature, frozen coefficients; not full MINCO
    coefficient/inner-point backpropagation or dense-trigger quadrature.
    """
    c = np.asarray(piece['coeffs'], dtype=float)[:2, ::-1].T
    duration = float(piece['T'])
    def evaluate(T, gradient=False):
        cost = 0.; chain = 0.
        for j in range(intervals + 1):
            fraction = j/intervals
            v, a, jerk, snap = [basis_derivative(c, fraction*T, k) for k in (1,2,3,4)]
            value, g, unused = probe(params, np.r_[v,a,jerk], gear, False)
            weight = .5 if j in (0, intervals) else 1.
            cost += weight*T/intervals*value
            chain += weight*(value/intervals + T/intervals*fraction*(g[:2]@a+g[2:4]@jerk+g[4:6]@snap))
        return (cost, chain) if gradient else cost
    cost, analytic = evaluate(duration, True)
    rows = []
    for multiplier in STEPS:
        h=multiplier*max(duration,.1)
        values=[evaluate(duration+k*h) for k in (-2.,-1.,1.,2.)]
        numeric=five_point(values,h)
        error=abs(analytic-numeric)/max(1.,abs(analytic),abs(numeric))
        rows.append(dict(multiplier=multiplier,h=h,five_point=numeric,normalized_error=error))
    return dict(cost=cost,analytic=analytic,finite_differences=rows,
        primary_max_normalized_error=rows[1]['normalized_error'],
        primary_pass=bool(rows[1]['normalized_error']<=TOLERANCE),
        scope='primary feasibility quadrature d/dT at frozen coefficients; excludes full MINCO and dense-trigger chain')


def collect(trial, prior, probe):
    records, path = canonical_records(trial)
    if path is None or digest(path)!=prior['replay_sha256'] or prior['evidence_valid'] is not True:
        raise ValueError('Invalid prior kinematics evidence')
    captures=[r for r in records if r.get('kind')=='capture']
    if len(captures)!=1:
        raise ValueError('Expected single capture')
    params, wheel_params, evidence=native_parameters(trial)
    intervals=evidence['quadrature_intervals']; resolution=evidence['dense_resolution']
    results=[]; coverage=[]; time_results=[]
    for phase in prior['phases']:
        rows=[r for r in records if r.get('kind')=='canonical_candidate' and r.get('phase')==phase['phase']]
        if len(rows)!=len(phase['candidates']):
            raise ValueError('Candidate count mismatch')
        selected=select_candidate_indices(phase['candidates'])
        speeds=[]; dense_count=0; primary_count=0; min_node=None
        for i,(record,old) in enumerate(zip(rows,phase['candidates'])):
            if vector_hash(record.get('x'))!=old['candidate_x_hash'] or old['evidence_valid'] is not True:
                raise ValueError('Canonical candidate hash mismatch')
            candidate=canonical_candidate(record,captures[0]['frame']['now'])
            if candidate is None:
                raise ValueError('Missing canonical candidate')
            common=dict(trial=trial.name,phase=phase['phase'],stage=record['stage'],
                        iteration=record['iteration'],candidate_x_hash=vector_hash(record['x']))
            lowest=None
            for pi,part in enumerate(candidate['parts']):
                for qi,piece in enumerate(part['pieces']):
                    for local,dense in quadrature_nodes(piece,intervals,resolution):
                        v=derivative(piece,local,1)
                        speed=float(np.linalg.norm(v))
                        primary_count+=not dense;dense_count+=dense
                        speeds.append(speed)
                        new_low=lowest is None or speed<lowest['velocity_norm']
                        new_global=min_node is None or speed<min_node['velocity_norm']
                        if new_low or new_global:
                            acc,jer=[derivative(piece,local,k) for k in (2,3)]
                            node=dict(common,part=pi,piece=qi,local_t=local,dense_sample=dense,
                                      gear=part['gear'],state=np.r_[v,acc,jer].tolist(),velocity_norm=speed)
                            if new_low:lowest=node
                            if new_global:min_node=node
            if i not in selected:
                continue
            chosen=[]
            for channel in CHANNELS:
                peak=old['diagnostic_10ms_with_boundaries']['peaks'][channel]
                value=wheel_values(candidate,peak['t'],wheel_params)
                chosen.append(dict(part=value['part'],piece=value['piece'],local_t=value['local_t'],
                    state=value['velocity']+value['acceleration']+value['jerk'],gear=value['gear'],
                    source='diagnostic_peak_'+channel,dense_sample=False))
            chosen.append(dict(lowest,source='minimum_actual_objective_node'))
            seen=set()
            for node in chosen:
                identity=(node['part'],node['piece'],node['local_t'],node['dense_sample'])
                if identity in seen:continue
                seen.add(identity)
                check=finite_difference(probe,params,node['state'],node['gear'],node['dense_sample'])
                results.append(dict(common,part=node['part'],piece=node['piece'],local_t=node['local_t'],
                    source=node['source'],**check))
            # Highest violating peak's piece (deterministic before new results).
            peak=max((old['diagnostic_10ms_with_boundaries']['peaks'][k] for k in CHANNELS),
                     key=lambda p:p['ratio_to_final_limit'])
            part=candidate['parts'][peak['part']];piece=part['pieces'][peak['piece']]
            time_results.append(dict(common,part=peak['part'],piece=peak['piece'],
                                     **time_chain(probe,params,piece,part['gear'],intervals)))
        coverage.append(dict(phase=phase['phase'],candidates_scanned=len(rows),selected_indices=selected,
            primary_nodes=primary_count,dense_nodes=dense_count,minimum_velocity_norm=min(speeds),
            nodes_below_floor=sum(s<1e-4 for s in speeds),minimum_node=min_node))
    return dict(trial=trial.name,parameter_evidence=evidence,
                coverage=coverage,gradient_checks=results,time_chain_checks=time_results)


def synthetic(params, probe):
    states=[('ordinary_forward',[.15,.04,.1,.3,.2,-.1],1),
            ('ordinary_reverse',[-.15,.04,.1,.3,.2,-.1],-1),
            ('low_above_floor',[1.1e-4,0.,.1,.2,.3,-.1],1),
            ('floor_boundary',[1e-4,0.,.1,.2,.3,-.1],1),
            ('inside_floor',[5e-5,0.,.1,.2,.3,-.1],1),
            ('zero_velocity',[0.,0.,.1,.2,.3,-.1],1)]
    return [dict(name=name,**finite_difference(probe,params,state,gear,dense))
            for name,state,gear in states for dense in (False,True)]


def protocol():
    return dict(schema='backend_wheel_gradient_audit_protocol_v1',
        no_solver_runs=True,no_ROS=True,no_production_edits=True,
        candidate_selection='per trial/phase: initial, last accepted, max accepted wheel-ratio; deduplicated',
        point_selection='four diagnostic wheel peaks plus minimum actual primary/dense objective node',
        coverage='minimum speeds at actual primary and triggered dense nodes for every saved canonical candidate',
        independent_variable_order=['vx','vy','ax','ay','jx','jy'],
        finite_difference='two-point centered and five-point centered with opposite-sample pairing, three prespecified steps',
        steps=list(STEPS),primary_step_index=1,normalized_tolerance=TOLERANCE,
        clamp_boundary='exact 1e-4 velocity norm is nondifferentiable; do not label analytic mismatch as a smooth-point failure',
        time_chain='primary quadrature with frozen coefficients; not a complete native MINCO gradient test',
        no_improvement_claim='numerical consistency audit only; no trajectory-delivery or solver improvement claim')


def run(root, audit, out, build):
    root=Path(root).resolve();audit=Path(audit).resolve();out=Path(out).resolve();build=Path(build).resolve()
    if out.exists():raise ValueError('Fresh output required')
    prior=read_json(audit)
    if Path(prior['source']).resolve()!=root or prior['schema']!='backend_kinematics_audit_v1':
        raise ValueError('Unexpected prior audit/schema')
    source=build/'source/poly_traj_optimizer.cpp'
    manifest=read_json(build/'build_manifest.json')
    if digest(source)!=manifest['isolated_source_sha256']['poly_traj_optimizer.cpp']:
        raise ValueError('Unattested native optimizer source')
    text=source.read_text();unused1,unused2,contract=source_contract(text)
    out.mkdir(parents=True,exist_ok=False)
    (out/'protocol.json').write_text(json.dumps(protocol(),indent=2)+'\n')
    cpp=out/'wheel_gradient_probe.cpp';cpp.write_text(cpp_probe(text))
    library=out/'wheel_gradient_probe.so'
    command=['g++','-O2','-std=c++17','-shared','-fPIC','-I/usr/include/eigen3',str(cpp),'-o',str(library)]
    compiled=subprocess.run(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    (out/'compile.log').write_text(compiled.stdout)
    compiled.check_returncode()
    probe=Probe(library)
    trials=[]
    for old in prior['trials']:
        trial=root/old['trial']
        if Path(old['parameter_evidence']['source']).resolve()!=source:
            raise ValueError('Prior source differs from requested baseline')
        trials.append(collect(trial,old,probe))
    params,unused,evidence=native_parameters(root/prior['trials'][0]['trial'])
    controls=synthetic(params,probe)
    real=[c for t in trials for c in t['gradient_checks']]
    times=[c for t in trials for c in t['time_chain_checks']]
    result=dict(schema='backend_wheel_gradient_audit_v1',source=str(root),
        baseline_build=str(build),baseline_manifest_sha256=digest(build/'build_manifest.json'),
        native_source_sha256=digest(source),prior_kinematics_audit=str(audit),
        prior_kinematics_audit_sha256=digest(audit),probe_sha256=digest(library),
        probe_source_sha256=digest(cpp),auditor_sha256=digest(Path(__file__)),
        compiler_command=command,source_contract=contract,protocol=protocol(),trials=trials,
        synthetic_controls=controls,
        summary=dict(real_checks=len(real),real_categories=dict(Counter(c['category'] for c in real)),
            real_pass=sum(c['primary_pass'] is True for c in real),
            real_fail=sum(c['primary_pass'] is False for c in real),
            worst_real_error=max(c['primary_max_normalized_error'] for c in real),
            primary_time_chain_checks=len(times),time_chain_pass=sum(c['primary_pass'] for c in times),
            worst_time_chain_error=max(c['primary_max_normalized_error'] for c in times),
            scanned_candidates=sum(c['candidates_scanned'] for t in trials for c in t['coverage']),
            scanned_objective_nodes=sum(c['primary_nodes']+c['dense_nodes'] for t in trials for c in t['coverage']),
            actual_nodes_below_floor=sum(c['nodes_below_floor'] for t in trials for c in t['coverage'])),
        limitations=['The tested cost includes linear-speed, wheel-speed/acceleration and optional minimum-speed penalties.',
            'Physical wheel-model correctness and native vs final-check sampling differences are not established by finite differences.',
            'Full MINCO gradient/backpropagation, dense-trigger time chain, map and collision gradients are outside this audit.',
            'Synthetic low-speed defects are not causal evidence for captured requests unless the captured objective visits that region.',
            'Historical candidate native/posterior audits do not certify future online state/map admission.'])
    (out/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result['summary'],indent=2))
    return result


def recheck(previous_path,out):
    """Infrastructure-only FD repair: reuse hashed node coverage, not outcomes.

    Every selected derivative and time-chain check is recomputed against the
    original extracted native library/canonical inputs. The 713-candidate node
    coverage need not be rescanned because neither sampling nor source changed.
    """
    previous_path=Path(previous_path).resolve();out=Path(out).resolve()
    if out.exists():raise ValueError('Fresh recheck output required')
    old=read_json(previous_path)
    if old['schema']!='backend_wheel_gradient_audit_v1':raise ValueError('Unknown audit schema')
    build=Path(old['baseline_build']);source=build/'source/poly_traj_optimizer.cpp'
    if digest(source)!=old['native_source_sha256']:raise ValueError('Native source changed')
    library=previous_path.parent/'wheel_gradient_probe.so'
    if digest(library)!=old['probe_sha256']:raise ValueError('Probe library changed')
    if digest(Path(old['prior_kinematics_audit']))!=old['prior_kinematics_audit_sha256']:
        raise ValueError('Prior kinematics audit changed')
    probe=Probe(library);result=dict(old);trials=[]
    for old_trial in old['trials']:
        trial=Path(old['source'])/old_trial['trial']
        params,unused,evidence=native_parameters(trial)
        if evidence!=old_trial['parameter_evidence']:raise ValueError('Parameter evidence changed')
        records,replay=canonical_records(trial)
        if digest(replay)!=evidence.get('replay_sha256',digest(replay)):
            raise ValueError('Replay changed')
        old_prior=next(t for t in read_json(Path(old['prior_kinematics_audit']))['trials'] if t['trial']==trial.name)
        if digest(replay)!=old_prior['replay_sha256']:raise ValueError('Canonical replay changed')
        capture=next(r for r in records if r.get('kind')=='capture')
        lookup={(r['phase'],vector_hash(r['x'])):r for r in records if r.get('kind')=='canonical_candidate'}
        checks=[]
        for row in old_trial['gradient_checks']:
            if (row['phase'],row['candidate_x_hash']) not in lookup:raise ValueError('Lost source candidate')
            checks.append(dict(row,**finite_difference(probe,params,row['state_v_a_j'],row['gear'],row['dense_sample'])))
        times=[]
        for row in old_trial['time_chain_checks']:
            candidate=canonical_candidate(lookup[(row['phase'],row['candidate_x_hash'])],capture['frame']['now'])
            part=candidate['parts'][row['part']];piece=part['pieces'][row['piece']]
            times.append(dict(row,**time_chain(probe,params,piece,part['gear'],evidence['quadrature_intervals'])))
        trials.append(dict(old_trial,gradient_checks=checks,time_chain_checks=times))
    params,unused,evidence=native_parameters(Path(old['source'])/old['trials'][0]['trial'])
    result.update(trials=trials,synthetic_controls=synthetic(params,probe),auditor_sha256=digest(Path(__file__)),
        protocol=protocol(),infrastructure_recheck=dict(parent=str(previous_path),parent_sha256=digest(previous_path),
            reason='opposite-pair five-point FD arithmetic avoids large-constant cancellation',
            coverage_reused=True,numerical_checks_recomputed=True,solver_rerun=False),
        probe_library_path=str(library))
    real=[c for t in trials for c in t['gradient_checks']];times=[c for t in trials for c in t['time_chain_checks']]
    result['summary']=dict(old['summary'],real_pass=sum(c['primary_pass'] is True for c in real),
        real_fail=sum(c['primary_pass'] is False for c in real),
        worst_real_error=max(c['primary_max_normalized_error'] for c in real),
        time_chain_pass=sum(c['primary_pass'] for c in times),
        worst_time_chain_error=max(c['primary_max_normalized_error'] for c in times))
    out.mkdir(parents=True,exist_ok=False)
    (out/'protocol.json').write_text(json.dumps(protocol(),indent=2)+'\n')
    (out/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result['summary'],indent=2))
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=BASE/'results/backend_feasibility_20261025_01')
    parser.add_argument('--kinematics-audit',type=Path,default=BASE/'results/backend_kinematics_audit_20261025_01/summary.json')
    parser.add_argument('--out',type=Path,default=BASE/'results/wheel_gradient_20261026_01')
    parser.add_argument('--build',type=Path,default=BASE/'builds/seed_parameterization_20261025_01')
    parser.add_argument('--recheck-audit',type=Path,help='Reuse hashed coverage after FD-arithmetic infrastructure repair only')
    args=parser.parse_args()
    if args.recheck_audit:recheck(args.recheck_audit,args.out)
    else:run(args.root,args.kinematics_audit,args.out,args.build)
