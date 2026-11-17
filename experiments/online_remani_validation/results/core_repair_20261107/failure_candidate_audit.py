#!/usr/bin/env python3
"""Offline attribution of the saved failure; never supplies controls or parameters."""
import hashlib
import json
import os
from pathlib import Path
import resource
import sys
import time
from types import SimpleNamespace as NS

for name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[name] = '1'
import numpy as np
import yaml
from scipy.ndimage import map_coordinates

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
SOURCE = OUT/'failure_capture'
CORE = ROOT/'src/mm_remani/scripts/mppi'
sys.path.insert(0, str(CORE))
# Preserve the exact failed controller version while the main code is repaired.
sys.path.insert(0, str(SOURCE/'audited_source'))
from active_arm_model import PiperArmModel, DEFAULT_PROXY, DEFAULT_URDF
from active_arm_mppi import Snapshot, WholeBodyMPPI
from native_reference_adapter import NativeReferenceAdapter


def stamp(value):
    return NS(to_sec=lambda: value)


def restore_reference(adapter, events, diagnostic):
    now = diagnostic['plan_time']
    for event in events:
        if event['ros_time'] > now:
            break
        d = event['data']
        if event['kind'] == 'goal':
            message = NS(header=NS(stamp=stamp(d['stamp']), frame_id='world'),
                pose=NS(position=NS(x=d['x'], y=d['y'], z=0.),
                        orientation=NS(x=0., y=0., z=np.sin(d['yaw']/2), w=np.cos(d['yaw']/2))))
            assert adapter.feed_goal(message, event['ros_time']) == 'accepted'
        elif event['kind'] == 'trajectory':
            message = NS(header=NS(stamp=stamp(d['stamp']), frame_id=d['frame']),
                action=d['action'], trajectory_id=d['trajectory_id'], singul=d['singul'],
                trajectory=[NS(**p) for p in d['pieces']],
                serialize=lambda stream, raw=json.dumps(d).encode(): stream.write(raw))
            assert adapter.feed_trajectory(message, event['ros_time']) in ('replace', 'append', 'notice')
    return reference_at(adapter, diagnostic)


def reference_at(adapter, diagnostic):
    now = diagnostic['plan_time']
    adapter.progress = (now, diagnostic['native_phase'], diagnostic['native_time_scale'])
    path, info = adapter._reference(diagnostic['measured_state'][2], now)
    assert info['reference_hash'] == diagnostic['reference_hash']
    np.testing.assert_array_equal(info['native_arm_reference'], diagnostic['native_arm_reference'])
    return path, np.asarray(info['native_arm_reference'])


class RecordingSnapshot:
    def __init__(self, snapshot):
        self.snapshot = snapshot

    def query(self, spheres, times):
        self.spheres, self.times = spheres.copy(), times.copy()
        self.gaps, self.soft = self.snapshot.query(spheres, times)
        return self.gaps, self.soft


def main():
    began = time.monotonic()
    record = json.loads((SOURCE/'failure_status.json').read_text())
    d = record['status']['candidate_diagnostics']; now = d['plan_time']
    state = np.asarray(d['measured_state'])
    events = [json.loads(line) for line in (SOURCE/'navigation_events.jsonl').open()]
    status = [json.loads(line) for line in (SOURCE/'mppi_status.jsonl').open()]
    params = yaml.safe_load((SOURCE/'parameters.yaml').read_text())
    profile = json.loads((SOURCE/'experiment.json').read_text())['controller_profile']
    model = PiperArmModel.from_parameters(SOURCE/'planner_params.yaml')
    model.lower[5] = max(model.lower[5], -.35); model.upper[5] = min(model.upper[5], .35)
    kernel = WholeBodyMPPI(model, profile['base_limits'],
        (model.lower, model.upper, profile['arm_velocity'], profile['arm_acceleration']),
        profile['postures'], enabled=params['/whole_body_mppi/arm_enabled'],
        seed=params['/whole_body_mppi/seed'], horizon=profile['horizon'], step=1/profile['rate_hz'],
        random_samples=profile['random_samples'], arm_position_servo=params['/whole_body_mppi/arm_position_servo'])
    adapter = NativeReferenceAdapter(kernel, navigation_kernel=kernel)
    path, arm = restore_reference(adapter, events, d)
    prior = [r for r in status if r['status'].get('candidate_diagnostics', {}).get('plan_time', np.inf) < now][-1]
    pd = prior['status']['candidate_diagnostics']
    assert pd['mode'] == 'nominal'
    pp, pa = reference_at(adapter, pd)
    # plan() writes previous on nominal success, including when later delivery is rejected.
    kernel.previous = kernel.proposals(np.asarray(pd['measured_state']), pp, pa, [(1., 0.)])[0][0]
    path, arm = reference_at(adapter, d)
    controls, labels = kernel.proposals(state, path, arm)
    assert [list(x) for x in labels] == d['structured_labels']
    wire = np.load(SOURCE/'failure_snapshot.npy', allow_pickle=False)
    snapshot = Snapshot.decode(wire); h = snapshot.header
    np.testing.assert_array_equal(np.delete(h, 20), np.delete(d['snapshot_header'], 20))
    recorder = RecordingSnapshot(snapshot)
    cost, valid, gap, states = kernel.evaluate(state, controls, path, arm, recorder, now)
    diagnostics = kernel.last_evaluation_diagnostics
    errors = {}
    for key, value in dict(costs=cost, valid=valid, external_gaps=kernel.last_external_gaps,
                           **diagnostics).items():
        expected = d['structured_'+key]
        np.testing.assert_allclose(value, expected, rtol=0., atol=1e-11)
        errors[key] = float(np.max(np.abs(np.asarray(value, float)-np.asarray(expected, float))))
    spheres, times = recorder.spheres, recorder.times
    xyz = spheres[..., :3]; radius = spheres[..., 3].copy(); radius[:, :47] += h[11]
    indices = (xyz-h[8:11])/h[11]-.5
    inside = np.all((indices >= 0) & (indices < np.array(snapshot.distance.shape)-1), axis=-1)
    esdf = map_coordinates(snapshot.distance, np.moveaxis(indices, -1, 0), order=1,
                           prefilter=False, mode='nearest')
    esdf = np.where(inside, esdf, -1000.)
    nb, nd = int(h[12]), int(h[13])
    dynamic = wire[32+7*nb:32+7*(nb+nd)].reshape((-1, 7))
    age = np.maximum(0., times-h[18])
    epsilon = h[25]+h[26]*age+.5*h[27]*age**2
    nearest = np.full(radius.shape, np.inf); point_id = np.full(radius.shape, -1, int)
    # Exhaustive observed-point attribution; independent of query's AABB pruning.
    for index, point in enumerate(dynamic):
        distance = np.linalg.norm(xyz-(point[:3]+age[:, None, None]*point[3:6]), axis=-1)
        smaller = distance < nearest
        nearest[smaller] = distance[smaller]; point_id[smaller] = index
    static_gap = esdf-radius
    point_gap = nearest-radius
    dynamic_gap = point_gap-epsilon[:, None]
    external = np.minimum(static_gap, dynamic_gap)
    np.testing.assert_allclose(external.min(axis=1), recorder.gaps, rtol=0., atol=1e-12)
    independent_soft = np.maximum(0., .08-external)**3
    np.testing.assert_allclose(independent_soft.sum(axis=1)*180., recorder.soft, rtol=0., atol=1e-10)

    def at(row, sphere):
        point = int(point_id[row, sphere])
        return dict(relative_time_s=float(times[row]-now), sphere_index=int(sphere),
            sphere_group='fixed_structure' if sphere < 47 else 'arm',
            sphere_center_xyz_m=xyz[row, sphere].tolist(), radius_with_margin_m=float(radius[row, sphere]),
            source='dynamic_uncertainty' if dynamic_gap[row, sphere] < static_gap[row, sphere] else 'esdf',
            external_gap_m=float(external[row, sphere]), esdf_gap_m=float(static_gap[row, sphere]),
            observed_point_gap_without_epsilon_m=float(point_gap[row, sphere]),
            epsilon_m=float(epsilon[row]), dynamic_observation_age_s=float(age[row]),
            observed_point_index=point, observed_point_xyz_vxyz_id=dynamic[point].tolist(),
            predicted_point_xyz_m=(dynamic[point, :3]+age[row]*dynamic[point, 3:6]).tolist())

    candidates = []; cursor = 0
    for i, label in enumerate(labels):
        count = 1+round(diagnostics['checked_duration_s'][i]/(kernel.dt/2))
        rows = np.arange(cursor, cursor+count); values = external[rows]
        first = rows[np.flatnonzero(values.min(axis=1) < 0)[0]]
        worst_local = np.unravel_index(np.argmin(values), values.shape)
        worst = (rows[worst_local[0]], worst_local[1])
        static_min = float(static_gap[rows].min()); point_min = float(point_gap[rows].min())
        candidates.append(dict(index=i, label=list(label), cost=float(cost[i]), valid=bool(valid[i]),
            minimum_external_gap_m=float(gap[i]), minimum_esdf_gap_m=static_min,
            minimum_observed_points_gap_without_epsilon_m=point_min,
            zero_epsilon_diagnostic_valid=bool(min(static_min, point_min, diagnostics['configuration_gaps'][i]) >= 0
                and diagnostics['bounds_valid'][i] and diagnostics['braking_stopped'][i]),
            first_negative=at(first, np.argmin(external[first])), worst_negative=at(*worst),
            configuration_gap_m=diagnostics['configuration_gaps'][i],
            checked_duration_s=diagnostics['checked_duration_s'][i],
            terminal_arm_delta_rad=diagnostics['terminal_arm_delta'][i],
            trace=[dict(relative_time_s=float(times[r]-now), minimum_external_gap_m=float(external[r].min()),
                minimum_sphere_index=int(np.argmin(external[r])), minimum_esdf_gap_m=float(static_gap[r].min()),
                minimum_points_gap_without_epsilon_m=float(point_gap[r].min()),
                fixed_sphere44_dynamic_gap_m=float(dynamic_gap[r, 44])) for r in rows]))
        cursor += count
    assert cursor == len(times)

    # Universal necessary-condition witness: sphere44 is independent of all six q.
    # For any admissible base, displacement <= vmax*t + 2*rho*sin(wmax*t/2).
    # Triangle inequality bounds its distance from one actual predicted point from above.
    initial_sphere = kernel.spheres(state[None])[0, 44]
    rho = float(np.linalg.norm(model.fixed_spheres[44, :2]))
    reachability = []
    for tau in np.arange(0., kernel.H*kernel.dt+1e-9, kernel.dt/2):
        forecast_age = now-h[18]+tau
        forecast = dynamic[:, :3]+forecast_age*dynamic[:, 3:6]
        distances = np.linalg.norm(initial_sphere[:3]-forecast, axis=1)
        pid = int(np.argmin(distances))
        translation_bound = kernel.limits['vmax']*tau
        rotation_bound = 2*rho*np.sin(min(kernel.limits['wmax']*tau, np.pi)/2)
        eps = h[25]+h[26]*forecast_age+.5*h[27]*forecast_age**2
        upper = distances[pid]+translation_bound+rotation_bound-initial_sphere[3]-h[11]-eps
        reachability.append(dict(relative_time_s=float(tau), point_index=pid,
            original_center_to_forecast_point_distance_m=float(distances[pid]),
            translation_upper_bound_m=float(translation_bound), rotation_upper_bound_m=float(rotation_bound),
            epsilon_m=float(eps), all_admissible_base_fixed44_gap_upper_bound_m=float(upper)))
    witness = next((r for r in reachability if r['all_admissible_base_fixed44_gap_upper_bound_m'] < 0), None)
    report = dict(scope='Offline exact-frame modeled attribution; no ROS, hardware, parameter changes, or deployment recommendation.',
        units='metres, seconds, radians; sphere indices are zero-based',
        plan_time=now, scene_plan_elapsed_s=now-(record['ros_time']-record['elapsed_s']),
        snapshot_id=int(h[2]), snapshot_age_s=float(now-h[3]), observation_age_s=float(now-h[18]),
        dynamic_points=nd, dynamic_tracks=sorted(set(dynamic[:, 6].tolist())),
        uncertainty=dict(formula='epsilon(age)=0.08+0.25*age+0.5*0.75*age^2; age=plan_time-observation_stamp+rollout_time',
                         at_plan_m=float(h[25]+h[26]*(now-h[18])+.5*h[27]*(now-h[18])**2)),
        reference_reconstruction=dict(hash=d['reference_hash'], geometry_and_arm_samples_exact=True,
            message_serialization_note='JSON stub used only for irrelevant per-message digest; path/arm polynomial coefficients and numerical reference hash are exact.',
            previous_plan_time=pd['plan_time'], previous_status=prior['status']['state'],
            previous_reconstruction='Previous plan mode nominal writes proposals[0] before delivery checks; shifted normally by proposals().'),
        replay_max_absolute_errors=errors, native_servo_used=kernel._servo_native is not None,
        candidate_count=len(candidates), first_negative_sources=sorted(set(c['first_negative']['source'] for c in candidates)),
        zero_epsilon_diagnostic_valid_count=sum(c['zero_epsilon_diagnostic_valid'] for c in candidates),
        all_esdf_positive=all(c['minimum_esdf_gap_m'] > 0 for c in candidates),
        attribution_summary=dict(
            minimum_esdf_gap_m=min(c['minimum_esdf_gap_m'] for c in candidates),
            minimum_observed_points_gap_without_epsilon_m=min(c['minimum_observed_points_gap_without_epsilon_m'] for c in candidates),
            first_negative_sphere_indices=sorted(set(c['first_negative']['sphere_index'] for c in candidates)),
            first_negative_relative_time_range_s=[min(c['first_negative']['relative_time_s'] for c in candidates),
                                                 max(c['first_negative']['relative_time_s'] for c in candidates)],
            fixed44_conclusion='The specified isotropic uncertainty set and current motion limits make the hard prediction problem infeasible; this is a specification/perception-condition block, not a code bug remediable by more candidates or different weights.',
            failure_before_braking_tail=True,
            minimum_horizon_s=kernel.H*kernel.dt,
            posture0_terminal_q_change_deg=np.rad2deg(states[12, -1, 5:11]-state[5:11]).tolist(),
            nominal_vs_slowdown_predicted_state_max_difference=float(np.max(np.abs(states[0]-states[1]))),
            nominal_first_v_intent_m_s=float(controls[0, 0, 0]),
            slowdown_first_v_intent_m_s=float(controls[1, 0, 0]),
            candidate_model_note='Nominal and .65 slowdown produce identical predicted states here: guide minimum .08*.65=.052 exceeds vmax=.05. This reduces effective candidate diversity, but fixing it cannot overcome the proven fixed44 hard-envelope infeasibility.',
            next_requirement='Establish a defensible prediction-error/motion envelope and compatible simulation operating limits before claiming feasible avoidance; zero-epsilon diagnosis is not a deployable correction.'),
        fixed44=dict(local_xyz_radius_m=model.fixed_spheres[44].tolist(), initial_world_xyz_radius_m=initial_sphere.tolist(),
            fixed_structure_margin_m=float(h[11]), planar_rotation_radius_m=rho,
            maximum_translation_speed_m_s=kernel.limits['vmax'], maximum_yaw_speed_rad_s=kernel.limits['wmax'],
            necessary_condition='gap44(t) <= min_j ||c44(0)-(p_j+v_j*age)|| + vmax*t + 2*rho*sin(min(wmax*t,pi)/2) - r44 - grid_resolution - epsilon(age)',
            proof_scope='Upper bound uses reachable-set superset; negative witness proves no arm/base control satisfying current speed limits can satisfy the fixed-sphere dynamic hard envelope at that horizon sample. It does not prove physical collision, or continuous-time safety.',
            infeasible_under_current_envelope=witness is not None, first_negative_upper_bound_witness=witness,
            trace=reachability), candidates=candidates,
        limitations=['Zero epsilon uses recorded predicted surface points, not exact physical obstacle surfaces or a calibrated uncertainty bound.',
            'No physical calibration, unseen-obstacle coverage, or real hardware acceptance is inferred from this ideal sensor simulated failure.',
            'All per-sphere/time attribution is exhaustive over the production checked samples, not continuous collision detection.',
            'Only structured19 candidates reconstructed; finite random candidates are not required for the universal fixed44 bound.'])
    files = [Path(__file__).resolve(), SOURCE/'failure_status.json', SOURCE/'failure_snapshot.npy',
             SOURCE/'navigation_events.jsonl', SOURCE/'mppi_status.jsonl', SOURCE/'parameters.yaml',
             SOURCE/'planner_params.yaml', SOURCE/'audited_source/active_arm_mppi.py', CORE/'active_arm_model.py',
             CORE/'native_reference.py', SOURCE/'audited_source/native_reference_adapter.py', DEFAULT_PROXY, DEFAULT_URDF]
    library = ROOT/'devel/lib/libmppi_position_servo.so'
    if library.is_file():
        files.append(library)
    report['sha256'] = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    report['wall_s'] = time.monotonic()-began
    report['peak_rss_mib'] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024
    (OUT/'failure_candidate_audit.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: report[k] for k in ('replay_max_absolute_errors', 'candidate_count',
        'first_negative_sources', 'zero_epsilon_diagnostic_valid_count', 'all_esdf_positive',
        'fixed44', 'wall_s', 'peak_rss_mib')}, indent=2))


if __name__ == '__main__':
    main()
