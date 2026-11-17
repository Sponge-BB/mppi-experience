#!/usr/bin/python3
"""Offline current-kernel speed/forecast audit; no ROS, SDK or hardware imports."""
import os
for name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[name] = '1'
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = next(p for p in HERE.parents if (p/'src/mm_remani/scripts/mppi/active_arm_mppi.py').is_file())
SOURCE = ROOT/'src/mm_remani/scripts/mppi/active_arm_mppi.py'
sys.path.insert(0, str(SOURCE.parent))
from active_arm_mppi import Snapshot, WholeBodyMPPI


def collision_snapshot(now, source_age):
    """A distant dynamic point isolates the real query's temporal rejection."""
    header = np.zeros(32)
    header[:2] = 3., 1.
    header[3:5] = now-source_age, 1.
    header[5:8] = 8, 8, 8
    header[8:11], header[11] = (-1., -1., -1.), .5
    header[12:17] = 47, 1, 1, .35, 2.
    header[18:21] = now-source_age, 1., 1.
    header[28] = 1.
    point = [100., 100., 100., 0., 0., 0., 1.]
    return Snapshot.decode(np.r_[header, np.zeros(47*7), point, np.full(8**3, 10.)])


def inspect(vmax):
    limits = dict(vmax=vmax, wmax=.35, accel=.25, angular_accel=.8,
                  track=.37, wheel_v=.2296, wheel_a=.2975)
    kernel = WholeBodyMPPI(None, limits,
        ([-3.]*6, [3.]*6, [.15]*6, [.35]*6), [], horizon=8, random_samples=0)
    v, w = np.meshgrid(np.linspace(-vmax, vmax, 151), np.linspace(-.35, .35, 151))
    states = np.zeros((v.size, 17))
    states[:, 3:5] = np.c_[v.ravel(), w.ravel()]
    feasible = np.maximum(np.abs(states[:, 3]-.185*states[:, 4]),
                          np.abs(states[:, 3]+.185*states[:, 4])) <= .2296+1e-12
    states = states[feasible]
    initial = states.copy()
    counts = np.zeros(len(states), dtype=int)
    distances = np.zeros(len(states))
    native_max_error = 0.
    zero = np.zeros((len(states), 8))
    for step in range(40):
        before = states
        states = kernel.advance(before, zero, .05)
        reference = kernel._advance_legacy_numpy(before, zero, .05)
        native_max_error = max(native_max_error, float(np.max(np.abs(states-reference))))
        distances += np.linalg.norm(states[:, :2]-before[:, :2], axis=1)
        stopped = np.max(np.abs(states[:, 3:5]), axis=1) < .003
        counts[(counts == 0) & stopped] = step+1
    assert np.all(counts > 0)
    assert native_max_error < 1e-12
    tail = float(counts.max()*.05)

    # A bound for the entire continuous admissible velocity set, not just grid.
    # Zero command scales both v,w by the same positive factor each half-step.
    bmax = max(vmax/.25, .35/.8, min(.2296, vmax+.185*.35)/.2975)
    b = bmax
    for step in range(40):
        b *= 1.-.05/max(.12, b)
        quiet_bound = max(vmax, .35)*b/bmax
        if quiet_bound < .003:
            analytic_tail = (step+1)*.05
            break
    assert tail <= analytic_tail+1e-12
    now, age = 10., .35
    snapshot = collision_snapshot(now, age)
    assert snapshot.fresh(now)
    sphere = np.array([[[0., 0., 0., .01]]])
    forecasts = []
    for horizon in (.8, .7):
        times = now+.05*np.arange(round((horizon+tail)/.05)+1)
        gaps, _ = snapshot.query(np.repeat(sphere, len(times), axis=0), times)
        forecasts.append(dict(horizon_s=horizon,
            source_age_plus_horizon_and_tail_s=age+horizon+tail,
            minimum_gap_m=float(gaps.min()), all_queries_accepted=bool(np.all(gaps >= 0))))
    worst = int(counts.argmax())
    return dict(limits=limits, grid_samples=len(states), max_model_base_brake_s=tail,
        worst_grid_initial_v_w=initial[worst, 3:5].tolist(),
        max_integrated_base_brake_distance_m=float(distances.max()),
        analytic_base_brake_upper_bound_s=analytic_tail,
        analytic_zero_command_time_norm_bound_s=bmax,
        final_component_bound_at_analytic_tail=quiet_bound,
        native_library_loaded=kernel._legacy_native is not None,
        native_vs_numpy_max_abs_error=native_max_error,
        forecast_queries_at_max_source_age=forecasts,
        first_command_hold_plus_brake_plus_source_age_s=.30+tail+age)


def inspect_epoch_boundary():
    cases = []
    for offset in (0., .0000001, .0000005, .001, .123456, .35, .9999999):
        now = 1794310000.+offset
        for source_step in (-1, 0, 1):
            snapshot = collision_snapshot(now, .35)
            source = snapshot.header[18]
            if source_step:
                source = np.nextafter(source, np.inf if source_step > 0 else -np.inf)
            snapshot.header[3] = snapshot.header[18] = source
            times = now+.05*np.arange(34)  # Exact evaluate indexing: H7 + .95s tail.
            spheres = np.tile([[[0., 0., 0., .01]]], (len(times), 1, 1))
            gaps, _ = snapshot.query(spheres, times)
            cases.append(dict(now=now, source_step_ulp=source_step,
                source_age_s=float(now-source), fresh=snapshot.fresh(now),
                maximum_prediction_age_s=float(times[-1]-source),
                query_accepted=bool(np.all(gaps >= 0))))
    false_rejections = [c for c in cases if c['fresh'] and not c['query_accepted']]
    result = dict(cases=cases, fresh_count=sum(c['fresh'] for c in cases),
                  fresh_false_rejections=false_rejections, production_tolerances_changed=False)
    (HERE/'speed_020_epoch_boundary.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


def main():
    current, proposed = inspect(.12), inspect(.20)
    assert current['forecast_queries_at_max_source_age'][0]['all_queries_accepted']
    assert not proposed['forecast_queries_at_max_source_age'][0]['all_queries_accepted']
    assert proposed['forecast_queries_at_max_source_age'][1]['all_queries_accepted']
    report = dict(applied_to_production=False, current=current, proposed=proposed,
        recommendation={
            'minimum_full_coverage_config_changes': {'base_limits/vmax': .20, 'horizon': 7},
            'unchanged': ['dt=.1/rate=10Hz', 'wmax=.35', 'accel=.25', 'angular_accel=.8',
                'wheel_v=.2296', 'wheel_a=.2975', 'source/map age .35s',
                'dynamic forecast 2s', 'command hold .30s', 'deadline .095s',
                'full braking tail and collision/feedback/authority checks'],
            'tradeoff': 'Nominal control horizon changes from .8s to .7s; the full stopping tail is still checked. The full-age temporal budget is exactly 2s, without slack beyond the existing numeric comparison tolerance.',
            'vmax_only_alternative': 'Changing only vmax to .2 is fail-closed, but worst-speed candidates with dynamic source age above .25s exceed the 2s forecast and are rejected; do not claim full-profile usability at every still-fresh observation.',
            'no_extra_acceleration_claim': 'Do not increase deceleration parameters to make the arithmetic pass; no new hardware response capability was measured.'},
        normal_arm_tail_s=.45,
        ros_epoch_boundary=inspect_epoch_boundary(),
        crop_review={'half_extent_m': 2.,
            'proposed_nominal_translation_plus_brake_upper_bound_m':
                .2*.7+proposed['max_integrated_base_brake_distance_m'],
            'condition': 'Add actual whole-body sphere extent and map-boundary margin. All queried spheres still require map inclusion; no crop/ESDF margin change is recommended.'},
        limitations=['Bound assumes horizon-end base velocities obey the profile; measured overspeed is never clipped away and can still fail.',
            'Normal arm speeds <=.15rad/s brake in .45s at the unchanged .35rad/s^2 model. Higher measured dq remains independently checked and can exceed the available forecast or joint limits.',
            'The distant dynamic point tests real Snapshot.query time coverage, not obstacle-avoidance success or perception quality.',
            'Pure offline model arithmetic; actual deceleration and fixed_vel=1 arm response remain uncalibrated.'],
        sources={'kernel':str(SOURCE.relative_to(ROOT)),
                 'kernel_sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
                 'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()})
    output = HERE/'speed_020_review.json'
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'report':str(output), 'current_tail_s':current['max_model_base_brake_s'],
        'proposed_tail_s':proposed['max_model_base_brake_s'],
        'proposed_queries':proposed['forecast_queries_at_max_source_age']}, indent=2))


if __name__ == '__main__':
    main()
