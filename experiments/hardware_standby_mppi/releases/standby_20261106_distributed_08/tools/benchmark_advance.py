#!/usr/bin/env python3
"""Bounded offline comparison of the two advance clipping expressions.

No ROS or devices. The replay combines a recorded state with the final saved
cloud snapshot and a synthetic clock; it is not an exact live input cohort.
"""
import os
import sys
import resource

sys.dont_write_bytecode = True
for variable in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[variable] = '1'
resource.setrlimit(resource.RLIMIT_AS, (512 * 1024 ** 2, 512 * 1024 ** 2))

import ast
import hashlib
import json
import statistics
import time
import types
from pathlib import Path

import numpy as np


def main():
    release = Path(__file__).resolve().parents[1]
    frozen = release.parent / 'standby_20261106_stationary_07'
    sys.path[:0] = [str(release / 'runtime'), str(frozen / 'core'),
                    str(frozen / 'runtime')]
    import active_arm_mppi
    from active_arm_model import PiperArmModel
    from standby_whole_body_mppi import StandbyWholeBodyMPPI
    from hardware_cloud_snapshot import (SurfaceObservation, DynamicObservation,
                                         HardwareCloudSnapshot)
    from mppi_command_gateway import NativeLeaseValidator

    baseline_path = frozen / 'core/active_arm_mppi.py'
    source = ast.parse(baseline_path.read_text())
    cls = next(n for n in source.body if isinstance(n, ast.ClassDef)
               and n.name == 'WholeBodyMPPI')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef)
                  and n.name == 'advance')
    namespace = dict(vars(active_arm_mppi))
    exec(compile(ast.Module(body=[method], type_ignores=[]),
                 str(baseline_path), 'exec'), namespace)
    old, new = namespace['advance'], active_arm_mppi.WholeBodyMPPI.advance

    config = json.loads((frozen / 'config/hardware_shadow.json').read_text())
    arm = config['arm_limits']
    model = PiperArmModel.from_parameters(frozen / 'model/parameters.yaml',
                                         frozen / 'model/mm.urdf')

    def kernel():
        return StandbyWholeBodyMPPI(model, config['limits'],
            [arm[x] for x in ('lower', 'upper', 'velocity', 'acceleration')],
            config['postures'], horizon=8, random_samples=4)

    instance = kernel()
    rng = np.random.RandomState(55)
    cases, maximum_error = 0, 0.
    for shape in ((17,), (1, 17), (32, 17)):
        for dt in (.001, .01, .05, .1):
            for _ in range(20):
                state = rng.normal(size=shape)
                control = rng.normal(size=shape[:-1] + (8,))
                before, after = old(instance, state, control, dt), new(instance, state, control, dt)
                maximum_error = max(maximum_error, float(np.max(np.abs(before-after))))
                assert np.array_equal(before, after), 'advance output changed'
                cases += 1

    run = frozen / 'reports/actual_recording_13'
    rows = json.loads((run / 'shadow_status.json').read_text())
    row = next(r for r in rows if r['proposed_controls'] is not None
               and r['command_gateway']['reason'] == 'expired_during_revalidation')
    state, control = np.array(row['state']), np.array(row['proposed_controls'])
    reference = np.array([[*row['anchor'], 0.]])
    yaw = state[2]
    rotation = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                         [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])

    def observation(points, name):
        return SurfaceObservation.create(points, 100., 'odom', 'odom', name,
                                          max_points=15000 if name == 'static' else 4000)

    with np.load(run / 'snapshot.npz') as recording:
        raw = recording['raw']
        raw = raw[np.any(raw != 0, axis=1)]
        raw = (raw + [.28, 0, .117]) @ rotation.T + np.r_[state[:2], 0]
        filtered = (recording['filtered'] + [.28, 0, .117]) @ rotation.T + np.r_[state[:2], 0]
        snapshot = HardwareCloudSnapshot(observation(raw, 'raw'),
            observation(filtered, 'filtered'), observation(recording['background'], 'static'),
            DynamicObservation.create([], 100., 'odom', 'odom'), 100., True)

    before, after = kernel(), kernel()
    before.advance = types.MethodType(old, before)
    validators = {'before': NativeLeaseValidator(before), 'after': NativeLeaseValidator(after)}
    validators['before'].kernel.advance = types.MethodType(old, validators['before'].kernel)
    kernels = {'before': before, 'after': after}
    times = {name: {'final_ms': [], 'plan_ms': []} for name in kernels}
    control_error, trajectory_error, gap_error = 0., 0., 0.
    for iteration in range(16):
        outputs = {}
        for name in (('before', 'after') if iteration % 2 else ('after', 'before')):
            validator, current = validators[name], kernels[name]
            start = time.perf_counter()
            valid = validator(state, control, snapshot, 100.1, .14)
            times[name]['final_ms'].append(1000 * (time.perf_counter()-start))
            start = time.perf_counter()
            planned, info = current.plan(state, reference, state[5:11], snapshot, 100.1)
            times[name]['plan_ms'].append(1000 * (time.perf_counter()-start))
            outputs[name] = (planned, info, validator.last_min_gap, valid)
        first, second = outputs['before'], outputs['after']
        assert first[3] == second[3] and first[1]['feasible'] == second[1]['feasible']
        assert (first[0] is None) == (second[0] is None)
        if first[0] is not None:
            control_error = max(control_error, float(np.max(np.abs(first[0][0]-second[0][0]))))
            trajectory_error = max(trajectory_error, float(np.max(np.abs(first[0][1]-second[0][1]))))
            assert np.array_equal(first[0][0], second[0][0])
            assert np.array_equal(first[0][1], second[0][1])
        gap_error = max(gap_error, abs(first[2]-second[2]))
        assert first[2] == second[2], 'final minimum gap changed'

    summary = {name: {metric: {'median': statistics.median(values), 'maximum': max(values)}
                     for metric, values in metrics.items()} for name, metrics in times.items()}
    report = dict(scope='approximate_offline_replay_no_ros_no_hardware',
        change='two np.clip expressions in advance replaced with minimum(maximum)',
        baseline=str(baseline_path),
        baseline_sha256=hashlib.sha256(baseline_path.read_bytes()).hexdigest(),
        optimized=str(release / 'runtime/active_arm_mppi.py'),
        optimized_sha256=hashlib.sha256((release / 'runtime/active_arm_mppi.py').read_bytes()).hexdigest(),
        equivalence_cases=cases, maximum_advance_error=maximum_error,
        replay_iterations=16, maximum_control_error=control_error,
        maximum_trajectory_error=trajectory_error, maximum_final_gap_error=gap_error,
        timing_ms=summary,
        median_total_saving_ms=sum(summary['before'][key]['median']-summary['after'][key]['median']
                                   for key in ('final_ms', 'plan_ms')),
        peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024.,
        address_space_limit_mib=512, numerical_threads=1,
        checkpoints_deadlines_and_source_stamps_changed=False,
        continuous_live_delivery_passed=False,
        limitations=['recorded state plus final saved cloud snapshot, not one synchronized live cohort',
                     'synthetic clock; proposed control, not captured post-guard output',
                     'does not measure ROS contention, physical response or remote RViz load'])
    output = release / 'reports/timing_optimization.json'
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
