#!/usr/bin/env python3
"""Same recorded-input reset-state CABI profiling; never publishes commands.

Parent/native3D controls are tested against child DEMAND=0. DEMAND=1 is a
different policy, so equality applies to input identity, not its new actions.
Profiling overhead is separated from unprofiled timing and from online trials.
"""
import argparse
import ctypes as ct
import gzip
import hashlib
import json
import os
from pathlib import Path
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from native_collision_runtime import load_recorded_snapshot
from report_coupled_objective import lines, read

DP = ct.POINTER(ct.c_double)
PROFILE_NAMES = ['version', 'profile_active', 'effective_demand', 'plan_calls', 'plan_ns',
    'query_calls', 'query_ns', 'cache_hits', 'body_visits', 'base_plan_calls', 'base_plan_ns',
    'base_candidate_count', 'base_candidate_horizon_steps', 'v7_evaluate_calls', 'v7_evaluate_ns',
    'freepath_skipped', 'completed', 'native3d_enabled', 'reserved18', 'reserved19']


def ptr(array):
    return array.ctypes.data_as(DP)


def digest(array):
    return hashlib.sha256(np.asarray(array, dtype='<f8').tobytes()).hexdigest()


def configure(path):
    lib = ct.CDLL(str(path))
    lib.online_create.argtypes = [DP, DP, ct.c_int, ct.c_int]
    lib.online_create.restype = ct.c_void_p
    lib.online_destroy.argtypes = [ct.c_void_p]
    lib.online_reset.argtypes = [ct.c_void_p]
    lib.online_reference_changed.argtypes = [ct.c_void_p]
    lib.online_feedback.argtypes = [ct.c_void_p, ct.c_double, ct.c_double]
    lib.online_set_native_collision_snapshot.argtypes = [ct.c_void_p, DP, ct.c_int]
    lib.online_set_native_collision_clock.argtypes = [ct.c_void_p, ct.c_double]
    lib.online_step.argtypes = [ct.c_void_p, DP, DP, ct.c_int, DP, ct.c_int, ct.c_double,
        DP, ct.c_int, ct.c_int, ct.c_double, ct.c_double, ct.c_double, ct.c_int, DP]
    lib.online_step.restype = ct.c_int
    lib.online_selected_controls.argtypes = [ct.c_void_p, DP, ct.c_int]
    lib.online_selected_controls.restype = ct.c_int
    for name in ('online_debug', 'online_decision_debug', 'online_recovery_debug'):
        getattr(lib, name).argtypes = [ct.c_void_p, DP]
    lib.online_validate_execution.argtypes = [ct.c_void_p, DP, DP, ct.c_int, DP, ct.c_int,
        ct.c_double, ct.c_double, DP, ct.c_int, ct.c_int, ct.c_double, ct.c_double, ct.c_double,
        DP, ct.c_int, DP]
    lib.online_validate_execution.restype = ct.c_int
    if hasattr(lib, 'online_native_realtime_profile'):
        lib.online_native_realtime_profile.argtypes = [ct.c_void_p, DP, ct.c_int]
        lib.online_native_realtime_profile.restype = ct.c_int
    return lib


def recorded_inputs(trial):
    """Join exact solver copy-time, reference hash and native publication seq."""
    trial = Path(trial).resolve()
    cycles = [r for r in lines(trial/'state_mppi_cycles.jsonl') if r.get('online_step_succeeded')]
    if not cycles:
        raise ValueError('no actual successful CABI inputs')
    native = lines(trial/'native_collision_cycles.jsonl')
    native = {round(r['bindings']['solve']['ros_time'], 8): r for r in native if 'solve' in r.get('bindings', {})}
    refs = {r['sha256']: r for r in lines(trial/'consumed_references.jsonl')}
    diags = {round(r['t']+1000., 8): r for r in read(trial/'mppi_diagnostics.json', [])}
    raw_root = trial/'native3d_snapshots'
    snapshots = {int(r['header'][2]): r for r in lines(raw_root/'index.jsonl') if r['kind'] == 'snapshot'}
    shared_index = {int(r['sequence']): r for r in lines(trial/'collision_snapshots.jsonl')}
    selected = sorted(set((0, len(cycles)//2, len(cycles)-1)))
    result = []
    for ordinal in selected:
        cycle = cycles[ordinal]
        now = float(cycle['solve_ros_time'])
        entry = native[round(now, 8)]
        diag = diags[round(now, 8)]
        reference = np.ascontiguousarray(refs[cycle['reference_array_sha256']]['reference'], dtype=np.float64)
        if digest(reference) != cycle['reference_array_sha256']:
            raise ValueError('reference identity differs from actual solver input')

        def payload(phase):
            header = entry['bindings'][phase]['header']
            record = snapshots[int(header[2])]
            if list(record['header'][:20]) != list(header[:20]) or list(record['header'][21:]) != list(header[21:]):
                raise ValueError('recorded snapshot is not exact bound solver snapshot')
            value = load_recorded_snapshot(raw_root, record, self_contained=True)
            return value

        seq = int(diag['solve_collision_sequence'])
        with np.load(trial/'collision_snapshots'/('%05d.npz' % seq), allow_pickle=False) as archive:
            shared = np.ascontiguousarray(archive['payload'])
        if digest(shared) != shared_index[seq]['payload_sha256']:
            raise ValueError('planar proposal frame hash mismatch')
        nb, no = int(shared[11]), int(shared[12])
        obs_offset = 16+3*nb
        obs = np.ascontiguousarray(shared[obs_offset:obs_offset+6*no].reshape((-1, 6)))
        field = np.ascontiguousarray(shared[obs_offset+6*no:].reshape((int(shared[4]), int(shared[3]))))
        if not np.array_equal(obs, np.asarray(diag['observed_obstacles']).reshape((-1, 6))):
            raise ValueError('proposal actor input not identical to recorded solve')
        sample = dict(cycle_id=cycle['cycle_id'], ordinal=ordinal, source=str(trial),
            state=np.asarray(cycle['solve_state'], dtype=np.float64), reference=reference,
            validation_state=np.asarray(cycle['validation_state'], dtype=np.float64),
            native=payload('solve'), validation_native=payload('validation'),
            now=now, validation_now=entry['bindings']['validation']['ros_time'],
            validation_state_age=max(0., entry['bindings']['validation']['ros_time']-cycle['validation_odom_stamp']),
            obs=obs, field=field, gx=float(shared[5]), gy=float(shared[6]), resolution=float(shared[7]))
        sample['age'] = max(0., now-sample['native'][18])
        sample['validation_age'] = max(0., sample['validation_now']-sample['validation_native'][18])
        sample['identities'] = {name: digest(sample[name]) for name in
            ('state', 'reference', 'validation_state', 'native', 'validation_native', 'obs', 'field')}
        sample['identities']['clock'] = digest(np.array([now, sample['age'], sample['validation_now'],
            sample['validation_age'], sample['validation_state_age'], sample['gx'], sample['gy'], sample['resolution']]))
        result.append(sample)
    return result


def run_one(lib, sample, params, geometry, demand, profile):
    os.environ.update(NATIVE_COLLISION_3D='1', NATIVE_REALTIME_DEMAND=str(int(demand)),
        NATIVE_REALTIME_PROFILE=str(int(profile)), NATIVE_REALTIME_REPLAY='1')
    handle = lib.online_create(ptr(params), ptr(geometry), len(geometry), 1)
    if not handle:
        raise ValueError('CABI creation failed')
    try:
        lib.online_reset(handle)
        lib.online_reference_changed(handle)

        def bind(payload, now):
            if lib.online_set_native_collision_snapshot(handle, ptr(payload), len(payload)):
                raise ValueError('snapshot refused')
            if lib.online_set_native_collision_clock(handle, now):
                raise ValueError('clock refused')

        bind(sample['native'], sample['now'])
        out = np.zeros(8, dtype=np.float64)
        begin, cpu = time.perf_counter_ns(), time.thread_time_ns()
        code = lib.online_step(handle, ptr(sample['state']), ptr(sample['reference']), len(sample['reference']),
            ptr(sample['obs']), len(sample['obs']), sample['age'], ptr(sample['field']), sample['field'].shape[1],
            sample['field'].shape[0], sample['gx'], sample['gy'], sample['resolution'], 0, ptr(out))
        elapsed, cpu = (time.perf_counter_ns()-begin)/1e6, (time.thread_time_ns()-cpu)/1e6
        if code != 0 or not np.isfinite(out).all():
            raise ValueError('offline plan failed: '+str(code))
        count = lib.online_selected_controls(handle, None, 0)
        controls = np.zeros((count, 2), dtype=np.float64)
        lib.online_selected_controls(handle, ptr(controls), count)
        diagnostics = {}
        for name, size in [('online_debug', 8), ('online_decision_debug', 12), ('online_recovery_debug', 10)]:
            values = np.zeros(size, dtype=np.float64)
            getattr(lib, name)(handle, ptr(values))
            diagnostics[name] = values.tolist()
        stats = None
        if hasattr(lib, 'online_native_realtime_profile'):
            values = np.zeros(len(PROFILE_NAMES), dtype=np.float64)
            if lib.online_native_realtime_profile(handle, ptr(values), len(values)) != len(values):
                raise ValueError('wrong profiling schema')
            stats = dict(zip(PROFILE_NAMES, values.tolist()))
        bind(sample['validation_native'], sample['validation_now'])
        checked = np.zeros((650, 8), dtype=np.float64)
        report = np.zeros(6, dtype=np.float64)
        validation_code = lib.online_validate_execution(handle, ptr(sample['validation_state']), ptr(controls), count,
            ptr(sample['obs']), len(sample['obs']), sample['validation_age'], sample['validation_state_age'],
            ptr(sample['field']), sample['field'].shape[1], sample['field'].shape[0], sample['gx'], sample['gy'],
            sample['resolution'], ptr(checked), len(checked), ptr(report))
        return dict(code=int(code), action=out[:7].tolist(), controls=controls.tolist(), diagnostics=diagnostics,
            kernel_ms=float(out[7]), wall_ms=elapsed, thread_cpu_ms=cpu, profile=stats,
            validation_code=int(validation_code), validation_report=report.tolist(),
            input_sha256=sample['identities'], demand=demand, profiling=profile)
    finally:
        lib.online_destroy(handle)


def numeric_output(record):
    return {key: record[key] for key in ('code', 'action', 'controls', 'diagnostics')}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', type=Path, required=True)
    parser.add_argument('--parent', type=Path, default=HERE/'builds/native_collision_3d_20261027_04')
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if args.out.exists():
        raise FileExistsError('New output required')
    samples = recorded_inputs(args.trial)
    params = np.asarray(list(read(args.trial/'limits.json').values()), dtype=np.float64)
    geometry = np.ascontiguousarray(np.loadtxt(args.trial/'geometry.csv', delimiter=','), dtype=np.float64)
    parent = configure(args.parent/'build/libonline_mppi.so')
    child = configure(args.build/'build/libonline_mppi.so')
    records, checks = [], []
    for sample in samples:
        matrix = {}
        for name, lib, demand, profile in [('parent', parent, False, False), ('off', child, False, False),
                ('off_profile', child, False, True), ('on', child, True, False), ('on_profile', child, True, True)]:
            matrix[name] = run_one(lib, sample, params, geometry, demand, profile)
        checks.append(dict(cycle_id=sample['cycle_id'],
            off_exact_parent=numeric_output(matrix['parent']) == numeric_output(matrix['off']),
            off_profile_numeric_invariant=numeric_output(matrix['off']) == numeric_output(matrix['off_profile']),
            on_profile_numeric_invariant=numeric_output(matrix['on']) == numeric_output(matrix['on_profile']),
            all_exact_same_inputs=len({json.dumps(r['input_sha256'], sort_keys=True) for r in matrix.values()}) == 1,
            off_and_on_execution_passed=all(matrix[n]['validation_code'] == 0 for n in ('off', 'on')),
            profile_schema_valid=all(matrix[n]['profile']['version'] == 1 and matrix[n]['profile']['completed'] == 1
                and matrix[n]['profile']['profile_active'] == 1 for n in ('off_profile', 'on_profile'))))
        records.append(dict(cycle_id=sample['cycle_id'], matrix=matrix))
    passed = all(all(v is True for k, v in check.items() if k != 'cycle_id') for check in checks)
    result = dict(passed=passed, checks=checks, records=records,
        scope='actual recorded inputs, reset controller/RNG (seed1) per variant, not a full historical warm-state replay; no actuation',
        timer_note='profile times nested/inclusive, cannot add Base/query/V7 to plan; only unprofiled times assess raw replay runtime, online deadlines separate',
        libraries={name: dict(path=str(p.resolve()), sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for name, p in
            [('parent', args.parent/'build/libonline_mppi.so'), ('child', args.build/'build/libonline_mppi.so')]})
    args.out.mkdir(parents=True, exist_ok=False)
    (args.out/'replay.json').write_text(json.dumps(result, indent=2)+'\n')
    for record in records:
        for name, value in record['matrix'].items():
            print(record['cycle_id'], name, 'kernel_ms', round(value['kernel_ms'], 3), 'validation', value['validation_code'])
    print('CHECKS', json.dumps(checks), 'PASSED', passed)
    if not passed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
