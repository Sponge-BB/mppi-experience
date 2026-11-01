#!/usr/bin/env python3
"""Paired full-native MPPI cusp experiment; no ROS or command publication.

The old trial did not save its full 3-D field.  The saved measured stop state,
903-point reference, 67 collision spheres and limits are retained, but the
common open 3-D field below is newly constructed and labelled as such.  This
is a causal kernel test, not an exact historical perception replay.
"""
import argparse
import ctypes as ct
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import time

import numpy as np

from audits.replay_native_realtime import configure, ptr

HERE = Path(__file__).resolve().parent
TRIAL = HERE/'results/cheap_scaling_online_20261030_01/midcross_cheap_seed11'
PARENT = HERE/'builds/cheap_scaling_20261030_01'
NAMES = ('vmax', 'wmax', 'accel', 'angular_accel', 'track', 'wheel_v', 'wheel_a',
         'margin', 'max_age', 'max_prediction', 'pos_unc', 'vel_unc', 'acc_unc',
         'goal_pos', 'goal_yaw', 'goal_speed')
DEBUG_NAMES = ('has_reference', 'phase', 'phase_count', 'begin', 'end', 'gear',
               'endpoint_distance', 'endpoint_yaw_error', 'remaining_arc',
               'transitions', 'event', 'confirmations', 'reference_hash_hi', 'reference_hash_lo')
DP = ct.POINTER(ct.c_double)


def digest(x):
    return hashlib.sha256(np.asarray(x, dtype='<f8').tobytes()).hexdigest()


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def load_fixture():
    refrecord = json.loads((TRIAL/'consumed_references.jsonl').read_text().splitlines()[-1])
    ref = np.ascontiguousarray(refrecord['reference'], dtype=np.float64)
    if digest(ref) != refrecord['sha256']:
        raise ValueError('Saved numeric reference hash mismatch')
    state = np.asarray(read(TRIAL/'result.json')['final_state'], dtype=np.float64)
    config = read(TRIAL/'limits.json')
    body_record = read(TRIAL/'native_collision_3d_body.json')
    body = np.ascontiguousarray(body_record['body'], dtype=np.float64)
    if body.shape != (67, 7):
        raise ValueError('Expected unchanged 67-sphere original geometry')
    binding = json.loads((TRIAL/'native_collision_cycles.jsonl').read_text().splitlines()[-1])['bindings']['solve']
    header = np.asarray(binding['header'], dtype=np.float64)
    now = float(binding['ros_time'])
    nx, ny, nz = map(int, header[5:8])
    gx, gy, gz, resolution = header[8:12]
    x = gx+(np.arange(nx)+.5)*resolution
    y = gy+(np.arange(ny)+.5)*resolution
    # Newly constructed open box: vertical walls at crop boundaries.  Ground
    # and fixed-arm self checks retain their original explicit native policy.
    plane = np.minimum(np.minimum(x[:, None]-gx, gx+nx*resolution-x[:, None]),
                       np.minimum(y[None, :]-gy, gy+ny*resolution-y[None, :]))
    field3d = np.ascontiguousarray(np.broadcast_to(plane[:, :, None], (nx, ny, nz)))
    native = np.r_[header, body.ravel(), field3d.ravel()]
    native[20] = 1.
    field2d = np.ascontiguousarray(plane.T)
    with gzip.open(TRIAL/'mppi_diagnostics.json.gz', 'rt') as stream:
        last = json.load(stream)[-1]
    return dict(ref=ref, state=state, config=config,
        params=np.array([config[n] for n in NAMES], dtype=np.float64), body=body,
        geometry=np.ascontiguousarray(np.loadtxt(TRIAL/'geometry.csv', delimiter=',')),
        native=np.ascontiguousarray(native), now=now, field=field2d,
        gx=float(x[0]), gy=float(y[0]), resolution=float(resolution),
        expected_stop_cost=float(last['local_phase']['decision'][9]),
        expected_horizon_s=float(last['local_phase']['decision'][2]),
        constructed_perception=dict(source='NEW analytic vertical walls at old crop boundaries; not recorded old ESDF',
            has_full_esdf=True, original_crop_header=header.tolist(), native_body_sha256=digest(body),
            native_field_sha256=digest(field3d), native_payload_sha256=digest(native),
            body_count=len(body), dynamic_points=0, resolution_m=float(resolution)))


def prepare_library(build):
    manifest = read(build/'build_manifest.json')
    library = build/'build/libonline_mppi.so'
    if sha(library) != manifest['mppi_library_sha256']:
        raise ValueError('Built library hash mismatch')
    lib = configure(library)
    lib.online_accept_execution_controls.argtypes = [ct.c_void_p, DP, ct.c_int]
    lib.online_accept_execution_controls.restype = ct.c_int
    lib.online_dynamic_strategy_debug.argtypes = [ct.c_void_p, DP]
    lib.online_search_trace.argtypes = [ct.c_void_p, DP, ct.c_int]
    lib.online_search_trace.restype = ct.c_int
    lib.online_native3d_query.argtypes = [ct.c_void_p, ct.c_double, ct.c_double, ct.c_double,
                                         ct.c_double, ct.c_int, DP]
    lib.online_native3d_query.restype = ct.c_int
    if hasattr(lib, 'online_cusp_phase_debug'):
        lib.online_cusp_phase_debug.argtypes = [ct.c_void_p, DP]
        lib.online_cusp_phase_debug.restype = ct.c_int
    return lib


class Kernel:
    def __init__(self, lib, fixture, enabled):
        self.lib, self.f = lib, fixture
        os.environ.update(NATIVE_COLLISION_3D='1', NATIVE_REALTIME_DEMAND='1',
            NATIVE_DYNAMIC_FAST='1', NATIVE_REALTIME_PROFILE='0', NATIVE_REALTIME_REPLAY='1',
            MPPI_CUSP_PHASE_ASSOCIATION=str(int(enabled)))
        self.handle = lib.online_create(ptr(fixture['params']), ptr(fixture['geometry']), len(fixture['geometry']), 11)
        if not self.handle:
            raise ValueError('online_create failed')
        self.sequence = 0
        self.lib.online_reset(self.handle)
        # Initialise the observed historical branch with existing public APIs,
        # without actuation: both A/B observe the same remote moving track once.
        # It only sets d_had_dynamic.  The actual tested frames contain no track.
        self.bind(0., True)
        self.step(fixture['state'], np.array([[1e4, 1e4, .1, 0., .23, 1.]]), inspect=False)
        self.lib.online_reference_changed(self.handle)
        zeros = np.zeros((28, 2))
        if self.lib.online_accept_execution_controls(self.handle, ptr(zeros), len(zeros)):
            raise ValueError('Cannot initialise observed zero warm sequence')
        self.lib.online_feedback(self.handle, 0., 0.)

    def close(self):
        self.lib.online_destroy(self.handle)

    def bind(self, elapsed, full=False):
        self.sequence += 1
        self.elapsed = elapsed
        self.now = self.f['now']+elapsed
        payload = self.f['native'].copy() if full else self.f['native'][:32+67*7].copy()
        payload[2] += self.sequence
        for index in (3, 17, 18, 24):
            payload[index] += elapsed
        payload[20] = float(full)
        self.age = self.now-payload[18]
        if self.lib.online_set_native_collision_snapshot(self.handle, ptr(payload), len(payload)):
            raise ValueError('Full native 3-D snapshot bind failed')
        if self.lib.online_set_native_collision_clock(self.handle, self.now):
            raise ValueError('Native clock bind failed')

    def step(self, state, obs=None, inspect=True):
        f, lib, h = self.f, self.lib, self.handle
        obs = np.zeros((0, 6)) if obs is None else np.ascontiguousarray(obs)
        out = np.zeros(8)
        wall = time.perf_counter()
        code = lib.online_step(h, ptr(state), ptr(f['ref']), len(f['ref']), ptr(obs), len(obs), self.age,
            ptr(f['field']), f['field'].shape[1], f['field'].shape[0], f['gx'], f['gy'], f['resolution'], 0, ptr(out))
        if code:
            raise ValueError('Native MPPI plan rejected: '+str(code))
        if not inspect:
            return None
        count = lib.online_selected_controls(h, None, 0)
        controls = np.zeros((count, 2))
        lib.online_selected_controls(h, ptr(controls), count)
        decision, strategy, phase = np.zeros(12), np.zeros(4), np.zeros(14)
        lib.online_decision_debug(h, ptr(decision))
        lib.online_dynamic_strategy_debug(h, ptr(strategy))
        if hasattr(lib, 'online_cusp_phase_debug'):
            lib.online_cusp_phase_debug(h, ptr(phase))
        trace_n = lib.online_search_trace(h, None, 0)
        trace = np.zeros((trace_n, 8))
        lib.online_search_trace(h, ptr(trace), trace_n)
        candidates = [dict(index=int(r[0]), cost=float(r[4]), gap=float(r[5]), feasible=bool(r[6]), selected=bool(r[7]))
                      for r in trace if r[1] == 0.]
        rows, report = np.zeros((650, 8)), np.zeros(6)
        validation = lib.online_validate_execution(h, ptr(state), ptr(controls), count, ptr(obs), len(obs), self.age,
            0., ptr(f['field']), f['field'].shape[1], f['field'].shape[0], f['gx'], f['gy'], f['resolution'],
            ptr(rows), len(rows), ptr(report))
        return dict(code=code, command=out[:2].tolist(), controls=controls.tolist(),
            decision=decision.tolist(), dynamic_strategy=strategy.tolist(), phase=dict(zip(DEBUG_NAMES, phase.tolist())),
            candidates=candidates, validation_code=validation, validation=report.tolist(),
            kernel_ms=float(out[7]), wall_ms=(time.perf_counter()-wall)*1000.)


def plant(state, control, duration, c):
    s, u = state.copy(), np.asarray(control, dtype=float).copy()
    wheels = lambda x: (x[0]-.5*c['track']*x[1], x[0]+.5*c['track']*x[1])
    u /= max(1., abs(u[0])/c['vmax'], abs(u[1])/c['wmax'],
             *(abs(v)/c['wheel_v'] for v in wheels(u)))
    n = max(1, math.ceil(duration/.01-1e-10))
    h = duration/n
    for unused in range(n):
        a = (u-s[3:])/.12
        a /= max(1., abs(a[0])/c['accel'], abs(a[1])/c['angular_accel'],
                 *(abs(v)/c['wheel_a'] for v in wheels(a)))
        vn, wn = s[3:]+h*a
        vm, wm = .5*(s[3]+vn), .5*(s[4]+wn)
        angle = s[2]+.5*h*wm
        s[0] += h*vm*math.cos(angle)
        s[1] += h*vm*math.sin(angle)
        s[2] = math.atan2(math.sin(s[2]+h*wm), math.cos(s[2]+h*wm))
        s[3:] = vn, wn
    return s


def closed_loop(lib, f, enabled, start, duration):
    kernel = Kernel(lib, f, enabled)
    state, records, reached = start.copy(), [], False
    try:
        for i in range(int(duration/.05)+1):
            t = i*.05
            kernel.bind(t)
            result = kernel.step(state)
            records.append(dict(t=t, state=state.tolist(), **result))
            if result['validation_code']:
                break
            goal = np.linalg.norm(state[:2]-f['ref'][-1, :2])
            yaw = abs(math.atan2(math.sin(state[2]-f['ref'][-1, 2]), math.cos(state[2]-f['ref'][-1, 2])))
            if goal <= f['config']['goal_pos'] and yaw <= f['config']['goal_yaw'] and abs(state[3]) <= f['config']['goal_speed']:
                reached = True
                break
            state = plant(state, result['command'], .05, f['config'])
            kernel.lib.online_feedback(kernel.handle, *result['command'])
        return dict(goal_reached=reached, duration_s=records[-1]['t'], final_state=state.tolist(),
            final_goal_distance_m=float(np.linalg.norm(state[:2]-f['ref'][-1, :2])),
            validation_failures=sum(bool(r['validation_code']) for r in records),
            minimum_native_gap_m=min(r['validation'][1] for r in records),
            initial_phase=records[0]['phase'], final_phase=records[-1]['phase'],
            negative_commands=sum(r['command'][0]<-1e-4 for r in records),
            positive_commands=sum(r['command'][0]>1e-4 for r in records), records=records)
    finally:
        kernel.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=25.)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    f = load_fixture()
    child, parent = prepare_library(args.build), prepare_library(PARENT)
    np.savez_compressed(args.out/'common_inputs.npz', reference=f['ref'], state=f['state'],
        native_snapshot=f['native'], field=f['field'], geometry=f['geometry'], params=f['params'])
    fixed = {}
    for name, lib, enabled in (('parent', parent, False), ('off', child, False), ('on', child, True)):
        kernel = Kernel(lib, f, enabled)
        try:
            kernel.bind(0.)
            fixed[name] = kernel.step(f['state'])
        finally:
            kernel.close()
    for name in ('parent', 'off'):
        if abs(fixed[name]['decision'][9]-f['expected_stop_cost'])>1e-12 or np.linalg.norm(fixed[name]['command'])>1e-10:
            raise ValueError('Native A does not reproduce the recorded stop and merit: '+name)
    unchanged = all(fixed['parent'][k] == fixed['off'][k] for k in ('command', 'controls', 'decision', 'dynamic_strategy', 'validation_code', 'validation'))
    if not unchanged:
        raise ValueError('Default-off child changed the old full-native kernel output')
    summary = dict(scope='Full native MPPI kernel and original full-3D rollout/brake gates; synchronous numerical plant, no ROS/hardware',
        old_complete_perception_replay=False, constructed_perception=f['constructed_perception'],
        mode_initialization='One common non-executed remote moving observation, reference_changed, zero warm and feedback; tests use empty dynamic observations',
        reference_sha256=digest(f['ref']), state_sha256=digest(f['state']), expected_stop_cost=f['expected_stop_cost'],
        default_off_exact_parent=unchanged, fixed=fixed,
        source_files={n:sha(args.build/'source'/n) for n in ('online_kernel.hpp', 'mppi_v12.hpp', 'online_api.cpp', 'cusp_phase_cursor.hpp')},
        library_sha256=sha(args.build/'build/libonline_mppi.so'),
        timing_note='No wall-clock execution deadline simulated; this experiment isolates association and native feasibility, online trials test deadlines separately',
        branches={})
    (args.out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    # Both late-stop branches retain the identical measured start.  A second
    # B fixture starts in the legitimate reverse gear before the final cusp.
    near = np.r_[f['ref'][800, :3], f['ref'][800, 3], 0.]
    cases = [('late_off', False, f['state']), ('late_on', True, f['state']), ('reverse_to_forward_on', True, near)]
    for name, enabled, state in cases:
        answer = closed_loop(child, f, enabled, state, args.duration)
        records = answer.pop('records')
        with (args.out/(name+'.jsonl')).open('w') as stream:
            for row in records:
                stream.write(json.dumps(row)+'\n')
        summary['branches'][name] = answer
        (args.out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        print('CUSP_NATIVE', name, json.dumps(answer), flush=True)
    print('CUSP_NATIVE_COMPLETE', args.out, flush=True)


if __name__ == '__main__':
    main()
