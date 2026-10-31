#!/usr/bin/env python3
"""Replay real native static parity snapshots through the compiled MPPI ABI.

Read-only with respect to trial/build evidence. No ROS, simulation or actuation.
The separate output also tests execution denial when only native 3-D dynamic
points exist (the legacy planar obstacle list is empty).
"""
import argparse
import ctypes as ct
import hashlib
import json
import math
import os
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from native_collision_runtime import HEADER, load_recorded_snapshot

DP = ct.POINTER(ct.c_double)


def pointer(a):
    return a.ctypes.data_as(DP)


def audit(trial, build):
    trial, build = Path(trial).resolve(), Path(build).resolve()
    root = trial/'native3d_snapshots'
    records = [json.loads(row) for row in (root/'index.jsonl').read_text().splitlines()]
    parity_records = [row for row in records if row['kind'] == 'parity']
    if not parity_records:
        raise ValueError('no actual native parity evidence')
    os.environ['NATIVE_COLLISION_3D'] = '1'
    library = build/'build/libonline_mppi.so'
    lib = ct.CDLL(str(library))
    lib.online_create.argtypes = [DP, DP, ct.c_int, ct.c_int]
    lib.online_create.restype = ct.c_void_p
    lib.online_destroy.argtypes = [ct.c_void_p]
    lib.online_set_native_collision_snapshot.argtypes = [ct.c_void_p, DP, ct.c_int]
    lib.online_set_native_collision_snapshot.restype = ct.c_int
    lib.online_set_native_collision_clock.argtypes = [ct.c_void_p, ct.c_double]
    lib.online_set_native_collision_clock.restype = ct.c_int
    lib.online_native3d_query.argtypes = [ct.c_void_p] + [ct.c_double]*4 + [ct.c_int, DP]
    lib.online_native3d_query.restype = ct.c_int
    lib.online_validate_execution.argtypes = [ct.c_void_p, DP, DP, ct.c_int, DP,
        ct.c_int, ct.c_double, ct.c_double, DP, ct.c_int, ct.c_int,
        ct.c_double, ct.c_double, ct.c_double, DP, ct.c_int, DP]
    lib.online_validate_execution.restype = ct.c_int
    params = np.array(list(json.loads((trial/'limits.json').read_text()).values()), dtype=np.float64)
    body2d = np.ascontiguousarray(np.loadtxt(trial/'geometry.csv', delimiter=','), dtype=np.float64)
    handle = lib.online_create(pointer(params), pointer(body2d), len(body2d), 11)
    if not handle:
        raise RuntimeError('handle creation failed')
    rows, snapshots = [], []

    def bind(payload, now=None):
        if lib.online_set_native_collision_snapshot(handle, pointer(payload), len(payload)):
            raise ValueError('actual exported snapshot refused by compiled ABI')
        if lib.online_set_native_collision_clock(handle, float(payload[3] if now is None else now)):
            raise ValueError('clock refused')

    def validate(payload, pose):
        bind(payload)
        state = np.array([*pose, 0., 0.], dtype=np.float64)
        controls = np.zeros((2, 2), dtype=np.float64)
        obs = np.empty((0, 6), dtype=np.float64)
        field = np.full((2, 2), 10., dtype=np.float64)  # proposal API only
        out = np.zeros((2048, 8), dtype=np.float64)
        report = np.zeros(6, dtype=np.float64)
        code = lib.online_validate_execution(handle, pointer(state), pointer(controls), 2,
            pointer(obs), 0, 0., 0., pointer(field), 2, 2, -1., -1., 1.,
            pointer(out), len(out), pointer(report))
        return dict(code=int(code), report=report.tolist(), planar_obstacle_count=0)

    try:
        for record in parity_records:
            with (root/'parity.bin').open('rb') as stream:
                stream.seek(record['offset'])
                raw = stream.read(record['length'])
            if hashlib.sha256(raw).hexdigest() != record['sha256']:
                raise ValueError('parity evidence hash mismatch')
            parity = np.frombuffer(raw, dtype='<f8')
            if parity[0] != 4 or parity[1] != 1 or len(parity) != 6+5*int(parity[5]):
                raise ValueError('invalid parity shape')
            matching = [r for r in records if r['kind'] == 'snapshot'
                and r['header'][14] == parity[2] and r['header'][17] == parity[3]
                and r['cached_map_available'] and r['header'][22] == 1 and r['header'][23] == 1]
            if not matching:
                raise ValueError('no exact map-acquisition match for native parity')
            payload = load_recorded_snapshot(root, matching[0], self_contained=True)
            # Native checkcollision is a STATIC map/geometry query. The first
            # matching raw frame can precede dynamic tracker readiness. For
            # this diagnostic only, remove dynamic samples and normalize the
            # unrelated live-perception gate. Original bytes stay untouched.
            nb, nd = int(payload[12]), int(payload[13])
            payload = np.concatenate((payload[:HEADER+7*nb], payload[HEADER+7*(nb+nd):]))
            payload[13] = 0
            payload[4] = payload[19] = 1
            payload[18] = payload[3]
            bind(payload)
            snapshots.append(payload)
            for x, y, yaw, native, collision_type in parity[6:].reshape((-1, 5)):
                out = np.zeros(5, dtype=np.float64)
                code = lib.online_native3d_query(handle, x, y, yaw, float(payload[3]), 1, pointer(out))
                valid = code == 0 and out[4] == 1 and np.isfinite(out[[0, 1, 2]]).all()
                collision = bool(out[0] < 0) if valid else None
                rows.append(dict(pose=[x, y, yaw], native_collision=bool(native),
                    native_type=int(collision_type), query_valid=bool(valid),
                    gap_m=float(out[0]), cubic_soft_cost=float(out[1]),
                    match=bool(valid and collision == bool(native)),
                    esdf_sequence=int(payload[14]), map_acquired=float(payload[17])))
        safe = next((r for r in rows if r['match'] and not r['native_collision']), None)
        if safe is None:
            raise ValueError('no safe pose for actual execution ABI control test')
        base = snapshots[0]
        nb, nd = int(base[12]), int(base[13])
        static = np.concatenate((base[:HEADER+7*nb], base[HEADER+7*(nb+nd):]))
        static[13] = 0
        static[18] = static[3]
        static[28] = 1
        safe_validation = validate(static, safe['pose'])
        # A native surface point at a body sample is definitely intersecting,
        # regardless of the absence of a planar actor proposal.
        bx, by, bz = static[HEADER:HEADER+3]
        x, y, yaw = safe['pose']
        point = [x+math.cos(yaw)*bx-math.sin(yaw)*by,
                 y+math.sin(yaw)*bx+math.cos(yaw)*by, bz, 0., 0., 0., 123.]
        dynamic = np.concatenate((static[:HEADER+7*nb], point, static[HEADER+7*nb:]))
        dynamic[13] = 1
        dynamic[2] += 1
        dynamic_validation = validate(dynamic, safe['pose'])
        bind(static, float(static[3]+.36))
        probe = np.zeros(5, dtype=np.float64)
        stale_code = lib.online_native3d_query(handle, *safe['pose'], float(static[3]+.36), 1, pointer(probe))
        malformed = static.copy()
        malformed[25] = .01
        malformed_code = lib.online_set_native_collision_snapshot(handle, pointer(malformed), len(malformed))
        invalidated_code = lib.online_native3d_query(handle, *safe['pose'], float(static[3]), 1, pointer(probe))
        native_collisions = sum(r['native_collision'] for r in rows)
        passed = (all(r['match'] for r in rows) and 0 < native_collisions < len(rows) and safe_validation['code'] == 0
            and dynamic_validation['code'] == 3 and stale_code != 0
            and malformed_code != 0 and invalidated_code != 0)
        return dict(passed=passed, trial=str(trial), library=str(library),
            library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),
            parity_count=len(rows), matches=sum(r['match'] for r in rows), rows=rows,
            native_collision_cases=native_collisions, native_clear_cases=len(rows)-native_collisions,
            static_safe_control=safe_validation, native3d_only_dynamic_collision=dynamic_validation,
            stale_query_code=int(stale_code), unsupported_uncertainty_binding_code=int(malformed_code),
            invalidated_query_code=int(invalidated_code),
            static_diagnostic_normalization='dynamic points removed and unrelated live-health bits normalized for native static API parity only; original raw bytes untouched; no actuation',
            scope='actual native static checkcollision vs same recorded map/geometry compiled ABI; fixed-arm sampled geometry, no mesh/continuous safety claim')
    finally:
        lib.online_destroy(handle)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', type=Path, required=True)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.trial, args.build)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open('x') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print(json.dumps({key: result[key] for key in ('passed', 'parity_count', 'matches',
        'static_safe_control', 'native3d_only_dynamic_collision')}))
    if not result['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
