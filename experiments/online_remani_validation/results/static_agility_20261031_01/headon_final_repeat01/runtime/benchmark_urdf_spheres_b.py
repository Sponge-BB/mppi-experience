"""Same-input native 3-D point-query timing; NOT a full MPPI/online benchmark.

Only the fixed-structure sphere array changes. ESDF, dynamic point cloud,
fixed-arm samples, safety margins and collision engine are shared inputs.
No ROS master, control command, online simulation or new C++ code is used.
"""
import argparse
import ctypes as ct
import hashlib
import json
import os
from pathlib import Path
import time

import numpy as np

from native_collision_runtime import (
    DP, HEADER, decode_snapshot, freshness_reason, load_recorded_snapshot,
)

ROOT = Path(__file__).resolve().parent
DEFAULT_TRIAL = ROOT / 'results/watchdog_off_20261029_01/mid_cross_seed11'
DEFAULT_CANDIDATE = ROOT / 'results/urdf_sphere_simplification_20261029_01/candidate_coarse.json'
DEFAULT_LIBRARY = ROOT / 'builds/urdf_spheres_b_20261029_01/build/libonline_mppi.so'
DEFAULT_OUTPUT = ROOT / 'results/urdf_spheres_b_validation_20261029_01/query_benchmark.json'


def pointer(array):
    return array.ctypes.data_as(DP)


def digest(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(block)
    return result.hexdigest()


def select_snapshot(trial, target_time):
    root = trial / 'native3d_snapshots'
    candidates = []
    with (root / 'index.jsonl').open() as stream:
        for line in stream:
            record = json.loads(line)
            if record.get('kind') != 'snapshot' or not record.get('cached_map_available'):
                continue
            header = record['header']
            if (header[13] > 0 and header[28] == 1 and
                    all(header[i] == 1 for i in (4, 19, 22, 23))):
                candidates.append(record)
    if not candidates:
        raise ValueError('no healthy native snapshot with real dynamic points')
    record = min(candidates, key=lambda item: abs(item['header'][3] - target_time))
    payload = decode_snapshot(load_recorded_snapshot(root, record, self_contained=True).tobytes())
    reason = freshness_reason(payload, float(payload[3]))
    if reason:
        raise ValueError('selected snapshot is not healthy at publication: ' + reason)

    nearest = None
    with (trial / 'state_odom_events.jsonl').open() as stream:
        for line in stream:
            item = json.loads(line).get('odom_identity', {})
            if not item.get('validity', {}).get('state_finite'):
                continue
            stamp = item['stamp_sec'] + item['stamp_nsec'] * 1e-9
            delta = abs(stamp - payload[3])
            if nearest is None or delta < nearest[0]:
                nearest = (delta, stamp, item['state'])
    if nearest is None or nearest[0] > 0.1:
        raise ValueError('no actual odometry within 100 ms of snapshot publication')
    return record, payload, nearest


def candidate_payload(payload, candidate):
    nb, nd = int(payload[12]), int(payload[13])
    body = payload[HEADER:HEADER + 7 * nb].reshape((-1, 7))
    base = body[body[:, 6] == 0]
    arm = body[body[:, 6] == 1]
    if len(base) != 140 or len(arm) != 20:
        raise ValueError('recorded input must have legacy base140 + unchanged arm20')
    resolution = float(payload[11])
    physical_radius = base[:, 3] - resolution
    margin = base[:, 4] - physical_radius
    if not (np.allclose(physical_radius, 0.08, rtol=0, atol=1e-12) and
            np.allclose(margin, margin[0], rtol=0, atol=1e-12) and
            np.allclose(base[:, 5], base[0, 5], rtol=0, atol=1e-12)):
        raise ValueError('recorded base radii/margins/weights do not match the intended contract')
    spheres = candidate['spheres']
    if candidate['sphere_count'] != 47 or len(spheres) != 47:
        raise ValueError('candidate B must contain 47 fixed-structure spheres')
    replacement = np.array([
        list(sphere['center_ground']) + [sphere['radius'] + resolution,
            sphere['radius'] + margin[0], base[0, 5], 0.0]
        for sphere in spheres], dtype=np.float64)
    new_body = np.concatenate((replacement, arm))
    header = payload[:HEADER].copy()
    header[12] = 67
    modified = decode_snapshot(np.concatenate((header, new_body.ravel(),
        payload[HEADER + 7 * nb:])).astype('<f8').tobytes())
    # Explicitly preserve the entire recorded dynamic wire and ESDF tail.
    old_tail = payload[HEADER + 7 * nb:]
    if not np.array_equal(modified[HEADER + 7 * 67:], old_tail):
        raise ValueError('dynamic cloud or ESDF changed during body replacement')
    return modified, {'base_margin_m': float(margin[0]), 'resolution_m': resolution,
                      'base_weight': float(base[0, 5]), 'dynamic_points': nd,
                      'legacy_base_spheres': 140, 'candidate_base_spheres': 47,
                      'unchanged_arm_spheres': 20}


def queries_around_state(payload, state, count):
    rng = np.random.RandomState(11)
    poses = np.empty((count, 4), dtype=np.float64)
    poses[:, :3] = np.asarray(state[:3]) + rng.uniform(-0.1, 0.1, (count, 3))
    poses[:, 3] = float(payload[3]) + np.linspace(0.0, 1.0, count)
    if (np.min(poses[:, 3]) - payload[18] < -0.05 or
            np.max(poses[:, 3]) - payload[18] > payload[16]):
        raise ValueError('query times exceed the recorded CV prediction horizon')
    return poses


def check_crop(payload, queries):
    body = payload[HEADER:HEADER + 7 * int(payload[12])].reshape((-1, 7))
    origin = payload[8:11]
    dimensions = payload[5:8]
    for pose in queries:
        co, si = np.cos(pose[2]), np.sin(pose[2])
        points = body[:, :3].copy()
        points[:, 0] = pose[0] + co * body[:, 0] - si * body[:, 1]
        points[:, 1] = pose[1] + si * body[:, 0] + co * body[:, 1]
        fractional = (points - origin) / payload[11] - 0.5
        if np.any(fractional < 0) or np.any(fractional >= dimensions - 1):
            raise ValueError('recorded ESDF crop does not cover all sphere-centre queries; refusing benchmark')


def configure_library(path):
    library = ct.CDLL(str(path))
    library.online_create.argtypes = [DP, DP, ct.c_int, ct.c_int]
    library.online_create.restype = ct.c_void_p
    library.online_destroy.argtypes = [ct.c_void_p]
    library.online_set_native_collision_snapshot.argtypes = [ct.c_void_p, DP, ct.c_int]
    library.online_set_native_collision_snapshot.restype = ct.c_int
    library.online_set_native_collision_clock.argtypes = [ct.c_void_p, ct.c_double]
    library.online_set_native_collision_clock.restype = ct.c_int
    library.online_native3d_query.argtypes = [ct.c_void_p, ct.c_double, ct.c_double,
        ct.c_double, ct.c_double, ct.c_int, DP]
    library.online_native3d_query.restype = ct.c_int
    return library


def run_batch(library, handle, queries):
    output = np.empty(5, dtype=np.float64)
    out_pointer = pointer(output)
    outputs = np.empty((len(queries), 5), dtype=np.float64)
    query = library.online_native3d_query
    wall_start = time.perf_counter()
    cpu_start = time.process_time()
    for i, pose in enumerate(queries):
        code = query(handle, float(pose[0]), float(pose[1]), float(pose[2]),
                     float(pose[3]), 0, out_pointer)
        if code != 0 or output[4] != 1 or not np.isfinite(output[:4]).all():
            raise ValueError('native query failed at query %d, code %d: %r' % (i, code, output.tolist()))
        outputs[i] = output
    cpu_seconds = time.process_time() - cpu_start
    wall_seconds = time.perf_counter() - wall_start
    return {'wall_ms': wall_seconds * 1000, 'process_cpu_ms': cpu_seconds * 1000,
            'mean_query_wall_us': wall_seconds * 1e6 / len(queries),
            'mean_query_cpu_us': cpu_seconds * 1e6 / len(queries)}, outputs


def benchmark(args):
    trial, library_path = args.trial.resolve(), args.library.resolve()
    candidate = json.loads(args.candidate.read_text())
    record, legacy_payload, nearest = select_snapshot(trial, args.frame_time)
    coarse_payload, contract = candidate_payload(legacy_payload, candidate)
    queries = queries_around_state(legacy_payload, nearest[2], args.queries)
    for payload in (legacy_payload, coarse_payload):
        check_crop(payload, queries)
    limits = json.loads((trial / 'limits.json').read_text())
    parameters = np.ascontiguousarray(list(limits.values()), dtype=np.float64)
    proposal_body = np.ascontiguousarray(np.loadtxt(trial / 'geometry.csv', delimiter=','), dtype=np.float64)
    os.environ['NATIVE_COLLISION_3D'] = '1'
    library = configure_library(library_path)
    handles = {}
    batches = []
    last_outputs = {}
    try:
        for name, payload in (('legacy', legacy_payload), ('candidate_b', coarse_payload)):
            handle = library.online_create(pointer(parameters), pointer(proposal_body), len(proposal_body), 11)
            if not handle:
                raise RuntimeError('could not create ' + name + ' handle')
            handles[name] = handle
            if library.online_set_native_collision_snapshot(handle, pointer(payload), len(payload)):
                raise RuntimeError('snapshot binding failed: ' + name)
            if library.online_set_native_collision_clock(handle, float(payload[3])):
                raise RuntimeError('snapshot clock binding failed: ' + name)
            run_batch(library, handle, queries)  # Same-sized unmeasured warm-up.
        for round_index in range(args.rounds):
            order = ('legacy', 'candidate_b') if round_index % 2 == 0 else ('candidate_b', 'legacy')
            for name in order:
                timing, output = run_batch(library, handles[name], queries)
                batches.append(dict(round=round_index + 1, order=list(order), model=name, **timing))
                last_outputs[name] = output
    finally:
        for handle in handles.values():
            library.online_destroy(handle)
    summary = {}
    for name in ('legacy', 'candidate_b'):
        selected = [item for item in batches if item['model'] == name]
        summary[name] = {
            'total_spheres': int(legacy_payload[12] if name == 'legacy' else coarse_payload[12]),
            'median_batch_wall_ms': float(np.median([item['wall_ms'] for item in selected])),
            'median_batch_cpu_ms': float(np.median([item['process_cpu_ms'] for item in selected])),
            'median_query_wall_us': float(np.median([item['mean_query_wall_us'] for item in selected])),
            'median_query_cpu_us': float(np.median([item['mean_query_cpu_us'] for item in selected])),
            'hard_colliding_queries': int(np.sum(last_outputs[name][:, 0] < 0)),
            'minimum_hard_gap_m': float(np.min(last_outputs[name][:, 0])),
        }
    return {
        'scope': '同输入、同一新库的native3d三维碰撞查询微基准；不是完整MPPI、REMANI或在线避障耗时',
        'status': 'completed', 'ctypes_overhead_included': True,
        'host_exclusivity': '未预留独占CPU，可能与构建任务同时运行；采用AB/BA交替并同时报告进程CPU时间，不将结果当作独占主机基准',
        'timing_includes': 'Python循环、ctypes调用、返回值有限性校验及结果复制；不包括快照加载/绑定、建KD树及完整控制求解',
        'source_trial': str(trial), 'source_trial_online_valid': False,
        'source_trial_warning': '来源在线试验末段REMANI崩溃；本测试仅使用此前健康的不可变输入帧，不作为在线成功证据',
        'library': str(library_path), 'library_sha256': digest(library_path),
        'candidate_path': str(args.candidate.resolve()), 'candidate_sha256': digest(args.candidate),
        'frame': {'publication_stamp': float(legacy_payload[3]), 'sequence': float(legacy_payload[2]),
                  'dynamic_stamp': float(legacy_payload[18]), 'map_sha256': record['map_sha256'],
                  'odom_stamp': nearest[1], 'odom_time_difference_s': nearest[0],
                  'actual_state': nearest[2]},
        'input_contract': contract, 'queries_per_batch': args.queries, 'rounds': args.rounds,
        'order': 'AB/BA交替，各模型独立handle；每批512个不同pose/time，完整查询而非碰撞即返回',
        'query_time_range': [float(queries[0, 3]), float(queries[-1, 3])],
        'query_array_sha256': hashlib.sha256(queries.tobytes()).hexdigest(),
        'summary': summary,
        'median_wall_speedup': summary['legacy']['median_batch_wall_ms'] / summary['candidate_b']['median_batch_wall_ms'],
        'median_cpu_speedup': summary['legacy']['median_batch_cpu_ms'] / summary['candidate_b']['median_batch_cpu_ms'],
        'collision_results_warning': '几何模型不同，碰撞间隙和代价值可以不同；本测试不把结果差异当成算法故障或避障效果提升',
        'batches': batches,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', type=Path, default=DEFAULT_TRIAL)
    parser.add_argument('--candidate', type=Path, default=DEFAULT_CANDIDATE)
    parser.add_argument('--library', type=Path, default=DEFAULT_LIBRARY)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument('--frame-time', type=float, default=1015.0)
    parser.add_argument('--queries', type=int, default=512)
    parser.add_argument('--rounds', type=int, default=6)
    args = parser.parse_args()
    if args.queries < 2 or args.rounds < 2:
        parser.error('at least two queries and two order-alternating rounds are required')
    result = benchmark(args)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({'output': str(args.output), 'summary': result['summary'],
                      'median_wall_speedup': result['median_wall_speedup']}, ensure_ascii=False))


if __name__ == '__main__':
    main()
