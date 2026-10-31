#!/usr/bin/env python3
"""Read-only endpoint geometry audit of the simulation-only CropBox bypass.

Uses the final measured plant pose, actual recorded native dynamic snapshot,
and synthetic actor truth. No ROS, optimizer, control output, or new simulation.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np
import yaml


HERE = Path(__file__).resolve().parents[1]
DEFAULT_TRIAL = HERE / 'results/sim_crop_bypass_20261029_01/head_on_bypass_seed11'
DEFAULT_OUT = HERE / 'results/sim_crop_bypass_report_20261029_01/perception_endpoint_audit.json'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def body_world(body, pose):
    c, s = np.cos(pose[2]), np.sin(pose[2])
    centers = body[:, :3].copy()
    centers[:, :2] = centers[:, :2] @ np.array([[c, s], [-s, c]]) + pose[:2]
    return centers


def actor_gaps(centers, radii, actors):
    """Same finite-cylinder truth geometry as the frozen monitor, all balls."""
    gaps = []
    for actor in actors:
        radial = np.hypot(centers[:, 0] - actor['x'], centers[:, 1] - actor['y']) - actor['r']
        vertical = np.abs(centers[:, 2] - (.15 + 1.4) / 2) - (1.4 - .15) / 2
        distance = (np.hypot(np.maximum(radial, 0), np.maximum(vertical, 0)) +
                    np.minimum(np.maximum(radial, vertical), 0))
        gaps.append(distance - radii)
    return np.min(gaps, axis=0)


def audit(trial):
    trial = Path(trial).resolve()
    result = json.loads((trial / 'result.json').read_text())
    manifest = json.loads((trial / 'manifest.json').read_text())
    parameters = yaml.safe_load((trial / 'parameters.yaml').read_text())
    tree = ET.parse(trial / 'experiment.launch')
    nodes = {node.get('name'): node for node in tree.getroot().findall('node')}
    output = next(remap.get('to') for remap in nodes['passthrough_z'].findall('remap')
                  if remap.get('from') == '~output')
    crop_absent = 'crop_box_filter' not in nodes
    frames = [json.loads(line) for line in
              (trial / 'native3d_snapshots/index.jsonl').read_text().splitlines()]
    final_time = float(result['elapsed'])
    record = max((frame for frame in frames if frame['kind'] == 'snapshot' and
                  frame['header'][3] <= 1000 + final_time), key=lambda frame: frame['header'][3])
    with (trial / 'native3d_snapshots/frames.bin').open('rb') as stream:
        stream.seek(record['offset'])
        raw = stream.read(record['length'])
    if hashlib.sha256(raw).hexdigest() != record['prefix_sha256']:
        raise ValueError('Recorded snapshot prefix hash mismatch')
    payload = np.frombuffer(raw, dtype='<f8')
    nb, nd = int(payload[12]), int(payload[13])
    body = payload[32:32 + 7 * nb].reshape((-1, 7))
    dynamic = payload[32 + 7 * nb:32 + 7 * (nb + nd)].reshape((-1, 7))
    states = np.atleast_1d(np.genfromtxt(trial / 'state.csv', delimiter=',', names=True))
    actors = np.atleast_1d(np.genfromtxt(trial / 'obstacles.csv', delimiter=',', names=True))
    actual_actors = actors[np.isclose(actors['t'], final_time, atol=1e-6, rtol=0)]
    final_state = np.asarray(result['final_state'])
    if not len(actual_actors) or not len(dynamic):
        raise ValueError('Endpoint requires actual actor truth and recorded dynamic points')
    if abs(float(states[-1]['t']) - final_time) > 1e-6:
        raise ValueError('Final state CSV timestamp differs from result')
    centers = body_world(body, final_state[:3])
    physical_radii = body[:, 3] - np.where(body[:, 6] == 0, payload[11], 0)
    physical_gaps = actor_gaps(centers, physical_radii, actual_actors)
    contact = np.flatnonzero(np.isclose(physical_gaps, physical_gaps.min(), atol=1e-10, rtol=0))
    age = 1000 + final_time - float(payload[18])
    predicted = dynamic[:, :3] + age * dynamic[:, 3:6]
    distances = np.linalg.norm(centers[:, None, :] - predicted[None, :, :], axis=2)
    point_indices = np.argmin(distances, axis=1)
    hard_gaps = distances[np.arange(nb), point_indices] - body[:, 3]
    minimum_ball = int(np.argmin(hard_gaps))
    logs = sorted((trial / 'roslogs').glob('*/dynamic_obstacle_tracker-*-stdout.log'))
    residual_rows = []
    for path in logs:
        for line in path.read_text().splitlines():
            match = re.search(r'downsampled=(\d+) residual=(\d+).*health=(\d+)', line)
            if match:
                residual_rows.append(dict(downsampled=int(match[1]), residual=int(match[2]), health=int(match[3])))
    last_log = residual_rows[-1] if residual_rows else None
    cap = int(parameters['/dynamic_obstacle_tracker/maximum_output_points'])
    stride = max(1, (last_log['residual'] + cap - 1) // cap) if last_log else None
    source_files = ['experiment.launch', 'manifest.json', 'parameters.yaml', 'result.json',
                    'state.csv', 'obstacles.csv', 'native3d_snapshots/index.jsonl',
                    'runtime/simulator.py', 'runtime/native_collision_runtime.py']
    report = dict(
        trial=str(trial), script_sha256=sha(Path(__file__)),
        scope='Recorded endpoint audit only; actor truth is evaluation-only. Dynamic point query only, not combined static ESDF. No new simulation or performance measurement.',
        topology=dict(crop_node_absent=crop_absent, height_output=output,
                      bypass_verified=crop_absent and output == '/velodyne_points',
                      manifest_nodes_agree=not any(node['name'] == 'crop_box_filter' for node in manifest['nodes']),
                      launch_sha256_matches_manifest=sha(trial / 'experiment.launch') ==
                      manifest['simulation_perception_adjustment']['launch_sha256']),
        endpoint=dict(time_s=final_time, state=final_state.tolist(),
                      actors=[{name:float(actor[name]) for name in actors.dtype.names} for actor in actual_actors],
                      native_truth_monitor=result['native3d_truth_monitor'],
                      minimum_actor_physical_gap_m=float(physical_gaps.min()),
                      contact_balls_zero_based=[dict(index=int(i), local_body=body[i].tolist(),
                          world_center=centers[i].tolist(), physical_radius_m=float(physical_radii[i]),
                          physical_gap_m=float(physical_gaps[i]), dynamic_hard_gap_m=float(hard_gaps[i])) for i in contact]),
        recorded_snapshot=dict(sequence=int(payload[2]), publication_time_s=float(payload[3] - 1000),
            observation_time_s=float(payload[18] - 1000), extrapolation_s=age,
            dynamic_points=nd, body_samples=nb, health=float(payload[19]),
            velocity_min=dynamic[:, 3:6].min(axis=0).tolist(),
            velocity_max=dynamic[:, 3:6].max(axis=0).tolist(),
            minimum_dynamic_hard_gap_m=float(hard_gaps[minimum_ball]),
            minimum_gap_ball_zero_based=minimum_ball,
            minimum_gap_world_ball=centers[minimum_ball].tolist(),
            nearest_predicted_point=predicted[point_indices[minimum_ball]].tolist(),
            prefix_sha256=record['prefix_sha256']),
        sampling_evidence=dict(minimum_sensor_range_m=parameters['/dynamic_obstacle_tracker/minimum_range'],
            voxel_leaf_size_m=parameters['/dynamic_obstacle_tracker/voxel_leaf_size'],
            configured_output_cap=cap, last_tracker_log=last_log,
            stride_if_all_last_residual_points_are_one_qualified_actor=stride,
            note='Latest throttled tracker log is not guaranteed to be the identical observation. Range/voxel/stride stages are NOT fully replayed; this does not uniquely attribute missing surface points.'),
        input_sha256={name:sha(trial / name) for name in source_files},
        tracker_log_sha256={str(path.relative_to(trial)):sha(path) for path in logs},
        hash_scope='Hashes attest current recorded files and the exact frame prefix, not historical in-memory executables or every raw sensor point.')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', type=Path, default=DEFAULT_TRIAL)
    parser.add_argument('--out', type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()
    report = audit(args.trial)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(dict(out=str(args.out), topology=report['topology'],
          endpoint_physical_gap_m=report['endpoint']['minimum_actor_physical_gap_m'],
          snapshot=report['recorded_snapshot']), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
