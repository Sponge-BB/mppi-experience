#!/usr/bin/env python3
"""Audit one recorded encounter; no ROS, solver, or original-file changes.

Reconstruct ONLY synthetic actor raw points and the two configured PCL masks.
The recorded dynamic output is a separate observation, not a replay of the
voxel filter, clustering, tracking, or point-budget stages.
"""
import argparse
import ast
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TRIAL = ROOT / 'results/execution_timing_pair_20261028_01/dynamic_head_on_center_matched_seed11'
DEFAULT_OUT = ROOT / 'results/execution_timing_diagnosis_20261028_01'


def read_grid(source):
    """Read azimuth/height grids from the frozen cloud() comprehension."""
    tree = ast.parse(source)
    cloud = next(n for n in ast.walk(tree)
                 if isinstance(n, ast.FunctionDef) and n.name == 'cloud')
    grid = {}
    for node in ast.walk(cloud):
        if not isinstance(node, ast.comprehension) or not isinstance(node.target, ast.Name):
            continue
        call = node.iter
        if (isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)
                and call.func.attr == 'arange' and node.target.id in ('a', 'z')):
            values = []
            for arg in call.args:
                if isinstance(arg, ast.BinOp):  # frozen 2 * math.pi
                    values.append(2 * np.pi)
                else:
                    values.append(ast.literal_eval(arg))
            grid[node.target.id] = np.arange(*values)
    if set(grid) != {'a', 'z'} or 'np.linalg.norm(local[:,:2],axis=1)<5.' not in source:
        raise ValueError('frozen synthetic cloud recipe changed; review reconstruction')
    return grid


def pose_at(states, t):
    i = int(np.argmin(abs(states['t'] - t)))
    if abs(float(states['t'][i]) - t) > 1e-6:
        raise ValueError('observation has no exact recorded plant pose')
    return np.array([states['x'][i], states['y'][i], states['yaw'][i]])


def world_to_lidar(points, pose, offset):
    c, s = np.cos(pose[2]), np.sin(pose[2])
    rotation = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
    return (points - [pose[0], pose[1], 0]) @ rotation.T - offset


def lidar_to_world(points, pose, offset):
    c, s = np.cos(pose[2]), np.sin(pose[2])
    rotation = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
    return (points + offset) @ rotation + [pose[0], pose[1], 0]


def body_world(body, pose):
    c, s = np.cos(pose[2]), np.sin(pose[2])
    points = body[:, :3].copy()
    points[:, :2] = points[:, :2] @ np.array([[c, s], [-s, c]]) + pose[:2]
    return points


def point_gap(points, center, hard_radius):
    distances = np.linalg.norm(points - center, axis=1)
    i = int(np.argmin(distances))
    return dict(count=len(points), hard_gap_m=float(distances[i] - hard_radius),
                nearest_xyz=points[i].tolist())


def finite_cylinder_gap(center, actor, radius, lower, upper):
    radial = np.hypot(*(center[:2] - [actor['x'], actor['y']])) - actor['r']
    vertical = abs(center[2] - (lower + upper) / 2) - (upper - lower) / 2
    distance = np.hypot(max(radial, 0), max(vertical, 0)) + min(max(radial, vertical), 0)
    return float(distance - radius)


def audit(trial, out, sequence, body_index):
    trial, out = trial.resolve(), out.resolve()
    params = yaml.safe_load((trial / 'parameters.yaml').read_text())
    source_path = trial / 'runtime/simulator.py'
    source = source_path.read_text()
    grid = read_grid(source)
    states = np.genfromtxt(trial / 'state.csv', delimiter=',', names=True)
    actors = np.genfromtxt(trial / 'obstacles.csv', delimiter=',', names=True)
    records = [json.loads(line) for line in
               (trial / 'native3d_snapshots/index.jsonl').read_text().splitlines()]
    record = next(r for r in records if r['kind'] == 'snapshot' and int(r['header'][2]) == sequence)
    with (trial / 'native3d_snapshots/frames.bin').open('rb') as stream:
        stream.seek(record['offset'])
        raw = stream.read(record['length'])
    if hashlib.sha256(raw).hexdigest() != record['prefix_sha256']:
        raise ValueError('recorded snapshot prefix hash mismatch')
    payload = np.frombuffer(raw, dtype='<f8')
    nb, nd = int(payload[12]), int(payload[13])
    body = payload[32:32 + 7 * nb].reshape((-1, 7))
    dynamic = payload[32 + 7 * nb:32 + 7 * (nb + nd)].reshape((-1, 7))
    if nd == 0:
        raise ValueError('chosen snapshot has no actual dynamic output')
    t = float(payload[18] - 1000.)
    pose = pose_at(states, t)
    actor_rows = actors[np.abs(actors['t'] - t) < 1e-6]
    if len(actor_rows) == 0:
        raise ValueError('no actor truth at observation time')
    offset = np.array([params['/remani_planner_node/grid_map/lidar_offset_' + a] for a in 'xyz'])
    lo = np.array([params['/crop_box_filter/min_' + a] for a in 'xyz'])
    hi = np.array([params['/crop_box_filter/max_' + a] for a in 'xyz'])
    generated = np.concatenate([
        np.array([(actor['x'] + actor['r'] * np.cos(a),
                   actor['y'] + actor['r'] * np.sin(a), z)
                  for a in grid['a'] for z in grid['z']]) for actor in actor_rows])
    # create_cloud_xyz32 quantizes the sensor coordinates before PCL receives them.
    raw_lidar = world_to_lidar(generated, pose, offset)
    raw_lidar = raw_lidar[np.linalg.norm(raw_lidar[:, :2], axis=1) < 5.].astype(np.float32).astype(float)
    field = params['/passthrough_z/filter_field_name']
    axis = 'xyz'.index(field)
    height_keep = ((raw_lidar[:, axis] >= params['/passthrough_z/filter_limit_min']) &
                   (raw_lidar[:, axis] <= params['/passthrough_z/filter_limit_max']))
    if params['/passthrough_z/filter_limit_negative']:
        height_keep = ~height_keep
    height = raw_lidar[height_keep]
    inside = np.all((height >= lo) & (height <= hi), axis=1)
    crop_keep = ~inside if params['/crop_box_filter/negative'] else inside
    cropped = height[crop_keep]
    center = body_world(body, pose)[body_index]
    hard_radius = float(body[body_index, 3])
    physical_radius = hard_radius - (float(payload[11]) if body[body_index, 6] == 0 else 0.)
    stages = {name: point_gap(lidar_to_world(points, pose, offset), center, hard_radius)
              for name, points in [('raw_actor_reconstruction', raw_lidar),
                                   ('after_height_mask', height), ('after_crop_mask', cropped)]}
    stages['recorded_dynamic_output_separate_observation'] = point_gap(dynamic[:, :3], center, hard_radius)
    nearest = np.asarray(stages['after_height_mask']['nearest_xyz'])
    nearest_lidar = world_to_lidar(nearest[None, :], pose, offset)[0]
    final = states[-1]
    final_pose = np.array([final['x'], final['y'], final['yaw']])
    final_centers = body_world(body, final_pose)
    forecast = float(final['t'] - t)
    predicted = dynamic[:, :3] + forecast * dynamic[:, 3:6]
    distances = np.linalg.norm(final_centers[:, None, :] - predicted[None, :, :], axis=2)
    gaps = distances.min(axis=1) - body[:, 3]
    final_actors = actors[np.abs(actors['t'] - final['t']) < 1e-6]
    truth_gap = min(finite_cylinder_gap(final_centers[body_index], actor,
                                      physical_radius, .15, 1.4) for actor in final_actors)
    result = dict(
        scope='Read-only geometry audit. Reconstruct actor sensor points and height/CropBox masks only; no ROS, solver, or new closed-loop trial.',
        trial=str(trial), snapshot_sequence=sequence, body_index_zero_based=body_index,
        observation_time_s=t, observation_pose_xy_yaw=pose.tolist(),
        body_local_xyz=body[body_index, :3].tolist(), hard_radius_m=hard_radius,
        physical_radius_m=physical_radius, body_world_xyz_at_observation=center.tolist(),
        parameters=dict(crop_min_lidar_xyz=lo.tolist(), crop_max_lidar_xyz=hi.tolist(),
                        crop_negative=params['/crop_box_filter/negative'], lidar_offset_xyz=offset.tolist(),
                        height_min=params['/passthrough_z/filter_limit_min'], height_max=params['/passthrough_z/filter_limit_max']),
        stages=stages, crop_removed_actor_points=int((~crop_keep).sum()),
        closest_before_crop_lidar_xyz=nearest_lidar.tolist(),
        closest_before_crop_inside_box=bool(np.all((nearest_lidar >= lo) & (nearest_lidar <= hi))),
        collision_endpoint=dict(time_s=float(final['t']), pose_xy_yaw=final_pose.tolist(),
                                analytic_actor_physical_gap_body120_m=truth_gap,
                                snapshot_prediction_age_s=forecast,
                                recorded_dynamic_min_hard_gap_all_balls_m=float(gaps.min()),
                                recorded_dynamic_hard_gap_body120_m=float(gaps[body_index])),
        inputs_sha256={name: hashlib.sha256((trial / name).read_bytes()).hexdigest()
                       for name in ('parameters.yaml', 'state.csv', 'obstacles.csv', 'runtime/simulator.py')},
        snapshot_prefix_sha256=record['prefix_sha256'],
        limits=['Raw cloud was not recorded; actor points are deterministic reconstruction from the frozen synthetic sensor recipe and recorded pose/truth.',
                'Static-map points are excluded from reconstructed stage counts and actor-surface distances.',
                'Actual dynamic output is observed after additional voxel, range, static-background, clustering, tracking and point-budget stages; those stages are NOT replayed.',
                'This establishes specific external-obstacle point deletion, not an exclusive attribution of every missing point to CropBox.',
                'Between-cycle timing stall is a separate issue; this audit does not diagnose it.'])
    out.mkdir(parents=True, exist_ok=True)
    path = out / 'perception_audit.json'
    path.write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
    plot(out / 'perception_filtering.png', raw_lidar, height[~crop_keep], cropped,
         world_to_lidar(dynamic[:, :3], pose, offset), world_to_lidar(center[None, :], pose, offset)[0],
         physical_radius, hard_radius, lo, hi, stages)
    return path, result


def plot(path, raw, removed, kept, actual, center, physical_radius, hard_radius, lo, hi, stages):
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.3))
    for ax, dims, label in zip(axes, [(0, 1), (0, 2)], ['Top view', 'Side view']):
        a, b = dims
        ax.add_patch(Rectangle((lo[a], lo[b]), hi[a] - lo[a], hi[b] - lo[b],
                               color='#db4437', alpha=.08, label='Self-removal box'))
        ax.scatter(removed[:, a], removed[:, b], s=12, color='#db4437', alpha=.7, label='Removed actor points')
        ax.scatter(kept[:, a], kept[:, b], s=12, color='#2474ad', alpha=.65, label='Kept actor points')
        ax.scatter(actual[:, a], actual[:, b], s=16, color='#21883c', marker='x', label='Recorded dynamic output')
        ax.add_patch(Circle((center[a], center[b]), hard_radius, fill=False, linestyle='--',
                            color='#222222', label='Ball 120 check radius'))
        ax.add_patch(Circle((center[a], center[b]), physical_radius, fill=False, color='#222222'))
        ax.plot(center[a], center[b], 'k.', ms=4)
        points = np.vstack((raw, center))
        lower, upper = points[:, dims].min(axis=0) - .17, points[:, dims].max(axis=0) + .17
        ax.set_xlim(lower[0], upper[0]); ax.set_ylim(lower[1], upper[1])
        ax.set_aspect('equal'); ax.set_title(label + ' (lidar frame)')
        ax.set_xlabel('x (m)'); ax.set_ylabel('yz'[b - 1] + ' (m)')
        ax.grid(alpha=.15)
    axes[1].legend(loc='upper left', fontsize=7, framealpha=.95)
    before = stages['after_height_mask']['hard_gap_m'] * 100
    after = stages['after_crop_mask']['hard_gap_m'] * 100
    fig.suptitle('Same pose / ball 120: point gap before crop %+.2f cm; after crop %+.2f cm' % (before, after), fontsize=10)
    fig.tight_layout(rect=[0, 0, 1, .94]); fig.savefig(path, dpi=160); plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', type=Path, default=DEFAULT_TRIAL)
    parser.add_argument('--out', type=Path, default=DEFAULT_OUT)
    parser.add_argument('--sequence', type=int, default=476)
    parser.add_argument('--body-index', type=int, default=120)
    args = parser.parse_args()
    path, report = audit(args.trial, args.out, args.sequence, args.body_index)
    print(json.dumps({'report': str(path), 'stages': report['stages'],
                      'collision_endpoint': report['collision_endpoint']}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
