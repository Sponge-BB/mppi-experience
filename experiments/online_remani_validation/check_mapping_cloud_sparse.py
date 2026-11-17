#!/usr/bin/python3
"""Replay mapping-only reduction from a closed bag; never initialize ROS or publish."""
import argparse
import io
import json
from pathlib import Path
import resource
import sys
import time
import warnings

import numpy as np
import rosbag
import rospy
import tf2_py

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'src/mm_remani/scripts'))
from velodyne_transformer import (cloud_xyz, mapping_voxel_indices,
                                  mapping_xyz_cloud, rigid_transform)


def native_beams(world, xyz, sensor, origin, size, resolution, max_range):
    """Native pre-raycast projection/clipping and first-rayend order.

    Matching this ordered actual-endpoint list and sensor origin means the
    unchanged native RayCaster sees the identical sequence of inputs. This
    checks ray geometry, not a timing claim or hardware validation.
    """
    mask = (np.all(np.abs(xyz) < [8., 8., 3.], axis=1) & (world[:, 2] >= .15))
    points = world[mask].copy()
    in_map = np.all((points >= origin+1e-4) & (points <= origin+size-1e-4), axis=1)
    for index in np.flatnonzero(~in_map):
        diff = points[index]-sensor
        values = np.r_[(origin+size-sensor)[diff != 0.]/diff[diff != 0.],
                       (origin-sensor)[diff != 0.]/diff[diff != 0.]]
        positive = values[values > 0.]
        points[index] = sensor+(min(1e6, float(np.min(positive)))-1e-3)*diff
    delta = points-sensor
    length = np.linalg.norm(delta, axis=1)
    hit = in_map & (length <= max_range)
    far = length > max_range
    points[far] = sensor+delta[far]/length[far, None]*max_range
    keys = np.floor((points-origin)*(1./resolution)).astype(np.int64)
    dimensions = np.ceil(size/resolution).astype(np.int64)
    addresses = (keys[:, 0]*dimensions[1]+keys[:, 1])*dimensions[2]+keys[:, 2]
    _, first = np.unique(addresses, return_index=True)
    first.sort()
    bounds = (np.min(keys, axis=0).tolist(), np.max(keys, axis=0).tolist()) if len(keys) else None
    return points[first], set(map(tuple, keys[hit])), bounds


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    origin = np.array([-7.5, -5., 0.]); size = np.array([15., 10., 3.])
    resolution = .05; max_range = 4.5
    transforms = tf2_py.BufferCore(rospy.Duration(1000.))
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        with rosbag.Bag(args.bag) as bag:
            for topic, msg, _ in bag.read_messages(topics=['/tf', '/tf_static']):
                setter = transforms.set_transform_static if topic == '/tf_static' else transforms.set_transform
                for transform in msg.transforms:
                    setter(transform, 'closed_bag')
    rows = []
    with rosbag.Bag(args.bag) as bag:
        for _, msg, _ in bag.read_messages(topics=['/velodyne_points']):
            transform = transforms.lookup_transform_core('world', msg.header.frame_id, msg.header.stamp)
            rotation, sensor = rigid_transform(transform)
            xyz = cloud_xyz(msg)
            world = xyz @ rotation.T+sensor
            original = native_beams(world, xyz, sensor, origin, size, resolution, max_range)
            times = []
            for _ in range(3):
                started = time.perf_counter()
                projection_mask = (np.all(np.abs(xyz) < [8., 8., 3.], axis=1) &
                                   (world[:, 2] >= origin[2]+.15))
                indices = mapping_voxel_indices(world, sensor, resolution, origin, size,
                                                max_range, projection_mask)
                output = mapping_xyz_cloud(msg, indices)
                serialized = io.BytesIO(); output.serialize(serialized)
                times.append((time.perf_counter()-started)*1000.)
            mapped_xyz = cloud_xyz(output)
            mapped_world = mapped_xyz @ rotation.T+sensor
            reduced = native_beams(mapped_world, mapped_xyz, sensor, origin, size, resolution, max_range)
            endpoint_cells = set(map(tuple, np.floor((world-origin)*(1./resolution)).astype(np.int64)))
            kept_cells = set(map(tuple, np.floor((mapped_world-origin)*(1./resolution)).astype(np.int64)))
            row = {'stamp': msg.header.stamp.to_sec(), 'original_points': len(xyz),
                   'mapping_points': len(indices), 'original_payload_bytes': len(msg.data),
                   'mapping_payload_bytes': len(output.data), 'extra_branch_ms': float(np.median(times)),
                   'metadata_equal': output.header == msg.header,
                   'measured_xyz_equal': bool(np.array_equal(mapped_xyz, xyz[indices])),
                   'all_endpoint_cells_equal': endpoint_cells == kept_cells,
                   'native_first_beams_equal': bool(np.array_equal(original[0], reduced[0])),
                   'native_hit_cells_equal': original[1] == reduced[1],
                   'native_update_bounds_equal': original[2] == reduced[2],
                   'native_ray_count': len(original[0])}
            rows.append(row)
    checks = ['metadata_equal', 'measured_xyz_equal', 'all_endpoint_cells_equal',
              'native_first_beams_equal', 'native_hit_cells_equal', 'native_update_bounds_equal']
    report = {'bag': str(Path(args.bag).resolve()), 'frames': len(rows),
              'grid': {'origin': origin.tolist(), 'size': size.tolist(),
                       'resolution': resolution, 'max_ray_length': max_range},
              'mean_original_points': float(np.mean([row['original_points'] for row in rows])),
              'mean_mapping_points': float(np.mean([row['mapping_points'] for row in rows])),
              'mean_payload_ratio': float(np.mean([row['mapping_payload_bytes']/row['original_payload_bytes']
                                                   for row in rows])),
              'extra_branch_ms': {'median': float(np.median([row['extra_branch_ms'] for row in rows])),
                                  'p95': float(np.percentile([row['extra_branch_ms'] for row in rows], 95)),
                                  'max': float(max(row['extra_branch_ms'] for row in rows))},
              'passed': {key: sum(row[key] for row in rows) for key in checks},
              'peak_rss_mib': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024.,
              'limits': ['Closed bag with original grid parameters, not a live robot test.',
                         'Timing excludes existing TF/world work, kept_world collection/concatenation, '
                         'the first target-XYZ read, and live ROS transport.',
                         'Matching ray inputs does not reduce the ESDF region or prove lower end-to-end latency.'],
              'rows': rows}
    path = Path(args.output); path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({key: value for key, value in report.items() if key != 'rows'}, indent=2))
    if not rows or any(report['passed'][key] != len(rows) for key in checks):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
