#!/usr/bin/python3
"""Offline bounded raw occupancy comparison; no ROS graph or hardware access."""
import datetime
import json
import math
from pathlib import Path
import resource
import struct

import numpy as np
import rosbag
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

HERE = Path(__file__).resolve().parent
BASELINE_INDEX = 6
TZ = datetime.timezone(datetime.timedelta(hours=8))
BASELINE_LIMIT = datetime.datetime(2026, 11, 13, 11, 7, 27, tzinfo=TZ).timestamp()


def local(t):
    return datetime.datetime.fromtimestamp(float(t), TZ).isoformat(timespec='milliseconds')


def raw_bytes(msg):
    return msg[1] if isinstance(msg[1], bytes) else msg[1][0]


def doubles(raw):
    dims = struct.unpack_from('<I', raw)[0]
    offset = 4
    for _ in range(dims):
        length = struct.unpack_from('<I', raw, offset)[0]
        offset += 4 + length + 8
    offset += 4
    count = struct.unpack_from('<I', raw, offset)[0]
    offset += 4
    if offset + count * 8 != len(raw):
        raise ValueError('Float64MultiArray byte length mismatch')
    return np.frombuffer(raw, dtype='<f8', count=count, offset=offset)


def positions(mask, h):
    return h[8:11] + (np.argwhere(mask) + .5) * h[11]


def spatial(mask, h, pose):
    xyz = positions(mask, h)
    out = dict(count=int(len(xyz)))
    if not len(xyz):
        return out
    out['world_min'] = xyz.min(axis=0).tolist()
    out['world_max'] = xyz.max(axis=0).tolist()
    middle = xyz[(xyz[:, 2] >= .3) & (xyz[:, 2] <= 1.7)]
    if len(middle):
        bins = np.floor(middle[:, :2]/.2).astype(np.int32)
        values, counts = np.unique(bins, axis=0, return_counts=True)
        top = np.argsort(counts)[-6:][::-1]
        out['densest_xy_20cm_bins_z_0p3_to_1p7'] = [dict(
            world_center_xy=((values[i]+.5)*.2).tolist(), count=int(counts[i])) for i in top]
    if pose is not None:
        dx, dy = xyz[:, 0]-pose[0], xyz[:, 1]-pose[1]
        x = math.cos(pose[2])*dx + math.sin(pose[2])*dy
        y = -math.sin(pose[2])*dx + math.cos(pose[2])*dy
        front = (x >= .25) & (x <= 2.) & (np.abs(y) <= 1.) & (xyz[:, 2] >= .3) & (xyz[:, 2] <= 1.7)
        out['front_box_count'] = int(np.count_nonzero(front))
    return out


rows = []
baseline = previous = ever_new = last = reference_h = None
pose = None
prior_xyz = None
with rosbag.Bag(str(HERE/'capture.bag')) as bag:
    for topic, msg, stamp in bag.read_messages(topics=[
            '/mppi/collision_diagnostic', '/map_generator/global_cloud', '/odom'], raw=True):
        raw = raw_bytes(msg)
        receipt = stamp.to_sec()
        if topic == '/odom':
            if BASELINE_LIMIT-1. <= receipt <= BASELINE_LIMIT:
                odom = Odometry().deserialize(raw)
                q = odom.pose.pose.orientation
                pose = [odom.pose.pose.position.x, odom.pose.pose.position.y,
                        math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]
            continue
        if topic == '/map_generator/global_cloud':
            cloud = PointCloud2().deserialize(raw)
            if cloud.header.frame_id != 'world':
                raise ValueError('Prior cloud is not already in world')
            if cloud.row_step != cloud.width*cloud.point_step:
                raise ValueError('Prior cloud row padding is not supported')
            fields = {f.name: f for f in cloud.fields}
            dtype = np.dtype(dict(names=['x', 'y', 'z'],
                formats=[('<' if not cloud.is_bigendian else '>') +
                         {7: 'f4', 8: 'f8'}[fields[k].datatype] for k in ('x', 'y', 'z')],
                offsets=[fields[k].offset for k in ('x', 'y', 'z')], itemsize=cloud.point_step))
            data = np.frombuffer(cloud.data, dtype=dtype)
            prior_xyz = np.column_stack([data[k] for k in ('x', 'y', 'z')])
            prior_xyz = prior_xyz[np.isfinite(prior_xyz).all(axis=1)]
            continue
        a = doubles(raw)
        h = a[:40].copy()
        shape = tuple(h[5:8].astype(int))
        n = int(h[38])
        if h[32] != 20261112. or len(a) != 40+2*n or math.prod(shape) != n:
            raise ValueError('Unexpected diagnostic schema')
        occupied = (a[40+n:].astype(np.uint8).reshape(shape) & 1) != 0
        if reference_h is None:
            reference_h = h.copy()
        elif not np.array_equal(h[5:12], reference_h[5:12]) or h[34] != reference_h[34]:
            raise ValueError('ROI/epoch changed: world intersection alignment is required')
        index = len(rows)+1
        row = dict(index=index, receipt=receipt, receipt_local=local(receipt),
            map_commit=float(h[3]), map_commit_local=local(h[3]),
            acquisition=float(h[37]), acquisition_local=local(h[37]),
            epoch=int(h[34]), revision=int(h[35]), observed_sequence=int(h[36]),
            raw_occupied=int(np.count_nonzero(occupied)),
            header=h.tolist())
        if previous is not None:
            row['new_since_previous'] = spatial(occupied & ~previous, h, pose)
            row['cleared_since_previous'] = spatial(previous & ~occupied, h, pose)
        if index == BASELINE_INDEX:
            baseline = occupied.copy()
            ever_new = np.zeros(shape, dtype=bool)
        if baseline is not None:
            added = occupied & ~baseline
            ever_new |= added
            row['new_since_baseline'] = spatial(added, h, pose)
            row['baseline_cells_now_free'] = spatial(baseline & ~occupied, h, pose)
            row['ever_new_now_free_count'] = int(np.count_nonzero(ever_new & ~occupied))
            row['ever_new_still_occupied_count'] = int(np.count_nonzero(ever_new & occupied))
        rows.append(row)
        previous = occupied
        last = occupied
        print(index, row['receipt_local'], 'occ', row['raw_occupied'],
              'new/base', row.get('new_since_baseline', {}).get('count'),
              'cleared/base', row.get('baseline_cells_now_free', {}).get('count'), flush=True)

prior_support = np.zeros(tuple(reference_h[5:8].astype(int)), dtype=bool)
if prior_xyz is not None:
    idx = np.floor((prior_xyz-reference_h[8:11])/reference_h[11]+1e-9).astype(int)
    ok = ((idx >= 0) & (idx < reference_h[5:8])).all(axis=1)
    prior_support[tuple(idx[ok].T)] = True

result = dict(schema=1, baseline_index=BASELINE_INDEX,
    baseline_operator_confirmation_local='2026-11-13T11:07:27+08:00',
    complete_walk_and_leave_operator_reply_local='2026-11-13T11:09:33+08:00',
    walking_and_departure_exact_times_known=False,
    diagnostic_count=len(rows), first_diagnostic_local=rows[0]['receipt_local'],
    last_diagnostic_local=rows[-1]['receipt_local'],
    no_claim_of_confirmed_after_departure_diagnostic=True,
    world_grid_unchanged=True, frame='world', shape=reference_h[5:8].astype(int).tolist(),
    origin=reference_h[8:11].tolist(), resolution=float(reference_h[11]),
    baseline_odom_xyyaw=pose, raw_occupancy_bit=0,
    prior_cloud_points=None if prior_xyz is None else int(len(prior_xyz)),
    prior_cloud_support_voxels_in_roi=int(np.count_nonzero(prior_support)),
    # A published PCD cloud is useful support evidence, not the native static mask itself.
    prior_support_is_exact_native_static_mask=False,
    baseline_occupied_prior_supported=int(np.count_nonzero(baseline & prior_support)),
    baseline_occupied_not_prior_supported=int(np.count_nonzero(baseline & ~prior_support)),
    ever_new_prior_supported=int(np.count_nonzero(ever_new & prior_support)),
    ever_new_not_prior_supported=int(np.count_nonzero(ever_new & ~prior_support)),
    snapshot_rows=rows,
    caveats=[
        'Header3 is map occupancy commit ROS time, not input cloud source time.',
        'All snapshots share identical world ROI and epoch; free transitions are not ROI exits.',
        'Exact walking/departure times are unknown; added occupancy alone does not identify a person.',
        'No navigation goal was recorded in this segment.',
        'Published prior point endpoints are support evidence, not a directly exported static mask.'])
np.savez_compressed(HERE/'occupancy_audit_masks.npz', header=reference_h,
    baseline=baseline, ever_new=ever_new, last=last, prior_support=prior_support)
result['peak_rss_mib'] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024.
(HERE/'occupancy_snapshot_audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({k: v for k, v in result.items() if k != 'snapshot_rows'},ensure_ascii=False),flush=True)
