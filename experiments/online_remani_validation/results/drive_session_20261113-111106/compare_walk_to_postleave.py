#!/usr/bin/python3
"""Offline world-aligned comparison of two closed capture segments."""
import ast
import datetime
import gc
import json
import math
from pathlib import Path
import resource
import struct

import numpy as np
import rosbag
from geometry_msgs.msg import PoseStamped

HERE = Path(__file__).resolve().parent
OLD = HERE.parent/'drive_session_20261113-110030'
TZ = datetime.timezone(datetime.timedelta(hours=8))
helpers = ast.parse((OLD/'audit_occupancy_snapshots.py').read_text())
exec(compile(ast.Module(body=[n for n in helpers.body if isinstance(n, ast.FunctionDef)
    and n.name in ('local', 'raw_bytes', 'doubles', 'positions', 'spatial')],
    type_ignores=[]), str(OLD/'audit_occupancy_snapshots.py'), 'exec'))
audit = json.loads((OLD/'occupancy_snapshot_audit.json').read_text())
pose = audit['baseline_odom_xyyaw']
with np.load(OLD/'occupancy_audit_masks.npz') as z:
    old_h = z['header'].copy()
    masks = {k: z[k].copy() for k in ('baseline', 'ever_new', 'last', 'prior_support')}
origin = old_h[8:11]
res = old_h[11]
shape = old_h[5:8].astype(int)
# These are spatial/time subsets, not ground-truth human segmentation.
xx = origin[0]+(np.arange(shape[0])+.5)*res-pose[0]
yy = origin[1]+(np.arange(shape[1])+.5)*res-pose[1]
zz = origin[2]+(np.arange(shape[2])+.5)*res
x = math.cos(pose[2])*xx[:, None]+math.sin(pose[2])*yy[None, :]
y = -math.sin(pose[2])*xx[:, None]+math.cos(pose[2])*yy[None, :]
front = (((x >= .25) & (x <= 2.) & (np.abs(y) <= 1.))[:, :, None] &
         ((zz >= .3) & (zz <= 1.7))[None, None, :])
masks['last_new'] = masks['last'] & ~masks['baseline']
masks['ever_new_front_box'] = masks['ever_new'] & front
masks['last_new_front_box'] = masks['last_new'] & front
seen_early = masks['baseline'].copy()
late = np.zeros(tuple(shape), dtype=bool)
with rosbag.Bag(str(OLD/'capture.bag')) as bag:
    for i, (_, msg, _) in enumerate(bag.read_messages(topics=['/mppi/collision_diagnostic'], raw=True), 1):
        a = doubles(raw_bytes(msg)); n = int(a[38])
        occ = (a[40+n:].astype(np.uint8).reshape(tuple(shape)) & 1) != 0
        if 6 <= i <= 11:
            seen_early |= occ
        if i >= 12:
            late |= occ
del bag
gc.collect()
masks['late_front_new'] = late & ~seen_early & front
tracked = ('ever_new', 'last_new', 'ever_new_front_box', 'last_new_front_box', 'late_front_new')


def aligned_slices(h):
    if h[34] != old_h[34] or not math.isclose(h[11], res, abs_tol=1e-12):
        raise ValueError('Map epoch/resolution changed; clearing comparison invalid')
    floating = (origin-h[8:11])/res
    offset = np.rint(floating).astype(int)
    if not np.allclose(offset, floating, atol=1e-7):
        raise ValueError('World grids are not integer-aligned')
    dst_shape = h[5:8].astype(int)
    lo = np.maximum(0, -offset)
    hi = np.minimum(shape, dst_shape-offset)
    if np.any(lo >= hi):
        raise ValueError('World ROIs have no common volume')
    return tuple(slice(a, b) for a, b in zip(lo, hi)), tuple(
        slice(a, b) for a, b in zip(lo+offset, hi+offset))


def precise_esdf(points, h, field):
    uvw = (points-h[8:11])/h[11]-.5
    idx = np.floor(uvw).astype(int)
    frac = uvw-idx
    out = np.full(len(points), np.nan)
    for k, (ind, alpha) in enumerate(zip(idx, frac)):
        if np.any(ind < 0) or np.any(ind+1 >= h[5:8]):
            continue
        dist = 0.
        for dx in (0, 1):
            for dy in (0, 1):
                for dz in (0, 1):
                    w = ((alpha[0] if dx else 1-alpha[0]) *
                         (alpha[1] if dy else 1-alpha[1]) *
                         (alpha[2] if dz else 1-alpha[2]))
                    dist += w*field[tuple(ind+[dx, dy, dz])]
        out[k] = dist
    return out


with rosbag.Bag(str(HERE/'capture.bag')) as bag:
    goals = list(bag.read_messages(topics=['/move_base_simple/goal'], raw=True))
del bag
gc.collect()
if len(goals) != 1:
    raise ValueError('Expected one goal')
goal = PoseStamped().deserialize(raw_bytes(goals[0][1]))
q = goal.pose.orientation
goal_pose = [goal.pose.position.x, goal.pose.position.y,
             math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]
if goal.header.frame_id != 'world':
    raise ValueError('Goal frame is not world')
params = json.loads((HERE/'runtime_parameters.json').read_text())['/remani_planner_node']
fixed = np.asarray(params['mm']['fixed_collision_spheres'])
c, s = math.cos(goal_pose[2]), math.sin(goal_pose[2])
centers = fixed[:, :3].copy()
centers[:, :2] = centers[:, :2] @ np.array([[c, s], [-s, c]]) + goal_pose[:2]
rows = []
last_remaining_masks = {}
with rosbag.Bag(str(HERE/'capture.bag')) as bag:
    for i, (_, msg, stamp) in enumerate(bag.read_messages(topics=['/mppi/collision_diagnostic'], raw=True), 1):
        a = doubles(raw_bytes(msg)); h = a[:40].copy(); n = int(h[38])
        field = a[40:40+n].reshape(tuple(h[5:8].astype(int)))
        occ = (a[40+n:].astype(np.uint8).reshape(tuple(h[5:8].astype(int))) & 1) != 0
        src_slice, dst_slice = aligned_slices(h)
        row = dict(index=i, receipt_local=local(stamp.to_sec()),
            map_commit_local=local(h[3]), acquisition_local=local(h[37]),
            header=h.tolist(), comparisons={})
        for name in tracked:
            mask = masks[name]
            shared = mask[src_slice]
            remaining = shared & occ[dst_slice]
            free = shared & ~occ[dst_slice]
            rem_world = np.zeros(tuple(shape), dtype=bool)
            rem_world[src_slice] = remaining
            row['comparisons'][name] = dict(total=int(np.count_nonzero(mask)),
                covered=int(np.count_nonzero(shared)), remaining=int(np.count_nonzero(remaining)),
                now_free=int(np.count_nonzero(free)),
                outside_current_roi=int(np.count_nonzero(mask)-np.count_nonzero(shared)),
                remaining_spatial=spatial(rem_world, old_h, pose))
            last_remaining_masks[name] = rem_world
        distances = precise_esdf(centers, h, field)
        gaps = distances-fixed[:, 3]-res
        row['goal_fixed_structure_native_occupancy_esdf'] = dict(
            sphere_count=len(fixed), fully_in_roi=bool(np.isfinite(gaps).all()),
            minimum_gap=float(np.nanmin(gaps)), colliding_sphere_indices=np.flatnonzero(gaps < 0).tolist(),
            gap_by_sphere=gaps.tolist(), margin_policy='radius + one map resolution; safe=false',
            excludes_dynamic_overlay_and_arm_links=True)
        rows.append(row)
        print(i, row['receipt_local'], 'new/free',row['comparisons']['ever_new']['now_free'],
              'remain',row['comparisons']['ever_new']['remaining'],
              'front late',row['comparisons']['late_front_new']['remaining'],
              'goal_occupancy_min_gap',row['goal_fixed_structure_native_occupancy_esdf']['minimum_gap'],flush=True)

result = dict(schema=1,old_segment=str(OLD),new_segment=str(HERE),
    operator_confirmed_departure_local='2026-11-13T11:09:33+08:00',
    old_last_diag_local=audit['last_diagnostic_local'],
    goal_source_local=local(goal.header.stamp.to_sec()),
    goal_receipt_local=local(goals[0][2].to_sec()),goal_world_xyyaw=goal_pose,
    tracked_sets={name: spatial(masks[name],old_h,pose) for name in tracked},
    late_front_definition='In front box, first appearing in diagnostic12..15 (11:08:28..58), absent from diagnostic6..11; not ground-truth person labels.',
    rows=rows,
    caveats=['now_free counts only cells still inside the exact integer-aligned world ROI.',
        'Native occupancy ESDF includes both prior and live occupancy; replay excludes predictive overlay, arm links and timing validity.',
        'Map diagnostic header3 is commit time, not contributing cloud source time.',
        'Old walking/departure exact times are unavailable; not all additions are attributed to a human.'])
np.savez_compressed(HERE/'old_walk_residual_masks.npz',header=old_h,
    **{'last_remaining_'+name: value for name,value in last_remaining_masks.items()},
    **{'tracked_'+name: masks[name] for name in tracked})
result['peak_rss_mib']=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024.
(HERE/'walk_to_postleave_comparison.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print('Peak_RSS_MiB',result['peak_rss_mib'],flush=True)
