#!/usr/bin/env python3
"""Inspect residual base collisions from frozen native accepted checkpoints."""
import gzip
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from backend_base_feasibility_breakdown import state_values
from rootcause_components import TRIAL, HERE, Probe, attest, context
from audit_delivery_iterates import loads_preserving_cpp_zero

OUT = HERE/'results/root_cause_20261030_01'


class FrozenMap:
    def __init__(self):
        self.meta = json.loads((TRIAL/'candidate_audit/map.json').read_text())
        m = self.meta
        self.origin = np.asarray(m['map_origin'])
        self.res = m['resolution']
        self.shape = np.asarray(m['map_voxel_num'])
        self.esdf = np.frombuffer(gzip.open(m['esdf_path'], 'rb').read(), dtype=np.float64).reshape(self.shape)
        self.overlay = np.frombuffer(gzip.open(m['overlay_path'], 'rb').read(), dtype=np.float32).reshape(-1, 4)
        self.tree = cKDTree(self.overlay[:, :3])

    def static(self, points):
        points = np.asarray(points)
        idx = np.floor((points-self.origin)/self.res-.5).astype(int)
        diff = (points-(idx+.5)*self.res-self.origin)/self.res
        d = np.zeros(len(points)); g = np.zeros((len(points), 3))
        for x in range(2):
            for y in range(2):
                for z in range(2):
                    offset = np.array([x, y, z]); ids = np.clip(idx+offset, 0, self.shape-1)
                    values = self.esdf[ids[:, 0], ids[:, 1], ids[:, 2]]
                    factors = np.where(offset, diff, 1-diff)
                    d += np.prod(factors, axis=1)*values
                    for k in range(3):
                        other = [j for j in range(3) if j != k]
                        g[:, k] += np.prod(factors[:, other], axis=1)*values*(1 if offset[k] else -1)/self.res
        return d, g

    def query(self, points):
        stat, grad = self.static(points)
        dyn = np.full(len(points), np.inf)
        dyn_grad = np.zeros_like(grad)
        active_index = np.full(len(points), -1, dtype=int)
        m = self.meta
        maxdistance = np.minimum(np.maximum(stat, 0), m['overlay_influence_distance'])
        search = maxdistance+m['overlay_inflation']
        inside = np.all((points >= np.asarray(m['overlay_min'])-search[:, None]) &
                        (points <= np.asarray(m['overlay_max'])+search[:, None]), axis=1)
        for i in np.flatnonzero(inside & (maxdistance > 0)):
            query = points[i].astype(np.float32).astype(float)
            choices = self.tree.query_ball_point(query, search[i])
            if not choices:
                continue
            ds = np.linalg.norm(self.overlay[choices, :3].astype(float)-query, axis=1)
            gap = ds-self.overlay[choices, 3]
            at = int(np.argmin(gap)); index = choices[at]
            if gap[at] > maxdistance[i]:
                continue
            dyn[i], active_index[i] = gap[at], index
            if ds[at] > 1e-6:
                dyn_grad[i] = (points[i]-self.overlay[index, :3])/ds[at]
        uses_dyn = dyn < stat
        dist = np.minimum(stat, dyn)
        grad[uses_dyn] = dyn_grad[uses_dyn]
        return dist, grad, stat, dyn, active_index


def pose(value, ctx, t):
    at = float(t)
    for n, (coeff, times, gear) in enumerate(zip(value['b_parts'], value['times_parts'], ctx.gears)):
        for i, duration in enumerate(times):
            if at <= duration or (n == len(ctx.gears)-1 and i == len(times)-1):
                c = coeff[8*i:8*i+8]
                p = np.polynomial.polynomial.polyval(at, c)
                v, unused_a, unused_j = [q[0] for q in state_values(c, [at])]
                return np.asarray(p[:2]), math.atan2(gear*v[1], gear*v[0]), n, i, at
            at -= duration
    raise ValueError('trajectory time')


def samples(value, ctx, ts, spheres, field):
    centers = []; poses = []
    for t in ts:
        p, yaw, n, i, local = pose(value, ctx, t)
        rot = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        center = spheres[:, :3].copy(); center[:, :2] = center[:, :2]@rot.T+p
        centers.append(center); poses.append([*p, yaw, n, i, local])
    centers = np.asarray(centers)
    d, g, stat, dyn, index = field.query(centers.reshape(-1, 3))
    shape = (len(ts), len(spheres))
    return dict(times=np.asarray(ts), poses=np.asarray(poses), centers=centers,
        dist=d.reshape(shape), grad=g.reshape((*shape, 3)), static=stat.reshape(shape),
        dynamic=dyn.reshape(shape), overlay_index=index.reshape(shape),
        hard_gap=d.reshape(shape)-spheres[:, 3]-.05,
        soft_error=np.maximum(spheres[:, 3]+.1-d.reshape(shape), 0))


def detail(s, frame, ball, spheres, field):
    o = int(s['overlay_index'][frame, ball])
    return dict(t=float(s['times'][frame]), sphere_index=int(ball),
        robot_pose=s['poses'][frame, :3].tolist(), segment=int(s['poses'][frame, 3]),
        piece=int(s['poses'][frame, 4]), local_t=float(s['poses'][frame, 5]),
        sphere_local=spheres[ball].tolist(), center=s['centers'][frame, ball].tolist(),
        planning_distance=float(s['dist'][frame, ball]), static_distance=float(s['static'][frame, ball]),
        dynamic_distance=None if not np.isfinite(s['dynamic'][frame, ball]) else float(s['dynamic'][frame, ball]),
        hard_gap=float(s['hard_gap'][frame, ball]), soft_error=float(s['soft_error'][frame, ball]),
        distance_gradient=s['grad'][frame, ball].tolist(), gradient_norm=float(np.linalg.norm(s['grad'][frame, ball])),
        nearest_overlay=None if o < 0 else field.overlay[o].tolist())


def main():
    parameters = json.loads((TRIAL/'effective_real_params.json').read_text())
    spheres = np.asarray(parameters['/remani_planner_node/mm/fixed_collision_spheres'])
    runs = [loads_preserving_cpp_zero(l) for l in (TRIAL/'candidate_audit/endpoint_time_replay.jsonl').open()]
    run = next(r for r in runs if r.get('label') == 'original_060')
    ctx = context(run, parameters)
    probe = Probe(attest()['library']); field = FrozenMap()
    report = dict(scope='independent base-only residual-collision geometry; native acceptance authority is saved C++ verdict',
        sphere_count=len(spheres), native_map_metadata=field.meta, trials=[])
    wanted = [('native_run_02', 'original_initial_300'), ('native_run_03', 'original_delta1e6_300')]
    for directory, label in wanted:
        result = next(r for r in (json.loads(l) for l in (OUT/directory/'results.jsonl').open()) if r.get('label') == label)
        last = result['stages'][-1]; value = probe.evaluate(np.asarray(last['x']), ctx)
        total = sum(sum(t) for t in value['times_parts']); ts=[]; t=.05
        for unused in range(5, math.floor(total/.01)):
            ts.append(t); t += .01
        native = samples(value, ctx, ts, spheres, field)
        first = np.argwhere(native['hard_gap'] < 0)[0]
        worst = np.unravel_index(np.argmin(native['hard_gap']), native['hard_gap'].shape)
        opt_times = []; weights=[]; elapsed=0.
        for times in value['times_parts']:
            for duration in times:
                for j in range(17):
                    opt_times.append(elapsed+duration*j/16)
                    weights.append(duration/16*(.5 if j in (0, 16) else 1))
                elapsed += duration
        opt = samples(value, ctx, opt_times, spheres, field)
        opt_worst = np.unravel_index(np.argmin(opt['hard_gap']), opt['hard_gap'].shape)
        cost = 50000*np.sum(np.asarray(weights)[:, None]*opt['soft_error']**3)
        first_time=detail(native, *first, spheres, field)
        near = [i for i,t in enumerate(opt_times) if abs(t-first_time['t']) < .25]
        row = dict(run=directory, label=label, result=result['result'], native_reason=last['native_reason'],
            native_rejection_time=last['native_rejection_time'],
            independent_first_base_collision=first_time,
            independent_worst_base_collision=detail(native, *worst, spheres, field),
            optimizer_grid_worst=detail(opt, *opt_worst, spheres, field),
            optimizer_grid_colliding_nodes=int(np.sum(np.any(opt['hard_gap'] < 0, axis=1))),
            optimizer_grid_total_nodes=len(opt_times),
            independent_base_obstacle_integral=cost, saved_base_obstacle_integral=last['components']['base_obstacle'],
            relative_cost_error=abs(cost-last['components']['base_obstacle'])/max(1, abs(cost)),
            nearby_K16_samples=[detail(opt, i, int(np.argmin(opt['hard_gap'][i])), spheres, field) for i in near])
        if abs(first_time['t']-last['native_rejection_time']) > .011:
            raise ValueError('Independent first collision differs from native verdict')
        if row['relative_cost_error'] > 1e-4:
            raise ValueError('Independent obstacle cost differs from native integral: '+str(row['relative_cost_error']))
        report['trials'].append(row)
        print(json.dumps({k:v for k,v in row.items() if k != 'nearby_K16_samples'}, ensure_ascii=False, indent=2))
    (OUT/'collision_residual_analysis.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
