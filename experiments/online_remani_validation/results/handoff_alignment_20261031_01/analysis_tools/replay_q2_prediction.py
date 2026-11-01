#!/usr/bin/env python3
"""Read-only audit of an actually rejected candidate and its recorded live points.

Writes no results. This reconstructs time-aligned point/sphere checks, not the
missing historical native ESDF / swept overlay snapshot and not an MPPI policy.
"""
import json
import math
import hashlib
from pathlib import Path

import numpy as np

ROOT = Path('/home/l/remani_ws/experiments/online_remani_validation/results/frontend_rejoin_20261031_01/headon_penalty')


def load_rows(name):
    return [json.loads(line) for line in (ROOT / name).read_text().splitlines()]


def audit():
    events = load_rows('dual_timeline.jsonl')
    detail = next(r for r in events if r['event'] == 'latest_prediction_reject_detail')
    candidate = next(r for r in load_rows('static_width_trace.jsonl')
                     if r['request_id'] == 2 and r['stage'] == 'optimizer_return')
    parts = candidate['parts']
    body = np.array(json.loads((ROOT / 'native_collision_3d_body.json').read_text())['body'])
    points = np.array(detail['snapshot_points_xyz_velocity_track'])
    stamp = detail['snapshot_stamp']
    now = detail['query_stamp'] - detail['wall_ahead']
    epoch = detail['candidate_start']
    age = now - epoch
    horizon = 3.0 - (now - stamp)

    def state(t):
        for part in parts:
            for piece in part['pieces']:
                if t <= piece['T'] + 1e-10:
                    co = np.array(piece['coeffs'])
                    p = np.array([np.polyval(c, t) for c in co])
                    v = np.array([np.polyval(np.polyder(c), t) for c in co])
                    return p, math.atan2(part['gear'] * v[1], part['gear'] * v[0])
                t -= piece['T']
        raise ValueError('outside candidate')

    def gap(t, query):
        p, yaw = state(t)
        c, s = math.cos(yaw), math.sin(yaw)
        rot = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        centers = body[:, :3] @ rot.T + np.array([p[0], p[1], 0])
        predicted = points[:, :3] + points[:, 3:6] * (query - stamp)
        gaps = np.linalg.norm(centers[:, None, :] - predicted[None, :, :], axis=2) - body[:, 3, None]
        ix = np.unravel_index(np.argmin(gaps), gaps.shape)
        return dict(gap_m=float(gaps[ix]), body_index=int(ix[0]), point_index=int(ix[1]),
                    body_type=int(body[ix[0], 6]), candidate_xy=p[:2].tolist(), yaw=yaw,
                    local_t=float(t), query_stamp=float(query), ahead_s=float(query - now))

    def mapping(name, t_start, scale):
        # 50 ms local-time samples exactly match the native validation loop.
        samples = []
        last_t = t_start + horizon * scale
        t = t_start
        while True:
            samples.append(gap(t, now + (t - t_start) / scale))
            if t >= last_t - 1e-9:
                break
            t = min(last_t, t + .05)
        return dict(name=name, trajectory_t_at_now=t_start, execution_scale=scale,
                    minimum=min(samples, key=lambda x: x['gap_m']),
                    first_collision=next((x for x in samples if x['gap_m'] < 0), None),
                    sample_count=len(samples), samples=samples)

    robot = np.array([detail['measured_x'], detail['measured_y']])
    ts = np.linspace(0, 2, 2001)
    projection = float(ts[np.argmin([np.linalg.norm(state(t)[0][:2] - robot) for t in ts])])
    controls = [mapping('recorded_legacy_reset_half_speed', 0., .5),
                mapping('reset_full_speed_counterfactual', 0., 1.),
                mapping('epoch_progress_half_speed_counterfactual', age * .5, .5),
                mapping('epoch_progress_full_speed_counterfactual', age, 1.),
                mapping('projected_measured_progress_half_speed_counterfactual', projection, .5),
                mapping('projected_measured_progress_full_speed_counterfactual', projection, 1.)]
    source_names = ['dual_timeline.jsonl', 'static_width_trace.jsonl',
                    'native_collision_3d_body.json', 'failure_trace.jsonl', 'parameters.yaml']
    build_source = ROOT.parents[2] / 'builds/frontend_rejoin_20261031_02/source'
    code_names = ['remani_replan_fsm.cpp', 'mm_config.cpp', 'native_collision_export.hpp']
    hashes = {str(ROOT / name): hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
              for name in source_names}
    hashes.update({str(build_source / name): hashlib.sha256((build_source / name).read_bytes()).hexdigest()
                   for name in code_names})
    solve = next(r for r in load_rows('failure_trace.jsonl')
                 if r['candidate_hash'] == 11506901000537610774)
    source_identity = dict(request_id=2, optimizer_input_hash=solve['input_hash'],
                           native_candidate_hash=str(solve['candidate_hash']),
                           candidate_json_sha256=hashlib.sha256(json.dumps(candidate, sort_keys=True).encode()).hexdigest(),
                           dynamic_points_json_sha256=hashlib.sha256(json.dumps(detail['snapshot_points_xyz_velocity_track'], sort_keys=True).encode()).hexdigest(),
                           source_sha256=hashes)
    logged_gap = gap(detail['local_t'], detail['query_stamp'])
    assert abs(logged_gap['candidate_xy'][0] - detail['candidate_x']) < 1e-10
    assert abs(logged_gap['candidate_xy'][1] - detail['candidate_y']) < 1e-10
    assert abs(logged_gap['yaw'] - detail['candidate_yaw']) < 1e-10
    assert logged_gap['gap_m'] < 0 and logged_gap['body_type'] == detail['collision_type']
    assert abs(controls[0]['first_collision']['local_t'] - detail['local_t']) < 1e-10
    output = dict(scope='same rejected candidate, exact recorded latest CV points, native exported fixed-arm sphere radii; no historical map replay; alternative mappings are counterfactual, not safe execution authorization',
                  candidate_was_executed=False, source_identity=source_identity,
                  parameters=dict(prediction_model='per-point CV; original full point cloud',
                      uncertainty_m=0., map_resolution_m=.05,
                      hard_radius_rule='base sphere radius + 0.05 m map resolution; arm sphere radius; safe=false',
                      sphere_count=int(len(body)), base_spheres=int(np.sum(body[:, 6] == 0)),
                      arm_spheres=int(np.sum(body[:, 6] == 1)), dynamic_points=len(points),
                      body_radius_source='native_collision_3d_body.json body[:,3], native_collision_export.hpp',
                      collision_rule='minimum point-to-sphere surface gap < 0',
                      native_local_sample_step_s=.05, max_prediction_time_s=3.,
                      prediction_parameters_source='parameters.yaml dynamic_point_flow; position/velocity/acceleration uncertainty all zero',
                      mapping_source='remani_replan_fsm.cpp validatePredictedDynamicTrajectory; original rejection record execution_scale=0.5'),
                  now=now, candidate_epoch=epoch, candidate_age_s=age,
                  current_robot_xy=robot.tolist(),
                  measured_position_delta_from_candidate_start_m=float(np.linalg.norm(state(0)[0][:2] - robot)),
                  projected_measured_progress_t=projection,
                  original_logged_failure=logged_gap,
                  checks=controls)
    return output


if __name__ == '__main__':
    print(json.dumps(audit(), ensure_ascii=False, indent=2))
