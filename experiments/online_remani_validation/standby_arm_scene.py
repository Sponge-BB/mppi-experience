"""Standby whole-body experiment scene and an offline arm-only witness.

The clock starts at the experiment start, independently of robot motion.
Actors are ground truth for synthetic sensing and evaluation only.  No
controller may read ``witness_joints`` or the future actor schedule as a
control reference.  Both the base and the arm remain available to controllers.
"""
from copy import deepcopy
import math

import numpy as np

from active_arm_scene import cylinder_gaps


SCENE = 'standby_arm_moving_obstacle'
Q_START = np.deg2rad([0., 110., -60., 0., 30., 0.])
Q_RETRACT = np.deg2rad([0., 140., -60., 0., 15., 0.])


def scene_definition():
    return dict(
        schema='standby_arm_scene_v1', scene=SCENE,
        name='待机伸展机械臂与移动高位圆柱', duration_s=24.,
        time_origin='independent experiment start', appearance_s=2.,
        arena=[-1.2, 1.2, -1.4, 1.4],
        navigation_goal=None, base_constraint='free',
        q_start=Q_START.tolist(), posture_prior=Q_RETRACT.tolist(),
        actors=[dict(id=1, x=.16, y=.65, vx=0., vy=-.08, r=.13,
                     z_min=1.05, z_max=1.30)],
        witness=dict(outbound_start_s=4., transition_duration_s=2.,
                     return_start_s=16.,
                     role='offline arm-only feasibility witness; never a control command'),
        input_contract='Only synthetic measured cloud enters the controller',
        comparison=['native_piper', 'current_unmodified', 'current_standby_extension'],
        scope='native DH 67-sphere numerical plant; no hardware or mesh claim')


def validate_scene(scene):
    if scene.get('schema') != 'standby_arm_scene_v1':
        raise ValueError('Unknown standby scene schema')
    if scene.get('base_constraint') != 'free' or scene.get('navigation_goal') is not None:
        raise ValueError('This experiment requires a free base and no navigation goal')
    for name in ('q_start', 'posture_prior'):
        q = np.asarray(scene[name], dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError('Expected six finite joint angles: ' + name)
    if not 0 <= scene['appearance_s'] < scene['duration_s']:
        raise ValueError('Invalid independent scene time')
    ids = set()
    for actor in scene['actors']:
        if actor['id'] in ids or not all(math.isfinite(actor[k]) for k in
                  ('x', 'y', 'vx', 'vy', 'r', 'z_min', 'z_max')):
            raise ValueError('Invalid actor or duplicate ID')
        if actor['r'] <= 0 or actor['z_max'] <= actor['z_min']:
            raise ValueError('Invalid finite cylinder')
        ids.add(actor['id'])
    witness = scene['witness']
    if (witness['transition_duration_s'] <= 0 or witness['outbound_start_s'] < 0
            or witness['outbound_start_s'] + witness['transition_duration_s'] > witness['return_start_s']
            or witness['return_start_s'] + witness['transition_duration_s'] > scene['duration_s']):
        raise ValueError('Invalid offline witness timing')
    return True


def actors_3d_at(elapsed, scene=None):
    scene = scene_definition() if scene is None else scene
    validate_scene(scene)
    if not math.isfinite(elapsed):
        raise ValueError('Finite scene time required')
    if elapsed < scene['appearance_s']:
        return []
    actors = deepcopy(scene['actors'])
    dt = elapsed - scene['appearance_s']
    for actor in actors:
        actor['x'] += actor['vx'] * dt
        actor['y'] += actor['vy'] * dt
    return actors


def actors_at(elapsed, scene=None):
    """Legacy obstacle log rows x,y,r,id,vx,vy."""
    return [(a['x'], a['y'], a['r'], a['id'], a['vx'], a['vy'])
            for a in actors_3d_at(elapsed, scene)]


def obstacle_points(elapsed, scene=None, spacing=.035):
    if not math.isfinite(spacing) or spacing <= 0:
        raise ValueError('Positive finite point spacing required')
    points = []
    for a in actors_3d_at(elapsed, scene):
        angles = np.linspace(0., 2. * np.pi,
            max(12, int(math.ceil(2. * np.pi * a['r'] / spacing))), endpoint=False)
        heights = np.linspace(a['z_min'], a['z_max'],
            max(2, int(math.ceil((a['z_max'] - a['z_min']) / spacing)) + 1))
        points.extend((a['x'] + a['r'] * np.cos(angle),
                       a['y'] + a['r'] * np.sin(angle), z)
                      for angle in angles for z in heights)
        grid = np.arange(-a['r'], a['r'] + .5 * spacing, spacing)
        points.extend((a['x'] + x, a['y'] + y, z)
                      for z in (a['z_min'], a['z_max']) for x in grid for y in grid
                      if x*x + y*y <= a['r']**2)
    return np.asarray(points, dtype=np.float32).reshape((-1, 3))


def witness_joints(times, scene=None):
    """Offline cubic rest-to-rest q,dq,ddq; never exported to a controller."""
    scene = scene_definition() if scene is None else scene
    validate_scene(scene)
    ts = np.atleast_1d(np.asarray(times, dtype=float))
    if not np.isfinite(ts).all():
        raise ValueError('Finite witness times required')
    schedule = scene['witness']; duration = schedule['transition_duration_s']
    def cubic(start):
        a = np.clip((ts - start) / duration, 0., 1.)
        inside = (ts >= start) & (ts <= start + duration)
        return (3.*a*a - 2.*a*a*a,
                np.where(inside, 6.*a*(1.-a)/duration, 0.),
                np.where(inside, (6.-12.*a)/duration**2, 0.))
    out = cubic(schedule['outbound_start_s']); back = cubic(schedule['return_start_s'])
    scale = [a - b for a, b in zip(out, back)]
    q0 = np.asarray(scene['q_start']); delta = np.asarray(scene['posture_prior']) - q0
    return (q0 + scale[0][:, None]*delta,
            scale[1][:, None]*delta, scale[2][:, None]*delta)


def geometry_precheck(model, scene=None, step_s=.005):
    """Prove sampled arm-only avoidability, without constraining actual base."""
    scene = scene_definition() if scene is None else deepcopy(scene)
    validate_scene(scene)
    if not math.isfinite(step_s) or not 0 < step_s <= .01:
        raise ValueError('Geometry audit step must lie in (0, .01] s')
    times = np.linspace(0., scene['duration_s'],
                        int(math.ceil(scene['duration_s']/step_s)) + 1)
    q, dq, ddq = witness_joints(times, scene)
    base = np.zeros((len(times), 3))
    bodies = model.body_spheres(q, base)
    if bodies.shape[1:] != (67, 4):
        raise ValueError('Expected native 47 fixed + 20 arm spheres')
    start_body = model.body_spheres(np.asarray(scene['q_start']))
    witness_gaps = []; held_gaps = []; fixed_gaps = []
    for t, body in zip(times, bodies):
        actors = actors_3d_at(float(t), scene)
        witness_gaps.append(float(np.min(cylinder_gaps(body, actors))))
        held_gaps.append(float(np.min(cylinder_gaps(start_body, actors))))
        fixed_gaps.append(float(np.min(cylinder_gaps(start_body[:47], actors))))
    q_gap = np.min(np.minimum(q-model.lower, model.upper-q), axis=0)
    speed_gap = np.min(model.velocity_limits-np.abs(dq), axis=0)
    accel_gap = np.min(model.acceleration_limits-np.abs(ddq), axis=0)
    configuration_gap = model.configuration_gaps_batch(q)
    collision_indices = np.flatnonzero(np.asarray(held_gaps) < 0.)
    result = dict(schema='standby_geometry_precheck_v1',
        sample_count=len(times), maximum_step_s=float(np.max(np.diff(times))),
        held_arm_min_external_gap_m=float(np.min(held_gaps)),
        held_arm_first_sampled_collision_s=float(times[collision_indices[0]]) if len(collision_indices) else None,
        fixed_structure_min_external_gap_m=float(np.min(fixed_gaps)),
        arm_only_witness_min_external_gap_m=float(np.min(witness_gaps)),
        arm_only_witness_min_configuration_gap_m=float(np.min(configuration_gap)),
        arm_only_witness_position_margin_rad=q_gap.tolist(),
        arm_only_witness_velocity_margin_rad_s=speed_gap.tolist(),
        arm_only_witness_acceleration_margin_rad_s2=accel_gap.tolist(),
        witness_base_pose=[0., 0., 0.], actual_experiment_base_constraint='free',
        scope=('Offline sampled feasibility witness only. The actual controller does not '
               'receive its schedule. No online outcome, latency or mesh certification is implied.'))
    result['passed'] = bool(result['held_arm_min_external_gap_m'] < -.01
        and result['fixed_structure_min_external_gap_m'] > .05
        and result['arm_only_witness_min_external_gap_m'] > .05
        and np.min(configuration_gap) >= -1e-10
        and min(np.min(q_gap), np.min(speed_gap), np.min(accel_gap)) >= -1e-10)
    return result
