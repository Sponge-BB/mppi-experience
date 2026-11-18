"""Declared elevated obstacle for active-arm numerical ROS experiments.

Ground truth in this module is for the synthetic sensor and independent
evaluation only. The controller must consume its usual measured point cloud.
All dimensions are metres and all joint values are radians.
"""
from copy import deepcopy
import math

import numpy as np


SCENE = 'active_arm_elevated_obstacle'
Q_START = np.deg2rad([0., 110., -60., 0., 30., 0.])
# Predeclared configuration-space witness, not a motion command. The runner
# must verify it against its selected model before claiming arm avoidability.
Q_RETRACT = np.deg2rad([0., 140., -60., 0., 15., 0.])


def scene_definition():
    """Return an independent copy for the immutable per-trial manifest."""
    return dict(
        schema='active_arm_scene_v1', scene=SCENE,
        name='高处有限圆柱：固定臂与主动收臂',
        time_origin='measured motion anchor', appearance_s=2., persistent=True,
        goal=[3., 0.], arena=[-1.5, 4.5, -2., 2.],
        q_start=Q_START.tolist(), q_retract=Q_RETRACT.tolist(),
        q_retract_role='geometric feasibility witness, not a truth-driven command',
        actors=[dict(id=1, x=1.6, y=0., vx=0., vy=0., r=.14,
                     z_min=1.05, z_max=1.30)],
        input_contract='Only synthetic measured cloud enters planner/controller',
        comparison='same scene and q_start; A fixed arm, B optimized arm',
        scope='numerical ROS plant; fixed-base geometry retained; no hardware',
        geometry_precheck_required=True)


def validate_scene(scene):
    if scene.get('schema') != 'active_arm_scene_v1':
        raise ValueError('Unknown active-arm scene schema')
    for name in ('q_start', 'q_retract'):
        q = np.asarray(scene[name], dtype=float)
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            raise ValueError('Expected six finite joints: ' + name)
    if not np.isfinite(scene['appearance_s']) or scene['appearance_s'] < 0:
        raise ValueError('Invalid appearance time')
    ids = set()
    for a in scene['actors']:
        if a['id'] in ids or not all(np.isfinite(a[k]) for k in
                                    ('x', 'y', 'vx', 'vy', 'r', 'z_min', 'z_max')):
            raise ValueError('Invalid or duplicate actor')
        if a['r'] <= 0 or a['z_max'] <= a['z_min']:
            raise ValueError('Invalid finite cylinder dimensions')
        ids.add(a['id'])
    return True


def actors_3d_at(elapsed, scene=None):
    """Finite-cylinder truth, with appearance-relative prescribed motion."""
    scene = scene_definition() if scene is None else scene
    validate_scene(scene)
    if elapsed < scene['appearance_s']:
        return []
    dt = float(elapsed) - scene['appearance_s']
    rows = deepcopy(scene['actors'])
    for actor in rows:
        actor['x'] += dt * actor['vx']
        actor['y'] += dt * actor['vy']
    return rows


def actors_at(elapsed, scene=None):
    """Legacy six-column rows for logging/proposals, not 3-D authorization."""
    return [(a['x'], a['y'], a['r'], a['id'], a['vx'], a['vy'])
            for a in actors_3d_at(elapsed, scene)]


def obstacle_points(elapsed, scene=None, spacing=.035):
    """Dense finite surfaces for sensor generation, including both caps."""
    if not math.isfinite(spacing) or spacing <= 0:
        raise ValueError('Positive finite cloud spacing required')
    clouds = []
    for actor in actors_3d_at(elapsed, scene):
        r = actor['r']
        angles = np.linspace(0., 2. * np.pi,
                             max(12, int(math.ceil(2. * np.pi * r / spacing))),
                             endpoint=False)
        heights = np.linspace(actor['z_min'], actor['z_max'],
                              max(2, int(math.ceil((actor['z_max'] - actor['z_min']) / spacing)) + 1))
        side = [(actor['x'] + r * np.cos(angle), actor['y'] + r * np.sin(angle), z)
                for angle in angles for z in heights]
        grid = np.arange(-r, r + spacing * .5, spacing)
        caps = [(actor['x'] + x, actor['y'] + y, z)
                for z in (actor['z_min'], actor['z_max'])
                for x in grid for y in grid if x*x + y*y <= r*r]
        clouds.extend(side + caps)
    return np.asarray(clouds, dtype=np.float32).reshape((-1, 3))


def cylinder_gaps(spheres_world, actors):
    """Analytic sphere-versus-solid-finite-cylinder signed gaps."""
    spheres = np.asarray(spheres_world, dtype=float)
    if spheres.ndim != 2 or spheres.shape[1] != 4 or not len(spheres):
        raise ValueError('Expected nonempty N x 4 physical spheres')
    if not np.all(np.isfinite(spheres)) or np.any(spheres[:, 3] <= 0):
        raise ValueError('Invalid physical sphere geometry')
    gaps = np.full(len(spheres), np.inf)
    for a in actors:
        radial = np.hypot(spheres[:, 0] - a['x'], spheres[:, 1] - a['y']) - a['r']
        vertical = (np.abs(spheres[:, 2] - .5 * (a['z_min'] + a['z_max']))
                    - .5 * (a['z_max'] - a['z_min']))
        distance = (np.hypot(np.maximum(radial, 0.), np.maximum(vertical, 0.))
                    + np.minimum(np.maximum(radial, vertical), 0.))
        gaps = np.minimum(gaps, distance - spheres[:, 3])
    return gaps


def sphere_clearance(spheres_world, elapsed, scene=None):
    return float(np.min(cylinder_gaps(spheres_world, actors_3d_at(elapsed, scene))))


def register_scene(scene=None):
    """Register compatibility metadata before existing prepare_trial validation.

    The trial hook must additionally replace cloud height generation and the
    truth monitor with this module's finite 3-D surfaces. Registration alone
    would leave legacy actors incorrectly extending down to the floor.
    """
    import dynamic_scenarios as target
    scene = scene_definition() if scene is None else scene
    validate_scene(scene)
    actor_rows = tuple((a['x'], a['y'], a['vx'], a['vy'], a['r'])
                       for a in scene['actors'])
    target.CASES[scene['scene']] = ('active_arm', scene['name'], actor_rows)
    target.SCENES = tuple(dict.fromkeys((*target.SCENES, scene['scene'])))
    return deepcopy(scene)


def fixed_arm_bypass(scene=None, samples=301, lateral_amplitude=.30):
    """Predeclared smooth geometric witness; never a controller command."""
    scene = scene_definition() if scene is None else scene
    x = np.linspace(0., scene['goal'][0], samples)
    progress = np.clip((x-.30)/2.50, 0., 1.)
    y = -lateral_amplitude*np.sin(np.pi*progress)**2
    derivative = -lateral_amplitude*np.pi/2.50*np.sin(2.*np.pi*progress)
    return np.column_stack([x, y, np.arctan(derivative)])


def straight_path_precheck(model, scene=None, samples=121, joint_samples=257):
    """Geometry-only witness, not an online/controller success result."""
    scene = scene_definition() if scene is None else deepcopy(scene)
    validate_scene(scene)
    if samples < 2 or joint_samples < 17:
        raise ValueError('At least two base path samples required')
    actors = actors_3d_at(scene['appearance_s'], scene)
    result = {}
    for label, q in [('start', scene['q_start']), ('retract', scene['q_retract'])]:
        body = np.asarray(model.body_spheres(np.asarray(q)), dtype=float)
        arm = np.asarray(model.arm_spheres(np.asarray(q)), dtype=float)
        if len(body) != 67 or len(arm) != 20:
            raise ValueError('Expected 47 fixed plus 20 moving-arm spheres')
        fixed = body[:47]
        current = {'fixed': np.inf, 'arm': np.inf}
        for x in np.linspace(0., scene['goal'][0], samples):
            for name, source in [('fixed', fixed), ('arm', arm)]:
                world = source.copy()
                world[:, 0] += x
                current[name] = min(current[name], float(np.min(cylinder_gaps(world, actors))))
        result[label] = dict(fixed_min_gap_m=float(current['fixed']),
                             arm_min_gap_m=float(current['arm']),
                             model_check=model.check(np.asarray(q)))
    transition = [(1.-a)*np.asarray(scene['q_start']) + a*np.asarray(scene['q_retract'])
                  for a in np.linspace(0., 1., joint_samples)]
    sweep = [model.check(q) for q in transition]
    result['joint_transition_sampled_valid'] = all(row['valid'] for row in sweep)
    result['joint_transition'] = dict(
        samples=joint_samples, all_valid=result['joint_transition_sampled_valid'],
        min_self_gap_m=float(min(row['min_self_gap'] for row in sweep)),
        min_base_arm_gap_m=float(min(row['min_base_arm_gap'] for row in sweep)),
        min_ground_gap_m=float(min(row['min_ground_gap'] for row in sweep)),
        initial_base_pose_external_gap_m=float(min(
            np.min(cylinder_gaps(model.body_spheres(q), actors)) for q in transition)))
    distance = np.abs(np.asarray(scene['q_retract'])-scene['q_start'])
    speed, accel = model.velocity_limits, model.acceleration_limits
    individual_time = np.where(distance > speed*speed/accel,
                               distance/speed+speed/accel,
                               2.*np.sqrt(distance/accel))
    result['joint_transition']['rest_to_rest_rate_lower_bound_s'] = float(np.max(individual_time))
    path = fixed_arm_bypass(scene)
    bodies = model.body_spheres(np.broadcast_to(scene['q_start'], (len(path), 6)), path)
    bypass_gap = float(min(np.min(cylinder_gaps(body, actors)) for body in bodies))
    result['fixed_arm_bypass'] = dict(samples=len(path),
        maximum_lateral_offset_m=float(np.max(np.abs(path[:, 1]))),
        cylinder_min_physical_gap_m=bypass_gap,
        waypoints=path[::25].tolist(),
        scope='Geometric route availability only; no closed-loop or wheel-rate feasibility claim')
    result['passed'] = bool(result['start']['fixed_min_gap_m'] > .05
                            and result['start']['arm_min_gap_m'] < -.01
                            and result['retract']['fixed_min_gap_m'] > .05
                            and result['retract']['arm_min_gap_m'] > .05
                            and all(result[k]['model_check']['valid'] for k in ('start', 'retract'))
                            and result['joint_transition_sampled_valid']
                            and result['joint_transition']['initial_base_pose_external_gap_m'] > .05
                            and bypass_gap > .05)
    result['q_start_degrees'] = np.rad2deg(scene['q_start']).tolist()
    result['q_retract_degrees'] = np.rad2deg(scene['q_retract']).tolist()
    result['scope'] = ('Same straight base path and physical sphere geometry; sampled self/ground checks; '
                       'does not prove execution, tracking, native soft/hard admission, or timing feasibility')
    return result
