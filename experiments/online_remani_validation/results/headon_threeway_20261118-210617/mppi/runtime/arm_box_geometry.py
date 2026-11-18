"""One sudden, elevated rectangular obstacle: sensor and audit truth only."""
from copy import deepcopy
import math

import numpy as np

from active_arm_scene import Q_START, Q_RETRACT


def scene_definition():
    return dict(schema='arm_box_scene_v1', scene='arm_box_sudden_ahead',
        name='行驶中机械臂前方一米突然出现长方体',
        time_origin='measured motion anchor', appearance_s=2., persistent=True,
        goal=[3.4, 0.], arena=[-1.5, 4.5, -2., 2.],
        q_start=Q_START.tolist(), q_retract=Q_RETRACT.tolist(),
        q_retract_role='geometric feasibility witness; never an online motion command',
        box=dict(id=1, center=[None, None, 1.175], size=[.30, .40, .25]),
        front_gap_m=1., placement='current arm forward physical surface + 1 m along world x',
        input_contract='Only synthetic measured cloud enters planner/controller',
        scope='native numerical ROS plant; free base and arm; no hardware')


def validate_scene(scene):
    if scene.get('schema') != 'arm_box_scene_v1':
        raise ValueError('Unknown arm-box scene schema')
    for name in ('q_start', 'q_retract'):
        q = np.asarray(scene[name], dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError('Expected six finite joint angles: ' + name)
    arena = np.asarray(scene['arena'], dtype=float)
    if arena.shape != (4,) or not np.isfinite(arena).all() or not (arena[0] < arena[1] and arena[2] < arena[3]):
        raise ValueError('Invalid arena')
    size = np.asarray(scene['box']['size'], dtype=float)
    if size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0):
        raise ValueError('Expected three positive box dimensions')
    center = scene['box']['center']
    if len(center) != 3 or not math.isfinite(float(center[2])):
        raise ValueError('Expected finite box height')
    if not math.isfinite(scene['appearance_s']) or scene['appearance_s'] < 0:
        raise ValueError('Invalid appearance delay')
    if not math.isfinite(scene['front_gap_m']) or scene['front_gap_m'] <= 0:
        raise ValueError('Invalid appearance gap')
    return True


def place_box(model, q, base_pose, scene=None):
    """Fix the box world coordinates using the state at its first sensor frame."""
    scene = scene_definition() if scene is None else scene
    validate_scene(scene)
    arm = model.arm_spheres(np.asarray(q, dtype=float), np.asarray(base_pose, dtype=float))
    front_index = int(np.argmax(arm[:, 0] + arm[:, 3]))
    arm_front = float(arm[front_index, 0] + arm[front_index, 3])
    box = deepcopy(scene['box'])
    box['center'] = [arm_front + float(scene['front_gap_m']) + .5*box['size'][0],
                     float(arm[front_index, 1]), float(box['center'][2])]
    return dict(box=box, base_pose=np.asarray(base_pose, dtype=float).tolist(),
        q=np.asarray(q, dtype=float).tolist(), arm_front_x=arm_front,
        arm_front_sphere=front_index, front_gap_m=float(scene['front_gap_m']))


def box_gaps(spheres_world, box):
    """Exact signed sphere-versus-solid axis-aligned-box physical gaps."""
    spheres = np.asarray(spheres_world, dtype=float)
    if spheres.ndim != 2 or spheres.shape[1] != 4 or not len(spheres):
        raise ValueError('Expected nonempty N x 4 physical spheres')
    if not np.isfinite(spheres).all() or np.any(spheres[:, 3] <= 0):
        raise ValueError('Invalid physical spheres')
    if box is None:
        return np.full(len(spheres), np.inf)
    center = np.asarray(box['center'], dtype=float)
    half = .5*np.asarray(box['size'], dtype=float)
    if center.shape != (3,) or half.shape != (3,) or not np.isfinite(np.r_[center, half]).all() or np.any(half <= 0):
        raise ValueError('Invalid located box')
    delta = np.abs(spheres[:, :3]-center)-half
    distance = np.linalg.norm(np.maximum(delta, 0.), axis=1) + np.minimum(np.max(delta, axis=1), 0.)
    return distance-spheres[:, 3]


def obstacle_points(box, spacing=.035):
    """Uniform samples on the six finite faces, never a vertical cylinder."""
    if not math.isfinite(spacing) or spacing <= 0:
        raise ValueError('Positive finite cloud spacing required')
    if box is None:
        return np.empty((0, 3), dtype=np.float32)
    center = np.asarray(box['center'], dtype=float)
    size = np.asarray(box['size'], dtype=float)
    # Validation shares the analytic auditor's geometric contract.
    box_gaps(np.array([[0., 0., 0., .01]]), box)
    axes = [np.linspace(c-.5*s, c+.5*s, max(2, int(math.ceil(s/spacing))+1))
            for c, s in zip(center, size)]
    faces = []
    for fixed in range(3):
        other = [index for index in range(3) if index != fixed]
        a, b = np.meshgrid(axes[other[0]], axes[other[1]], indexing='ij')
        for edge in (axes[fixed][0], axes[fixed][-1]):
            face = np.empty((a.size, 3))
            face[:, fixed] = edge
            face[:, other[0]] = a.ravel()
            face[:, other[1]] = b.ravel()
            faces.append(face)
    return np.unique(np.vstack(faces), axis=0).astype(np.float32)


def straight_path_precheck(model, scene=None, runtime=None, samples=341, joint_samples=257):
    """A sampled geometric witness; it never steers the simulated robot."""
    scene = scene_definition() if scene is None else deepcopy(scene)
    validate_scene(scene)
    if samples < 2 or joint_samples < 17:
        raise ValueError('Insufficient geometry samples')
    # Offline nominal anchor; runtime verification replaces this with the real
    # first cloud's state and located box, rather than guessing appearance pose.
    runtime = place_box(model, scene['q_start'], [.5, 0., 0.], scene) if runtime is None else runtime
    box = runtime['box']
    start_pose = np.asarray(runtime['base_pose'], dtype=float)
    x = np.linspace(start_pose[0], scene['goal'][0], samples)
    poses = np.column_stack([x, np.full(len(x), start_pose[1]), np.zeros(len(x))])
    fixed_n = len(model.fixed_spheres)
    result = dict(box=deepcopy(box), appearance_base_pose=start_pose.tolist(),
                  appearance_front_gap_m=runtime['front_gap_m'])
    for label in ('start', 'retract'):
        q = np.asarray(scene['q_'+label], dtype=float)
        body = model.body_spheres(np.broadcast_to(q, (len(poses), 6)), poses)
        gaps = np.stack([box_gaps(spheres, box) for spheres in body])
        result[label] = dict(fixed_min_gap_m=float(gaps[:, :fixed_n].min()),
            arm_min_gap_m=float(gaps[:, fixed_n:].min()), model_check=model.check(q))
    start_q, end_q = np.asarray(scene['q_start']), np.asarray(scene['q_retract'])
    transition = (1.-np.linspace(0., 1., joint_samples)[:, None])*start_q + np.linspace(0., 1., joint_samples)[:, None]*end_q
    checks = [model.check(q) for q in transition]
    transition_gap = min(float(np.min(box_gaps(model.body_spheres(q, start_pose), box))) for q in transition)
    result['joint_transition'] = dict(samples=joint_samples, all_valid=all(row['valid'] for row in checks),
        min_self_gap_m=min(row['min_self_gap'] for row in checks),
        min_base_arm_gap_m=min(row['min_base_arm_gap'] for row in checks),
        min_ground_gap_m=min(row['min_ground_gap'] for row in checks),
        appearance_pose_external_gap_m=transition_gap)
    result['passed'] = bool(result['start']['fixed_min_gap_m'] > .05 and result['start']['arm_min_gap_m'] < -.01
        and result['retract']['fixed_min_gap_m'] > .05 and result['retract']['arm_min_gap_m'] > .05
        and result['start']['model_check']['valid'] and result['retract']['model_check']['valid']
        and result['joint_transition']['all_valid'] and transition_gap > .05)
    result['scope'] = 'Sampled same straight base path; arm-only geometric availability, not closed-loop feasibility'
    return result


def register_scene(scene=None):
    """Metadata only, for the existing isolated experiment preparation hook."""
    import dynamic_scenarios as target
    scene = scene_definition() if scene is None else scene
    validate_scene(scene)
    target.CASES[scene['scene']] = ('active_arm', scene['name'], ())
    target.SCENES = tuple(dict.fromkeys((*target.SCENES, scene['scene'])))
    return deepcopy(scene)
