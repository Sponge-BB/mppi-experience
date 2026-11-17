#!/usr/bin/env python3
"""Native Piper geometry and bounded joint integration for isolated SIL.

The authoritative sphere geometry reproduces MMConfig's six Standard-DH
transforms and twenty arm samples.  The URDF FK is exposed separately: the
workspace URDF and native DH use different link/tool conventions and are NOT
asserted equivalent.  No ROS, hardware control, mesh loading or file writes.

Self checking is a disclosed sphere-model approximation, not mesh certification.
Same/adjacent DH groups, physical upper-arm/forearm adjacency and two permanent
wrist-sample overlaps are excluded explicitly. Base/arm checking retains the native Piper policy:
links 1 and 2 attach to the mount and are excluded from fixed-structure checks.
"""

from pathlib import Path
import math
import xml.etree.ElementTree as ET

import numpy as np


ROOT = next((parent for parent in Path(__file__).resolve().parents
             if (parent/'src/my_description/robot/mm.urdf').is_file()),
            Path(__file__).resolve().parents[2])
DEFAULT_PARAMETERS = ROOT / ('experiments/online_remani_validation/results/'
                            'short_prefix_rejoin_20261101_01/midcross_B/parameters.yaml')
DEFAULT_URDF = ROOT / 'src/my_description/robot/mm.urdf'
Q_NOMINAL = np.deg2rad([0., 29.85, -60.098, -.002, 54.276, .003])
DH_ALPHA = np.array([-math.pi/2, 0., math.pi/2, -math.pi/2, math.pi/2, 0.])
DH_A = np.array([0., .28503, -.021984, 0., 0., 0.])
DH_D = np.array([.123, 0., 0., .25075, 0., .211])
DH_OFFSET = np.deg2rad([0., -172.22, -102.78, 0., 0., 0.])
LINK_POINTS = (
    ((0., 0., 0.), (0., .06, 0.), (0., .123, 0.)),
    ((0., 0., 0.), (-.07, 0., 0.), (-.14, 0., 0.), (-.21, 0., 0.), (-.285, 0., 0.)),
    ((0., 0., 0.), (.022, 0., 0.)),
    ((0., 0., 0.), (0., .06, 0.), (0., .12, 0.), (0., .18, 0.), (0., .25, 0.)),
    ((0., 0., 0.),),
    ((0., 0., 0.), (0., 0., -.07), (0., 0., -.14), (0., 0., -.211)),
)
# Self-collision policy only: DH groups 1 and 3 sample URDF link2 and link3.
# The intervening DH group 2 is the elbow connector, not another long link.
# Original piper_camera.srdf marks link2/link3 Adjacent; the frozen copy is
# model/piper_camera.srdf. This does not change FK, radii, or external obstacles.
SELF_COLLISION_PHYSICAL_ADJACENT_DH_GROUPS = frozenset({(1, 3)})


def _vector(value, size, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(name + ' must be a finite vector of length ' + str(size))
    return result


def _six(value, name):
    result = np.broadcast_to(np.asarray(value, dtype=float), (6,)).copy()
    if not np.isfinite(result).all():
        raise ValueError(name + ' must be finite')
    return result


def _rotation_rpy(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def _origin(element):
    transform = np.eye(4)
    if element is not None:
        transform[:3, 3] = _vector([float(v) for v in element.get('xyz', '0 0 0').split()], 3, 'xyz')
        transform[:3, :3] = _rotation_rpy(_vector(
            [float(v) for v in element.get('rpy', '0 0 0').split()], 3, 'rpy'))
    return transform


class PiperArmModel:
    """Twenty native arm spheres; fixed structure uses physical sphere radii."""

    def __init__(self, fixed_spheres, mount=(.005, 0., .68, 0., 0., 0.),
                 lower=(-2.618, -.05, -2.697, -1.832, -1.25, -3.14),
                 upper=(2.618, 3.14, .05, 1.832, 1.25, 3.14),
                 velocity_limits=.45, acceleration_limits=.9,
                 arm_radius=.034, ground_clearance=.03, urdf_path=DEFAULT_URDF):
        body = np.asarray(fixed_spheres, dtype=float)
        if body.ndim != 2 or body.shape[1] != 4 or not len(body) or not np.isfinite(body).all() or np.any(body[:, 3] <= 0.):
            raise ValueError('fixed_spheres must be nonempty finite N x 4 xyz/r')
        self.fixed_spheres = body.copy()
        self.lower = _six(lower, 'lower'); self.upper = _six(upper, 'upper')
        self.velocity_limits = _six(velocity_limits, 'velocity_limits')
        self.acceleration_limits = _six(acceleration_limits, 'acceleration_limits')
        if np.any(self.lower >= self.upper) or np.any(self.velocity_limits <= 0.) or np.any(self.acceleration_limits <= 0.):
            raise ValueError('joint bounds and rate limits must have positive ranges')
        self.arm_radius = float(arm_radius); self.ground_clearance = float(ground_clearance)
        if not math.isfinite(self.arm_radius) or self.arm_radius <= 0. or not math.isfinite(self.ground_clearance) or self.ground_clearance < 0.:
            raise ValueError('invalid sphere radius or ground clearance')
        xyz_ypr = _vector(mount, 6, 'mount')
        self.mount = np.eye(4)
        self.mount[:3, 3] = xyz_ypr[:3]
        self.mount[:3, :3] = _rotation_rpy(xyz_ypr[[5, 4, 3]])
        self.q_nominal = Q_NOMINAL.copy()
        self.link_ids = np.concatenate([np.full(len(points), i, dtype=int)
                                        for i, points in enumerate(LINK_POINTS)])
        self.local_points = np.asarray([p for points in LINK_POINTS for p in points], dtype=float)
        self._point_offsets = np.cumsum([0] + [len(p) for p in LINK_POINTS])
        self._self_pairs, self.excluded_self_pairs = self._collision_pairs()
        self.base_arm_checked_indices = np.flatnonzero(self.link_ids >= 2)
        self.excluded_base_arm_links = (0, 1)  # Native mm_config.cpp:1008.
        self.urdf_path = Path(urdf_path)
        self._urdf_root = ET.parse(self.urdf_path).getroot() if self.urdf_path.is_file() else None

    @classmethod
    def from_defaults(cls):
        return cls.from_parameters(DEFAULT_PARAMETERS)

    @classmethod
    def from_parameters(cls, path, urdf_path=DEFAULT_URDF):
        import yaml
        data = yaml.safe_load(Path(path).read_text())
        prefix = '/remani_planner_node/'
        get = lambda key, default=None: data.get(prefix+key, default)
        root = ET.parse(urdf_path).getroot()
        joints = {j.get('name'): j for j in root.findall('joint')}
        limits = [joints['joint'+str(i)].find('limit') for i in range(1, 7)]
        lower = [float(p.get('lower')) for p in limits]
        upper = [float(p.get('upper')) for p in limits]
        velocities = np.minimum(float(get('mm/manipulator_max_vel', .45)),
                                [float(p.get('velocity')) for p in limits])
        return cls(get('mm/fixed_collision_spheres'), get('mm/base_mani_fixed_joint_xyz_ypr'),
                   lower, upper, velocities, get('mm/manipulator_max_acc', .9),
                   get('mm/manipulator_thickness', .034), get('optimization/ground_safe_dis', .03), urdf_path)

    def _collision_pairs(self):
        # Keep the existing zero-length wrist sample exceptions. The elbow's
        # physical adjacency is handled separately from DH-frame numbering.
        attachment = {(10, 19), (11, 19)}
        checked, excluded = [], []
        for i in range(20):
            for j in range(i+1, 20):
                groups = (int(self.link_ids[i]), int(self.link_ids[j]))
                difference = abs(groups[0]-groups[1])
                reason = ('same_link' if difference == 0 else
                          'adjacent_links' if difference == 1 else
                          'physical_adjacent_urdf_link2_link3'
                          if groups in SELF_COLLISION_PHYSICAL_ADJACENT_DH_GROUPS else
                          'permanent_joint_sample_overlap' if (i, j) in attachment else None)
                if reason is None:
                    checked.append((i, j))
                else:
                    excluded.append({'sphere_i': i, 'sphere_j': j,
                                     'link_i': int(self.link_ids[i]), 'link_j': int(self.link_ids[j]), 'reason': reason})
        return np.asarray(checked, dtype=int), tuple(excluded)

    def _q_batch(self, q):
        values = np.asarray(q, dtype=float)
        single = values.ndim == 1
        if single: values = values[None, :]
        if values.ndim != 2 or values.shape[1] != 6 or not len(values) or not np.isfinite(values).all():
            raise ValueError('q must be finite length 6 or N x 6')
        return values, single

    def native_transforms(self, q):
        """Base-ground frame T_DH1..T_DH6, shape (6,4,4) or (N,6,4,4)."""
        values, single = self._q_batch(q)
        count = len(values); current = np.broadcast_to(self.mount, (count, 4, 4)).copy()
        frames = np.empty((count, 6, 4, 4))
        for i in range(6):
            theta = values[:, i]+DH_OFFSET[i]; c = np.cos(theta); s = np.sin(theta)
            ca, sa = math.cos(DH_ALPHA[i]), math.sin(DH_ALPHA[i])
            transform = np.zeros((count, 4, 4))
            transform[:, 0, 0] = c; transform[:, 0, 1] = -s*ca; transform[:, 0, 2] = s*sa; transform[:, 0, 3] = DH_A[i]*c
            transform[:, 1, 0] = s; transform[:, 1, 1] = c*ca; transform[:, 1, 2] = -c*sa; transform[:, 1, 3] = DH_A[i]*s
            transform[:, 2, 1] = sa; transform[:, 2, 2] = ca; transform[:, 2, 3] = DH_D[i]; transform[:, 3, 3] = 1.
            current = current @ transform; frames[:, i] = current
        return frames[0] if single else frames

    @staticmethod
    def _base_transform(spheres, base_pose):
        if base_pose is None: return spheres
        poses = np.asarray(base_pose, dtype=float)
        poses = np.broadcast_to(poses, (len(spheres), 3))
        if not np.isfinite(poses).all(): raise ValueError('base_pose must be finite x,y,yaw')
        result = spheres.copy(); c = np.cos(poses[:, 2])[:, None]; s = np.sin(poses[:, 2])[:, None]
        x, y = spheres[:, :, 0], spheres[:, :, 1]
        result[:, :, 0] = poses[:, 0, None]+c*x-s*y
        result[:, :, 1] = poses[:, 1, None]+s*x+c*y
        return result

    def arm_spheres(self, q, base_pose=None):
        """Physical arm spheres; supports batching, not joint-limit projection."""
        values, single = self._q_batch(q)
        frames = self.native_transforms(values)[:, self.link_ids]
        centers = np.einsum('nsij,sj->nsi', frames[:, :, :3, :3], self.local_points)+frames[:, :, :3, 3]
        spheres = np.concatenate([centers, np.full((len(values), 20, 1), self.arm_radius)], axis=2)
        spheres = self._base_transform(spheres, base_pose)
        return spheres[0] if single else spheres

    def body_spheres(self, q, base_pose=None):
        """Fixed structure + variable arm, 67 physical spheres with defaults."""
        values, single = self._q_batch(q)
        fixed = np.broadcast_to(self.fixed_spheres, (len(values), len(self.fixed_spheres), 4))
        spheres = np.concatenate([fixed, self.arm_spheres(values)], axis=1)
        spheres = self._base_transform(spheres, base_pose)
        return spheres[0] if single else spheres

    def body_spheres_batch(self, q, base_pose=None):
        """Explicit N x 6 -> N x 67 x 4 API for sampled whole-body rollout."""
        values, single = self._q_batch(q)
        if single: raise ValueError('body_spheres_batch expects N x 6')
        return self.body_spheres(values, base_pose)

    def configuration_gaps_batch(self, q):
        """Minimum self/base-arm/ground physical gap, metres, for each q.

        No external obstacles or joint-angle bounds are included; angle/rate
        limits must be checked separately rather than mixing units with metres.
        Ground gap includes the configured ground clearance.
        """
        values, single = self._q_batch(q)
        if single: raise ValueError('configuration_gaps_batch expects N x 6')
        spheres = self.arm_spheres(values)
        i, j = self._self_pairs.T
        self_gap = np.min(np.linalg.norm(spheres[:, i, :3]-spheres[:, j, :3], axis=2)
                          -spheres[:, i, 3]-spheres[:, j, 3], axis=1)
        arm = spheres[:, self.base_arm_checked_indices]
        base_gap = np.min(np.linalg.norm(arm[:, :, None, :3]-self.fixed_spheres[None, None, :, :3], axis=3)
                          -arm[:, :, None, 3]-self.fixed_spheres[None, None, :, 3], axis=(1, 2))
        ground_gap = np.min(spheres[:, :, 2]-spheres[:, :, 3]-self.ground_clearance, axis=1)
        return np.minimum(np.minimum(self_gap, base_gap), ground_gap)

    def joint_gaps(self, q, dq=None, ddq=None):
        values = _vector(q, 6, 'q')
        result = {'position': float(np.min(np.minimum(values-self.lower, self.upper-values)))}
        if dq is not None: result['velocity'] = float(np.min(self.velocity_limits-np.abs(_vector(dq, 6, 'dq'))))
        if ddq is not None: result['acceleration'] = float(np.min(self.acceleration_limits-np.abs(_vector(ddq, 6, 'ddq'))))
        return result

    def check(self, q, dq=None, ddq=None, collision_margin=0.):
        """Local joint/self/base/ground check. Does NOT check external obstacles.

        Ground uses sphere lower surfaces, stronger than native centre-only arm
        ground check. Fixed structure touching its own wheel/ground is excluded.
        Link and sphere IDs are zero-based in all diagnostic masks.
        """
        margin = float(collision_margin)
        if not math.isfinite(margin) or margin < 0.: raise ValueError('invalid collision margin')
        spheres = self.arm_spheres(q)
        if spheres.ndim != 2: raise ValueError('check expects one q; use check_batch for batches')
        gaps = self.joint_gaps(q, dq, ddq)
        i, j = self._self_pairs.T
        self_gaps = np.linalg.norm(spheres[i, :3]-spheres[j, :3], axis=1)-spheres[i, 3]-spheres[j, 3]
        pair_index = int(np.argmin(self_gaps)); pair = self._self_pairs[pair_index]
        arm = spheres[self.base_arm_checked_indices]
        base_gaps = np.linalg.norm(arm[:, None, :3]-self.fixed_spheres[None, :, :3], axis=2)-arm[:, None, 3]-self.fixed_spheres[None, :, 3]
        base_index = np.unravel_index(np.argmin(base_gaps), base_gaps.shape)
        ground = spheres[:, 2]-spheres[:, 3]-self.ground_clearance
        violations = [key+'_limit' for key, value in gaps.items() if value < -1e-10]
        if self_gaps[pair_index] < margin-1e-10: violations.append('arm_self_collision')
        if base_gaps[base_index] < margin-1e-10: violations.append('base_arm_collision')
        if np.min(ground) < margin-1e-10: violations.append('ground_collision')
        return {'valid': not violations, 'violations': violations, 'joint_gaps': gaps,
                'min_self_gap': float(self_gaps[pair_index]), 'self_pair': pair.tolist(),
                'min_base_arm_gap': float(base_gaps[base_index]),
                'base_arm_pair': [int(self.base_arm_checked_indices[base_index[0]]), int(base_index[1])],
                'min_ground_gap': float(np.min(ground)),
                'scope': 'native sampled spheres, disclosed pair exclusions; no external obstacle check'}

    def check_batch(self, q, collision_margin=0.):
        values, _ = self._q_batch(q)
        return [self.check(value, collision_margin=collision_margin) for value in values]

    def collision_policy(self):
        return {'sphere_link_ids': self.link_ids.tolist(), 'checked_self_pairs': self._self_pairs.tolist(),
                'excluded_self_pairs': list(self.excluded_self_pairs),
                'excluded_base_arm_links': list(self.excluded_base_arm_links),
                'base_arm_exclusion_reason': 'native Piper mounting adjacency rule; not full mesh certification',
                'ground_rule': 'arm sphere bottom >= ground_clearance'}

    def integrate(self, q, dq, velocity_command, dt, check_geometry=True):
        """One constant-acceleration step with rate/position limits.

        Position limits are enforced by intersecting admissible next velocities;
        an impossible state is rejected, never repaired by teleporting q. Returns
        q/dq/ddq plus valid/reason. No collision shortcut is implied by clipping.
        """
        position = _vector(q, 6, 'q'); velocity = _vector(dq, 6, 'dq')
        command = _vector(velocity_command, 6, 'velocity_command'); dt = float(dt)
        if not math.isfinite(dt) or dt <= 0.: raise ValueError('dt must be finite and positive')
        if any(value < -1e-10 for value in self.joint_gaps(position, velocity).values()):
            return {'q': position.copy(), 'dq': velocity.copy(), 'ddq': np.zeros(6), 'valid': False, 'reason': 'invalid_initial_joint_state'}
        initial_stop = position+velocity*np.abs(velocity)/(2*self.acceleration_limits)
        if np.any(initial_stop < self.lower-1e-10) or np.any(initial_stop > self.upper+1e-10):
            return {'q': position.copy(), 'dq': velocity.copy(), 'ddq': np.zeros(6), 'valid': False, 'reason': 'initial_joint_stop_limit'}
        low = np.maximum(-self.velocity_limits, velocity-self.acceleration_limits*dt)
        high = np.minimum(self.velocity_limits, velocity+self.acceleration_limits*dt)
        low = np.maximum(low, 2*(self.lower-position)/dt-velocity)
        high = np.minimum(high, 2*(self.upper-position)/dt-velocity)
        if np.any(low > high+1e-12):
            return {'q': position.copy(), 'dq': velocity.copy(), 'ddq': np.zeros(6), 'valid': False, 'reason': 'no_bounded_joint_step'}
        next_velocity = np.clip(command, low, high)
        next_position = position+.5*dt*(velocity+next_velocity)
        acceleration = (next_velocity-velocity)/dt
        # A candidate must retain a joint-limit stopping option. This does not
        # certify stopping against external obstacles; the runner owns that.
        stopping_position = next_position+next_velocity*np.abs(next_velocity)/(2*self.acceleration_limits)
        stop_ok = np.all(stopping_position >= self.lower-1e-10) and np.all(stopping_position <= self.upper+1e-10)
        verdict = self.check(next_position, next_velocity, acceleration) if check_geometry else {'valid': True, 'violations': []}
        valid = bool(stop_ok and verdict['valid'])
        return {'q': next_position, 'dq': next_velocity, 'ddq': acceleration,
                'valid': valid, 'reason': ('ok' if valid else 'joint_stop_limit' if not stop_ok else ','.join(verdict['violations'])),
                'geometry': verdict, 'clipped': bool(np.any(np.abs(next_velocity-command) > 1e-12))}

    def urdf_link_transforms(self, q):
        """Independent URDF link poses relative to URDF base_link (zero wheels).

        This is for audits; native arm_spheres never silently switches to it.
        Missing URDF or disconnected/cyclic trees are explicit failures.
        """
        values = _vector(q, 6, 'q')
        if self._urdf_root is None: raise FileNotFoundError(str(self.urdf_path))
        joints = list(self._urdf_root.findall('joint'))
        transforms = {'base_link': np.eye(4)}
        pending = joints[:]
        while pending:
            progress = False
            for joint in pending[:]:
                parent = joint.find('parent').get('link'); child = joint.find('child').get('link')
                if parent not in transforms: continue
                if child in transforms: raise ValueError('repeated URDF child/cycle')
                local = _origin(joint.find('origin')); name = joint.get('name')
                if name in ['joint'+str(i) for i in range(1, 7)]:
                    angle = values[int(name[5:])-1]
                    axis = _vector([float(x) for x in joint.find('axis').get('xyz').split()], 3, 'joint axis')
                    norm = np.linalg.norm(axis)
                    if norm < 1e-12: raise ValueError('zero joint axis')
                    axis /= norm; a, b, c = axis
                    skew = np.array([[0., -c, b], [c, 0., -a], [-b, a, 0.]])
                    rotation = np.eye(4); rotation[:3, :3] = np.eye(3)+math.sin(angle)*skew+(1-math.cos(angle))*(skew @ skew)
                    local = local @ rotation
                transforms[child] = transforms[parent] @ local
                pending.remove(joint); progress = True
            if not progress: raise ValueError('URDF disconnected from base_link or cyclic')
        return transforms
