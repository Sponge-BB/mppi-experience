#!/usr/bin/env python3
"""Native Piper geometry and bounded joint integration for whole-body MPPI.

The authoritative sphere geometry reproduces MMConfig's six Standard-DH
transforms and twenty arm samples.  The URDF FK is exposed separately: the
workspace URDF and native DH use different link/tool conventions and are NOT
asserted equivalent.  No ROS, hardware control, mesh loading or file writes.

Self checking remains a sphere-model approximation. Two coarse shoulder/wrist
rejections may be cleared by separating planes between original mesh hulls;
this is not full-robot mesh certification or physical calibration.
Same/adjacent DH groups, physical upper-arm/forearm adjacency and two permanent
wrist-sample overlaps are excluded explicitly. Base/arm checking retains the native Piper policy:
links 1 and 2 attach to the mount and are excluded from fixed-structure checks.
"""

from pathlib import Path
import json
import math
import xml.etree.ElementTree as ET

import numpy as np


PACKAGE = Path(__file__).resolve().parents[2]
DEFAULT_URDF = PACKAGE.parent / 'my_description/robot/mm.urdf'
DEFAULT_PROXY = PACKAGE / 'config/mppi_shoulder_wrist_proxy.json'
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


class ShoulderWristProxy:
    """Two conservative refinements for original shoulder/upper-arm meshes.

    The asset is built offline after applying the original collision origins
    and mesh scene transforms. Runtime uses no mesh library. A positive gap
    proves separation of (arm_base, link1), or both complete-triangle upper-arm
    hulls, from (link5, link6). A complete-triangle link5 decomposition avoids
    filling its concave waist when the whole-link hull cannot separate.
    A nonpositive gap never clears coarse rejection.
    """

    numerical_tolerance = .0001  # Total 0.1 mm numerical padding, not calibration.
    # Additional link5-frame support directions, from the 2026-11-06 complete-
    # hull audit and 2026-11-07 support-plane check. Directions select a test,
    # not a safe pose: every test projects ALL original hull vertices at q.
    # No positive certificate is reused across poses or collision pairs.
    shoulder_support_axes = (
        (-0.9954197235233002, 0.042160942448799234, -0.0858022665937532),
        (0.9944288043049222, -0.04444383511520801, 0.09558294141181611),
    )

    def __init__(self, path, urdf_root):
        data = json.loads(Path(path).read_text())
        if (data.get('schema') != 'offline_shoulder_wrist_convex_support_candidate_v1'
                or data.get('units') != 'metres_and_radians' or urdf_root is None):
            raise ValueError('unsupported shoulder/wrist proxy or missing URDF')
        self.vertices = {}
        for name in ('arm_base', 'link1', 'link5', 'link6'):
            item = data['proxies'][name]
            vertices = np.asarray(item['vertices'], dtype=float)
            if (item.get('coordinate_frame') != name
                    or item.get('collision_origin_and_scene_transforms_applied') is not True
                    or vertices.ndim != 2 or vertices.shape[1] != 3
                    or len(vertices) < 4 or len(vertices) > 4096
                    or len(vertices) != item.get('hull_vertex_count')
                    or not np.isfinite(vertices).all()
                    or np.linalg.matrix_rank(vertices-vertices[0]) != 3):
                raise ValueError('invalid shoulder/wrist hull: '+name)
            vertices.setflags(write=False)
            self.vertices[name] = vertices
        certificate = data['certificate']
        if certificate.get('direction_frame') != 'link5':
            raise ValueError('shoulder/wrist axis must use link5 frame')
        axis = _vector(certificate['baseline']['axis_local_link5'], 3, 'proxy axis')
        if abs(np.linalg.norm(axis)-1.) > 1e-8:
            raise ValueError('shoulder/wrist axis must be unit length')
        self.axis = axis / np.linalg.norm(axis)
        self.axis.setflags(write=False)
        chain = data['joint_chain_arm_base_to_link6']
        if len(chain) != 6:
            raise ValueError('incomplete shoulder/wrist joint chain')
        joints = {joint.get('name'): joint for joint in urdf_root.findall('joint')}
        self.origins = []
        for i, item in enumerate(chain, 1):
            name, parent, child = 'joint'+str(i), ('arm_base' if i == 1 else 'link'+str(i-1)), 'link'+str(i)
            joint = joints[name]
            origin = _origin(ET.Element('origin', item['origin']))
            axis = _vector(item['axis'], 3, name+' axis')
            urdf_axis = _vector([float(v) for v in joint.find('axis').get('xyz').split()], 3, name+' URDF axis')
            if (item['name'] != name or item['parent'] != parent or item['child'] != child
                    or joint.get('type') != 'revolute'
                    or joint.find('parent').get('link') != parent
                    or joint.find('child').get('link') != child
                    or not np.array_equal(axis, [0., 0., 1.])
                    or not np.array_equal(axis, urdf_axis)
                    or not np.allclose(origin, _origin(joint.find('origin')), rtol=0., atol=1e-12)):
                raise ValueError('shoulder/wrist proxy differs from URDF: '+name)
            self.origins.append(origin)
        self.origins = np.asarray(self.origins)
        self.origins.setflags(write=False)
        projection = self.vertices['link5'] @ self.axis
        self.link5_interval = (float(projection.min()), float(projection.max()))
        self.shoulder_axes = []
        for values in self.shoulder_support_axes:
            axis = _vector(values, 3, 'additional shoulder axis')
            axis = axis / np.linalg.norm(axis)
            projection = self.vertices['link5'] @ axis
            axis.setflags(write=False)
            self.shoulder_axes.append((axis, (float(projection.min()), float(projection.max()))))
        upper = data['upper_arm_certificate']
        parts = upper['parts']
        split = upper['split']
        if (split.get('frame') != 'link2' or split.get('axis') != 2 or split.get('value') != 0
                or split.get('crossing_triangles') != 'complete triangle included in both pieces'
                or upper.get('collision_origin_and_scene_transforms_applied') is not True
                or set(parts) != {'low', 'high'}
                or upper.get('original_triangle_count', 0) <= 0
                or upper.get('covered_triangle_count') != upper['original_triangle_count']
                or upper.get('duplicated_crossing_triangles', -1) < 0
                or sum(part['triangle_count'] for part in parts.values()) !=
                   upper['original_triangle_count']+upper['duplicated_crossing_triangles']):
            raise ValueError('incomplete original upper-arm triangle coverage')
        self.upper_parts = {}
        for name in ('low', 'high'):
            item = parts[name]
            vertices = np.asarray(item['vertices'], dtype=float)
            axis = _vector(item['axis_local_link5'], 3, 'upper-arm proxy axis')
            if (item.get('coordinate_frame') != 'link2'
                    or vertices.ndim != 2 or vertices.shape[1] != 3
                    or len(vertices) < 4 or len(vertices) > 4096
                    or len(vertices) != item.get('hull_vertex_count')
                    or not np.isfinite(vertices).all()
                    or np.linalg.matrix_rank(vertices-vertices[0]) != 3
                    or abs(np.linalg.norm(axis)-1.) > 1e-8
                    or item['triangle_count'] <= 0):
                raise ValueError('invalid upper-arm hull: '+name)
            axis = axis / np.linalg.norm(axis)
            projection = self.vertices['link5'] @ axis
            vertices.setflags(write=False); axis.setflags(write=False)
            self.upper_parts[name] = (vertices, axis, (float(projection.min()), float(projection.max())))

        self.wrist_parts = []
        wrist = data.get('wrist_certificate')
        if wrist is not None:
            parts, split = wrist['parts'], wrist['split']
            if (split != {'frame': 'link5', 'axis': 1, 'value': .03,
                          'crossing_triangles': 'complete crossing triangle included in both pieces'}
                    or wrist.get('source_mesh_sha256') != data['proxies']['link5']['source_mesh_sha256']
                    or wrist.get('collision_origin_and_scene_transforms_applied') is not True
                    or set(parts) != {'low', 'high'}
                    or wrist.get('original_triangle_count', 0) <= 0
                    or wrist.get('covered_triangle_count') != wrist['original_triangle_count']
                    or wrist.get('duplicated_crossing_triangles', -1) < 0
                    or sum(p['triangle_count'] for p in parts.values()) !=
                       wrist['original_triangle_count']+wrist['duplicated_crossing_triangles']):
                raise ValueError('incomplete original wrist triangle coverage')
            for name in ('low', 'high'):
                item = parts[name]
                vertices = np.asarray(item['vertices'], dtype=float)
                axis = _vector(item['axis'], 3, 'wrist support axis')
                if (item.get('coordinate_frame') != 'link5'
                        or item.get('direction_frame') not in ('link1', 'link5')
                        or vertices.ndim != 2 or vertices.shape[1] != 3
                        or not 4 <= len(vertices) <= 4096
                        or len(vertices) != item.get('hull_vertex_count')
                        or not np.isfinite(vertices).all()
                        or np.linalg.matrix_rank(vertices-vertices[0]) != 3
                        or abs(np.linalg.norm(axis)-1.) > 1e-8
                        or item['triangle_count'] <= 0):
                    raise ValueError('invalid wrist hull: '+name)
                vertices.setflags(write=False); axis.setflags(write=False)
                self.wrist_parts.append((vertices, axis, int(item['direction_frame'][-1])))

    def gaps(self, q):
        """Shoulder (0,18) support gap, including numerical padding."""
        return self.pair_gaps(q)[:, 0]

    def pair_gaps(self, q, refine_shoulder=None):
        """N x 6 -> N x 2 padded gaps, ordered (0,18), (7,18)."""
        q = np.asarray(q, dtype=float)
        if q.ndim != 2 or q.shape[1] != 6 or not len(q) or not np.isfinite(q).all():
            raise ValueError('proxy q must be finite nonempty N x 6')
        count = len(q)
        refine_shoulder = (np.ones(count, dtype=bool) if refine_shoulder is None
                           else np.asarray(refine_shoulder, dtype=bool))
        if refine_shoulder.shape != (count,):
            raise ValueError('shoulder refinement mask must match q')
        rotation = np.broadcast_to(np.eye(3), (count, 3, 3)).copy()
        position = np.zeros((count, 3))
        frames = {}
        for i, origin in enumerate(self.origins):
            position = position + rotation @ origin[:3, 3]
            rotation = rotation @ origin[:3, :3]
            c, s = np.cos(q[:, i]), np.sin(q[:, i])
            turn = np.zeros((count, 3, 3))
            turn[:, 0, 0] = c; turn[:, 0, 1] = -s
            turn[:, 1, 0] = s; turn[:, 1, 1] = c; turn[:, 2, 2] = 1.
            rotation = rotation @ turn
            if i in (0, 1, 4, 5):
                frames[i+1] = (rotation, position)
        r5, t5 = frames[5]

        def interval(vertices, direction, frame=None, rows=None):
            origin = t5 if rows is None else t5[rows]
            if frame is None:
                local_axis, offset = direction, -np.sum(origin*direction, axis=1)
            else:
                r, t = frame
                if rows is not None:
                    r, t = r[rows], t[rows]
                local_axis = np.einsum('nji,nj->ni', r, direction)
                offset = np.sum((t-origin)*direction, axis=1)
            projection = local_axis @ vertices.T
            return projection.min(axis=1)+offset, projection.max(axis=1)+offset

        def against_wrist(low_a, high_a, direction, link5_interval, rows=None):
            low_tool, high_tool = interval(self.vertices['link6'], direction, frames[6], rows)
            low_b = np.minimum(link5_interval[0], low_tool)
            high_b = np.maximum(link5_interval[1], high_tool)
            return np.maximum(low_a-high_b, low_b-high_a)-self.numerical_tolerance

        direction = r5 @ self.axis
        low_base, high_base = interval(self.vertices['arm_base'], direction)
        low_shoulder, high_shoulder = interval(self.vertices['link1'], direction, frames[1])
        low_a, high_a = np.minimum(low_base, low_shoulder), np.maximum(high_base, high_shoulder)
        shoulder = against_wrist(low_a, high_a, direction, self.link5_interval)
        for axis, link5_interval in self.shoulder_axes:
            rows = np.flatnonzero(refine_shoulder & (shoulder <= 0.))
            if not len(rows):
                break
            direction = r5[rows] @ axis
            low_base, high_base = interval(self.vertices['arm_base'], direction, rows=rows)
            low_link, high_link = interval(self.vertices['link1'], direction, frames[1], rows)
            gap = against_wrist(np.minimum(low_base, low_link), np.maximum(high_base, high_link),
                                direction, link5_interval, rows)
            shoulder[rows] = np.maximum(shoulder[rows], gap)
        rows = np.flatnonzero(refine_shoulder & (shoulder <= 0.))
        if len(rows) and self.wrist_parts:
            # Both complete triangle pieces, each together with the full tool,
            # must separate from arm_base AND link1 at the actual joint angles.
            # Directions belong to mesh/link frames, never to a recorded pose.
            fine = np.full(len(rows), np.inf)
            for vertices, axis, axis_frame in self.wrist_parts:
                direction = frames[axis_frame][0][rows] @ axis
                low_base, high_base = interval(self.vertices['arm_base'], direction, rows=rows)
                low_link, high_link = interval(self.vertices['link1'], direction, frames[1], rows)
                piece = interval(vertices, direction, frames[5], rows)
                gap = against_wrist(np.minimum(low_base, low_link), np.maximum(high_base, high_link),
                                    direction, piece, rows)
                fine = np.minimum(fine, gap)
            shoulder[rows] = np.maximum(shoulder[rows], fine)
        upper = []
        for vertices, axis, link5_interval in self.upper_parts.values():
            direction = r5 @ axis
            low_a, high_a = interval(vertices, direction, frames[2])
            upper.append(against_wrist(low_a, high_a, direction, link5_interval))
        # Each part encloses complete assigned triangles, including all crossing
        # triangles in both parts. Both parts must be separated at this q.
        return np.column_stack((shoulder, np.minimum(upper[0], upper[1])))


class PiperArmModel:
    """Twenty native arm spheres; fixed structure uses physical sphere radii."""

    def __init__(self, fixed_spheres, mount=(.005, 0., .68, 0., 0., 0.),
                 lower=(-2.618, -.05, -2.697, -1.832, -1.25, -3.14),
                 upper=(2.618, 3.14, .05, 1.832, 1.25, 3.14),
                 velocity_limits=.45, acceleration_limits=.9,
                 arm_radius=.034, ground_clearance=.03, urdf_path=DEFAULT_URDF,
                 shoulder_wrist_proxy=None):
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
        self.shoulder_wrist_proxy = (ShoulderWristProxy(shoulder_wrist_proxy, self._urdf_root)
                                     if shoulder_wrist_proxy is not None else None)
        self._shoulder_wrist_pairs = np.array([
            int(np.flatnonzero(np.all(self._self_pairs == pair, axis=1))[0])
            for pair in ((0, 18), (7, 18))])

    @classmethod
    def from_parameters(cls, path, urdf_path=DEFAULT_URDF, proxy_path=DEFAULT_PROXY):
        """Read current planner parameters (nested mapping or legacy ROS dump)."""
        import yaml
        data = path if isinstance(path, dict) else yaml.safe_load(Path(path).read_text())
        prefix = '/remani_planner_node/'
        def get(key, default=None):
            if prefix+key in data:
                return data[prefix+key]
            value = data
            for part in key.split('/'):
                if not isinstance(value, dict) or part not in value:
                    return default
                value = value[part]
            return value
        root = ET.parse(urdf_path).getroot()
        joints = {j.get('name'): j for j in root.findall('joint')}
        limits = [joints['joint'+str(i)].find('limit') for i in range(1, 7)]
        lower = [float(p.get('lower')) for p in limits]
        upper = [float(p.get('upper')) for p in limits]
        velocities = np.minimum(float(get('mm/manipulator_max_vel', .45)),
                                [float(p.get('velocity')) for p in limits])
        return cls(get('mm/fixed_collision_spheres'), get('mm/base_mani_fixed_joint_xyz_ypr'),
                   lower, upper, velocities, get('mm/manipulator_max_acc', .9),
                   get('mm/manipulator_thickness', .034), get('optimization/ground_safe_dis', .03), urdf_path,
                   proxy_path)

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
        self_gap = np.min(self.self_gaps(values, spheres), axis=1)
        arm = spheres[:, self.base_arm_checked_indices]
        base_gap = np.min(np.linalg.norm(arm[:, :, None, :3]-self.fixed_spheres[None, None, :, :3], axis=3)
                          -arm[:, :, None, 3]-self.fixed_spheres[None, None, :, 3], axis=(1, 2))
        ground_gap = np.min(spheres[:, :, 2]-spheres[:, :, 3]-self.ground_clearance, axis=1)
        return np.minimum(np.minimum(self_gap, base_gap), ground_gap)

    def self_gaps(self, q, arm_spheres=None):
        """Shared per-pair gaps for scalar checks, MPPI and final validation.

        Only negative coarse (0,18)/(7,18) gaps are eligible. Positive
        convex separation clears that rejection; every other pair is unchanged.
        Supplied spheres must correspond to q, optionally with a rigid base pose.
        """
        values, single = self._q_batch(q)
        spheres = self.arm_spheres(values) if arm_spheres is None else np.asarray(arm_spheres, dtype=float)
        if single and spheres.ndim == 2:
            spheres = spheres[None]
        if spheres.shape != (len(values), 20, 4) or not np.isfinite(spheres).all():
            raise ValueError('arm_spheres must match q with twenty finite xyz/r spheres')
        i, j = self._self_pairs.T
        gaps = np.linalg.norm(spheres[:, i, :3]-spheres[:, j, :3], axis=2)-spheres[:, i, 3]-spheres[:, j, 3]
        if self.shoulder_wrist_proxy is not None:
            pairs = self._shoulder_wrist_pairs
            rows = np.flatnonzero(np.any(gaps[:, pairs] < 0., axis=1))
            if len(rows):
                # Base-only candidate differences do not affect self geometry.
                # Reuse certificates only for exactly identical joint vectors.
                unique_q, inverse = np.unique(values[rows], axis=0, return_inverse=True)
                shoulder_rows = np.zeros(len(unique_q), dtype=bool)
                np.logical_or.at(shoulder_rows, inverse, gaps[rows, pairs[0]] < 0.)
                fine = self.shoulder_wrist_proxy.pair_gaps(unique_q, shoulder_rows)[inverse]
                for column, pair in enumerate(pairs):
                    positive = (gaps[rows, pair] < 0.) & np.isfinite(fine[:, column]) & (fine[:, column] > 0.)
                    gaps[rows[positive], pair] = fine[positive, column]
        return gaps[0] if single else gaps

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
        self_gaps = self.self_gaps(q, spheres)
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
                'scope': ('native sampled spheres, disclosed pair exclusions'
                          + ('; shoulder/wrist convex support refinement'
                             if self.shoulder_wrist_proxy is not None else '')
                          + '; no external obstacle check')}

    def check_batch(self, q, collision_margin=0.):
        values, _ = self._q_batch(q)
        return [self.check(value, collision_margin=collision_margin) for value in values]

    def collision_policy(self):
        return {'sphere_link_ids': self.link_ids.tolist(), 'checked_self_pairs': self._self_pairs.tolist(),
                'excluded_self_pairs': list(self.excluded_self_pairs),
                'excluded_base_arm_links': list(self.excluded_base_arm_links),
                'base_arm_exclusion_reason': 'native Piper mounting adjacency rule; not full mesh certification',
                'ground_rule': 'arm sphere bottom >= ground_clearance',
                'shoulder_wrist_refinement': (None if self.shoulder_wrist_proxy is None else {
                    'coarse_pairs': [[0, 18], [7, 18]], 'trigger': 'negative coarse gap only',
                    'shoulder_meshes': ['arm_base', 'link1'],
                    'upper_arm_mesh': 'link2; two full-triangle enclosing hulls split at local z=0',
                    'wrist_meshes': ['link5', 'link6'],
                    'method': 'positive support separation for every complete original mesh hull involved',
                    'additional_shoulder_axes': len(self.shoulder_wrist_proxy.shoulder_axes),
                    'complete_triangle_wrist_parts': len(self.shoulder_wrist_proxy.wrist_parts),
                    'numerical_tolerance_m': self.shoulder_wrist_proxy.numerical_tolerance,
                    'nonpositive_certificate': 'retain coarse rejection',
                    'physical_calibration': False})}

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
