"""Read-only hardware inputs. No ROS or actuator dependencies.

These surface queries support shadow diagnostics only. They do not certify
unobserved space, sensor coverage, mesh geometry, or hardware execution.
"""
from dataclasses import dataclass
import math
import numpy as np
from scipy.spatial import cKDTree


JOINT_NAMES = tuple('joint%d' % i for i in range(1, 7))


def finite_stamp(stamp):
    stamp = float(stamp)
    if not math.isfinite(stamp) or stamp <= 0:
        raise ValueError('invalid_stamp')
    return stamp


def fresh(stamp, now, max_age=.25):
    return bool(np.isfinite([stamp, now]).all() and stamp > 0 and -.05 <= now-stamp <= max_age)


def transform_points(points, translation, quaternion):
    """Transform xyz to the declared world frame using a timestamped TF."""
    p = np.asarray(points, dtype=float).reshape((-1, 3))
    t = np.asarray(translation, dtype=float)
    q = np.asarray(quaternion, dtype=float)
    if t.shape != (3,) or q.shape != (4,) or not np.isfinite(np.r_[p.ravel(), t, q]).all():
        raise ValueError('invalid_transform_or_points')
    norm = np.linalg.norm(q)
    if abs(norm-1.) > .02:
        raise ValueError('invalid_tf_quaternion')
    x, y, z, w = q/norm
    r = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                  [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                  [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return p @ r.T+t, r


def require_world_frame(frame, world_frame):
    if frame != world_frame:
        raise ValueError('dynamic_world_frame_required')


class JointPositionDifferencer:
    """Radians from named positions only; driver velocity units are ignored."""
    def __init__(self, lower, upper, velocity, min_dt=.01, max_dt=.25):
        self.lower, self.upper, self.velocity = [np.asarray(v, dtype=float) for v in (lower, upper, velocity)]
        if any(v.shape != (6,) or not np.isfinite(v).all() for v in (self.lower, self.upper, self.velocity)):
            raise ValueError('invalid_joint_limits')
        self.min_dt, self.max_dt = min_dt, max_dt
        self.previous = None

    def update(self, names, positions, stamp, frame='', allowed_frames=('', 'base_link')):
        stamp = finite_stamp(stamp)
        if frame not in allowed_frames:
            self.previous = None
            raise ValueError('unexpected_joint_frame')
        if len(names) != len(positions) or len(set(names)) != len(names) or any(n not in names for n in JOINT_NAMES):
            self.previous = None
            raise ValueError('missing_or_duplicate_joint_names')
        q = np.asarray([positions[names.index(n)] for n in JOINT_NAMES], dtype=float)
        if not np.isfinite(q).all() or np.any(q < self.lower-1e-5) or np.any(q > self.upper+1e-5):
            self.previous = None
            raise ValueError('joint_position_invalid_radians')
        previous = self.previous
        if previous is None:
            self.previous = (stamp, q.copy())
            raise ValueError('joint_velocity_warming_up')
        dt = stamp-previous[0]
        if dt <= 0:
            raise ValueError('nonmonotonic_joint_stamp')
        if dt < self.min_dt:
            raise ValueError('joint_dt_too_short')
        self.previous = (stamp, q.copy())
        if dt > self.max_dt:
            raise ValueError('joint_dt_too_long')
        dq = (q-previous[1])/dt
        if np.any(np.abs(dq) > self.velocity+.10):
            raise ValueError('joint_velocity_or_units_invalid')
        return q, dq


@dataclass(frozen=True)
class SurfaceObservation:
    stamp: float
    frame: str
    points: np.ndarray
    source: str
    tree: object

    @classmethod
    def create(cls, points, stamp, frame, world_frame, source, max_points=4000):
        finite_stamp(stamp)
        if frame != world_frame:
            raise ValueError('surface_tf_missing')
        p = np.asarray(points, dtype=float).reshape((-1, 3)).copy()
        if not len(p):
            raise ValueError('empty_'+source)
        if not np.isfinite(p).all():
            raise ValueError('nonfinite_'+source)
        if len(p) > max_points:
            p = p[np.linspace(0, len(p)-1, max_points, dtype=int)]
        p.setflags(write=False)
        return cls(float(stamp), frame, p, source, cKDTree(p))


@dataclass(frozen=True)
class DynamicObservation:
    stamp: float
    frame: str
    groups: tuple
    point_count: int

    @classmethod
    def create(cls, rows, stamp, frame, world_frame, min_confidence=.5, max_speed=2.):
        finite_stamp(stamp)
        require_world_frame(frame, world_frame)
        a = np.asarray(rows, dtype=float).reshape((-1, 8))
        if not len(a):
            raise ValueError('empty_dynamic_cloud')
        if not np.isfinite(a).all() or np.any(a[:, 7] < 0) or np.any(a[:, 7] != np.floor(a[:, 7])):
            raise ValueError('invalid_dynamic_cloud')
        if np.any((a[:, 6] < 0) | (a[:, 6] > 1)) or np.any(np.linalg.norm(a[:, 3:6], axis=1) > max_speed):
            raise ValueError('invalid_dynamic_velocity_or_confidence')
        a = a[a[:, 6] >= min_confidence]
        if not len(a):
            raise ValueError('no_confident_dynamic_points')
        groups = []
        for key in np.unique(a[:, [3, 4, 5, 7]], axis=0):
            points = a[np.all(a[:, [3, 4, 5, 7]] == key, axis=1), :3].copy()
            velocity = key[:3].copy()
            points.setflags(write=False); velocity.setflags(write=False)
            groups.append((int(key[3]), velocity, cKDTree(points)))
        return cls(float(stamp), frame, tuple(groups), len(a))


@dataclass(frozen=True)
class HardwareCloudSnapshot:
    raw: SurfaceObservation
    filtered: SurfaceObservation
    static: SurfaceObservation
    dynamic: DynamicObservation
    health_stamp: float
    healthy: bool
    max_age: float = .25
    horizon: float = 2.5
    padding: float = .08
    position_uncertainty: float = .08
    velocity_uncertainty: float = .25
    acceleration_bound: float = .75

    @property
    def stamp(self):
        return self.dynamic.stamp

    def rejection_reason(self, now):
        for name in ('raw', 'filtered', 'static', 'dynamic'):
            observation = getattr(self, name)
            if observation is None:
                return name+'_not_received'
            # A static PCD is latched and intentionally has no live-age gate.
            if name != 'static' and not fresh(observation.stamp, now, self.max_age):
                return name+'_stale'
        if not self.healthy or not fresh(self.health_stamp, now, self.max_age):
            return 'perception_health_unavailable'
        return None

    def fresh(self, now):
        return self.rejection_reason(now) is None

    def query(self, spheres, times):
        s = np.asarray(spheres, dtype=float)
        shape = s.shape[:-2]
        times = np.broadcast_to(np.asarray(times), shape)
        age = times-self.dynamic.stamp
        distance = np.full(s.shape[:-1], np.inf)
        # Raw and filtered clouds stay separate, with zero velocity; unqualified
        # objects must not silently disappear when predictive tracks are empty.
        for observation in (self.static, self.raw, self.filtered):
            d = observation.tree.query(s[..., :3].reshape((-1, 3)), k=1)[0].reshape(distance.shape)
            distance = np.minimum(distance, d-self.padding)
        unc = self.position_uncertainty+self.velocity_uncertainty*np.maximum(0., age)+.5*self.acceleration_bound*np.maximum(0., age)**2
        for _, velocity, tree in self.dynamic.groups:
            point = s[..., :3]-np.maximum(0., age)[..., None, None]*velocity
            d = tree.query(point.reshape((-1, 3)), k=1)[0].reshape(distance.shape)
            distance = np.minimum(distance, d-unc[..., None])
        ok = (age >= -.05) & (age <= self.horizon)
        surface = distance-s[..., 3]
        gap = np.min(surface, axis=-1)
        soft = 180.*np.sum(np.maximum(0., .08-surface)**3, axis=-1)
        return np.where(ok, gap, -1e3), soft
