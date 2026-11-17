"""Single-target surface-cloud prediction for the explicit standby experiment.

Only timestamped world-frame sensor points are accepted.  No scenario or actor
data is imported.  Consecutive observed centroids estimate planar velocity.
The 2 cm surface padding covers the declared 2 cm surface sampling spacing;
it is applied to physical sphere radii without a distance-field gradient.
"""
from dataclasses import dataclass
import numpy as np
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class ObservedCloudSnapshot:
    stamp: float
    points: np.ndarray
    velocity: np.ndarray
    tree: object
    padding: float = .02
    horizon: float = 3.
    max_age: float = .25

    def fresh(self, now):
        return bool(-.05 <= now-self.stamp <= self.max_age)

    def query(self, spheres, times):
        spheres = np.asarray(spheres, dtype=float)
        shape = spheres.shape[:-2]
        age = np.broadcast_to(np.asarray(times)-self.stamp, shape)
        forecast_ok = (age >= -.05) & (age <= self.horizon+1e-9)
        if self.tree is None:
            gap = np.full(shape, 10.)
            return np.where(forecast_ok, gap, -1e3), np.zeros(shape)
        points = spheres[..., :3]-np.maximum(0., age)[..., None, None]*self.velocity
        distance = self.tree.query(points.reshape((-1, 3)), k=1)[0].reshape(spheres.shape[:-1])
        radii = spheres[..., 3]+self.padding
        gap = np.min(distance-radii, axis=-1)
        soft = np.sum(np.maximum(0., radii+.08-distance)**3, axis=-1)*180.
        return np.where(forecast_ok, gap, -1e3), soft


class ObservedCloudTracker:
    def __init__(self, padding=.02, horizon=3., max_age=.25, max_speed=1.):
        self.padding = float(padding); self.horizon = float(horizon)
        self.max_age = float(max_age); self.max_speed = float(max_speed)
        if not np.isfinite([self.padding, self.horizon, self.max_age, self.max_speed]).all() or \
                min(self.padding, self.horizon, self.max_age, self.max_speed) <= 0.:
            raise ValueError('invalid observed-cloud configuration')
        self.previous = None
        self.latest = None

    def update(self, points, stamp):
        points = np.asarray(points, dtype=float).reshape((-1, 3)).copy()
        stamp = float(stamp)
        if not np.isfinite(stamp) or stamp <= 0. or not np.isfinite(points).all():
            raise ValueError('invalid observed point cloud')
        if self.latest is not None and stamp <= self.latest.stamp:
            raise ValueError('nonmonotonic observed cloud')
        velocity = np.zeros(3)
        if len(points):
            centroid = np.mean(points, axis=0)
            if self.previous is not None:
                old_stamp, old_centroid = self.previous
                dt = stamp-old_stamp
                if 1e-5 < dt <= .5:
                    velocity[:2] = (centroid[:2]-old_centroid[:2])/dt
                    speed = np.linalg.norm(velocity[:2])
                    if speed > self.max_speed:
                        raise ValueError('observed target velocity exceeds declared bound')
            self.previous = (stamp, centroid)
            tree = cKDTree(points)
        else:
            self.previous = None
            tree = None
        points.setflags(write=False); velocity.setflags(write=False)
        self.latest = ObservedCloudSnapshot(stamp, points, velocity, tree,
            self.padding, self.horizon, self.max_age)
        return self.latest
