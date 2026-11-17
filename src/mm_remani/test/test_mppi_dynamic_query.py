"""Conservative dynamic-query pruning versus all-pairs distances; no ROS."""
from pathlib import Path
import sys
import unittest

import numpy as np
from scipy.ndimage import map_coordinates
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts/mppi'))
from active_arm_mppi import Snapshot


def exhaustive(snapshot, spheres, times):
    h = snapshot.header
    points = spheres[..., :3]
    f = (points-h[8:11])/h[11]-.5
    inside = np.all((f >= 0) & (f < np.array(snapshot.distance.shape)-1), axis=-1)
    distances = map_coordinates(snapshot.distance, np.moveaxis(f, -1, 0),
                                order=1, prefilter=False, mode='nearest')
    distances = np.where(inside, distances, -1e3)
    age = np.broadcast_to(np.asarray(times)-h[18], spheres.shape[:-2])
    if snapshot.groups:
        distances = np.where(((age >= -.05) & (age <= h[16]+1e-9))[..., None], distances, -1e3)
    future = np.maximum(age, 0.)
    margin = h[25]+h[26]*future+.5*h[27]*future**2
    for velocity, tree in snapshot.groups:
        moved = points-future[..., None, None]*velocity
        difference = moved[..., None, :]-tree.data
        nearest = np.min(np.sqrt(np.sum(difference*difference, axis=-1)), axis=-1)
        distances = np.minimum(distances, nearest-margin[..., None])
    radii = spheres[..., 3].copy(); radii[..., :47] += h[11]
    return np.min(distances-radii, axis=-1), np.sum(np.maximum(0., radii+.08-distances)**3, axis=-1)*180.


class CountingTree:
    def __init__(self, points):
        self.tree = cKDTree(points)
        self.data, self.mins, self.maxes = self.tree.data, self.tree.mins, self.tree.maxes
        self.queried = 0

    def query(self, points, **kwargs):
        self.queried += len(points)
        return self.tree.query(points, **kwargs)


class DynamicQueryTests(unittest.TestCase):
    def snapshot(self, groups, uncertainty=(0., 0., 0.)):
        h = np.zeros(32); h[8:11] = -2.; h[11] = .2; h[16] = 2.; h[18] = 100.
        h[25:28] = uncertainty
        grid = np.random.RandomState(9).uniform(.3, 1.5, (20, 20, 20))
        return Snapshot(h, grid, groups)

    def test_random_multi_group_shapes_uncertainty_and_forecast_boundaries(self):
        rng = np.random.RandomState(927)
        groups = [(rng.uniform(-.3, .3, 3), cKDTree(rng.uniform(-.8, .8, (23, 3))))
                  for _ in range(2)]
        for shape in ((67, 4), (8, 67, 4), (2, 4, 67, 4)):
            spheres = rng.uniform(-1.5, 1.5, shape); spheres[..., 3] = rng.uniform(.02, .15, shape[:-1])
            times = (np.resize([99.94, 99.95, 100., 100.1, 100.8, 102., 102.001, 100.5], shape[:-2])
                     if shape[:-2] else 100.8)
            for uncertainty in ((0., 0., 0.), (.08, .25, .75), (.02, .01, .02)):
                snapshot = self.snapshot(groups, uncertainty)
                for unknown in (False, True):
                    sample = spheres.copy()
                    if unknown:sample[..., 0, :3] = 2.5
                    for actual, expected in zip(snapshot.query(sample, times), exhaustive(snapshot, sample, times)):
                        np.testing.assert_allclose(actual, expected, rtol=1e-13, atol=1e-12)

    def test_distant_spheres_are_skipped_without_losing_minimum_or_soft_cost(self):
        tree = CountingTree([[0., 0., 0.]])
        snapshot = self.snapshot([(np.zeros(3), tree)])
        spheres = np.tile([1., 1., 1., .02], (1, 67, 1))
        spheres[0, 47] = [.07, 0., 0., .02]  # Positive 5cm gap and nonzero soft cost.
        actual = snapshot.query(spheres, 100.)
        expected = exhaustive(snapshot, spheres, 100.)
        for a, b in zip(actual, expected):np.testing.assert_allclose(a, b, atol=1e-15, rtol=0)
        self.assertGreater(tree.queried, 0)
        self.assertLess(tree.queried, 67)
        self.assertGreater(actual[1][0], 0.)

    def test_exact_minimum_ties_and_soft_boundary_remain_included(self):
        snapshot = self.snapshot([(np.zeros(3), cKDTree([[0., 0., 0.]]))])
        snapshot.distance[:] = 10.
        for separation in (.02, .1-1e-12, .1, .1+1e-12, .3):
            spheres = np.tile([1., 1., 1., .02], (1, 67, 1))
            spheres[0, 47:49] = [separation, 0., 0., .02]
            for actual, expected in zip(snapshot.query(spheres, 100.), exhaustive(snapshot, spheres, 100.)):
                np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-15)


if __name__ == '__main__':
    unittest.main()
