"""Geometry/provenance checks for the free-base standby scene."""
import unittest
import numpy as np

from active_arm_model import PiperArmModel
from standby_arm_scene import (actors_3d_at, geometry_precheck, obstacle_points,
                               scene_definition, validate_scene, witness_joints)


class StandbySceneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = PiperArmModel.from_defaults()

    def test_independent_clock_and_free_base(self):
        scene = scene_definition()
        self.assertTrue(validate_scene(scene))
        self.assertEqual(scene['base_constraint'], 'free')
        self.assertIsNone(scene['navigation_goal'])
        self.assertEqual(actors_3d_at(1.99), [])
        self.assertAlmostEqual(actors_3d_at(2.)[0]['y'], .65)
        self.assertAlmostEqual(actors_3d_at(10.)[0]['y'], .01)
        scene['actors'][0]['x'] = 9.
        self.assertAlmostEqual(scene_definition()['actors'][0]['x'], .16)

    def test_only_finite_height_surfaces(self):
        points = obstacle_points(10.)
        self.assertEqual(points.shape[1], 3)
        self.assertGreater(len(points), 100)
        self.assertTrue(np.all(points[:, 2] >= 1.05-1e-6))
        self.assertTrue(np.all(points[:, 2] <= 1.30+1e-6))
        self.assertEqual(obstacle_points(0.).shape, (0, 3))
        with self.assertRaises(ValueError):
            obstacle_points(10., spacing=0.)

    def test_offline_witness_rest_to_rest(self):
        times = np.array([0., 4., 6., 16., 18., 24.])
        q, dq, _ = witness_joints(times)
        start = np.asarray(scene_definition()['q_start'])
        retract = np.asarray(scene_definition()['posture_prior'])
        np.testing.assert_allclose(q[[0, 1, 4, 5]], np.tile(start, (4, 1)))
        np.testing.assert_allclose(q[[2, 3]], np.tile(retract, (2, 1)))
        np.testing.assert_allclose(dq, 0.)

    def test_obstacle_hits_held_arm_but_arm_only_route_exists(self):
        result = geometry_precheck(self.model)
        self.assertTrue(result['passed'], result)
        self.assertLess(result['held_arm_min_external_gap_m'], -.01)
        self.assertGreater(result['arm_only_witness_min_external_gap_m'], .05)
        self.assertGreater(result['held_arm_first_sampled_collision_s'], 8.)
        self.assertEqual(result['actual_experiment_base_constraint'], 'free')


if __name__ == '__main__':
    unittest.main()
