"""Independent geometry and sensor consistency checks; no ROS or actuation."""
import unittest

import numpy as np

import active_arm_scene as scene


class ActiveArmSceneTests(unittest.TestCase):
    def test_appearance_and_no_truth_controller_field(self):
        definition = scene.scene_definition()
        self.assertEqual(scene.actors_at(1.99), [])
        self.assertEqual(scene.obstacle_points(1.99).shape, (0, 3))
        self.assertEqual(len(scene.actors_at(2.)), 1)
        definition['actors'][0]['x'] = 99.
        self.assertNotEqual(scene.scene_definition()['actors'][0]['x'], 99.)

    def test_vertical_separation_changes_collision(self):
        actor = dict(x=0., y=0., r=.2, z_min=1., z_max=1.3)
        self.assertGreater(scene.cylinder_gaps([[0., 0., .6, .1]], [actor])[0], .29)
        self.assertLess(scene.cylinder_gaps([[0., 0., 1.15, .1]], [actor])[0], 0.)
        self.assertAlmostEqual(scene.cylinder_gaps([[.4, 0., 1.15, .1]], [actor])[0], .1)

    def test_cloud_finite_height_matches_declared_solid(self):
        definition = scene.scene_definition()
        points = scene.obstacle_points(2., definition)
        actor = definition['actors'][0]
        self.assertGreater(len(points), 100)
        self.assertAlmostEqual(float(points[:, 2].min()), actor['z_min'], places=6)
        self.assertAlmostEqual(float(points[:, 2].max()), actor['z_max'], places=6)
        radial = np.hypot(points[:, 0]-actor['x'], points[:, 1]-actor['y'])
        self.assertLessEqual(float(radial.max()), actor['r']+1e-6)
        tiny_spheres = np.column_stack([points, np.full(len(points), 1e-6)])
        self.assertLess(float(np.max(np.abs(scene.cylinder_gaps(tiny_spheres, [actor])))), 2e-6)

    def test_prescribed_motion_uses_appearance_epoch(self):
        definition = scene.scene_definition()
        definition['actors'][0].update(vx=.1, vy=-.2)
        actor = scene.actors_3d_at(3., definition)[0]
        self.assertAlmostEqual(actor['x'], 1.7)
        self.assertAlmostEqual(actor['y'], -.2)

    def test_native_geometry_has_arm_and_base_alternatives(self):
        from active_arm_model import PiperArmModel
        result = scene.straight_path_precheck(PiperArmModel.from_defaults())
        self.assertTrue(result['passed'], result)
        self.assertLess(result['start']['arm_min_gap_m'], -.01)
        self.assertGreater(result['retract']['arm_min_gap_m'], .05)
        self.assertGreater(result['fixed_arm_bypass']['cylinder_min_physical_gap_m'], .05)
        self.assertEqual(result['joint_transition']['samples'], 257)


if __name__ == '__main__':
    unittest.main()
