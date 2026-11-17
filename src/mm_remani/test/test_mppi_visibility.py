"""Offline tests of the simulator's geometric visibility; no ROS imports/master."""
import ast
import math
import os
from pathlib import Path
import unittest

for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'
import numpy as np


def load_visibility():
    path = Path(__file__).resolve().parents[1]/'scripts'/'standby_simulator.py'
    definitions = [node for node in ast.parse(path.read_text()).body if
        (isinstance(node, ast.FunctionDef) and node.name in
         ('actor_at', 'obstacle_points', 'cylinder_occludes', 'sensor_cloud')) or
        (isinstance(node, ast.Assign) and any(isinstance(name, ast.Name) and
         name.id == 'SENSOR_MODELS' for name in node.targets))]
    namespace = {'np': np, 'math': math}
    exec(compile(ast.Module(body=definitions, type_ignores=[]), str(path), 'exec'), namespace)
    return (namespace[name] for name in ('actor_at', 'obstacle_points', 'cylinder_occludes', 'sensor_cloud'))


actor_at, obstacle_points, cylinder_occludes, sensor_cloud = load_visibility()
EMPTY = np.empty((0, 3))


class VisibilityTests(unittest.TestCase):
    def test_enter_blind_scene_starts_visible_without_changing_original_scenes(self):
        self.assertIsNone(actor_at(1.99, 'navigation_arm_enter_blind'))
        np.testing.assert_array_equal(actor_at(2., 'navigation_arm_crossing'), [.45,.65,.13,1.05,1.30])
        np.testing.assert_array_equal(actor_at(2., 'arm_crossing'), [.16,.65,.13,1.05,1.30])
        initial = actor_at(2., 'navigation_arm_enter_blind')
        late = actor_at(8., 'navigation_arm_enter_blind')
        self.assertAlmostEqual((late[1]-initial[1])/6., -.08)
        first, visible = sensor_cloud(EMPTY, initial, np.zeros(3), np.array([.28,0.,.002]), .115, 'mid360')
        lost, invisible = sensor_cloud(EMPTY, late, np.zeros(3), np.array([.28,0.,.002]), .115, 'mid360')
        self.assertGreater(len(first), 25)
        self.assertGreater(visible['actor_visible_fraction'], .1)
        self.assertEqual(len(lost), 0)
        self.assertEqual(invisible['actor_in_fov_points'], 0)
        ideal, _ = sensor_cloud(EMPTY, late, np.zeros(3), np.array([.28,0.,.002]), .115, 'ideal')
        self.assertEqual(len(ideal), len(obstacle_points(late)))

    def test_elevation_limits_near_blind_range_and_all_azimuths(self):
        points = []
        expected = []
        for azimuth in (0., math.pi/2., math.pi, -math.pi/2.):
            for elevation, accepted in ((-7.001, False), (-7., True), (0., True),
                                         (52., True), (52.001, False)):
                angle = math.radians(elevation)
                points.append([math.cos(angle)*math.cos(azimuth),
                               math.cos(angle)*math.sin(azimuth), math.sin(angle)])
                expected.append(accepted)
        points.extend([[.0999, 0., 0.], [.1, 0., 0.]])
        expected.extend([False, True])
        points = np.asarray(points)
        cloud, stats = sensor_cloud(points, None, np.zeros(3), np.zeros(3), 0., 'mid360')
        np.testing.assert_allclose(cloud, points[np.asarray(expected)], rtol=0., atol=1e-15)
        self.assertEqual(stats['static_visible_points'], sum(expected))

    def test_cylinder_near_far_faces_caps_and_foreground_occlusion(self):
        actor = np.array([2., 0., .5, 0., 2.])
        points = np.array([[1.5, 0., 1.], [2.5, 0., 1.], [2., 0., 2.],
                           [2., 0., 0.], [3., 0., 1.], [3., 2., 1.]])
        np.testing.assert_array_equal(cylinder_occludes(points, np.array([0.,0.,1.]), actor),
                                      [False, True, True, True, True, False])
        caps = np.array([[2.,0.,2.], [2.,0.,0.]])
        np.testing.assert_array_equal(cylinder_occludes(caps, np.array([0.,0.,3.]), actor), [False, True])
        np.testing.assert_array_equal(cylinder_occludes(caps, np.array([0.,0.,-1.]), actor), [True, False])

    def test_parallel_vertical_rays_and_float32_surface_precision(self):
        actor = np.array([2., 0., .13, 1.05, 1.30])
        # A ray parallel to the cylinder axis hits the bottom first; an outside
        # parallel ray never hits. This covers the zero quadratic coefficient.
        points = np.array([[2.,0.,1.05], [2.,0.,1.30]])
        np.testing.assert_array_equal(cylinder_occludes(points, np.array([2.,0.,0.]), actor), [False, True])
        self.assertFalse(cylinder_occludes(np.array([[3.,0.,2.]]), np.array([3.,0.,0.]), actor)[0])
        near = np.array([[2.-.13, 0., 1.2]], dtype=np.float32)
        self.assertFalse(cylinder_occludes(near, np.array([0.,0.,1.2]), actor)[0])

    def test_high_close_actor_is_unobserved_and_logged_without_filled_points(self):
        actor = np.array([.28, .20, .05, 1.05, 1.30])
        offset = np.array([.28, 0., .002])
        cloud, stats = sensor_cloud(EMPTY, actor, np.zeros(3), offset, .115, 'mid360')
        self.assertEqual(cloud.shape, (0,3))
        self.assertGreater(stats['actor_sampled_points'], 0)
        self.assertEqual(stats['actor_visible_points'], 0)
        self.assertEqual(stats['actor_visible_fraction'], 0.)
        self.assertEqual(stats['actor_fov_rejected_points'], stats['actor_sampled_points'])
        self.assertIsNone(stats['actor_visible_z_max_m'])
        self.assertGreater(stats['actor_elevation_min_deg'], 52.)
        self.assertAlmostEqual(stats['sensor_world_z_m'], .117)

    def test_partial_actor_view_reports_actual_visible_height_and_keeps_wall_prior_untouched(self):
        actor = np.array([1., 0., .10, .4, 1.3])
        walls = np.array([[1.5, 0., .7], [1.5, 1., .7], [1.5, 1., 3.]])
        original_walls = walls.copy()
        cloud, stats = sensor_cloud(walls, actor, np.zeros(3), np.array([.28,0.,.002]), .115, 'mid360')
        np.testing.assert_array_equal(walls, original_walls)
        self.assertEqual(stats['static_visible_points'], 1)  # behind cylinder / above FOV removed
        self.assertGreater(stats['actor_visible_fraction'], 0.)
        self.assertLess(stats['actor_visible_fraction'], 1.)
        self.assertGreater(stats['actor_occluded_points'], 0)
        self.assertLess(stats['actor_visible_z_max_m'], actor[4])
        angles = np.rad2deg(np.arctan2(cloud[:,2], np.linalg.norm(cloud[:,:2], axis=1)))
        self.assertTrue(np.all((angles >= -7.-1e-9) & (angles <= 52.+1e-9)))
        self.assertEqual(len(cloud), stats['static_visible_points']+stats['actor_visible_points'])

    def test_world_translation_yaw_and_mount_height_are_applied(self):
        walls = np.array([[1.5, .5, .4], [-1., -.5, .2]])
        offset = np.array([.28, -.04, .002])
        yaw = .71
        rotation = np.array([[math.cos(yaw), -math.sin(yaw), 0.],
                             [math.sin(yaw), math.cos(yaw), 0.], [0.,0.,1.]])
        shift = np.array([1.2, -.5, 0.])
        # Use only walls for rotational equivalence: resampling a cylinder after
        # rotation changes its finite angular sampling lattice.
        reference, first = sensor_cloud(walls, None, np.zeros(3), offset, .115, 'mid360')
        rotated, second = sensor_cloud(walls @ rotation.T+shift, None,
            np.array([shift[0],shift[1],yaw]), offset, .115, 'mid360')
        np.testing.assert_allclose(reference, rotated, rtol=0., atol=1e-14)
        self.assertEqual(first['static_visible_points'], second['static_visible_points'])
        point = np.array([[.78,0.,.9]])
        low, _ = sensor_cloud(point, None, np.zeros(3), np.array([.28,0.,.002]), .115, 'mid360')
        high, _ = sensor_cloud(point, None, np.zeros(3), np.array([.28,0.,.002]), .68, 'mid360')
        self.assertEqual((len(low), len(high)), (0,1))

    def test_ideal_preserves_all_sampled_points_and_order(self):
        actor = np.array([.28, .2, .05, 1.05, 1.30])
        walls = np.array([[1.,0.,3.], [.28,0.,.12]])
        state, offset = np.array([.2,-.1,.4]), np.array([.28,0.,.002])
        cloud, stats = sensor_cloud(walls, actor, state, offset, .115, 'ideal')
        c, s = math.cos(state[2]), math.sin(state[2])
        inverse = np.array([[c,s,0.],[-s,c,0.],[0.,0.,1.]])
        expected = (np.concatenate([walls,obstacle_points(actor)])-np.r_[state[:2],.115]) @ inverse.T-offset
        np.testing.assert_array_equal(cloud, expected)
        self.assertEqual(stats['actor_visible_fraction'], 1.)
        self.assertEqual(stats['actor_occluded_points'], 0)
        self.assertEqual(stats['static_visible_points'], len(walls))

    def test_empty_scene_and_unknown_model(self):
        cloud, stats = sensor_cloud(EMPTY, None, np.zeros(3), np.zeros(3), .115, 'mid360')
        self.assertEqual(cloud.shape, (0,3))
        self.assertFalse(stats['actor_present'])
        self.assertIsNone(stats['actor_visible_fraction'])
        with self.assertRaises(ValueError):
            sensor_cloud(EMPTY, None, np.zeros(3), np.zeros(3), .115, 'unknown')


if __name__ == '__main__':
    unittest.main()
