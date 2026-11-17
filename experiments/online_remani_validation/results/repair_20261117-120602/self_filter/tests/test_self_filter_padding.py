"""Filter-only fixed-sphere corrections; no ROS master or robot commands."""
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

import numpy as np

ROOT = next(p for p in Path(__file__).resolve().parents
            if (p/'src/mm_remani/test/test_robot_self_filter.py').is_file())
sys.path.insert(0, str(ROOT/'src/mm_remani/test'))
import test_robot_self_filter as fixture
from velodyne_transformer import self_filter_padding, outside_self_spheres, cloud_xyz


class FixedPaddingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixture.RobotSelfFilterTests.setUpClass()
        cls.model = fixture.RobotSelfFilterTests.model
        cls.q = fixture.RobotSelfFilterTests.q_a
        cls.spheres = cls.model.body_spheres(cls.q)
        cls.points = np.array([
            [.1728383687378493, -.00020761928009899872, .2408999074127342],
            [.18877529079151872, .011616975250937876, .19420893496497227],
            [.1907732404673227, .012317346633954562, .19517789414149042],
        ])

    def candidate(self):
        return self_filter_padding(self.model.fixed_spheres, [[14, .011], [28, .011]])

    def test_empty_overrides_preserve_default_masks_and_physical_geometry(self):
        before = self.model.fixed_spheres.copy()
        padding = self_filter_padding(before, [])
        np.testing.assert_array_equal(padding, np.full(67, .01))
        rng = np.random.RandomState(17)
        samples = rng.uniform([-.6, -.6, .15], [.6, .6, 1.4], (4097, 3))
        samples = np.vstack([samples, self.points])
        np.testing.assert_array_equal(outside_self_spheres(samples, self.spheres),
                                      outside_self_spheres(samples, self.spheres, padding))
        self.candidate()
        np.testing.assert_array_equal(before, self.model.fixed_spheres)

    def test_bad_override_configuration_is_rejected(self):
        bad = [None, {}, [[14]], [[14, .011, 0]], [(14, .011)],
               [[-1, .011]], [[47, .011]], [[66, .011]], [[True, .011]],
               [[14., .011]], [['14', .011]], [[14, .011], [14, .012]],
               [[14, -.01]], [[14, .009]], [[14, float('nan')]],
               [[14, float('inf')]], [[14, '.011']], [[14, True]]]
        for config in bad:
            with self.subTest(config=config), self.assertRaises(ValueError):
                self_filter_padding(self.model.fixed_spheres, config)
        for geometry in (self.model.fixed_spheres[:-1], np.full((47, 4), np.nan)):
            with self.assertRaises(ValueError):
                self_filter_padding(geometry, [])

    def test_bad_padding_shapes_or_nonfinite_entries_are_rejected(self):
        for padding in ([.01], np.full(66, .01), np.full((67, 1), .01),
                        np.full(67, np.nan), np.full(67, -.001)):
            with self.subTest(shape=np.asarray(padding).shape), self.assertRaises(ValueError):
                outside_self_spheres(self.points, self.spheres, padding)

    def test_three_recorded_witnesses_removed_only_with_opt_in(self):
        np.testing.assert_array_equal(outside_self_spheres(self.points, self.spheres), [True]*3)
        np.testing.assert_array_equal(outside_self_spheres(self.points, self.spheres,
                                                         self.candidate()), [False]*3)
        changed = np.flatnonzero(self.candidate() != .01)
        np.testing.assert_array_equal(changed, [14, 28])

    def test_external_points_just_beyond_local_shell_survive_pose_changes(self):
        indices = [28, 14, 14]
        centers = self.spheres[indices, :3]
        directions = self.points-centers
        directions /= np.linalg.norm(directions, axis=1)[:, None]
        external = centers+directions*(self.spheres[indices, 3]+.0111)[:, None]
        for yaw in (0., .7, -2.1):
            c, s = np.cos(yaw), np.sin(yaw)
            rotation = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
            moved_points = external@rotation.T+[1.2, -.8, 0.]
            spheres = self.model.body_spheres(self.q, [1.2, -.8, yaw])
            with self.subTest(yaw=yaw):
                np.testing.assert_array_equal(outside_self_spheres(moved_points, spheres,
                                                                 self.candidate()), [True]*3)

    def test_unselected_fixed_and_arm_shells_keep_original_padding(self):
        # Isolate spheres so another sphere cannot accidentally explain retention.
        spheres = np.column_stack([np.arange(67)*3., np.zeros(67), np.zeros(67), np.full(67, .1)])
        points = spheres[:, :3]+[.1105, 0., 0.]
        padding = self_filter_padding(spheres[:47], [[14, .011], [28, .011]])
        keep = outside_self_spheres(points, spheres, padding)
        np.testing.assert_array_equal(np.flatnonzero(~keep), [14, 28])

    def test_selected_shell_expands_broad_phase_boundary(self):
        spheres = np.column_stack([np.arange(67)*3., np.zeros(67), np.zeros(67), np.full(67, .1)])
        spheres[14, 1] = 10.
        point = spheres[14, :3]+[0., .1105, 0.]
        padding = self_filter_padding(spheres[:47], [[14, .011]])
        self.assertTrue(outside_self_spheres(point[None], spheres)[0])
        self.assertFalse(outside_self_spheres(point[None], spheres, padding)[0])

    def test_same_voxel_external_endpoint_survives_filter_then_mapping(self):
        f = fixture.RobotSelfFilterTests()
        f.setUp()
        f.node.self_filter_padding = self.candidate()
        f.node.mapping_pub = Mock()
        f.node.mapping_pub.get_num_connections.return_value = 1
        f.node.mapping_resolution = .05
        f.node.mapping_origin = np.array([-7.5, -5., 0.])
        f.node.mapping_size = np.array([15., 10., 3.])
        f.node.mapping_max_ray_length = 4.5
        f.node.mapping_local_range = np.array([8., 8., 3.])
        self_return = self.points[1]
        center = self.spheres[14, :3]
        direction = (self_return-center)/np.linalg.norm(self_return-center)
        external = center+direction*(self.spheres[14, 3]+.0115)
        np.testing.assert_array_equal(np.floor((self_return-f.node.mapping_origin)/.05),
                                      np.floor((external-f.node.mapping_origin)/.05))
        f.deliver([self_return, external])
        output = f.output()
        self.assertEqual(len(output), 1)
        self.assertEqual(output[0, 3], 11.)  # The actual second source measurement.
        np.testing.assert_allclose(output[0, :3]+f.sensor_translation, external, atol=1e-7)
        full = f.node.pub.publish.call_args.args[0]
        mapping = f.node.mapping_pub.publish.call_args.args[0]
        self.assertEqual(mapping.header, full.header)
        np.testing.assert_array_equal(cloud_xyz(mapping), cloud_xyz(full))

    def test_constructor_defaults_and_rejects_invalid_configuration_before_subscription(self):
        for overrides in ([], [[14, .011], [28, .011]], [[47, .011]]):
            values = {'~filter_self': True, '~mapping_sparse': False,
                      '~target_frame': 'livox_frame', '~fixed_padding_overrides': overrides,
                      '/remani_planner_node': {}}
            with patch('velodyne_transformer.rospy.init_node'), \
                    patch('velodyne_transformer.tf2_ros.Buffer'), \
                    patch('velodyne_transformer.tf2_ros.TransformListener'), \
                    patch('velodyne_transformer.rospy.get_param', side_effect=lambda k, d=None: values.get(k, d)), \
                    patch('active_arm_model.PiperArmModel.from_parameters', return_value=self.model), \
                    patch('velodyne_transformer.rospy.Publisher'), \
                    patch('velodyne_transformer.rospy.Subscriber') as subscribe, \
                    patch('velodyne_transformer.rospy.loginfo'):
                if overrides == [[47, .011]]:
                    with self.assertRaises(ValueError):
                        fixture.CloudTransformer()
                    subscribe.assert_not_called()
                else:
                    node = fixture.CloudTransformer()
                    expected = np.full(67, .01) if not overrides else self.candidate()
                    np.testing.assert_array_equal(node.self_filter_padding, expected)


if __name__ == '__main__':
    unittest.main()
