"""Mapping-only native voxel representatives; no ROS master or robot commands."""
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from velodyne_transformer import cloud_xyz, mapping_voxel_indices, mapping_xyz_cloud
import test_robot_self_filter as fixture


class MappingCloudTests(unittest.TestCase):
    def indices(self, points, sensor=(0., 0., .117)):
        return mapping_voxel_indices(np.asarray(points), sensor, .05,
                                     [-7.5, -5., 0.], [15., 10., 3.], 4.5)

    def test_preserves_first_measurement_and_input_order(self):
        points = [[2.011, .101, .501], [1.011, .101, .501],
                  [2.012, .102, .502], [1.012, .102, .502],
                  [1.061, .101, .501]]
        np.testing.assert_array_equal(self.indices(points), [0, 1, 4])
        # The last point is a separate 5cm obstacle cell, even though it is
        # close enough that a coarser general-purpose VoxelGrid could erase it.
        keys = np.floor((np.asarray(points)-[-7.5, -5., 0.])/.05).astype(int)
        self.assertEqual(set(map(tuple, keys)), set(map(tuple, keys[self.indices(points)])))

    def test_keeps_all_long_beams_before_native_range_clipping(self):
        points = [[4.501, .101, .501], [4.502, .102, .502]]
        np.testing.assert_array_equal(self.indices(points), [0, 1])

    def test_keeps_map_exterior_and_border_beams(self):
        points = [[7.501, .101, .501], [7.502, .102, .502],
                  [-7.49999, .101, .501], [-7.49998, .102, .502]]
        np.testing.assert_array_equal(self.indices(points, sensor=(7.4, 0., .1)), [0, 1, 2, 3])

    def test_invalid_geometry_is_rejected(self):
        for resolution in (0., -.05, float('nan')):
            with self.assertRaises(ValueError):
                mapping_voxel_indices(np.empty((0, 3)), [0., 0., 0.], resolution,
                                      [-7.5, -5., 0.], [15., 10., 3.], 4.5)
        np.testing.assert_array_equal(self.indices(np.empty((0, 3))), [])

    def test_outside_projection_first_point_cannot_remove_valid_hit(self):
        points = np.array([[2.601, .101, 1.501], [2.602, .102, 1.502]])
        indices = mapping_voxel_indices(points, [0., 0., .117], .05,
                                        [-7.5, -5., 0.], [15., 10., 3.], 4.5,
                                        np.array([False, True]))
        np.testing.assert_array_equal(indices, [0, 1])
        with self.assertRaises(ValueError):
            mapping_voxel_indices(points, [0., 0., .117], .05,
                                  [-7.5, -5., 0.], [15., 10., 3.], 4.5, [True])

    def test_xyz_subset_preserves_source_metadata_and_measured_values(self):
        header = Header(seq=42, stamp=rospy.Time.from_sec(12.345), frame_id='livox_frame')
        fields = [PointField(name, i*4, PointField.FLOAT32, 1)
                  for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
        full = point_cloud2.create_cloud(header, fields, [(1., 2., 3., 9.), (4., 5., 6., 8.)])
        output = mapping_xyz_cloud(full, np.array([1]))
        self.assertEqual(output.header, header)
        self.assertEqual(output.point_step, 12)
        self.assertEqual(output.row_step, 12)
        self.assertEqual([field.name for field in output.fields], ['x', 'y', 'z'])
        np.testing.assert_array_equal(cloud_xyz(output), [[4., 5., 6.]])
        self.assertEqual(full.point_step, 16)
        self.assertEqual([field.name for field in full.fields], ['x', 'y', 'z', 'intensity'])

    def test_xyz_reader_respects_big_endian_padded_rows_and_offsets(self):
        fields = [PointField('z', 0, PointField.FLOAT32, 1),
                  PointField('x', 4, PointField.FLOAT32, 1),
                  PointField('y', 8, PointField.FLOAT32, 1)]
        full = PointCloud2(height=2, width=1, fields=fields, is_bigendian=True,
                           point_step=12, row_step=16,
                           data=struct.pack('>fff', 3., 1., 2.)+b'pad!'+
                                struct.pack('>fff', 6., 4., 5.)+b'pad!')
        np.testing.assert_array_equal(cloud_xyz(full), [[1., 2., 3.], [4., 5., 6.]])


class MappingCallbackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixture.RobotSelfFilterTests.setUpClass()

    def setUp(self):
        self.fixture = fixture.RobotSelfFilterTests()
        self.fixture.setUp()
        self.node = self.fixture.node
        self.node.mapping_pub = Mock()
        self.node.mapping_pub.get_num_connections.return_value = 1
        self.node.mapping_resolution = .05
        self.node.mapping_origin = np.array([-7.5, -5., 0.])
        self.node.mapping_size = np.array([15., 10., 3.])
        self.node.mapping_max_ray_length = 4.5
        self.node.mapping_local_range = np.array([8., 8., 3.])
        self.points = [[1.011, .101, .501], [1.012, .102, .502], [1.061, .101, .501]]

    def test_full_tracking_cloud_is_identical_with_mapping_enabled(self):
        self.node.mapping_pub.get_num_connections.return_value = 0
        self.fixture.deliver(self.points)
        before = self.node.pub.publish.call_args.args[0]
        self.node.mapping_pub.get_num_connections.return_value = 1
        self.fixture.deliver(self.points)
        after = self.node.pub.publish.call_args.args[0]
        self.assertEqual(after, before)
        mapping = self.node.mapping_pub.publish.call_args.args[0]
        self.assertEqual(mapping.header, after.header)
        np.testing.assert_array_equal(cloud_xyz(mapping), cloud_xyz(after)[[0, 2]])

    def test_no_mapping_subscriber_skips_all_deduplication_and_packaging(self):
        self.node.mapping_pub.get_num_connections.return_value = 0
        with patch('velodyne_transformer.mapping_voxel_indices', side_effect=AssertionError), \
                patch('velodyne_transformer.mapping_xyz_cloud', side_effect=AssertionError):
            self.fixture.deliver(self.points)
        self.node.pub.publish.assert_called_once()
        self.node.mapping_pub.publish.assert_not_called()

    def test_mapping_failure_does_not_suppress_original_cloud(self):
        with patch('velodyne_transformer.mapping_voxel_indices', side_effect=ValueError('bad map')):
            self.fixture.deliver(self.points)
        self.node.pub.publish.assert_called_once()
        self.node.mapping_pub.publish.assert_not_called()

    def test_transformed_output_uses_its_actual_points_and_sensor_origin(self):
        self.node.target_frame = 'mapped_lidar'
        original_lookup = self.node.tf_buffer.lookup_transform.side_effect
        def lookup(target, source, stamp, timeout):
            if (target, source) == ('mapped_lidar', 'livox_frame'):
                result = fixture.transform(self.fixture.scan_stamp, [-.1, 0., 0.])
                result.header.frame_id = 'mapped_lidar'
                return result
            if (target, source) == ('world', 'mapped_lidar'):
                return fixture.transform(self.fixture.scan_stamp, [.38, 0., .117])
            return original_lookup(target, source, stamp, timeout)
        self.node.tf_buffer.lookup_transform.side_effect = lookup
        with patch('velodyne_transformer.mapping_voxel_indices', wraps=mapping_voxel_indices) as dedup:
            self.fixture.deliver(self.points)
        output = self.node.pub.publish.call_args.args[0]
        mapping = self.node.mapping_pub.publish.call_args.args[0]
        self.assertEqual(mapping.header.frame_id, 'mapped_lidar')
        args = dedup.call_args.args
        np.testing.assert_allclose(args[1], [.38, 0., .117], atol=1e-12)
        np.testing.assert_allclose(args[0], cloud_xyz(output)+[.38, 0., .117], atol=1e-12)

    def test_tilted_sensor_projection_boundary_preserves_later_valid_hit(self):
        original_lookup = self.node.tf_buffer.lookup_transform.side_effect
        angle = np.deg2rad(60.)
        rotation = np.array([[np.cos(angle), 0., np.sin(angle)], [0., 1., 0.],
                             [-np.sin(angle), 0., np.cos(angle)]])
        def lookup(target, source, stamp, timeout):
            if (target, source) == ('world', 'livox_frame'):
                result = fixture.transform(self.fixture.scan_stamp, self.fixture.sensor_translation)
                result.transform.rotation.y = np.sin(angle/2.)
                result.transform.rotation.w = np.cos(angle/2.)
                return result
            return original_lookup(target, source, stamp, timeout)
        self.node.tf_buffer.lookup_transform.side_effect = lookup
        xyz = np.array([[.011, .101, 3.001], [.011, .101, 2.999]])
        world = xyz @ rotation.T+self.fixture.sensor_translation
        keys = np.floor((world-self.node.mapping_origin)*20.).astype(int)
        np.testing.assert_array_equal(keys[0], keys[1])
        fields = [PointField(name, 4*i, PointField.FLOAT32, 1)
                  for i, name in enumerate(('x', 'y', 'z'))]
        cloud = point_cloud2.create_cloud(Header(stamp=rospy.Time.from_sec(self.fixture.scan_stamp),
                                                 frame_id='livox_frame'), fields, xyz)
        with patch('velodyne_transformer.rospy.get_time', return_value=self.fixture.now), \
                patch('velodyne_transformer.rospy.logwarn_throttle'):
            self.node.cloud_cb(cloud)
        full = self.node.pub.publish.call_args.args[0]
        mapping = self.node.mapping_pub.publish.call_args.args[0]
        self.assertEqual(full.width, 2)
        np.testing.assert_array_equal(cloud_xyz(mapping), cloud_xyz(full))

    def test_additional_work_cannot_publish_expired_mapping_evidence(self):
        with patch('velodyne_transformer.rospy.get_time', side_effect=[100.04, 100.04, 100.20]), \
                patch('velodyne_transformer.rospy.logwarn_throttle'):
            self.node.cloud_cb(self.fixture.cloud(self.points))
        self.node.pub.publish.assert_called_once()
        self.node.mapping_pub.publish.assert_not_called()


class MappingLaunchTests(unittest.TestCase):
    def test_real_default_and_explicit_switch_keep_tracker_on_full_cloud(self):
        from roslaunch.config import ROSLaunchConfig
        from roslaunch.xmlloader import XmlLoader
        path = Path(__file__).resolve().parents[1]/'launch/exp0_real.launch'
        for arguments, sparse in (([], True), (['use_mppi:=false'], False),
                                  (['enable_mapping_cloud_sparse:=false'], False),
                                  (['use_mppi:=false', 'enable_mapping_cloud_sparse:=true'], True)):
            with self.subTest(arguments=arguments):
                config = ROSLaunchConfig()
                XmlLoader().load(str(path), config,
                                argv=['hardware_enabled:=false']+arguments, verbose=False)
                planner = next(node for node in config.nodes if node.name == 'remani_planner_node')
                topic = '/velodyne_points_mapping' if sparse else '/velodyne_points'
                self.assertIn(['~grid_map/cloud', topic], planner.remap_args)
                self.assertEqual(config.params['/robot_self_filter/mapping_sparse'].value, sparse)
                self.assertEqual(config.params['/dynamic_obstacle_tracker/input_topic'].value,
                                 '/velodyne_points')


if __name__ == '__main__':
    unittest.main()
