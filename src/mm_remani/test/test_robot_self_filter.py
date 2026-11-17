"""Physical-sphere sensor filtering at the source epoch; no ROS node/master."""
from collections import deque
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'scripts'))
sys.path.insert(0, str(PACKAGE/'scripts/mppi'))
from active_arm_model import PiperArmModel
from velodyne_transformer import CloudTransformer, outside_self_spheres


def transform(stamp, translation=(0., 0., 0.), yaw=0.):
    msg = TransformStamped(); msg.header.stamp = rospy.Time.from_sec(stamp)
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = translation
    msg.transform.rotation.z = np.sin(yaw/2); msg.transform.rotation.w = np.cos(yaw/2)
    return msg


class RobotSelfFilterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        config = PACKAGE.parent/'mm_remani_planner/plan_manage/config'
        params = yaml.safe_load((config/'piper_param.yaml').read_text())
        params['mm'].update(yaml.safe_load((config/'fixed_collision_spheres_b.yaml').read_text())['mm'])
        cls.model = PiperArmModel.from_parameters(params)
        cls.q_a = np.deg2rad([0., 110., -60., 0., 30., 0.])
        cls.q_b = np.deg2rad([25., 140., -60., 0., 15., 0.])

    def setUp(self):
        self.now = 100.04; self.scan_stamp = 100.03
        self.sensor_translation = np.array([.28, 0., .117])
        node = self.node = CloudTransformer.__new__(CloudTransformer)
        node.filter_self = True; node.target_frame = 'livox_frame'; node.model = self.model
        node.joint_lock = threading.Lock(); node.q = self.q_a.copy(); node.joint_stamp = self.scan_stamp
        node.joint_condition = threading.Condition(node.joint_lock)
        node.joint_history = deque([(self.scan_stamp, self.q_a.copy())], maxlen=64)
        node.pub = Mock(); node.tf_buffer = Mock()
        def lookup(target, source, stamp, timeout):
            self.assertAlmostEqual(stamp.to_sec(), self.scan_stamp)
            if (target, source) == ('world', 'livox_frame'):
                return transform(self.scan_stamp, self.sensor_translation)
            if (target, source) == ('world', 'base_link'):
                return transform(self.scan_stamp, [0., 0., .115])
            raise AssertionError('unexpected transform: '+repr((target, source)))
        node.tf_buffer.lookup_transform.side_effect = lookup

    def cloud(self, world_points):
        fields = [PointField(name, i*4, PointField.FLOAT32, 1)
                  for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
        source = np.asarray(world_points).reshape((-1, 3))-self.sensor_translation
        rows = [tuple(p)+(float(i+10),) for i, p in enumerate(source)]
        return point_cloud2.create_cloud(Header(stamp=rospy.Time.from_sec(self.scan_stamp),
                                                 frame_id='livox_frame'), fields, rows)

    def deliver(self, points):
        with patch('velodyne_transformer.rospy.get_time', side_effect=lambda: self.now), \
                patch('velodyne_transformer.rospy.logwarn_throttle'):
            self.node.cloud_cb(self.cloud(points))

    def output(self):
        result = self.node.pub.publish.call_args.args[0]
        self.assertEqual(result.header.frame_id, 'livox_frame')
        self.assertAlmostEqual(result.header.stamp.to_sec(), self.scan_stamp)
        return np.asarray(list(point_cloud2.read_points(result)), dtype=float)

    def test_changing_arm_posture_removes_only_current_robot_hits(self):
        a = self.model.body_spheres(self.q_a); b = self.model.body_spheres(self.q_b)
        old = a[47:, :3][outside_self_spheres(a[47:, :3], b)][0]
        new = b[47:, :3][outside_self_spheres(b[47:, :3], a)][0]
        for q, self_hit, external in ((self.q_a, old, new), (self.q_b, new, old)):
            with self.subTest(q=q.tolist()):
                self.node.q = q.copy(); self.node.joint_history.clear()
                self.node.joint_history.append((self.scan_stamp, q.copy()))
                self.node.pub.reset_mock(); self.deliver([self_hit, external])
                output = self.output()
                self.assertEqual(len(output), 1)
                np.testing.assert_allclose(output[0, :3]+self.sensor_translation, external, atol=1e-7)
                self.assertEqual(output[0, 3], 11.)

    def test_real_obstacle_inside_old_big_box_but_outside_physical_spheres_is_retained(self):
        external = np.array([-.5, .3, .7])
        self.assertTrue(np.all(external >= [-.8, -.85, -.2]))
        self.assertTrue(np.all(external <= [.45, .55, .9]))
        spheres = self.model.body_spheres(self.q_a)
        self.assertTrue(outside_self_spheres(external[None], spheres)[0])
        self.deliver([spheres[3, :3], external])
        output = self.output(); self.assertEqual(len(output), 1)
        np.testing.assert_allclose(output[0, :3]+self.sensor_translation, external, atol=1e-7)

    def test_scan_joint_interpolation_uses_bracketing_source_times(self):
        later = self.q_a.copy(); later[0] += .01
        self.node.q = later; self.node.joint_stamp = 100.04
        self.node.joint_history = deque([(100.02, self.q_a.copy()), (100.04, later.copy())], maxlen=64)
        q, oldest = self.node.joints_at_scan(100.03, 100.04)
        np.testing.assert_allclose(q, (self.q_a+later)/2, atol=1e-12)
        self.assertAlmostEqual(oldest, 100.02)
        point = self.model.body_spheres(q)[-1, :3]
        self.deliver([point]); self.node.pub.publish.assert_called_once()
        self.assertEqual(self.node.pub.publish.call_args.args[0].width, 0)

    def test_old_or_duplicate_feedback_cannot_clear_newer_valid_pose(self):
        for stamp in (99.8, self.scan_stamp):
            with self.subTest(stamp=stamp):
                msg = JointState(); msg.header.stamp = rospy.Time.from_sec(stamp)
                msg.name = ['joint%d'%i for i in range(1, 7)]
                # Even invalid content in an obsolete source has no authority
                # to revoke the pose already accepted from a newer source.
                msg.position = [np.nan]*6
                with patch('velodyne_transformer.rospy.get_time', return_value=self.now), \
                        patch('velodyne_transformer.rospy.logwarn_throttle'):
                    self.node.joints_cb(msg)
                np.testing.assert_array_equal(self.node.q, self.q_a)
                self.assertEqual(self.node.joint_stamp, self.scan_stamp)
                self.assertEqual(len(self.node.joint_history), 1)

    def test_invalid_feedback_without_obsolete_source_still_clears_pose(self):
        for stamp in (0., np.nan, 100.04):
            with self.subTest(stamp=stamp):
                self.node.q = self.q_a.copy()
                self.node.joint_history = deque([(self.scan_stamp, self.q_a.copy())], maxlen=64)
                msg = Mock(); msg.header.stamp.to_sec.return_value = stamp
                msg.name = ['joint%d'%i for i in range(1, 7)]
                msg.position = ([np.nan]*6 if stamp == 100.04 else self.q_a.tolist())
                with patch('velodyne_transformer.rospy.get_time', return_value=self.now), \
                        patch('velodyne_transformer.rospy.logwarn_throttle'):
                    self.node.joints_cb(msg)
                self.assertIsNone(self.node.q)
                self.assertFalse(self.node.joint_history)

    def test_invalid_message_cannot_revoke_source_accepted_during_validation(self):
        newer = JointState(); newer.header.stamp = rospy.Time.from_sec(100.04)
        newer.name = ['joint%d'%i for i in range(1, 7)]
        newer.position = self.q_b.tolist()
        node = self.node
        class InterleavedNames(list):
            delivered = False
            def index(names, value):
                if not names.delivered:
                    names.delivered = True
                    node.joints_cb(newer)
                return super().index(value)
        invalid = JointState(); invalid.header.stamp = rospy.Time.from_sec(100.035)
        invalid.name = InterleavedNames(newer.name); invalid.position = [np.nan]*6
        with patch('velodyne_transformer.rospy.get_time', return_value=self.now), \
                patch('velodyne_transformer.rospy.logwarn_throttle'):
            node.joints_cb(invalid)
        np.testing.assert_array_equal(node.q, self.q_b)
        self.assertEqual(node.joint_stamp, 100.04)
        self.assertEqual(len(node.joint_history), 2)

    def test_stale_scan_log_identifies_scan_source_age_before_joint_lookup(self):
        self.now = 100.24
        with patch('velodyne_transformer.rospy.get_time', return_value=self.now), \
                patch('velodyne_transformer.rospy.logwarn_throttle') as warning, \
                patch.object(self.node, 'joints_at_scan') as lookup:
            self.node.cloud_cb(self.cloud([[1., 0., .5]]))
        lookup.assert_not_called()
        message = str(warning.call_args.args[-1])
        self.assertIn('self-filter scan source time', message)
        self.assertIn('stamp=100.030000000', message)
        self.assertIn('now=100.240000000', message)
        self.assertIn('age_ms=210.000', message)
        self.node.pub.publish.assert_not_called()

    def test_stale_scan_does_not_latch_recovery_with_new_scan_and_joints(self):
        self.now = 103.04
        self.deliver([[1., 0., .5]])
        self.node.pub.publish.assert_not_called()
        self.scan_stamp = self.now
        msg = JointState(); msg.header.stamp = rospy.Time.from_sec(self.scan_stamp)
        msg.name = ['joint%d'%i for i in range(1, 7)]; msg.position = self.q_a.tolist()
        with patch('velodyne_transformer.rospy.get_time', return_value=self.now):
            self.node.joints_cb(msg)
        self.deliver([[1., 0., .5]])
        self.node.pub.publish.assert_called_once()

    def test_latest_feedback_one_hundred_ms_after_scan_cannot_replace_missing_history(self):
        self.node.joint_stamp = 100.13; self.now = 100.14
        self.node.joint_history = deque([(100.13, self.q_a.copy())], maxlen=64)
        self.deliver([[1., 0., .5]])
        self.node.pub.publish.assert_not_called()

    def delayed_joint_source(self, stamp, q, delay=.025):
        errors = []
        def deliver():
            try:
                time.sleep(delay)
                msg = JointState(); msg.header.stamp = rospy.Time.from_sec(stamp)
                msg.name = ['joint%d'%i for i in range(1, 7)]
                msg.position = np.asarray(q).tolist()
                self.node.joints_cb(msg)
            except BaseException as exc:
                errors.append(exc)
        thread = threading.Thread(target=deliver)
        thread.start()
        return thread, errors

    def lagging_joint_history(self, stamp=100.):
        self.node.joint_stamp = stamp
        self.node.joint_history = deque([(stamp, self.q_a.copy())], maxlen=64)

    def test_real_thread_delayed_source_sample_unblocks_same_epoch_cloud_and_interpolates(self):
        self.lagging_joint_history()
        later = self.q_a.copy(); later[0] += .007
        expected_q = self.q_a+(later-self.q_a)*(.03/.035)
        body_point = self.model.body_spheres(expected_q)[-1, :3]
        # A true joint callback on another thread must be able to acquire the
        # lock while cloud processing awaits its missing right-hand sample.
        with patch('velodyne_transformer.rospy.get_time', return_value=self.now), \
                patch('velodyne_transformer.rospy.logwarn_throttle'), \
                patch.object(self.node.model, 'body_spheres', wraps=self.node.model.body_spheres) as geometry:
            thread, errors = self.delayed_joint_source(100.035, later)
            try:
                self.node.cloud_cb(self.cloud([body_point, [-.5, .3, .7]]))
            finally:
                thread.join(timeout=.5)
        self.assertFalse(thread.is_alive())
        self.assertFalse(errors)
        self.node.pub.publish.assert_called_once()
        np.testing.assert_allclose(geometry.call_args.args[0], expected_q, atol=1e-12)
        self.assertAlmostEqual(self.node.joint_history[-1][0], 100.035)
        output = self.output(); self.assertEqual(len(output), 1)

    def test_missing_delayed_sample_times_out_without_publishing_or_using_old_q(self):
        self.lagging_joint_history()
        began = time.monotonic()
        self.deliver([[1., 0., .5]])
        elapsed = time.monotonic()-began
        self.assertGreaterEqual(elapsed, .045)
        self.assertLess(elapsed, .15)
        self.node.pub.publish.assert_not_called()
        self.node.tf_buffer.lookup_transform.assert_not_called()
        self.assertEqual(self.node.joint_stamp, 100.)

    def test_invalid_delayed_sample_wakes_and_rejects_instead_of_fabricating_alignment(self):
        self.lagging_joint_history()
        invalid = self.q_a.copy(); invalid[0] = np.nan
        with patch('velodyne_transformer.rospy.get_time', return_value=self.now), \
                patch('velodyne_transformer.rospy.logwarn_throttle'):
            thread, errors = self.delayed_joint_source(100.035, invalid)
            try:
                self.node.cloud_cb(self.cloud([[1., 0., .5]]))
            finally:
                thread.join(timeout=.5)
        self.assertFalse(thread.is_alive())
        self.assertFalse(errors)
        self.assertIsNone(self.node.q)
        self.node.pub.publish.assert_not_called()
        self.node.tf_buffer.lookup_transform.assert_not_called()

    def test_already_stale_feedback_is_rejected_without_waiting_for_replacement(self):
        self.lagging_joint_history(99.8)
        self.deliver([[1., 0., .5]])
        self.node.pub.publish.assert_not_called()
        self.node.tf_buffer.lookup_transform.assert_not_called()

    def test_wait_time_is_charged_to_original_scan_age_budget(self):
        # This synthetic combination probes the deadline calculation directly;
        # the ordinary callback additionally rejects a stale source beforehand.
        self.lagging_joint_history()
        self.now = 100.145
        began = time.monotonic()
        with self.assertRaises(ValueError):
            self.node.joints_at_scan(100.03, self.now)
        self.assertLess(time.monotonic()-began, .05)

    def test_stale_or_missing_joint_feedback_suppresses_frame(self):
        for missing in (False, True):
            with self.subTest(missing=missing):
                self.node.q = None if missing else self.q_a.copy()
                self.node.joint_stamp = 99.; self.node.pub.reset_mock()
                self.deliver([[1., 0., .5]])
                self.node.pub.publish.assert_not_called()

    def test_missing_exact_tf_suppresses_frame_without_latest_fallback(self):
        self.node.tf_buffer.lookup_transform.side_effect = tf2_ros.ExtrapolationException('no exact scan TF')
        self.deliver([[1., 0., .5]])
        self.node.pub.publish.assert_not_called()
        self.assertEqual(self.node.tf_buffer.lookup_transform.call_count, 1)
        self.assertAlmostEqual(self.node.tf_buffer.lookup_transform.call_args.args[2].to_sec(), self.scan_stamp)

    def test_missing_exact_base_tf_also_suppresses_frame(self):
        self.node.tf_buffer.lookup_transform.side_effect = [
            transform(self.scan_stamp, self.sensor_translation),
            tf2_ros.ExtrapolationException('base transform missing at scan epoch')]
        self.deliver([[1., 0., .5]])
        self.node.pub.publish.assert_not_called()
        self.assertEqual(self.node.tf_buffer.lookup_transform.call_count, 2)
        for call in self.node.tf_buffer.lookup_transform.call_args_list:
            self.assertAlmostEqual(call.args[2].to_sec(), self.scan_stamp)

    def test_large_joint_history_gap_is_not_treated_as_observed_pose(self):
        self.node.joint_history = deque([(99.99, self.q_a.copy()), (100.06, self.q_b.copy())], maxlen=64)
        self.node.joint_stamp = 100.06; self.now = 100.065
        self.deliver([[1., 0., .5]])
        self.node.pub.publish.assert_not_called()

    def test_visualization_mode_retains_its_legacy_latest_tf_fallback(self):
        self.node.filter_self = False; self.node.target_frame = 'world'
        self.node.tf_buffer.lookup_transform.side_effect = [
            tf2_ros.ExtrapolationException('visualization-only exact TF missing'),
            transform(self.scan_stamp)]
        with patch('velodyne_transformer.tf2_sensor_msgs.do_transform_cloud', return_value=self.cloud([[1., 0., .5]])):
            self.deliver([[1., 0., .5]])
        self.node.pub.publish.assert_called_once()
        self.assertEqual(self.node.tf_buffer.lookup_transform.call_args.args[2].to_sec(), 0.)

    def test_invalid_model_suppresses_even_an_empty_cloud(self):
        self.node.model = Mock(body_spheres=Mock(return_value=np.zeros((67, 4))))
        self.deliver(np.empty((0, 3)))
        self.node.pub.publish.assert_not_called()

    def test_chunking_matches_dense_reference_and_rejects_nonfinite_points(self):
        rng = np.random.RandomState(7)
        spheres = self.model.body_spheres(self.q_a)
        points = rng.uniform([-.9, -.8, -.1], [.8, .8, 1.4], size=(5001, 3))
        expected = np.all(np.sum((points[:, None, :]-spheres[None, :, :3])**2, axis=-1) >
                          (spheres[:, 3]+.01)**2, axis=1)
        np.testing.assert_array_equal(outside_self_spheres(points, spheres), expected)
        points[0, 0] = np.nan
        self.assertFalse(outside_self_spheres(points, spheres)[0])

    def test_aabb_broad_phase_matches_brute_spheres_at_random_and_float_boundaries(self):
        rng = np.random.RandomState(17)
        for q in (self.q_a, self.q_b):
            spheres = self.model.body_spheres(q, [.3, -.2, .7])
            radii = spheres[:, 3]+.01
            points = [rng.uniform([-2., -2., .2], [2., 2., 1.5], (7001, 3))]
            # Exact axis extremities and adjacent representable floats probe
            # sphere and global AABB faces, including translated robot poses.
            for axis in range(3):
                for sign in (-1., 1.):
                    face = spheres[:, :3].copy(); face[:, axis] += sign*radii
                    points.extend((face, np.nextafter(face, -np.inf), np.nextafter(face, np.inf)))
            points = np.concatenate(points)
            delta = points[:, None, :]-spheres[None, :, :3]
            expected = ~np.any(np.einsum('nsi,nsi->ns', delta, delta) <= radii**2, axis=1)
            np.testing.assert_array_equal(outside_self_spheres(points, spheres), expected)

    def test_joint_names_are_used_instead_of_array_order(self):
        msg = JointState(); msg.header.stamp = rospy.Time.from_sec(100.04)
        msg.name = ['joint%d'%i for i in range(6, 0, -1)]
        msg.position = self.q_b[::-1].tolist()
        with patch('velodyne_transformer.rospy.get_time', return_value=100.04):
            self.node.joints_cb(msg)
        np.testing.assert_allclose(self.node.q, self.q_b)


if __name__ == '__main__':
    unittest.main()
