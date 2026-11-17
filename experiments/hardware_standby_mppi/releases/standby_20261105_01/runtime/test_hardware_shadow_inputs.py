"""Meaningful unit checks for the hardware shadow input boundary."""
import ast
from pathlib import Path
import unittest

import numpy as np

from hardware_cloud_snapshot import (DynamicObservation, HardwareCloudSnapshot,
    JOINT_NAMES, JointPositionDifferencer, SurfaceObservation, transform_points)


class Inputs(unittest.TestCase):
    def estimator(self):
        return JointPositionDifferencer([-3.]*6, [3.]*6, [.45]*6)

    def test_joint_names_order_and_velocity_from_position_radians(self):
        d = self.estimator()
        names = list(reversed(JOINT_NAMES))
        with self.assertRaisesRegex(ValueError, 'warming_up'):
            d.update(names, [0.]*6, 10.)
        q, dq = d.update(names, [.01, .02, .03, .04, .05, .04], 10.1)
        np.testing.assert_allclose(q, [.04, .05, .04, .03, .02, .01])
        np.testing.assert_allclose(dq, q/.1)

    def test_invalid_names_frame_timestamp_units_and_discontinuity(self):
        for names, positions, stamp, frame, reason in [
            (list(JOINT_NAMES)[:-1], [0.]*5, 10., '', 'names'),
            (list(JOINT_NAMES), [0.]*6, 0., '', 'stamp'),
            (list(JOINT_NAMES), [0.]*6, 10., 'camera', 'frame'),
            (list(JOINT_NAMES), [90.]*6, 10., '', 'radians')]:
            with self.assertRaisesRegex(ValueError, reason):
                self.estimator().update(names, positions, stamp, frame)
        d = self.estimator()
        with self.assertRaises(ValueError):
            d.update(list(JOINT_NAMES), [0.]*6, 10.)
        with self.assertRaisesRegex(ValueError, 'nonmonotonic'):
            d.update(list(JOINT_NAMES), [0.]*6, 10.)
        with self.assertRaisesRegex(ValueError, 'too_long'):
            d.update(list(JOINT_NAMES), [0.]*6, 10.5)
        with self.assertRaisesRegex(ValueError, 'velocity_or_units'):
            d.update(list(JOINT_NAMES), [.2]*6, 10.6)

    def test_transform_has_translation_rotation_and_rejects_invalid_tf(self):
        p, r = transform_points([[1., 0., 1.]], [2., 0., 0.], [0., 0., np.sqrt(.5), np.sqrt(.5)])
        np.testing.assert_allclose(p, [[2., 1., 1.]], atol=1e-12)
        np.testing.assert_allclose(r @ [1., 0., 0.], [0., 1., 0.], atol=1e-12)
        with self.assertRaisesRegex(ValueError, 'quaternion'):
            transform_points([[0., 0., 0.]], [0., 0., 0.], [0., 0., 0., 0.])

    def surface(self, source='raw', stamp=10.):
        return SurfaceObservation.create([[10., 10., 10.]], stamp, 'odom', 'odom', source)

    def dynamic(self, rows=None):
        rows = [[1., 0., 1., -.1, 0., 0., .9, 1.]] if rows is None else rows
        return DynamicObservation.create(rows, 10., 'odom', 'odom')

    def test_empty_cloud_or_missing_tf_is_not_free_space(self):
        with self.assertRaisesRegex(ValueError, 'empty'):
            SurfaceObservation.create([], 10., 'odom', 'odom', 'raw')
        with self.assertRaisesRegex(ValueError, 'tf_missing'):
            SurfaceObservation.create([[0., 0., 0.]], 10., 'lidar', 'odom', 'raw')
        with self.assertRaisesRegex(ValueError, 'empty_dynamic'):
            self.dynamic([])
        with self.assertRaisesRegex(ValueError, 'world_frame'):
            DynamicObservation.create([[0.]*8], 10., 'lidar', 'odom')

    def test_snapshot_requires_live_raw_filtered_dynamic_and_health(self):
        s = self.surface()
        d = self.dynamic()
        snap = HardwareCloudSnapshot(s, s, s, d, 10., True)
        self.assertTrue(snap.fresh(10.1))
        self.assertFalse(snap.fresh(10.4))
        self.assertEqual(HardwareCloudSnapshot(None, s, s, d, 10., True).rejection_reason(10.1), 'raw_not_received')
        self.assertEqual(HardwareCloudSnapshot(s, s, s, d, 10., False).rejection_reason(10.1), 'perception_health_unavailable')

    def test_independent_target_velocities_and_uncertainty(self):
        d = self.dynamic([[1., 0., 1., -.1, 0., 0., .9, 1.],
                          [3., 0., 1., .2, 0., 0., .9, 2.]])
        self.assertEqual(len(d.groups), 2)
        s = self.surface()
        snap = HardwareCloudSnapshot(s, s, s, d, 10., True)
        spheres = np.array([[[0., 0., 1., .1]], [[0., 0., 1., .1]]])
        gap, _ = snap.query(spheres, [10., 11.])
        self.assertAlmostEqual(gap[0], 1.-.08-.1)
        self.assertAlmostEqual(gap[1], .9-(.08+.25+.5*.75)-.1)
        late, _ = snap.query(spheres[:1], [13.])
        self.assertLess(late[0], -100.)

    def test_node_publishers_are_only_prefixed_diagnostics(self):
        tree = ast.parse(Path(__file__).with_name('hardware_shadow_node.py').read_text())
        publishers = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and
            isinstance(n.func, ast.Attribute) and n.func.attr == 'Publisher']
        self.assertEqual(len(publishers), 3)
        for p in publishers:
            expr = p.args[0]
            self.assertIsInstance(expr, ast.BinOp)
            self.assertEqual(expr.left.attr, 'namespace')
            self.assertIn(expr.right.value, ('/status', '/markers', '/proposed_controls_never_executed'))


if __name__ == '__main__':
    unittest.main()
