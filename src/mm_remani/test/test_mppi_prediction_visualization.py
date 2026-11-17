"""Prediction observation must not alter control selection or failure."""
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'scripts'))
sys.path.insert(0, str(PACKAGE/'scripts'/'mppi'))
from active_arm_mppi import WholeBodyMPPI
from standby_whole_body_mppi import StandbyWholeBodyMPPI
from prediction_visualization import encode_prediction, decode_prediction
from mppi_prediction_visualizer import render_prediction, PredictionVisualizer
from whole_body_mppi import WholeBodyNode, reference_wait_hold


class Robot:
    def body_spheres_batch(self, q, base):
        spheres = np.zeros((len(q), 2, 4))
        spheres[:, :, :2] = base[:, None, :2]
        spheres[:, 0, 2] = .2
        spheres[:, 1, 2] = .6+q[:, 0]
        spheres[:, :, 3] = .05
        return spheres

    def configuration_gaps_batch(self, q):
        return np.ones(len(q))


class PredictionVisualizationTests(unittest.TestCase):
    def kernel(self, standby=False):
        kind = StandbyWholeBodyMPPI if standby else WholeBodyMPPI
        return kind(Robot(), dict(vmax=.2, wmax=.35, accel=.25,
                    angular_accel=.8, track=.37, wheel_v=.2296, wheel_a=.2975),
                    (np.full(6, -1.), np.ones(6), np.full(6, .15), np.full(6, .35)),
                    [[0., .2, 0., 0., 0., 0.]], horizon=7, random_samples=4)

    def test_observation_preserves_selection_and_failure_in_both_modes(self):
        state = np.zeros(17)
        reference = np.c_[np.linspace(0., .8, 20), np.zeros((20, 2)), np.full(20, .12)]
        for standby in (False, True):
            for collision in (False, True):
                original = self.kernel(standby); observed = self.kernel(standby)
                observed.capture_rollouts = True
                snapshot = SimpleNamespace(query=lambda spheres, times:
                    (np.full(len(spheres), -1. if collision else 1.), np.zeros(len(spheres))))
                a, ai = original.plan(state, reference, np.zeros(6), snapshot, 100.)
                b, bi = observed.plan(state, reference, np.zeros(6), snapshot, 100.)
                self.assertEqual(ai['feasible'], bi['feasible'])
                self.assertIsNone(original.visual_rollouts)
                self.assertIsNotNone(observed.visual_rollouts)
                if a is None:
                    self.assertIsNone(b)
                    self.assertFalse(observed.visual_rollouts[1].any())
                else:
                    np.testing.assert_array_equal(a[0], b[0])
                    np.testing.assert_array_equal(a[1], b[1])
                wire = encode_prediction(100., .1, 0 if b is None else 1,
                                         observed.visual_rollouts, None if b is None else b[1])
                self.assertLess(wire.nbytes, 10*1024)
                self.assertEqual(decode_prediction(wire)['status'], 0 if b is None else 1)

    def test_invalid_packets_rejected_before_rendering(self):
        wire = encode_prediction(100., .1, 0, (np.zeros((2, 8, 3)), np.zeros(2)))
        for index, value in ((0, 0.), (2, np.nan), (4, 3.), (5, 1000.), (6, .5), (8, .5)):
            bad = wire.copy(); bad[index] = value
            with self.assertRaises(ValueError):
                decode_prediction(bad)
        with self.assertRaises(ValueError):
            decode_prediction(wire[:-1])

    def test_render_uses_source_time_physical_spheres_and_status(self):
        selected = np.zeros((8, 17)); selected[:, 0] = np.linspace(1., 1.14, 8)
        selected[:, 5] = np.linspace(0., .1, 8)
        # Two-arm-sphere test model needs four spheres for the compatibility
        # endpoint, so use a duplicated physical geometry with distinct z.
        model = Robot()
        physical = model.body_spheres_batch(selected[:, 5:11], selected[:, :3])
        model.body_spheres_batch = lambda q, base: np.tile(physical, (1, 2, 1))
        wire = encode_prediction(100., .1, 1, (selected[None, :, :3], [True]), selected)
        markers, path, _ = render_prediction(model, decode_prediction(wire))
        self.assertEqual(path.header.stamp.to_sec(), 100.)
        ghosts = [m for m in markers.markers if m.ns == 'mppi_whole_body_future']
        self.assertEqual(len(ghosts), 4)
        self.assertTrue(all(abs(m.scale.x-.1) < 1e-12 for m in ghosts))
        self.assertAlmostEqual(ghosts[0].points[0].x, 1.)
        self.assertAlmostEqual(ghosts[-1].points[0].x, 1.14)
        self.assertTrue(any('FIRST COMMAND DELIVERED' in m.text for m in markers.markers))

    def test_no_subscriber_disables_export_and_viewer_unsubscribes(self):
        node = WholeBodyNode.__new__(WholeBodyNode)
        node.visual_hz = 2.; node.last_visual = 0.
        node.rollout_pub = Mock(); node.rollout_pub.get_num_connections.return_value = 0
        self.assertFalse(node.visualization_due(100.))
        viewer = PredictionVisualizer.__new__(PredictionVisualizer)
        viewer.marker_pub = Mock(); viewer.path_pub = Mock(); viewer.arm_pub = Mock()
        for publisher in (viewer.marker_pub, viewer.path_pub, viewer.arm_pub):
            publisher.get_num_connections.return_value = 0
        import threading
        viewer.lock = threading.Lock(); viewer.latest = np.ones(20)
        subscriber = viewer.subscriber = Mock(); viewer.visible = False
        viewer.tick(None)
        subscriber.unregister.assert_called_once()
        self.assertIsNone(viewer.subscriber)
        self.assertIsNone(viewer.latest)

    def test_visualization_publication_failure_cannot_reject_control(self):
        node = WholeBodyNode.__new__(WholeBodyNode)
        node.rollout_pub = Mock(); node.rollout_pub.publish.side_effect = ValueError('display transport failed')
        node.reject = Mock()
        kernel = SimpleNamespace(dt=.1, visual_rollouts=(np.zeros((1, 8, 3)), [True]))
        with patch('whole_body_mppi.rospy.logwarn_throttle'):
            node.publish_prediction(kernel, np.zeros((8, 17)), 100., 1)
        node.reject.assert_not_called()

    def test_hold_braking_prediction_uses_its_actual_half_step(self):
        kernel = self.kernel(); state = np.zeros(17); state[3] = .1
        _, prediction = reference_wait_hold(kernel, state)
        node = WholeBodyNode.__new__(WholeBodyNode); node.rollout_pub = Mock()
        node.publish_prediction(kernel, prediction, 100., 1, kernel.dt/2)
        packet = decode_prediction(node.rollout_pub.publish.call_args.args[0].data)
        self.assertEqual(packet['dt'], .05)
        self.assertEqual(len(packet['selected']), len(prediction))


if __name__ == '__main__':
    unittest.main()
