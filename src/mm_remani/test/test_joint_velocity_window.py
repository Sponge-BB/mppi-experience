"""Planning-only joint velocity conditioning; no ROS master or hardware."""
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

import rospy
from roslaunch.config import ROSLaunchConfig
from roslaunch.xmlloader import XmlLoader
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import filter_joint_states as feedback


EPOCH = 1794456400.0


def message(stamp, q, velocity=None, effort=None):
    result = JointState()
    result.header.stamp = rospy.Time.from_sec(stamp)
    result.header.frame_id = 'arm_feedback'
    result.name = list(feedback.TARGET_JOINTS)
    result.position = list(q)
    result.velocity = [] if velocity is None else list(velocity)
    result.effort = [] if effort is None else list(effort)
    return result


class JointVelocityWindowTests(unittest.TestCase):
    def estimator(self):
        return feedback.SourceTimeVelocityWindow(window_s=.05, max_gap_s=.15)

    def assert_velocity(self, actual, expected, tolerance=1e-5):
        self.assertIsNotNone(actual)
        self.assertEqual(len(actual), 6)
        for value, wanted in zip(actual, expected):
            self.assertAlmostEqual(value, wanted, delta=tolerance)

    def test_sustained_speed_is_not_clipped_to_the_planning_limit(self):
        estimator = self.estimator()
        speeds = [1.2, -1.2, .4, -.4, .15, 0.]
        delivered = 0
        for index in range(61):
            stamp = EPOCH + .005 * index
            elapsed = stamp - EPOCH
            velocity = estimator.update(stamp, [speed * elapsed for speed in speeds])
            if elapsed < .049:
                self.assertIsNone(velocity)
            elif elapsed > .051:
                self.assert_velocity(velocity, speeds)
                delivered += 1
        self.assertGreater(delivered, 40)

    def test_constant_acceleration_has_half_window_velocity_lag(self):
        estimator = self.estimator()
        accelerations = [.35, -.35, .2, -.2, 0., .1]
        for index in range(61):
            stamp = EPOCH + index * .005
            elapsed = stamp - EPOCH
            velocity = estimator.update(stamp, [.5 * a * elapsed**2 for a in accelerations])
            if elapsed > .051:
                self.assert_velocity(velocity, [a * (elapsed - .025) for a in accelerations])

    def test_isolated_position_pulse_is_not_zeroed_or_amplified_by_five_ms(self):
        estimator = self.estimator()
        observed = []
        for index in range(51):
            stamp = EPOCH + .005 * index
            q = [.0058 if index == 20 else 0.] + [0.] * 5
            velocity = estimator.update(stamp, q)
            if velocity is not None:
                observed.append(velocity[0])
                self.assertTrue(all(abs(v) < 1e-12 for v in velocity[1:]))
        self.assertGreater(max(observed), .11)
        self.assertLess(min(observed), -.11)
        self.assertLessEqual(max(abs(v) for v in observed), .11601)
        self.assertEqual(observed[-1], 0.)

    def test_irregular_source_spacing_preserves_linear_motion(self):
        estimator = self.estimator()
        speeds = [1., -.8, .6, -.4, .2, -.1]
        stamp = EPOCH
        outputs = 0
        for index in range(70):
            stamp += (.003, .008, .006, .011)[index % 4]
            elapsed = stamp - EPOCH
            velocity = estimator.update(stamp, [v * elapsed for v in speeds])
            if elapsed > .065:
                self.assert_velocity(velocity, speeds)
                outputs += 1
        self.assertGreater(outputs, 40)

    def test_warmup_and_long_gap_require_new_window(self):
        estimator = self.estimator()
        self.assertIsNone(estimator.update(EPOCH, [0.] * 6))
        self.assertIsNone(estimator.update(EPOCH + .025, [.025] * 6))
        self.assert_velocity(estimator.update(EPOCH + .055, [.055] * 6), [1.] * 6)
        # Never bridge the outage or silently reuse a pre-outage velocity.
        restart = EPOCH + .3
        self.assertIsNone(estimator.update(restart, [10.] * 6))
        self.assertIsNone(estimator.update(restart + .025, [10.025] * 6))
        self.assert_velocity(estimator.update(restart + .055, [10.055] * 6), [1.] * 6)

    def test_duplicate_and_backwards_samples_do_not_advance_history(self):
        estimator = self.estimator()
        estimator.update(EPOCH, [0.] * 6)
        self.assertIsNone(estimator.update(EPOCH, [100.] * 6))
        self.assertIsNone(estimator.update(EPOCH - .01, [-100.] * 6))
        self.assertIsNone(estimator.update(EPOCH + .025, [.025] * 6))
        self.assert_velocity(estimator.update(EPOCH + .055, [.055] * 6), [1.] * 6)
        self.assertIsNone(estimator.update(EPOCH + .055, [100.] * 6))
        self.assert_velocity(estimator.update(EPOCH + .080, [.080] * 6), [1.] * 6)


class JointVelocityCallbackTests(unittest.TestCase):
    def setUp(self):
        self.publisher = Mock()
        self.patchers = [
            patch.object(feedback, 'pub', self.publisher, create=True),
            patch.object(feedback, '_previous', None),
            patch.object(feedback, '_velocity_estimator', None),
        ]
        for patcher in self.patchers:
            patcher.start()
            self.addCleanup(patcher.stop)

    def test_conditioned_callback_keeps_current_position_source_time_and_effort(self):
        feedback._velocity_estimator = feedback.SourceTimeVelocityWindow(.05, .15)
        start = message(EPOCH, [0.] * 6, [99.] * 6, [.2] * 6)
        feedback.joint_cb(start)
        self.publisher.publish.assert_not_called()
        middle = message(EPOCH + .025, [.025] * 6, [99.] * 6, [.3] * 6)
        feedback.joint_cb(middle)
        self.publisher.publish.assert_not_called()
        current = message(EPOCH + .055, [.055] * 6, [99.] * 6, [.4] * 6)
        feedback.joint_cb(current)
        output = self.publisher.publish.call_args.args[0]
        self.assertIs(output.header, current.header)
        self.assertEqual(output.name, list(feedback.TARGET_JOINTS))
        self.assertEqual(output.position, current.position)
        self.assertEqual(output.effort, current.effort)
        for value in output.velocity:
            self.assertAlmostEqual(value, 1., delta=1e-5)

    def test_disabled_estimator_preserves_supplied_velocity(self):
        current = message(EPOCH, [.02] * 6, [1.2, -1.2, .4, -.4, .15, 0.], [.4] * 6)
        feedback.joint_cb(current)
        output = self.publisher.publish.call_args.args[0]
        self.assertIs(output.header, current.header)
        self.assertEqual(output.position, current.position)
        self.assertEqual(output.velocity, current.velocity)
        self.assertEqual(output.effort, current.effort)

    def test_disabled_estimator_keeps_legacy_missing_velocity_fallback(self):
        feedback.joint_cb(message(EPOCH, [0.] * 6))
        self.publisher.publish.assert_not_called()
        feedback.joint_cb(message(EPOCH + .05, [.01] * 6))
        output = self.publisher.publish.call_args.args[0]
        for value in output.velocity:
            self.assertAlmostEqual(value, .2, delta=1e-5)

    def test_repeated_source_time_does_not_publish_new_feedback(self):
        feedback._velocity_estimator = feedback.SourceTimeVelocityWindow(.05, .15)
        feedback.joint_cb(message(EPOCH, [0.] * 6))
        feedback.joint_cb(message(EPOCH + .055, [.055] * 6))
        self.assertEqual(self.publisher.publish.call_count, 1)
        feedback.joint_cb(message(EPOCH + .055, [100.] * 6))
        feedback.joint_cb(message(EPOCH + .025, [-100.] * 6))
        self.assertEqual(self.publisher.publish.call_count, 1)


class JointVelocityLaunchTests(unittest.TestCase):
    def test_real_mppi_shares_conditioned_feedback_and_other_modes_disable_window(self):
        launch = Path(__file__).resolve().parents[1] / 'launch/exp0_real.launch'
        for arguments, window, mppi_present in (
                ([], .05, True),
                (['use_mppi:=false'], 0., False),
                (['use_real_arm:=false'], 0., True)):
            with self.subTest(arguments=arguments):
                config = ROSLaunchConfig()
                # XML resolution does not launch nodes or contact a master.
                XmlLoader().load(str(launch), config,
                                argv=['hardware_enabled:=false'] + arguments,
                                verbose=False)
                self.assertEqual(config.params['/filter_joint_states/velocity_window_s'].value,
                                 window)
                mppi = [node for node in config.nodes if node.name == 'whole_body_mppi']
                self.assertEqual(bool(mppi), mppi_present)
                if mppi:
                    self.assertIn(('/joint_states_single', '/mm/joint_states'),
                                  [tuple(remap) for remap in mppi[0].remap_args])


if __name__ == '__main__':
    unittest.main()
