"""Fault-boundary checks; no ROS or hardware required."""

import importlib.util
from pathlib import Path
import sys
import unittest


MODULE = Path(__file__).resolve().parents[1] / "runtime/hardware_command_guard.py"
SPEC = importlib.util.spec_from_file_location("hardware_command_guard_tested", str(MODULE))
guard_module = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = guard_module
SPEC.loader.exec_module(guard_module)
GuardConfig = guard_module.GuardConfig
HardwareCommandGuard = guard_module.HardwareCommandGuard


class HardwareCommandGuardTests(unittest.TestCase):
    def setUp(self):
        self.config = GuardConfig(joint_lower=(-2.,) * 6,
                                  joint_upper=(2.,) * 6)
        self.guard = HardwareCommandGuard(self.config)
        self.zero = (0.,) * 6

    def arm(self, q=None, now=10.):
        self.assertTrue(self.guard.update_measurement(q or self.zero, self.zero, now))
        self.assertTrue(self.guard.set_enabled(True, now))

    def send(self, sequence=1, stamp=10., lease=.25, now=None, controls=None):
        return self.guard.accept_command(
            controls if controls is not None else (.05, .10, .10, 0., 0., 0., 0., 0.),
            sequence, stamp, lease, stamp if now is None else now)

    def assert_stopped(self, output, reason):
        self.assertFalse(output.actuate)
        self.assertTrue(output.stop_required)
        self.assertEqual(output.reason, reason)
        self.assertEqual((output.base_v, output.base_w), (0., 0.))
        self.assertEqual(output.joint_velocities, self.zero)

    def test_default_disabled_never_accepts_or_authorizes_motion(self):
        self.guard.update_measurement(self.zero, self.zero, 10.)
        self.assertFalse(self.send())
        output = self.guard.tick(10.01)
        self.assertFalse(output.actuate)
        self.assertFalse(output.stop_required)
        self.assertEqual(output.reason, "disabled")
        self.assertEqual(output.joint_positions, self.zero)

    def test_enabling_requires_fresh_feedback_and_a_new_command(self):
        self.assertFalse(self.guard.set_enabled(True, 10.))
        self.arm(now=10.01)
        output = self.guard.tick(10.02)
        self.assertFalse(output.actuate)
        self.assertEqual(output.reason, "awaiting_command")
        self.assertTrue(self.send(stamp=10.02))
        self.assertTrue(self.guard.tick(10.04).actuate)
        self.guard.set_enabled(False, 10.05)
        self.guard.set_enabled(True, 10.05)
        self.assertFalse(self.guard.tick(10.06).actuate)

    def test_tick_does_not_renew_lease_or_use_unapproved_suffix(self):
        self.arm()
        self.assertTrue(self.send(lease=.10))
        first = self.guard.tick(10.02)
        second = self.guard.tick(10.04)
        self.assertAlmostEqual(first.lease_remaining, .08)
        self.assertAlmostEqual(second.lease_remaining, .06)
        self.guard.update_measurement(self.zero, self.zero, 10.10)
        output = self.guard.tick(10.10)
        self.assert_stopped(output, "command_expired")
        self.assertFalse(self.send(sequence=2, stamp=10.10))
        self.assertFalse(self.guard.enabled)

    def test_lease_counts_from_source_stamp_not_late_receipt(self):
        self.arm()
        self.assertTrue(self.send(stamp=10., lease=.10, now=10.08))
        output = self.guard.tick(10.09)
        self.assertAlmostEqual(output.lease_remaining, .01)
        self.guard.update_measurement(self.zero, self.zero, 10.101)
        self.assert_stopped(self.guard.tick(10.101), "command_expired")

    def test_replayed_sequence_revokes_existing_motion(self):
        self.arm()
        self.assertTrue(self.send(sequence=4))
        self.guard.tick(10.02)
        self.assertFalse(self.send(sequence=4, stamp=10.03))
        self.assert_stopped(self.guard.tick(10.04), "command_sequence_rejected")
        self.assertTrue(self.guard.set_enabled(True, 10.05))
        self.assertFalse(self.send(sequence=3, stamp=10.05))

    def test_source_stamp_and_lease_validation_fail_closed(self):
        cases = [(9.8, .25, "command_stale"),
                 (10.02, .25, "command_from_future"),
                 (10., .26, "command_lease_rejected"),
                 (10., 0., "command_lease_rejected")]
        for stamp, lease, reason in cases:
            with self.subTest(reason=reason, lease=lease):
                self.guard = HardwareCommandGuard(self.config)
                self.arm()
                self.assertFalse(self.send(stamp=stamp, lease=lease, now=10.))
                self.assert_stopped(self.guard.tick(10.01), reason)

    def test_feedback_expiry_stops_even_with_longer_valid_command(self):
        self.arm()
        self.assertTrue(self.send())
        self.assertTrue(self.guard.tick(10.02).actuate)
        output = self.guard.tick(10.151)
        self.assert_stopped(output, "feedback_stale")
        self.assertEqual(output.joint_positions, self.zero)

    def test_old_feedback_cannot_be_refreshed_by_new_receipt_time(self):
        self.arm()
        self.assertTrue(self.send())
        self.guard.update_measurement(self.zero, self.zero, 10.20, stamp=10.)
        self.assert_stopped(self.guard.tick(10.20), "feedback_stale")

    def test_position_goal_uses_measurement_anchor_and_bounded_lead(self):
        self.arm()
        self.assertTrue(self.send(controls=(100., -100.) + (100.,) * 6))
        outputs = []
        for index in range(1, 7):
            now = 10. + index * .02
            self.guard.update_measurement(self.zero, self.zero, now)
            outputs.append(self.guard.tick(now))
        for previous, current in zip(outputs, outputs[1:]):
            self.assertLessEqual(current.base_v, self.config.max_base_v)
            self.assertLessEqual(abs(current.base_w), self.config.max_base_w)
            self.assertLessEqual(current.base_v - previous.base_v, .10 * .02 + 1e-12)
            self.assertLessEqual(current.joint_velocities[0] - previous.joint_velocities[0],
                                 .35 * .02 + 1e-12)
        last = outputs[-1]
        self.assertAlmostEqual(last.joint_positions[0],
                               last.joint_velocities[0] * self.config.position_horizon)
        self.assertLessEqual(abs(last.joint_positions[0]), self.config.max_position_lead)
        # Measured arm stayed still: accumulated previous goals are NOT added.
        self.assertLess(last.joint_positions[0],
                        sum(item.joint_positions[0] for item in outputs))

    def test_joint6_effective_bound_and_stopping_option(self):
        self.assertEqual(self.config.joint_lower[5], -.35)
        self.assertEqual(self.config.joint_upper[5], .35)
        q = self.zero[:5] + (.34999,)
        self.arm(q=q)
        self.assertTrue(self.send(controls=(0.,) * 7 + (100.,)))
        output = self.guard.tick(10.02)
        self.assertLessEqual(output.joint_positions[5], .35)
        self.assertLessEqual(output.joint_velocities[5] ** 2 / (2 * .35),
                             .35 - q[5] + 1e-12)

    def test_measured_joint_limit_violation_cannot_be_reenabled(self):
        self.arm()
        self.assertTrue(self.send())
        bad_q = self.zero[:5] + (.351,)
        self.assertFalse(self.guard.update_measurement(bad_q, self.zero, 10.02))
        self.assert_stopped(self.guard.tick(10.02), "joint_position_limit")
        self.assertEqual(self.guard.tick(10.02).joint_positions, bad_q)
        self.assertFalse(self.guard.set_enabled(True, 10.02))

    def test_measured_stopping_violation_cannot_be_cleared_by_enable(self):
        q = self.zero[:5] + (.349,)
        dq = self.zero[:5] + (.10,)
        self.assertFalse(self.guard.update_measurement(q, dq, 10.))
        self.assertFalse(self.guard.set_enabled(True, 10.))
        self.assert_stopped(self.guard.tick(10.), "joint_stopping_limit")

    def test_unexpected_joint_tracking_error_stops_with_actual_hold(self):
        self.arm()
        self.assertTrue(self.send())
        self.guard.tick(10.02)
        measured = (.20,) + self.zero[1:]
        self.assertFalse(self.guard.update_measurement(measured, self.zero, 10.04))
        output = self.guard.tick(10.04)
        self.assert_stopped(output, "joint_tracking_error")
        self.assertEqual(output.joint_positions, measured)

    def test_stalled_timer_cannot_make_large_acceleration_step(self):
        self.arm()
        self.assertTrue(self.send())
        self.guard.update_measurement(self.zero, self.zero, 10.14)
        output = self.guard.tick(10.14)
        self.assertLessEqual(output.base_v, .10 * self.config.max_tick_step)
        self.assertLessEqual(output.joint_velocities[0], .35 * self.config.max_tick_step)

    def test_nonfinite_or_rollout_command_is_rejected(self):
        for command in [(float("nan"),) + (0.,) * 7,
                        [(.01,) * 8] * 20]:
            self.guard = HardwareCommandGuard(self.config)
            self.arm()
            self.assertFalse(self.send(controls=command))
            self.assert_stopped(self.guard.tick(10.01), "invalid_command")

    def test_clock_regression_revokes_motion(self):
        self.arm()
        self.assertTrue(self.send())
        self.guard.tick(10.02)
        self.assert_stopped(self.guard.tick(10.01), "clock_regressed")

    def test_invalid_clock_fails_closed_without_throwing(self):
        self.arm()
        self.assertTrue(self.send())
        self.assert_stopped(self.guard.tick(float("nan")), "invalid_clock")

    def test_enable_requires_actual_boolean(self):
        self.guard.update_measurement(self.zero, self.zero, 10.)
        self.assertFalse(self.guard.set_enabled("false", 10.))
        self.assert_stopped(self.guard.tick(10.01), "invalid_enable")


if __name__ == "__main__":
    unittest.main()
