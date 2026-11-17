"""Synthetic integration acceptance; no ROS, SDK connection or device access."""

from dataclasses import replace
from pathlib import Path
import sys
import unittest

RELEASE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RELEASE / "tools"))
from run_offline_validation import (FakeClock, JOINT_NAMES, MppiCommandGateway,
                                    NativeLeaseValidator, PiperLeaseGate,
                                    Q_START, RecordingSDK, RecordingSink,
                                    make_envelopes, make_fixture)
import numpy as np


class OfflineMppiPipelineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        (cls.model, cls.config, cls.kernel, cls.snapshot,
         cls.state, cls.reference) = make_fixture()
        cls.planned, cls.diagnostics = cls.kernel.plan(
            cls.state, cls.reference, Q_START, cls.snapshot, 1000.05)
        if cls.planned is None:
            raise AssertionError("synthetic moving-obstacle fixture has no MPPI plan")

    def setUp(self):
        self.clock = FakeClock()
        self.sink = RecordingSink()
        self.gateway = MppiCommandGateway(self.config, NativeLeaseValidator(self.kernel),
                                           self.sink, self.clock)
        self.assertTrue(self.gateway.update_feedback(tuple(self.state), 1000.))
        self.cohort, self.plan = make_envelopes(self.state, self.planned[0][0])

    def enable(self):
        self.assertTrue(self.gateway.set_enabled(True))
        self.clock.advance(.05)

    def test_actual_mppi_preserves_all_eight_coordinates_and_base_candidates(self):
        self.assertEqual(self.diagnostics["sample_dimensions"], 8)
        self.assertTrue(np.all(np.asarray(self.diagnostics["noise_rms_per_coordinate"]) > 0.))
        controls = np.asarray(self.diagnostics["structured_first_base_commands"])
        self.assertTrue(np.any(controls[:, 0] > 0.))
        self.assertTrue(np.any(controls[:, 0] < 0.))
        self.assertTrue(np.any(controls[:, 1] > 0.))
        self.assertTrue(np.any(controls[:, 1] < 0.))
        self.assertGreater(self.diagnostics["arm_motion"], .1)
        self.assertEqual(len(self.model.body_spheres(Q_START)), 67)

    def test_default_disabled_emits_no_command(self):
        self.clock.advance(.05)
        self.assertFalse(self.gateway.submit(self.plan, self.cohort, self.snapshot))
        self.assertEqual(self.sink.packets, [])

    def test_real_mppi_to_guard_to_mock_sdk_preserves_shorter_input_expiry(self):
        sdk = RecordingSDK()
        driver = PiperLeaseGate(sdk, allow_motion=True)
        self.assertTrue(driver.update_feedback(JOINT_NAMES, tuple(Q_START),
                                               1000., 10., 1000.))
        self.assertTrue(driver.enable(True, 10., 1000.))
        self.enable()
        self.assertTrue(self.gateway.submit(self.plan, self.cohort, self.snapshot))
        self.assertEqual(len(self.sink.packets), 1)
        packet = self.sink.packets[0]
        self.assertAlmostEqual(packet.expires_at, 1000.12)
        self.assertAlmostEqual(packet.source_stamp, 1000.05)
        self.assertLessEqual(max(abs(x) for x in packet.joint_velocities), .0175+1e-9)
        self.assertGreater(max(abs(x) for x in packet.joint_velocities), 0.)
        goal_delta = np.asarray(packet.joint_positions)-Q_START
        np.testing.assert_allclose(goal_delta, np.asarray(packet.joint_velocities)*.07,
                                   atol=1e-10)
        self.assertTrue(driver.command(JOINT_NAMES, packet.joint_positions,
                                       packet.sequence, packet.source_stamp,
                                       self.clock.monotonic, self.clock.wall,
                                       expires_at=packet.expires_at))
        expected = tuple(round(q*1000*180/np.pi) for q in packet.joint_positions)
        self.assertEqual(sdk.calls[-1], ("JointCtrl", expected))
        self.assertAlmostEqual(driver.lease_expiry, 10.12)
        self.clock.advance(.071)
        self.gateway.tick()
        self.assertFalse(driver.tick(self.clock.monotonic, self.clock.wall))
        self.assertEqual(sdk.calls[-1][0], "MotionCtrl_1")
        self.assertTrue(self.sink.stops)

    def test_watchdog_never_republishes_or_renews_old_plan(self):
        self.enable()
        self.assertTrue(self.gateway.submit(self.plan, self.cohort, self.snapshot))
        expiry = self.sink.packets[0].expires_at
        for _ in range(3):
            self.clock.advance(.01)
            self.gateway.tick()
        self.assertEqual(len(self.sink.packets), 1)
        self.assertEqual(self.sink.packets[0].expires_at, expiry)

    def test_recheck_uses_hardware_acceleration_without_mutating_planner(self):
        original_limits = dict(self.kernel.limits)
        original_acceleration = self.kernel.amax.copy()
        original_velocity = self.kernel.vmax.copy()
        validator = NativeLeaseValidator(self.kernel, config=self.config)
        self.assertIsNot(validator.kernel, self.kernel)
        self.assertLessEqual(validator.kernel.limits["accel"],
                             self.config.max_base_acceleration)
        self.assertLessEqual(validator.kernel.limits["angular_accel"],
                             self.config.max_base_angular_acceleration)
        self.assertTrue(np.all(validator.kernel.amax <= self.config.joint_acceleration))
        self.assertTrue(np.all(validator.kernel.vmax <= self.config.joint_velocity))
        self.assertEqual(self.kernel.limits, original_limits)
        np.testing.assert_array_equal(self.kernel.amax, original_acceleration)
        np.testing.assert_array_equal(self.kernel.vmax, original_velocity)

    def test_validation_completion_cannot_renew_an_expired_command(self):
        def delayed_validator(state, control, snapshot, wall, remaining):
            self.clock.advance(.071)
            return True

        self.gateway = MppiCommandGateway(self.config, delayed_validator,
                                           self.sink, self.clock)
        self.assertTrue(self.gateway.update_feedback(tuple(self.state), 1000.))
        self.enable()
        self.assertFalse(self.gateway.submit(self.plan, self.cohort, self.snapshot))
        self.assertEqual(self.sink.packets, [])
        self.assertTrue(self.sink.stops)

    def test_sequence_replay_stops_without_second_packet(self):
        self.enable()
        self.assertTrue(self.gateway.submit(self.plan, self.cohort, self.snapshot))
        self.clock.advance(.001)
        self.assertFalse(self.gateway.submit(self.plan, self.cohort, self.snapshot))
        self.assertEqual(len(self.sink.packets), 1)
        self.assertTrue(self.sink.stops)

    def test_dead_input_cohort_cannot_be_renewed_by_a_new_plan_stamp(self):
        self.enable()
        stale = replace(self.cohort, expires_at=1000.04)
        self.assertFalse(self.gateway.submit(self.plan, stale, self.snapshot))
        self.assertEqual(self.sink.packets, [])

    def test_wrong_cohort_and_frame_are_rejected_before_publish(self):
        self.enable()
        wrong = replace(self.cohort, cohort_id="another-observation", frame_id="lidar")
        self.assertFalse(self.gateway.submit(self.plan, wrong, self.snapshot))
        self.assertEqual(self.sink.packets, [])

    def test_validator_sees_bounded_control_and_rejection_latches(self):
        observed = []

        def rejecting_validator(state, control, snapshot, wall, remaining):
            observed.append((tuple(control), remaining))
            return False

        self.gateway = MppiCommandGateway(self.config, rejecting_validator,
                                           self.sink, self.clock)
        self.assertTrue(self.gateway.update_feedback(tuple(self.state), 1000.))
        self.enable()
        self.assertFalse(self.gateway.submit(self.plan, self.cohort, self.snapshot))
        self.assertEqual(len(observed), 1)
        command, remaining = observed[0]
        self.assertLessEqual(abs(command[0]), .005+1e-9)
        self.assertLessEqual(abs(command[1]), .015+1e-9)
        self.assertLessEqual(max(abs(x) for x in command[2:]), .0175+1e-9)
        self.assertAlmostEqual(remaining, .07)
        self.assertEqual(self.sink.packets, [])
        self.assertTrue(self.sink.stops)
        # A new plan cannot silently clear a collision rejection.
        self.assertFalse(self.gateway.submit(replace(self.plan, sequence=2),
                                             self.cohort, self.snapshot))

    def test_nonzero_base_command_uses_standard_twist_fields(self):
        self.enable()
        # Transport semantics are tested independently of which base action the
        # synthetic arm-only-clearance planner happens to prefer.
        plan = replace(self.plan, control=(.04, -.10, 0., 0., 0., 0., 0., 0.))
        self.assertTrue(self.gateway.submit(plan, self.cohort, self.snapshot))
        packet = self.sink.packets[0]
        self.assertGreater(packet.base_v, 0.)
        self.assertLess(packet.base_w, 0.)
        twist = packet.twist()
        self.assertEqual(tuple(twist["linear"]), (packet.base_v, 0., 0.))
        self.assertEqual(tuple(twist["angular"]), (0., 0., packet.base_w))

    def test_native_validator_rejects_collision_in_measured_initial_state(self):
        from scipy.spatial import cKDTree
        from active_arm_mppi import Snapshot
        center = self.kernel.spheres(self.state[None])[0, -1, :3]
        blocked = Snapshot(self.snapshot.header.copy(), self.snapshot.distance,
                           [(np.zeros(3), cKDTree(center[None]))])
        self.enable()
        self.assertFalse(self.gateway.submit(self.plan, self.cohort, blocked))
        self.assertEqual(self.sink.packets, [])
        self.assertTrue(self.sink.stops)


if __name__ == "__main__":
    unittest.main()
