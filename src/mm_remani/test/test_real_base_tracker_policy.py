#!/usr/bin/env python3

import os
import sys
import unittest


SCRIPT_DIRECTORY = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "scripts"))
sys.path.insert(0, SCRIPT_DIRECTORY)

from real_base_tracker import RealBaseTracker  # noqa: E402


class RealBaseTrackerPolicyTest(unittest.TestCase):
    def test_observed_turn_boundary_is_alignment_not_fault(self):
        turn_alignment = RealBaseTracker.is_turn_alignment(
            0.018, 0.05, 0.510, 0.10, 0.08, 0.10)
        self.assertTrue(turn_alignment)
        self.assertFalse(RealBaseTracker.tracking_error_is_unsafe(
            0.018, 0.510, turn_alignment, 0.18, 0.50))

    def test_large_yaw_error_while_translating_remains_fault(self):
        turn_alignment = RealBaseTracker.is_turn_alignment(
            0.018, 0.20, 0.510, 0.10, 0.08, 0.10)
        self.assertFalse(turn_alignment)
        self.assertTrue(RealBaseTracker.tracking_error_is_unsafe(
            0.018, 0.510, turn_alignment, 0.18, 0.50))

    def test_position_error_is_never_hidden_by_turn_alignment(self):
        turn_alignment = RealBaseTracker.is_turn_alignment(
            0.19, 0.0, 1.0, 0.10, 0.08, 0.10)
        self.assertFalse(turn_alignment)
        self.assertTrue(RealBaseTracker.tracking_error_is_unsafe(
            0.19, 1.0, turn_alignment, 0.18, 0.50))

    def test_small_normal_tracking_error_needs_no_alignment(self):
        turn_alignment = RealBaseTracker.is_turn_alignment(
            0.02, 0.05, 0.05, 0.10, 0.08, 0.10)
        self.assertFalse(turn_alignment)
        self.assertFalse(RealBaseTracker.tracking_error_is_unsafe(
            0.02, 0.05, turn_alignment, 0.18, 0.50))

    def test_active_alignment_ignores_entry_speed_chatter(self):
        self.assertTrue(RealBaseTracker.update_turn_alignment_state(
            True, 0.07, 0.12, 0.60,
            0.10, 0.08, 0.10, 0.07, 0.18))

    def test_active_alignment_exits_after_yaw_converges(self):
        self.assertFalse(RealBaseTracker.update_turn_alignment_state(
            True, 0.07, 0.12, 0.06,
            0.10, 0.08, 0.10, 0.07, 0.18))

    def test_active_alignment_never_masks_position_stop(self):
        turn_alignment = RealBaseTracker.update_turn_alignment_state(
            True, 0.19, 0.0, 0.80,
            0.10, 0.08, 0.10, 0.07, 0.18)
        self.assertFalse(turn_alignment)
        self.assertTrue(RealBaseTracker.tracking_error_is_unsafe(
            0.19, 0.80, turn_alignment, 0.18, 0.50))

    def test_alignment_command_has_deadzone_floor_and_cap(self):
        self.assertAlmostEqual(
            RealBaseTracker.bounded_alignment_angular_command(
                0.08, 1.0, 0.12, 0.45),
            0.12)
        self.assertAlmostEqual(
            RealBaseTracker.bounded_alignment_angular_command(
                -1.0, 1.0, 0.12, 0.45),
            -0.45)

    def test_soft_tracking_error_requires_consecutive_cycles(self):
        count = 0
        for _ in range(4):
            count = RealBaseTracker.update_tracking_unsafe_count(count, True)
            self.assertFalse(RealBaseTracker.tracking_fault_should_latch(
                True, False, count, 5))
        count = RealBaseTracker.update_tracking_unsafe_count(count, True)
        self.assertTrue(RealBaseTracker.tracking_fault_should_latch(
            True, False, count, 5))

    def test_safe_cycle_clears_soft_fault_confirmation(self):
        count = RealBaseTracker.update_tracking_unsafe_count(3, False)
        self.assertEqual(count, 0)
        self.assertFalse(RealBaseTracker.tracking_fault_should_latch(
            False, False, count, 5))

    def test_hard_tracking_error_latches_immediately(self):
        self.assertTrue(RealBaseTracker.tracking_fault_should_latch(
            True, True, 1, 5))


if __name__ == "__main__":
    unittest.main()
