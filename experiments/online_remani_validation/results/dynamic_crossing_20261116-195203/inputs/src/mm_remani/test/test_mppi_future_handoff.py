"""Future-splice protocol behavior with the real adapter; no ROS master.

These tests cover reference continuity and rejection at the transport boundary,
not physical tracking or whole-robot collision safety.
"""
import unittest

import numpy as np

import test_mppi_native_phase as phases
from native_reference_adapter import NativeReferenceAdapter


class FutureHandoffTests(unittest.TestCase):
    def setUp(self):
        self.standby, self.navigation = phases.Kernel(), phases.Kernel()
        self.adapter = NativeReferenceAdapter(self.standby, self.navigation)
        self.adapter.feed_goal(phases.goal(), 10.01)
        self.adapter.feed_trajectory(phases.trajectory(), 10.11)
        self.adapter.feed_progress(phases.progress(20., phase=.2, scale=.5), 20.)

    def plan(self, now):
        return self.adapter.plan(np.zeros(17), np.zeros((1, 4)),
                                 np.zeros(6), None, now)

    def future(self, receipt=20.2):
        # Exactly the endpoint predicted from the frozen old phase/scale at
        # epoch 20.0 for activation 20.7; base and arm share this same epoch.
        endpoint = .2+.5*.7
        self.adapter.feed_trajectory(phases.trajectory(
            start=20.7, x=1.+.2*endpoint, arm_offset=endpoint), receipt)

    def test_old_base_and_arm_continue_until_one_atomic_future_activation(self):
        self.future()
        revision = self.adapter.revision
        for now in (20.21, 20.31, 20.41, 20.51, 20.61):
            self.adapter.feed_progress(phases.progress(
                now, phase=0., scale=0., start=20.7), now)
            result, info = self.plan(now)
            self.assertIsNotNone(result)
            self.assertEqual(self.adapter.revision, revision)
            self.assertEqual(info['source_stamp'], 10.1)
            phase = .2+.5*(now-20.)
            path, arm = self.navigation.calls[-1]
            self.assertAlmostEqual(path[0, 0], 1.+.2*phase)
            self.assertAlmostEqual(arm[0, 0], phase+.5*.1)
        self.adapter.feed_progress(phases.progress(
            20.7, phase=0., scale=1., start=20.7), 20.7)
        result, info = self.plan(20.7)
        self.assertIsNotNone(result)
        self.assertEqual(info['source_stamp'], 20.7)
        self.assertEqual(self.adapter.revision, revision+1)
        path, arm = self.navigation.calls[-1]
        self.assertAlmostEqual(path[0, 0], 1.+.2*.55)
        self.assertAlmostEqual(arm[0, 0], .55+.1)
        self.assertIsNone(self.adapter.pending)

    def test_twenty_ms_delivery_lead_cannot_bypass_existing_thirty_ms_quiet_gate(self):
        self.future(receipt=20.68)
        self.adapter.feed_progress(phases.progress(
            20.7, phase=0., scale=1., start=20.7), 20.7)
        result, info = self.plan(20.7)
        self.assertIsNone(result)
        self.assertEqual(info['reference_status'], 'native_reference_quiet_interval')
        self.assertEqual(self.adapter.native.start, 10.1)
        self.assertIsNotNone(self.adapter.pending)
        result, info = self.plan(20.72)
        self.assertIsNotNone(result)
        self.assertEqual(info['source_stamp'], 20.7)

    def test_appended_future_piece_restarts_quiet_gate_without_mutating_old_arm(self):
        self.future()
        self.adapter.feed_trajectory(phases.trajectory(
            start=20.7, ident=2, x=1.+.2*.55+.4, arm_offset=2.55), 20.69)
        self.assertEqual(self.adapter.native.last_id, 1)
        self.assertEqual(self.adapter.pending['native'].last_id, 2)
        self.adapter.feed_progress(phases.progress(
            20.7, phase=0., scale=1., start=20.7), 20.7)
        result, info = self.plan(20.7)
        self.assertIsNone(result)
        self.assertEqual(info['reference_status'], 'native_reference_quiet_interval')
        self.assertEqual(self.adapter.native.start, 10.1)
        result, info = self.plan(20.73)
        self.assertIsNotNone(result)
        self.assertEqual(self.adapter.native.last_id, 2)
        self.assertEqual(info['source_stamp'], 20.7)

    def test_old_identity_progress_cannot_authorize_future_reference(self):
        self.future()
        self.adapter.feed_progress(phases.progress(
            20.7, phase=.55, scale=.5, start=10.1), 20.7)
        result, info = self.plan(20.7)
        self.assertIsNone(result)
        self.assertEqual(info['reference_status'], 'waiting_native_progress')
        self.assertEqual(self.adapter.native.start, 10.1)

    def test_abort_removes_old_and_future_and_delayed_progress_cannot_revive_them(self):
        self.future()
        stop = phases.trajectory()
        stop.action = 2
        self.assertEqual(self.adapter.feed_trajectory(stop, 20.4), 'cancel')
        self.assertIsNone(self.adapter.pending)
        self.assertFalse(self.adapter.native.refs)
        self.assertEqual(self.adapter.feed_progress(phases.progress(
            20.7, phase=0., scale=1., start=20.7), 20.7),
            'ignored_without_native_reference')
        result, info = self.plan(20.71)
        self.assertIsNone(result)
        self.assertEqual(info['reference_status'], 'native_reference_cancelled')


if __name__ == '__main__':
    unittest.main(verbosity=2)
