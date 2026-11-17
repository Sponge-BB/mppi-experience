"""Gate evidence and diagnostic locking preserve the real delivery decisions."""
import json
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np
import test_mppi_deadline_priority as deadline_fixture
from whole_body_mppi import checked_input_evidence, measured_delivery_lock


class LatencyEvidenceTests(unittest.TestCase):
    def test_gate_header_is_frozen_and_age_failures_are_distinct(self):
        h = np.zeros(32); h[2] = 42; h[3] = 99.9; h[4] = 1
        h[15] = .35; h[18] = 99.5; h[19] = h[28] = 1
        node = NS(base_stamp=99.8, joint_stamp=99.99, snapshot=NS(header=h))
        evidence = checked_input_evidence(node, 'before_delivery', 100.)
        h[2] = 43; h[3] = 100.
        self.assertEqual(evidence['snapshot_header'][2], 42.)
        self.assertAlmostEqual(evidence['map_age_ms'], 100.)
        self.assertTrue(evidence['map_age_ok'])
        self.assertFalse(evidence['base_fresh'])
        self.assertTrue(evidence['joint_fresh'])
        self.assertFalse(evidence['dynamic_age_ok'])

    def test_lock_duration_survives_rejection_and_releases_both_locks(self):
        node = NS(lock=threading.Lock(), reference=NS(lock=threading.Lock()))
        result = {}
        with patch('whole_body_mppi.time.monotonic', side_effect=[1., 1.004, 1.010]):
            with self.assertRaisesRegex(ValueError, 'rejected'):
                with measured_delivery_lock(node, result):
                    raise ValueError('rejected')
        self.assertEqual(result, dict(delivery_lock_wait_ms=4., delivery_lock_hold_ms=6.))
        self.assertFalse(node.lock.locked()); self.assertFalse(node.reference.lock.locked())

    def test_real_cycle_retains_final_gate_and_existing_deadline_rejection(self):
        f = deadline_fixture.DeadlinePriorityTests(methodName='runTest'); f.setUp()
        f.check_seconds = .060
        result = f.cycle()
        self.assertEqual(result['state'], 'delivery_deadline_exceeded')
        d = result['candidate_diagnostics']
        self.assertEqual(d['capture_inputs']['stage'], 'capture')
        self.assertEqual(d['checked_inputs']['stage'], 'before_delivery')
        self.assertEqual(d['delivery_lock_hold_ms'], 60.)
        self.assertEqual(d['first_step_check_ms'], 60.)
        f.assert_zero_without_arm()

    def test_initial_expired_snapshot_has_gate_evidence_before_any_solve(self):
        f = deadline_fixture.DeadlinePriorityTests(methodName='runTest'); f.setUp()
        f.node.snapshot = NS(fresh=lambda now: False)
        result = f.cycle()
        self.assertEqual(result['state'], 'collision_snapshot_unavailable')
        self.assertEqual(result['candidate_diagnostics']['checked_inputs']['stage'], 'capture')
        f.node.reference.plan.assert_not_called()
        self.assertNotIn('delivery_lock_hold_ms', result['candidate_diagnostics'])
        f.assert_zero_without_arm()


if __name__ == '__main__':
    unittest.main()
