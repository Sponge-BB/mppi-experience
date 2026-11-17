import json
from pathlib import Path
import sys
import unittest

RELEASE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RELEASE/"runtime"))
from piper_command_pair import PiperCommandPair
from piper_lease_gate import JOINT_NAMES, PiperLeaseGate
from test_piper_lease_gate import FakeSDK


class PairTests(unittest.TestCase):
    def setUp(self):
        self.sdk = FakeSDK()
        self.q = (0., 1., -1., 0., 0., 0.)
        self.gate = PiperLeaseGate(self.sdk, allow_motion=True)
        self.pair = PiperCommandPair(self.gate)
        self.gate.update_feedback(JOINT_NAMES, self.q, 1010., 10., 1010.)
        self.assertTrue(self.pair.enable(True, 10., 1010.))

    def meta(self, seq=1, stamp=1010.01, expiry=1010.09, t=10.02):
        return self.pair.lease(json.dumps(dict(sequence=seq, source_stamp=stamp,
                                               expires_at=expiry)), t, 1000+t)

    def joint(self, seq=1, stamp=1010.01, t=10.02, **kwargs):
        return self.pair.joint(JOINT_NAMES, self.q, seq, stamp, t, 1000+t, **kwargs)

    def test_metadata_then_joint_executes_once(self):
        self.assertFalse(self.meta())
        self.assertTrue(self.joint(t=10.03))
        self.assertEqual([x[0] for x in self.sdk.calls], ["EnableArm", "MotionCtrl_2", "JointCtrl"])
        self.assertAlmostEqual(self.gate.lease_expiry, 10.09)

    def test_joint_then_metadata_executes_once(self):
        self.assertFalse(self.joint())
        self.assertTrue(self.meta(t=10.03))

    def test_duplicate_partial_does_not_renew_pair_timeout(self):
        self.meta()
        self.assertFalse(self.meta(t=10.06))
        self.assertFalse(self.pair.tick(10.071, 1010.071))
        self.assertEqual(self.gate.reason, "command_pair_timeout")

    def test_watchdog_rejects_unpaired_joint(self):
        self.joint()
        self.assertFalse(self.pair.tick(10.071, 1010.071))
        self.assertFalse(self.gate.enabled)
        self.assertFalse(self.meta(t=10.072))

    def test_pending_metadata_expiry_is_not_extended_to_pair_ttl(self):
        self.meta(expiry=1010.03)
        self.assertFalse(self.pair.tick(10.031, 1010.031))
        self.assertEqual(self.gate.reason, "command_pair_expired")

    def test_mismatched_stamp_latches_stop(self):
        self.meta()
        self.assertFalse(self.joint(stamp=1010.011))
        self.assertEqual(self.gate.reason, "command_pair_stamp_mismatch")

    def test_conflicting_metadata_latches_stop(self):
        self.meta()
        self.assertFalse(self.meta(expiry=1010.10))
        self.assertEqual(self.gate.reason, "command_pair_conflict")

    def test_replayed_complete_command_does_not_execute_again(self):
        self.meta()
        self.joint()
        self.assertFalse(self.meta(t=10.03))
        self.assertEqual(len([c for c in self.sdk.calls if c[0] == "JointCtrl"]), 1)
        self.assertFalse(self.gate.enabled)

    def test_bounded_cache_closes_on_overflow(self):
        for seq in range(1, 5):
            self.meta(seq=seq)
        self.assertEqual(len(self.pair.pending), 4)
        self.assertFalse(self.meta(seq=5))
        self.assertEqual(self.gate.reason, "command_pair_capacity")
        self.assertEqual(self.pair.pending, {})

    def test_reenable_discards_old_pending_half(self):
        self.joint()
        self.pair.enable(False, 10.03, 1010.03)
        self.assertTrue(self.pair.enable(True, 10.04, 1010.04))
        self.assertFalse(self.meta(seq=2, stamp=1010.05, t=10.05))
        self.assertTrue(self.joint(seq=2, stamp=1010.05, t=10.05))

    def test_nonempty_velocity_or_effort_is_not_a_position_command(self):
        self.assertFalse(self.joint(velocities=[0.]*6))
        self.assertEqual(self.gate.reason, "command_pair_joint_invalid")

    def test_metadata_malformed_is_not_an_executable_command(self):
        for text in ('{}', '[]', '{"sequence":1}', 'x'*513,
                     '{"sequence":1,"source_stamp":1010.01,"expires_at":NaN}'):
            self.setUp()
            self.assertFalse(self.pair.lease(text, 10.02, 1010.02))
            self.assertFalse(self.gate.enabled)
            self.assertFalse(any(c[0] == "JointCtrl" for c in self.sdk.calls))


if __name__ == "__main__":
    unittest.main()
