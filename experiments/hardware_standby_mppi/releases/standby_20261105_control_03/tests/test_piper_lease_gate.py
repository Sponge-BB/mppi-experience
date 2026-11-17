import math
from pathlib import Path
import sys
import threading
import unittest
import xml.etree.ElementTree as ET

RELEASE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RELEASE/"runtime"))
from piper_lease_gate import JOINT_NAMES, PiperLeaseGate, open_existing_sdk


class FakeSDK:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.calls = []
        self.connect_kwargs = None
        self.fail_joint = False

    def ConnectPort(self, **kwargs):
        self.connect_kwargs = kwargs

    def EnableArm(self, *args):
        self.calls.append(("EnableArm", args))

    def MotionCtrl_1(self, *args):
        self.calls.append(("MotionCtrl_1", args))

    def MotionCtrl_2(self, *args):
        self.calls.append(("MotionCtrl_2", args))

    def JointCtrl(self, *args):
        if self.fail_joint:
            raise OSError("fake transport failed")
        self.calls.append(("JointCtrl", args))


class GateTests(unittest.TestCase):
    def setUp(self):
        self.sdk = FakeSDK()
        self.gate = PiperLeaseGate(self.sdk, allow_motion=True)
        self.q = [0., 1., -1., 0., 0., 0.]

    def feedback(self, t=10., stamp=None):
        return self.gate.update_feedback(JOINT_NAMES, self.q, 1000.+t if stamp is None else stamp,
                                         t, 1000.+t)

    def armed(self):
        self.assertTrue(self.feedback())
        self.assertTrue(self.gate.enable(True, 10., 1010.))

    def command(self, t=10.01, seq=1, stamp=None, q=None):
        return self.gate.command(JOINT_NAMES, self.q if q is None else q, seq,
                                 1000.+t if stamp is None else stamp, t, 1000.+t)

    def test_default_is_zero_transmit_even_with_sdk_and_valid_input(self):
        self.gate = PiperLeaseGate(self.sdk)
        self.assertTrue(self.feedback())
        self.assertFalse(self.gate.enable(True, 10., 1010.))
        self.assertFalse(self.command())
        self.gate.tick(11., 1011.)
        self.gate.stop("shutdown")
        self.assertEqual(self.sdk.calls, [])

    def test_sdk_startup_skips_protocol_initialization(self):
        sdk = open_existing_sdk(FakeSDK, "not_a_device")
        self.assertEqual(sdk.kwargs, dict(can_name="not_a_device"))
        self.assertEqual(sdk.connect_kwargs, dict(piper_init=False))
        self.assertEqual(sdk.calls, [])

    def test_never_enable_without_verified_fresh_feedback(self):
        self.assertFalse(self.gate.enable(True, 10., 1010.))
        self.assertEqual(self.sdk.calls, [])
        self.feedback()
        self.assertFalse(self.gate.enable(True, 10.16, 1010.16))
        self.assertEqual(self.sdk.calls, [])

    def test_original_conversion_and_sdk_calls_reused(self):
        self.armed()
        self.assertTrue(self.command())
        self.assertEqual(self.sdk.calls[1], ("MotionCtrl_2", (1, 1, 5, 0)))
        self.assertEqual(self.sdk.calls[2], ("JointCtrl", tuple(round(v*1000*180/math.pi) for v in self.q)))

    def test_timeout_latches_and_late_callback_cannot_restart(self):
        self.armed()
        self.assertTrue(self.command())
        self.feedback(10.12)
        self.assertFalse(self.gate.tick(10.17, 1010.17))
        calls = list(self.sdk.calls)
        self.assertEqual(calls[-1], ("MotionCtrl_1", (1, 0, 0)))
        self.assertFalse(self.command(10.18, seq=2))
        self.gate.tick(10.20, 1010.20)
        self.assertEqual(self.sdk.calls, calls)

    def test_late_callback_itself_checks_expiry_without_watchdog(self):
        self.armed()
        self.feedback(10.12)
        self.assertFalse(self.command(10.151))
        self.assertEqual(self.gate.reason, "command_timeout")
        self.assertEqual(self.sdk.calls[-1][0], "MotionCtrl_1")

    def test_source_stamp_lease_not_receipt_lease(self):
        self.armed()
        self.assertTrue(self.command(10.10, stamp=1010.01))
        self.feedback(10.11)
        self.assertAlmostEqual(self.gate.lease_expiry, 10.16)
        self.assertFalse(self.gate.tick(10.161, 1010.161))

    def test_repeated_enable_does_not_renew_lease(self):
        self.armed()
        self.assertFalse(self.gate.enable(True, 10.10, 1010.10))
        self.assertAlmostEqual(self.gate.lease_expiry, 10.15)
        self.assertEqual(len(self.sdk.calls), 1)

    def test_replay_sequence_stops_and_latches(self):
        self.armed()
        self.assertTrue(self.command())
        self.assertFalse(self.command(10.02, seq=1))
        self.assertEqual(self.gate.reason, "command_replay")
        self.assertFalse(self.gate.enabled)

    def test_reenable_does_not_accept_command_generated_before_enable(self):
        self.armed()
        self.command()
        self.gate.stop()
        self.feedback(10.03)
        self.assertTrue(self.gate.enable(True, 10.04, 1010.04))
        self.assertFalse(self.command(10.05, seq=2, stamp=1010.035))
        self.assertEqual(self.gate.reason, "command_before_enable_or_replayed")

    def test_reenable_keeps_sequence_floor(self):
        self.armed()
        self.command(seq=9)
        self.gate.stop()
        self.feedback(10.03)
        self.assertTrue(self.gate.enable(True, 10.04, 1010.04))
        self.assertFalse(self.command(10.05, seq=8))
        self.assertEqual(self.gate.reason, "command_replay")

    def test_reenable_allows_fresh_command_with_new_sequence(self):
        self.armed()
        self.command()
        self.gate.stop()
        self.feedback(10.03)
        self.assertTrue(self.gate.enable(True, 10.04, 1010.04))
        self.assertTrue(self.command(10.05, seq=2))

    def test_feedback_timeout_stops(self):
        self.armed()
        self.assertTrue(self.command(10.10))
        self.assertFalse(self.gate.tick(10.151, 1010.151))
        self.assertEqual(self.gate.reason, "feedback_timeout")

    def test_feedback_replay_cannot_freshen_measurement(self):
        self.armed()
        self.assertFalse(self.feedback(10.02, stamp=1010.))
        self.assertFalse(self.gate.enabled)
        self.assertIsNone(self.gate.feedback)

    def test_feedback_source_stamp_determines_freshness(self):
        self.assertTrue(self.feedback(10., stamp=1009.90))
        self.assertAlmostEqual(self.gate.feedback_expiry, 10.05)
        self.assertFalse(self.gate.enable(True, 10.06, 1010.06))

    def test_joint6_limit_and_position_lead(self):
        self.armed()
        target = self.q[:]
        target[5] = .36
        self.assertFalse(self.command(q=target))
        self.assertEqual(self.gate.reason, "position_limit")
        self.feedback(10.02)
        self.assertTrue(self.gate.enable(True, 10.03, 1010.03))
        target[5] = .031
        self.assertFalse(self.command(10.04, seq=2, q=target))
        self.assertEqual(self.gate.reason, "position_lead_limit")

    def test_invalid_or_future_command_fails_closed(self):
        self.armed()
        self.assertFalse(self.command(stamp=1010.10))
        self.assertEqual(self.gate.reason, "command_stamp_invalid")

    def test_invalid_feedback_fails_closed(self):
        self.armed()
        q = self.q[:]
        q[0] = math.nan
        self.assertFalse(self.gate.update_feedback(JOINT_NAMES, q, 1010.01, 10.01, 1010.01))
        self.assertFalse(self.gate.enabled)

    def test_joint_transport_error_stops_and_latches(self):
        self.armed()
        self.sdk.fail_joint = True
        self.assertFalse(self.command())
        self.assertFalse(self.gate.enabled)
        self.assertEqual(self.sdk.calls[-1], ("MotionCtrl_1", (1, 0, 0)))

    def test_clock_jump_stops(self):
        self.armed()
        self.assertFalse(self.gate.tick(10.01, 1011.01))
        self.assertEqual(self.gate.reason, "clock_offset_changed")

    def test_lock_prevents_stop_interleaving_mode_and_joint(self):
        self.armed()
        entered = threading.Event()
        release = threading.Event()
        original_mode = self.sdk.MotionCtrl_2
        def blocked_mode(*args):
            original_mode(*args)
            entered.set()
            self.assertTrue(release.wait(1.))
        self.sdk.MotionCtrl_2 = blocked_mode
        cmd = threading.Thread(target=self.command)
        cmd.start()
        self.assertTrue(entered.wait(1.))
        stop = threading.Thread(target=self.gate.stop)
        stop.start()
        release.set()
        cmd.join(1.)
        stop.join(1.)
        self.assertFalse(cmd.is_alive())
        self.assertFalse(stop.is_alive())
        self.assertEqual([x[0] for x in self.sdk.calls],
                         ["EnableArm", "MotionCtrl_2", "JointCtrl", "MotionCtrl_1"])

    def test_launch_disconnected_defaults_and_private_remaps(self):
        root = ET.parse(RELEASE/"config/reused_piper.launch").getroot()
        args = {a.attrib['name']: a.attrib['default'] for a in root.findall('arg')}
        self.assertEqual(args['connect_transport'], 'false')
        self.assertEqual(args['allow_motion'], 'false')
        remaps = root.findall('.//remap')
        self.assertTrue(remaps)
        self.assertTrue(all(r.attrib['to'].startswith('/standby_trial_20261105_') for r in remaps))


if __name__ == "__main__":
    unittest.main()
