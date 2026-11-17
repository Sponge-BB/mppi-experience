"""Offline protocol and receive-boundary checks.  Never touches a CAN socket."""

import ast
import importlib.util
import math
from pathlib import Path
import socket
import struct
import sys
import unittest
from unittest.mock import patch


RUNTIME = Path(__file__).resolve().parents[1] / "runtime"
sys.path.insert(0, str(RUNTIME))
import piper_feedback_decode as decode
import piper_can_receive_only as receiver


def payload(first=0, second=0):
    return struct.pack(">ii", first, second)


class PiperProtocolTests(unittest.TestCase):
    def test_exact_ids_units_sign_and_big_endian(self):
        for index, can_id in enumerate(decode.PAIR_IDS):
            pair, angles = decode.decode_joint_pair(can_id, payload(90000, -180000))
            self.assertEqual(pair, index)
            self.assertAlmostEqual(angles[0], math.pi / 2.)
            self.assertAlmostEqual(angles[1], -math.pi)

    def test_flags_control_ids_bad_dlc_and_corruption_are_rejected(self):
        for can_id, data in [(0x155, payload()),
                             (0x2A5 | decode.CAN_RTR_FLAG, payload()),
                             (0x2A5 | decode.CAN_EFF_FLAG, payload()),
                             (0x2A5 | decode.CAN_ERR_FLAG, payload()),
                             (0x2A5, b"\0" * 7),
                             (0x2A5, payload(2147483647, 0))]:
            with self.subTest(can_id=can_id, data=data):
                with self.assertRaises(ValueError):
                    decode.decode_joint_pair(can_id, data)

    def test_socketcan_classic_layout_requires_exact_frame_and_dlc(self):
        frame = decode.CLASSIC_CAN_FRAME.pack(0x2A6, 8, payload(-45000, 30000))
        can_id, index, angles = decode.decode_socketcan_frame(frame)
        self.assertEqual((can_id, index), (0x2A6, 1))
        self.assertAlmostEqual(angles[0], -math.pi / 4.)
        for bad in (frame[:-1], frame + b"\0",
                    decode.CLASSIC_CAN_FRAME.pack(0x2A6, 7, payload())):
            with self.assertRaises(ValueError):
                decode.decode_socketcan_frame(bad)

    def test_no_zero_filled_partial_state_or_repeat_freshness(self):
        assembler = decode.JointFeedbackAssembler()
        self.assertTrue(assembler.ingest(0x2A5, payload(1000, 2000), 10.))
        self.assertIsNone(assembler.snapshot(10.001))
        self.assertEqual(assembler.last_reason, "awaiting_all_joint_pairs")
        assembler.ingest(0x2A6, payload(3000, 4000), 10.001)
        assembler.ingest(0x2A7, payload(5000, 6000), 10.002)
        first = assembler.snapshot(10.003)
        self.assertEqual(len(first.positions), 6)
        self.assertFalse(first.velocity_valid)
        self.assertEqual(first.velocities, ())
        self.assertEqual(first.stamp_monotonic, 10.)
        self.assertIsNone(assembler.snapshot(10.004))
        self.assertEqual(assembler.last_reason, "awaiting_new_cohort")
        assembler.ingest(0x2A5, payload(1100, 2100), 10.02)
        self.assertIsNone(assembler.snapshot(10.021))
        self.assertEqual(assembler.last_reason, "awaiting_new_cohort")

    def test_velocity_uses_each_pair_receive_time(self):
        assembler = decode.JointFeedbackAssembler(velocity_alpha=1.)
        for index, can_id in enumerate(decode.PAIR_IDS):
            assembler.ingest(can_id, payload(0, 0), 10. + index * .001)
        assembler.snapshot(10.003)
        for index, can_id in enumerate(decode.PAIR_IDS):
            assembler.ingest(can_id, payload(100, -100), 10.02 + index * .001)
        state = assembler.snapshot(10.023)
        self.assertTrue(state.velocity_valid)
        self.assertAlmostEqual(state.velocities[0], 100 * decode.ANGLE_SCALE / .02)
        self.assertAlmostEqual(state.velocities[1], -state.velocities[0])

    def test_stale_missing_or_unsynchronized_pairs_prevent_state(self):
        assembler = decode.JointFeedbackAssembler()
        for index, can_id in enumerate(decode.PAIR_IDS):
            assembler.ingest(can_id, payload(), 10. + index * .001)
        self.assertIsNone(assembler.snapshot(10.151))
        self.assertEqual(assembler.last_reason, "feedback_stale")
        assembler = decode.JointFeedbackAssembler()
        for index, can_id in enumerate(decode.PAIR_IDS):
            assembler.ingest(can_id, payload(), 10. + index * .02)
        self.assertIsNone(assembler.snapshot(10.041))
        self.assertEqual(assembler.last_reason, "joint_pairs_unsynchronized")

    def test_out_of_order_pair_does_not_replace_newer_actual_state(self):
        assembler = decode.JointFeedbackAssembler()
        assembler.ingest(0x2A5, payload(1000, 2000), 10.)
        self.assertFalse(assembler.ingest(0x2A5, payload(0, 0), 9.99))
        assembler.ingest(0x2A6, payload(), 10.001)
        assembler.ingest(0x2A7, payload(), 10.002)
        state = assembler.snapshot(10.003)
        self.assertAlmostEqual(state.positions[0], 1000 * decode.ANGLE_SCALE)

    def test_long_gap_or_implausible_speed_does_not_fabricate_velocity(self):
        for step, dt, expected in [(100, .3, "derivative_interval_invalid"),
                                    (90000, .02, "derivative_rate_implausible")]:
            assembler = decode.JointFeedbackAssembler()
            for can_id in decode.PAIR_IDS:
                assembler.ingest(can_id, payload(), 10.)
            assembler.snapshot(10.)
            for can_id in decode.PAIR_IDS:
                assembler.ingest(can_id, payload(step, 0), 10. + dt)
            state = assembler.snapshot(10. + dt)
            self.assertFalse(state.velocity_valid)
            self.assertEqual(state.velocities, ())
            self.assertEqual(state.velocity_reason, expected)


class ReceiveBoundaryTests(unittest.TestCase):
    def test_filters_allow_only_three_standard_data_feedback_ids(self):
        filters = receiver.exact_feedback_filters()
        decoded = [struct.unpack("=II", filters[index:index + 8])
                   for index in range(0, len(filters), 8)]
        self.assertEqual([entry[0] for entry in decoded], list(decode.PAIR_IDS))
        mask = decode.CAN_SFF_MASK | decode.CAN_EFF_FLAG | decode.CAN_RTR_FLAG
        self.assertTrue(all(entry[1] == mask for entry in decoded))
        # CAN_ERR_FLAG in can_mask routes these subscriptions to the kernel's
        # RX_ERR list, which would silently hide normal 0x2A5/6/7 feedback.
        self.assertTrue(all(not entry[1] & decode.CAN_ERR_FLAG for entry in decoded))

    def test_kernel_timestamp_preserves_queue_age(self):
        ancillary = [(socket.SOL_SOCKET, receiver.SO_TIMESTAMPNS,
                      receiver.TIMESPEC.pack(1000, 0))]
        stamp = receiver.receive_stamp_from_ancillary(ancillary, 1000.10, 50.)
        self.assertAlmostEqual(stamp, 49.90)
        with self.assertRaises(ValueError):
            receiver.receive_stamp_from_ancillary(ancillary, 1000.20, 50.)
        with self.assertRaises(ValueError):
            receiver.receive_stamp_from_ancillary([], 1000., 50.)

    def test_future_or_invalid_kernel_timestamp_rejected(self):
        for sec, nsec in [(1001, 0), (1000, 1000000000), (1000, -1)]:
            ancillary = [(socket.SOL_SOCKET, receiver.SO_TIMESTAMPNS,
                          receiver.TIMESPEC.pack(sec, nsec))]
            with self.assertRaises(ValueError):
                receiver.receive_stamp_from_ancillary(ancillary, 1000., 50.)

    def test_other_ros_master_refused(self):
        for uri in ("", "http://127.0.0.1:11311", "http://192.168.1.50:11461"):
            with self.assertRaises(ValueError):
                receiver.require_isolated_master(uri)
        receiver.require_isolated_master("http://127.0.0.1:11461")
        receiver.require_isolated_master("http://localhost:11461")

    def test_socket_lifecycle_calls_no_transmit_or_interface_setup(self):
        class FakeSocket:
            def __init__(self):
                self.calls = []

            def setsockopt(self, *args):
                self.calls.append(("setsockopt", args))

            def settimeout(self, value):
                self.calls.append(("settimeout", value))

            def bind(self, address):
                self.calls.append(("bind", address))

            def recvmsg(self, *args):
                self.calls.append(("recvmsg", args))
                return (decode.CLASSIC_CAN_FRAME.pack(0x2A5, 8, payload()),
                        [(socket.SOL_SOCKET, receiver.SO_TIMESTAMPNS,
                          receiver.TIMESPEC.pack(1000, 0))], 0, ("can0",))

            def close(self):
                self.calls.append(("close",))

        fake = FakeSocket()
        with patch.object(receiver.socket, "socket", return_value=fake), \
                patch.object(receiver.time, "time", return_value=1000.), \
                patch.object(receiver.time, "monotonic", return_value=50.):
            read_only = receiver.ReceiveOnlySocket("can0")
            read_only.open()
            self.assertEqual(read_only.receive(), (0x2A5, payload(), 50.))
            read_only.close()
        self.assertEqual([entry[0] for entry in fake.calls],
                         ["setsockopt", "setsockopt", "setsockopt", "settimeout",
                          "bind", "recvmsg", "close"])
        # CAN_RAW_ERR_FILTER is option 2; leaving it untouched retains the
        # kernel default of no error-frame delivery. ERR decoding is still
        # rejected by test_flags_control_ids_bad_dlc_and_corruption_are_rejected.
        self.assertFalse(any(entry[0] == "setsockopt" and
                             entry[1][:2] == (receiver.SOL_CAN_RAW, 2)
                             for entry in fake.calls))

    def test_receiver_source_has_no_sdk_or_can_transmit_calls(self):
        tree = ast.parse((RUNTIME / "piper_can_receive_only.py").read_text())
        called_names = {node.func.attr for node in ast.walk(tree)
                        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)}
        self.assertFalse(called_names & {"send", "sendall", "sendto", "ConnectPort",
                                         "MotionCtrl_1", "MotionCtrl_2", "EnableArm",
                                         "JointCtrl", "GripperCtrl"})
        imports = [node for node in ast.walk(tree)
                   if isinstance(node, (ast.Import, ast.ImportFrom))]
        self.assertFalse(any("piper_sdk" in ast.dump(node) for node in imports))


if __name__ == "__main__":
    unittest.main()
