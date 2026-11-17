"""Offline gate and unchanged UART reuse tests. No ROS, serial, or real SDK."""

import importlib.util
import json
from pathlib import Path
import struct
import sys
import tempfile
import threading
import types
import unittest
from unittest.mock import patch


RELEASE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RELEASE / "runtime"))
from base_lease_gate import BaseLeaseGate, REQUIRED_CHECKS, ZERO_VELOCITY, validate_readiness


def command(sequence=1, stamp=100.01, expiry=100.11, v=.04, w=-.10):
    return dict(sequence=sequence, source_stamp=stamp, expires_at=expiry,
                base_v=v, base_w=w)


class BaseGateTests(unittest.TestCase):
    def setUp(self):
        self.gate = BaseLeaseGate(allow_motion=True)
        self.assertTrue(self.gate.enable(True, 10., 100.))

    def test_default_disabled_and_no_sideways_velocity(self):
        inactive = BaseLeaseGate()
        self.assertFalse(inactive.enable(True, 10., 100.))
        self.assertFalse(inactive.command(command(), 10.01, 100.01))
        self.assertEqual(inactive.output(10.02, 100.02)[0], ZERO_VELOCITY)
        self.assertTrue(self.gate.command(command(), 10.01, 100.01))
        self.assertEqual(self.gate.output(10.02, 100.02), ((.04, 0., -.10), False))

    def test_receipt_does_not_extend_original_absolute_expiry(self):
        self.assertTrue(self.gate.command(command(), 10.09, 100.09))
        self.assertAlmostEqual(self.gate.deadline, 10.11)
        self.assertEqual(self.gate.output(10.105, 100.105)[0], (.04, 0., -.1))
        self.assertEqual(self.gate.output(10.111, 100.111), (ZERO_VELOCITY, True))
        self.assertEqual(self.gate.reason, "command_timeout")

    def test_short_input_expiry_is_preserved(self):
        self.assertTrue(self.gate.command(command(expiry=100.015), 10.01, 100.01))
        self.assertAlmostEqual(self.gate.deadline, 10.015)
        self.assertEqual(self.gate.output(10.016, 100.016)[0], ZERO_VELOCITY)

    def test_replay_stops_and_fresh_command_cannot_reenable(self):
        self.assertTrue(self.gate.command(command(), 10.01, 100.01))
        self.assertFalse(self.gate.command(command(), 10.02, 100.02))
        self.assertEqual(self.gate.reason, "command_replay")
        self.assertFalse(self.gate.command(command(sequence=2, stamp=100.03), 10.03, 100.03))
        self.assertEqual(self.gate.output(10.04, 100.04)[0], ZERO_VELOCITY)

    def test_late_callback_cannot_revive_expired_gate_before_timer(self):
        self.assertTrue(self.gate.command(command(), 10.01, 100.01))
        self.assertFalse(self.gate.command(command(sequence=2, stamp=100.12, expiry=100.20),
                                          10.12, 100.12))
        self.assertEqual(self.gate.reason, "command_timeout")

    def test_repeated_enable_does_not_extend_deadline(self):
        self.assertFalse(self.gate.enable(True, 10.05, 100.05))
        self.assertAlmostEqual(self.gate.deadline, 10.15)

    def test_new_enable_requires_new_stamp_and_preserves_sequence_floor(self):
        self.assertTrue(self.gate.command(command(), 10.01, 100.01))
        self.gate.stop()
        self.assertTrue(self.gate.enable(True, 10.02, 100.02))
        self.assertFalse(self.gate.command(command(sequence=2), 10.03, 100.03))
        self.assertEqual(self.gate.reason, "command_before_enable_or_replayed")
        self.assertTrue(self.gate.enable(True, 10.04, 100.04))
        self.assertFalse(self.gate.command(command(sequence=1, stamp=100.05, expiry=100.14),
                                          10.05, 100.05))
        self.assertEqual(self.gate.reason, "command_replay")

    def test_clock_offset_and_monotonic_regression_stop(self):
        for now, wall, reason in [(10.02, 101.02, "clock_offset_changed"),
                                  (9.99, 99.99, "clock_invalid")]:
            with self.subTest(reason=reason):
                gate = BaseLeaseGate(True)
                gate.enable(True, 10., 100.)
                gate.command(command(), 10.01, 100.01)
                self.assertEqual(gate.output(now, wall), (ZERO_VELOCITY, True))
                self.assertEqual(gate.reason, reason)

    def test_invalid_messages_stop_instead_of_silent_clamping(self):
        cases = [command(v=.051), command(w=-.151), command(v=float("nan")),
                 command(stamp=100.1, expiry=100.2), command(expiry=100.009),
                 command(expiry=100.2), command(sequence=True),
                 command(v=True), {**command(), "linear_y": .01},
                 {key: value for key, value in command().items() if key != "expires_at"}]
        for payload in cases:
            with self.subTest(payload=payload):
                gate = BaseLeaseGate(True)
                gate.enable(True, 10., 100.)
                self.assertFalse(gate.command(payload, 10.01, 100.01))
                self.assertFalse(gate.enabled)

    def test_readiness_all_fixed_checks_required_with_strict_bool(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "readiness.json"
            self.assertFalse(validate_readiness(path)[0])
            for checks in ({}, {"localization_stable": True},
                           {key: False for key in REQUIRED_CHECKS},
                           {key: 1 for key in REQUIRED_CHECKS}):
                path.write_text(json.dumps({"required_checks": checks}))
                self.assertFalse(validate_readiness(path)[0])
            path.write_text(json.dumps({"required_checks": {key: True for key in REQUIRED_CHECKS}}))
            self.assertTrue(validate_readiness(path)[0])


class FakeTimer:
    def __init__(self, period, callback, **_kwargs):
        self.period, self.callback = period, callback
        self.stopped = False

    def shutdown(self):
        self.stopped = True


class FakeThread:
    def __init__(self, **_kwargs):
        self.started = False

    def start(self):
        self.started = True

    def is_alive(self):
        return False


class FakeSerial:
    def __init__(self, *_args, **kwargs):
        self.kwargs, self.frames, self.is_open = kwargs, [], True

    def write(self, data):
        self.frames.append(data)
        return len(data)

    def close(self):
        self.is_open = False


class Message:
    def __init__(self, data=None):
        self.data = data


def load_driver():
    ros = types.ModuleType("rospy")
    ros.params, ros.subscribers, ros.publishers, ros.init_calls = {}, [], [], []
    ros.init_node = lambda *args, **kwargs: ros.init_calls.append((args, kwargs))
    ros.get_name = lambda: "/standby_trial_20261105_integration_04/base/leased_base_driver"
    ros.get_param = lambda key, default=None: ros.params.get(key, default)
    ros.Duration = lambda value: types.SimpleNamespace(to_sec=lambda: value)
    ros.Timer = FakeTimer
    ros.Subscriber = lambda name, *args, **kwargs: ros.subscribers.append(name)
    ros.Publisher = lambda name, *args, **kwargs: (ros.publishers.append(name) or
                                                  types.SimpleNamespace(publish=lambda _msg: None))
    ros.on_shutdown = lambda _callback: None
    for name in ("loginfo", "logwarn", "logerr", "logwarn_throttle", "logerr_throttle",
                 "signal_shutdown"):
        setattr(ros, name, lambda *args, **kwargs: None)
    serial = types.ModuleType("serial")
    serial.Serial = FakeSerial
    serial.SerialException = type("SerialException", (Exception,), {})
    serial.SerialTimeoutException = type("SerialTimeoutException", (Exception,), {})
    geometry = types.ModuleType("geometry_msgs.msg")
    geometry.Twist = Message
    std = types.ModuleType("std_msgs.msg")
    std.Bool = std.String = Message
    modules = {"rospy": ros, "serial": serial, "geometry_msgs.msg": geometry,
               "std_msgs.msg": std}
    with patch.dict(sys.modules, modules):
        spec = importlib.util.spec_from_file_location(
            "offline_leased_base_driver", RELEASE / "runtime/leased_base_driver.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    module._MonotonicTimer = FakeTimer
    module.original.threading = types.SimpleNamespace(Lock=threading.Lock, Event=threading.Event,
                                                       Thread=FakeThread,
                                                       current_thread=threading.current_thread)
    return module, ros


class BaseDriverReuseTests(unittest.TestCase):
    def setUp(self):
        self.module, self.ros = load_driver()
        self.master = patch.dict("os.environ", {"ROS_MASTER_URI": "http://127.0.0.1:11461"})
        self.master.start()
        self.addCleanup(self.master.stop)

    def test_default_never_constructs_serial_and_initializes_ros_once(self):
        with patch.object(self.module.original.serial, "Serial", side_effect=AssertionError("UART opened")):
            node = self.module.LeasedBaseDriver()
        self.assertFalse(node.transport_connected)
        self.assertEqual(len(self.ros.init_calls), 1)
        self.assertNotIn("/cmd_vel", self.ros.subscribers)
        self.assertIn(self.module.NAMESPACE + "/leased_command", self.ros.subscribers)

    def test_connect_refuses_missing_readiness_before_serial(self):
        self.ros.params["~connect_transport"] = True
        with tempfile.TemporaryDirectory() as folder, patch.object(self.module, "RELEASE", Path(folder)):
            with patch.object(self.module.original.serial, "Serial", side_effect=AssertionError("UART opened")):
                with self.assertRaisesRegex(RuntimeError, "readiness_unavailable"):
                    self.module.LeasedBaseDriver()

    def test_original_codec_receive_methods_and_bounded_uart_are_reused(self):
        self.ros.params.update({"~connect_transport": True, "~allow_motion": True})
        with tempfile.TemporaryDirectory() as folder, patch.object(self.module, "RELEASE", Path(folder)):
            path = Path(folder) / "config"
            path.mkdir()
            (path / "hardware_readiness.json").write_text(json.dumps(
                {"required_checks": {key: True for key in REQUIRED_CHECKS}}))
            node = self.module.LeasedBaseDriver()
        self.assertEqual(len(self.ros.init_calls), 1)
        self.assertEqual(node.ser.kwargs["write_timeout"], .005)
        self.assertEqual(node.send_timer.period.to_sec(), .02)
        self.assertIs(self.module.LeasedBaseDriver._consume_rx_bytes,
                      self.module.original.CmdVelToUartNode._consume_rx_bytes)
        self.assertIs(self.module.LeasedBaseDriver.serial_read_loop,
                      self.module.original.CmdVelToUartNode.serial_read_loop)
        self.assertIs(self.module.LeasedBaseDriver.feedback_timer_callback,
                      self.module.original.CmdVelToUartNode.feedback_timer_callback)
        self.assertNotIn("/cmd_vel", self.ros.subscribers)
        self.assertNotIn("/my_car_vel", self.ros.publishers)
        self.assertTrue(node._write_velocity_frame((.04, 0., -.1)))
        self.assertEqual(node.ser.frames[-1], struct.pack("<B3fB", 0xAA, .04, 0., -.1, 0x55))
        node.gate.enable(True, 10., 100.)
        node.gate.command(command(), 10.01, 100.01)
        self.assertFalse(node.cmd_vel_callback(Message()))
        self.assertEqual(node.gate.reason, "legacy_cmd_vel_rejected")
        node.shutdown_hook()

    def test_unsafe_serial_timeout_configuration_is_rejected(self):
        self.ros.params["~write_timeout"] = .05
        facade = self.module._OriginalRosAdapter(self.ros)
        with self.assertRaisesRegex(ValueError, "write_timeout"):
            facade.get_param("~write_timeout", .05)


if __name__ == "__main__":
    unittest.main()
