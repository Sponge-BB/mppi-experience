from pathlib import Path
import sys
import unittest

RELEASE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RELEASE/"python"))
sys.path.insert(0, str(RELEASE/"runtime"))
import can
from piper_sdk.hardware_port.can_encapsulation import C_STD_CAN
from bounded_can_transport import BoundedCanTransport, BoundedSendError, install_bounded_can_transport


class FakeBus:
    def __init__(self):
        self.calls = []
        self.error = None
        self.result = None

    def send(self, message, timeout=None):
        self.calls.append((message, timeout))
        if self.error:
            raise self.error
        return self.result


class FakeCAN:
    # Execute the actual frozen SDK frame construction, with a fake bus and
    # explicit healthy flag. No C_STD_CAN or socket constructor is called.
    SendCanMessage = C_STD_CAN.SendCanMessage

    def __init__(self):
        self.bus = FakeBus()
        self.channel_name = "offline_fake"
        self.healthy = True

    def is_can_bus_ok(self):
        return self.healthy


class FakeSDK:
    def __init__(self):
        self._C_PiperInterface__arm_can = FakeCAN()

    def GetCanBus(self):
        return self._C_PiperInterface__arm_can


class TransportTests(unittest.TestCase):
    def setUp(self):
        self.can = FakeCAN()
        self.t = [10.]
        self.transport = BoundedCanTransport(self.can, clock=lambda: self.t[0])

    def test_original_encoder_reused_and_timeout_is_finite(self):
        result = self.transport.SendCanMessage(0x155, bytes(range(8)))
        msg, timeout = self.can.bus.calls[0]
        self.assertEqual(msg.arbitration_id, 0x155)
        self.assertEqual(bytes(msg.data), bytes(range(8)))
        self.assertEqual(msg.dlc, 8)
        self.assertFalse(msg.is_extended_id)
        self.assertEqual(timeout, .005)
        self.assertIs(result, self.transport.CAN_STATUS.SEND_MESSAGE_SUCCESS)

    def test_remaining_lease_bounds_each_frame(self):
        self.transport.set_deadline(10.002)
        self.transport.SendCanMessage(0x155, bytes(8))
        self.assertAlmostEqual(self.can.bus.calls[0][1], .002)
        self.t[0] = 10.003
        with self.assertRaises(BoundedSendError):
            self.transport.SendCanMessage(0x156, bytes(8))
        self.assertEqual(len(self.can.bus.calls), 1)

    def test_timeout_canerror_is_not_silently_swallowed(self):
        self.can.bus.error = can.CanOperationError("fake queue full")
        with self.assertRaises(BoundedSendError):
            self.transport.SendCanMessage(0x155, bytes(8))
        self.assertEqual(self.transport.success_count, 0)

    def test_silent_skip_is_a_transport_fault(self):
        self.can.healthy = False
        with self.assertRaises(BoundedSendError):
            self.transport.SendCanMessage(0x155, bytes(8))
        self.assertEqual(self.can.bus.calls, [])

    def test_stop_can_attempt_bounded_send_after_motion_expiry(self):
        self.transport.set_deadline(9.)
        with self.assertRaises(BoundedSendError):
            self.transport.SendCanMessage(0x155, bytes(8))
        self.transport.set_deadline(None)
        self.transport.SendCanMessage(0x150, bytes(8))
        self.assertEqual(self.can.bus.calls[0][1], .005)

    def test_bus_replacement_refused(self):
        self.can.bus = FakeBus()
        with self.assertRaises(BoundedSendError):
            self.transport.SendCanMessage(0x155, bytes(8))

    def test_unknown_success_return_refused(self):
        self.can.bus.result = False
        with self.assertRaises(BoundedSendError):
            self.transport.SendCanMessage(0x155, bytes(8))

    def test_external_install_reuses_original_sdk_object(self):
        sdk = FakeSDK()
        wrapped = install_bounded_can_transport(sdk, clock=lambda: 10.)
        self.assertIs(wrapped.original, sdk)
        self.assertIs(sdk.GetCanBus(), wrapped.transport)
        wrapped.set_transport_deadline(10.1)
        self.assertEqual(wrapped.transport.deadline, 10.1)

    def test_unknown_sdk_layout_refused(self):
        class UnknownSDK:
            def GetCanBus(self):
                return FakeCAN()
        with self.assertRaises(BoundedSendError):
            install_bounded_can_transport(UnknownSDK())


if __name__ == "__main__":
    unittest.main()
