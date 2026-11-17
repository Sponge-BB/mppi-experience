"""External adapter for frozen Piper SDK 0.6.1 CAN transport.

The original SDK still encodes all frames. No CAN is opened by this module.
Call install_bounded_can_transport on a constructed SDK BEFORE ConnectPort.
Socket send timeout is best effort: Python/OS scheduling and hardware are not
real-time, and a submitted frame is not proof that the actuator stopped.
"""
import math
import threading
import time
from types import SimpleNamespace


class BoundedSendError(RuntimeError):
    """Not a can.CanError: the original wrapper must not swallow failures."""


class BoundedCanTransport:
    def __init__(self, original_can, send_timeout_s=.005, clock=time.monotonic):
        if not math.isfinite(send_timeout_s) or not 0 < send_timeout_s <= .01:
            raise ValueError("send_timeout_s must be in (0, .01]")
        bus = original_can.bus
        if bus is None or not callable(getattr(bus, "send", None)):
            raise BoundedSendError("CAN bus was not opened")
        self.original = original_can
        self.bus = bus
        self.send_timeout_s = float(send_timeout_s)
        self.clock = clock
        self.deadline = None
        self.success_count = 0
        self.lock = threading.RLock()
        self._raw_send = bus.send
        self.CAN_STATUS = SimpleNamespace(SEND_MESSAGE_SUCCESS=object())
        # Retain the actual BusABC object, required by original is_can_bus_ok.
        # Alter only this isolated instance's method, never the SDK source or
        # global BusABC implementation.
        bus.send = self._send

    def __getattr__(self, name):
        return getattr(self.original, name)

    def set_deadline(self, deadline):
        with self.lock:
            if deadline is not None and not math.isfinite(deadline):
                raise BoundedSendError("invalid send deadline")
            self.deadline = deadline

    def _send(self, message, timeout=None):
        with self.lock:
            cap = self.send_timeout_s
            if self.deadline is not None:
                cap = min(cap, self.deadline-self.clock())
            if cap <= 0:
                raise BoundedSendError("motion lease expired before CAN send")
            try:
                result = self._raw_send(message, timeout=cap)
            except Exception as exc:
                raise BoundedSendError("CAN send failed: "+type(exc).__name__) from exc
            if result is not None:
                raise BoundedSendError("unexpected CAN send result")
            self.success_count += 1
            if self.deadline is not None and self.clock() >= self.deadline:
                raise BoundedSendError("CAN send completed after motion lease")

    def SendCanMessage(self, arbitration_id, data):
        with self.lock:
            if self.original.bus is not self.bus:
                raise BoundedSendError("CAN bus replacement is not authorized")
            before = self.success_count
            self.original.SendCanMessage(arbitration_id, data)
            if self.success_count != before+1:
                raise BoundedSendError("original CAN wrapper skipped or hid send failure")
            # This SDK interface expects CAN_STATUS, but its CAN wrapper lacks
            # that enum and returns None even on success. Supply a local sentinel
            # ONLY after the underlying bus.send completed successfully.
            return self.CAN_STATUS.SEND_MESSAGE_SUCCESS


class BoundedSDK:
    def __init__(self, sdk, transport):
        self.original = sdk
        self.transport = transport

    def __getattr__(self, name):
        return getattr(self.original, name)

    def set_transport_deadline(self, deadline):
        self.transport.set_deadline(deadline)


def install_bounded_can_transport(sdk, send_timeout_s=.005, clock=time.monotonic):
    original_can = sdk.GetCanBus()
    # Deliberately tied to the frozen SDK class. Refuse unknown versions;
    # guessing an attribute could leave an unbounded alternate send path.
    field = "_C_PiperInterface__arm_can"
    if getattr(sdk, field, None) is not original_can:
        raise BoundedSendError("unexpected frozen SDK CAN object layout")
    transport = BoundedCanTransport(original_can, send_timeout_s, clock)
    setattr(sdk, field, transport)
    return BoundedSDK(sdk, transport)
