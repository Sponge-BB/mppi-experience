"""Selective dispatch for the installed Piper V2 SDK's frequent feedback frames.

Construction does not install the callback or touch CAN. Rare frames and SDK FK
use the original parser. Only the six scalar-copy feedback updaters may consume
the reusable message; parameter replies can retain objects and must not use it.
"""
from threading import local

from piper_sdk.piper_msgs.msg_v2 import PiperMessage


class FeedbackDispatch:
    def __init__(self, piper):
        self._piper = piper
        self._original = piper.ParseCANFrame
        self._decode = piper._C_PiperInterface__parser.DecodeMessage
        self._increment = piper._C_PiperInterface__fps_counter.increment
        self._messages = local()
        groups = (
            ((0x2A1,), 'UpdateArmStatus'),
            ((0x2A2, 0x2A3, 0x2A4), 'UpdateArmEndPoseState'),
            ((0x2A5, 0x2A6, 0x2A7), 'UpdateArmJointState'),
            ((0x2A8,), 'UpdateArmGripperState'),
            (range(0x251, 0x257), 'UpdateDriverInfoHighSpdFeedback'),
            (range(0x261, 0x267), 'UpdateDriverInfoLowSpdFeedback'),
        )
        self._updates = {
            can_id: getattr(piper, '_C_PiperInterface__' + method)
            for ids, method in groups for can_id in ids
        }

    def __call__(self, frame):
        update = self._updates.get(frame.arbitration_id)
        if update is None or self._piper._C_PiperInterface__start_sdk_fk_cal:
            return self._original(frame)
        # The SDK has one receive thread. Thread-local scratch also keeps this
        # callback correct if a caller independently parses another stream.
        message = getattr(self._messages, 'message', None)
        if message is None:
            message = self._messages.message = PiperMessage()
        if self._decode(frame, message):
            self._increment('CanMonitor')
            update(message)
