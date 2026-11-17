"""Selective dispatch for the installed Piper V2 SDK's frequent feedback frames.

Construction does not install the callback or touch CAN. Rare frames and SDK FK
use the original parser. Only the six scalar-copy feedback updaters may consume
the reusable message; parameter replies can retain objects and must not use it.
"""
from threading import local
import time

from piper_sdk.piper_msgs.msg_v2 import PiperMessage


class FeedbackDispatch:
    def __init__(self, piper, diagnostics=False):
        self._piper = piper
        self._original = piper.ParseCANFrame
        self._decode = piper._C_PiperInterface__parser.DecodeMessage
        self._increment = piper._C_PiperInterface__fps_counter.increment
        self._messages = local()
        self._diagnostics = diagnostics
        self._path = ('idle', 0., 0, 0.)
        self._last_joint_dispatch_finished = 0.
        self._last_joint_dispatch_source = 0.
        self._max_callback_ms = 0.
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
        if not self._diagnostics:
            return self._dispatch(frame)
        began = time.perf_counter()
        self._path = ('decode', began, frame.arbitration_id, frame.timestamp)
        try:
            return self._dispatch(frame)
        finally:
            done = time.perf_counter()
            self._max_callback_ms = max(self._max_callback_ms, (done-began)*1000.)
            if frame.arbitration_id in (0x2A5, 0x2A6, 0x2A7):
                self._last_joint_dispatch_finished, self._last_joint_dispatch_source = done, frame.timestamp
            self._path = ('idle', done, frame.arbitration_id, frame.timestamp)

    def path_diagnostics(self):
        # Diagnostic observations only; never grant freshness or control.
        stage, stamp, ident, source = self._path
        return dict(stage=stage, stage_age_ms=round((time.perf_counter()-stamp)*1000., 3),
                    can_id=hex(ident), source_stamp=source,
                    last_joint_dispatch_finished=self._last_joint_dispatch_finished,
                    last_joint_dispatch_source=self._last_joint_dispatch_source,
                    max_callback_ms=round(self._max_callback_ms, 3))

    def _dispatch(self, frame):
        update = self._updates.get(frame.arbitration_id)
        if update is None or self._piper._C_PiperInterface__start_sdk_fk_cal:
            if self._diagnostics:
                self._path = ('original_parser', self._path[1], frame.arbitration_id, frame.timestamp)
            return self._original(frame)
        # The SDK has one receive thread. Thread-local scratch also keeps this
        # callback correct if a caller independently parses another stream.
        message = getattr(self._messages, 'message', None)
        if message is None:
            message = self._messages.message = PiperMessage()
        if self._decode(frame, message):
            if self._diagnostics:
                self._path = ('sdk_update', self._path[1], frame.arbitration_id, frame.timestamp)
            self._increment('CanMonitor')
            update(message)
