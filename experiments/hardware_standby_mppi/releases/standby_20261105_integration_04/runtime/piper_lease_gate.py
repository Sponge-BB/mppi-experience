"""Single-lock adapter around the existing Piper SDK, never an encoder.

The SDK is injected. Importing this module cannot open CAN. All wall-clock
timestamps are checked once and their original expiry is mapped to monotonic
time. tick() must be called by an independent watchdog, not the ROS callback
queue. This is a process-level guard, not a safety-rated emergency stop.
"""

import math
import threading


JOINT_NAMES = tuple("joint%d" % i for i in range(1, 7))
LOWER = (-2.618, -.05, -2.697, -1.832, -1.25, -.35)
UPPER = (2.618, 3.14, .05, 1.832, 1.25, .35)


def open_existing_sdk(factory, can_port, transport_installer=None):
    """Use the frozen, unmodified SDK; deliberately skip PiperInit requests."""
    sdk = factory(can_name=can_port)
    if transport_installer is not None:
        sdk = transport_installer(sdk)
    sdk.ConnectPort(piper_init=False)
    return sdk


class PiperLeaseGate:
    def __init__(self, sdk=None, allow_motion=False, lease_s=.15,
                 feedback_max_age_s=.15, max_position_lead_rad=.03, fixed_vel=5):
        if type(allow_motion) is not bool:
            raise ValueError("allow_motion must be a bool")
        if not (0 < lease_s <= .20 and 0 < feedback_max_age_s <= .20
                and 0 < max_position_lead_rad <= .03):
            raise ValueError("invalid guard limits")
        if type(fixed_vel) is not int or not 1 <= fixed_vel <= 10:
            raise ValueError("fixed_vel must be an integer in [1, 10]")
        self.sdk = sdk
        self.allow_motion = allow_motion
        self.lease_s = float(lease_s)
        self.feedback_max_age_s = float(feedback_max_age_s)
        self.max_lead = float(max_position_lead_rad)
        self.fixed_vel = fixed_vel
        self.lock = threading.RLock()
        self.enabled = False
        self.reason = "disabled_at_startup"
        self.last_sequence = -1
        self.last_source_stamp = -math.inf
        self.enable_source_floor = math.inf
        self.feedback = None
        self.feedback_expiry = -math.inf
        self.feedback_source_stamp = -math.inf
        self.lease_expiry = -math.inf
        self.last_monotonic = -math.inf
        self.last_clock_offset = None
        self.tx_operations = 0

    def _times(self, now, wall):
        if (not math.isfinite(now) or not math.isfinite(wall)
                or now < self.last_monotonic):
            self._stop("clock_invalid")
            return False
        offset = wall - now
        if self.last_clock_offset is not None and abs(offset-self.last_clock_offset) > .02:
            self._stop("clock_offset_changed")
            self.feedback = None
            self.feedback_expiry = -math.inf
            self.last_clock_offset = offset
            self.last_monotonic = now
            return False
        self.last_monotonic = now
        self.last_clock_offset = offset
        return True

    def _tx(self, method, *args):
        # Every SDK operation is reached only while self.lock is held.
        if self.sdk is None:
            raise RuntimeError("transport is disconnected")
        deadline_setter = getattr(self.sdk, "set_transport_deadline", None)
        if deadline_setter is not None:
            # Stop uses the adapter's per-frame timeout, independent of the
            # expired motion lease. It is still only a best-effort stop.
            deadline_setter(None if method == "MotionCtrl_1" else self.lease_expiry)
        getattr(self.sdk, method)(*args)
        self.tx_operations += 1

    def _stop(self, reason):
        was_enabled = self.enabled
        self.enabled = False
        self.reason = reason
        self.lease_expiry = -math.inf
        self.enable_source_floor = math.inf
        if was_enabled:
            # Latch BEFORE attempting CAN; transport errors cannot reopen gate.
            try:
                self._tx("MotionCtrl_1", 0x01, 0, 0)
            except Exception as exc:
                self.reason = reason + ":stop_transport_error:" + type(exc).__name__

    def update_feedback(self, names, positions, source_stamp, now, wall):
        with self.lock:
            if not self._times(now, wall):
                return False
            try:
                if tuple(names) != JOINT_NAMES or len(positions) != 6:
                    raise ValueError("joint_names_or_length")
                q = tuple(float(v) for v in positions)
                age = wall-float(source_stamp)
                if not all(math.isfinite(v) for v in q) or not math.isfinite(age):
                    raise ValueError("nonfinite_feedback")
                if age < -.002 or age >= self.feedback_max_age_s:
                    raise ValueError("feedback_stamp_invalid")
                if source_stamp <= self.feedback_source_stamp:
                    raise ValueError("feedback_replay")
                if any(v < lo-1e-6 or v > hi+1e-6 for v, lo, hi in zip(q, LOWER, UPPER)):
                    raise ValueError("feedback_outside_limits")
            except (TypeError, ValueError, OverflowError) as exc:
                self.feedback = None
                self.feedback_expiry = -math.inf
                self._stop(str(exc))
                return False
            self.feedback = q
            self.feedback_source_stamp = float(source_stamp)
            self.feedback_expiry = now + self.feedback_max_age_s-max(0., age)
            return True

    def enable(self, requested, now, wall):
        with self.lock:
            if not self._times(now, wall):
                return False
            if type(requested) is not bool:
                self._stop("invalid_enable_request")
                return False
            if not requested:
                self._stop("explicit_disable")
                return True
            if self.enabled:
                # Repeated enable messages must not renew an existing lease.
                return False
            if not self.allow_motion or self.sdk is None:
                self.reason = "motion_not_allowed_or_transport_disconnected"
                return False
            if self.feedback is None or now >= self.feedback_expiry:
                self.reason = "fresh_verified_feedback_required"
                return False
            self.enable_source_floor = wall
            self.lease_expiry = now+self.lease_s
            self.reason = "awaiting_new_command"
            # An enable frame may reach the bus before a transport fault is
            # reported. Treat that case as possibly enabled and attempt stop.
            self.enabled = True
            try:
                self._tx("EnableArm", 7)
            except Exception as exc:
                self._stop("enable_transport_error:"+type(exc).__name__)
                return False
            return True

    def command(self, names, positions, sequence, source_stamp, now, wall,
                expires_at=None):
        with self.lock:
            if not self._times(now, wall):
                return False
            if not self.enabled:
                return False
            # An arriving callback cannot revive a timed-out lease, even if the
            # watchdog thread has not run yet.
            if now >= self.lease_expiry:
                self._stop("command_timeout")
                return False
            if self.feedback is None or now >= self.feedback_expiry:
                self._stop("feedback_timeout")
                return False
            try:
                if type(sequence) is not int or not 0 <= sequence <= 0xffffffff:
                    raise ValueError("invalid_sequence")
                if sequence <= self.last_sequence:
                    raise ValueError("command_replay")
                if tuple(names) != JOINT_NAMES or len(positions) != 6:
                    raise ValueError("joint_names_or_length")
                q = tuple(float(v) for v in positions)
                stamp = float(source_stamp)
                age = wall-stamp
                if (not all(math.isfinite(v) for v in q) or not math.isfinite(age)
                        or age < -.002 or age >= self.lease_s):
                    raise ValueError("command_stamp_invalid")
                if stamp < self.enable_source_floor or stamp <= self.last_source_stamp:
                    raise ValueError("command_before_enable_or_replayed")
                if expires_at is None or isinstance(expires_at, bool):
                    raise ValueError("explicit_expiry_required")
                expiry = float(expires_at)
                if (not math.isfinite(expiry) or expiry <= stamp or expiry <= wall
                        or expiry > stamp+self.lease_s+1e-9):
                    raise ValueError("command_expiry_invalid")
                if any(v < lo or v > hi for v, lo, hi in zip(q, LOWER, UPPER)):
                    raise ValueError("position_limit")
                if any(abs(v-measured) > self.max_lead for v, measured in zip(q, self.feedback)):
                    raise ValueError("position_lead_limit")
            except (TypeError, ValueError, OverflowError) as exc:
                self._stop(str(exc))
                return False
            # This is precisely the original driver's radians -> millidegrees
            # conversion and SDK API. No CAN packing/IDs are reimplemented.
            integer_targets = tuple(round(v*1000*180/math.pi) for v in q)
            self.last_sequence = sequence
            self.last_source_stamp = stamp
            # Receipt, watchdog, and transport retries never renew the
            # producer's original deadline (which may precede stamp+lease).
            self.lease_expiry = now + (expiry-wall)
            try:
                self._tx("MotionCtrl_2", 0x01, 0x01, self.fixed_vel, 0)
                self._tx("JointCtrl", *integer_targets)
            except Exception as exc:
                self._stop("command_transport_error:"+type(exc).__name__)
                return False
            self.reason = "command_accepted"
            return True

    def tick(self, now, wall):
        with self.lock:
            if not self._times(now, wall):
                return False
            if self.enabled:
                if self.feedback is None or now >= self.feedback_expiry:
                    self._stop("feedback_timeout")
                elif now >= self.lease_expiry:
                    self._stop("command_timeout")
            return self.enabled

    def stop(self, reason="explicit_stop"):
        with self.lock:
            self._stop(reason)

    def status(self):
        with self.lock:
            return dict(enabled=self.enabled, reason=self.reason,
                        allow_motion=self.allow_motion,
                        transport_connected=self.sdk is not None,
                        last_sequence=self.last_sequence,
                        sdk_tx_operations=self.tx_operations,
                        motion_validated=False)
