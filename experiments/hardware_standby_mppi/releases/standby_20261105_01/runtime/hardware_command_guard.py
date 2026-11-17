"""Pure logic for the isolated standby MPPI hardware command boundary.

This module never imports ROS, opens a device, enables a motor or publishes a
message.  ``actuate`` authorizes motion only; a future transport must always be
able to send the returned zero/hold and honour ``stop_required`` independently.
Every supplied timestamp is in ONE monotonic clock domain.  A ROS adapter must
validate source ROS stamps before converting their age into this domain.

Piper accepts position goals, not the six MPPI joint velocity fields.  Goals
below are short, bounded offsets from the latest measured joint positions.
The unexecuted suffix of an MPPI rollout can never renew the command lease.
"""

from dataclasses import dataclass
import math
from typing import Optional, Tuple


def _vector(values, size, label):
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(label + " must be numeric") from error
    if len(result) != size or not all(math.isfinite(value) for value in result):
        raise ValueError(label + " must contain %d finite values" % size)
    return result


def _clip(value, lower, upper):
    return max(lower, min(upper, value))


@dataclass(frozen=True)
class GuardConfig:
    joint_lower: Tuple[float, ...]
    joint_upper: Tuple[float, ...]
    joint_velocity: Tuple[float, ...] = (.15,) * 6
    joint_acceleration: Tuple[float, ...] = (.35,) * 6
    max_base_v: float = .05
    max_base_w: float = .15
    max_base_acceleration: float = .10
    max_base_angular_acceleration: float = .30
    max_lease: float = .25
    command_max_age: float = .10
    feedback_max_age: float = .15
    future_tolerance: float = .005
    position_horizon: float = .10
    max_position_lead: float = .03
    max_tracking_error: float = .08
    max_tick_step: float = .05

    def __post_init__(self):
        lower = list(_vector(self.joint_lower, 6, "joint_lower"))
        upper = list(_vector(self.joint_upper, 6, "joint_upper"))
        # The original real-robot bridge protects the Piper wrist with this
        # narrower envelope.  The optimizer must use these SAME effective bounds.
        lower[5] = max(lower[5], -.35)
        upper[5] = min(upper[5], .35)
        object.__setattr__(self, "joint_lower", tuple(lower))
        object.__setattr__(self, "joint_upper", tuple(upper))
        for name in ("joint_velocity", "joint_acceleration"):
            value = _vector(getattr(self, name), 6, name)
            if any(item <= 0.0 for item in value):
                raise ValueError(name + " must be positive")
            object.__setattr__(self, name, value)
        if any(lo >= hi for lo, hi in zip(lower, upper)):
            raise ValueError("effective joint bounds have an empty interval")
        for name in ("max_base_v", "max_base_w", "max_base_acceleration",
                     "max_base_angular_acceleration", "max_lease", "command_max_age",
                     "feedback_max_age", "position_horizon", "max_position_lead",
                     "max_tracking_error", "max_tick_step"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(name + " must be finite and positive")
            object.__setattr__(self, name, value)
        if not math.isfinite(self.future_tolerance) or self.future_tolerance < 0.0:
            raise ValueError("future_tolerance must be finite and nonnegative")


@dataclass(frozen=True)
class GuardOutput:
    base_v: float
    base_w: float
    joint_positions: Optional[Tuple[float, ...]]
    joint_velocities: Tuple[float, ...]
    actuate: bool
    stop_required: bool
    reason: str
    command_sequence: Optional[int]
    lease_remaining: float


class HardwareCommandGuard:
    """Fail-closed, explicitly enabled state machine for ONE control source.

    The caller must schedule ``tick`` independently of the solver.  Any fault
    revokes the lease and disables motion until a new explicit ``set_enabled``
    followed by a NEW command.  Repeated ``tick`` never extends a lease.
    """

    def __init__(self, config):
        if not isinstance(config, GuardConfig):
            raise TypeError("config must be GuardConfig")
        self.config = config
        self.enabled = False
        self.fault = None
        self._q = None
        self._dq = None
        self._feedback_valid = False
        self._measurement_stamp = None
        self._last_tick = None
        self._last_now = None
        self._controls = None
        self._expires = None
        self._last_sequence = None
        self._last_command_stamp = None
        self._applied = (0.0,) * 8
        self._last_setpoint = None

    def _time(self, now):
        try:
            now = float(now)
            if not math.isfinite(now):
                raise ValueError("nonfinite clock")
        except (TypeError, ValueError, OverflowError):
            self._trip("invalid_clock")
            return False
        if self._last_now is not None and now < self._last_now:
            self._trip("clock_regressed")
            return False
        self._last_now = now
        return True

    def _trip(self, reason):
        self.enabled = False
        self.fault = reason
        self._controls = None
        self._expires = None
        self._applied = (0.0,) * 8
        self._last_setpoint = None

    def _feedback_fault(self, now):
        if self._q is None:
            return "feedback_missing"
        if not self._feedback_valid:
            return self.fault or "invalid_feedback"
        if now - self._measurement_stamp > self.config.feedback_max_age:
            return "feedback_stale"
        if self._measurement_stamp - now > self.config.future_tolerance:
            return "feedback_from_future"
        return None

    def update_measurement(self, q, dq, now, stamp=None):
        """Store actual feedback, rejecting missing, old or impossible states.

        ``stamp`` is the measurement time, not a receipt-time replacement.  A
        duplicate old measurement may be stored but cannot refresh its age.
        """
        if not self._time(now):
            return False
        now = self._last_now
        self._feedback_valid = False
        try:
            q = _vector(q, 6, "q")
            dq = _vector(dq, 6, "dq")
            stamp = float(now if stamp is None else stamp)
            if not math.isfinite(stamp):
                raise ValueError("stamp is not finite")
        except (TypeError, ValueError, OverflowError):
            self._trip("invalid_feedback")
            return False
        if stamp - now > self.config.future_tolerance:
            self._trip("feedback_from_future")
            return False
        if self._measurement_stamp is not None and stamp < self._measurement_stamp:
            self._trip("feedback_out_of_order")
            return False
        # Retain the newest finite actual position for a safe hold result even
        # when it violates the configured operating envelope.
        self._q, self._dq, self._measurement_stamp = q, dq, stamp
        if any(value < lo or value > hi for value, lo, hi in
               zip(q, self.config.joint_lower, self.config.joint_upper)):
            self._trip("joint_position_limit")
            return False
        if any(abs(speed) > limit + 1e-9 for speed, limit in
               zip(dq, self.config.joint_velocity)):
            self._trip("joint_velocity_limit")
            return False
        for pos, speed, accel, lo, hi in zip(
                q, dq, self.config.joint_acceleration,
                self.config.joint_lower, self.config.joint_upper):
            stopping = pos + speed * abs(speed) / (2.0 * accel)
            if stopping < lo - 1e-9 or stopping > hi + 1e-9:
                self._trip("joint_stopping_limit")
                return False
        if (self._last_setpoint is not None and
                max(abs(actual - goal) for actual, goal in
                    zip(q, self._last_setpoint)) > self.config.max_tracking_error):
            self._trip("joint_tracking_error")
            return False
        self._feedback_valid = True
        return True

    def set_enabled(self, enabled, now):
        """Explicit arm/disarm.  Enabling never reuses a cached old command."""
        if not self._time(now):
            return False
        now = self._last_now
        if not isinstance(enabled, bool):
            self._trip("invalid_enable")
            return False
        self.enabled = False
        self._controls = None
        self._expires = None
        self._applied = (0.0,) * 8
        self._last_setpoint = None
        self._last_tick = float(now)
        if not enabled:
            self.fault = None
            return True
        reason = self._feedback_fault(now)
        if reason:
            self._trip(reason)
            return False
        # Invalid measured states cannot be cleared just by toggling enable.
        if any(value < lo or value > hi for value, lo, hi in
               zip(self._q, self.config.joint_lower, self.config.joint_upper)):
            self._trip("joint_position_limit")
            return False
        if any(abs(speed) > limit + 1e-9 for speed, limit in
               zip(self._dq, self.config.joint_velocity)):
            self._trip("joint_velocity_limit")
            return False
        self.fault = None
        self.enabled = True
        return True

    def accept_command(self, controls, sequence, stamp, lease, now):
        """Accept only the validated FIRST 8D control, never a future rollout."""
        if not self._time(now):
            return False
        now = self._last_now
        if not self.enabled:
            return False
        try:
            controls = _vector(controls, 8, "controls")
            if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
                raise ValueError("sequence must be a nonnegative integer")
            stamp, lease = float(stamp), float(lease)
            if not math.isfinite(stamp) or not math.isfinite(lease):
                raise ValueError("invalid command time")
        except (TypeError, ValueError, OverflowError):
            self._trip("invalid_command")
            return False
        if self._last_sequence is not None and sequence <= self._last_sequence:
            self._trip("command_sequence_rejected")
            return False
        if (self._last_command_stamp is not None and
                stamp < self._last_command_stamp):
            self._trip("command_out_of_order")
            return False
        if stamp - now > self.config.future_tolerance:
            self._trip("command_from_future")
            return False
        if now - stamp > self.config.command_max_age:
            self._trip("command_stale")
            return False
        if lease <= 0.0 or lease > self.config.max_lease:
            self._trip("command_lease_rejected")
            return False
        expires = min(stamp + lease, now + lease)
        if expires <= now:
            self._trip("command_expired")
            return False
        reason = self._feedback_fault(now)
        if reason:
            self._trip(reason)
            return False
        self._controls = controls
        self._expires = expires
        self._last_sequence = sequence
        self._last_command_stamp = stamp
        return True

    def _hold(self, reason, stop_required):
        return GuardOutput(0.0, 0.0, self._q, (0.0,) * 6, False,
                           stop_required, reason, self._last_sequence, 0.0)

    def tick(self, now):
        """Return a bounded motion command or immediate zero/actual-position hold."""
        if not self._time(now):
            return self._hold(self.fault, True)
        now = self._last_now
        if not self.enabled:
            return self._hold(self.fault or "disabled", self.fault is not None)
        reason = self._feedback_fault(now)
        if reason:
            self._trip(reason)
            return self._hold(reason, True)
        if self._controls is None:
            return self._hold("awaiting_command", False)
        if now >= self._expires:
            self._trip("command_expired")
            return self._hold("command_expired", True)
        # A stalled timer must not create a large acceleration step on resume.
        dt = min(self.config.max_tick_step,
                 max(0.0, now - (self._last_tick if self._last_tick is not None else now)))
        self._last_tick = float(now)
        limits = (self.config.max_base_v, self.config.max_base_w) + self.config.joint_velocity
        acceleration = (self.config.max_base_acceleration,
                        self.config.max_base_angular_acceleration) + self.config.joint_acceleration
        applied = []
        for requested, old, limit, accel in zip(self._controls, self._applied, limits, acceleration):
            target = _clip(requested, -limit, limit)
            applied.append(_clip(target, old - accel * dt, old + accel * dt))
        remaining = self._expires - now
        horizon = min(self.config.position_horizon, remaining)
        positions = []
        for index, (pos, lo, hi, accel) in enumerate(zip(
                self._q, self.config.joint_lower, self.config.joint_upper,
                self.config.joint_acceleration)):
            # Reserve a joint-limit stopping option as well as a bounded goal.
            lower_speed = -min(math.sqrt(max(0.0, 2.0 * accel * (pos - lo))),
                               (pos - lo) / horizon, self.config.max_position_lead / horizon)
            upper_speed = min(math.sqrt(max(0.0, 2.0 * accel * (hi - pos))),
                              (hi - pos) / horizon, self.config.max_position_lead / horizon)
            applied[index + 2] = _clip(applied[index + 2], lower_speed, upper_speed)
            positions.append(_clip(pos + applied[index + 2] * horizon, lo, hi))
        self._applied = tuple(applied)
        self._last_setpoint = tuple(positions)
        return GuardOutput(applied[0], applied[1], tuple(positions), tuple(applied[2:]),
                           True, False, "ok", self._last_sequence, remaining)
