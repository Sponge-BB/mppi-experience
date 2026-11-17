"""Pure base command lease; importing this module never opens UART or ROS."""

import json
import math
from pathlib import Path
import threading


COMMAND_FIELDS = frozenset(("sequence", "source_stamp", "expires_at", "base_v", "base_w"))
ZERO_VELOCITY = (0.0, 0.0, 0.0)
REQUIRED_CHECKS = frozenset(("localization_stable", "collision_model_validated",
                             "obstacle_coverage_validated", "actuator_response_validated"))


def validate_readiness(path):
    """Require a nonempty, explicit all-true set of hardware checks."""
    try:
        data = json.loads(Path(path).read_text())
        checks = data.get("required_checks") if isinstance(data, dict) else None
        if not isinstance(checks, dict) or not checks:
            return False, "readiness_required_checks_missing"
        missing = sorted(REQUIRED_CHECKS - set(checks))
        if missing:
            return False, "readiness_required_checks_missing:" + ",".join(missing)
        failed = sorted(str(key) for key, value in checks.items() if value is not True)
        if failed:
            return False, "readiness_incomplete:" + ",".join(failed)
        return True, "readiness_complete"
    except (OSError, ValueError, TypeError) as exc:
        return False, "readiness_unavailable:" + type(exc).__name__


class BaseLeaseGate:
    """No receipt-time renewal, no lateral command, and explicit fault reset.

    Source stamps are wall/ROS time, while the output deadline uses monotonic
    time. Both clocks must be supplied so clock changes revoke the lease. The
    sender must hold ``lock`` from output selection until its bounded write.
    """

    def __init__(self, allow_motion=False, max_lease=.15, max_v=.05, max_w=.15):
        if type(allow_motion) is not bool:
            raise ValueError("allow_motion must be bool")
        values = (max_lease, max_v, max_w)
        if (any(type(v) not in (int, float) or not math.isfinite(v) for v in values)
                or not 0 < max_lease <= .15 or not 0 < max_v <= .05
                or not 0 < max_w <= .15):
            raise ValueError("invalid base limits")
        self.allow_motion = allow_motion
        self.max_lease = float(max_lease)
        self.max_v = float(max_v)
        self.max_w = float(max_w)
        self.lock = threading.RLock()
        self.enabled = False
        self.reason = "disabled_at_startup"
        self.velocity = ZERO_VELOCITY
        self.deadline = -math.inf
        self.enable_source_floor = math.inf
        self.last_sequence = -1
        self.last_source_stamp = -math.inf
        self.last_monotonic = -math.inf
        self.last_clock_offset = None

    def stop(self, reason="explicit_disable"):
        with self.lock:
            self.enabled = False
            self.reason = reason
            self.velocity = ZERO_VELOCITY
            self.deadline = -math.inf
            self.enable_source_floor = math.inf

    def _times(self, now, wall):
        if (any(type(value) not in (int, float) or not math.isfinite(value)
                for value in (now, wall)) or now < self.last_monotonic):
            self.stop("clock_invalid")
            return False
        offset = wall - now
        old_offset = self.last_clock_offset
        self.last_clock_offset = offset
        self.last_monotonic = now
        if old_offset is not None and abs(offset - old_offset) > .02:
            self.stop("clock_offset_changed")
            return False
        return True

    def enable(self, requested, now, wall):
        with self.lock:
            if not self._times(now, wall):
                return False
            if type(requested) is not bool:
                self.stop("invalid_enable_request")
                return False
            if not requested:
                self.stop()
                return True
            if not self.allow_motion:
                self.stop("motion_not_allowed")
                return False
            if self.enabled:
                return False
            self.enabled = True
            self.reason = "awaiting_new_command"
            self.velocity = ZERO_VELOCITY
            self.enable_source_floor = wall
            self.deadline = now + self.max_lease
            return True

    def command(self, payload, now, wall):
        with self.lock:
            if not self._times(now, wall):
                return False
            if not self.enabled:
                return False
            if now >= self.deadline:
                self.stop("command_timeout")
                return False
            try:
                if not isinstance(payload, dict) or set(payload) != COMMAND_FIELDS:
                    raise ValueError("command_fields_invalid")
                sequence = payload["sequence"]
                if type(sequence) is not int or not 0 <= sequence <= 0xffffffff:
                    raise ValueError("sequence_invalid")
                if sequence <= self.last_sequence:
                    raise ValueError("command_replay")
                raw = tuple(payload[key] for key in
                            ("source_stamp", "expires_at", "base_v", "base_w"))
                if any(type(value) not in (int, float) or not math.isfinite(value)
                       for value in raw):
                    raise ValueError("command_nonfinite_or_type")
                stamp, expires_at, base_v, base_w = raw
                if stamp < self.enable_source_floor or stamp <= self.last_source_stamp:
                    raise ValueError("command_before_enable_or_replayed")
                if stamp > wall + .002:
                    raise ValueError("command_from_future")
                if expires_at <= stamp or expires_at - stamp > self.max_lease + 1e-9:
                    raise ValueError("lease_invalid")
                if expires_at <= wall:
                    raise ValueError("command_expired")
                if abs(base_v) > self.max_v or abs(base_w) > self.max_w:
                    raise ValueError("velocity_limit")
            except (TypeError, ValueError, OverflowError) as exc:
                self.stop(str(exc))
                return False
            self.last_sequence = sequence
            self.last_source_stamp = float(stamp)
            # The immutable absolute expiry, not receipt time, owns this lease.
            self.deadline = now + (expires_at - wall)
            self.velocity = (float(base_v), 0.0, float(base_w))
            self.reason = "command_accepted"
            return True

    def output(self, now, wall):
        with self.lock:
            if not self._times(now, wall):
                return ZERO_VELOCITY, True
            if self.enabled and now >= self.deadline:
                self.stop("command_timeout")
            if not self.enabled:
                return ZERO_VELOCITY, True
            return self.velocity, False

    def status(self):
        with self.lock:
            return dict(enabled=self.enabled, reason=self.reason,
                        allow_motion=self.allow_motion, last_sequence=self.last_sequence,
                        motion_validated=False)
