"""Bounded pairing of a six-joint command and its immutable lease metadata.

No ROS or device imports. Call tick from the driver's independent watchdog.
Enable through this object so an explicit re-enable discards incomplete pairs.
"""
import json
import math
import threading


class PiperCommandPair:
    def __init__(self, gate, ttl_s=.05, max_pending=4):
        if not 0 < ttl_s <= .05 or type(max_pending) is not int or not 1 <= max_pending <= 4:
            raise ValueError("invalid pair bounds")
        self.gate = gate
        self.ttl_s = ttl_s
        self.max_pending = max_pending
        self.pending = {}
        self.lock = threading.RLock()
        self.reason = "disabled_at_startup"

    def _fault(self, reason):
        self.pending.clear()
        self.reason = reason
        self.gate.stop(reason)
        return False

    def enable(self, requested, now, wall):
        with self.lock:
            result = self.gate.enable(requested, now, wall)
            if result:
                self.pending.clear()
                self.reason = "awaiting_pair" if requested else "explicit_disable"
            return result

    def tick(self, now, wall):
        with self.lock:
            if not math.isfinite(now) or not math.isfinite(wall):
                return self._fault("pair_clock_invalid")
            for item in self.pending.values():
                if now >= item["received"]+self.ttl_s:
                    return self._fault("command_pair_timeout")
                if "lease" in item and wall >= item["lease"][1]:
                    return self._fault("command_pair_expired")
            if not self.gate.tick(now, wall):
                self.pending.clear()
                return False
            return True

    def _put(self, kind, sequence, payload, now, wall):
        if not self.tick(now, wall):
            return False
        if type(sequence) is not int or not 0 <= sequence <= 0xffffffff:
            return self._fault("pair_sequence_invalid")
        if sequence <= self.gate.last_sequence:
            return self._fault("command_pair_replay")
        if sequence not in self.pending:
            if len(self.pending) >= self.max_pending:
                return self._fault("command_pair_capacity")
            self.pending[sequence] = {"received": now}
        item = self.pending[sequence]
        if kind in item:
            if item[kind] != payload:
                return self._fault("command_pair_conflict")
            return False  # an identical partial duplicate never renews its TTL
        item[kind] = payload
        if "joint" not in item or "lease" not in item:
            return False
        names, q, stamp = item["joint"]
        lease_stamp, expiry = item["lease"]
        if abs(stamp-lease_stamp) > 1e-6:
            return self._fault("command_pair_stamp_mismatch")
        del self.pending[sequence]
        accepted = self.gate.command(names, q, sequence, min(stamp, lease_stamp),
                                     now, wall, expires_at=expiry)
        if not accepted:
            self.pending.clear()
        self.reason = "command_pair_accepted" if accepted else self.gate.reason
        return accepted

    def lease(self, text, now, wall):
        with self.lock:
            if not self.gate.enabled:
                return False
            try:
                if not isinstance(text, str) or len(text) > 512:
                    raise ValueError("metadata_size")
                data = json.loads(text)
                if not isinstance(data, dict) or set(data) != {"sequence", "source_stamp", "expires_at"}:
                    raise ValueError("metadata_keys")
                if any(isinstance(data[key], bool) or not isinstance(data[key], (int, float))
                       for key in ("source_stamp", "expires_at")):
                    raise ValueError("metadata_type")
                stamp, expiry = float(data["source_stamp"]), float(data["expires_at"])
                if (not math.isfinite(stamp) or not math.isfinite(expiry)
                        or stamp > wall+.002 or expiry <= wall or expiry <= stamp
                        or expiry > stamp+self.gate.lease_s+1e-9):
                    raise ValueError("metadata_expiry")
                return self._put("lease", data["sequence"], (stamp, expiry), now, wall)
            except (ValueError, TypeError, OverflowError):
                return self._fault("command_lease_invalid")

    def joint(self, names, positions, sequence, source_stamp, now, wall,
              velocities=(), efforts=()):
        with self.lock:
            if not self.gate.enabled:
                return False
            try:
                if len(velocities) or len(efforts) or len(names) != 6 or len(positions) != 6:
                    raise ValueError("joint_shape")
                q = tuple(float(v) for v in positions)
                stamp = float(source_stamp)
                if not math.isfinite(stamp) or not all(math.isfinite(v) for v in q):
                    raise ValueError("joint_nonfinite")
                return self._put("joint", sequence, (tuple(names), q, stamp), now, wall)
            except (ValueError, TypeError, OverflowError):
                return self._fault("command_pair_joint_invalid")
