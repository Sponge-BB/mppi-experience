"""Receive-side Piper joint CAN decoding, with no SDK or hardware imports.

Protocol evidence: locally installed piper_sdk 0.6.1, protocol_v2 decoder
and ArmMsgFeedBackJointStates.  IDs 0x2A5/6/7 each contain two signed int32
BIG-endian angles in 0.001 degree.  This module never constructs TX frames.
"""

from dataclasses import dataclass
import math
import struct


CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x7FF
PAIR_IDS = (0x2A5, 0x2A6, 0x2A7)
PAIR_INDEX = {can_id: index for index, can_id in enumerate(PAIR_IDS)}
CLASSIC_CAN_FRAME = struct.Struct("=IB3x8s")
ANGLE_SCALE = math.pi / 180000.0


def decode_joint_pair(can_id, payload):
    """Return (pair_index, q_pair_rad); reject flags, wrong IDs and bad DLC."""
    if (not isinstance(can_id, int) or can_id not in PAIR_INDEX or
            can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)):
        raise ValueError("not a standard Piper joint feedback data frame")
    if not isinstance(payload, (bytes, bytearray)) or len(payload) != 8:
        raise ValueError("joint feedback DLC must be exactly 8")
    raw = struct.unpack(">ii", payload)
    # A generous corruption check, not the robot's operating joint limits.
    # Feedback is never clamped to an assumed safe configuration.
    if any(abs(value) > 360000 for value in raw):
        raise ValueError("joint feedback angle exceeds 360-degree sanity bound")
    return PAIR_INDEX[can_id], tuple(value * ANGLE_SCALE for value in raw)


def decode_socketcan_frame(frame):
    if not isinstance(frame, (bytes, bytearray)) or len(frame) != CLASSIC_CAN_FRAME.size:
        raise ValueError("expected a 16-byte classic SocketCAN frame")
    can_id, dlc, payload = CLASSIC_CAN_FRAME.unpack(frame)
    if dlc != 8:
        raise ValueError("joint feedback DLC must be exactly 8")
    pair_index, angles = decode_joint_pair(can_id, payload)
    return can_id, pair_index, angles


@dataclass(frozen=True)
class JointFeedbackSnapshot:
    positions: tuple
    velocities: tuple
    velocity_valid: bool
    velocity_reason: str
    stamp_monotonic: float
    newest_stamp_monotonic: float
    pair_skew: float


class JointFeedbackAssembler:
    """Publish only a fresh, newly received cohort of ALL three joint pairs.

    No zero-filled partial states, repeat-time freshness or motor RPM guesses.
    Velocity is an explicitly estimated position derivative, absent at startup
    or when timing / plausible-rate checks fail.  Every pair retains its own
    conservative kernel-receive timestamp.
    """

    def __init__(self, max_age=.15, max_pair_skew=.03,
                 min_velocity_dt=.002, max_velocity_dt=.20,
                 max_estimated_velocity=5.0, velocity_alpha=.25):
        for value in (max_age, max_pair_skew, min_velocity_dt,
                      max_velocity_dt, max_estimated_velocity):
            if not math.isfinite(value) or value <= 0.:
                raise ValueError("assembler bounds must be finite and positive")
        if max_velocity_dt < min_velocity_dt or not 0. < velocity_alpha <= 1.:
            raise ValueError("invalid derivative configuration")
        self.max_age = max_age
        self.max_pair_skew = max_pair_skew
        self.min_velocity_dt = min_velocity_dt
        self.max_velocity_dt = max_velocity_dt
        self.max_estimated_velocity = max_estimated_velocity
        self.velocity_alpha = velocity_alpha
        self._pairs = {}
        self._previous = None
        self._filtered_velocity = None
        self.last_reason = "awaiting_all_joint_pairs"
        self.accepted_frames = 0
        self.rejected_frames = 0
        self.published_snapshots = 0

    def ingest(self, can_id, payload, stamp_monotonic):
        try:
            index, angles = decode_joint_pair(can_id, payload)
            stamp = float(stamp_monotonic)
            if not math.isfinite(stamp) or stamp < 0.:
                raise ValueError("invalid receive timestamp")
            if index in self._pairs and stamp <= self._pairs[index][1]:
                raise ValueError("duplicate or out-of-order receive timestamp")
        except (TypeError, ValueError, OverflowError):
            self.rejected_frames += 1
            return False
        self._pairs[index] = (angles, stamp)
        self.accepted_frames += 1
        return True

    def snapshot(self, now_monotonic):
        now = float(now_monotonic)
        if not math.isfinite(now):
            self.last_reason = "invalid_clock"
            return None
        if len(self._pairs) != 3:
            self.last_reason = "awaiting_all_joint_pairs"
            return None
        pairs = tuple(self._pairs[index] for index in range(3))
        stamps = tuple(pair[1] for pair in pairs)
        oldest, newest = min(stamps), max(stamps)
        if newest > now:
            self.last_reason = "feedback_from_future"
            return None
        if now - oldest > self.max_age:
            self.last_reason = "feedback_stale"
            return None
        if newest - oldest > self.max_pair_skew:
            self.last_reason = "joint_pairs_unsynchronized"
            return None
        if self._previous is not None and any(
                current[1] <= previous[1] for current, previous in
                zip(pairs, self._previous)):
            self.last_reason = "awaiting_new_cohort"
            return None
        positions = tuple(q for angles, _ in pairs for q in angles)
        velocity = ()
        velocity_valid = False
        velocity_reason = "first_cohort_no_derivative"
        if self._previous is not None:
            derivatives = []
            times_valid = True
            for (angles, stamp), (old_angles, old_stamp) in zip(pairs, self._previous):
                dt = stamp - old_stamp
                if not self.min_velocity_dt <= dt <= self.max_velocity_dt:
                    times_valid = False
                    break
                derivatives.extend((q - old_q) / dt for q, old_q in zip(angles, old_angles))
            if not times_valid:
                velocity_reason = "derivative_interval_invalid"
            elif any(abs(value) > self.max_estimated_velocity for value in derivatives):
                velocity_reason = "derivative_rate_implausible"
            else:
                if self._filtered_velocity is not None:
                    alpha = self.velocity_alpha
                    derivatives = [alpha * value + (1. - alpha) * previous for
                                   value, previous in zip(derivatives, self._filtered_velocity)]
                velocity = tuple(derivatives)
                velocity_valid = True
                velocity_reason = "estimated_from_position_and_receive_time"
        self._filtered_velocity = velocity if velocity_valid else None
        self._previous = pairs
        self.published_snapshots += 1
        self.last_reason = "ok"
        return JointFeedbackSnapshot(positions, velocity, velocity_valid,
                                     velocity_reason, oldest, newest, newest - oldest)
