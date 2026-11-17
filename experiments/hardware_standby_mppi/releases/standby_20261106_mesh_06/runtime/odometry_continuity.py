"""ROS-independent, latched rejection of stale or discontinuous odometry.

This checks pose changes directly. A downstream bridge may have clipped its
reported twist; such a twist is not evidence that a pose jump is plausible.
There is deliberately no reset method. Recovery requires a new gate instance
after the localization source has been stopped, diagnosed, and restarted.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class OdometryGateConfig:
    max_age_s: float = 0.25
    future_tolerance_s: float = 0.02
    max_gap_s: float = 0.5
    max_linear_speed_m_s: float = 1.0
    position_noise_m: float = 0.02
    max_angular_speed_rad_s: float = 3.0
    angle_noise_rad: float = 0.05
    initial_origin_xy: tuple = (0.0, 0.0)
    max_initial_radius_m: float = 2.0

    def __post_init__(self):
        for name in ('max_age_s', 'future_tolerance_s', 'max_gap_s',
                     'max_linear_speed_m_s', 'position_noise_m',
                     'max_angular_speed_rad_s', 'angle_noise_rad',
                     'max_initial_radius_m'):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0:
                raise ValueError('invalid_odometry_gate_config_' + name)
        if len(self.initial_origin_xy) != 2 or not all(
                math.isfinite(float(x)) for x in self.initial_origin_xy):
            raise ValueError('invalid_odometry_gate_initial_origin')


class OdometryContinuityGate:
    def __init__(self, config=None):
        self.config = config or OdometryGateConfig()
        self.last = None
        self.fault = None
        self.accepted = 0

    def reject(self, reason):
        if self.fault is None:
            self.fault = reason
        raise ValueError('odometry_requires_restart:' + self.fault)

    def validate(self, stamp, now_ros, now_monotonic, position, quaternion):
        if self.fault is not None:
            self.reject(self.fault)
        c = self.config
        try:
            position = tuple(float(v) for v in position)
            quaternion = tuple(float(v) for v in quaternion)
            clocks = tuple(float(v) for v in (stamp, now_ros, now_monotonic))
        except (TypeError, ValueError, OverflowError):
            self.reject('invalid_numeric_input')
        if len(position) != 3 or len(quaternion) != 4 or not all(
                math.isfinite(v) for v in clocks + position + quaternion):
            self.reject('invalid_numeric_input')
        stamp, now_ros, now_monotonic = clocks
        if stamp <= 0 or now_ros <= 0 or now_monotonic < 0:
            self.reject('invalid_clock')
        age = now_ros - stamp
        if age > c.max_age_s or age < -c.future_tolerance_s:
            self.reject('stale_or_future_measurement')
        norm = math.sqrt(sum(v*v for v in quaternion))
        if abs(norm - 1.0) > 0.01:
            self.reject('invalid_quaternion_norm')
        quaternion = tuple(v/norm for v in quaternion)
        if self.last is None:
            ox, oy = c.initial_origin_xy
            if math.hypot(position[0]-ox, position[1]-oy) > c.max_initial_radius_m:
                self.reject('initial_pose_outside_local_origin')
        else:
            old_stamp, old_receipt, old_position, old_quaternion = self.last
            dt = stamp-old_stamp
            receipt_dt = now_monotonic-old_receipt
            if dt <= 0:
                self.reject('nonmonotonic_measurement_stamp')
            if receipt_dt < 0:
                self.reject('monotonic_clock_regression')
            if dt > c.max_gap_s or receipt_dt > c.max_gap_s:
                self.reject('measurement_or_receipt_gap')
            distance = math.sqrt(sum((a-b)**2 for a, b in zip(position, old_position)))
            if distance > c.max_linear_speed_m_s*dt+c.position_noise_m:
                self.reject('position_jump')
            # abs(dot) makes q and -q identical and handles the yaw wrap.
            dot = abs(sum(a*b for a, b in zip(quaternion, old_quaternion)))
            angle = 2.0*math.acos(min(1.0, max(0.0, dot)))
            if angle > c.max_angular_speed_rad_s*dt+c.angle_noise_rad:
                self.reject('orientation_jump')
        self.last = (stamp, now_monotonic, position, quaternion)
        self.accepted += 1
        return True

    def status(self):
        return dict(accepted_samples=self.accepted, requires_restart=self.fault is not None,
                    fault=self.fault, validation='direct_pose_and_measurement_receipt_continuity')

    def expire(self, now_monotonic):
        """Latch absence of fresh receipt after the source first became valid."""
        if self.fault is not None:
            self.reject(self.fault)
        if self.last is not None and float(now_monotonic)-self.last[1] > self.config.max_gap_s:
            self.reject('measurement_receipt_timeout')
