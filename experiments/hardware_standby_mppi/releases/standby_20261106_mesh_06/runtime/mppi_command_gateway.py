"""First-control gateway. Imports never open ROS, CAN or a serial device.

The solver owns an immutable input cohort and original wall-clock deadlines.
The guard uses monotonic time. Every new plan is sent ONCE, after checking the
limited command against the latest measured state. Watchdog ticks only stop;
they cannot turn an old rollout into new commands. Hardware commissioning is
separate: these sampled kinematic checks do not certify actuator dynamics.
"""
from dataclasses import dataclass
import math
import threading
import time

from hardware_command_guard import GuardConfig, HardwareCommandGuard


def finite_vector(values, n):
    v = tuple(float(x) for x in values)
    if len(v) != n or not all(math.isfinite(x) for x in v):
        raise ValueError("invalid_vector")
    return v


@dataclass(frozen=True)
class InputCohort:
    cohort_id: str
    state: tuple
    state_stamp: float
    obstacle_stamp: float
    expires_at: float
    frame_id: str

    def __post_init__(self):
        object.__setattr__(self, 'state', finite_vector(self.state, 17))


@dataclass(frozen=True)
class PlanEnvelope:
    sequence: int
    source_stamp: float
    expires_at: float
    cohort_id: str
    control: tuple

    def __post_init__(self):
        object.__setattr__(self, 'control', finite_vector(self.control, 8))


@dataclass(frozen=True)
class CommandPacket:
    sequence: int
    source_stamp: float
    expires_at: float
    base_v: float
    base_w: float
    joint_positions: tuple
    joint_velocities: tuple

    def lease_metadata(self):
        return dict(sequence=self.sequence, source_stamp=self.source_stamp,
                    expires_at=self.expires_at)

    def twist(self):
        # This is standard body-frame velocity, NOT REMANI car_cmd pose fields.
        return dict(linear=(self.base_v, 0., 0.), angular=(0., 0., self.base_w))


class RecordingSink:
    """Bounded recording sink: no hardware dependency and no device access."""
    def __init__(self, capacity=128):
        self.capacity = capacity
        self.packets = []
        self.stops = []

    def send(self, packet):
        self.packets.append(packet)
        del self.packets[:-self.capacity]

    def stop(self, reason):
        self.stops.append(str(reason))
        del self.stops[:-self.capacity]


class NativeLeaseValidator:
    """Recheck the LIMITED control, lease/braking and bounded position path.

    Uses the frozen native model. The extra position interpolation covers the
    commanded short goal as well as the velocity model; neither predicts the
    unmeasured physical Piper servo response. Keep hardware readiness blocked
    until that response, geometry and perception have been commissioned.
    """
    def __init__(self, kernel, position_horizon=.10, config=None):
        import copy
        import numpy as np
        config = config or GuardConfig(tuple(kernel.qmin), tuple(kernel.qmax),
                                      position_horizon=position_horizon)
        self.kernel = copy.copy(kernel)
        self.kernel.limits = dict(kernel.limits)
        for key, limit in [('vmax', config.max_base_v), ('wmax', config.max_base_w),
                           ('accel', config.max_base_acceleration),
                           ('angular_accel', config.max_base_angular_acceleration)]:
            self.kernel.limits[key] = min(self.kernel.limits[key], limit)
        self.kernel.qmin = np.maximum(kernel.qmin, config.joint_lower)
        self.kernel.qmax = np.minimum(kernel.qmax, config.joint_upper)
        self.kernel.vmax = np.minimum(kernel.vmax, config.joint_velocity)
        self.kernel.amax = np.minimum(kernel.amax, config.joint_acceleration)
        self.position_horizon = config.position_horizon
        self.last_min_gap = None

    def __call__(self, state, control, snapshot, wall, remaining):
        import numpy as np
        state = np.asarray(state, dtype=float)
        control = np.asarray(control, dtype=float)
        if not snapshot.fresh(wall) or not 0 < remaining <= .15 + 1e-9:
            return False
        valid, gap = self.kernel.lease_brake(
            state, control[None], snapshot, wall, lease=remaining)
        self.last_min_gap = gap
        if not valid or not math.isfinite(gap):
            return False
        horizon = min(self.position_horizon, remaining)
        dt = min(.01, remaining)
        count = int(math.ceil(remaining / dt))
        dt = remaining / count
        path = [state.copy()]
        z = state.copy()
        for j in range(1, count + 1):
            z = self.kernel.advance(z, control, dt)
            z[5:11] = state[5:11] + control[2:] * min(j * dt, horizon)
            path.append(z.copy())
        path = np.asarray(path)
        spheres = self.kernel.spheres(path)
        gaps, _ = snapshot.query(spheres, wall + dt * np.arange(count + 1))
        gaps = np.minimum(gaps, self.kernel.configuration_gap(spheres, path[:, 5:11]))
        self.last_min_gap = min(gap, float(np.min(gaps)))
        return bool(np.isfinite(gaps).all() and np.all(gaps >= 0.) and
                    np.all(path[:, 5:11] >= self.kernel.qmin - 1e-9) and
                    np.all(path[:, 5:11] <= self.kernel.qmax + 1e-9))


def system_clock():
    return time.monotonic(), time.time()


class MppiCommandGateway:
    def __init__(self, config, validator, sink, clock=system_clock, frame_id='odom'):
        self.guard = HardwareCommandGuard(config)
        self.validator, self.sink, self.clock = validator, sink, clock
        if (isinstance(validator, NativeLeaseValidator) and
                abs(validator.position_horizon-config.position_horizon) > 1e-12):
            raise ValueError('validator_position_horizon_mismatch')
        self.frame_id = frame_id
        self.lock = threading.RLock()
        self.state = None
        self.state_stamp = -math.inf
        self.enabled = False
        self.reason = 'disabled_at_startup'
        self.last_sequence = -1
        self.last_source_stamp = -math.inf
        self.source_floor = math.inf
        self.expiry = -math.inf
        self.last_now = -math.inf
        self.clock_offset = None
        self.sent = 0

    def _stop(self, reason):
        self.enabled = False
        self.reason = str(reason)
        self.expiry = -math.inf
        self.guard.set_enabled(False, max(0., self.last_now))
        try:
            self.sink.stop(self.reason)
        except Exception as exc:
            self.reason += ':stop_transport_error:' + type(exc).__name__
        return False

    def _time(self):
        now, wall = self.clock()
        if (not math.isfinite(now) or not math.isfinite(wall) or now < self.last_now
                or (self.clock_offset is not None and
                    abs(wall - now - self.clock_offset) > .02)):
            self._stop('clock_invalid')
            raise ValueError('clock_invalid')
        self.last_now, self.clock_offset = now, wall - now
        return now, wall

    def update_feedback(self, state, source_stamp):
        with self.lock:
            try:
                now, wall = self._time()
                state = finite_vector(state, 17)
                stamp = float(source_stamp)
                if (not math.isfinite(stamp) or stamp < self.state_stamp or
                        not 0. <= wall - stamp < self.guard.config.feedback_max_age):
                    raise ValueError('feedback_stale_or_replayed')
                if abs(state[3]) > self.guard.config.max_base_v + .02 or abs(state[4]) > self.guard.config.max_base_w + .04:
                    raise ValueError('base_feedback_velocity_limit')
                if not self.guard.update_measurement(state[5:11], state[11:17], now,
                                                     stamp=now - (wall - stamp)):
                    raise ValueError(self.guard.fault)
            except (ValueError, TypeError, OverflowError) as exc:
                return self._stop(str(exc))
            self.state, self.state_stamp = state, stamp
            return True

    def set_enabled(self, enabled):
        with self.lock:
            try:
                now, wall = self._time()
            except ValueError:
                return False
            if type(enabled) is not bool:
                return self._stop('invalid_enable')
            if not enabled:
                self._stop('explicit_disable')
                return True
            if self.enabled:
                return False  # A repeated enable never renews anything.
            if not self.guard.set_enabled(True, now):
                return self._stop(self.guard.fault or 'feedback_required')
            self.enabled, self.reason = True, 'awaiting_new_plan'
            self.source_floor = wall
            self.expiry = now + .15
            return True

    def submit(self, plan, cohort, snapshot):
        with self.lock:
            if not self.enabled:
                return False
            try:
                now, wall = self._time()
                if now >= self.expiry:
                    raise ValueError('command_expired')
                if not isinstance(plan, PlanEnvelope) or not isinstance(cohort, InputCohort):
                    raise ValueError('typed_plan_and_cohort_required')
                if (type(plan.sequence) is not int or not 0 <= plan.sequence <= 0xffffffff
                        or plan.sequence <= self.last_sequence):
                    raise ValueError('command_replay')
                stamps = (plan.source_stamp, plan.expires_at, cohort.state_stamp,
                          cohort.obstacle_stamp, cohort.expires_at)
                if not all(math.isfinite(x) for x in stamps):
                    raise ValueError('invalid_stamp')
                if (plan.cohort_id != cohort.cohort_id or not cohort.cohort_id or
                        cohort.frame_id != self.frame_id):
                    raise ValueError('input_cohort_mismatch')
                if (plan.source_stamp < self.source_floor or
                        plan.source_stamp <= self.last_source_stamp or
                        not 0. <= wall - plan.source_stamp <= self.guard.config.command_max_age or
                        cohort.state_stamp > plan.source_stamp or
                        cohort.obstacle_stamp > plan.source_stamp):
                    raise ValueError('plan_source_stamp_invalid')
                if (not 0. <= wall - cohort.state_stamp < self.guard.config.feedback_max_age
                        or not 0. <= wall - cohort.obstacle_stamp < .25 or
                        not snapshot.fresh(wall)):
                    raise ValueError('planning_inputs_stale')
                if self.state is None or wall - self.state_stamp >= self.guard.config.feedback_max_age:
                    raise ValueError('feedback_stale')
                # Recheck from latest feedback; reject a large departure from
                # the state that generated the candidate before any conversion.
                if (math.hypot(self.state[0]-cohort.state[0], self.state[1]-cohort.state[1]) > .05
                        or abs(math.atan2(math.sin(self.state[2]-cohort.state[2]),
                                          math.cos(self.state[2]-cohort.state[2]))) > .10
                        or max(abs(a-b) for a,b in zip(self.state[5:11], cohort.state[5:11])) > .03):
                    raise ValueError('planning_state_departed')
                deadline = min(plan.expires_at, cohort.expires_at, plan.source_stamp + .15,
                               cohort.state_stamp + self.guard.config.feedback_max_age,
                               cohort.obstacle_stamp + .25,
                               self.state_stamp + self.guard.config.feedback_max_age)
                if deadline <= wall:
                    raise ValueError('input_or_plan_expired')
                source_mono = now - (wall - plan.source_stamp)
                if not self.guard.accept_command(plan.control, plan.sequence, source_mono,
                                                  deadline-plan.source_stamp, now):
                    raise ValueError(self.guard.fault or 'guard_rejected')
                output = self.guard.tick(now)
                if not output.actuate:
                    raise ValueError(output.reason)
                control = (output.base_v, output.base_w) + output.joint_velocities
                if not self.validator(self.state, control, snapshot, wall, deadline-wall):
                    raise ValueError('limited_control_collision_rejected')
                # Collision checking takes time. Its completion cannot freshen
                # the inputs, stamp, position goal or command lease.
                send_now, send_wall = self._time()
                if send_wall >= deadline or not snapshot.fresh(send_wall):
                    raise ValueError('expired_during_revalidation')
                # Original q target may lie beyond the remaining hold horizon
                # after expensive validation. Reject rather than mutate it after
                # validation; short callbacks (<20ms) remain within this tolerance.
                if send_now-now > .020:
                    raise ValueError('revalidation_deadline_missed')
                packet = CommandPacket(plan.sequence, plan.source_stamp, deadline,
                                       output.base_v, output.base_w,
                                       output.joint_positions, output.joint_velocities)
                self.sink.send(packet)
                self.sent += 1
                self.last_sequence, self.last_source_stamp = plan.sequence, plan.source_stamp
                done_now, done_wall = self._time()
                self.expiry = min(now + deadline - wall, send_now + deadline - send_wall)
                if done_now >= self.expiry or done_wall >= deadline:
                    raise ValueError('expired_during_transport')
                self.reason = 'command_sent'
                return True
            except Exception as exc:
                return self._stop(str(exc) or type(exc).__name__)

    def tick(self):
        with self.lock:
            try:
                now, wall = self._time()
            except ValueError:
                return False
            if self.enabled:
                if now >= self.expiry:
                    self._stop('command_expired')
                elif wall-self.state_stamp >= self.guard.config.feedback_max_age:
                    self._stop('feedback_stale')
            return self.enabled

    def status(self):
        with self.lock:
            return dict(enabled=self.enabled, reason=self.reason, sent=self.sent,
                        last_sequence=self.last_sequence, hardware_motion_validated=False)
