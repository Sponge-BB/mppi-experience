"""Read-only native-reference handoff; no synthetic path or execution authority.

PolynomialTraj has neither a goal identifier nor an atomic bundle terminator.
Goal association uses original timestamps; a received prefix is usable only
after 30 ms quiet. This does not implement fast-to-slow prefix injection.
Base and arm use the original controller's validated virtual trajectory phase.
A progress scale change slows the reference clock, not the actuator. It is a
soft tracking update, not a trajectory revision or a safety-stop command. The
next cycle samples the updated phase; delivery still checks the actual command
against current feedback, collision prediction and explicit safety speed limits.
A reference is limited to its current gear block. Joint samples reaching that
block's end hold its endpoint until controller progress enters the next block.
"""
import threading
import time
import numpy as np

from active_arm_mppi import WholeBodyMPPI
from native_reference import NativeReference, fingerprint, sampled_reference


class NativeReferenceAdapter:
    def __init__(self, standby_kernel, navigation_kernel=None):
        self.standby = standby_kernel
        k = standby_kernel
        self.navigation = navigation_kernel if navigation_kernel is not None else WholeBodyMPPI(
            k.model, k.limits, (k.qmin, k.qmax, k.vmax, k.amax), k.postures,
            enabled=k.enabled, seed=k.seed, horizon=k.H, step=k.dt,
            random_samples=k.random_samples, arm_position_servo=k.arm_position_servo)
        self.native = NativeReference()
        self.lock = threading.RLock()
        self.plan_lock = threading.Lock()
        self.active = False
        self.goal_stamp = self.goal_received = None
        self.min_start = self.last_start = 0.
        self.arm_pieces = []
        self.revision = 0
        self.planned_revision = -1
        self.reason = 'standby_no_navigation_goal'
        self.progress = None
        self.progress_max_age = .25
        # ROS /clock and progress are separate transports; one 5 ms clock tick
        # can arrive after its progress message. Match the feedback skew budget.
        self.progress_clock_skew = .005 + 1e-8

    def _clear(self, reason):
        self.native.clear()
        self.arm_pieces = []
        self.progress = None
        self.standby.previous = self.navigation.previous = None
        self.reason = reason
        self.revision += 1

    def feed_goal(self, msg, now):
        with self.lock:
            try:
                stamp = float(msg.header.stamp.to_sec())
                p, q = msg.pose.position, msg.pose.orientation
                values = [now, stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w]
                if (not np.isfinite(values).all() or stamp <= 0 or stamp > now+.05 or
                        msg.header.frame_id != 'world' or
                        abs(np.linalg.norm([q.x, q.y, q.z, q.w])-1.) > .01):
                    raise ValueError('invalid_goal')
                if self.goal_stamp is not None and stamp <= self.goal_stamp:
                    return 'stale_goal'
            except (AttributeError, TypeError, ValueError, OverflowError):
                self.active = True
                self.goal_stamp = self.goal_received = None
                self._clear('invalid_goal')
                return 'invalid'
            self.active = True
            self.goal_stamp, self.goal_received = stamp, float(now)
            self.min_start, self.last_start = max(stamp, now), 0.
            self._clear('waiting_native_reference_after_goal')
            return 'accepted'

    def finish(self):
        """Return to measured-pose standby after the planner reports arrival."""
        with self.lock:
            self.active = False
            self.goal_stamp = self.goal_received = None
            self.min_start = self.last_start = 0.
            self._clear('standby_after_finish')

    def feed_progress(self, msg, now):
        """Consume mm_controller's {start, virtual seconds, time scale}."""
        with self.lock:
            if not self.active or not self.native.refs:
                return 'ignored_without_native_reference'
            try:
                stamp = float(msg.header.stamp.to_sec())
                start, phase, scale = (float(msg.vector.x), float(msg.vector.y), float(msg.vector.z))
                if not np.isfinite([now, stamp, start, phase, scale]).all():
                    raise ValueError('invalid_native_progress')
                if abs(start-self.native.start) > 1e-6:
                    return 'stale_progress'
                if self.progress is not None and stamp <= self.progress[0]:
                    return 'stale_progress'
                if not -self.progress_clock_skew <= now-stamp <= self.progress_max_age:
                    raise ValueError('native_progress_stale')
                if phase < 0. or phase > 300. or not 0. <= scale <= 1.:
                    raise ValueError('invalid_native_progress')
                if self.progress is not None and phase+1e-6 < self.progress[1]:
                    raise ValueError('native_progress_reversed')
            except (AttributeError, TypeError, ValueError, OverflowError) as exc:
                self.progress = None
                self.reason = str(exc)
                self.revision += 1
                return 'invalid'
            # Ordinary progress arrives at 100 Hz; it is not a new trajectory.
            # Revisions invalidate plans only on goal/trajectory changes/errors.
            self.progress = (stamp, phase, scale)
            return 'accepted'

    def _phase(self, now):
        if self.progress is None:
            reason = (self.reason if self.reason in ('invalid_native_progress',
                      'native_progress_stale', 'native_progress_reversed') else 'waiting_native_progress')
            return None, None, reason
        stamp, phase, scale = self.progress
        if not -self.progress_clock_skew <= now-stamp <= self.progress_max_age:
            return None, None, 'native_progress_stale'
        # Back-project the bounded clock skew rather than advance a future phase.
        return max(0., phase+scale*(now-stamp)), scale, None

    def _base_reference(self, phase, scale, yaw):
        ends = np.asarray([t[-1] for t in self.native.times])
        block = int(np.searchsorted(ends, phase, side='right'))
        times = self.native.times[block]
        path = sampled_reference([self.native.refs[block]], yaw)
        # Adjacent polynomial pieces both contain their common endpoint.
        unique = np.r_[np.diff(times) > 1e-12, True]
        times, path = times[unique], path[unique]
        current = np.array([np.interp(phase, times, path[:, i]) for i in (0, 1, 3)])
        angle = np.interp(phase, times, np.unwrap(path[:, 2]))
        current = np.array([current[0], current[1], (angle+np.pi) % (2*np.pi)-np.pi,
                            current[2]])
        if scale == 0.:
            # Paused means hold the current reference pose, not follow the
            # remaining spatial curve (whose guide has a minimum speed).
            path = np.repeat(current[None], 2, axis=0)
        else:
            path = np.vstack((current, path[times > phase+1e-12]))
        path[:, 3] *= scale
        return path, block, float(ends[block])

    def _arm_samples(self, relative_times):
        ends = np.cumsum([p[0] for p in self.arm_pieces])
        times = np.clip(np.asarray(relative_times), 0., ends[-1])
        # At an exact block boundary, retain the ending piece rather than
        # borrowing the next gear's first joint sample for endpoint holding.
        indices = np.minimum(np.searchsorted(ends, times, side='left'), len(ends)-1)
        local = times-np.r_[0., ends[:-1]][indices]
        coefficients = np.asarray([p[1] for p in self.arm_pieces])[indices]
        q = np.zeros((len(times), 6))
        for power in range(8):
            q = q*local[:, None]+coefficients[:, :, power]
        if not np.isfinite(q).all():
            raise ValueError('nonfinite_native_arm_samples')
        return q

    def feed_trajectory(self, msg, now):
        with self.lock:
            if not self.active or self.goal_stamp is None:
                return 'ignored_without_valid_goal'
            if msg.action in (3, 4):
                return 'notice'
            if msg.action in (2, 5):
                self.min_start = max(self.min_start, now)
                self._clear('native_reference_cancelled')
                return 'cancel'
            try:
                stamp = float(msg.header.stamp.to_sec())
                if (not np.isfinite(now) or now < self.goal_received or
                        stamp < self.min_start or
                        (msg.trajectory_id == 1 and stamp <= self.last_start)):
                    return 'stale_reference'
                result = self.native.feed(msg, now)
                if result == 'invalid':
                    raise ValueError(self.native.error)
                if result == 'replace':
                    self.arm_pieces = []
                    self.progress = None
                    self.standby.previous = self.navigation.previous = None
                self.arm_pieces.extend((p.duration, np.asarray(p.data).reshape((8, 8), order='F')[2:].copy())
                                       for p in msg.trajectory)
                if self.native.times[-1][-1] > 300. or len(self.arm_pieces) > 1000:
                    raise ValueError('native_prefix_exceeds_budget')
                self.last_start = self.native.start
            except (AttributeError, TypeError, ValueError, OverflowError) as exc:
                self._clear(str(exc))
                return 'invalid'
            self.revision += 1
            self.reason = 'native_reference_quiet_interval'
            return result

    def _reference(self, yaw, now):
        reason = self.reason
        path = None
        if self.native.refs:
            phase, scale, progress_error = self._phase(now)
            if now < self.native.start:
                reason = 'native_reference_not_started'
            elif now-self.native.changed < .03:
                reason = 'native_reference_quiet_interval'
            elif progress_error is not None:
                reason = progress_error
            elif phase >= self.native.times[-1][-1]:
                reason = 'native_reference_expired'
            else:
                path, block, block_end = self._base_reference(phase, scale, yaw)
                reason = 'native_reference_ready'
        diagnostics = dict(reference_source='native_polynomial' if self.active else 'standby',
            reference_status=reason, source_stamp=self.native.start if self.native.refs else None,
            goal_stamp=self.goal_stamp, goal_received_stamp=self.goal_received,
            message_hashes=list(self.native.messages), reference_hash=None,
            native_prefix_atomic=False, native_prefix_segments=self.native.last_id,
            fast_to_slow_prefix_enabled=False, native_arm_reference=None,
            native_phase=None, native_time_scale=None, native_gear_block=None)
        if path is not None:
            entire_path = sampled_reference(self.native.refs, yaw)
            diagnostics['reference_hash'] = fingerprint(np.column_stack((
                np.concatenate(self.native.times), entire_path,
                self._arm_samples(np.concatenate(self.native.times)))))
            times = np.minimum(block_end, phase+scale*self.navigation.dt*np.arange(1, self.navigation.H+1))
            diagnostics['native_arm_reference'] = self._arm_samples(times).tolist()
            diagnostics['native_arm_sample_phases'] = times.tolist()
            diagnostics['native_phase'] = phase
            diagnostics['native_time_scale'] = scale
            diagnostics['native_progress_stamp'] = self.progress[0]
            diagnostics['native_gear_block'] = block
            diagnostics['native_gear'] = self.native.gears[block]
            diagnostics['native_block_end_phase'] = block_end
            diagnostics['native_arm_tail_policy'] = 'hold_current_gear_block_endpoint'
        return path, diagnostics

    def snapshot_path(self, yaw, now):
        """Return a fresh owned N x 4 sampled path, or None; never restamp it."""
        with self.lock:
            return self._reference(yaw, now)[0]

    def status(self, now, yaw=0.):
        with self.lock:
            return self._reference(yaw, now)[1]

    def plan(self, state, reference, arm_reference, snapshot, now):
        began = time.monotonic()
        with self.plan_lock:
            with self.lock:
                revision = self.revision
                path, diagnostics = self._reference(float(state[2]), now)
                if self.planned_revision != revision:
                    self.standby.previous = self.navigation.previous = None
                    self.planned_revision = revision
                if self.active and path is None:
                    return None, diagnostics
                kernel = self.navigation if self.active else self.standby
                if self.active:
                    reference, arm_reference = path, np.asarray(diagnostics['native_arm_reference'])
            result, info = kernel.plan(state, reference, arm_reference, snapshot, now)
            with self.lock:
                if revision != self.revision:
                    kernel.previous = None
                    result = None
                    diagnostics['reference_status'] = 'reference_changed_during_plan'
                elif self.active:
                    phase, scale, error = self._phase(now+time.monotonic()-began)
                    if error is not None:
                        result = None
                        diagnostics['reference_status'] = error+'_during_plan'
                    elif phase >= self.native.times[-1][-1]:
                        result = None
                        diagnostics['reference_status'] = 'native_reference_expired_during_plan'
                    elif int(np.searchsorted([t[-1] for t in self.native.times], phase, side='right')) != diagnostics['native_gear_block']:
                        result = None
                        diagnostics['reference_status'] = 'native_gear_changed_during_plan'
                    # A falling scale lets lagging feedback catch the reference.
                    # Rejecting each 100 Hz scale update would instead stop the
                    # tracker and increase that lag. Keep this sampled objective;
                    # collision times remain wall/ROS time, never virtual phase.
                if result is None:
                    kernel.previous = None
            return result, dict(info, **diagnostics)
