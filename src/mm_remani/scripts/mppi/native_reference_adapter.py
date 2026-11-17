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
    def __init__(self, standby_kernel, navigation_kernel=None, clock=None, resume_distance=.05,
                 yaw_hold_speed=.04):
        self.standby = standby_kernel
        k = standby_kernel
        self.navigation = navigation_kernel if navigation_kernel is not None else WholeBodyMPPI(
            k.model, k.limits, (k.qmin, k.qmax, k.vmax, k.amax), k.postures,
            enabled=k.enabled, seed=k.seed, horizon=k.H, step=k.dt,
            random_samples=k.random_samples, arm_position_servo=k.arm_position_servo)
        self.native = NativeReference()
        self.lock = threading.RLock()
        self.plan_lock = threading.Lock()
        # ROS/sim time can advance differently from the wall-time solve budget.
        # The ROS node supplies its actual clock for source-age checks.
        self.clock = clock
        self.resume_distance = float(resume_distance)
        if not np.isfinite(self.resume_distance) or self.resume_distance < 0.:
            raise ValueError('invalid native base recovery distance')
        self.yaw_hold_speed = float(yaw_hold_speed)
        if not np.isfinite(self.yaw_hold_speed) or self.yaw_hold_speed < 0.:
            raise ValueError('invalid native yaw hold speed')
        self.active = False
        self.goal_stamp = self.goal_received = None
        self.goal_pose = None
        self.min_start = self.last_start = 0.
        self.arm_pieces = []
        self.revision = 0
        self.planned_revision = -1
        self.reason = 'standby_no_navigation_goal'
        self.progress = None
        self.pending = None
        self.progress_max_age = .25
        # ROS /clock and progress are separate transports; one 5 ms clock tick
        # can arrive after its progress message. Match the feedback skew budget.
        self.progress_clock_skew = .005 + 1e-8
        self.terminal_window = None

    def _clear(self, reason):
        self.native.clear()
        self.arm_pieces = []
        self.progress = None
        self.pending = None
        self.terminal_window = None
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
                self.goal_pose = None
                self._clear('invalid_goal')
                return 'invalid'
            self.active = True
            self.goal_stamp, self.goal_received = stamp, float(now)
            norm_sq = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
            yaw = np.arctan2(2.*(q.w*q.z+q.x*q.y), norm_sq-2.*(q.y*q.y+q.z*q.z))
            self.goal_pose = np.array([p.x, p.y, yaw])
            self.min_start, self.last_start = max(stamp, now), 0.
            self._clear('waiting_native_reference_after_goal')
            return 'accepted'

    def finish(self):
        """Return to measured-pose standby after the planner reports arrival."""
        with self.lock:
            self.active = False
            self.goal_stamp = self.goal_received = None
            self.goal_pose = None
            self.min_start = self.last_start = 0.
            self._clear('standby_after_finish')

    def feed_progress(self, msg, now):
        """Consume mm_controller's {start, virtual seconds, time scale}."""
        with self.lock:
            if not self.active or not (self.native.refs or self.pending is not None):
                return 'ignored_without_native_reference'
            pending = None
            try:
                stamp = float(msg.header.stamp.to_sec())
                start, phase, scale = (float(msg.vector.x), float(msg.vector.y), float(msg.vector.z))
                if not np.isfinite([now, stamp, start, phase, scale]).all():
                    raise ValueError('invalid_native_progress')
                if self.pending is not None and abs(start-self.pending['native'].start) <= 1e-6:
                    pending = self.pending
                elif abs(start-self.native.start) > 1e-6 or not self.native.refs:
                    return 'stale_progress'
                previous = pending['progress'] if pending is not None else self.progress
                if previous is not None and stamp <= previous[0]:
                    return 'stale_progress'
                if not -self.progress_clock_skew <= now-stamp <= self.progress_max_age:
                    raise ValueError('native_progress_stale')
                if phase < 0. or phase > 300. or not 0. <= scale <= 1.:
                    raise ValueError('invalid_native_progress')
                if previous is not None and phase+1e-6 < previous[1]:
                    raise ValueError('native_progress_reversed')
            except (AttributeError, TypeError, ValueError, OverflowError) as exc:
                if pending is not None:
                    pending['progress'] = None
                    pending['reason'] = str(exc)
                else:
                    self.progress = None
                    self.reason = str(exc)
                    self.revision += 1
                return 'invalid'
            # Ordinary progress arrives at 100 Hz; it is not a new trajectory.
            # Revisions invalidate plans only on goal/trajectory changes/errors.
            if pending is not None:
                pending['progress'] = (stamp, phase, scale)
                pending['reason'] = 'waiting_native_progress'
                self._activate_pending(now)
            else:
                self.progress = (stamp, phase, scale)
            return 'accepted'

    def _pending_phase_error(self, now):
        progress = self.pending['progress']
        if progress is None:
            return self.pending['reason']
        if not -self.progress_clock_skew <= now-progress[0] <= self.progress_max_age:
            return 'native_progress_stale'
        return None

    def _activate_pending(self, now):
        """Swap base, joints and progress as one reference at its activation."""
        pending = self.pending
        if (pending is None or now < pending['native'].start or
                now-pending['native'].changed < .03 or self._pending_phase_error(now) is not None):
            return False
        self.native, self.arm_pieces, self.progress = (
            pending['native'], pending['arm_pieces'], pending['progress'])
        self.pending = None
        self.standby.previous = self.navigation.previous = None
        self.terminal_window = None
        self.reason = 'native_reference_ready'
        self.revision += 1
        return True

    def _phase(self, now):
        if self.progress is None:
            reason = (self.reason if self.reason in ('invalid_native_progress',
                      'native_progress_stale', 'native_progress_reversed') else 'waiting_native_progress')
            return None, None, reason
        stamp, phase, scale = self.progress
        if not -self.progress_clock_skew <= now-stamp <= self.progress_max_age:
            # mm_controller changes to the future start immediately. Preserve
            # the old phase only until that activation, backed by fresh progress
            # from the pending reference; never apply its phase to the old curve.
            bridge = (now >= stamp and self.pending is not None and
                      now < self.pending['native'].start and
                      self._pending_phase_error(now) is None)
            if not bridge:
                return None, None, 'native_progress_stale'
        # Back-project the bounded clock skew rather than advance a future phase.
        return max(0., phase+scale*(now-stamp)), scale, None

    def _base_reference(self, phase, scale, yaw):
        # Spatial preview for display/status only. Execution and scoring use
        # the exact timed samples below, never this resampled remaining path.
        ends = np.asarray([t[-1] for t in self.native.times])
        # Native SingulTrajData advances blocks only when t > duration.
        block = int(np.searchsorted(ends, phase, side='left'))
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

    def _base_samples(self, phase, scale, yaw, block):
        """Exact same-phase base pose/feedforward for every rollout step.

        Columns are x, y, yaw, signed speed, angular speed, unscaled speed.
        Curvature and yaw use the original derivative, so clock slowdown never
        changes the heading target. Beyond this gear block, hold its endpoint;
        only the native clock can authorize the following block.
        """
        pieces = self.native.polynomials[block]
        ends = np.cumsum([p[0] for p in pieces])
        start = 0. if block == 0 else self.native.times[block-1][-1]
        phases = phase + scale*self.navigation.dt*np.arange(self.navigation.H+1)
        local_phases = np.clip(phases-start, 0., ends[-1])
        indices = np.minimum(np.searchsorted(ends, local_phases, side='left'), len(ends)-1)
        local = local_phases-np.r_[0., ends[:-1]][indices]
        coefficients = np.asarray([p[1] for p in pieces])[indices]
        p = np.zeros((len(local), 2)); v = p.copy(); a = p.copy()
        for power in range(8):
            p = p*local[:, None]+coefficients[:, :, power]
            if power < 7:
                v = v*local[:, None]+(7-power)*coefficients[:, :, power]
            if power < 6:
                a = a*local[:, None]+(7-power)*(6-power)*coefficients[:, :, power]
        speed = np.linalg.norm(v, axis=1)
        defined = speed > self.yaw_hold_speed
        gear = self.native.gears[block]
        heading = np.where(defined, np.arctan2(gear*v[:, 1], gear*v[:, 0]), yaw)
        omega = np.zeros(len(local))
        omega[defined] = scale*(v[defined, 0]*a[defined, 1]-v[defined, 1]*a[defined, 0])/speed[defined]**2
        signed_speed = gear*scale*speed
        at_end = phases >= start+ends[-1]
        signed_speed[at_end] = 0.; omega[at_end] = 0.
        samples = np.c_[p, heading, signed_speed, omega, speed]
        if not np.isfinite(samples).all():
            raise ValueError('nonfinite_native_base_samples')
        return samples

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
                future = msg.trajectory_id == 1 and stamp > now
                if future:
                    target = dict(native=NativeReference(), arm_pieces=[], progress=None,
                                  reason='waiting_native_progress')
                elif (msg.trajectory_id != 1 and self.pending is not None and
                      abs(stamp-self.pending['native'].start) <= 1e-6):
                    target = self.pending
                else:
                    target = None
                    if msg.trajectory_id != 1 and stamp != self.native.start:
                        return 'stale_reference'
                native = self.native if target is None else target['native']
                result = native.feed(msg, now)
                if result == 'invalid':
                    raise ValueError(native.error)
                pieces = [(p.duration, np.asarray(p.data).reshape((8, 8), order='F')[2:].copy())
                          for p in msg.trajectory]
                if native.times[-1][-1] > 300.:
                    raise ValueError('native_prefix_exceeds_budget')
                if target is not None:
                    target['arm_pieces'].extend(pieces)
                    if len(target['arm_pieces']) > 1000:
                        raise ValueError('native_prefix_exceeds_budget')
                    self.pending = target
                    self.last_start = max(self.last_start, native.start)
                    self.terminal_window = None
                    # Receiving future pieces does not invalidate an otherwise
                    # executable old reference. Activation has its own revision.
                    self._activate_pending(now)
                    return result
                if result == 'replace':
                    self.arm_pieces = []
                    self.progress = None
                    self.pending = None
                    self.standby.previous = self.navigation.previous = None
                self.arm_pieces.extend(pieces)
                if self.native.times[-1][-1] > 300. or len(self.arm_pieces) > 1000:
                    raise ValueError('native_prefix_exceeds_budget')
                self.last_start = self.native.start
                self.terminal_window = None
            except (AttributeError, TypeError, ValueError, OverflowError) as exc:
                self._clear(str(exc))
                return 'invalid'
            self.revision += 1
            self.reason = 'native_reference_quiet_interval'
            return result

    def _reference(self, yaw, now):
        self._activate_pending(now)
        reason = self.reason
        path = None
        pending_due = self.pending is not None and now >= self.pending['native'].start
        if pending_due:
            reason = ('native_reference_quiet_interval' if now-self.pending['native'].changed < .03
                      else self._pending_phase_error(now))
        elif self.native.refs:
            phase, scale, progress_error = self._phase(now)
            if now < self.native.start:
                reason = 'native_reference_not_started'
            elif now-self.native.changed < .03:
                reason = 'native_reference_quiet_interval'
            elif progress_error is not None:
                reason = progress_error
            # Match mm_controller's completion tolerance: its progress can
            # stop just below the exact endpoint when it switches to STAY.
            elif phase >= self.native.times[-1][-1]-1e-6:
                reason = 'native_reference_expired'
            else:
                path, block, block_end = self._base_reference(phase, scale, yaw)
                reason = 'native_reference_ready'
        elif self.pending is not None:
            reason = 'native_reference_not_started'
        diagnostics = dict(reference_source='native_polynomial' if self.active else 'standby',
            reference_status=reason, source_stamp=self.native.start if self.native.refs else None,
            goal_stamp=self.goal_stamp, goal_received_stamp=self.goal_received,
            message_hashes=list(self.native.messages), reference_hash=None,
            native_prefix_atomic=False, native_prefix_segments=self.native.last_id,
            fast_to_slow_prefix_enabled=False, native_arm_reference=None,
            native_phase=None, native_time_scale=None, native_gear_block=None,
            reference_revision=self.revision,
            pending_start_stamp=self.pending['native'].start if self.pending is not None else None)
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
            diagnostics['native_phase_source'] = ('handoff_bridge' if now-self.progress[0] > self.progress_max_age
                                                  else 'native_progress')
            diagnostics['native_gear_block'] = block
            diagnostics['native_gear'] = self.native.gears[block]
            diagnostics['native_base_reference_mode'] = 'timed_native_pose'
            diagnostics['native_base_resume_distance'] = self.resume_distance
            diagnostics['native_base_samples'] = self._base_samples(phase, scale, yaw, block).tolist()
            diagnostics['native_yaw_hold_speed'] = self.yaw_hold_speed
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

    def terminal_pose_command(self, state, now, config):
        """Bounded yaw/joint settling at this goal's expired endpoint.

        The caller still owns current-state/map, stopping-envelope and execution
        checks. This does not renew the expired path or mark the goal finished.
        """
        with self.lock:
            state = np.asarray(state, dtype=float)
            if (not self.active or self.goal_stamp is None or self.goal_pose is None
                    or self.pending is not None or not self.native.refs or not self.arm_pieces
                    or state.shape != (17,) or not np.isfinite(state).all()
                    or now < self.native.start or now-self.native.changed < .03):
                return None
            phase, _, error = self._phase(now)
            end = self.native.times[-1][-1]
            if error is not None or phase < end-1e-6:
                return None
            # Segment identity is stricter than the measured arrival tolerance:
            # a local repair ending near the goal must not claim final alignment.
            if np.linalg.norm(self.native.refs[-1][-1, :2]-self.goal_pose[:2]) >= .01:
                return None
            arm_end = self._arm_samples(np.array([end]))[0]
            if (np.linalg.norm(state[:2]-self.goal_pose[:2]) > config['position_tolerance']
                    or np.max(np.abs(state[5:11]-arm_end)) > max(.30, config['arm_tolerance'])):
                return None
            identity = (self.goal_stamp, self.native.start, self.native.epoch, self.native.last_id)
            if self.terminal_window is None or self.terminal_window[0] != identity:
                self.terminal_window = (identity, now)
            elapsed = now-self.terminal_window[1]
            if elapsed < 0. or elapsed >= config['timeout']:
                return None
            error = (self.goal_pose[2]-state[2]+np.pi) % (2*np.pi)-np.pi
            command = np.zeros(8)
            if abs(error) > config['yaw_tolerance']:
                command[1] = np.clip(config['gain']*error, -config['max_w'], config['max_w'])
            kernel = self.navigation
            if kernel.enabled:
                arm_error = arm_end-state[5:11]
                velocity = np.clip(1.8*arm_error-.2*state[11:17], -kernel.vmax, kernel.vmax)
                lead = .12+.5*kernel.dt if kernel.arm_position_servo else kernel.dt
                # Match the target sent by checked_first_step in either arm
                # model. Stay inside the node's strict .03 rad feedback-lead
                # gate, including subtraction roundoff at nonzero joint poses.
                delta = np.clip(lead*velocity, np.minimum(arm_error, 0.), np.maximum(arm_error, 0.))
                delta = np.clip(delta, np.maximum(kernel.qmin-state[5:11], -.02),
                                np.minimum(kernel.qmax-state[5:11], .02))
                command[2:] = delta/lead
            return command

    def plan(self, state, reference, arm_reference, snapshot, now):
        began = time.monotonic()
        with self.plan_lock:
            with self.lock:
                # Progress can arrive after the caller captured its cycle time
                # but before these locks are acquired. Compare its source stamp
                # with the actual clock, and anchor this rollout to that same
                # time. Never clamp a clock rollback or extend source validity.
                plan_now = self.clock() if self.clock is not None else now
                path, diagnostics = self._reference(float(state[2]), plan_now)
                revision = self.revision
                if self.planned_revision != revision:
                    self.standby.previous = self.navigation.previous = None
                    self.planned_revision = revision
                if self.active and path is None:
                    return None, diagnostics
                kernel = self.navigation if self.active else self.standby
                tracking = None
                if self.active:
                    samples = np.asarray(diagnostics['native_base_samples'])
                    reference, arm_reference = samples[:, :4], np.asarray(diagnostics['native_arm_reference'])
                    tracking = dict(gear=diagnostics['native_gear'], resume_distance=self.resume_distance,
                        identity=(self.goal_stamp, self.native.start, self.native.epoch, self.revision,
                                  diagnostics['native_gear_block'], diagnostics['native_gear']),
                        base_samples=samples, yaw_hold_speed=self.yaw_hold_speed)
            result, info = kernel.plan(state, reference, arm_reference, snapshot, plan_now,
                                      **({'tracking': tracking} if tracking is not None else {}))
            with self.lock:
                if revision != self.revision:
                    kernel.previous = None
                    result = None
                    diagnostics['reference_status'] = 'reference_changed_during_plan'
                elif self.active:
                    checked_now = self.clock() if self.clock is not None else now+time.monotonic()-began
                    if self.pending is not None and checked_now >= self.pending['native'].start:
                        kernel.previous = None
                        diagnostics['reference_status'] = 'reference_changed_during_plan'
                        return None, dict(info, **diagnostics)
                    phase, scale, error = self._phase(checked_now)
                    if error is not None:
                        result = None
                        diagnostics['reference_status'] = error+'_during_plan'
                    elif phase >= self.native.times[-1][-1]:
                        result = None
                        diagnostics['reference_status'] = 'native_reference_expired_during_plan'
                    elif int(np.searchsorted([t[-1] for t in self.native.times], phase, side='left')) != diagnostics['native_gear_block']:
                        result = None
                        diagnostics['reference_status'] = 'native_gear_changed_during_plan'
                    # A falling scale lets lagging feedback catch the reference.
                    # Rejecting each 100 Hz scale update would instead stop the
                    # tracker and increase that lag. Keep this sampled objective;
                    # collision times remain wall/ROS time, never virtual phase.
                if result is None:
                    kernel.previous = None
            return result, dict(info, **diagnostics)
