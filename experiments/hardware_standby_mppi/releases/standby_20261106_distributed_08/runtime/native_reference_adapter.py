"""Read-only native-reference handoff; no synthetic path or execution authority.

PolynomialTraj has neither a goal identifier nor an atomic bundle terminator.
Goal association uses original timestamps; a received prefix is usable only
after 30 ms quiet. This does not implement fast-to-slow prefix injection.
Six joint polynomials are sampled at the original clock phase for each MPPI
step; only samples beyond the received prefix hold its endpoint posture.
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
            enabled=k.enabled, horizon=k.H, step=k.dt, random_samples=k.random_samples)
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

    def _clear(self, reason):
        self.native.clear()
        self.arm_pieces = []
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

    def _arm_samples(self, relative_times):
        ends = np.cumsum([p[0] for p in self.arm_pieces])
        times = np.clip(np.asarray(relative_times), 0., ends[-1])
        indices = np.minimum(np.searchsorted(ends, times, side='right'), len(ends)-1)
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
            end = self.native.start + self.native.times[-1][-1]
            if now < self.native.start:
                reason = 'native_reference_not_started'
            elif now >= end:
                reason = 'native_reference_expired'
            elif now-self.native.changed < .03:
                reason = 'native_reference_quiet_interval'
            else:
                path = sampled_reference(self.native.refs, yaw)
                reason = 'native_reference_ready'
        diagnostics = dict(reference_source='native_polynomial' if self.active else 'standby',
            reference_status=reason, source_stamp=self.native.start if self.native.refs else None,
            goal_stamp=self.goal_stamp, goal_received_stamp=self.goal_received,
            message_hashes=list(self.native.messages), reference_hash=None,
            native_prefix_atomic=False, native_prefix_segments=self.native.last_id,
            fast_to_slow_prefix_enabled=False, native_arm_reference=None)
        if path is not None:
            diagnostics['reference_hash'] = fingerprint(np.column_stack((
                np.concatenate(self.native.times), path,
                self._arm_samples(np.concatenate(self.native.times)))))
            times = now-self.native.start+self.navigation.dt*np.arange(1, self.navigation.H+1)
            diagnostics['native_arm_reference'] = self._arm_samples(times).tolist()
            diagnostics['native_arm_sample_times'] = (self.native.start+times).tolist()
            diagnostics['native_arm_tail_policy'] = 'hold_received_prefix_endpoint'
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
                elif self.active and now+time.monotonic()-began >= self.native.start+self.native.times[-1][-1]:
                    result = None
                    diagnostics['reference_status'] = 'native_reference_expired_during_plan'
            return result, dict(info, **diagnostics)
