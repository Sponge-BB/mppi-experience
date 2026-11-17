"""Whole-body sampling controller using REMANI geometry and observations.

The arm geometry is recomputed for every predicted configuration.  The native
ESDF and observed dynamic points are inputs; scenario truth is never imported.
Units are metres, seconds and radians.  This is a kinematic/lagged-velocity
model, not a torque controller or a continuous-time safety certificate.
"""
from dataclasses import dataclass
import ctypes
import math
from pathlib import Path
import numpy as np
from scipy.ndimage import map_coordinates
from scipy.spatial import cKDTree


def _load_position_servo_library():
    """Optional small kernel; legacy velocity mode never calls this loader."""
    paths = (Path(__file__).resolve().parents[4]/'devel/lib/libmppi_position_servo.so',
             'libmppi_position_servo.so')
    for path in paths:
        try:
            library = ctypes.CDLL(str(path))
            function = library.mppi_position_servo
            vector = np.ctypeslib.ndpointer(dtype=np.float64, ndim=1, shape=(6,), flags='C_CONTIGUOUS')
            function.argtypes = [np.ctypeslib.ndpointer(dtype=np.float64, ndim=2,
                flags=('C_CONTIGUOUS', 'WRITEABLE')), ctypes.c_size_t, ctypes.c_void_p,
                vector, vector, vector, vector, ctypes.c_double]
            function.restype = ctypes.c_int
            # An older library remains usable through its arm-only ABI.
            function.whole_body = getattr(library, 'mppi_whole_body_servo', None)
            if function.whole_body is not None:
                function.whole_body.argtypes = [function.argtypes[0], ctypes.c_size_t,
                    ctypes.c_void_p, ctypes.c_void_p,
                    np.ctypeslib.ndpointer(dtype=np.float64, ndim=1, shape=(7,), flags='C_CONTIGUOUS'),
                    vector, vector, vector, vector, ctypes.c_double]
                function.whole_body.restype = ctypes.c_int
            return function
        except (OSError, AttributeError):
            continue
    return None


def _load_legacy_velocity_library():
    """Optional equivalent velocity integrator; old/missing libraries fall back."""
    paths = (Path(__file__).resolve().parents[4]/'devel/lib/libmppi_position_servo.so',
             'libmppi_position_servo.so')
    for path in paths:
        try:
            function = ctypes.CDLL(str(path)).mppi_whole_body_legacy_velocity
            vector = np.ctypeslib.ndpointer(dtype=np.float64, ndim=1, shape=(6,), flags='C_CONTIGUOUS')
            function.argtypes = [np.ctypeslib.ndpointer(dtype=np.float64, ndim=2,
                flags=('C_CONTIGUOUS', 'WRITEABLE')), ctypes.c_size_t, ctypes.c_void_p,
                np.ctypeslib.ndpointer(dtype=np.float64, ndim=1, shape=(7,), flags='C_CONTIGUOUS'),
                vector, vector, ctypes.c_double]
            function.restype = ctypes.c_int
            return function
        except (OSError, AttributeError):
            continue
    return None


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class SnapshotCacheMiss(ValueError):
    """A valid delta needs a full field; this is not a tracking fault."""


@dataclass
class Snapshot:
    header: np.ndarray
    distance: np.ndarray
    groups: list

    @classmethod
    def decode(cls, wire, previous=None):
        a = np.asarray(wire, dtype=np.float64)
        if a.size < 32 or not np.isfinite(a).all() or tuple(a[:2]) != (3., 1.):
            raise ValueError('invalid native collision snapshot')
        h = a[:32].copy()
        shape = tuple(int(v) for v in h[5:8])
        nb, nd = int(h[12]), int(h[13])
        prefix = 32 + 7 * (nb + nd)
        if (min(shape) < 2 or nb < 1 or nd < 0 or h[11] <= 0 or
                not np.array_equal(h[5:8], shape) or int(np.prod(shape)) > 500000 or
                nb != h[12] or nd != h[13] or np.any(h[25:28] < 0)):
            raise ValueError('invalid native collision dimensions')
        cells = int(np.prod(shape))
        if a.size != prefix + (cells if h[20] == 1 else 0):
            raise ValueError('incomplete native collision snapshot')
        if h[20] == 1:
            field = a[prefix:].reshape(shape).copy()
        elif previous is not None and np.array_equal(h[[5,6,7,8,9,10,11,14,17]], previous.header[[5,6,7,8,9,10,11,14,17]]):
            field = previous.distance
        else:
            raise SnapshotCacheMiss('delta without identical cached ESDF')
        dynamic = a[32 + 7 * nb:prefix].reshape((-1, 7))
        groups = []
        if len(dynamic):
            keys = np.unique(dynamic[:, 3:7], axis=0)
            for key in keys:
                pts = dynamic[np.all(dynamic[:, 3:7] == key, axis=1), :3]
                groups.append((key[:3].copy(), cKDTree(pts)))
        return cls(h, field, groups)

    def fresh(self, now):
        h = self.header
        return bool(h[4] == 1 and (h[28] == 0 or h[19] == 1) and
                    -.05 <= now - h[3] <= h[15] and
                    (h[28] == 0 or -.05 <= now - h[18] <= h[15]))

    def query(self, spheres, time):
        """Physical surface gap and soft penalty, for (..., sphere, xyz/r)."""
        s = np.asarray(spheres)
        points = s[..., :3]
        h = self.header
        f = (points - h[8:11]) / h[11] - .5
        shape = np.array(self.distance.shape)
        inside = np.all((f >= 0) & (f < shape - 1), axis=-1)
        # Same trilinear interpolant; keep the original domain rejection below.
        d = map_coordinates(self.distance, np.moveaxis(f, -1, 0),
                            order=1, prefilter=False, mode='nearest')
        d = np.where(inside, d, -1e3)
        # The native fixed structure includes one resolution of margin.
        radii = s[..., 3].copy()
        radii[..., :47] += h[11]
        age=np.broadcast_to(np.asarray(time)-h[18],s.shape[:-2])
        if self.groups:
            forecast_ok=(age>=-.05)&(age<=h[16]+1e-9)
            d=np.where(forecast_ok[...,None],d,-1e3)
            future = np.maximum(0., age)
            uncertainty = h[25]+h[26]*future+.5*h[27]*future**2
            for velocity,tree in self.groups:
                moved = points-future[..., None, None]*velocity
                # AABB distance is a lower bound on nearest-point distance.
                offset = np.maximum(np.maximum(tree.mins-moved, moved-tree.maxes), 0.)
                lower = np.sqrt(np.sum(offset*offset, axis=-1))-uncertainty[..., None]
                # One actual point provides an upper bound on the minimum gap.
                offset = moved-tree.data[0]
                sample_gap = np.sqrt(np.sum(offset*offset, axis=-1))-uncertainty[..., None]-radii
                upper_gap = np.minimum(np.min(d-radii, axis=-1), np.min(sample_gap, axis=-1))
                # Query every sphere that could change the global minimum OR
                # the soft penalty. Equality and a roundoff guard are included.
                threshold = np.minimum(d, np.maximum(upper_gap[..., None]+radii, radii+.08))
                needed = lower <= threshold+1e-10
                if np.any(needed):
                    nearest = tree.query(moved[needed], k=1)[0]
                    margin = np.broadcast_to(uncertainty[..., None], d.shape)[needed]
                    d[needed] = np.minimum(d[needed], nearest-margin)
        gap = np.min(d-radii, axis=-1)
        soft = np.sum(np.maximum(0., radii + .08 - d)**3, axis=-1) * 180.
        return gap, soft


class WholeBodyMPPI:
    def __init__(self, model, limits, arm_limits, postures, enabled=True, seed=11,
                 horizon=20, step=.1, random_samples=8, arm_position_servo=False):
        self.model, self.limits = model, limits
        self.qmin, self.qmax, self.vmax, self.amax = [np.asarray(x, dtype=float) for x in arm_limits]
        self.postures = np.asarray(postures, dtype=float)
        self.enabled = enabled
        self.seed = seed
        self.rng = np.random.RandomState(self.seed)
        self.H, self.dt, self.random_samples = horizon, step, random_samples
        # Optional nominal simulator response, never a calibrated Piper model.
        self.arm_position_servo = bool(arm_position_servo)
        self._servo_native = _load_position_servo_library() if self.arm_position_servo else None
        self._legacy_native = None if self.arm_position_servo else _load_legacy_velocity_library()
        self.previous = None
        self._tracking_key = None
        self._tracking_capture = False
        self._tracking_gear = None
        # Populated only for a subscribed, rate-limited visualization cycle.
        # Reuse evaluated states; never rerun a rollout or a collision query.
        self.capture_rollouts = False
        self.visual_rollouts = None

    def record_visual_rollouts(self, states, valid, reset=False):
        if not self.capture_rollouts:
            return
        candidates = states[:32, :64, :3].copy()
        flags = np.asarray(valid[:32], dtype=bool).copy()
        if not reset and self.visual_rollouts is not None:
            old, old_flags = self.visual_rollouts
            candidates = np.concatenate((old, candidates))[:32]
            flags = np.concatenate((old_flags, flags))[:32]
        self.visual_rollouts = candidates, flags

    def position_target(self, state, controls):
        """Map velocity intent to a bounded position lead for one control period.

        The .12 s position loop and .12 s velocity loop describe the simulator.
        Use self.dt even when prediction is subdivided into half steps. The
        position lead is explicitly capped at .03 rad before joint bounds.
        """
        q = np.asarray(state)[..., 5:11]
        velocity = np.minimum(np.maximum(np.asarray(controls)[..., 2:], -self.vmax), self.vmax)
        lead = np.minimum(np.maximum((.12+.5*self.dt)*velocity, -.03), .03)
        return np.minimum(np.maximum(q+lead, self.qmin), self.qmax)

    def _advance_base(self, state, controls, dt):
        """Original base arithmetic, shared by velocity and position arm modes."""
        z = np.asarray(state).copy()
        u = np.asarray(controls).copy()
        c = self.limits
        h = c['track']/2
        scale = np.maximum.reduce([np.ones(u.shape[:-1]), np.abs(u[...,0])/c['vmax'],
            np.abs(u[...,1])/c['wmax'], np.abs(u[...,0]-h*u[...,1])/c['wheel_v'],
            np.abs(u[...,0]+h*u[...,1])/c['wheel_v']])
        u[...,:2] /= scale[...,None]
        da = (u[...,:2]-z[...,3:5])/.12
        scale = np.maximum.reduce([np.ones(u.shape[:-1]),np.abs(da[...,0])/c['accel'],
            np.abs(da[...,1])/c['angular_accel'],np.abs(da[...,0]-h*da[...,1])/c['wheel_a'],
            np.abs(da[...,0]+h*da[...,1])/c['wheel_a']])
        vv = z[...,3:5] + dt*da/scale[...,None]
        mid = (z[...,3:5]+vv)/2
        angle = z[...,2] + dt*mid[...,1]/2
        z[...,0] += dt*mid[...,0]*np.cos(angle)
        z[...,1] += dt*mid[...,0]*np.sin(angle)
        z[...,2] = wrap(z[...,2]+dt*mid[...,1])
        z[...,3:5] = vv
        return z, u

    def _advance_position_arm(self, z, target, dt):
        """In-place nominal servo integration; target=None means velocity braking."""
        if self._servo_native is None:
            return self._advance_position_arm_numpy(z, target, dt)
        if (not isinstance(z, np.ndarray) or z.dtype != np.float64 or z.ndim < 1
                or z.shape[-1] != 17 or not z.flags.c_contiguous or not z.flags.writeable):
            raise ValueError('native servo state requires writable contiguous float64 (...,17)')
        flat = z.reshape((-1, 17))
        target_array = (None if target is None else
            np.ascontiguousarray(np.broadcast_to(target, z.shape[:-1]+(6,)), dtype=np.float64))
        bounds = tuple(np.ascontiguousarray(value, dtype=np.float64)
                       for value in (self.qmin, self.qmax, self.vmax, self.amax))
        if any(value.shape != (6,) for value in bounds):
            raise ValueError('native servo limits require six joints')
        status = self._servo_native(flat, len(flat), None if target_array is None else target_array.ctypes.data,
                                    *bounds, dt)
        if status:
            raise ValueError('native position servo rejected invalid inputs: '+str(status))
        return z

    def _advance_position_arm_numpy(self, z, target, dt):
        """Original 10 ms NumPy integration retained for fallback and equivalence."""
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('positive finite position-servo integration step required')
        count = max(1, int(math.ceil(dt/.01)))
        step = dt/count
        for _ in range(count):
            q, velocity = z[..., 5:11].copy(), z[..., 11:17].copy()
            desired = np.zeros_like(velocity) if target is None else (target-q)/.12
            lower = -np.minimum(self.vmax, np.sqrt(2*self.amax*np.maximum(q-self.qmin, 0.)))
            upper = np.minimum(self.vmax, np.sqrt(2*self.amax*np.maximum(self.qmax-q, 0.)))
            desired = np.minimum(np.maximum(desired, lower), upper)
            acceleration = np.minimum(np.maximum((desired-velocity)/.12, -self.amax), self.amax)
            next_velocity = velocity+step*acceleration
            z[..., 5:11] = q+step*(velocity+next_velocity)/2
            z[..., 11:17] = next_velocity
        return z

    def _advance_position(self, state, controls, target, dt):
        """One base update plus unchanged 10 ms arm steps, on a fresh state copy."""
        function = getattr(self._servo_native, 'whole_body', None)
        if function is None:
            shape = np.asarray(state).shape[:-1]+(8,)
            u = np.zeros(shape) if controls is None else np.broadcast_to(controls, shape)
            z, _ = self._advance_base(state, u, dt)
            return self._advance_position_arm(z, target, dt)
        z = np.asarray(state).copy(order='C')
        if z.dtype != np.float64 or z.ndim < 1 or z.shape[-1] != 17:
            raise ValueError('native servo state requires float64 (...,17)')
        flat = z.reshape((-1, 17))
        u = (None if controls is None else np.ascontiguousarray(
            np.broadcast_to(controls, z.shape[:-1]+(8,)), dtype=np.float64))
        goal = (None if target is None else np.ascontiguousarray(
            np.broadcast_to(target, z.shape[:-1]+(6,)), dtype=np.float64))
        base = np.array([self.limits[key] for key in
            ('track', 'vmax', 'wmax', 'wheel_v', 'accel', 'angular_accel', 'wheel_a')], dtype=np.float64)
        bounds = tuple(np.ascontiguousarray(value, dtype=np.float64)
                       for value in (self.qmin, self.qmax, self.vmax, self.amax))
        if any(value.shape != (6,) for value in bounds):
            raise ValueError('native servo limits require six joints')
        status = function(flat, len(flat), None if u is None else u.ctypes.data,
                          None if goal is None else goal.ctypes.data, base, *bounds, dt)
        if status:
            raise ValueError('native whole-body servo rejected invalid inputs: '+str(status))
        return z

    def advance(self, state, controls, dt=None, arm_target=None):
        dt = self.dt if dt is None else dt
        if self.arm_position_servo:
            target = self.position_target(state, controls) if arm_target is None else np.asarray(arm_target)
            return self._advance_position(state, controls, target, dt)
        if arm_target is not None:
            raise ValueError('explicit position target requires arm_position_servo')
        if (self._legacy_native is not None and np.asarray(state).dtype == np.float64
                and np.asarray(controls).dtype == np.float64):
            z = np.asarray(state).copy(order='C')
            if z.ndim < 1 or z.shape[-1] != 17:
                raise ValueError('native velocity state requires float64 (...,17)')
            flat = z.reshape((-1, 17))
            u = np.ascontiguousarray(np.broadcast_to(controls, z.shape[:-1]+(8,)))
            base = np.array([self.limits[key] for key in
                ('track', 'vmax', 'wmax', 'wheel_v', 'accel', 'angular_accel', 'wheel_a')], dtype=np.float64)
            bounds = tuple(np.ascontiguousarray(value, dtype=np.float64)
                           for value in (self.vmax, self.amax))
            if any(value.shape != (6,) for value in bounds):
                raise ValueError('native velocity limits require six joints')
            status = self._legacy_native(flat, len(flat), u.ctypes.data, base, *bounds, dt)
            if status:
                raise ValueError('native legacy velocity rejected invalid inputs: '+str(status))
            return z
        return self._advance_legacy_numpy(state, controls, dt)

    def _advance_legacy_numpy(self, state, controls, dt):
        """Unchanged legacy arithmetic for old libraries and numeric comparison."""
        z, u = self._advance_base(state, controls, dt)
        qa, va = z[...,5:11].copy(), z[...,11:17].copy()
        target_v = np.minimum(np.maximum(u[...,2:], -self.vmax), self.vmax)
        va2 = va + np.minimum(np.maximum(target_v-va, -self.amax*dt), self.amax*dt)
        z[...,5:11] = qa + dt*(va+va2)/2
        z[...,11:17] = va2
        return z

    def advance_position_brake(self, state, dt=None):
        """Zero base velocity target and zero desired joint velocity after expiry."""
        if not self.arm_position_servo:
            raise ValueError('position braking requires arm_position_servo')
        dt = self.dt if dt is None else dt
        return self._advance_position(state, None, None, dt)

    def _tracking_state(self, state, tracking):
        """Measured geometry selects capture; phase/scale never select a mode."""
        key = gear = None
        samples = None
        resume = None
        if tracking is not None:
            gear = tracking['gear']
            key = tracking['identity']
            samples = np.asarray(tracking['base_samples'], dtype=float)
            resume = float(tracking['resume_distance'])
            if (gear not in (-1, 1) or samples.shape != (self.H+1, 6)
                    or not np.isfinite(samples).all() or not np.isfinite(resume) or resume < 0.):
                raise ValueError('invalid native timed tracking reference')
        changed = key != self._tracking_key or gear != self._tracking_gear
        previous_capture = self._tracking_capture
        if changed:
            self._tracking_key, self._tracking_gear = key, gear
            self._tracking_capture = False
        if tracking is None:
            self._tracking_capture = False
        else:
            delta = samples[0, :2]-state[:2]
            lateral = -np.sin(state[2])*delta[0]+np.cos(state[2])*delta[1]
            self._tracking_capture = bool(np.linalg.norm(delta) > .7*resume
                if self._tracking_capture else abs(lateral) > resume)
        if changed or previous_capture != self._tracking_capture:
            self.previous = None
        return gear, self._tracking_capture, resume, samples

    def _native_base_guide(self, states, target, scales, angles, gear, capture, resume, yaw_hold_speed):
        """Track one virtual-time pose, including after its clock is stopped.

        Body-frame trajectory feedback uses signed speed for reverse travel.
        At zero feedforward, this law alone cannot remove pure lateral error;
        capture that same pose when lateral error leaves the native recovery
        envelope, then return to pose tracking inside it. This geometric mode
        and its hysteresis are identical at every clock scale. Each rollout
        owns its own capture state and cannot advance the measured stage.
        """
        delta = target[:2]-states[:, :2]
        c, s = np.cos(states[:, 2]), np.sin(states[:, 2])
        ex, ey = c*delta[:, 0]+s*delta[:, 1], -s*delta[:, 0]+c*delta[:, 1]
        distance = np.linalg.norm(delta, axis=1)
        capture = np.where(capture, distance > .7*resume, np.abs(ey) > resume)
        # Match mm_controller's hold-yaw convention at a zero-speed singularity.
        yaw_error = (wrap(target[2]-states[:, 2]) if target[5] > yaw_hold_speed
                     else np.zeros(len(states)))
        yaw_error = wrap(yaw_error+angles)
        v = target[3]*np.cos(yaw_error)+.9*ex
        w = target[4]+target[3]*ey+1.8*yaw_error
        bearing = wrap(np.arctan2(delta[:, 1], delta[:, 0])+
                       (np.pi if gear < 0 else 0.)+angles-states[:, 2])
        v = np.where(capture, gear*.9*distance*np.maximum(0., np.cos(bearing)), v)
        w = np.where(capture, 2.*bearing, w)
        return np.c_[np.clip(v, -self.limits['vmax'], self.limits['vmax'])*scales,
                     np.clip(w, -self.limits['wmax'], self.limits['wmax'])], capture

    def _base_guide(self, states, reference, near, delta, goal_dist, scales, angles):
        """Unchanged spatial guide for non-native/standby callers."""
        direction = np.where(reference[near, 3] < -.01, -1., 1.)
        desired = np.arctan2(delta[:, 1], delta[:, 0])+np.where(direction < 0, np.pi, 0.)+angles
        error = wrap(desired-states[:, 2])
        speed = np.minimum(np.minimum(self.limits['vmax'],
            np.maximum(.08, np.abs(reference[near, 3]))), .9*goal_dist)
        controls = np.c_[direction*speed*scales*np.maximum(0., np.cos(error)),
                         np.clip(2.*error, -self.limits['wmax'], self.limits['wmax'])]
        end = goal_dist < .025
        controls[end, 0] = 0.
        controls[end, 1] = np.clip(1.8*wrap(reference[-1, 2]-states[end, 2]),
                                 -self.limits['wmax'], self.limits['wmax'])
        return controls

    def guide(self, state, reference, speed_scale=1., angle_offset=0., tracking=None):
        gear, capture, resume, samples = self._tracking_state(state, tracking)
        if tracking is not None:
            controls, _ = self._native_base_guide(np.asarray(state)[None], samples[0],
                speed_scale, angle_offset, gear, capture, resume, tracking['yaw_hold_speed'])
            return controls[0]
        pos = state[:2]
        d = np.linalg.norm(reference[:,:2]-pos, axis=1)
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(reference[:,:2], axis=0), axis=1))]
        d[arc > .30] = np.inf
        k = int(np.argmin(d))
        # Restrict this prototype to the selected reference's current gear.
        end = min(len(reference)-1, k+1)
        while end+1 < len(reference) and np.linalg.norm(reference[end,:2]-pos) < .30:
            end += 1
        delta = reference[end,:2]-pos
        goal_dist = np.linalg.norm(reference[-1,:2]-pos)
        controls = self._base_guide(np.asarray(state)[None], reference, np.array([k]),
            delta[None], np.array([goal_dist]), speed_scale, angle_offset)
        return controls[0]

    def proposals(self, state, reference, arm_reference, base_modes=None, tracking=None):
        gear, measured_capture, resume, samples = self._tracking_state(state, tracking)
        targets = [np.asarray(arm_reference), state[5:11]]
        if self.enabled:
            targets.extend(self.postures)
        else:
            targets = [state[5:11]]
        unique=[]
        for target in targets:
            target = np.broadcast_to(np.asarray(target, dtype=float), (self.H, 6))
            if not np.isfinite(target).all():
                raise ValueError('nonfinite arm reference')
            if not any(np.max(np.abs(target-other))<1e-7 for other in unique):unique.append(target)
        targets=unique
        labels=[]
        # First keep the base close to its nominal motion and vary arm shape.
        # Then evaluate slowdown, small steering and reverse alternatives.
        base_modes = ([(1.,0.),(.65,0.),(0.,0.),(.8,-.50),(.8,.50),(-.55,0.)]
                      if base_modes is None else base_modes)
        target_rows=[];scales=[];angles=[]
        for arm_id,target in enumerate(targets):
            for scale,angle in base_modes:
                target_rows.append(target);scales.append(scale);angles.append(angle)
                labels.append((arm_id,np.sign(angle)))
        target_rows=np.asarray(target_rows);scales=np.asarray(scales);angles=np.asarray(angles)
        z=np.tile(state,(len(labels),1));controls=np.empty((len(labels),self.H,8))
        capture = np.full(len(labels), measured_capture, dtype=bool)
        arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(reference[:,:2],axis=0),axis=1))]
        for k in range(self.H):
            if tracking is not None:
                ub, capture = self._native_base_guide(z, samples[k], scales, angles,
                    gear, capture, resume, tracking['yaw_hold_speed'])
            else:
                distances=np.sum((z[:,None,:2]-reference[None,:,:2])**2,axis=2)
                distances[:, arc > .30+(k+1)*self.dt*self.limits['vmax']] = np.inf
                near=np.argmin(distances,axis=1)
                look=np.minimum(len(reference)-1,np.searchsorted(arc,arc[near]+.30))
                delta=reference[look,:2]-z[:,:2]
                goal_dist=np.linalg.norm(reference[-1,:2]-z[:,:2],axis=1)
                ub = self._base_guide(z, reference, near, delta, goal_dist, scales, angles)
            ua=np.clip(1.8*(target_rows[:,k,:]-z[:,5:11])-.2*z[:,11:17],-self.vmax,self.vmax)
            if not self.enabled:ua[:]=0.
            controls[:,k]=np.c_[ub,ua];z=self.advance(z,controls[:,k])
        if self.previous is not None and len(base_modes)>1:
            controls=np.concatenate([controls,np.r_[self.previous[1:],self.previous[-1:]][None]])
            labels.append((-1,0))
        return controls, labels

    def spheres(self, state):
        return self.model.body_spheres_batch(state[...,5:11], state[...,:3])

    def configuration_gap(self,spheres,q):
        # These three checks are invariant to planar base translation/yaw.
        # Base alternatives often share identical arm states: evaluate each q
        # once in the native frame, then restore the candidate/time ordering.
        # External obstacles still use every rollout's global spheres above.
        unique, inverse = np.unique(q, axis=0, return_inverse=True)
        return self.model.configuration_gaps_batch(unique)[inverse]

    def evaluate(self, state, controls, reference, arm_reference, snapshot, now, tracking=None):
        controls = np.asarray(controls, dtype=float)
        K = len(controls)
        states = np.empty((K,self.H+1,17));states[:,0] = state
        # Integrate first, then batch all FK and distance queries. This avoids
        # repeating Python dispatch and temporary geometry at every sample.
        dense=[];z=states[:,0].copy();sample_dt=self.dt/2
        for k in range(self.H):
            # A delivered position target is held across both integration halves.
            target = self.position_target(z, controls[:,k]) if self.arm_position_servo else None
            for half in range(2):
                z=self.advance(z,controls[:,k],sample_dt,arm_target=target);dense.append(z)
            states[:,k+1]=z
        tail=[];z=states[:,-1].copy()
        tail_steps=np.full(K,40,dtype=int);stopped=np.zeros(K,dtype=bool)
        for j in range(40):
            z=(self.advance_position_brake(z,sample_dt) if self.arm_position_servo
               else self.advance(z,np.zeros((K,8)),sample_dt))
            tail.append(z)
            quiet=(np.max(np.abs(z[:,3:5]),axis=1)<.003)&(np.max(np.abs(z[:,11:17]),axis=1)<.003)
            tail_steps[quiet & ~stopped]=j+1;stopped|=quiet
            if np.all(stopped):break
        full=np.stack([states[:,0]]+dense+tail,axis=1)
        L=full.shape[1]
        # Include each candidate's first stopped sample. Longer tails of other
        # candidates are batch padding, not part of this candidate's prediction.
        checked=np.arange(L)[None,:]<=2*self.H+tail_steps[:,None]
        flat=full[checked]
        times=np.broadcast_to(now+sample_dt*np.arange(L),(K,L))[checked]
        spheres=self.spheres(flat)
        external,obstacle_cost=snapshot.query(spheres,times)
        external_rows=np.full((K,L),np.inf);external_rows[checked]=external
        self.last_external_gaps=np.min(external_rows,axis=1)
        config=self.configuration_gap(spheres,flat[:,5:11])
        configuration_rows=np.full((K,L),np.inf);configuration_rows[checked]=config
        all_gaps=np.full((K,L),np.inf);all_gaps[checked]=np.minimum(external,config)
        bounds=np.ones((K,L),dtype=bool)
        bounds[checked]=np.all((flat[:,5:11]>=self.qmin-1e-9)&(flat[:,5:11]<=self.qmax+1e-9),axis=1)
        # A clear but unfinished braking tail is not a valid stopping witness.
        valid=stopped & np.all((all_gaps>=0)&bounds,axis=1)
        gaps=np.min(all_gaps,axis=1)
        external_violation=(external_rows<0.) & checked
        self.last_evaluation_diagnostics={
            'configuration_gaps':np.min(configuration_rows,axis=1).tolist(),
            'bounds_valid':np.all(bounds,axis=1).tolist(),
            'braking_stopped':stopped.tolist(),
            'first_external_violation_s':[float(sample_dt*np.argmax(row)) if row.any() else None
                                          for row in external_violation],
            'braking_tail_s':(sample_dt*tail_steps).tolist(),
            'checked_duration_s':(sample_dt*(2*self.H+tail_steps)).tolist(),
            'terminal_arm_delta':(states[:,-1,5:11]-state[5:11]).tolist(),
        }
        obstacle_rows=np.zeros((K,L));obstacle_rows[checked]=obstacle_cost
        z=states[:,1:]
        progress_axis = reference[-1,:2]-state[:2]
        progress_axis /= max(1e-9,np.linalg.norm(progress_axis))
        if tracking is None:
            near=np.min(np.sum((z[:,:,None,:2]-reference[None,None,:,:2])**2,axis=-1),axis=-1)
        else:
            targets = np.asarray(tracking['base_samples'])[1:]
            near = np.sum((z[:, :, :2]-targets[None, :, :2])**2, axis=-1)
            # Use the same pose objective when obstacles activate refinement.
            # Convert yaw error to metres with the existing recovery radius,
            # retaining the existing position-error weight below.
            yaw_error = wrap(targets[None, :, 2]-z[:, :, 2])
            yaw_error[:, targets[:, 5] <= tracking['yaw_hold_speed']] = 0.
            near += (tracking['resume_distance']*yaw_error)**2
        travel=(z[:,:,:2]-state[:2])@progress_axis
        side=(z[:,:,0]-state[0])*(-progress_axis[1])+(z[:,:,1]-state[1])*progress_axis[0]
        posture=np.mean(((z[:,:,5:11]-arm_reference)/(self.qmax-self.qmin))**2,axis=-1)
        initial=np.tile(np.r_[state[3:5],state[11:17]],(K,1))
        du=np.diff(np.concatenate([initial[:,None,:],controls],axis=1),axis=1)
        soft=obstacle_rows[:,1:1+2*self.H].reshape((K,self.H,2)).mean(axis=2)
        cost=self.dt*np.sum(soft+9.*near+2.5*side**2+.10*posture-.8*travel+.025*np.sum(du**2,axis=-1),axis=1)
        cost += 5.*np.linalg.norm(states[:,-1,:2]-reference[-1,:2],axis=1)-4.*((states[:,-1,:2]-state[:2])@progress_axis)
        return cost,valid,gaps,states

    def lease_brake_states(self,state,controls,lease=.25):
        """Predict the actual leased prefix followed by simultaneous braking.

        controls=None checks immediate zero targets from the measured state.
        The initial configuration is included. No endpoint teleport is used.
        """
        z=state.copy();states=[z.copy()];step=self.dt/2;t=0.
        for j in range(70):
            if controls is not None and t<lease-1e-9:
                # The actuator receives only the FIRST command, held until a
                # replacement arrives or its lease expires. Future planned
                # controls must not be assumed during a lost-update interval.
                u=controls[0]
            else:u=np.zeros(8)
            z=self.advance(z,u,step);states.append(z.copy());t+=step
            if t>=lease and np.max(np.abs(z[3:5]))<.003 and np.max(np.abs(z[11:17]))<.003:break
        return np.asarray(states)

    def position_lease_brake_states(self, state, control, lease=.30):
        """Hold one position target through the lease, then brake without return.

        Samples include the initial state and each half control period. A lease
        boundary inside a sample is integrated separately without extending it.
        Passing control=None predicts immediate braking from the measured state.
        """
        if not self.arm_position_servo or not math.isfinite(lease) or lease < 0:
            raise ValueError('position servo and finite nonnegative lease required')
        z = np.asarray(state).copy(); states = [z.copy()]
        step = self.dt/2; elapsed = 0.
        target = None if control is None else self.position_target(z, control)
        for _ in range(70):
            hold = min(step, max(0., lease-elapsed)) if control is not None else 0.
            if hold > 1e-12:
                z = self.advance(z, control, hold, arm_target=target)
            if step-hold > 1e-12:
                z = self.advance_position_brake(z, step-hold)
            elapsed += step; states.append(z.copy())
            if elapsed >= lease and np.max(np.abs(z[3:5])) < .003 and np.max(np.abs(z[11:17])) < .003:
                return np.asarray(states)
        raise ValueError('position-servo braking did not settle within prediction budget')

    def lease_brake(self,state,controls,snapshot,now,lease=.25):
        a=self.lease_brake_states(state,controls,lease)
        spheres=self.spheres(a)
        gaps,_=snapshot.query(spheres,now+self.dt/2*np.arange(len(a)))
        gaps=np.minimum(gaps,self.configuration_gap(spheres,a[:,5:11]))
        bounds=np.all((a[:,5:11]>=self.qmin-1e-9)&(a[:,5:11]<=self.qmax+1e-9))
        return bool(np.all(gaps>=0) and bounds),float(np.min(gaps))

    def plan(self,state,reference,arm_reference,snapshot,now,tracking=None):
        # First test continuing nominal base progress while changing only the
        # arm. Broader base maneuvers are generated if this set is infeasible.
        us,labels=self.proposals(state,reference,arm_reference,[(1.,0.)],tracking=tracking)
        cost,valid,gaps,states = self.evaluate(state,us,reference,arm_reference,snapshot,now,tracking=tracking)
        self.record_visual_rollouts(states, valid, reset=True)
        external_gaps=self.last_external_gaps.copy()
        intervention=not valid[0] or external_gaps[0]<.12
        if not intervention:
            self.previous=us[0].copy()
            return (us[0],states[0]),{'feasible':int(valid.sum()),'candidates':len(us),
                'cost':float(cost[0]),'min_gap':float(gaps[0]),'external_min_gap':float(external_gaps[0]),
                'arm_motion':float(np.linalg.norm(states[0,-1,5:11]-state[5:11])),
                'intervention':False,'mode':'nominal'}
        arm_only = bool(np.any(valid))
        mode = 'arm_only' if arm_only else 'whole_body_fallback'
        if not arm_only:
            us,labels=self.proposals(state,reference,arm_reference,tracking=tracking)
            cost,valid,gaps,states=self.evaluate(state,us,reference,arm_reference,snapshot,now,tracking=tracking)
            self.record_visual_rollouts(states, valid, reset=True)
        diagnostics={
            'structured_labels':[list(x) for x in labels],
            'structured_valid':valid.tolist(), 'structured_costs':cost.tolist(),
            'structured_external_gaps':self.last_external_gaps.tolist(),
            **{'structured_'+k:v for k,v in self.last_evaluation_diagnostics.items()},
        }
        ids = np.flatnonzero(valid)
        # A finite posture library is not an infeasibility proof. Even if all
        # seeds fail, refine the least violating one within the same bounded
        # sampling budget. Every result still needs full collision acceptance.
        best = ids[np.argmin(cost[ids])] if len(ids) else int(np.argmax(gaps))
        seed = us[best]
        noise = np.zeros((self.random_samples,self.H,8))
        sigma = np.r_[.012,.08,np.full(6,.035)]
        # Preserve the nominal tracking sequence through random refinement,
        # not only through structured candidate generation. The base still
        # follows the reference; this does not impose a stationary base.
        if arm_only: sigma[:2] = 0.
        for k in range(self.H):
            noise[:,k] = (.75*noise[:,k-1] if k else 0.)+self.rng.normal(size=(self.random_samples,8))*sigma
        if not self.enabled:noise[:,:,2:] = 0.
        extras = seed[None,:,:]+noise
        if len(extras):
            ec,ev,eg,es = self.evaluate(state,extras,reference,arm_reference,snapshot,now,tracking=tracking)
            self.record_visual_rollouts(es, ev)
        else:
            ec=eg=np.empty(0);ev=np.empty(0,dtype=bool);es=np.empty((0,self.H+1,17))
        diagnostics.update(random_valid=ev.tolist(), random_gaps=eg.tolist())
        if not len(ids) and not np.any(ev):
            return None,dict(diagnostics,feasible=0,candidates=len(us)+len(extras),
                             min_gap=float(max(np.max(gaps),np.max(eg) if len(eg) else -np.inf)),mode=mode)
        same = [i for i in ids if labels[i] == labels[best]]
        pool = np.concatenate([us[same],extras[ev]])
        pc = np.r_[cost[same],ec[ev]]
        w = np.exp(-(pc-pc.min())/.15);w/=w.sum()
        mean = np.sum(pool*w[:,None,None],axis=0)
        if arm_only: mean[:,:2] = seed[:,:2]
        mc,mv,mg,ms = self.evaluate(state,mean[None],reference,arm_reference,snapshot,now,tracking=tracking)
        self.record_visual_rollouts(ms, mv)
        incumbent=min(cost[best] if len(ids) else np.inf,ec[ev].min() if np.any(ev) else np.inf)
        if mv[0] and mc[0] <= incumbent+.03:
            selected,score,gap,prediction = mean,mc[0],mg[0],ms[0]
        elif np.any(ev) and (not len(ids) or ec[ev].min()<cost[best]):
            eids=np.flatnonzero(ev);e=eids[np.argmin(ec[eids])]
            selected,score,gap,prediction = extras[e],ec[e],eg[e],es[e]
        else:
            selected,score,gap,prediction = us[best],cost[best],gaps[best],states[best]
        self.previous = selected.copy()
        return (selected,prediction),dict(diagnostics,**{'feasible':int(valid.sum()+ev.sum()),'candidates':int(len(us)+len(extras)+1),
            'cost':float(score),'min_gap':float(gap),'arm_motion':float(np.linalg.norm(prediction[-1,5:11]-state[5:11])),
            'intervention':True,'mode':mode})
