"""Whole-body sampling controller for an isolated REMANI numerical experiment.

The arm geometry is recomputed for every predicted configuration.  The native
ESDF and observed dynamic points are inputs; scenario truth is never imported.
Units are metres, seconds and radians.  This is a kinematic/lagged-velocity
model, not a torque controller or a continuous-time safety certificate.
"""
from dataclasses import dataclass
import math
import numpy as np
from scipy.spatial import cKDTree


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


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
        if min(shape) < 2 or nb < 1 or nd < 0 or h[11] <= 0:
            raise ValueError('invalid native collision dimensions')
        cells = int(np.prod(shape))
        if a.size != prefix + (cells if h[20] == 1 else 0):
            raise ValueError('incomplete native collision snapshot')
        if h[20] == 1:
            field = a[prefix:].reshape(shape).copy()
        elif previous is not None and np.array_equal(h[[5,6,7,8,9,10,11,14]], previous.header[[5,6,7,8,9,10,11,14]]):
            field = previous.distance
        else:
            raise ValueError('delta without identical cached ESDF')
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
        return bool(h[4] == 1 and h[19] == 1 and
                    -.05 <= now - h[3] <= h[15] and
                    -.05 <= now - h[18] <= h[15])

    def query(self, spheres, time):
        """Physical surface gap and soft penalty, for (..., sphere, xyz/r)."""
        s = np.asarray(spheres)
        points = s[..., :3]
        h = self.header
        f = (points - h[8:11]) / h[11] - .5
        shape = np.array(self.distance.shape)
        inside = np.all((f >= 0) & (f < shape - 1), axis=-1)
        ij = np.clip(np.floor(f).astype(int), 0, shape - 2)
        frac = f - ij
        d = np.zeros(points.shape[:-1])
        for dx in (0, 1):
            for dy in (0, 1):
                for dz in (0, 1):
                    w = ((frac[..., 0] if dx else 1-frac[..., 0]) *
                         (frac[..., 1] if dy else 1-frac[..., 1]) *
                         (frac[..., 2] if dz else 1-frac[..., 2]))
                    d += w * self.distance[ij[..., 0]+dx, ij[..., 1]+dy, ij[..., 2]+dz]
        d = np.where(inside, d, -1e3)
        age=np.broadcast_to(np.asarray(time)-h[18],s.shape[:-2])
        if self.groups:
            forecast_ok=(age>=-.05)&(age<=h[16]+1e-9)
            d=np.where(forecast_ok[...,None],d,-1e3)
            for velocity,tree in self.groups:
                moved=points-np.maximum(0.,age)[...,None,None]*velocity
                dd=tree.query(moved.reshape((-1,3)),k=1)[0].reshape(d.shape)
                d=np.minimum(d,dd)
        # Match the current native geometry: base hard radius includes one
        # map resolution; arm hard radius is its physical sphere radius.
        radii = s[..., 3].copy()
        radii[..., :47] += h[11]
        gap = np.min(d-radii, axis=-1)
        soft = np.sum(np.maximum(0., radii + .08 - d)**3, axis=-1) * 180.
        return gap, soft


class WholeBodyMPPI:
    def __init__(self, model, limits, arm_limits, postures, enabled=True, seed=11,
                 horizon=20, step=.1, random_samples=8):
        self.model, self.limits = model, limits
        self.qmin, self.qmax, self.vmax, self.amax = [np.asarray(x, dtype=float) for x in arm_limits]
        self.postures = np.asarray(postures, dtype=float)
        self.enabled = enabled
        self.rng = np.random.RandomState(seed)
        self.H, self.dt, self.random_samples = horizon, step, random_samples
        self.previous = None

    def advance(self, state, controls, dt=None):
        dt = self.dt if dt is None else dt
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
        qa, va = z[...,5:11].copy(), z[...,11:17].copy()
        target_v = np.minimum(np.maximum(u[...,2:], -self.vmax), self.vmax)
        va2 = va + np.minimum(np.maximum(target_v-va, -self.amax*dt), self.amax*dt)
        z[...,5:11] = qa + dt*(va+va2)/2
        z[...,11:17] = va2
        return z

    def guide(self, state, reference, speed_scale=1., angle_offset=0.):
        pos = state[:2]
        d = np.linalg.norm(reference[:,:2]-pos, axis=1)
        k = int(np.argmin(d))
        # Restrict this prototype to the selected reference's current gear.
        end = min(len(reference)-1, k+1)
        while end+1 < len(reference) and np.linalg.norm(reference[end,:2]-pos) < .30:
            end += 1
        delta = reference[end,:2]-pos
        goal_dist = np.linalg.norm(reference[-1,:2]-pos)
        if goal_dist < .025:
            err = wrap(reference[-1,2]-state[2])
            return np.array([0., np.clip(1.8*err,-self.limits['wmax'],self.limits['wmax'])])
        gear = -1 if reference[k,3] < -.01 else 1
        yaw = math.atan2(delta[1],delta[0]) + (np.pi if gear < 0 else 0.) + angle_offset
        error = wrap(yaw-state[2])
        speed = min(.22, max(.08, abs(reference[k,3])), .9*goal_dist)
        return np.array([gear*speed*speed_scale*max(0.,math.cos(error)),
                         np.clip(2.*error,-self.limits['wmax'],self.limits['wmax'])])

    def proposals(self, state, reference, arm_reference, base_modes=None):
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
        arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(reference[:,:2],axis=0),axis=1))]
        for k in range(self.H):
            distances=np.sum((z[:,None,:2]-reference[None,:,:2])**2,axis=2)
            near=np.argmin(distances,axis=1)
            look=np.minimum(len(reference)-1,np.searchsorted(arc,arc[near]+.30))
            delta=reference[look,:2]-z[:,:2]
            gear=np.where(reference[near,3]<-.01,-1.,1.)
            goal_dist=np.linalg.norm(reference[-1,:2]-z[:,:2],axis=1)
            desired=np.arctan2(delta[:,1],delta[:,0])+np.where(gear<0,np.pi,0.)+angles
            error=wrap(desired-z[:,2])
            speed=np.minimum(np.minimum(.22,np.maximum(.08,np.abs(reference[near,3]))),.9*goal_dist)
            ub=np.c_[gear*speed*scales*np.maximum(0.,np.cos(error)),
                     np.clip(2.*error,-self.limits['wmax'],self.limits['wmax'])]
            end=goal_dist<.025
            ub[end,0]=0.;ub[end,1]=np.clip(1.8*wrap(reference[-1,2]-z[end,2]),-self.limits['wmax'],self.limits['wmax'])
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

    def evaluate(self, state, controls, reference, arm_reference, snapshot, now):
        K = len(controls)
        states = np.empty((K,self.H+1,17));states[:,0] = state
        # Integrate first, then batch all FK and distance queries. This avoids
        # repeating Python dispatch and temporary geometry at every sample.
        dense=[];z=states[:,0].copy();sample_dt=self.dt/2
        for k in range(self.H):
            for half in range(2):
                z=self.advance(z,controls[:,k],sample_dt);dense.append(z)
            states[:,k+1]=z
        tail=[];z=states[:,-1].copy()
        for j in range(40):
            z=self.advance(z,np.zeros((K,8)),sample_dt);tail.append(z)
            if np.max(np.abs(z[:,3:5]))<.003 and np.max(np.abs(z[:,11:17]))<.003:break
        full=np.stack([states[:,0]]+dense+tail,axis=1)
        L=full.shape[1];flat=full.reshape((-1,17))
        times=np.broadcast_to(now+sample_dt*np.arange(L),(K,L)).ravel()
        spheres=self.spheres(flat)
        external,obstacle_cost=snapshot.query(spheres,times)
        self.last_external_gaps=np.min(external.reshape((K,L)),axis=1)
        config=self.configuration_gap(spheres,flat[:,5:11])
        all_gaps=np.minimum(external,config).reshape((K,L))
        bounds=np.all((flat[:,5:11]>=self.qmin-1e-9)&(flat[:,5:11]<=self.qmax+1e-9),axis=1).reshape((K,L))
        valid=np.all((all_gaps>=0)&bounds,axis=1)
        gaps=np.min(all_gaps,axis=1)
        z=states[:,1:]
        progress_axis = reference[-1,:2]-state[:2]
        progress_axis /= max(1e-9,np.linalg.norm(progress_axis))
        near=np.min(np.sum((z[:,:,None,:2]-reference[None,None,:,:2])**2,axis=-1),axis=-1)
        travel=(z[:,:,:2]-state[:2])@progress_axis
        side=(z[:,:,0]-state[0])*(-progress_axis[1])+(z[:,:,1]-state[1])*progress_axis[0]
        posture=np.mean(((z[:,:,5:11]-arm_reference)/(self.qmax-self.qmin))**2,axis=-1)
        initial=np.tile(np.r_[state[3:5],state[11:17]],(K,1))
        du=np.diff(np.concatenate([initial[:,None,:],controls],axis=1),axis=1)
        soft=obstacle_cost.reshape((K,L))[:,1:1+2*self.H].reshape((K,self.H,2)).mean(axis=2)
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

    def lease_brake(self,state,controls,snapshot,now,lease=.25):
        a=self.lease_brake_states(state,controls,lease)
        spheres=self.spheres(a)
        gaps,_=snapshot.query(spheres,now+self.dt/2*np.arange(len(a)))
        gaps=np.minimum(gaps,self.configuration_gap(spheres,a[:,5:11]))
        bounds=np.all((a[:,5:11]>=self.qmin-1e-9)&(a[:,5:11]<=self.qmax+1e-9))
        return bool(np.all(gaps>=0) and bounds),float(np.min(gaps))

    def plan(self,state,reference,arm_reference,snapshot,now):
        # First test continuing nominal base progress while changing only the
        # arm. Broader base maneuvers are generated if this set is infeasible.
        us,labels=self.proposals(state,reference,arm_reference,[(1.,0.)])
        cost,valid,gaps,states = self.evaluate(state,us,reference,arm_reference,snapshot,now)
        external_gaps=self.last_external_gaps.copy()
        intervention=not valid[0] or external_gaps[0]<.12
        if not intervention:
            self.previous=us[0].copy()
            return (us[0],states[0]),{'feasible':int(valid.sum()),'candidates':len(us),
                'cost':float(cost[0]),'min_gap':float(gaps[0]),'external_min_gap':float(external_gaps[0]),
                'arm_motion':float(np.linalg.norm(states[0,-1,5:11]-state[5:11])),
                'intervention':False}
        if not np.any(valid):
            us,labels=self.proposals(state,reference,arm_reference)
            cost,valid,gaps,states=self.evaluate(state,us,reference,arm_reference,snapshot,now)
        ids = np.flatnonzero(valid)
        if not len(ids):
            return None,{'feasible':0,'candidates':len(us),'min_gap':float(np.max(gaps))}
        best = ids[np.argmin(cost[ids])]
        seed = us[best]
        noise = np.zeros((self.random_samples,self.H,8))
        sigma = np.r_[.012,.08,np.full(6,.035)]
        for k in range(self.H):
            noise[:,k] = (.75*noise[:,k-1] if k else 0.)+self.rng.normal(size=(self.random_samples,8))*sigma
        if not self.enabled:noise[:,:,2:] = 0.
        extras = seed[None,:,:]+noise
        ec,ev,eg,es = self.evaluate(state,extras,reference,arm_reference,snapshot,now)
        same = [i for i in ids if labels[i] == labels[best]]
        pool = np.concatenate([us[same],extras[ev]])
        pc = np.r_[cost[same],ec[ev]]
        w = np.exp(-(pc-pc.min())/.15);w/=w.sum()
        mean = np.sum(pool*w[:,None,None],axis=0)
        mc,mv,mg,ms = self.evaluate(state,mean[None],reference,arm_reference,snapshot,now)
        if mv[0] and mc[0] <= min(cost[best],ec[ev].min() if np.any(ev) else np.inf)+.03:
            selected,score,gap,prediction = mean,mc[0],mg[0],ms[0]
        elif np.any(ev) and ec[ev].min()<cost[best]:
            eids=np.flatnonzero(ev);e=eids[np.argmin(ec[eids])]
            selected,score,gap,prediction = extras[e],ec[e],eg[e],es[e]
        else:
            selected,score,gap,prediction = us[best],cost[best],gaps[best],states[best]
        self.previous = selected.copy()
        return (selected,prediction),{'feasible':int(valid.sum()+ev.sum()),'candidates':int(len(us)+len(extras)+1),
            'cost':float(score),'min_gap':float(gap),'arm_motion':float(np.linalg.norm(prediction[-1,5:11]-state[5:11])),
            'intervention':True}
