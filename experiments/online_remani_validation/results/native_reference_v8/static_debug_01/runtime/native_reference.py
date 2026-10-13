"""Validated native PolynomialTraj reference; no synthetic fallback path.

Native protocol: id=1 replaces, subsequent contiguous ids append, all with
the same start stamp. There is no segment-count/end-of-bundle field; consumers
use the received prefix after a short quiet interval, never claim atomicity.
"""
import hashlib
import io
import numpy as np
from simulator import polynomial


def fingerprint(array):
    return hashlib.sha256(np.asarray(array, dtype='<f8').tobytes()).hexdigest()


class NativeReference:
    def __init__(self):
        self.refs=[]; self.times=[]; self.start=0.; self.last_id=0
        self.revision=0; self.epoch=0; self.changed=0.; self.error=None
        self.messages=[]

    def clear(self):
        self.refs=[]; self.times=[]; self.messages=[]; self.last_id=0

    def feed(self, msg, now):
        # WARN_START/FINAL are informational in the native controller.
        if msg.action in (3, 4):
            return 'notice'
        self.revision+=1; self.changed=now
        if msg.action in (2, 5):
            self.clear(); self.error=None
            return 'cancel'
        try:
            if msg.action!=1 or msg.trajectory_id<1 or msg.singul not in (-1, 1):
                raise ValueError('invalid native action/id/direction')
            stamp=msg.header.stamp.to_sec()
            if not np.isfinite(stamp) or stamp<=0 or msg.header.frame_id not in ('', 'world'):
                raise ValueError('invalid native start stamp/frame')
            if not msg.trajectory or len(msg.trajectory)>1000:
                raise ValueError('empty/oversized native trajectory')
            for p in msg.trajectory:
                if p.num_dim!=8 or p.num_order!=7 or not 0<p.duration<=120 or not np.isfinite(p.duration):
                    raise ValueError('expected fixed-arm 8D degree-7 native polynomial')
                if len(p.data)!=64 or not np.all(np.isfinite(p.data)):
                    raise ValueError('invalid native coefficients')
            if sum(p.duration for p in msg.trajectory)>300:
                raise ValueError('native trajectory exceeds sampling budget')
            if msg.trajectory_id!=1 and (msg.trajectory_id!=self.last_id+1 or stamp!=self.start):
                raise ValueError('missing/out-of-order native segment or mismatched stamp')
            rows=polynomial(msg, step=.04)
            if not np.all(np.isfinite(rows[:, [0,1,2,4]])):
                raise ValueError('nonfinite sampled trajectory')
            if msg.trajectory_id==1:
                self.clear(); self.start=stamp; self.epoch+=1
            elif np.linalg.norm(self.refs[-1][-1,:2]-rows[0,1:3])>.02:
                raise ValueError('discontinuous native segment')
            offset=self.times[-1][-1] if self.times else 0.
            self.refs.append(rows[:,1:5].copy()); self.times.append(rows[:,0]+offset)
            raw=io.BytesIO(); msg.serialize(raw)
            self.messages.append(hashlib.sha256(raw.getvalue()).hexdigest())
            self.last_id=msg.trajectory_id; self.error=None
            return 'replace' if msg.trajectory_id==1 else 'append'
        except (ValueError, OverflowError) as exc:
            self.clear(); self.error=str(exc)
            return 'invalid'


def sampled_reference(refs, fallback_yaw):
    reference=np.ascontiguousarray(np.concatenate(refs), dtype=np.float64)
    valid=np.flatnonzero(np.isfinite(reference[:,2]))
    if len(valid):
        reference[:,2]=np.interp(np.arange(len(reference)), valid, np.unwrap(reference[valid,2]))
        reference[:,2]=np.arctan2(np.sin(reference[:,2]), np.cos(reference[:,2]))
    else:
        reference[:,2]=fallback_yaw
    return reference
