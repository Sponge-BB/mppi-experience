"""Atomic native references retaining all six arm coordinates and derivatives."""
import hashlib
import math
import numpy as np


def message_key(msg):
    import io
    stream=io.BytesIO();msg.serialize(stream)
    return stream.getvalue().hex()


class WholeBodyReference:
    def __init__(self, messages, revision):
        self.messages=messages;self.revision=revision
        rows=[];offset=0.;previous=None
        for index,msg in enumerate(messages):
            if msg.action!=1 or msg.trajectory_id!=index+1 or msg.singul not in (-1,1):
                raise ValueError('invalid native reference transaction')
            if index and msg.header.stamp!=messages[0].header.stamp:
                raise ValueError('mixed native reference epoch')
            if msg.header.frame_id not in ('','world') or not msg.trajectory:
                raise ValueError('invalid native reference frame or empty trajectory')
            for piece in msg.trajectory:
                if piece.num_dim!=8 or piece.num_order!=7 or not 0<piece.duration<=120:
                    raise ValueError('expected eight-dimensional degree-seven polynomial')
                coef=np.asarray(piece.data).reshape((8,8),order='F')
                if not np.isfinite(coef).all():raise ValueError('nonfinite polynomial')
                ts=np.linspace(0,piece.duration,max(3,int(piece.duration/.04)+1))
                p=np.column_stack([np.polyval(c,ts) for c in coef])
                v=np.column_stack([np.polyval(np.polyder(c),ts) for c in coef])
                if previous is not None and np.max(np.abs(p[0]-previous))>.025:
                    raise ValueError('discontinuous complete whole-body reference')
                previous=p[-1]
                speed=np.linalg.norm(v[:,:2],axis=1)*msg.singul
                yaw=np.arctan2(msg.singul*v[:,1],msg.singul*v[:,0])
                yaw[np.abs(speed)<1e-5]=np.nan
                rows.append(np.column_stack((offset+ts,p[:,:2],yaw,speed,p[:,2:],v[:,2:])))
                offset+=piece.duration
        self.rows=np.concatenate(rows)
        good=np.flatnonzero(np.isfinite(self.rows[:,3]))
        if not len(good):raise ValueError('reference has no valid base direction')
        self.rows[:,3]=np.interp(np.arange(len(self.rows)),good,np.unwrap(self.rows[good,3]))
        self.rows[:,3]=(self.rows[:,3]+np.pi)%(2*np.pi)-np.pi
        self.duration=offset
        self.start=messages[0].header.stamp.to_sec()
        self.hashes=[hashlib.sha256(bytes.fromhex(message_key(m))).hexdigest() for m in messages]

    @property
    def base(self):
        return self.rows[:,1:5]

    def associate(self,state,initial=False):
        rows=self.rows
        ids=np.arange(len(rows))
        if initial:ids=ids[rows[:,0]<=min(2.,self.duration)]
        d=np.sum((rows[ids,1:3]-state[:2])**2,axis=1)
        # Joint state prevents accepting another arm configuration at the same
        # planar position. This is association, not a continuity certificate.
        d+=.015*np.mean((rows[ids,5:11]-state[5:11])**2,axis=1)
        yaw=(rows[ids,3]-state[2]+np.pi)%(2*np.pi)-np.pi
        d+=.004*yaw*yaw
        return int(ids[np.argmin(d)])


class WholeBodySession:
    def __init__(self):
        self.goal_key=None;self.generation=0;self.revision=0
        self.states={};self.enabled=False;self.hard_stop=True
        self.guide=None;self.invalid=None

    def goal(self,msg):
        self.goal_key=message_key(msg);self.generation+=1;self.guide=None;self.invalid=None
        self.enabled,self.hard_stop=self.states.get(self.goal_key,(False,True))

    def state(self,text):
        parts=text.split('\n')
        if len(parts)!=3 or parts[1] not in ('0','1') or parts[2] not in ('0','1'):
            self.invalid='invalid_execution_state';return
        value=(parts[1]=='1',parts[2]=='1');self.states[parts[0]]=value
        if parts[0]==self.goal_key:self.enabled,self.hard_stop=value

    def transaction(self,text,now):
        from quadrotor_msgs.msg import PolynomialTraj
        parts=text.split('\n')
        if not parts or parts[0]!=self.goal_key:return 'other_goal'
        count=int(parts[1])
        if not 1<=count<=1000 or len(parts)!=count+2:raise ValueError('incomplete transaction')
        msgs=[PolynomialTraj().deserialize(bytes.fromhex(p)) for p in parts[2:]]
        self.revision+=1
        if len(msgs)==1 and msgs[0].action in (2,5):
            self.guide=None;self.invalid='native_abort';return 'cancel'
        guide=WholeBodyReference(msgs,self.revision)
        if guide.start<=0 or not np.isfinite(guide.rows).all():raise ValueError('invalid complete reference')
        self.guide=guide;self.invalid=None
        return 'replace'

    def authority(self):
        if self.invalid:return self.invalid
        if self.hard_stop:return 'native_hard_stop'
        if not self.enabled:return 'no_active_task'
        if self.guide is None:return 'waiting_native_trajectory'
        return 'ok'
