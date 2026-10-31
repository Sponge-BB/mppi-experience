"""Project measured base pose onto a locally continuous polynomial time axis."""
import numpy as np


def project(state, reference, times, previous):
    p=reference[:-1,:2];v=reference[1:,:2]-p;length2=np.einsum('ij,ij->i',v,v)
    fraction=np.clip(np.einsum('ij,ij->i',state[:2]-p,v)/np.maximum(length2,1e-15),0.,1.)
    nearest=p+fraction[:,None]*v;d=np.linalg.norm(nearest-state[:2],axis=1)
    t=times[:-1]+fraction*(times[1:]-times[:-1])
    eligible=(t>=max(0.,previous-.3))&(t<=previous+2.)&(length2>1e-12)
    if not np.any(eligible):return previous,0.,float('inf')
    cost=np.where(eligible,d+1e-7*np.abs(t-previous),np.inf);i=int(np.argmin(cost))
    velocity=state[3]*np.array([np.cos(state[2]),np.sin(state[2])]);tangent=v[i]/np.sqrt(length2[i])
    speed=abs(reference[i,3]*(1-fraction[i])+reference[i+1,3]*fraction[i])
    scale=float(np.clip(np.dot(velocity,tangent)/max(.02,speed),0.,1.))
    return float(t[i]),scale,float(d[i])
