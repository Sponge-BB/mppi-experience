"""Atomic collision input from detector-labelled measurements, never scene truth."""
import math
import numpy as np
from scipy.ndimage import distance_transform_edt


def collision_frame(points,static,gx=-7.5,gy=-5.,res=.05,nx=300,ny=200):
    # x,y,z,vx,vy,vz,confidence,track_id,dynamic
    p=np.asarray(points,dtype=np.float64).reshape((-1,9))
    fixed=np.asarray(static,dtype=np.float64).reshape((-1,3))
    if not len(p) or not np.all(np.isfinite(p)) or not np.all(np.isfinite(fixed)):
        raise ValueError('empty/nonfinite classified frame or static map')
    if not np.all(np.isin(p[:,8],[0.,1.])):
        raise ValueError('invalid membership label')
    moving=p[:,8]==1
    if np.any(moving & ((p[:,7]<=0)|(p[:,7]!=np.floor(p[:,7]))|(p[:,6]<=0)|(p[:,6]>1))):
        raise ValueError('dynamic sample without valid track/confidence')
    p=p[(p[:,2]>.08)&(p[:,2]<1.6)]
    moving=p[:,8]==1
    static_points=np.concatenate([fixed,p[~moving,:3]])
    static_points=static_points[(static_points[:,2]>.08)&(static_points[:,2]<1.6)]
    ix=np.floor((static_points[:,0]-gx)/res).astype(int);iy=np.floor((static_points[:,1]-gy)/res).astype(int)
    inside=(ix>=0)&(iy>=0)&(ix<nx)&(iy<ny)
    grid=np.ones((ny,nx),dtype=bool);grid[iy[inside],ix[inside]]=False
    field=np.ascontiguousarray(np.maximum(0.,distance_transform_edt(grid)*res-math.sqrt(2)*res))
    obs=[]
    for ident in np.unique(p[moving,7]):
        q=p[moving & (p[:,7]==ident)]
        # A track's exported velocity must describe a single rigid CV target.
        v=np.median(q[:,3:5],axis=0)
        if np.max(np.linalg.norm(q[:,3:5]-v,axis=1))>1e-4:
            raise ValueError('inconsistent within-track velocity')
        center=(q[:,:2].min(axis=0)+q[:,:2].max(axis=0))/2
        radius=np.linalg.norm(q[:,:2]-center,axis=1).max()+.025
        obs.append([*center,*v,radius,1.])
    return field,np.ascontiguousarray(obs,dtype=np.float64).reshape((-1,6)),{
        'measured_points':len(p),'dynamic_points':int(moving.sum()),
        'retained_measured_static_points':int((~moving).sum()),'protected_pcd_points':len(fixed),
        'dynamic_tracks':len(obs)}
