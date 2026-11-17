#!/usr/bin/python3
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
import json, pathlib, re, math
import numpy as np
root=pathlib.Path(__file__).resolve().parent
fixtures=[json.loads(l) for l in (root/'replay_full_fixture/selected_surfaces.jsonl').open()]
def direction(source,target,shift,tolerance,axes):
 n=min(64,len(source));p=source[np.arange(n)*len(source)//n];q=target
 d0=p[:,None,:]-q[None,:,:];d1=(p+shift)[:,None,:]-q[None,:,:]
 s0=(d0[:,:,axes]**2).sum(axis=2);s1=(d1[:,:,axes]**2).sum(axis=2)
 nn0=s0.argmin(axis=1);nn1=s1.argmin(axis=1);e0=s0[np.arange(n),nn0];e1=s1[np.arange(n),nn1]
 mid=(n-1)//2;idx=np.argsort(e1)[mid];a=float(np.partition(e0,mid)[mid]);b=float(np.partition(e1,mid)[mid]);pair=d1[idx,nn1[idx],:]
 flags={'stationary_distinct':a>tolerance*tolerance,'translated_close':b<=tolerance*tolerance,'translation_improves':b<.25*a}
 return {'stationary_m':math.sqrt(a),'translated_m':math.sqrt(b),'ratio':b/a if a else None,'flags':flags,'accepted':all(flags.values()),'median_pair_delta_xyz':pair.astype(float).tolist()}
out=[]
for x in fixtures:
 a=np.array(x['previous'],dtype=np.float32);b=np.array(x['current'],dtype=np.float32);shift=np.array([x['dx'],x['dy'],0],dtype=np.float32)
 r={k:v for k,v in x.items() if k not in ('previous','current')};r['previous_stats']={'n':len(a),'min':a.min(0).astype(float).tolist(),'max':a.max(0).astype(float).tolist()};r['current_stats']={'n':len(b),'min':b.min(0).astype(float).tolist(),'max':b.max(0).astype(float).tolist()}
 for name,axes,tol in [('original_3d',[0,1,2],.05),('diagnostic_xy',[0,1],.05),('diagnostic_one_voxel',[0,1,2],.10)]:
  r[name]={'forward':direction(a,b,shift,tol,axes),'reverse':direction(b,a,-shift,tol,axes)}
  r[name]['accepted']=r[name]['forward']['accepted'] and r[name]['reverse']['accepted']
 out.append(r)
(root/'selected_surface_analysis.json').write_text(json.dumps(out,indent=2)+'\n')
for r in out:
 print(r['stamp'],r['id'],r['previous_stats']['n'],r['current_stats']['n'])
 for name in ('original_3d','diagnostic_xy','diagnostic_one_voxel'):
  v=r[name];print(name,v['accepted'],[(d,round(v[d]['stationary_m'],4),round(v[d]['translated_m'],4),v[d]['flags'],v[d]['median_pair_delta_xyz']) for d in ('forward','reverse')])
