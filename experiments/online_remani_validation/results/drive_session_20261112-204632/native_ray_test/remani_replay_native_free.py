#!/usr/bin/env python3
"""Offline only: bag TF and recorded native mapping endpoints; no ROS node."""
import collections, ctypes, hashlib, json, math, resource, time
from pathlib import Path
import numpy as np
import rosbag, rospy, tf2_py

ROOT=Path('/home/l/remani_ws/experiments/online_remani_validation/results/drive_session_20261112-204632')
GOAL=1794487836.690945
audit=json.loads((ROOT/'goal_occupancy_audit.json').read_text())
blockers=[b for b in audit['blocking_voxels'] if b['occupancy_history'][-1]['code']==3]
indices=np.ascontiguousarray([b['native_index'] for b in blockers],dtype=np.int32)
centers=np.asarray([b['center'] for b in blockers])
K=len(blockers)
print('targets',K,indices.tolist(),flush=True)
lib=ctypes.CDLL('/tmp/remani_native_ray_probe.so')
ip=ctypes.POINTER(ctypes.c_int); dp=ctypes.POINTER(ctypes.c_double); up=ctypes.POINTER(ctypes.c_uint64)
lib.configure_targets.argtypes=[ip,ctypes.c_int]
lib.replay_frame.argtypes=[dp,ctypes.c_int,dp,up,up]
lib.configure_targets(indices.ctypes.data_as(ip),K)
buf=tf2_py.BufferCore(rospy.Duration(1000.))
tf_counts=collections.Counter(); tf_errors=[]
with rosbag.Bag(str(ROOT/'capture.bag'),'r') as bag:
 for topic,msg,stamp in bag.read_messages(topics=['/tf','/tf_static']):
  for tr in msg.transforms:
   try:
    if topic=='/tf_static':buf.set_transform_static(tr,'recorded_bag')
    else:buf.set_transform(tr,'recorded_bag')
    tf_counts[(tr.header.frame_id,tr.child_frame_id,topic)]+=1
   except Exception as exc:
    if len(tf_errors)<10:tf_errors.append(str(exc))
print('TF loaded',sum(tf_counts.values()),'rss_MiB',resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,flush=True)

def exact_transform(msg):
 tr=buf.lookup_transform_core('world',msg.header.frame_id,msg.header.stamp)
 q=tr.transform.rotation;t=tr.transform.translation
 x,y,z,w=np.asarray([q.x,q.y,q.z,q.w])/np.linalg.norm([q.x,q.y,q.z,q.w])
 rotation=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
  [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
  [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
 return rotation,np.asarray([t.x,t.y,t.z],dtype=np.float64)

def xyz(msg):
 offsets={f.name:f.offset for f in msg.fields}
 dt=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[offsets[k] for k in ['x','y','z']],'itemsize':msg.point_step})
 data=np.ndarray((msg.height,msg.width),dtype=dt,buffer=msg.data,strides=(msg.row_step,msg.point_step))
 return np.column_stack([data[k].ravel() for k in ['x','y','z']])

def new_group():
 return {'frames':0,'input_points':0,'projected_points':0,'hits':[0]*K,'free_touches':[0]*K,'full_DDA_touches':[0]*K,'frames_hit':[0]*K,'frames_free':[0]*K,'frames_would_clear_no_hit':[0]*K,'frames_hit_wins_over_free':[0]*K,'full_DDA_frames_free':[0]*K,'suppressed_by_rayend':[0]*K,'suppressed_after_shared_traverse':[0]*K}

def add(group,row):
 group['frames']+=1;group['input_points']+=row['input_points'];group['projected_points']+=row['projected_points']
 c=np.asarray(row['counts'],dtype=np.uint64);a=c[:,0];f=c[:,1]+c[:,3];ff=c[:,1]+c[:,2]
 for name,values in [('hits',a),('free_touches',f),('full_DDA_touches',ff),('frames_hit',a>0),('frames_free',f>0),('frames_would_clear_no_hit',(a==0)&(f>0)),('frames_hit_wins_over_free',(a>0)&(f>0)),('full_DDA_frames_free',ff>0),('suppressed_by_rayend',c[:,4]),('suppressed_after_shared_traverse',c[:,5])]:
  group[name]=[v+int(x) for v,x in zip(group[name],values)]

summary={'targets':blockers,'groups':{},'tf_errors':tf_errors,'mapping_errors':[], 'first_hits':{},'first_full_free':{}}
columns=['endpoint_hits','endpoint_misses','full_DDA_misses','per_frame_dedup_DDA_misses','suppressed_by_endpoint_dedup','suppressed_by_shared_traverse','first_hit_input_index_plus1','first_full_free_input_index_plus1']
summary['count_columns']=columns
out=np.zeros((K,8),dtype=np.uint64);totals=np.zeros(9,dtype=np.uint64);start=time.monotonic();n=0
# Prioritize the requested after-goal result, then run the earlier walking interval.
for label,timefilter in [('after_goal',lambda t:t>=GOAL),('before_goal',lambda t:t<GOAL)]:
 group=summary['groups'][label]=new_group()
 with (ROOT/('native_rays_'+label+'.jsonl')).open('w') as stream,rosbag.Bag(str(ROOT/'capture.bag'),'r') as bag:
  for topic,msg,stamp in bag.read_messages(topics=['/velodyne_points_mapping']):
   source=msg.header.stamp.to_sec()
   if not timefilter(source):continue
   try:
    rotation,origin=exact_transform(msg);sensor=xyz(msg);world=sensor@rotation.T+origin
    mask=np.isfinite(sensor).all(axis=1)&np.isfinite(world).all(axis=1)&(np.abs(sensor)<[8,8,3]).all(axis=1)&(world[:,2]>=.15)
    orig_ids=np.flatnonzero(mask)[:21000];points=np.ascontiguousarray(world[orig_ids],dtype=np.float64)
    result=lib.replay_frame(points.ctypes.data_as(dp),len(points),origin.ctypes.data_as(dp),out.ctypes.data_as(up),totals.ctypes.data_as(up))
    if result:raise RuntimeError('native replay result '+str(result))
    row={'source':source,'receive':stamp.to_sec(),'seq':msg.header.seq,'frame':msg.header.frame_id,'origin':origin.tolist(),'rotation_xyzw':[float(v) for v in [buf.lookup_transform_core('world',msg.header.frame_id,msg.header.stamp).transform.rotation.x,buf.lookup_transform_core('world',msg.header.frame_id,msg.header.stamp).transform.rotation.y,buf.lookup_transform_core('world',msg.header.frame_id,msg.header.stamp).transform.rotation.z,buf.lookup_transform_core('world',msg.header.frame_id,msg.header.stamp).transform.rotation.w]],'input_points':len(sensor),'projected_points':len(points),'counts':out.tolist(),'totals':totals.tolist()}
    stream.write(json.dumps(row,separators=(',',':'))+'\n');add(group,row)
    for k in range(K):
     for name,col in [('first_hits',6),('first_full_free',7)]:
      if out[k,col] and str(k) not in summary[name]:
       ii=orig_ids[int(out[k,col])-1]
       summary[name][str(k)]={'source':source,'input_point_index':int(ii),'sensor_endpoint':sensor[ii].tolist(),'world_endpoint':world[ii].tolist(),'sensor_origin':origin.tolist(),'target_index':indices[k].tolist(),'counts':out[k].tolist(),'group':label}
    n+=1
    if n%200==0:print('processed',n,'label',label,'elapsed',round(time.monotonic()-start,1),flush=True)
   except Exception as exc:
    if len(summary['mapping_errors'])<30:summary['mapping_errors'].append({'source':source,'error':str(exc)})
 summary['elapsed_s']=time.monotonic()-start
 summary['method']={'exact_input':'Recorded native mapping PointXYZ endpoints, original order; exact source-time TF2 BufferCore world<-frame; finite abs(sensor_xyz)<[8,8,3], worldz>=.15, cap21000.',
 'raycast':'Links unmodified production raycast.cpp RayCaster::setInput/step; original map clipping and 4.5m clipping retained. Endpoint hits registered before rayend dedup; misses registered before traverse break. Any hit wins same-frame misses.',
 'frame_dedup_scope':'Native per-frame rayend/traverse logic with fresh 32bit epoch per recorded input. Does not reproduce cross-frame char epoch wrap because exact fused frame selection/history is unavailable.',
 'full_DDA':'Full DDA column ignores only early rayend/traverse exits and is an upper bound on available free touches of native selected input. Zero full_DDA touches excludes dedup and fusion selection as causes of missing free rays for those cells within captured native inputs.',
 'fusion_limit':'Recorded mapping ~10Hz; native fuses ~6Hz and coalesces scans. Per-input replay does not assert that every input was fused. Diagnostic h3 is fusion commit wall-clock time, not cloud source time; it cannot directly identify a source-stamped cloud. No precise fused-frame matches are claimed.',
 'upstream_limit':'No inference that physical raw rays were absent. Other three recorded stages are only1Hz.',
 'map_origin':[-7.5,-5,0],'map_size':[15,10,3],'resolution':.05,'max_ray_length':4.5}
 summary['source_sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path('/tmp/remani_native_ray_probe.cpp'),Path('/tmp/remani_native_ray_probe.so'),Path('/home/l/remani_ws/src/mm_remani_planner/plan_env/src/raycast.cpp')]}
 summary['peak_rss_mib']=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024
 (ROOT/'native_rays_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
 print('GROUP_COMPLETE',label,json.dumps(group),'errors',summary['mapping_errors'],flush=True)
print('DONE elapsed',time.monotonic()-start,'peakRSSMiB',summary['peak_rss_mib'],flush=True)
