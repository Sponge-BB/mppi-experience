#!/usr/bin/env python3
"""Read-only offline counterfactual; no publishers, node, SDK, or ROS master."""
import collections, ctypes, hashlib, json, resource, time
from pathlib import Path
import numpy as np
import rosbag, rospy, tf2_py

ROOT=Path('/home/l/remani_ws/experiments/online_remani_validation/results/drive_session_20261112-204632')
GOAL=1794487836.690945
audit=json.loads((ROOT/'goal_occupancy_audit.json').read_text())
blockers=[b for b in audit['blocking_voxels'] if b['occupancy_history'][-1]['code']==3]
indices=np.ascontiguousarray([b['native_index'] for b in blockers],dtype=np.int32);K=len(indices)
lib=ctypes.CDLL('/tmp/remani_native_ray_probe.so');ip=ctypes.POINTER(ctypes.c_int);dp=ctypes.POINTER(ctypes.c_double);up=ctypes.POINTER(ctypes.c_uint64)
lib.configure_targets.argtypes=[ip,ctypes.c_int];lib.replay_frame.argtypes=[dp,ctypes.c_int,dp,up,up]
lib.configure_targets(indices.ctypes.data_as(ip),K)
buffer=tf2_py.BufferCore(rospy.Duration(1000.));err=[]
with rosbag.Bag(str(ROOT/'capture.bag'),'r') as bag:
 for topic,msg,stamp in bag.read_messages(topics=['/tf','/tf_static']):
  for tr in msg.transforms:
   try:
    if topic=='/tf_static':buffer.set_transform_static(tr,'recorded_bag')
    else:buffer.set_transform(tr,'recorded_bag')
   except Exception as exc:
    if len(err)<10:err.append(str(exc))
def transform(frame,stamp):
 t=buffer.lookup_transform_core('world',frame,stamp).transform
 x,y,z,w=np.asarray([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w])/np.linalg.norm([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w])
 R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)], [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)], [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
 return R,np.array([t.translation.x,t.translation.y,t.translation.z])
def xyz(msg):
 off={f.name:f.offset for f in msg.fields};dt=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[off[k] for k in ['x','y','z']],'itemsize':msg.point_step})
 a=np.ndarray((msg.height,msg.width),dtype=dt,buffer=msg.data,strides=(msg.row_step,msg.point_step));return np.column_stack([a[k].ravel() for k in ['x','y','z']])
def group():return {'frames':0,'input_points':0,'projected_points':0,'endpoint_hits':[0]*K,'deduplicated_free_touches':[0]*K,'full_DDA_free_touches':[0]*K,'frames_hit':[0]*K,'frames_free':[0]*K,'full_DDA_frames_free':[0]*K,'suppressed_by_endpoint_dedup':[0]*K,'suppressed_after_shared_traverse':[0]*K}
groups={name:group() for name in ['raw_endpoints','same_raw_with_height_filter']};examples={};errors=[];out=np.zeros((K,8),dtype=np.uint64);tot=np.zeros(9,dtype=np.uint64);start=time.monotonic()
with (ROOT/'native_raw_counterfactual_frames.jsonl').open('w') as dst,rosbag.Bag(str(ROOT/'capture.bag'),'r') as bag:
 for topic,msg,stamp in bag.read_messages(topics=['/cloud_registered']):
  source=msg.header.stamp.to_sec()
  if source<GOAL:continue
  try:
   R,o=transform(msg.header.frame_id,msg.header.stamp);raw=xyz(msg);world=raw@R.T+o
   R,origin=transform('livox_frame',msg.header.stamp);sensor=(world-origin)@R
   base=np.isfinite(world).all(axis=1)&np.isfinite(sensor).all(axis=1)&(np.abs(sensor)<[8,8,3]).all(axis=1)&(world[:,2]>=.15)
   row={'source':source,'receive':stamp.to_sec(),'seq':msg.header.seq,'source_frame':msg.header.frame_id,'lidar_origin':origin.tolist(),'input_points':len(raw),'groups':{}}
   for name,mask in [('raw_endpoints',base),('same_raw_with_height_filter',base&(sensor[:,2]>=.01)&(sensor[:,2]<=1.55))]:
    ids=np.flatnonzero(mask)[:21000];points=np.ascontiguousarray(world[ids],dtype=np.float64)
    code=lib.replay_frame(points.ctypes.data_as(dp),len(points),origin.ctypes.data_as(dp),out.ctypes.data_as(up),tot.ctypes.data_as(up))
    if code:raise RuntimeError('replay '+str(code))
    row['groups'][name]={'projected_points':len(points),'counts':out.tolist(),'totals':tot.tolist()}
    g=groups[name];g['frames']+=1;g['input_points']+=len(raw);g['projected_points']+=len(points)
    values={'endpoint_hits':out[:,0],'deduplicated_free_touches':out[:,1]+out[:,3],'full_DDA_free_touches':out[:,1]+out[:,2],'frames_hit':out[:,0]>0,'frames_free':out[:,1]+out[:,3]>0,'full_DDA_frames_free':out[:,1]+out[:,2]>0,'suppressed_by_endpoint_dedup':out[:,4],'suppressed_after_shared_traverse':out[:,5]}
    for key,value in values.items():g[key]=[a+int(b) for a,b in zip(g[key],value)]
    for k in range(K):
     key=name+':'+str(k)
     if out[k,3]>0 and key not in examples:
      # Native helper records first full-ray point; retain that explicit limit.
      ii=ids[int(out[k,7])-1]
      examples[key]={'source':source,'target_index':indices[k].tolist(),'sensor_endpoint':sensor[ii].tolist(),'world_endpoint':world[ii].tolist(),'lidar_origin':origin.tolist(),'counts':out[k].tolist(),'point_index':int(ii),'example_note':'First full DDA free-touch endpoint in this frame; frame also has a deduplicated native free touch.'}
   dst.write(json.dumps(row,separators=(',',':'))+'\n')
  except Exception as exc:
   if len(errors)<20:errors.append({'source':source,'message':str(exc)})
result={'purpose':'Offline counterfactual to isolate height endpoint deletion, using recorded 1Hz raw point samples and actual native DDA.','target_order':[{'index':x['native_index'],'center':x['center']} for x in blockers],'groups':groups,'examples':examples,'tf_load_errors':err,'replay_errors':errors,'elapsed_s':time.monotonic()-start,'peak_rss_mib':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,'method':{'transform':'Exact source TF world<-raw frame, world<-livox_frame, then sensor=(world-origin)R.','native_DDA':'Same production RayCaster and original map4.5m clip, original endpoint order, per-frame rayend/traverse rules, arbitrary endpoint hit wins misses.','projection':'Finite abs(livox_xyz)<[8,8,3], worldz>=.15, at most21000points.','height_comparison':'Same raw frames with or without additional livoxz between.01 and1.55; bounds inclusive.','limits':['Raw sample only1Hz; not a lossless original scan stream.','Raw counterfactual does not reapply robot sphere self filtering or the mapping branch5cm endpoint dedup, so it is not an exact alternate deployed pipeline.','Per-frame fresh epoch excludes native char wrap; full DDA supplied separately.','No claim these raw scans were natively fused; this is an offline availability test.']},'source_sha256':{'probe_cpp':hashlib.sha256(Path('/tmp/remani_native_ray_probe.cpp').read_bytes()).hexdigest(),'raycast_cpp':hashlib.sha256(Path('/home/l/remani_ws/src/mm_remani_planner/plan_env/src/raycast.cpp').read_bytes()).hexdigest()}}
(ROOT/'native_raw_counterfactual_summary.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({'groups':groups,'errors':errors,'elapsed_s':result['elapsed_s'],'peak_rss_mib':result['peak_rss_mib']},indent=2),flush=True)
