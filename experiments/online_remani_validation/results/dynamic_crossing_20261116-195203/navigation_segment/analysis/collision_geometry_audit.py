import json,pathlib,re,math,time,os
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
import numpy as np,rosbag,rospy,tf2_py
out=pathlib.Path(json.loads(pathlib.Path('/tmp/remani_acceptance_current.json').read_text())['output'])
w=json.loads((out/'analysis/collision_witness_audit.json').read_text())['live_collision_witnesses']
buffer=tf2_py.BufferCore(rospy.Duration(900))
transform_count=0
with rosbag.Bag(str(out/'core.bag')) as bag:
 for topic,m,t in bag.read_messages(topics=['/tf','/tf_static']):
  for tr in m.transforms:
   if topic=='/tf_static':buffer.set_transform_static(tr,'recorded_tf')
   else:buffer.set_transform(tr,'recorded_tf')
   transform_count+=1

def xyz(m):
 f={v.name:v.offset for v in m.fields}
 dt=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[f[k] for k in ['x','y','z']],'itemsize':m.point_step})
 a=np.ndarray((m.height,m.width),dtype=dt,buffer=m.data,strides=(m.row_step,m.point_step)).reshape(-1)
 return np.column_stack([a[k] for k in ['x','y','z']]).astype(float)
def world_xyz(m):
 pts=xyz(m)
 frame=m.header.frame_id.lstrip('/')
 if frame=='world':return pts
 tr=buffer.lookup_transform_core('world',frame,m.header.stamp).transform
 q=tr.rotation;x,y,z,v=q.x,q.y,q.z,q.w
 R=np.array([[1-2*y*y-2*z*z,2*x*y-2*z*v,2*x*z+2*y*v],[2*x*y+2*z*v,1-2*x*x-2*z*z,2*y*z-2*x*v],[2*x*z-2*y*v,2*y*z+2*x*v,1-2*x*x-2*y*y]])
 return pts@R.T+np.array([tr.translation.x,tr.translation.y,tr.translation.z])
prior=None
with rosbag.Bag(str(out/'geometry.bag')) as bag:
 for topic,m,t in bag.read_messages(topics=['/map_generator/global_cloud']):prior=world_xyz(m);break
results=[]
with rosbag.Bag(str(out/'geometry.bag')) as bag:
 for idx,e in enumerate(w):
  msg=e['message'];center=np.fromstring(re.search(r'voxel_center=\[([^]]+)\]',msg).group(1),sep=' ')
  actual=np.fromstring(re.search(r'actual_xy_q=\[([^]]+)\]',msg).group(1),sep=' ');yaw=float(re.search(r'actual_yaw=([^ ]+)',msg).group(1));stamp=e['source']
  delta=center[:2]-actual[:2];co,si=math.cos(yaw),math.sin(yaw)
  r={'witness_number':idx+1,'source':stamp,'voxel_center':center.tolist(),'body_xy_at_stop':[co*delta[0]+si*delta[1],-si*delta[0]+co*delta[1]],'prior_nearest_distance_m':float(np.linalg.norm(prior-center,axis=1).min()) if prior is not None else None,'cloud_samples':[]}
  for topic,m,t in bag.read_messages(topics=['/velodyne_points_mapping','/velodyne_points','/livox/lidar_no_ground'],start_time=rospy.Time.from_sec(stamp-.6),end_time=rospy.Time.from_sec(stamp+.15)):
   try:
    pts=world_xyz(m);finite=np.isfinite(pts).all(axis=1);pts=pts[finite]
    exact=np.all(np.abs(pts-center)<=.0250001,axis=1);near=np.linalg.norm(pts-center,axis=1)<.06
    nearpts=pts[near]
    # Small spatial summary only, no entire point cloud retained.
    r['cloud_samples'].append({'topic':topic,'receive':t.to_sec(),'source':m.header.stamp.to_sec(),'frame':m.header.frame_id,'point_count':len(pts),'exact_voxel_points':int(exact.sum()),'within_6cm_points':int(near.sum()),'nearest_distance_m':float(np.linalg.norm(pts-center,axis=1).min()) if len(pts) else None,'near_examples':nearpts[:5].tolist()})
   except Exception as exc:r['cloud_samples'].append({'topic':topic,'receive':t.to_sec(),'error':repr(exc)})
  results.append(r)
  print(json.dumps({'witness':idx+1,'body_xy':r['body_xy_at_stop'],'prior_nearest':r['prior_nearest_distance_m'],'exact_points_by_topic':{topic:sum(x.get('exact_voxel_points',0) for x in r['cloud_samples'] if x['topic']==topic) for topic in ['/velodyne_points_mapping','/velodyne_points','/livox/lidar_no_ground']},'transform_errors':sum('error' in x for x in r['cloud_samples'])}),flush=True)
(out/'analysis/collision_geometry_audit.json').write_text(json.dumps({'transform_count':transform_count,'prior_points':len(prior) if prior is not None else 0,'witnesses':results,'boundary':'Cloud voxel match and body proximity do not alone identify physical object; transforms use recorded source timestamps.'},ensure_ascii=False,indent=2)+'\n')
