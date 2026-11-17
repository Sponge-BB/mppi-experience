#!/usr/bin/env python3
"""Offline geometric pipeline replay; never creates a ROS node/publisher."""
import collections,ctypes,hashlib,json,math,resource,sys,time
from pathlib import Path
import numpy as np
import rosbag,rospy,tf2_py
WS=Path('/home/l/remani_ws');ROOT=WS/'experiments/online_remani_validation/results/drive_session_20261112-204632';GOAL=1794487836.690945
sys.path[:0]=[str(WS/'src/mm_remani/scripts'),str(WS/'src/mm_remani/scripts/mppi')]
from velodyne_transformer import outside_self_spheres,mapping_voxel_indices,rigid_transform,cloud_xyz
from active_arm_model import PiperArmModel
params=json.loads((ROOT/'runtime_parameters.json').read_text())
model=PiperArmModel.from_parameters(params['/remani_planner_node'])
audit=json.loads((ROOT/'goal_occupancy_audit.json').read_text());targets=[b for b in audit['blocking_voxels'] if b['occupancy_history'][-1]['code']==3];K=len(targets)
index=np.ascontiguousarray([t['native_index'] for t in targets],dtype=np.int32)
lib=ctypes.CDLL('/tmp/remani_native_ray_probe.so');ip=ctypes.POINTER(ctypes.c_int);dp=ctypes.POINTER(ctypes.c_double);up=ctypes.POINTER(ctypes.c_uint64)
lib.configure_targets.argtypes=[ip,ctypes.c_int];lib.replay_frame.argtypes=[dp,ctypes.c_int,dp,up,up];lib.configure_targets(index.ctypes.data_as(ip),K)
tf=tf2_py.BufferCore(rospy.Duration(1000.));joints=[];load_errors=[];last_joint=0.
with rosbag.Bag(str(ROOT/'capture.bag'),'r') as bag:
 for topic,msg,stamp in bag.read_messages(topics=['/tf','/tf_static','/mm/joint_states']):
  if topic=='/mm/joint_states':
   t=msg.header.stamp.to_sec()
   if t<=last_joint:continue
   ids=[msg.name.index('joint'+str(i)) for i in range(1,7)];q=np.asarray([msg.position[i] for i in ids])
   if np.isfinite(q).all() and (q>=model.lower).all() and (q<=model.upper).all():joints.append((t,q));last_joint=t
  else:
   for tr in msg.transforms:
    try:
     if topic=='/tf_static':tf.set_transform_static(tr,'offline_recorded_bag')
     else:tf.set_transform(tr,'offline_recorded_bag')
    except Exception as exc:
     if len(load_errors)<10:load_errors.append(str(exc))
jt=np.asarray([s for s,q in joints]);jq=np.asarray([q for s,q in joints]);del joints
def transform(frame,stamp):return rigid_transform(tf.lookup_transform_core('world',frame,stamp))
def q_at(t):
 j=int(np.searchsorted(jt,t))
 if j<len(jt) and abs(jt[j]-t)<1.e-8:return jq[j].copy(),[float(jt[j])]
 if not 0<j<len(jt):raise ValueError('no bracketing joint samples')
 if jt[j]-jt[j-1]>.05+1e-9:raise ValueError('joint interpolation gap >50ms')
 w=(t-jt[j-1])/(jt[j]-jt[j-1]);return (1-w)*jq[j-1]+w*jq[j],[float(jt[j-1]),float(jt[j])]
groups={name:{'frames':0,'raw_points':0,'after_height':0,'after_self':0,'mapping_points':0,'native_projected_points':0,'endpoint_hits':[0]*K,'deduplicated_free_touches':[0]*K,'full_DDA_free_touches':[0]*K,'frames_would_clear_no_hit':[0]*K} for name in ['old_height1_55','new_height3_00']}
errors=[];max_gap=0.;examples={};out=np.zeros((K,8),dtype=np.uint64);tot=np.zeros(9,dtype=np.uint64);start=time.monotonic()
with (ROOT/'native_pipeline_height_frames.jsonl').open('w') as stream,rosbag.Bag(str(ROOT/'capture.bag'),'r') as bag:
 for topic,msg,receipt in bag.read_messages(topics=['/cloud_registered']):
  source=msg.header.stamp.to_sec()
  if source<GOAL:continue
  try:
   R,p=transform(msg.header.frame_id,msg.header.stamp);world_raw=cloud_xyz(msg)@R.T+p
   R,origin=transform('livox_frame',msg.header.stamp)
   # PCL/PointCloud2 transformed sensor XYZ is float32 before height/self filters.
   sensor=np.asarray((world_raw-origin)@R,dtype=np.float32)
   Rb,pb=transform('base_link',msg.header.stamp);yaw=math.atan2(Rb[1,0],Rb[0,0]);q,source_joints=q_at(source)
   max_gap=max(max_gap,max(source_joints)-min(source_joints));spheres=model.body_spheres(q,[pb[0],pb[1],yaw])
   row={'source':source,'receive':receipt.to_sec(),'raw_seq':msg.header.seq,'joint_sources':source_joints,'q':q.tolist(),'base_xyyaw':[float(pb[0]),float(pb[1]),yaw],'sensor_origin':origin.tolist(),'groups':{}}
   for name,height in [('old_height1_55',1.55),('new_height3_00',3.)]:
    ids=np.flatnonzero(np.isfinite(sensor).all(axis=1)&(sensor[:,2]>=.01)&(sensor[:,2]<=height));kept_sensor=sensor[ids];world=kept_sensor.astype(np.float64)@R.T+origin
    mask=outside_self_spheres(world,spheres,padding=.01);kept_sensor=kept_sensor[mask];world=world[mask]
    projection=(np.abs(kept_sensor)<[8,8,3]).all(axis=1)&(world[:,2]>=.15)
    select=mapping_voxel_indices(world,origin,.05,np.array([-7.5,-5.,0.]),np.array([15.,10.,3.]),4.5,projection)
    projected_ids=select[projection[select]][:21000];projected=np.ascontiguousarray(world[projected_ids],dtype=np.float64)
    code=lib.replay_frame(projected.ctypes.data_as(dp),len(projected),origin.ctypes.data_as(dp),out.ctypes.data_as(up),tot.ctypes.data_as(up))
    if code:raise ValueError('native replay error '+str(code))
    counts={'raw_points':len(sensor),'after_height':len(ids),'after_self':len(world),'mapping_points':len(select),'native_projected_points':len(projected)}
    row['groups'][name]=dict(counts,counts=out.tolist(),native_totals=tot.tolist());g=groups[name];g['frames']+=1
    for k,v in counts.items():g[k]+=v
    for key,values in [('endpoint_hits',out[:,0]),('deduplicated_free_touches',out[:,1]+out[:,3]),('full_DDA_free_touches',out[:,1]+out[:,2]),('frames_would_clear_no_hit',(out[:,0]==0)&(out[:,1]+out[:,3]>0))]:g[key]=[a+int(b) for a,b in zip(g[key],values)]
    for k in range(K):
     key=name+':'+str(k)
     if key not in examples and out[k,0]==0 and out[k,1]+out[k,3]>0:
      j=int(out[k,7])-1;si=projected_ids[j]
      examples[key]={'source':source,'target':index[k].tolist(),'sensor_endpoint':kept_sensor[si].tolist(),'world_endpoint':world[si].tolist(),'native_counts':out[k].tolist(),'note':'First full DDA endpoint in a frame with a real deduplicated free touch and no endpoint hit; source geometry only.'}
   stream.write(json.dumps(row,separators=(',',':'))+'\n')
  except Exception as exc:
   if len(errors)<20:errors.append({'source':source,'error':str(exc)})
result={'groups':groups,'target_order':[{'index':b['native_index'],'center':b['center']} for b in targets],'examples':examples,'errors':errors,'tf_load_errors':load_errors,'joint_samples':len(jt),'max_joint_bracket_gap_s':max_gap,'elapsed_s':time.monotonic()-start,'peak_rss_mib':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,'method':{'input':'147 sampled raw registered clouds after goal; native raw input order retained.','transform':'Full source-stamped TF2 world<-raw, world<-livox_frame, world<-base_link; transformed livox XYZ rounds to float32 before height/self filtering.','joint_state':'Actual full bag /mm/joint_states source stamps, bracketing linear interpolation, maximum50ms bracket as production geometry policy. No200ms subsampling.','self_filter':'Actual imported outside_self_spheres padding.01, PiperArmModel from recorded planner parameters,47 fixed+20 arm spheres, planar measured base x/y/yaw.','mapping':'Actual imported mapping_voxel_indices at5cm, map origin[-7.5,-5,0],size[15,10,3],maxray4.5; original endpoint order and projected-point policy.','native':'Actual production raycast.cpp DDA, native original clipping/endpoints and rayend/traverse dedup; endpoint hit wins misses.','comparison':'Only height max1.55 versus3.0; min.01 retained. Sensor projection absxyz<[8,8,3],worldz>=.15,capacity21000.','limits':['Geometric offline replay only. Raw clouds sampled1Hz; not full10Hz alternate pipeline.','TF/PCL original transform arithmetic is approximated with exact source TF and float32 output rounding; tiny boundary differences possible.','Does not replay online freshness thresholds, callback scheduling, publisher transport, native input coalescing, or char epoch wrap.','No hardware validation or runtime latency claim.']},'source_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [WS/'src/mm_remani/scripts/velodyne_transformer.py',WS/'src/mm_remani/scripts/mppi/active_arm_model.py',ROOT/'runtime_parameters.json',Path('/tmp/remani_native_ray_probe.cpp'),WS/'src/mm_remani_planner/plan_env/src/raycast.cpp']}}
(ROOT/'native_pipeline_height_summary.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:result[k] for k in ['groups','errors','joint_samples','max_joint_bracket_gap_s','elapsed_s','peak_rss_mib']},indent=2),flush=True)
