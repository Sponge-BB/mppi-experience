#!/usr/bin/env python3
"""Read-only current-scene map self-consistency; no control or optimizer calls."""
import os,sys,json,struct,math,resource,time
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
sys.path[:0]=['/opt/ros/noetic/lib/python3/dist-packages','/home/l/remani_ws/devel/lib/python3/dist-packages','/home/l/remani_ws/src/mm_remani/scripts/mppi']
from pathlib import Path
import numpy as np,rosbag,yaml
from scipy.ndimage import distance_transform_edt,map_coordinates
from scipy.spatial import cKDTree
from active_arm_model import PiperArmModel
from active_arm_mppi import Snapshot
R=Path(__file__).resolve().parent;WS=Path('/home/l/remani_ws');began=time.monotonic()
def array(raw):
 off=4;nd=struct.unpack_from('<I',raw)[0]
 for _ in range(nd):nl=struct.unpack_from('<I',raw,off)[0];off+=4+nl+8
 off+=4;n=struct.unpack_from('<I',raw,off)[0];off+=4
 return np.frombuffer(raw,dtype='<f8',offset=off,count=n)
def cloud(msg):
 f={x.name:x for x in msg.fields};dt=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[f[k].offset for k in ['x','y','z']],'itemsize':msg.point_step});a=np.frombuffer(msg.data,dtype=dt,count=msg.width*msg.height);out=np.stack([a[k] for k in ['x','y','z']],axis=1);return out[np.isfinite(out).all(axis=1)]
odom=[];joints=[];statuses=[];global_pc=None;dynamic=[];diag=None;tf=[]
with rosbag.Bag(str(R/'stopped_scene.bag')) as bag:
 for topic,msg,t in bag.read_messages():
  if topic=='/mppi/collision_diagnostic':
   # Deserialize is bounded to ~6.5 MiB; retain ndarray rather than voxel lists.
   a=np.asarray(msg.data,dtype=float);h=a[:40].copy();n=int(h[38]);shape=tuple(int(x) for x in h[5:8]);dist=a[40:40+n].reshape(shape).copy();code=a[40+n:].astype(np.uint8).reshape(shape);del a;diag={'receive':t.to_sec(),'header':h.tolist()}
  elif topic=='/odom':
   p=msg.pose.pose;o=p.orientation;tw=msg.twist.twist;yaw=math.atan2(2*(o.w*o.z+o.x*o.y),1-2*(o.y*o.y+o.z*o.z));odom.append({'source':msg.header.stamp.to_sec(),'frame':msg.header.frame_id,'xyyaw':[p.position.x,p.position.y,yaw],'z':p.position.z,'velocity':[tw.linear.x,tw.angular.z]})
  elif topic=='/joint_states_single':
   ix=[msg.name.index('joint'+str(i)) for i in range(1,7)];joints.append({'source':msg.header.stamp.to_sec(),'q':[msg.position[i] for i in ix],'dq':[msg.velocity[i] for i in ix]})
  elif topic=='/mppi/status':statuses.append({'receive':t.to_sec(),'data':json.loads(msg.data)})
  elif topic=='/map_generator/global_cloud':global_pc=cloud(msg);global_meta={'source':msg.header.stamp.to_sec(),'frame':msg.header.frame_id,'point_count':len(global_pc)}
  elif topic=='/dynamic_points':dynamic.append({'source':msg.header.stamp.to_sec(),'count':msg.width*msg.height if hasattr(msg,'width') else len(msg.points)})
  elif topic in ['/tf','/tf_static']:
   for tr in msg.transforms:tf.append({'parent':tr.header.frame_id,'child':tr.child_frame_id,'source':tr.header.stamp.to_sec(),'xyz':[tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z],'q':[tr.transform.rotation.x,tr.transform.rotation.y,tr.transform.rotation.z,tr.transform.rotation.w]})
assert diag is not None
acq=h[37];ob=min(odom,key=lambda r:abs(r['source']-acq));jb=min(joints,key=lambda r:abs(r['source']-h[24]));params=yaml.safe_load((WS/'src/mm_remani_planner/plan_manage/config/piper_param.yaml').read_text());params['mm'].update(yaml.safe_load((WS/'src/mm_remani_planner/plan_manage/config/fixed_collision_spheres_b.yaml').read_text())['mm']);model=PiperArmModel.from_parameters(params);recorded_state=next(np.asarray(r['data']['candidate_diagnostics']['measured_state']) for r in statuses if len(r['data'].get('candidate_diagnostics',{}).get('measured_state',[]))==17);spheres=model.body_spheres(recorded_state[5:11],recorded_state[:3]);radii=spheres[:,3].copy();radii[:47]+=h[11]
raw=(code&1)!=0;inflated=(code&2)!=0
origin=h[8:11];res=h[11]
occ_ids=np.argwhere(raw);occ_xyz=origin+(occ_ids+.5)*res;tree=cKDTree(occ_xyz);raw_nearest,raw_ids=tree.query(spheres[:,:3]);infl_ids=np.argwhere(inflated);infl_xyz=origin+(infl_ids+.5)*res;itree=cKDTree(infl_xyz);infl_nearest,infl_at=itree.query(spheres[:,:3]);f=(spheres[:,:3]-origin)/res-.5
sampled=map_coordinates(dist,f.T,order=1,prefilter=False,mode='nearest');gap=sampled-radii;raw_gap=raw_nearest-radii;infl_gap=infl_nearest-radii
# Independent signed EDT at same generation; compare well inside XY crop.
positive=distance_transform_edt(~inflated,sampling=res);negative=distance_transform_edt(inflated,sampling=res);expected=positive.copy();inside=negative>0;expected[inside]=positive[inside]-negative[inside]+res;expected=np.minimum(expected,.6)
xy=np.indices(shape[:2]);interior=(xy[0]*res>.65)&((shape[0]-1-xy[0])*res>.65)&(xy[1]*res>.65)&((shape[1]-1-xy[1])*res>.65);error=abs(dist-expected);err=error[interior,:]
# Which raw points coincide with the recorded static global cloud's voxelisation?
# The recorded /global_cloud is kept in its actual frame; do not guess transform.
static_note='Recorded global_cloud frame must be aligned before assigning static/live labels.';static_candidates={}
if global_pc is not None:
 static_candidates={'frame':global_meta['frame'],'global_points':len(global_pc)}
 spheres_out=[]
 for i in np.argsort(raw_gap)[:12]:
  spheres_out.append({'index':int(i),'xyz':spheres[i,:3].tolist(),'physical_radius':float(spheres[i,3]),'planning_radius':float(radii[i]),'esdf':float(sampled[i]),'esdf_gap':float(gap[i]),'nearest_raw_center_distance':float(raw_nearest[i]),'raw_gap':float(raw_gap[i]),'nearest_raw_xyz':occ_xyz[raw_ids[i]].tolist(),'nearest_raw_voxel_index':occ_ids[raw_ids[i]].tolist(),'native_raw_api_return_if_no_static_match':float(min(radii[i],raw_nearest[i])),'native_raw_api_collision_if_no_static_match':bool(raw_nearest[i]<radii[i]),'distance_cap_active_at_sphere':bool(sampled[i]>=.6-1e-10),'nearest_inflated_center_distance':float(infl_nearest[i]),'inflated_gap':float(infl_gap[i])})
else:spheres_out=[]
result={'scope':'Only the later stopped scene, not either historical cancellation generation. Diagnostic codes bit0=raw occupied, bit1=inflated occupied; no static bit exists in schema1. All distance comparisons below use this one diagnostic generation.','diagnostic':diag,'raw_occupied':int(raw.sum()),'inflated_occupied':int(inflated.sum()),'codes_counts':{str(int(v)):int(np.count_nonzero(code==v)) for v in np.unique(code)},'odom_used':ob,'odom_age_from_acquisition_ms':(acq-ob['source'])*1000,'joint_used':jb,'joint_age_from_cache_stamp_ms':(h[24]-jb['source'])*1000,'dynamic':dynamic,'global_cloud':global_meta,'static_classification':static_candidates,'static_note':static_note,'query_point_provenance':{'method':'Sphere centers computed from first actually recorded MPPI measured_state; no world/odom TF assumption is introduced here. This state was acquired after the diagnostic, so this checks two APIs at identical points in one current map, not an exact same-time robot clearance.','measured_state':recorded_state.tolist(),'status_receive':statuses[0]['receive']},'body_summary':{'sphere_count':len(spheres),'min_esdf_gap':float(gap.min()),'min_raw_gap':float(raw_gap.min()),'min_inflated_gap':float(infl_gap.min()),'negative_esdf_gap_indices':np.flatnonzero(gap<0).tolist(),'negative_raw_gap_indices':np.flatnonzero(raw_gap<0).tolist(),'negative_inflated_gap_indices':np.flatnonzero(infl_gap<0).tolist()},'closest_raw_spheres':spheres_out,'independent_edt':{'comparison_voxels':int(err.size),'maximum_error':float(err.max()),'mismatch_gt1e_8':int(np.count_nonzero(err>1e-8))},'status':statuses,'tf':tf,'elapsed_s':time.monotonic()-began,'peak_rss_mib':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024}
(R/'stopped_scene_map_consistency.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps({k:result[k] for k in ['diagnostic','body_summary','closest_raw_spheres','independent_edt','global_cloud','elapsed_s','peak_rss_mib']},ensure_ascii=False,indent=2))
