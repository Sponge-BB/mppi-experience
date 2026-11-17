#!/usr/bin/python3
"""Read-only exact-stamp frozen input extraction. No ROS node or publishers."""
import collections,json,math,sys
from pathlib import Path
import numpy as np
import rosbag
ROOT=Path('/home/l/remani_ws')
BASE=ROOT/'experiments/online_remani_validation/results/fast_refresh_capture_20261114-152721/analysis_153308'
OUT=Path(__file__).resolve().with_name('inputs.json')
sys.path.insert(0,str(ROOT/'src/mm_remani/scripts/mppi'))
from native_reference import NativeReference
T0=1794641296.5504305
stamp_key=lambda t:round(float(t),6)
review=json.loads((BASE/'frozen_reference_review.json').read_text())
review_rows={stamp_key(r['rel']):r for r in review['rows']}
statuses=[];events=[]
with rosbag.Bag(str(BASE/'snapshot.bag')) as bag:
 for topic,msg,t in bag.read_messages(topics=['/mppi/status','/planning/trajectory']):
  if topic=='/planning/trajectory': events.append((t.to_sec(),topic,msg));continue
  d=json.loads(msg.data)
  if d.get('state')=='command_delivered' and d.get('native_time_scale')==0. and stamp_key(t.to_sec()-T0) in review_rows:
   statuses.append((t.to_sec(),d));events.append((t.to_sec(),topic,d))
need_base={stamp_key(d['candidate_diagnostics']['capture_inputs']['base_stamp']) for _,d in statuses}
need_joints={stamp_key(d['candidate_diagnostics']['capture_inputs']['joint_stamp']) for _,d in statuses}
odom={};joints={};odom_stamps={};joint_stamps={}
with rosbag.Bag(str(BASE/'snapshot.bag')) as bag:
 for topic,msg,t in bag.read_messages(topics=['/odom','/mm/joint_states']):
  key=stamp_key(msg.header.stamp.to_sec())
  if topic=='/odom' and key in need_base:
   p,q=msg.pose.pose.position,msg.pose.pose.orientation
   odom_stamps[key]=msg.header.stamp.to_sec()
   odom[key]=[p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),msg.twist.twist.linear.x,msg.twist.twist.angular.z]
  elif topic=='/mm/joint_states' and key in need_joints:
   joint_stamps[key]=msg.header.stamp.to_sec()
   ids=[msg.name.index('joint%d'%i) for i in range(1,7)]
   joints[key]=[msg.position[i] for i in ids]+[msg.velocity[i] for i in ids]
native=NativeReference();arm_pieces=[];natives={};rows=[]
for t,topic,msg in sorted(events,key=lambda e:e[0]):
 if topic=='/planning/trajectory':
  kind=native.feed(msg,t)
  if kind=='replace':arm_pieces=[]
  if kind in ('replace','append'):
   for p in msg.trajectory:
    arm_pieces.append([p.duration,np.asarray(p.data).reshape((p.num_dim,p.num_order+1),order='F')[2:,:].tolist()])
  continue
 capture=msg['candidate_diagnostics']['capture_inputs'];bk=stamp_key(capture['base_stamp']);jk=stamp_key(capture['joint_stamp'])
 assert bk in odom and jk in joints,(bk in odom,jk in joints)
 assert odom_stamps[bk]==capture['base_stamp'] and joint_stamps[jk]==capture['joint_stamp']
 row=review_rows[stamp_key(t-T0)]
 assert abs(native.start-(T0+row['native_start_rel']))<1e-7
 assert np.max(np.abs(np.asarray(odom[bk][:3])-np.array([row['recorded_odom'][k] for k in ('x','y','yaw')])))<1e-10
 nk='%0.6f:%d'%(native.start,native.last_id)
 if nk not in natives:natives[nk]=dict(refs=[r.tolist() for r in native.refs],times=[r.tolist() for r in native.times],gears=native.gears[:],start=native.start,arm_pieces=arm_pieces[:])
 rows.append(dict(review=row,state=odom[bk]+joints[jk],native=nk,mode=msg['mode'],base_stamp=capture['base_stamp'],joint_stamp=capture['joint_stamp'],speed_limit=msg.get('base_safety_speed_limit'),observed=[msg['base_v'],msg['base_w']]))
out=dict(scope='Exact numerical controller inputs from recorded source stamps. No independent TF, complete map or vehicle calibration. Frozen nominal proposal replay only.',rows=rows,natives=natives)
OUT.write_text(json.dumps(out,separators=(',',':'))+'\n')
print(json.dumps(dict(output=str(OUT),rows=len(rows),native_snapshots=len(natives),modes=dict(collections.Counter(r['mode'] for r in rows)),missing_base=len(need_base-set(odom)),missing_joints=len(need_joints-set(joints)),bytes=OUT.stat().st_size),indent=2))
