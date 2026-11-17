import json
from pathlib import Path
import rosbag,rospy
P=Path('/home/l/remani_ws/experiments/online_remani_validation/results/dynamic_geometry_capture_20261114-175950')
out=P/'analysis_clear_stutter/stop_clouds/transforms.jsonl'
counts={}
with rosbag.Bag(str(P/'dynamic_capture.bag')) as b,out.open('w') as f:
 for topic,m,t in b.read_messages(topics=['/tf_static']):
  for q in m.transforms:
   f.write(json.dumps(dict(topic=topic,bag=t.to_sec(),stamp=q.header.stamp.to_sec(),parent=q.header.frame_id,child=q.child_frame_id,translation=[getattr(q.transform.translation,k) for k in 'xyz'],rotation=[getattr(q.transform.rotation,k) for k in 'xyzw']))+'\n')
 for topic,m,t in b.read_messages(topics=['/tf'],start_time=rospy.Time.from_sec(1794650928),end_time=rospy.Time.from_sec(1794650945)):
  for q in m.transforms:
   counts[(q.header.frame_id,q.child_frame_id)]=counts.get((q.header.frame_id,q.child_frame_id),0)+1
   f.write(json.dumps(dict(topic=topic,bag=t.to_sec(),stamp=q.header.stamp.to_sec(),parent=q.header.frame_id,child=q.child_frame_id,translation=[getattr(q.transform.translation,k) for k in 'xyz'],rotation=[getattr(q.transform.rotation,k) for k in 'xyzw']))+'\n')
print([(k,v) for k,v in counts.items()])
