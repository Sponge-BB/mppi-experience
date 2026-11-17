import json,time
from pathlib import Path
import numpy as np
import rosbag,rospy
P=Path('/home/l/remani_ws/experiments/online_remani_validation/results/dynamic_geometry_capture_20261114-175950')
O=P/'analysis_clear_stutter/stop_clouds';O.mkdir(exist_ok=True)
events=[1794650930.353382,1794650933.8426497,1794650944.1902032]
index=[]; began=time.monotonic()
with rosbag.Bag(str(P/'dynamic_capture.bag')) as bag:
 for ei,t in enumerate(events):
  for topic,m,bt in bag.read_messages(topics=['/velodyne_points_mapping'],start_time=rospy.Time.from_sec(t-.8),end_time=rospy.Time.from_sec(t+.2)):
   fields={f.name:f for f in m.fields}
   assert all(fields[k].datatype==7 for k in ['x','y','z'])
   endian='>' if m.is_bigendian else '<'
   cols=[np.ndarray((m.height,m.width),dtype=endian+'f4',buffer=m.data,offset=fields[k].offset,strides=(m.row_step,m.point_step)).ravel() for k in ['x','y','z']]
   xyz=np.column_stack(cols);xyz=xyz[np.isfinite(xyz).all(axis=1)]
   name='event%d_frame%d.npz'%(ei,len(index));np.savez_compressed(str(O/name),xyz=xyz)
   index.append(dict(file=name,event=ei,bag=bt.to_sec(),source=m.header.stamp.to_sec(),frame=m.header.frame_id,points=len(xyz)))
(O/'index.json').write_text(json.dumps(index,indent=2)+'\n')
print(json.dumps(dict(count=len(index),frames=list(set(x['frame'] for x in index)),seconds=time.monotonic()-began)))
