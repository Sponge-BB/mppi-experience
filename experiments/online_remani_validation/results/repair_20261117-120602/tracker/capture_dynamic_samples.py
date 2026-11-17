#!/usr/bin/python3
import os
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:17641',ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1',MALLOC_ARENA_MAX='2',OPENBLAS_NUM_THREADS='1')
import pathlib,time,threading,json,struct,resource,ctypes
threading.stack_size(256*1024);ctypes.CDLL('libc.so.6').mallopt(-8,2)
resource.setrlimit(resource.RLIMIT_AS,(512*1024**2,512*1024**2));os.nice(15)
import rospy,rosbag
from sensor_msgs.msg import PointCloud2
root=pathlib.Path(__file__).resolve().parent/'replay_xy_shared_full';os.environ['ROS_LOG_DIR']=str(root/'capture_ros_logs')
records=[];size=0;done=threading.Event();bag=rosbag.Bag(str(root/'nonempty_dynamic_samples.bag'),'w',compression='lz4')
def cb(m):
 global size
 if not m.width*m.height or done.is_set():return
 if records and m.header.stamp.to_sec()-records[-1]['source_stamp']<.8:return
 if size+len(m.data)>1024**2:done.set();return
 fields={f.name:f.offset for f in m.fields}
 ids=sorted({struct.unpack_from('<I',m.data,i*m.point_step+fields['track_id'])[0] for i in range(m.width*m.height)})
 row={'source_stamp':m.header.stamp.to_sec(),'seq':m.header.seq,'frame_id':m.header.frame_id,'width':m.width,'height':m.height,'point_step':m.point_step,'row_step':m.row_step,'is_bigendian':m.is_bigendian,'is_dense':m.is_dense,'fields':[{'name':f.name,'offset':f.offset,'datatype':f.datatype,'count':f.count} for f in m.fields],'track_ids':ids,'data_bytes':len(m.data)}
 bag.write('/dynamic_points',m,m.header.stamp);size+=len(m.data);records.append(row)
 if len(records)>=5:done.set()
rospy.init_node('dynamic_sample_capture',disable_signals=True)
sub=rospy.Subscriber('/dynamic_points',PointCloud2,cb,queue_size=10,tcp_nodelay=True)
done.wait(150)
sub.unregister();bag.close();(root/'nonempty_dynamic_samples.json').write_text(json.dumps({'records':records,'payload_bytes':size,'schema_unchanged':True},indent=2)+'\n');rospy.signal_shutdown('captured offline samples');print('captured',len(records),'frames',size,'bytes')
