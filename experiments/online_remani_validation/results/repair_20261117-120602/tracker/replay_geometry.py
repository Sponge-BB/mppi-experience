#!/usr/bin/python3
"""Isolated, acknowledged tracker replay. Publishes only TF, clock, static map and geometry."""
import os
os.environ.update(MALLOC_ARENA_MAX='2',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',ROS_MASTER_URI='http://127.0.0.1:17641',ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1')
import argparse, pathlib, json, time, subprocess, threading, socket, xmlrpc.client, signal, resource, sys, importlib.util
root=pathlib.Path(__file__).resolve().parent
parser=argparse.ArgumentParser(); parser.add_argument('--limit',type=int,default=0);parser.add_argument('--tag',default='full');parser.add_argument('--fixture',action='store_true');parser.add_argument('--binary');parser.add_argument('--behavior',action='store_true');args=parser.parse_args()
out=root/('replay_'+args.tag);out.mkdir(exist_ok=True)
os.environ['ROS_LOG_DIR']=str(out/'ros_logs')
os.nice(15);resource.setrlimit(resource.RLIMIT_AS,(1024**3,1024**3))
import ctypes, gc
ctypes.CDLL('libc.so.6').mallopt(-8,2)
threading.stack_size(256*1024)
import rosbag, rospy
from rosgraph_msgs.msg import Clock, Log
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Int32, Float64
case=root.parent.parent/'dynamic_acceptance_20261117-103634'
parameters=json.loads((case/'latest_parameters.json').read_text())['parameters']
fields=['cluster_count','confirmed_track_count','motion_qualified_track_count','residual_point_count','stationary_background_anchor_count','maximum_confirmed_speed']
expected={k:[] for k in fields}; tfs=[]; static={}
cache=root/'replay_tf_cache.bag'; metrics_cache=root/'replay_expected_metrics.json'
if cache.exists() and metrics_cache.exists():
 expected=json.loads(metrics_cache.read_text())
 source=cache; topics=['/tf','/tf_static']
else:source=case/'core.bag'; topics=['/tf','/tf_static']+['/dynamic_obstacle_tracker/'+k for k in fields]
with rosbag.Bag(str(source)) as bag:
 for topic,msg,t in bag.read_messages(topics=topics):
  if topic=='/tf':tfs.append((t.to_sec(),msg))
  elif topic=='/tf_static':
   for tr in msg.transforms:static[(tr.header.frame_id,tr.child_frame_id)]=tr
  else:expected[topic.rsplit('/',1)[-1]].append(msg.data)
del bag, msg
gc.collect(); ctypes.CDLL('libc.so.6').malloc_trim(0)
if source!=cache:
 with rosbag.Bag(str(cache),'w',compression='lz4') as bag:
  bag.write('/tf_static',TFMessage(list(static.values())),rospy.Time.from_sec(1794883000.6564484))
  for stamp,msg in tfs:bag.write('/tf',msg,rospy.Time.from_sec(stamp))
 del bag,msg
 metrics_cache.write_text(json.dumps(expected)+'\n')
print('inputs',len(tfs),'dynamic TF messages',len(static),'static edges; peak_rss_kib',resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,flush=True)
processes=[];logs=[];log_file=(out/'rosout.jsonl').open('w'); metrics_file=(out/'metrics.jsonl').open('w');cv=threading.Condition();counts={k:0 for k in fields};values={};dynamic=[];static_ready=threading.Event()
def on_log(msg):
 if msg.name!='/dynamic_obstacle_tracker':return
 log_file.write(json.dumps({'stamp':msg.header.stamp.to_sec(),'level':msg.level,'msg':msg.msg})+'\n');log_file.flush()
 if 'Static reference ready' in msg.msg:static_ready.set()
def on_metric(msg,key):
 with cv:counts[key]+=1;values[key]=msg.data;cv.notify_all()
def on_dynamic(msg):
 row={'stamp':msg.header.stamp.to_sec(),'points':msg.width*msg.height}
 if row['points'] and args.behavior:
  import struct
  fields={f.name:f.offset for f in msg.fields};zs=[struct.unpack_from('<f',msg.data,i*msg.point_step+fields['z'])[0] for i in range(row['points'])]
  row.update(z_min=min(zs),z_max=max(zs),field_names=list(fields))
 dynamic.append(row)
def wait_connection(pub, seconds=8):
 deadline=time.monotonic()+seconds
 while pub.get_num_connections()==0:
  if time.monotonic()>deadline:raise RuntimeError('no connection '+pub.resolved_name)
  time.sleep(.02)
def start(cmd,name):
 log=(out/(name+'.log')).open('w');logs.append(log)
 p=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p);return p
summary={'geometry_frames':0,'metric_mismatches':{k:0 for k in fields},'first_mismatches':[],'tf_static_edges':[list(k) for k in static], 'behavior_change':args.behavior}
try:
 sock=socket.socket();existing=sock.connect_ex(('127.0.0.1',17641))==0;sock.close()
 if existing:raise RuntimeError('refuse to use already occupied master port')
 start(['/opt/ros/noetic/bin/roscore','-p','17641'],'master')
 deadline=time.monotonic()+12
 while True:
  try:
   if xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/tracker_replay')[0]==1:break
  except Exception:pass
  if time.monotonic()>deadline:raise RuntimeError('isolated master did not start')
  time.sleep(.1)
 rospy.set_param('/use_sim_time',True)
 rospy.set_param('/dynamic_obstacle_tracker',parameters['dynamic_obstacle_tracker'])
 rospy.set_param('/dynamic_obstacle_tracker/surface_diagnostics',True)
 # TF is already recorded/prefilled; avoid simulated-time timeout sleeping.
 rospy.set_param('/dynamic_obstacle_tracker/tf_timeout',0.0)
 rospy.init_node('tracker_replay',disable_signals=True)
 clock_pub=rospy.Publisher('/clock',Clock,queue_size=1,latch=True,tcp_nodelay=True)
 tf_pub=rospy.Publisher('/tf',TFMessage,queue_size=1000,tcp_nodelay=True)
 static_pub=rospy.Publisher('/tf_static',TFMessage,queue_size=1,latch=True)
 map_pub=rospy.Publisher('/map_generator/global_cloud',PointCloud2,queue_size=1,latch=True)
 cloud_pub=rospy.Publisher('/velodyne_points',PointCloud2,queue_size=1)
 subs=[rospy.Subscriber('/rosout',Log,on_log,queue_size=1000)]
 for k in fields:subs.append(rospy.Subscriber('/dynamic_obstacle_tracker/'+k,Float64 if k.startswith('maximum_') else Int32,on_metric,callback_args=k,queue_size=10,tcp_nodelay=True))
 subs.append(rospy.Subscriber('/dynamic_points',PointCloud2,on_dynamic,queue_size=10,tcp_nodelay=True))
 clock_pub.publish(Clock(rospy.Time.from_sec(1794883000.6564484)))
 static_pub.publish(TFMessage(list(static.values())))
 if args.fixture:os.environ['REMANI_SURFACE_FIXTURE']=str(out/'selected_surfaces.jsonl')
 node_path=args.binary or str(root/'build'/('dynamic_obstacle_tracker_fixture' if args.fixture else 'dynamic_obstacle_tracker_node'))
 node=start([node_path,'__name:=dynamic_obstacle_tracker'],'node')
 wait_connection(cloud_pub);wait_connection(map_pub);wait_connection(tf_pub)
 spec=importlib.util.spec_from_file_location('original_pcd_publisher','/home/l/remani_ws/src/mm_remani/scripts/publish_shifted_pcd.py');pcd=importlib.util.module_from_spec(spec);spec.loader.exec_module(pcd)
 p=pcd.ShiftedPcdPublisher.__new__(pcd.ShiftedPcdPublisher); pc=parameters['global_map_publisher']
 p.raw_arr,p.field_names=p.load_binary_pcd(pc['pcd_file'])
 assert pc['input_is_preprocessed'] and p.field_names==['x','y','z']
 for name,key in [('robot_x','robot_map_x'),('robot_y','robot_map_y'),('robot_z','robot_map_z'),('map_roll','map_roll'),('map_pitch','map_pitch'),('map_yaw','map_yaw')]:setattr(p,name,pc[key])
 p.T_world_map=p.initial_transform();arr=p.transform_raw_points_locked()
 mapmsg=pcd.make_pointcloud_msg('world',1,arr,[PointField('x',0,PointField.FLOAT32,1),PointField('y',4,PointField.FLOAT32,1),PointField('z',8,PointField.FLOAT32,1)],12)
 mapmsg.header.stamp=rospy.Time.from_sec(1794883000.6564484);map_pub.publish(mapmsg)
 time.sleep(.3)
 if node.poll() is not None:raise RuntimeError('tracker exited')
 index=0; tf_index=0;previous_bt=0;start_wall=time.monotonic()
 with rosbag.Bag(str(case/'geometry.bag')) as bag:
  for topic,cloud,t in bag.read_messages(topics=['/velodyne_points']):
   if args.limit and index>=args.limit:break
   bt=t.to_sec()
   # Prefill recorded TF by 0.2 s; lookup still uses the exact cloud source stamp.
   # This replay checks geometry/classification, not original callback scheduling.
   batch_tf=[]
   while tf_index<len(tfs) and tfs[tf_index][0]<=bt+.5:
    batch_tf.extend(tfs[tf_index][1].transforms);tf_index+=1
   if batch_tf:tf_pub.publish(TFMessage(batch_tf))
   clock_pub.publish(Clock(t))
   time.sleep(.06 if index==0 or bt-previous_bt>.2 else .02)
   previous_bt=bt
   with cv:before=dict(counts)
   cloud_pub.publish(cloud)
   deadline=time.monotonic()+3
   with cv:
    while not all(counts[k]==before[k]+1 for k in fields):
     if time.monotonic()>deadline:raise RuntimeError('frame %d metric acknowledgement failed: before=%s after=%s'%(index,before,counts))
     cv.wait(.01)
    row={'index':index,'bag_t':bt,'source_t':cloud.header.stamp.to_sec(),**values}
   for k in fields:
    if row[k]!=expected[k][index]:
     summary['metric_mismatches'][k]+=1
     if len(summary['first_mismatches'])<20:summary['first_mismatches'].append({'index':index,'source_t':row['source_t'],'field':k,'original':expected[k][index],'replay':row[k]})
   metrics_file.write(json.dumps(row)+'\n')
   index+=1;summary['geometry_frames']=index
   if index%250==0:metrics_file.flush();print('frame',index,'elapsed',round(time.monotonic()-start_wall,1),'mismatches',summary['metric_mismatches'],flush=True)
 time.sleep(.3)
 summary.update(elapsed_s=time.monotonic()-start_wall,dynamic_frames=len(dynamic),dynamic_nonempty=sum(x['points']>0 for x in dynamic),tf_messages_published=tf_index,success=True)
except Exception as e:
 summary['error']=repr(e);summary['success']=False
 raise
finally:
 (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
 (out/'dynamic.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in dynamic))
 try:rospy.signal_shutdown('offline replay done')
 except Exception:pass
 for p in reversed(processes):
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGINT)
   try:p.wait(timeout=5)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
 log_file.close();metrics_file.close()
 for f in logs:f.close()
 print(json.dumps(summary),flush=True)
