import json, math, os, pathlib, resource, signal, socket, subprocess, sys, time, xmlrpc.client
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11443', ROS_IP='127.0.0.1',
    ROS_HOME='/tmp/remani_snapshot_probe_ros', ROS_LOG_DIR='/tmp/remani_snapshot_probe_ros/log',
    OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
os.environ.pop('ROS_HOSTNAME',None)
import numpy as np
import rospy, tf2_ros
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Float64MultiArray, Header
from roslaunch.config import ROSLaunchConfig
from roslaunch.xmlloader import XmlLoader

root=pathlib.Path('/home/l/remani_ws')
out=root/'experiments/online_remani_validation/results/snapshot_pipeline_fix_20261109'
out.mkdir(parents=True,exist_ok=True)
label=sys.argv[1]
duration=float(sys.argv[2]) if len(sys.argv)>2 else 24.
cfg=ROSLaunchConfig()
XmlLoader().load(str(root/'src/mm_remani/launch/exp0_real.launch'),cfg,
                argv=['hardware_enabled:=false'],verbose=False)
planner=next(n for n in cfg.nodes if n.name=='remani_planner_node')
socket.setdefaulttimeout(1)
master_log=(out/(label+'_master.log')).open('w')
node_log=(out/(label+'_planner.log')).open('w')
master=subprocess.Popen(['roscore','-p','11443'],stdout=master_log,stderr=subprocess.STDOUT,start_new_session=True)
node=None; mppi_node=None; rows=[]; mppi_states=[]; start=None
def limited():
    resource.setrlimit(resource.RLIMIT_AS,(2048*1024**2,2048*1024**2))
try:
    for _ in range(40):
        try:
            if xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/snapshot_probe')[0]==1:break
        except Exception: time.sleep(.2)
    else: raise RuntimeError('isolated master failed to start')
    rospy.init_node('snapshot_probe_readonly',disable_signals=True)
    for key,value in cfg.params.items():
        if key.startswith('/remani_planner_node/'):
            rospy.set_param(key,value.value)
    rospy.set_param('/use_sim_time',False)
    rospy.set_param('/remani_planner_node/grid_map/performance_log_interval',1.)
    args=[str(root/'devel/lib/remani_planner/remani_planner_node'),'__name:=remani_planner_node']
    args += [source+':='+target for source,target in planner.remap_args]
    node=subprocess.Popen(args,stdout=node_log,stderr=subprocess.STDOUT,start_new_session=True,preexec_fn=limited)
    if '_mppi' in label:
        from std_msgs.msg import String
        for key,value in cfg.params.items():
            if key.startswith('/whole_body_mppi/'):
                rospy.set_param(key,value.value)
        rospy.set_param('/whole_body_mppi/execute',False)
        # This tests steady computation only, without granting arm startup
        # authority or impersonating Piper execution feedback.
        rospy.set_param('/whole_body_mppi/native_arm_startup',False)
        rospy.set_param('/whole_body_mppi/startup_to_standby',False)
        mppi_log=(out/(label+'_mppi.log')).open('w')
        mppi_node=subprocess.Popen(['/usr/bin/python3','-B',str(root/'src/mm_remani/scripts/whole_body_mppi.py'),
            '__name:=whole_body_mppi'],stdout=mppi_log,stderr=subprocess.STDOUT,start_new_session=True)
        status_sub=rospy.Subscriber('/mppi/status',String,
            lambda msg:mppi_states.append(dict(elapsed=time.monotonic()-start if start else -1,
                                               **json.loads(msg.data))),queue_size=1)
    def snapshot(msg):
        h=np.asarray(msg.data[:32]).tolist(); now=rospy.get_time()
        rows.append(dict(received=now,elapsed=time.monotonic()-start if start else -1,
                         map_age=now-h[3],dynamic_age=now-h[18],joint_age=now-h[24],header=h))
    sub=rospy.Subscriber('/mppi/collision_snapshot',numpy_msg(Float64MultiArray),snapshot,
                         queue_size=1,buff_size=8*1024**2,tcp_nodelay=True)
    odom_pub=rospy.Publisher('/odom',Odometry,queue_size=1)
    joint_pub=rospy.Publisher('/mm/joint_states',JointState,queue_size=1)
    raw_joint_pub=rospy.Publisher('/joint_states_single',JointState,queue_size=1)
    cloud_pub=rospy.Publisher('/velodyne_points',PointCloud2,queue_size=1)
    dyn_pub=rospy.Publisher('/dynamic_points',PointCloud2,queue_size=1)
    global_pub=rospy.Publisher('/map_generator/global_cloud',PointCloud2,queue_size=1,latch=True)
    with (root/'src/mm_remani/maps/exp0_planner.pcd').open('rb') as f:
        while not f.readline().startswith(b'DATA '): pass
        global_points=np.frombuffer(f.read(),dtype='<f4').reshape(-1,3)
    global_pub.publish(point_cloud2.create_cloud_xyz32(Header(frame_id='world',stamp=rospy.Time.now()),global_points))
    broadcaster=tf2_ros.StaticTransformBroadcaster()
    tr=TransformStamped();tr.header.frame_id='world';tr.child_frame_id='probe_lidar'
    tr.transform.translation.x=.28;tr.transform.translation.z=.117;tr.transform.rotation.w=1.
    tr.header.stamp=rospy.Time.now();broadcaster.sendTransform(tr)
    params={k:v.value for k,v in cfg.params.items()}
    q=np.deg2rad(params['/remani_planner_node/fsm/init_state'][2:]).tolist()
    fields=[PointField(name,i*4,PointField.FLOAT32,1) for i,name in enumerate(('x','y','z','vx','vy','vz','confidence'))]
    fields.append(PointField('track_id',28,PointField.UINT32,1))
    yy,zz=np.meshgrid(np.linspace(-1.5,1.5,50),np.linspace(.1,1.3,40))
    points=np.column_stack([np.full(yy.size,1.6),yy.ravel(),zz.ravel()])
    if '_room' in label:
        # Same ~2k returns, spread over the actual room rather than one plane.
        points=global_points[::14].astype(float)-np.array([.28,0.,.117])
    rng=np.random.RandomState(31)
    start=time.monotonic();next_cloud=start
    while time.monotonic()-start<duration and not rospy.is_shutdown():
        if node.poll() is not None: raise RuntimeError('isolated planner exited: '+str(node.returncode))
        now=rospy.Time.now();stamp=rospy.Time.from_sec(now.to_sec()-.02)
        joints=JointState();joints.header.stamp=stamp;joints.name=['joint%d'%i for i in range(1,7)]
        joints.position=q;joints.velocity=[0.]*6;joint_pub.publish(joints);raw_joint_pub.publish(joints)
        if time.monotonic()>=next_cloud:
            elapsed=time.monotonic()-start
            odom=Odometry();odom.header.frame_id='world';odom.child_frame_id='base_link';odom.header.stamp=stamp
            odom.pose.pose.orientation.w=1.;odom_pub.publish(odom)
            cloud_points=points.copy()
            if elapsed>=12:
                if '_room' in label:cloud_points+=rng.uniform(-.04,.04,cloud_points.shape)
                else:cloud_points[:,0]+=.075*math.sin(elapsed*2.)
            if not ('_outage' in label and elapsed>=duration-5.):
                cloud_pub.publish(point_cloud2.create_cloud_xyz32(Header(frame_id='probe_lidar',stamp=stamp),cloud_points))
                dyn_pub.publish(point_cloud2.create_cloud(Header(frame_id='world',stamp=stamp),fields,[]))
            next_cloud+=.1
        time.sleep(.01)
    sub.unregister()
finally:
    for process in [mppi_node,node,master]:
        if process is None or process.poll() is not None:continue
        os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
    node_log.close();master_log.close()
    (out/(label+'_headers.json')).write_text(json.dumps(rows,indent=2)+'\n')
    if mppi_states:(out/(label+'_mppi_states.json')).write_text(json.dumps(mppi_states,indent=2)+'\n')
summary=dict(scope='isolated localhost master; real planner and synthetic point/joint/odom publishers; labels containing mppi add execute=false MPPI computation with native startup disabled; no drivers, actuator publishers, or hardware',label=label,
             max_child_rss_kib=resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss,phases={})
phases=[('steady',5.,12.),('changing',14.,24.)]
if duration>24:phases.append(('changing_extended',14.,duration-5.))
if '_outage' in label:phases.append(('input_outage',duration-4.,duration))
for phase,lo,hi in phases:
    selected=[r for r in rows if lo<=r['elapsed']<hi]
    if not selected:raise RuntimeError('no snapshots in '+phase)
    good=[r for r in selected if r['header'][4]==1]
    summary['phases'][phase]=dict(packets=len(selected),healthy=len(good),
        healthy_fraction=len(good)/len(selected),map_age_median_ms=float(np.median([r['map_age'] for r in selected])*1000),
        map_age_p95_ms=float(np.percentile([r['map_age'] for r in selected],95)*1000),
        dynamic_invalid=sum(r['header'][19]!=1 for r in selected),
        joint_age_p95_ms=float(np.percentile([r['joint_age'] for r in selected],95)*1000),
        healthy_remaining_under_85ms=sum(.35-r['map_age']<.085 for r in good))
    if mppi_states:
        from collections import Counter
        summary['phases'][phase]['mppi_states']=dict(Counter(r['state'] for r in mppi_states if lo<=r['elapsed']<hi))
        delivered=[r for r in mppi_states if lo<=r['elapsed']<hi and r['state']=='candidate_only']
        if len(delivered)>1:
            summary['phases'][phase]['candidate_gap_max_ms']=max(b['elapsed']-a['elapsed'] for a,b in zip(delivered,delivered[1:]))*1000
            summary['phases'][phase]['modeled_lease_gap_violations']=sum(b['elapsed']-(a['elapsed']-a['cycle_ms']/1000)>.30 for a,b in zip(delivered,delivered[1:]))
(out/(label+'_summary.json')).write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
