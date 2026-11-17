"""Bounded stationary measurement: subscriptions only, no control publishers."""
import json
import os
from pathlib import Path
import socket
import threading
import time
import xmlrpc.client

import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import Bool, Float64, Float64MultiArray, Int32, String, UInt8MultiArray
from nav_msgs.msg import Odometry
from piper_msgs.msg import PiperExecutionState
from rosgraph_msgs.msg import Log

socket.setdefaulttimeout(3)
out = Path(__file__).parent / (os.environ.get('SAMPLE_NAME', 'local') + '.json')
master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
result = {'scope': 'Stationary subscriptions only; no motion or enable output.',
          'graph_before': master.getSystemState('/local_esdf_observe')[2]}
rospy.init_node('local_esdf_observe', anonymous=True, disable_signals=True, disable_rosout=True)
lock = threading.Lock()
rows, logs, resources = {}, [], []
def save(topic, record):
    record['received'] = rospy.get_time()
    with lock:
        rows.setdefault(topic, []).append(record)
def snap(m):
    if len(m.data) >= 32:
        save('/mppi/collision_snapshot', {'header': m.data[:32].tolist()})
def cloud(topic):
    return lambda m: save(topic, {'stamp': m.header.stamp.to_sec(), 'points': m.width*m.height, 'frame': m.header.frame_id})
def value(topic):
    return lambda m: save(topic, {'value': list(m.data) if isinstance(m.data, (bytes, tuple)) else m.data})
def joint(m):
    save('/joint_states_single', {'stamp': m.header.stamp.to_sec(), 'q': list(m.position)})
def odom(m):
    p=m.pose.pose.position
    save('/odom', {'stamp': m.header.stamp.to_sec(), 'xyz': [p.x,p.y,p.z]})
def execution(m):
    def scalar(v):
        if hasattr(v,'to_sec'): return v.to_sec()
        if hasattr(v,'stamp'): return v.stamp.to_sec()
        if isinstance(v,bytes): return list(v)
        return v
    save('/piper_execution_state', {k: scalar(getattr(m,k)) for k in m.__slots__})
def log(m):
    if any(s in m.msg for s in ['GridMap', 'DynamicPointFlow', 'DynamicTracker', 'Reject ICP', 'Not match']):
        with lock:
            logs.append({'stamp': m.header.stamp.to_sec(), 'name': m.name, 'level': m.level, 'msg': m.msg})
subs = [rospy.Subscriber('/mppi/collision_snapshot', numpy_msg(Float64MultiArray), snap, queue_size=1, buff_size=8*1024*1024),
        rospy.Subscriber('/joint_states_single', JointState, joint, queue_size=1),
        rospy.Subscriber('/odom', Odometry, odom, queue_size=1),
        rospy.Subscriber('/piper_execution_state', PiperExecutionState, execution, queue_size=1),
        rospy.Subscriber('/rosout_agg', Log, log, queue_size=100)]
for t in ['/dynamic_points', '/velodyne_points']:
    subs.append(rospy.Subscriber(t, PointCloud2, cloud(t), queue_size=1, buff_size=2*1024*1024))
for t,typ in [('/piper_motor_enable_states',UInt8MultiArray),('/mppi/status',String)]+[
        ('/dynamic_obstacle_tracker/'+n,c) for n,c in [('perception_healthy',Bool),('motion_qualified_track_count',Int32),('input_age',Float64),('processing_ms',Float64)]]:
    subs.append(rospy.Subscriber(t,typ,value(t),queue_size=1))
start=time.monotonic()
for _ in range(30):
    mem={line.split(':')[0]:int(line.split()[1]) for line in Path('/proc/meminfo').read_text().splitlines() if line.startswith(('MemAvailable:', 'SwapFree:'))}
    resources.append({'elapsed':time.monotonic()-start, **mem})
    time.sleep(1)
for s in subs:
    s.unregister()
result.update(seconds=time.monotonic()-start, graph_after=master.getSystemState('/local_esdf_observe')[2], records=rows, logs=logs, resources=resources)
def stats(a):
    return {k:float(v) for k,v in zip(['min','p50','p95','max'],np.percentile(a,[0,50,95,100]))} if len(a) else None
summary={'sample_seconds':result['seconds'], 'topic_counts': {t:len(r) for t,r in rows.items()}, 'min_mem_available_mib':min(r['MemAvailable'] for r in resources)/1024}
rr=rows.get('/mppi/collision_snapshot',[])
if rr:
    h=np.array([r['header'] for r in rr]); now=np.array([r['received'] for r in rr]); age=now-h[:,3]
    map_fresh=(age>=-.05)&(age<=h[:,15]); dyn_fresh=(h[:,19]==1)&((h[:,28]==0)|((now-h[:,18]>=-.05)&(now-h[:,18]<=h[:,15])))
    summary.update(map_age_s=stats(age),map_fresh_fraction=float(map_fresh.mean()),dynamic_valid_fraction=float((h[:,19]==1).mean()),controller_fresh_fraction=float(((h[:,4]==1)&map_fresh&dyn_fresh).mean()),header_valid_fraction=float((h[:,4]==1).mean()),map_age_limit_s=np.unique(h[:,15]).tolist(),dimensions=h[-1,5:8].tolist(),last_header=h[-1].tolist())
    for label,idx in [('distance_generation',14),('completed_fusion',29)]:
        ix=np.flatnonzero(np.r_[False,np.diff(h[:,idx])!=0])
        summary[label]={'changes':len(ix),'arrival_period_s':stats(np.diff(now[ix])),'arrival_age_s':stats(age[ix])}
for t in ['/dynamic_points','/velodyne_points','/joint_states_single','/odom']:
    rr=rows.get(t,[])
    if rr:
        summary[t]={'received_period_s':stats(np.diff([r['received'] for r in rr])),'age_s':stats([r['received']-r['stamp'] for r in rr]),'last':rr[-1]}
for t in ['/piper_execution_state','/piper_motor_enable_states','/mppi/status']:
    summary[t]=rows.get(t,[])[-1:] or None
summary['command_publishers']={t:dict(result['graph_after'][0]).get(t,[]) for t in ['/cmd_vel','/joint_ctrl_single','/enable_flag']}
result['summary']=summary
out.write_text(json.dumps(result,indent=2)+'\n')
out.with_name(out.stem+'_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
rospy.signal_shutdown('Stationary measurement completed')
