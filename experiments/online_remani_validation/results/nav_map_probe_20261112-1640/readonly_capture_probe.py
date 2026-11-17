import collections
import json
import os
import sys
import threading
import time
from pathlib import Path

import rospy
import roslib.message
import tf2_ros
from rospy import AnyMsg
from std_msgs.msg import String

sys.path.insert(0, '/home/l/remani_ws/experiments/online_remani_validation')
from record_drive_session import array_header, field_key

out = Path('/home/l/remani_ws/experiments/online_remani_validation/results/nav_map_probe_20261112-1640')
out.mkdir(exist_ok=False)
lock = threading.Lock()
latest = {}
fields = collections.OrderedDict()
packets = collections.OrderedDict()
statuses = []
diagnostics = collections.deque(maxlen=2)
mapping_history = collections.deque(maxlen=80)
counts = collections.Counter()

def callback(msg, topic):
    now = rospy.get_time()
    raw = msg._buff
    with lock:
        counts[topic] += 1
        latest[topic] = (raw, dict(msg._connection_header), now)
        if topic == '/mppi/collision_snapshot':
            h = array_header(raw)
            key = field_key(h)
            packets[key] = (raw, h, now)
            if h[20] == 1:
                fields[key] = (raw, h, now)
            while len(fields) > 3:
                fields.popitem(last=False)
            while len(packets) > 3:
                packets.popitem(last=False)
        elif topic == '/mppi/collision_diagnostic':
            diagnostics.append((raw, now))
        elif topic == '/mppi/status':
            status = String().deserialize(raw)
            statuses.append(dict(receive_ros=now, status=json.loads(status.data)))
        elif topic == '/velodyne_points_mapping':
            mapping_history.append((raw, now))

os.environ['ROS_IP'] = '192.168.137.128'
os.environ.pop('ROS_HOSTNAME', None)
os.environ['ROS_MASTER_URI'] = 'http://192.168.137.107:11311'
rospy.init_node('nav_map_readonly_probe', anonymous=True, disable_signals=True)
tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(15.))
tf_listener = tf2_ros.TransformListener(tf_buffer)
topics = ['/mppi/status', '/mppi/collision_snapshot', '/mppi/collision_diagnostic',
          '/odom', '/mm/joint_states', '/velodyne_points', '/velodyne_points_mapping',
          '/dynamic_points', '/planning/safety_speed_limit', '/planning/trajectory',
          '/map_generator/global_cloud']
subs = [rospy.Subscriber(topic, AnyMsg, callback, callback_args=topic,
                        queue_size=1, buff_size=8*1024**2) for topic in topics]
time.sleep(7)
for sub in subs:
    sub.unregister()
summary = dict(counts=counts, topics={})
transforms = {}
for topic in ('/velodyne_points', '/velodyne_points_mapping', '/dynamic_points'):
    if topic not in latest:
        continue
    from sensor_msgs.msg import PointCloud2
    cloud = PointCloud2().deserialize(latest[topic][0])
    try:
        tr = tf_buffer.lookup_transform('world', cloud.header.frame_id,
                                        cloud.header.stamp, rospy.Duration(.2)).transform
        transforms[topic] = dict(source_frame=cloud.header.frame_id,
            source_stamp=cloud.header.stamp.to_sec(),
            translation=[tr.translation.x, tr.translation.y, tr.translation.z],
            rotation=[tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w])
    except Exception as exc:
        transforms[topic] = dict(error=str(exc))
(out/'cloud_transforms.json').write_text(json.dumps(transforms, indent=2))
history = []
for index, (raw, receive) in enumerate(mapping_history):
    from sensor_msgs.msg import PointCloud2
    cloud = PointCloud2().deserialize(raw)
    try:
        tr = tf_buffer.lookup_transform('world', cloud.header.frame_id,
                                        cloud.header.stamp, rospy.Duration(.05)).transform
        filename = 'mapping_%03d.msg' % index
        (out/filename).write_bytes(raw)
        history.append(dict(file=filename, source_stamp=cloud.header.stamp.to_sec(),
            receive_ros=receive,
            translation=[tr.translation.x, tr.translation.y, tr.translation.z],
            rotation=[tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w]))
    except Exception as exc:
        history.append(dict(source_stamp=cloud.header.stamp.to_sec(), error=str(exc)))
(out/'mapping_history.json').write_text(json.dumps(history, indent=2))
with lock:
    (out/'statuses.json').write_text(json.dumps(statuses, ensure_ascii=False, indent=2))
    for topic, (raw, header, receive) in latest.items():
        if topic == '/mppi/collision_snapshot':
            continue
        name = topic.strip('/').replace('/', '_')
        (out/(name+'.msg')).write_bytes(raw)
        summary['topics'][topic] = dict(file=name+'.msg', type=header['type'], receive_ros=receive)
        if topic not in ('/velodyne_points', '/velodyne_points_mapping', '/dynamic_points', '/mppi/collision_diagnostic', '/map_generator/global_cloud'):
            obj = roslib.message.get_message_class(header['type'])().deserialize(raw)
            summary['topics'][topic]['value'] = str(obj)[:12000]
    pairs = []
    for index, key in enumerate(fields):
        if key not in packets:
            continue
        (out/('field%d.msg' % index)).write_bytes(fields[key][0])
        (out/('packet%d.msg' % index)).write_bytes(packets[key][0])
        pairs.append(dict(index=index, header=list(packets[key][1]), receive_ros=packets[key][2]))
    summary['map_pairs'] = pairs
    for index, (raw, receive) in enumerate(diagnostics):
        (out/('diagnostic%d.msg' % index)).write_bytes(raw)
    summary['diagnostic_count'] = len(diagnostics)
(out/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2))
print(json.dumps(dict(directory=str(out), counts=counts, map_pairs=len(pairs),
                     status_states=collections.Counter(x['status'].get('state') for x in statuses)), ensure_ascii=False))
rospy.signal_shutdown('read-only capture complete')
