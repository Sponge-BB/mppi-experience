#!/usr/bin/python3
"""Bounded, read-only startup recorder. No publisher, service, SDK or CAN."""
import datetime
import json
import math
import os
from pathlib import Path
import resource
import struct
import sys
import threading
import time
from collections import Counter

stamp = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
out = Path(__file__).resolve().parent/'results'/('startup_listen_'+stamp)
out.mkdir(parents=True, exist_ok=True)
os.environ['ROS_LOG_DIR'] = str(out/'ros')
import rospy
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PolynomialTraj
from piper_msgs.msg import PiperExecutionState
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String, UInt8MultiArray

navigation = '--navigation' in sys.argv
arm_only = '--arm-only' in sys.argv or navigation

lock = threading.Lock()
log = (out/'events.jsonl').open('a', buffering=1)
counts = Counter()
last_sample = {}
last_flags = {}
latest = {}

def record(topic, values, minimum_interval=0., flags=None):
    now = time.time()
    with lock:
        counts[topic] += 1
        latest[topic] = values
        changed = flags is not None and last_flags.get(topic) != flags
        if changed:
            last_flags[topic] = flags
        if not changed and now-last_sample.get(topic, 0.) < minimum_interval:
            return
        last_sample[topic] = now
        log.write(json.dumps(dict(received=now, topic=topic, **values), ensure_ascii=False)+'\n')
    if changed and topic in ('/mppi/status', '/piper_execution_state'):
        print(json.dumps(dict(event=topic, **values), ensure_ascii=False), flush=True)

def snapshot(msg):
    try:
        b = memoryview(msg._buff)
        dims, = struct.unpack_from('<I', b, 0)
        if dims > 16:
            raise ValueError('unexpected layout')
        offset = 4
        for _ in range(dims):
            length, = struct.unpack_from('<I', b, offset)
            offset += 4+length+8
        offset += 4  # layout.data_offset
        count, = struct.unpack_from('<I', b, offset)
        offset += 4
        if count < 32 or offset+count*8 != len(b):
            raise ValueError('incomplete snapshot')
        h = struct.unpack_from('<32d', b, offset)
        if not all(math.isfinite(v) for v in h):
            raise ValueError('nonfinite snapshot header')
        now = rospy.get_time()
        record('/mppi/collision_snapshot', dict(header=h,
            map_age_ms=(now-h[3])*1000, dynamic_age_ms=(now-h[18])*1000,
            cached_joint_age_ms=(now-h[24])*1000,
            copy_to_receive_ms=(now-h[17])*1000))
    except (ValueError, struct.error) as exc:
        record('recorder_error', dict(input='/mppi/collision_snapshot', error=str(exc)), 1.)

def cloud(topic, msg):
    try:
        b = memoryview(msg._buff)
        _, sec, nsec, frame_len = struct.unpack_from('<IIII', b, 0)
        height, width = struct.unpack_from('<II', b, 16+frame_len)
        source = sec+nsec*1e-9
        record(topic, dict(stamp=source, age_ms=(rospy.get_time()-source)*1000,
                           points=height*width))
    except struct.error as exc:
        record('recorder_error', dict(input=topic, error=str(exc)), 1.)

def execution(msg):
    flags = (msg.enabled, msg.command_active, msg.stop_latched, msg.feedback_valid)
    record('/piper_execution_state', dict(stamp=msg.header.stamp.to_sec(),
        enabled=flags[0], command_active=flags[1], stop_latched=flags[2],
        feedback_valid=flags[3], valid_until=msg.valid_until.to_sec()), .05, flags)

def joints(topic, msg):
    record(topic, dict(stamp=msg.header.stamp.to_sec(),
        age_ms=(rospy.get_time()-msg.header.stamp.to_sec())*1000,
        names=list(msg.name), position=list(msg.position), velocity=list(msg.velocity)),
        .05 if topic == '/joint_states_single' else 0.)

def status(msg):
    try:
        data = json.loads(msg.data)
        record('/mppi/status', data, flags=data.get('state'))
    except ValueError as exc:
        record('recorder_error', dict(input='/mppi/status', error=str(exc)), 1.)

def odometry(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    record('/odom', dict(stamp=msg.header.stamp.to_sec(), frame=msg.header.frame_id,
        x=p.x, y=p.y, z=p.z, quaternion=[q.x, q.y, q.z, q.w],
        v=msg.twist.twist.linear.x, w=msg.twist.twist.angular.z), .05)

def trajectory(msg):
    record('/planning/trajectory', dict(stamp=msg.header.stamp.to_sec(),
        action=msg.action, trajectory_id=msg.trajectory_id, singul=msg.singul,
        pieces=[dict(num_dim=p.num_dim, num_order=p.num_order, duration=p.duration,
                     data=list(p.data)) for p in msg.trajectory]))

def rosout(msg):
    if msg.name not in ('/whole_body_mppi', '/piper_ctrl_single_node',
                        '/remani_planner_node', '/robot_self_filter', '/dynamic_obstacle_tracker'):
        return
    if msg.level >= Log.WARN or any(s in msg.msg for s in (
            'Piper native', 'startup_ready', '[GridMap][pipeline]', '[GridMap][LocalESDF]')):
        record('/rosout_agg', dict(stamp=msg.header.stamp.to_sec(),
               node=msg.name, level=msg.level, message=msg.msg))

rospy.init_node('hardware_startup_readonly', anonymous=True, disable_signals=True)
subs = [rospy.Subscriber('/mppi/status', String, status, queue_size=10),
        rospy.Subscriber('/piper_execution_state', PiperExecutionState, execution,
                         queue_size=1, tcp_nodelay=True),
        rospy.Subscriber('/rosout_agg', Log, rosout, queue_size=100)]
if navigation:
    subs.extend([
        rospy.Subscriber('/mm_controller_node/trajectory_progress', Vector3Stamped,
                         lambda m:record('/mm_controller_node/trajectory_progress',
                         dict(stamp=m.header.stamp.to_sec(), start=m.vector.x,
                              phase=m.vector.y, scale=m.vector.z)),
                         queue_size=1, tcp_nodelay=True),
        rospy.Subscriber('/move_base_simple/goal', PoseStamped,
                         lambda m:record('/move_base_simple/goal',
                         dict(stamp=m.header.stamp.to_sec(), frame=m.header.frame_id,
                              x=m.pose.position.x, y=m.pose.position.y,
                              quaternion=[m.pose.orientation.x, m.pose.orientation.y,
                                          m.pose.orientation.z, m.pose.orientation.w])),
                         queue_size=1, tcp_nodelay=True),
        rospy.Subscriber('/odom', Odometry, odometry, queue_size=1, tcp_nodelay=True),
        rospy.Subscriber('/planning/trajectory', PolynomialTraj, trajectory, queue_size=20),
        rospy.Subscriber('/planning/finish', Bool, lambda m:record('/planning/finish',
                         dict(finished=m.data)), queue_size=10),
        rospy.Subscriber('/planning/safety_speed_limit', Float64,
                         lambda m:record('/planning/safety_speed_limit', dict(limit=m.data)), queue_size=10),
        rospy.Subscriber('/mppi/collision_snapshot', rospy.AnyMsg, snapshot,
                         queue_size=1, buff_size=8*1024**2, tcp_nodelay=True),
        rospy.Subscriber('/cmd_vel', Twist, lambda m:record('/cmd_vel',
                         dict(v=m.linear.x, w=m.angular.z)), queue_size=20, tcp_nodelay=True),
        rospy.Subscriber('/my_car_vel', Twist, lambda m:record('/my_car_vel',
                         dict(v=m.linear.x, w=m.angular.z)), queue_size=20, tcp_nodelay=True),
        rospy.Subscriber('/tracking_fault', Bool, lambda m:record('/tracking_fault',
                         dict(active=m.data)), queue_size=20)])
if arm_only:
    subs.append(rospy.Subscriber('/piper_motor_enable_states', UInt8MultiArray,
        lambda m:record('/piper_motor_enable_states', dict(enabled=list(m.data)),
                        .2, tuple(m.data)), queue_size=1, tcp_nodelay=True))
else:
    subs.extend([
        rospy.Subscriber('/mppi/collision_snapshot', rospy.AnyMsg, snapshot,
                         queue_size=1, buff_size=8*1024**2, tcp_nodelay=True),
        rospy.Subscriber('/cmd_vel', Twist, lambda m:record('/cmd_vel',
                         dict(v=m.linear.x, w=m.angular.z)), queue_size=1)])
for topic in ('/joint_states_single', '/joint_ctrl_single'):
    subs.append(rospy.Subscriber(topic, JointState, lambda m,t=topic:joints(t,m),
                                queue_size=1, tcp_nodelay=True))
for topic in (() if arm_only else ('/cloud_registered', '/livox/lidar_no_ground', '/velodyne_points')):
    subs.append(rospy.Subscriber(topic, rospy.AnyMsg, lambda m,t=topic:cloud(t,m),
                                queue_size=1, buff_size=2*1024**2, tcp_nodelay=True))
(out/'listener.json').write_text(json.dumps(dict(pid=os.getpid(), node=rospy.get_name(),
    master=rospy.get_master().getUri()[2], mode='subscriptions only; no command/service/SDK',
    arm_only=arm_only, navigation=navigation, maximum_duration_s=1200), indent=2)+'\n')
print(json.dumps(dict(ready=True, output=str(out), node=rospy.get_name()), ensure_ascii=False), flush=True)
start = time.monotonic()
try:
    while not rospy.is_shutdown() and time.monotonic()-start < 1200:
        time.sleep(1.)
        with lock:
            (out/'summary.json').write_text(json.dumps(dict(counts=counts, latest=latest,
                max_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss), ensure_ascii=False, indent=2)+'\n')
        if int(time.monotonic()-start) % 10 == 0:
            print(json.dumps(dict(listening=True, counts=dict(counts),
                  max_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)), flush=True)
finally:
    for sub in subs:
        sub.unregister()
    rospy.signal_shutdown('read-only listener completed')
    log.close()
