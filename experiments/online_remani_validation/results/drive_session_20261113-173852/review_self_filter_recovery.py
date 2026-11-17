#!/usr/bin/env python3
"""Small offline source-level reproductions. Never starts a ROS node/master."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
from collections import deque
import io
import json
from pathlib import Path
import struct
import sys
from unittest.mock import patch

ROOT=Path('/home/l/remani_ws')
sys.path.insert(0,str(ROOT/'src/mm_remani/test'))
import test_robot_self_filter as fixture
from sensor_msgs.msg import JointState
from rospy.msg import deserialize_messages
from std_msgs.msg import String

fixture.RobotSelfFilterTests.setUpClass()

def node_case():
    case=fixture.RobotSelfFilterTests();case.setUp();return case

def joint(case,stamp):
    msg=JointState();msg.header.stamp=fixture.rospy.Time.from_sec(stamp)
    msg.name=['joint%d'%i for i in range(1,7)];msg.position=case.q_a.tolist()
    case.node.joints_cb(msg)

case=node_case()
with patch('velodyne_transformer.rospy.get_time',side_effect=lambda:case.now),patch('velodyne_transformer.rospy.logwarn_throttle'):
    case.now=103.04
    case.node.cloud_cb(case.cloud([[1.,0.,.5]]))
    old_dropped=not case.node.pub.publish.called
    case.scan_stamp=case.now
    joint(case,case.now)
    case.node.cloud_cb(case.cloud([[1.,0.,.5]]))
    new_recovers=case.node.pub.publish.call_count==1

late=node_case()
with patch('velodyne_transformer.rospy.get_time',return_value=late.now),patch('velodyne_transformer.rospy.logwarn_throttle'):
    joint(late,100.04)
    valid_before=late.node.q is not None
    joint(late,99.8)
    old_packet_clears_new=late.node.q is None and len(late.node.joint_history)==0
    joint(late,100.045)
    next_fresh_recovers=late.node.q is not None

# Emulate TCPROS receive chunks, not a live socket. Each payload exceeds the
# transformer's default 64KiB recv. queue_size=1 prunes only complete messages
# in one read batch, so an already queued stream can still deliver every old
# frame in order when each message spans multiple recv calls.
wire=b''
for i in range(8):
    payload=io.BytesIO();String(data=('frame%02d:'%i)+'x'*(200*1024)).serialize(payload)
    data=payload.getvalue();wire+=struct.pack('<I',len(data))+data

def receive_chunks(chunk):
    stream=io.BytesIO();seen=[]
    for offset in range(0,len(wire),chunk):
        stream.write(wire[offset:offset+chunk]);queue=[]
        deserialize_messages(stream,queue,String,queue_size=1)
        seen.extend(m.data[:8] for m in queue)
    return seen

small=receive_chunks(65536)
large=receive_chunks(len(wire))
report=dict(no_master_or_ros_node=True,
    stale_cloud_then_fresh_pair=dict(stale_dropped=old_dropped,fresh_scan_and_joint_immediately_recovers=new_recovers),
    late_stale_joint_invalidates_newer_state=dict(valid_before=valid_before,old_packet_clears_newer_valid_state=old_packet_clears_new,next_fresh_recovers=next_fresh_recovers,
        applicability='Demonstrable robustness defect, but this capture does not establish that out-of-order old joint packets reached the self-filter.'),
    tcpros_queue1_chunk_test=dict(payload_bytes=200*1024,frames=8,recv64k_delivered=small,recv_entire_queued_stream_delivered=large,
        applicability='Proves queue_size=1 alone is not a cross-recv latest-frame guarantee; this synthetic test does not reproduce or measure the observed 51-second duration.'))
assert old_dropped and new_recovers and valid_before and old_packet_clears_new and next_fresh_recovers
assert len(small)==8 and large==['frame07:']
destination=Path(__file__).with_name('self_filter_recovery_reproductions.json')
destination.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
