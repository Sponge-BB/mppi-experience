#!/usr/bin/python3
"""Synthetic input integration check, restricted to our isolated monitor master."""
import json
import math
import os
from pathlib import Path
import time
import xmlrpc.client

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Bool, Header, String
from visualization_msgs.msg import Marker, MarkerArray

ROOT = Path(__file__).resolve().parent
RELEASE = ROOT / 'releases/standby_20261105_01'
URI = 'http://127.0.0.1:11461'
FORBIDDEN = ('/cmd_vel', '/joint_ctrl_single', '/mm_controller_node/joint_cmd',
             '/move_base_simple/goal', '/sim/whole_body_command')


def main():
    assert os.environ.get('ROS_MASTER_URI') == URI
    session = json.loads((RELEASE / 'run/session.json').read_text())
    assert session['master_uri'] == URI and session['hardware_publishers'] is False
    master = xmlrpc.client.ServerProxy(URI)
    code, _, graph = master.getSystemState('/standby_synthetic_fixture_audit')
    assert code == 1
    assert not any(topic in FORBIDDEN for topic, _ in graph[0])
    assert all(all('hardware_standby_shadow' in node or node == '/rosout'
                   for node in nodes) for _, nodes in graph[0])
    config = json.loads((RELEASE / 'config/hardware_shadow.json').read_text())
    namespace = config['namespace']; topics = config['topics']
    rospy.init_node('synthetic_input_fixture', anonymous=True, disable_signals=True)
    statuses = []; marker_counts = []
    def status_callback(msg):
        statuses.append(json.loads(msg.data))
    def marker_callback(msg):
        marker_counts.append(sum(m.type == Marker.SPHERE and m.action == Marker.ADD
                                 for m in msg.markers))
    subscribers = [rospy.Subscriber(namespace + '/status', String, status_callback),
                   rospy.Subscriber(namespace + '/markers', MarkerArray, marker_callback)]
    pubs = {key: rospy.Publisher(topics[key], kind, queue_size=1)
            for key, kind in [('odom', Odometry), ('joints', JointState),
                              ('raw_cloud', PointCloud2), ('filtered_cloud', PointCloud2),
                              ('static_cloud', PointCloud2), ('dynamic_cloud', PointCloud2),
                              ('perception_health', Bool)]}
    fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
              for i, name in enumerate(('x', 'y', 'z', 'vx', 'vy', 'vz', 'confidence'))]
    fields.append(PointField(name='track_id', offset=28, datatype=PointField.UINT32, count=1))
    q = [0., math.radians(110.), math.radians(-60.), 0., math.radians(30.), 0.]
    begin = time.monotonic()
    try:
        while time.monotonic() - begin < 12.:
            stamp = rospy.Time.now()
            odom = Odometry(); odom.header.stamp = stamp
            odom.header.frame_id = 'odom'; odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = 1.; odom.pose.pose.position.y = -.2
            odom.pose.pose.position.z = .115
            odom.pose.pose.orientation.z = math.sin(.05)
            odom.pose.pose.orientation.w = math.cos(.05)
            joints = JointState(); joints.header.stamp = stamp
            joints.header.frame_id = 'base_link'
            joints.name = ['joint%d' % i for i in range(1, 7)]
            joints.position = q; joints.velocity = [9999.] * 6
            pubs['odom'].publish(odom); pubs['joints'].publish(joints)
            header = Header(stamp=stamp, frame_id='odom')
            points = [(6.+.03*i, 6., .8+.02*(i % 5)) for i in range(30)]
            cloud = point_cloud2.create_cloud_xyz32(header, points)
            for key in ('raw_cloud', 'filtered_cloud', 'static_cloud'):
                pubs[key].publish(cloud)
            dynamic = [(5.+.02*i, 4., 1.1, .08, 0., 0., 1., 7) for i in range(10)]
            pubs['dynamic_cloud'].publish(point_cloud2.create_cloud(header, fields, dynamic))
            pubs['perception_health'].publish(Bool(data=True))
            time.sleep(.05)
    finally:
        for pub in pubs.values():
            pub.unregister()
    time.sleep(1.2)
    assert statuses
    assert all(not row['hardware_execution_eligible'] and not row['commands_executed']
               for row in statuses)
    anchored = [row for row in statuses if row['anchor'] is not None]
    assert anchored and max(marker_counts) == 67
    assert abs(anchored[-1]['anchor'][0] - 1.) < 1e-6
    assert abs(anchored[-1]['anchor'][1] + .2) < 1e-6
    assert any(row['state'] is not None and max(abs(v) for v in row['state'][11:]) < 1e-8
               for row in anchored), 'Unverified driver velocities must be ignored'
    assert statuses[-1]['status'] in ('state_stale', 'waiting_valid_inputs', 'waiting_real_state')
    _, _, graph = master.getSystemState('/standby_synthetic_fixture_audit')
    assert not any(topic in FORBIDDEN for topic, _ in graph[0])
    report = {'scope': 'isolated synthetic inputs; not a real robot avoidance result',
              'passed': True, 'messages': len(statuses),
              'statuses': {key: sum(row['status'] == key for row in statuses)
                           for key in sorted(set(row['status'] for row in statuses))},
              'measured_sphere_count': max(marker_counts),
              'anchor_captured_from_fixture': anchored[-1]['anchor'],
              'candidates_proposed_never_executed': sum(row['proposed_controls'] is not None
                                                        for row in statuses),
              'solver_calls': sum(bool(row.get('kernel_diagnostics')) for row in statuses),
              'hardware_execution_eligible': False, 'hardware_publishers': False,
              'after_fixture_status': statuses[-1]['status'],
              'elapsed_s': time.monotonic() - begin}
    (ROOT / 'synthetic_input_integration_audit.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    for sub in subscribers:
        sub.unregister()
    rospy.signal_shutdown('synthetic fixture completed; no drivers involved')


if __name__ == '__main__':
    main()
