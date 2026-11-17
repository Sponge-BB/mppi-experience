#!/usr/bin/python3
"""Latch ten actually observed filtered frames for this stationary shadow run."""
import time
import threading
import numpy as np
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from pointcloud_numpy import read_points_numpy
from hardware_cloud_snapshot import transform_points

NS = '/standby_trial_20261106_stationary_07/perception'
WORLD = 'standby_feedback_02_odom'


def main():
    rospy.init_node('stationary_background')
    buffer = tf2_ros.Buffer(debug=False); listener = tf2_ros.TransformListener(buffer)
    publisher = rospy.Publisher(NS+'/background', PointCloud2, queue_size=1, latch=True)
    state = {'odom': None, 'frames': [], 'anchor': None, 'done': False}
    lock = threading.Lock(); start = time.monotonic()

    def odom(msg):
        with lock:
            state['odom'] = msg

    def cloud(msg):
        with lock:
            if state['done'] or time.monotonic()-start < 6.:
                return
            o = state['odom']
            if (o is None or rospy.get_time()-o.header.stamp.to_sec() > .25
                    or abs(o.twist.twist.linear.x) > .02 or abs(o.twist.twist.angular.z) > .04):
                state['frames'] = []; state['anchor'] = None
                return
            xy = np.array([o.pose.pose.position.x, o.pose.pose.position.y])
            if state['anchor'] is None:
                state['anchor'] = xy
            if np.linalg.norm(xy-state['anchor']) > .02:
                state['frames'] = []; state['anchor'] = xy
                return
            try:
                tr = buffer.lookup_transform(WORLD, msg.header.frame_id,
                                              msg.header.stamp, rospy.Duration(.02)).transform
                points = read_points_numpy(msg, ('x', 'y', 'z'))
                points, _ = transform_points(points, [tr.translation.x, tr.translation.y, tr.translation.z],
                                             [tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w])
                points = points[np.isfinite(points).all(axis=1)]
                if not len(points):
                    return
                if len(points) > 12000:
                    points = points[np.linspace(0, len(points)-1, 12000, dtype=int)]
                state['frames'].append(points)
                if len(state['frames']) < 10:
                    return
                points = np.concatenate(state['frames'])
                _, indices = np.unique(np.floor(points/.04).astype(np.int32), axis=0, return_index=True)
                points = points[indices]
                if len(points) > 15000:
                    points = points[np.linspace(0, len(points)-1, 15000, dtype=int)]
                points = np.ascontiguousarray(points, dtype='<f4')
                fields = [PointField(name, i*4, PointField.FLOAT32, 1) for i, name in enumerate(('x','y','z'))]
                output = PointCloud2(header=Header(stamp=msg.header.stamp, frame_id=WORLD), height=1,
                                    width=len(points), fields=fields, is_bigendian=False, point_step=12,
                                    row_step=12*len(points), data=points.tobytes(), is_dense=True)
                publisher.publish(output)
                state['done'] = True; state['frames'] = []
                rospy.loginfo('Latched %d observed background points; no free-space certification.', len(points))
            except (ValueError, tf2_ros.TransformException) as exc:
                rospy.logwarn_throttle(3., 'Background waiting for valid TF: %s', str(exc))

    rospy.Subscriber('/standby_trial_20261105_feedback_02/sensors/odom', Odometry, odom, queue_size=1)
    rospy.Subscriber(NS+'/filtered_points', PointCloud2, cloud, queue_size=1, buff_size=4*1024**2)
    rospy.spin()


if __name__ == '__main__':
    main()
