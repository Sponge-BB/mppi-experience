#!/usr/bin/env python3
"""Subscribe to /odom and broadcast odom->base_link TF (dynamic).

Features:
- Subscribes to a configurable odom topic and republishes a TF from `frame_id` -> `child_frame`.
- Respects ROS simulated time (waits if necessary).
- Optionally publishes a static transform from `base_link` to another frame (example: dome).
- Periodically republishes the last transform so TF listeners get updates even if odom messages are sparse.
"""
import rospy
from nav_msgs.msg import Odometry
import tf2_ros
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
import math


class OdomToTF(object):
    def __init__(self):
        rospy.init_node('odom_to_tf')

        # Parameters
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')
        self.child_frame = rospy.get_param('~child_frame', 'base_link')
        self.frame_id = rospy.get_param('~frame_id', 'odom')
        self.queue_size = rospy.get_param('~queue_size', 10)
        self.pub_rate = rospy.get_param('~pub_rate', 10.0)  # Hz for periodic republish

        # Optional static transform from base_link to another frame (e.g., dome)
        self.publish_base_to_extra = rospy.get_param('~publish_base_to_extra', False)
        self.extra_frame = rospy.get_param('~extra_frame', 'dome')
        self.extra_xyz = rospy.get_param('~extra_xyz', [0.0, 0.0, 0.0])
        self.extra_rpy = rospy.get_param('~extra_rpy', [0.0, 0.0, 0.0])

        # TF broadcasters
        self.br = tf2_ros.TransformBroadcaster()
        self.static_br = tf2_ros.StaticTransformBroadcaster() if self.publish_base_to_extra else None

        # internal state
        self._last_transform = None
        self._last_stamp = None

        # wait for /clock if using simulated time and it's not published yet
        if rospy.get_param('/use_sim_time', False):
            rospy.loginfo('use_sim_time is true: waiting for /clock to start')
            # wait up to a short timeout for /clock to appear
            timeout = rospy.get_time() + 5.0 if rospy.get_time() != 0 else None
            # simple sleep to let clock start (ROS will proceed even if not yet publishing)
            rospy.sleep(0.1)

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=self.queue_size)

        # optionally publish static base->extra transform once
        if self.publish_base_to_extra and self.static_br is not None:
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = self.child_frame
            t.child_frame_id = self.extra_frame
            t.transform.translation.x = float(self.extra_xyz[0])
            t.transform.translation.y = float(self.extra_xyz[1])
            t.transform.translation.z = float(self.extra_xyz[2])
            # convert rpy to quaternion
            q = self._rpy_to_quat(*self.extra_rpy)
            t.transform.rotation = q
            self.static_br.sendTransform(t)
            rospy.loginfo('Published static %s -> %s', self.child_frame, self.extra_frame)

        rospy.loginfo('odom_to_tf: listening to %s, broadcasting %s->%s',
                      self.odom_topic, self.frame_id, self.child_frame)

        rospy.spin()

    def odom_cb(self, msg: Odometry):
        t = TransformStamped()
        # prefer message timestamp if available and non-zero
        if msg.header.stamp and msg.header.stamp.secs != 0:
            stamp = msg.header.stamp
        else:
            stamp = rospy.Time.now()

        # Prevent duplicate timestamp warnings from TF tree
        if self._last_stamp and self._last_stamp == stamp:
            return
        self._last_stamp = stamp

        t.header.stamp = stamp
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        # save last transform (copy) for periodic republish
        self._last_transform = t
        # broadcast immediately
        self.br.sendTransform(t)

    @staticmethod
    def _rpy_to_quat(roll, pitch, yaw):
        # returns geometry_msgs/Quaternion
        q = Quaternion()
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q


if __name__ == '__main__':
    try:
        OdomToTF()
    except rospy.ROSInterruptException:
        pass

