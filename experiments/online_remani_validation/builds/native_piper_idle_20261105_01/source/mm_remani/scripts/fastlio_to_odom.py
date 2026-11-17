#!/usr/bin/env python3
"""
FAST-LIO2 /Odometry (camera_init→body) → /odom (odom→base_link).

FAST-LIO2 body = IMU/LiDAR frame.  LiDAR is mounted 0.28 m forward and
0.115 m above the robot's base_link.  We subtract this offset so /odom
represents the base_link centre, matching the URDF and collision model.
"""
import rospy
import numpy as np
import tf.transformations as tft
from geometry_msgs.msg import TransformStamped, Quaternion, Point
from nav_msgs.msg import Odometry
import tf2_ros

class FastLIOToOdomBridge:
    def __init__(self):
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._last_stamp = rospy.Time(0)
        self.planar_base_link = rospy.get_param('~planar_base_link', True)
        self.base_z = rospy.get_param('~base_z', 0.115)
        self.lidar_offset = np.array([
            rospy.get_param('~lidar_offset_x', 0.28),
            rospy.get_param('~lidar_offset_y', 0.0),
            rospy.get_param('~lidar_offset_z', 0.002),
        ])
        rospy.Subscriber('/Odometry', Odometry, self.cb, queue_size=1, tcp_nodelay=True)
        rospy.loginfo("[fastlio_to_odom] /Odometry → /odom (body→base_link corrected), planar=%s",
                      self.planar_base_link)

    def cb(self, msg):
        if msg.header.stamp == self._last_stamp:
            return
        self._last_stamp = msg.header.stamp

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        R = tft.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        roll, pitch, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])

        if self.planar_base_link:
            q_out = Quaternion(*tft.quaternion_from_euler(0.0, 0.0, yaw))
            R_offset = tft.euler_matrix(0.0, 0.0, yaw)[:3, :3]
            rospy.loginfo_throttle(
                1.0,
                "[fastlio_to_odom] flatten base_link r/p: raw roll=%.3f pitch=%.3f yaw=%.3f",
                roll, pitch, yaw)
        else:
            q_out = q
            R_offset = R

        # body ≈ LiDAR;  base_link = body - R * LIDAR_OFFSET
        body_pos = np.array([p.x, p.y, p.z])
        base_pos = body_pos - R_offset @ self.lidar_offset
        # Chassis z = 0.115 (matches original real_odom_publisher.py).
        # Collision model CarState2T() hardcodes z=0.
        # URDF wheels: joint z≈-0.03 + radius 0.08 → bottom at base_z-0.11.
        # base_z=0.115 puts wheel bottoms at 0.005 above ground.
        base_pos[2] = self.base_z

        # ---- /odom ----
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position = Point(x=base_pos[0], y=base_pos[1], z=base_pos[2])
        odom.pose.pose.orientation = q_out
        odom.twist = msg.twist
        self.odom_pub.publish(odom)

        # ---- TF odom → base_link ----
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = base_pos[0]
        t.transform.translation.y = base_pos[1]
        t.transform.translation.z = base_pos[2]
        t.transform.rotation = q_out
        self.tf_broadcaster.sendTransform(t)


if __name__ == '__main__':
    rospy.init_node('fastlio_to_odom')
    FastLIOToOdomBridge()
    rospy.spin()
