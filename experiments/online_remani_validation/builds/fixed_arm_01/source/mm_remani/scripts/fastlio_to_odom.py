#!/usr/bin/env python3
"""
FAST-LIO2 /Odometry (camera_init→body) → /odom (odom→base_link).

FAST-LIO2 body = IMU/LiDAR frame.  LiDAR is mounted 0.28 m forward and
0.115 m above the robot's base_link.  We subtract this offset so /odom
represents the base_link centre, matching the URDF and collision model.
"""
import math
import rospy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros

class FastLIOToOdomBridge:
    def __init__(self):
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._last_stamp = rospy.Time(0)
        self._last_base_pose = None
        self._filtered_twist = [0.0, 0.0, 0.0]
        self.planar_base_link = rospy.get_param('~planar_base_link', True)
        self.base_z = rospy.get_param('~base_z', 0.115)
        self.lidar_offset = (
            rospy.get_param('~lidar_offset_x', 0.28),
            rospy.get_param('~lidar_offset_y', 0.0),
            rospy.get_param('~lidar_offset_z', 0.002),
        )
        self.twist_filter_alpha = min(
            1.0, max(0.01, rospy.get_param('~twist_filter_alpha', 0.35)))
        self.max_derived_linear_speed = max(
            0.1, rospy.get_param('~max_derived_linear_speed', 1.0))
        self.max_derived_angular_speed = max(
            0.1, rospy.get_param('~max_derived_angular_speed', 3.0))
        rospy.Subscriber('/Odometry', Odometry, self.cb, queue_size=1, tcp_nodelay=True)
        rospy.loginfo("[fastlio_to_odom] /Odometry → /odom (body→base_link corrected), planar=%s",
                      self.planar_base_link)

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def yaw_from_quaternion(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def rotate_vector(q, vector):
        # Quaternion-vector rotation without allocating NumPy matrices.
        vx, vy, vz = vector
        tx = 2.0 * (q.y * vz - q.z * vy)
        ty = 2.0 * (q.z * vx - q.x * vz)
        tz = 2.0 * (q.x * vy - q.y * vx)
        return (
            vx + q.w * tx + q.y * tz - q.z * ty,
            vy + q.w * ty + q.z * tx - q.x * tz,
            vz + q.w * tz + q.x * ty - q.y * tx,
        )

    @staticmethod
    def clamp(value, limit):
        return max(-limit, min(limit, value))

    def derive_twist(self, stamp, base_x, base_y, yaw):
        if self._last_base_pose is None:
            self._last_base_pose = (stamp, base_x, base_y, yaw)
            return self._filtered_twist

        last_stamp, last_x, last_y, last_yaw = self._last_base_pose
        self._last_base_pose = (stamp, base_x, base_y, yaw)
        dt = (stamp - last_stamp).to_sec()
        if dt <= 1e-3 or dt > 0.5:
            return self._filtered_twist

        world_vx = (base_x - last_x) / dt
        world_vy = (base_y - last_y) / dt
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        body_vx = cos_yaw * world_vx + sin_yaw * world_vy
        body_vy = -sin_yaw * world_vx + cos_yaw * world_vy
        yaw_rate = self.normalize_angle(yaw - last_yaw) / dt

        body_vx = self.clamp(body_vx, self.max_derived_linear_speed)
        body_vy = self.clamp(body_vy, self.max_derived_linear_speed)
        yaw_rate = self.clamp(yaw_rate, self.max_derived_angular_speed)
        alpha = self.twist_filter_alpha
        measured = (body_vx, body_vy, yaw_rate)
        self._filtered_twist = [
            (1.0 - alpha) * old + alpha * new
            for old, new in zip(self._filtered_twist, measured)
        ]
        return self._filtered_twist

    def cb(self, msg):
        if msg.header.stamp == self._last_stamp:
            return
        self._last_stamp = msg.header.stamp

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = self.yaw_from_quaternion(q)

        if self.planar_base_link:
            half_yaw = 0.5 * yaw
            q_out = (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            ox, oy, oz = self.lidar_offset
            rotated_offset = (
                cos_yaw * ox - sin_yaw * oy,
                sin_yaw * ox + cos_yaw * oy,
                oz,
            )
        else:
            q_out = (q.x, q.y, q.z, q.w)
            rotated_offset = self.rotate_vector(q, self.lidar_offset)

        # body ≈ LiDAR;  base_link = body - R * LIDAR_OFFSET
        base_x = p.x - rotated_offset[0]
        base_y = p.y - rotated_offset[1]
        # Chassis z = 0.115 (matches original real_odom_publisher.py).
        # Collision model CarState2T() hardcodes z=0.
        # URDF wheels: joint z≈-0.03 + radius 0.08 → bottom at base_z-0.11.
        # base_z=0.115 puts wheel bottoms at 0.005 above ground.
        base_z = self.base_z
        body_vx, body_vy, yaw_rate = self.derive_twist(
            msg.header.stamp, base_x, base_y, yaw)

        # ---- /odom ----
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose = msg.pose
        odom.pose.pose.position.x = base_x
        odom.pose.pose.position.y = base_y
        odom.pose.pose.position.z = base_z
        odom.pose.pose.orientation.x = q_out[0]
        odom.pose.pose.orientation.y = q_out[1]
        odom.pose.pose.orientation.z = q_out[2]
        odom.pose.pose.orientation.w = q_out[3]
        odom.twist.covariance = msg.twist.covariance
        odom.twist.twist.linear.x = body_vx
        odom.twist.twist.linear.y = body_vy
        odom.twist.twist.angular.z = yaw_rate
        self.odom_pub.publish(odom)

        # ---- TF odom → base_link ----
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = base_x
        t.transform.translation.y = base_y
        t.transform.translation.z = base_z
        t.transform.rotation.x = q_out[0]
        t.transform.rotation.y = q_out[1]
        t.transform.rotation.z = q_out[2]
        t.transform.rotation.w = q_out[3]
        self.tf_broadcaster.sendTransform(t)


if __name__ == '__main__':
    rospy.init_node('fastlio_to_odom')
    FastLIOToOdomBridge()
    rospy.spin()
