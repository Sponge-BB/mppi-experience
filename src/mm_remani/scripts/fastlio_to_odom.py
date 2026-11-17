#!/usr/bin/env python3
"""
FAST-LIO2 /Odometry (camera_init→body) → /odom (odom→base_link).

FAST-LIO2 body is the IMU frame. The real-robot entry point uses FAST-LIO's
LiDAR-to-IMU calibration for the perception TF, while base_link stays planar.
Other entry points retain the previous body-as-LiDAR convention by default.
"""
import math
import rospy
from geometry_msgs.msg import Quaternion, TransformStamped
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
        self.publish_lidar_tf = rospy.get_param('~publish_lidar_tf', False)
        self.lidar_extrinsic_translation = (0.0, 0.0, 0.0)
        self.lidar_extrinsic_rotation = Quaternion(0.0, 0.0, 0.0, 1.0)
        if self.publish_lidar_tf:
            lidar_frame = str(rospy.get_param('~lidar_frame', 'livox_frame')).strip()
            if (not lidar_frame or lidar_frame.startswith('/') or
                    lidar_frame in ('body', 'base_link', 'odom', 'camera_init',
                                    'map', 'world')):
                raise ValueError('lidar_frame must be a distinct TF child frame')
            translation, rotation = self.validate_lidar_extrinsic(
                rospy.get_param('/mapping/extrinsic_T'),
                rospy.get_param('/mapping/extrinsic_R'))
            self.lidar_extrinsic_translation = translation
            self.lidar_extrinsic_rotation = rotation
            self.static_lidar_broadcaster = tf2_ros.StaticTransformBroadcaster()
            lidar_tf = TransformStamped()
            lidar_tf.header.stamp = rospy.Time.now()
            lidar_tf.header.frame_id = 'body'
            lidar_tf.child_frame_id = lidar_frame
            (lidar_tf.transform.translation.x,
             lidar_tf.transform.translation.y,
             lidar_tf.transform.translation.z) = translation
            lidar_tf.transform.rotation = rotation
            self.static_lidar_broadcaster.sendTransform(lidar_tf)
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
    def validate_lidar_extrinsic(translation, rotation):
        """Read the same p_IMU = R_LI * p_lidar + T_LI used by FAST-LIO."""
        if len(translation) != 3 or len(rotation) != 9:
            raise ValueError('FAST-LIO extrinsic_T/R must contain 3/9 values')
        t = tuple(float(value) for value in translation)
        r = tuple(float(value) for value in rotation)
        if not all(math.isfinite(value) for value in t + r):
            raise ValueError('FAST-LIO extrinsic_T/R must be finite')
        for row in range(3):
            for other in range(3):
                dot = sum(r[3 * row + k] * r[3 * other + k] for k in range(3))
                if abs(dot - (1.0 if row == other else 0.0)) > 1e-5:
                    raise ValueError('FAST-LIO extrinsic_R must be orthonormal')
        determinant = (r[0] * (r[4] * r[8] - r[5] * r[7]) -
                       r[1] * (r[3] * r[8] - r[5] * r[6]) +
                       r[2] * (r[3] * r[7] - r[4] * r[6]))
        if abs(determinant - 1.0) > 1e-5:
            raise ValueError('FAST-LIO extrinsic_R must be a right-handed rotation')

        # Matrix-to-quaternion conversion without importing NumPy into this
        # small bridge. All branches include valid rotations near 180 degrees.
        trace = r[0] + r[4] + r[8]
        if trace > 0.0:
            scale = 2.0 * math.sqrt(trace + 1.0)
            q = Quaternion((r[7] - r[5]) / scale,
                           (r[2] - r[6]) / scale,
                           (r[3] - r[1]) / scale, 0.25 * scale)
        else:
            axis = max(range(3), key=lambda index: r[4 * index])
            other, last = (axis + 1) % 3, (axis + 2) % 3
            scale = 2.0 * math.sqrt(
                1.0 + r[4 * axis] - r[4 * other] - r[4 * last])
            xyz = [0.0, 0.0, 0.0]
            xyz[axis] = 0.25 * scale
            xyz[other] = (r[3 * axis + other] + r[3 * other + axis]) / scale
            xyz[last] = (r[3 * axis + last] + r[3 * last + axis]) / scale
            q = Quaternion(*xyz, (r[3 * last + other] -
                                  r[3 * other + last]) / scale)
        return t, q

    @staticmethod
    def compose_rotation(first, second):
        return Quaternion(
            first.w * second.x + first.x * second.w + first.y * second.z - first.z * second.y,
            first.w * second.y - first.x * second.z + first.y * second.w + first.z * second.x,
            first.w * second.z + first.x * second.y - first.y * second.x + first.z * second.w,
            first.w * second.w - first.x * second.x - first.y * second.y - first.z * second.z)

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
        if self.publish_lidar_tf and msg.child_frame_id != 'body':
            rospy.logerr_throttle(2.0, '[fastlio_to_odom] Expected FAST-LIO body odometry')
            return
        if msg.header.stamp == self._last_stamp:
            return
        self._last_stamp = msg.header.stamp

        p = msg.pose.pose.position
        body_q = msg.pose.pose.orientation
        body_to_lidar = self.rotate_vector(body_q, self.lidar_extrinsic_translation)
        q = self.compose_rotation(body_q, self.lidar_extrinsic_rotation)
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

        # FAST-LIO estimates the IMU pose. First locate the actual LiDAR with
        # full 3D attitude, then subtract the configured base-to-LiDAR offset
        # in the planar heading used by the chassis controller.
        base_x = p.x + body_to_lidar[0] - rotated_offset[0]
        base_y = p.y + body_to_lidar[1] - rotated_offset[1]
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
        odom.pose.covariance = list(msg.pose.covariance)
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
