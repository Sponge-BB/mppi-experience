#!/usr/bin/env python3
"""
Simple mecanum base controller for testing and visualization.
Subscribes to /cmd_vel (geometry_msgs/Twist) and publishes:
- /odom (nav_msgs/Odometry)
- TF odom -> base_link
- /wheel_vels (std_msgs/Float64MultiArray) with 4 wheel angular velocities [fl, fr, rl, rr]

This node integrates commanded velocities to update pose and wheel angles for visualization.
"""
import rospy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import tf
import tf2_ros
import math
import time


class MecanumController(object):
    def __init__(self):
        rospy.init_node('mecanum_controller', anonymous=False)

        # parameters
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.075)  # meters
        self.wheel_base = rospy.get_param('~wheel_base', 0.36)     # front-back distance (L)
        self.wheel_track = rospy.get_param('~wheel_track', 0.28)   # left-right distance (W)
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.rate_hz = rospy.get_param('~rate', 50)
        self.cmd_timeout = rospy.get_param('~cmd_timeout', 0.5)

        # state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # wheel angles (rad) for visualizing wheel rotation if needed
        self.wheel_pos = [0.0, 0.0, 0.0, 0.0]  # fl, fr, rl, rr

        # commanded velocities (in base frame)
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_omega = 0.0
        self.last_cmd_time = rospy.Time.now()

        # publishers/subscribers
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.wheel_pub = rospy.Publisher('/wheel_vels', Float64MultiArray, queue_size=5)
        self.publish_joint_states = rospy.get_param('~publish_joint_states', True)
        if self.publish_joint_states:
            self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=5)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_cb)

        # joint names for wheels (ensure match your URDF if you want wheel rotation visualized)
        self.joint_names = [rospy.get_param('~joint_fl', 'fl_wheel_joint'),
                            rospy.get_param('~joint_fr', 'fr_wheel_joint'),
                            rospy.get_param('~joint_rl', 'rl_wheel_joint'),
                            rospy.get_param('~joint_rr', 'rr_wheel_joint')]

        # odom message template
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.odom_frame
        self.odom_msg.child_frame_id = self.base_frame

        self.joint_msg = JointState()
        self.joint_msg.name = self.joint_names

        rospy.loginfo('Mecanum controller started (r=%s, L=%s, W=%s)', self.wheel_radius, self.wheel_base, self.wheel_track)

    def cmd_vel_cb(self, msg):
        self.cmd_vx = msg.linear.x
        self.cmd_vy = msg.linear.y
        self.cmd_omega = msg.angular.z
        self.last_cmd_time = rospy.Time.now()

    def body_to_wheel(self, vx, vy, wz):
        # L and W distances from center to wheels: use half distances if needed
        L = self.wheel_base / 2.0
        W = self.wheel_track / 2.0
        R = self.wheel_radius
        # Factor (L+W)
        k = L + W
        # wheel order: fl, fr, rl, rr
        w_fl = (1.0 / R) * (vx - vy - k * wz)
        w_fr = (1.0 / R) * (vx + vy + k * wz)
        w_rl = (1.0 / R) * (vx + vy - k * wz)
        w_rr = (1.0 / R) * (vx - vy + k * wz)
        return [w_fl, w_fr, w_rl, w_rr]

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        last_time = rospy.Time.now()
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - last_time).to_sec()
            if dt <= 0:
                dt = 1.0 / float(self.rate_hz)
            last_time = now

            # timeout cmd
            if (now - self.last_cmd_time).to_sec() > self.cmd_timeout:
                vx = vy = wz = 0.0
            else:
                vx = self.cmd_vx
                vy = self.cmd_vy
                wz = self.cmd_omega

            # integrate robot pose in odom frame
            # velocities are in robot frame; rotate to world frame
            dx = (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
            dy = (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
            dtheta = wz * dt

            self.x += dx
            self.y += dy
            self.yaw += dtheta

            # publish odom
            self.odom_msg.header.stamp = now
            self.odom_msg.pose.pose.position.x = self.x
            self.odom_msg.pose.pose.position.y = self.y
            self.odom_msg.pose.pose.position.z = 0.0
            q = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
            self.odom_msg.pose.pose.orientation.x = q[0]
            self.odom_msg.pose.pose.orientation.y = q[1]
            self.odom_msg.pose.pose.orientation.z = q[2]
            self.odom_msg.pose.pose.orientation.w = q[3]

            self.odom_msg.twist.twist.linear.x = vx
            self.odom_msg.twist.twist.linear.y = vy
            self.odom_msg.twist.twist.angular.z = wz

            self.odom_pub.publish(self.odom_msg)

            # broadcast tf odom -> base_link
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            try:
                self.tf_broadcaster.sendTransform(t)
            except Exception:
                pass

            # compute wheel velocities and integrate wheel positions
            wheel_ws = self.body_to_wheel(vx, vy, wz)
            # integrate angles
            for i in range(4):
                self.wheel_pos[i] += wheel_ws[i] * dt

            # publish wheel velocities (for other controllers) and joint_states for visualization
            wm = Float64MultiArray()
            wm.data = wheel_ws
            self.wheel_pub.publish(wm)

            self.joint_msg.header.stamp = now
            # if URDF/wheel joint names match, these positions will drive visual rotation
            if self.publish_joint_states:
                self.joint_msg.position = list(self.wheel_pos)
                # velocities optional
                self.joint_msg.velocity = list(wheel_ws)
                self.joint_pub.publish(self.joint_msg)

            rate.sleep()


if __name__ == '__main__':
    try:
        node = MecanumController()
        node.spin()
    except rospy.ROSInterruptException:
        pass
