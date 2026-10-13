#!/usr/bin/env python3
"""Simple open-loop odometry publisher.

Subscribes to /cmd_vel (geometry_msgs/Twist) and integrates to produce
nav_msgs/Odometry on /odom. This is a temporary helper to supply odom
when a proper controller/motion source is not publishing it.

Parameters (ROS params, optional):
  ~odom_topic (string) default: /odom
  ~frame_id (string) default: odom
  ~child_frame (string) default: base_link
  ~publish_rate (float) Hz default: 50.0
"""

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf_conversions
import math


class OdomOpenLoop(object):
    def __init__(self):
        rospy.init_node('odom_openloop')
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')
        self.frame_id = rospy.get_param('~frame_id', 'odom')
        self.child_frame = rospy.get_param('~child_frame', 'base_link')
        self.rate = rospy.get_param('~publish_rate', 50.0)

        # internal pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # last received cmd and time
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self._last_time = rospy.Time.now()

        rospy.Subscriber('/cmd_vel', Twist, self.cmd_cb, queue_size=1)
        self.pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=1)

        rospy.loginfo('odom_openloop: publishing %s at %.1f Hz', self.odom_topic, self.rate)
        self.loop()

    def cmd_cb(self, msg: Twist):
        # store commanded velocities (in robot frame)
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.wz = msg.angular.z

    def loop(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - self._last_time).to_sec()
            if dt <= 0.0:
                dt = 1.0 / self.rate
            self._last_time = now

            # integrate pose in robot frame -> world frame
            # approximate small-angle integration
            if abs(self.wz) < 1e-6:
                dx = (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * dt
                dy = (self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)) * dt
            else:
                # consider rotation during the interval
                dx = (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * dt
                dy = (self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)) * dt

            # update pose
            self.x += dx
            self.y += dy
            self.yaw += self.wz * dt

            # normalize yaw
            self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

            # publish odometry
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.frame_id
            odom.child_frame_id = self.child_frame
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.yaw)
            odom.pose.pose.orientation = Quaternion(*q)

            odom.twist.twist.linear.x = self.vx
            odom.twist.twist.linear.y = self.vy
            odom.twist.twist.angular.z = self.wz

            self.pub.publish(odom)
            rate.sleep()


if __name__ == '__main__':
    try:
        OdomOpenLoop()
    except rospy.ROSInterruptException:
        pass
