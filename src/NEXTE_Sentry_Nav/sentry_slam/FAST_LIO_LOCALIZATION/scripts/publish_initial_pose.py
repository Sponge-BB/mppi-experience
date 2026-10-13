#!/usr/bin/python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import argparse

import rospy
import tf.transformations
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('x', type=float)
    parser.add_argument('y', type=float)
    parser.add_argument('z', type=float)
    parser.add_argument('yaw', type=float)
    parser.add_argument('pitch', type=float)
    parser.add_argument('roll', type=float)
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('publish_initial_pose')
    topic = rospy.get_param('~topic', '/initialpose')
    pub_pose = rospy.Publisher(topic, PoseWithCovarianceStamped, queue_size=1)

    quat = tf.transformations.quaternion_from_euler(args.roll, args.pitch, args.yaw)
    xyz = [args.x, args.y, args.z]

    initial_pose = PoseWithCovarianceStamped()
    initial_pose.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
    initial_pose.header.frame_id = 'map'

    rospy.loginfo('Initial Pose on %s: x=%.2f y=%.2f z=%.2f yaw=%.2f pitch=%.2f roll=%.2f',
                  topic, args.x, args.y, args.z, args.yaw, args.pitch, args.roll)

    # Publish at 1Hz until ICP initializes.  Wait until a subscriber has
    # connected at least once so this node does not exit before localization
    # finishes loading the PCD map and starts waiting on the initial pose topic.
    rate = rospy.Rate(1)
    saw_subscriber = False
    while not rospy.is_shutdown():
        initial_pose.header.stamp = rospy.Time.now()
        pub_pose.publish(initial_pose)
        connections = pub_pose.get_num_connections()
        if connections > 0:
            saw_subscriber = True
        # Stop once global_localization has consumed the pose and unsubscribed.
        if saw_subscriber and connections == 0:
            rospy.loginfo('ICP initialized — stopping initial pose publisher.')
            break
        rate.sleep()
