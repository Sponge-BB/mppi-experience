#!/usr/bin/python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import copy
import _thread

import numpy as np
import rospy
import tf
import tf.transformations
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry

cur_odom_to_baselink = None
cur_map_to_odom = None


def pose_to_mat(pose_msg):
    return np.matmul(
        tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
        tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),
    )


def transform_fusion():
    global cur_odom_to_baselink, cur_map_to_odom

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(FREQ_PUB_LOCALIZATION)
    while not rospy.is_shutdown():
        rate.sleep()

        cur_odom = copy.copy(cur_odom_to_baselink)
        if cur_map_to_odom is not None:
            T_map_to_odom = pose_to_mat(cur_map_to_odom)
            if MAX_CORRECTION_AGE > 0 and cur_map_to_odom.header.stamp != rospy.Time(0):
                correction_age = (rospy.Time.now() - cur_map_to_odom.header.stamp).to_sec()
                if correction_age > MAX_CORRECTION_AGE:
                    rospy.logwarn_throttle(
                        1.0,
                        'ICP correction is stale: %.3fs since last /map_to_odom',
                        correction_age)
        else:
            T_map_to_odom = np.eye(4)

        if PUBLISH_TF:
            br.sendTransform(tf.transformations.translation_from_matrix(T_map_to_odom),
                             tf.transformations.quaternion_from_matrix(T_map_to_odom),
                             rospy.Time.now(),
                             ODOM_FRAME, MAP_FRAME)
        if cur_odom is not None:
            # 发布全局定位的odometry
            localization = Odometry()
            T_odom_to_base_link = pose_to_mat(cur_odom)
            # 这里T_map_to_odom短时间内变化缓慢 暂时不考虑与T_odom_to_base_link时间同步
            T_map_to_base_link = np.matmul(T_map_to_odom, T_odom_to_base_link)
            xyz = tf.transformations.translation_from_matrix(T_map_to_base_link)
            quat = tf.transformations.quaternion_from_matrix(T_map_to_base_link)
            localization.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
            localization.twist = cur_odom.twist

            localization.header.stamp = cur_odom.header.stamp
            localization.header.frame_id = MAP_FRAME
            localization.child_frame_id = BASE_FRAME
            # rospy.loginfo_throttle(1, '{}'.format(np.matmul(T_map_to_odom, T_odom_to_base_link)))
            pub_localization.publish(localization)


def cb_save_cur_odom(odom_msg):
    global cur_odom_to_baselink
    cur_odom_to_baselink = odom_msg


def cb_save_map_to_odom(odom_msg):
    global cur_map_to_odom
    cur_map_to_odom = odom_msg


if __name__ == '__main__':
    rospy.init_node('transform_fusion')
    rospy.loginfo('Transform Fusion Node Inited...')

    # tf and localization publishing frequency (HZ)
    FREQ_PUB_LOCALIZATION = rospy.get_param('~publish_frequency', 50.0)
    MAP_FRAME = rospy.get_param('~map_frame', 'map')
    ODOM_FRAME = rospy.get_param('~odom_frame', 'camera_init')
    BASE_FRAME = rospy.get_param('~base_frame', 'body')
    MAX_CORRECTION_AGE = rospy.get_param('~max_correction_age', 1.5)
    PUBLISH_TF = rospy.get_param('~publish_tf', True)

    rospy.loginfo('Transform fusion: publish_tf=%s, frame %s -> %s, localization at %.1f Hz',
                  PUBLISH_TF, MAP_FRAME, ODOM_FRAME, FREQ_PUB_LOCALIZATION)

    rospy.Subscriber('/Odometry', Odometry, cb_save_cur_odom, queue_size=1)
    rospy.Subscriber('/map_to_odom', Odometry, cb_save_map_to_odom, queue_size=1)

    pub_localization = rospy.Publisher('/localization', Odometry, queue_size=1)

    # 发布定位消息
    _thread.start_new_thread(transform_fusion, ())

    rospy.spin()
