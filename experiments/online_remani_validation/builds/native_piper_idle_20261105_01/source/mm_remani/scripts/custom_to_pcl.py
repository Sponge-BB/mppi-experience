#!/usr/bin/env python3
"""Convert livox_ros_driver2/CustomMsg → sensor_msgs/PointCloud2.

The MID360 publishes CustomMsg on /livox/lidar (for FAST-LIO2),
but the nodelet filter pipeline and REMANI planner need PointCloud2.
This node bridges the two.
"""
import rospy
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import struct


def cb(msg):
    points = []
    for p in msg.points:
        # pack intensity into the 'rgb' field so RViz can color by it
        b = struct.pack('B', int(p.reflectivity))
        rgb = struct.unpack('f', b + b'\x00\x00\x00')[0]
        points.append([p.x, p.y, p.z, rgb])
    header = msg.header
    fields = [
        pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
        pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
        pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
        pc2.PointField('intensity', 12, pc2.PointField.FLOAT32, 1),
    ]
    out = pc2.create_cloud(header, fields, points)
    pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('custom_to_pcl')
    pub = rospy.Publisher('/livox/lidar_pcl2', PointCloud2, queue_size=10)
    rospy.Subscriber('/livox/lidar', CustomMsg, cb, queue_size=10)
    rospy.loginfo('custom_to_pcl: /livox/lidar (CustomMsg) → /livox/lidar_pcl2 (PointCloud2)')
    rospy.spin()
