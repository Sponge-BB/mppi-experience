#!/usr/bin/python3
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf2_sensor_msgs.tf2_sensor_msgs

class CloudTransformer:
    def __init__(self):
        rospy.init_node('velodyne_transformer')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.target_frame = rospy.get_param('~target_frame', 'world')
        self.pub = rospy.Publisher('~output', PointCloud2, queue_size=10)
        self.sub = rospy.Subscriber('~input', PointCloud2, self.cloud_cb, queue_size=10)
        rospy.loginfo("Transformer node started, target_frame: " + self.target_frame)

    def cloud_cb(self, msg):
        try:
            trans = self.tf_buffer.lookup_transform(self.target_frame, msg.header.frame_id,
                                                    msg.header.stamp, rospy.Duration(0.1))
            cloud_out = tf2_sensor_msgs.do_transform_cloud(msg, trans)
            self.pub.publish(cloud_out)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logdebug("Transform lookup failed: %s" % e)

if __name__ == '__main__':
    CloudTransformer()
    rospy.spin()
