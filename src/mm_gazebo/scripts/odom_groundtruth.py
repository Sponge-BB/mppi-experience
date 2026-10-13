#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class OdomGT:
    def __init__(self):
        rospy.init_node('odom_groundtruth')
        self.pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.model_name = rospy.get_param('~model_name', 'mm_robot')
        self.odom = Odometry()
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'base_link'
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.cb, queue_size=1)
        rospy.spin()
        
    def cb(self, msg):
        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            return
            
        self.odom.header.stamp = rospy.Time.now()
        self.odom.pose.pose = msg.pose[idx]
        self.odom.twist.twist = msg.twist[idx]
        self.pub.publish(self.odom)

if __name__ == '__main__':
    OdomGT()
