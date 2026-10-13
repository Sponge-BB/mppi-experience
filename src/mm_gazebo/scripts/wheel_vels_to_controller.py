#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64, Float64MultiArray

def callback(msg):
    # msg.data assumed [fl, fr, rl, rr] in mecanum_controller
    # but ensure ordering mapping to controllers: our YAML uses fr, fl, br, bl joints names.
    # Check mecanum_controller joint ordering: it used [fl, fr, rl, rr]
    fl, fr, rl, rr = msg.data
    # Map to our controller topics:
    pub_fl.publish(Float64(fl))
    pub_fr.publish(Float64(fr))
    # Map rear-left/rear-right properly to bl/br controllers
    # msg.data ordering is [fl, fr, rl, rr]
    pub_bl.publish(Float64(rl))
    pub_br.publish(Float64(rr))

if __name__ == '__main__':
    rospy.init_node('wheel_vels_to_controller')
    pub_fr = rospy.Publisher('/gazebo/fr_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_fl = rospy.Publisher('/gazebo/fl_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_br = rospy.Publisher('/gazebo/br_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_bl = rospy.Publisher('/gazebo/bl_wheel_velocity_controller/command', Float64, queue_size=1)
    rospy.Subscriber('/wheel_vels', Float64MultiArray, callback, queue_size=1)
    rospy.spin()