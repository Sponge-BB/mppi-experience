#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class CmdVelToWheels(object):
    def __init__(self):
        rospy.init_node('cmd_vel_to_model_twist', anonymous=False)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.08)
        self.wheel_separation = rospy.get_param('~wheel_separation', 0.411)

        self.pub_fl = rospy.Publisher('/gazebo/fl_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_fr = rospy.Publisher('/gazebo/fr_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_bl = rospy.Publisher('/gazebo/bl_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_br = rospy.Publisher('/gazebo/br_wheel_velocity_controller/command', Float64, queue_size=1)

        rospy.Subscriber('/cmd_vel', Twist, self.cb_cmd_vel, queue_size=1)
        rospy.loginfo("cmd_vel -> wheel velocity controllers (r=%.3f, track=%.3f)", self.wheel_radius, self.wheel_separation)
        rospy.spin()

    def cb_cmd_vel(self, msg):
        v = msg.linear.x
        wz = msg.angular.z
        
        half_track = self.wheel_separation * 0.5
        v_left = v - wz * half_track
        v_right = v + wz * half_track
        
        omega_left = v_left / self.wheel_radius
        omega_right = v_right / self.wheel_radius

        self.pub_fl.publish(omega_left)
        self.pub_bl.publish(omega_left)
        self.pub_fr.publish(omega_right)
        self.pub_br.publish(omega_right)

if __name__ == '__main__':
    try:
        CmdVelToWheels()
    except rospy.ROSInterruptException:
        pass
