#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


class BaseTracker:
    def __init__(self):
        rospy.init_node('gazebo_base_tracker')

        self.cmd_pub  = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.err_pub  = rospy.Publisher('/tracking_error', Float64MultiArray, queue_size=10)
        self.cmd_sub  = rospy.Subscriber('/mm_controller_node/car_cmd', Twist, self.car_cmd_cb)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.has_odom = False

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_w  = 0.0
        self.has_target = False

        # P gains
        self.k_x = 9.0
        self.k_y = 10.0
        self.k_yaw = 6.0

        rospy.Timer(rospy.Duration(0.05), self.control_loop)
        rospy.loginfo("Gazebo base tracker started.")
        rospy.spin()

    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.has_odom = True

    def car_cmd_cb(self, msg):
        self.target_x = msg.linear.x
        self.target_y = msg.linear.y
        self.target_yaw = msg.linear.z
        self.target_vx = msg.angular.x
        self.target_vy = msg.angular.y
        self.target_w  = msg.angular.z
        self.has_target = True

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def control_loop(self, event):
        if not self.has_odom or not self.has_target:
            return

        # Target local feedforward velocity
        v_r = self.target_vx * math.cos(self.target_yaw) + self.target_vy * math.sin(self.target_yaw)

        # Errors in global frame
        ex_global = self.target_x - self.current_x
        ey_global = self.target_y - self.current_y

        # Errors in local frame
        ex = math.cos(self.current_yaw) * ex_global + math.sin(self.current_yaw) * ey_global
        ey = -math.sin(self.current_yaw) * ex_global + math.cos(self.current_yaw) * ey_global
        eyaw = self.normalize_angle(self.target_yaw - self.current_yaw)

        dist = math.hypot(ex_global, ey_global)

        # If we just reached the end and standard motion is done, stop
        if dist < 0.15 and abs(eyaw) < 0.1 and abs(v_r) < 0.05:
            err_msg = Float64MultiArray()
            err_msg.data = [dist, ex, ey, eyaw]
            self.err_pub.publish(err_msg)
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return

        cmd = Twist()

        # Smooth trajectory tracking control law for non-holonomic mobile robots
        cmd.linear.x = v_r * math.cos(eyaw) + self.k_x * ex
        cmd.angular.z = self.target_w + self.k_yaw * eyaw + self.k_y * v_r * ey

        # publish tracking error [dist, ex, ey, eyaw]
        err_msg = Float64MultiArray()
        err_msg.data = [dist, ex, ey, eyaw]
        self.err_pub.publish(err_msg)

        # limits
        if cmd.linear.x > 3.0: cmd.linear.x = 3.0
        if cmd.linear.x < -3.0: cmd.linear.x = -3.0
        if cmd.angular.z > 2.0: cmd.angular.z = 2.0
        if cmd.angular.z < -2.0: cmd.angular.z = -2.0

        self.cmd_pub.publish(cmd)


if __name__ == '__main__':
    try:
        BaseTracker()
    except rospy.ROSInterruptException:
        pass
