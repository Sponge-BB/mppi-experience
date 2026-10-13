#!/usr/bin/env python3
"""Convert /cmd_vel Twist to mecanum wheel velocities and publish /wheel_vels

Publishes std_msgs/Float64MultiArray with data [fl, fr, rl, rr]
which matches the expected ordering in wheel_vels_to_controller.py bridge.

Params (ROS parameters, optional):
    ~wheel_radius (m) default 0.075
  ~wheel_base_length (m) L distance from center to front/back (half length) default 0.19
  ~wheel_base_width (m)  W distance from center to left/right (half width) default 0.20
  ~rate (Hz) publish rate default 50
  ~max_wheel_speed (rad/s) optional clamp
"""
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
import math


class CmdVelToWheelVels(object):
    def __init__(self):
        rospy.init_node('cmd_vel_to_wheel_vels')

        self.wheel_radius = rospy.get_param('~wheel_radius', 0.075)
        # Use half-length/half-width or use (L+W) as combined distance; user can tune
        self.wheel_base_length = rospy.get_param('~wheel_base_length', 0.19)
        self.wheel_base_width = rospy.get_param('~wheel_base_width', 0.20)
        self.rate_hz = rospy.get_param('~rate', 50)
        self.max_wheel_speed = rospy.get_param('~max_wheel_speed', 1000.0)
        # optional closed-loop scaling to match actual robot vx to commanded vx
        self.use_feedback = rospy.get_param('~use_feedback', False)
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')
        # angular scaling to increase/decrease rotational responsiveness
        self.angular_scale = rospy.get_param('~angular_scale', 1.0)
        # smoothing for measured velocity and scale factor
        self._meas_vx = 0.0
        self._meas_alpha = rospy.get_param('~meas_alpha', 0.2)
        self._scale = 1.0
        self._scale_alpha = rospy.get_param('~scale_alpha', 0.1)
        # Optional PID closed-loop for vx tracking
        self.use_pid_feedback = rospy.get_param('~use_pid_feedback', False)
        self.pid_kp = rospy.get_param('~pid_kp', 0.5)
        self.pid_ki = rospy.get_param('~pid_ki', 0.1)
        self._int_err = 0.0
        self._last_time = rospy.Time.now()
        self.scale_min = rospy.get_param('~scale_min', 0.3)
        self.scale_max = rospy.get_param('~scale_max', 3.0)

        # combined distance from center to wheel used in kinematics
        self.l_plus_w = self.wheel_base_length + self.wheel_base_width

        self.last_twist = Twist()

        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_cb, queue_size=1)
        self.pub = rospy.Publisher('/wheel_vels', Float64MultiArray, queue_size=1)
        if self.use_feedback:
            rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(1.0 / float(self.rate_hz)), self.timer_cb)

        rospy.loginfo('cmd_vel_to_wheel_vels started: wheel_r=%.3f L=%.3f W=%.3f rate=%d',
                      self.wheel_radius, self.wheel_base_length, self.wheel_base_width, self.rate_hz)

    def cmd_vel_cb(self, msg):
        self.last_twist = msg

    def _odom_cb(self, msg):
        try:
            meas = msg.twist.twist.linear.x
        except Exception:
            return
        # exponential smoothing
        self._meas_vx = self._meas_alpha * meas + (1.0 - self._meas_alpha) * self._meas_vx


    def timer_cb(self, event):
        vx = self.last_twist.linear.x
        vy = self.last_twist.linear.y
        wz = self.last_twist.angular.z * self.angular_scale

        # Mecanum wheel kinematics (standard 4-wheel layout)
        # Compute wheel linear velocities (m/s) then convert to rad/s by dividing by wheel radius
        # Ordering: fl, fr, rl, rr
        fl_lin = vx - vy - (self.l_plus_w) * wz
        fr_lin = vx + vy + (self.l_plus_w) * wz
        rl_lin = vx + vy - (self.l_plus_w) * wz
        rr_lin = vx - vy + (self.l_plus_w) * wz

        # Convert to angular velocity (rad/s)
        fl = fl_lin / self.wheel_radius
        fr = fr_lin / self.wheel_radius
        rl = rl_lin / self.wheel_radius
        rr = rr_lin / self.wheel_radius

        # feedback scaling: either simple smoothing or PID
        if self.use_pid_feedback and self.use_feedback:
            # PID on linear velocity error -> adjust multiplicative scale
            now = event.current_real if hasattr(event, 'current_real') else rospy.Time.now()
            dt = (now - self._last_time).to_sec() if self._last_time is not None else (1.0 / float(self.rate_hz))
            if dt <= 0:
                dt = 1.0 / float(self.rate_hz)
            self._last_time = now
            err = vx - self._meas_vx
            self._int_err += err * dt
            delta = self.pid_kp * err + self.pid_ki * self._int_err
            # update multiplicative scale
            self._scale += delta
            # clamp
            self._scale = max(min(self._scale, self.scale_max), self.scale_min)
        elif self.use_feedback:
            if abs(self._meas_vx) > 1e-4:
                target_vx = vx
                inst_factor = target_vx / (self._meas_vx + 1e-8)
                inst_factor = max(min(inst_factor, self.scale_max), self.scale_min)
                self._scale = (1.0 - self._scale_alpha) * self._scale + self._scale_alpha * inst_factor
            else:
                self._scale = 1.0
        else:
            self._scale = 1.0

        # apply scale to wheel angular speeds
        fl *= self._scale
        fr *= self._scale
        rl *= self._scale
        rr *= self._scale

        # clamp
        fl = max(min(fl, self.max_wheel_speed), -self.max_wheel_speed)
        fr = max(min(fr, self.max_wheel_speed), -self.max_wheel_speed)
        rl = max(min(rl, self.max_wheel_speed), -self.max_wheel_speed)
        rr = max(min(rr, self.max_wheel_speed), -self.max_wheel_speed)

        msg = Float64MultiArray()
        msg.data = [fl, fr, rl, rr]
        self.pub.publish(msg)


if __name__ == '__main__':
    try:
        node = CmdVelToWheelVels()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

