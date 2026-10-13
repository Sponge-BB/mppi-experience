#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, Float64MultiArray

class RealBaseTracker:
    def __init__(self):
        rospy.init_node('real_base_tracker', anonymous=True)

        # 1. 发布给实车底盘的速度控制指令
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.err_pub = rospy.Publisher('/tracking_error', Float64MultiArray, queue_size=10)
        self.fault_pub = rospy.Publisher('/tracking_fault', Bool, queue_size=1, latch=True)
        self.safety_speed_limit = None

        # 2. 接收由 REMANI 规划器发出的期望轨迹点（包裹在 Twist 里）
        #    linear.x = 目标X坐标
        #    linear.y = 目标Y坐标
        #    linear.z = 目标Yaw偏航角
        #    angular.x = 目标线速度X
        #    angular.y = 目标线速度Y
        #    angular.z = 期望角速度 (omega feedforward)
        self.cmd_sub = rospy.Subscriber('/mm_controller_node/car_cmd', Twist, self.car_cmd_cb)

        # 3. 接收刚才写的实车的实时定位Odometry
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.safety_speed_sub = rospy.Subscriber(
            '/planning/safety_speed_limit', Float64, self.safety_speed_limit_cb)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.has_odom = False
        self.odom_receive_stamp = rospy.Time(0)
        self.odom_header_stamp = rospy.Time(0)

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_w  = 0.0
        self.has_target = False

        # =======================================================
        # 实车轨迹跟踪 P 参数
        # =======================================================
        self.k_x = 1.5     # 前进方向误差纠正（降低减少震荡）
        self.k_y = 1.0     # 侧向偏离纠正（降低减少过调）
        self.k_yaw = 1.5   # 转角纠正（降低减少剧烈拐弯）

        # 最大限幅将从统配文件 (`real_speed.yaml`) 中的参数服务器拉取
        self.max_v = rospy.get_param('~tracker_max_v', 0.6)   # 最大线速度
        self.max_w = rospy.get_param('~tracker_max_w', 1.5)   # 最大角速度
        self.max_linear_accel = max(0.01, rospy.get_param('~tracker_max_linear_accel', 0.35))
        self.max_angular_accel = max(0.01, rospy.get_param('~tracker_max_angular_accel', 1.0))
        self.control_rate = max(10.0, rospy.get_param('~control_rate', 50.0))
        self.command_timeout = max(0.05, rospy.get_param('~command_timeout', 0.3))
        self.odom_timeout = max(0.05, rospy.get_param('~odom_timeout', 0.25))
        self.odom_header_timeout = max(
            self.odom_timeout, rospy.get_param('~odom_header_timeout', 0.25))
        self.tracking_stop_distance = max(
            0.05, rospy.get_param('~tracking_stop_distance', 0.18))
        self.tracking_stop_yaw = max(
            0.1, rospy.get_param('~tracking_stop_yaw', 0.50))
        # Ordinary reference-error crossings must persist for several 50 Hz
        # cycles.  Large errors remain an immediate hard stop, so debounce
        # removes millimetre-level trajectory handoff spikes without weakening
        # the safety envelope for a genuinely lost base.
        self.tracking_fault_confirm_frames = max(
            1, int(rospy.get_param('~tracking_fault_confirm_frames', 5)))
        self.tracking_hard_stop_distance = max(
            self.tracking_stop_distance,
            rospy.get_param('~tracking_hard_stop_distance', 0.35))
        self.tracking_hard_stop_yaw = max(
            self.tracking_stop_yaw,
            rospy.get_param('~tracking_hard_stop_yaw', 1.20))
        self.tracking_recover_distance = min(
            self.tracking_stop_distance,
            max(0.02, rospy.get_param('~tracking_recover_distance', 0.08)))
        self.tracking_recover_yaw = min(
            self.tracking_stop_yaw,
            max(0.05, rospy.get_param('~tracking_recover_yaw', 0.25)))
        self.tracking_recover_frames = max(
            1, int(rospy.get_param('~tracking_recover_frames', 5)))
        self.goal_position_tolerance = max(0.01, rospy.get_param('~goal_position_tolerance', 0.05))
        self.goal_yaw_tolerance = max(0.01, rospy.get_param('~goal_yaw_tolerance', 0.1))
        self.goal_speed_tolerance = max(0.0, rospy.get_param('~goal_speed_tolerance', 0.05))
        self.final_alignment_position_tolerance = max(
            self.goal_position_tolerance,
            rospy.get_param('~final_alignment_position_tolerance', 0.10))
        self.final_alignment_max_angular_speed = max(
            0.05, rospy.get_param('~final_alignment_max_angular_speed', 0.25))
        self.final_alignment_k_yaw = max(
            0.1, rospy.get_param('~final_alignment_k_yaw', 1.0))
        # A non-holonomic trajectory can legitimately change its tangent near
        # zero speed (start, cusp, reverse handoff, terminal alignment). Treat
        # that state as a bounded in-place turn when position is already
        # aligned, instead of latching a yaw tracking fault.
        self.turn_alignment_position_tolerance = min(
            self.tracking_stop_distance,
            max(self.goal_position_tolerance,
                rospy.get_param('~turn_alignment_position_tolerance', 0.10)))
        self.turn_alignment_max_reference_speed = max(
            self.goal_speed_tolerance,
            rospy.get_param('~turn_alignment_max_reference_speed', 0.08))
        self.turn_alignment_exit_yaw = min(
            self.goal_yaw_tolerance,
            max(0.01, rospy.get_param('~turn_alignment_exit_yaw', 0.07)))
        self.turn_alignment_min_angular_speed = min(
            self.final_alignment_max_angular_speed,
            max(0.0, rospy.get_param(
                '~turn_alignment_min_angular_speed', 0.12)))
        self.turn_alignment_active = False
        self.target_stamp = rospy.Time(0)
        self.last_cmd = Twist()
        self.last_control_stamp = rospy.Time(0)
        self.tracking_fault = False
        self.tracking_unsafe_count = 0
        self.tracking_recover_count = 0
        self.fault_pub.publish(Bool(data=False))
        rospy.on_shutdown(self.publish_stop)

        # Match the 50 Hz UART sender so acceleration limiting is applied in
        # smaller increments instead of visible 20 Hz angular-speed steps.
        rospy.Timer(rospy.Duration(1.0 / self.control_rate), self.control_loop)
        rospy.loginfo("实车轨迹追踪器(Tracker)已启动，正在将 car_cmd 转换为实车 cmd_vel...")
        rospy.spin()

    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.has_odom = True
        self.odom_receive_stamp = rospy.Time.now()
        self.odom_header_stamp = msg.header.stamp

    def car_cmd_cb(self, msg):
        self.target_x = msg.linear.x
        self.target_y = msg.linear.y
        self.target_yaw = msg.linear.z
        self.target_vx = msg.angular.x
        self.target_vy = msg.angular.y
        self.target_w  = msg.angular.z
        self.has_target = True
        self.target_stamp = rospy.Time.now()

    def safety_speed_limit_cb(self, msg):
        if not math.isfinite(msg.data) or msg.data < 0.0:
            self.safety_speed_limit = None
            return
        self.safety_speed_limit = max(0.0, msg.data)

    def publish_stop(self):
        self.last_cmd = Twist()
        self.last_control_stamp = rospy.Time.now()
        self.cmd_pub.publish(self.last_cmd)

    def set_tracking_fault(self, active, reason=""):
        if active == self.tracking_fault:
            return
        self.tracking_fault = active
        self.tracking_unsafe_count = 0
        self.tracking_recover_count = 0
        self.fault_pub.publish(Bool(data=active))
        if active:
            rospy.logerr("Tracking safety stop: %s", reason)
        else:
            rospy.loginfo("Tracking recovered on a trajectory aligned with current odometry")

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def limit_rate(target, previous, max_rate, dt):
        delta = max_rate * max(0.0, dt)
        return max(previous - delta, min(previous + delta, target))

    @staticmethod
    def is_turn_alignment(dist, reference_speed, yaw_error,
                          position_tolerance, max_reference_speed,
                          yaw_tolerance):
        """Return true only for a low-speed yaw change at an aligned pose."""
        return (dist <= position_tolerance and
                abs(reference_speed) <= max_reference_speed and
                abs(yaw_error) >= yaw_tolerance)

    @staticmethod
    def tracking_error_is_unsafe(dist, yaw_error, turn_alignment,
                                 stop_distance, stop_yaw):
        """Keep yaw protection while translating; never mask position loss."""
        return (dist > stop_distance or
                (abs(yaw_error) > stop_yaw and not turn_alignment))

    @staticmethod
    def update_tracking_unsafe_count(previous_count, unsafe):
        """Count consecutive unsafe control cycles; any safe cycle clears."""
        return previous_count + 1 if unsafe else 0

    @staticmethod
    def tracking_fault_should_latch(soft_unsafe, hard_unsafe,
                                    unsafe_count, confirm_frames):
        """Hard errors stop now; threshold-edge errors require persistence."""
        return (hard_unsafe or
                (soft_unsafe and unsafe_count >= max(1, confirm_frames)))

    @staticmethod
    def update_turn_alignment_state(active, dist, reference_speed, yaw_error,
                                    position_tolerance,
                                    max_reference_speed,
                                    enter_yaw_tolerance,
                                    exit_yaw_tolerance,
                                    stop_distance):
        """Latch alignment until yaw converges or position safety is lost."""
        if active:
            return (dist <= stop_distance and
                    abs(yaw_error) > exit_yaw_tolerance)
        return RealBaseTracker.is_turn_alignment(
            dist, reference_speed, yaw_error,
            position_tolerance, max_reference_speed,
            enter_yaw_tolerance)

    @staticmethod
    def bounded_alignment_angular_command(yaw_error, gain,
                                          min_speed, max_speed):
        """Generate a useful but bounded turn command outside the deadband."""
        command = gain * yaw_error
        if abs(command) < 1e-9:
            return 0.0
        command = math.copysign(max(abs(command), min_speed), command)
        return max(-max_speed, min(max_speed, command))

    def control_loop(self, event):
        if not self.has_odom or not self.has_target:
            self.publish_stop()
            return

        now = rospy.Time.now()
        odom_receive_age = (now - self.odom_receive_stamp).to_sec()
        odom_header_age = (
            (now - self.odom_header_stamp).to_sec()
            if not self.odom_header_stamp.is_zero() else 0.0)
        if (odom_receive_age > self.odom_timeout or
                odom_header_age > self.odom_header_timeout):
            self.set_tracking_fault(
                True,
                "odom stale (receive %.3fs, header %.3fs)" %
                (odom_receive_age, odom_header_age))
            self.publish_stop()
            return

        if (now - self.target_stamp).to_sec() > self.command_timeout:
            rospy.logwarn_throttle(1.0, "Trajectory command timed out; stopping base")
            self.publish_stop()
            return

        if self.last_control_stamp.is_zero():
            dt = 0.05
        else:
            dt = min(0.2, max(0.0, (now - self.last_control_stamp).to_sec()))
        self.last_control_stamp = now

        # Target local feedforward velocity
        v_r = self.target_vx * math.cos(self.target_yaw) + self.target_vy * math.sin(self.target_yaw)

        # 全局坐标下的误差计算
        ex_global = self.target_x - self.current_x
        ey_global = self.target_y - self.current_y

        # 转换到车体局部坐标系下的前后、左右误差
        ex = math.cos(self.current_yaw) * ex_global + math.sin(self.current_yaw) * ey_global
        ey = -math.sin(self.current_yaw) * ex_global + math.cos(self.current_yaw) * ey_global
        eyaw = self.normalize_angle(self.target_yaw - self.current_yaw)

        dist = math.hypot(ex_global, ey_global)
        final_alignment = (
            dist <= self.final_alignment_position_tolerance and
            abs(v_r) < self.goal_speed_tolerance and
            abs(eyaw) >= self.goal_yaw_tolerance)
        alignment_was_active = self.turn_alignment_active
        alignment_entry = final_alignment or self.is_turn_alignment(
            dist, v_r, eyaw,
            self.turn_alignment_position_tolerance,
            self.turn_alignment_max_reference_speed,
            self.goal_yaw_tolerance)
        self.turn_alignment_active = self.update_turn_alignment_state(
            alignment_was_active, dist, v_r, eyaw,
            self.turn_alignment_position_tolerance,
            self.turn_alignment_max_reference_speed,
            self.goal_yaw_tolerance,
            self.turn_alignment_exit_yaw,
            self.tracking_stop_distance)
        turn_alignment = alignment_entry or self.turn_alignment_active
        if turn_alignment != alignment_was_active:
            rospy.loginfo(
                "Yaw alignment %s: error %.3f rad, reference speed %.3f m/s",
                "entered" if turn_alignment else "completed", eyaw, v_r)
        err_msg = Float64MultiArray()
        err_msg.data = [dist, ex, ey, eyaw]
        self.err_pub.publish(err_msg)

        # A severely displaced robot must not cut across the optimized path to
        # chase a future reference point. The planner receives this latched
        # fault and generates a replacement trajectory from measured odometry.
        if self.tracking_fault:
            if (dist <= self.tracking_recover_distance and
                    abs(eyaw) <= self.tracking_recover_yaw):
                self.tracking_recover_count += 1
                if self.tracking_recover_count >= self.tracking_recover_frames:
                    self.set_tracking_fault(False)
            else:
                self.tracking_recover_count = 0
            if self.tracking_fault:
                self.publish_stop()
                return
        else:
            soft_unsafe = self.tracking_error_is_unsafe(
                dist, eyaw, turn_alignment,
                self.tracking_stop_distance, self.tracking_stop_yaw)
            hard_unsafe = self.tracking_error_is_unsafe(
                dist, eyaw, turn_alignment,
                self.tracking_hard_stop_distance,
                self.tracking_hard_stop_yaw)
            self.tracking_unsafe_count = self.update_tracking_unsafe_count(
                self.tracking_unsafe_count, soft_unsafe)
            if self.tracking_fault_should_latch(
                    soft_unsafe, hard_unsafe,
                    self.tracking_unsafe_count,
                    self.tracking_fault_confirm_frames):
                self.set_tracking_fault(
                    True,
                    ("reference error %.3fm / %.3frad exceeds soft "
                     "%.3fm / %.3frad for %d cycle(s); hard limits "
                     "%.3fm / %.3frad") %
                    (dist, abs(eyaw), self.tracking_stop_distance,
                     self.tracking_stop_yaw,
                     self.tracking_unsafe_count,
                     self.tracking_hard_stop_distance,
                     self.tracking_hard_stop_yaw))
                self.publish_stop()
                return
            if soft_unsafe:
                rospy.logwarn_throttle(
                    1.0,
                    "Transient tracking error %.3fm / %.3frad (%d/%d); "
                    "waiting for persistence before replanning",
                    dist, abs(eyaw), self.tracking_unsafe_count,
                    self.tracking_fault_confirm_frames)

        # 如果已经非常接近终点，直接刹车停稳
        if (dist < self.goal_position_tolerance and
                abs(eyaw) < self.goal_yaw_tolerance and
                abs(v_r) < self.goal_speed_tolerance):
            self.publish_stop()
            return

        # ========================================================
        # 控制律: 角速度前馈 + P纠正
        # cmd.angular.z = omega_des + k_yaw * eyaw + k_y * |v| * ey
        # ========================================================
        cmd = Twist()
        if turn_alignment:
            cmd.linear.x = 0.0
            cmd.angular.z = self.bounded_alignment_angular_command(
                eyaw, self.final_alignment_k_yaw,
                self.turn_alignment_min_angular_speed,
                self.final_alignment_max_angular_speed)
            rospy.loginfo_throttle(
                1.0,
                "Bounded yaw alignment: error %.3f rad, reference speed %.3f m/s",
                eyaw, v_r)
        else:
            cmd.linear.x = v_r * math.cos(eyaw) + self.k_x * ex
            cmd.angular.z = self.target_w + self.k_yaw * eyaw + self.k_y * v_r * ey

        # 安全限幅
        if cmd.linear.x > self.max_v:  cmd.linear.x = self.max_v
        if cmd.linear.x < -self.max_v: cmd.linear.x = -self.max_v
        if cmd.angular.z > self.max_w: cmd.angular.z = self.max_w
        if cmd.angular.z < -self.max_w: cmd.angular.z = -self.max_w

        if self.safety_speed_limit is not None:
            if self.safety_speed_limit <= 1e-3:
                self.publish_stop()
                return
            else:
                cmd.linear.x = max(-self.safety_speed_limit,
                                   min(self.safety_speed_limit, cmd.linear.x))

        # Keep trajectory handoffs physically smooth. The polynomial trajectory
        # already respects planning limits; this only bounds tracker corrections.
        cmd.linear.x = self.limit_rate(cmd.linear.x, self.last_cmd.linear.x,
                                       self.max_linear_accel, dt)
        cmd.angular.z = self.limit_rate(cmd.angular.z, self.last_cmd.angular.z,
                                        self.max_angular_accel, dt)
        self.last_cmd = cmd

        # 下发给底盘驱动节点
        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    try:
        RealBaseTracker()
    except rospy.ROSInterruptException:
        pass
