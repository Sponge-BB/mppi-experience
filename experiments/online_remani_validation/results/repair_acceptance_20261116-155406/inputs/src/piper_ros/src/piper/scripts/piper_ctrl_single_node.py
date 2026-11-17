#!/usr/bin/env python3
# -*-coding:utf8-*-
# 本文件为控制单个机械臂节点，控制夹爪机械臂运动（固定关节速度版本）
from typing import (
    Optional,
)
import rospy
import rosnode
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, UInt8MultiArray
from std_srvs.srv import SetBool, SetBoolResponse
import time
import threading
import argparse
import math
import xml.etree.ElementTree as ET
from piper_sdk import *
from piper_sdk import C_PiperInterface
from piper_feedback_dispatch import FeedbackDispatch
from std_srvs.srv import Trigger, TriggerResponse
from piper_msgs.msg import PiperStatusMsg, PiperExecutionState, PosCmd, PiperEulerPose
from piper_msgs.srv import Enable, EnableResponse
from piper_msgs.srv import Gripper, GripperResponse
from piper_msgs.srv import GoZero, GoZeroResponse
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_euler  # 用于欧拉角到四元数的转换

ARM_JOINT_NAMES = tuple('joint%d' % i for i in range(1, 7))
SDK_JOINT_GROUPS = ('ArmJoint_12', 'ArmJoint_34', 'ArmJoint_56')
# Fresh feedback still grants control for only 150 ms. A short loss of that
# permission holds the existing finite MOVE J target; 300 ms latches a stop.
FEEDBACK_VALID_AGE = .15
FEEDBACK_STOP_AGE = .30

def check_ros_master():
    try:
        rosnode.rosnode_ping('rosout', max_count=1, verbose=False)
        rospy.loginfo("ROS Master is running.")
    except rosnode.ROSNodeIOException:
        rospy.logerr("ROS Master is not running.")
        raise RuntimeError("ROS Master is not running.")

class C_PiperRosNode():
    """机械臂ros节点（固定关节速度）"""
    def __init__(self) -> None:
        check_ros_master()
        rospy.init_node('piper_ctrl_single_node', anonymous=True)
        # 外部param参数
        # can路由名称
        self.can_port = "can0"
        if rospy.has_param('~can_port'):
            self.can_port = rospy.get_param("~can_port")
            rospy.loginfo("%s is %s", rospy.resolve_name('~can_port'), self.can_port)
        else: 
            rospy.loginfo("未找到can_port参数")
            exit(0)
        # 是否自动使能，默认不自动使能
        self.auto_enable = False
        if rospy.has_param('~auto_enable'):
            if(rospy.get_param("~auto_enable")):
                self.auto_enable = True
        rospy.loginfo("%s is %s", rospy.resolve_name('~auto_enable'), self.auto_enable)
        # 是否有夹爪，默认为有
        self.gripper_exist = True
        if rospy.has_param('~gripper_exist'):
            if(not rospy.get_param("~gripper_exist")):
                self.gripper_exist = False
        rospy.loginfo("%s is %s", rospy.resolve_name('~gripper_exist'), self.gripper_exist)
        # 是否是打开了rviz控制，默认为不是，如果打开了，gripper订阅的joint7关节消息会乘2倍-------已弃用
        self.rviz_ctrl_flag = False
        if rospy.has_param('~rviz_ctrl_flag'):
            if(rospy.get_param("~rviz_ctrl_flag")):
                self.rviz_ctrl_flag = True
        rospy.loginfo("%s is %s", rospy.resolve_name('~rviz_ctrl_flag'), self.rviz_ctrl_flag)
        # 夹爪的数值倍数，默认为1
        self.gripper_val_mutiple = 1  # 默认值
        if rospy.has_param('~gripper_val_mutiple'):
            gripper_val_mutiple = rospy.get_param("~gripper_val_mutiple")
            # 检查是否为数字（浮动数或整数）
            if isinstance(gripper_val_mutiple, (int, float)):
                # 确保值在合理范围内
                if gripper_val_mutiple <= 0:
                    rospy.logwarn("Invalid gripper_val_mutiple value: must be positive. Using default value of 1.")
                    self.gripper_val_mutiple = 1  # 设置为默认值
                else:
                    self.gripper_val_mutiple = gripper_val_mutiple
            else:
                rospy.logwarn("Invalid gripper_val_mutiple type. Expected int or float. Using default value of 1.")
                self.gripper_val_mutiple = 1  # 设置为默认值
        else:
            rospy.logwarn("No gripper_val_mutiple param. Using default value of 1.")
            self.gripper_val_mutiple = 1  # 设置为默认值
        rospy.loginfo("%s is %s", rospy.resolve_name('~gripper_val_mutiple'), self.gripper_val_mutiple)
        # 固定关节运动速度 (范围 0-100, 推荐 20-60)
        self.fixed_vel = rospy.get_param('~fixed_vel', 15)
        rospy.loginfo("%s is %s", rospy.resolve_name('~fixed_vel'), self.fixed_vel)
        self.auto_enable_timeout = rospy.get_param('~auto_enable_timeout', 10.0)
        self.exit_on_enable_timeout = rospy.get_param('~exit_on_enable_timeout', False)
        self.auto_reenable_on_drop = rospy.get_param('~auto_reenable_on_drop', True)
        self.auto_reenable_interval = rospy.get_param('~auto_reenable_interval', 1.0)
        # Optional MPPI command lease; zero preserves native enable behaviour.
        self.command_timeout = float(rospy.get_param('~command_timeout', 0.0))
        self.max_command_age = float(rospy.get_param('~max_command_age', 0.30))
        if (not math.isfinite(self.command_timeout) or self.command_timeout < 0 or
                not math.isfinite(self.max_command_age) or
                (self.command_timeout > 0 and self.max_command_age <= 0)):
            raise ValueError('invalid command_timeout/max_command_age')
        self._command_lock = threading.RLock()
        self._command_latched = False
        self._command_deadline = None
        self._last_command_stamp = 0.0
        self._command_seen = False
        self._feedback_hold = False
        self._feedback_stamp = 0.0
        self._feedback_monotonic = 0.0
        self._feedback_arrivals = ()
        self._feedback_position = None
        self._feedback_velocity = None
        self._last_joint_counts = (0, 0, 0)
        self._last_arm_status_count = 0
        self._last_arm_status_stamp = 0.0
        self._execution_source_stamp = 0.0
        self._previous_feedback = None
        self._watchdog_done = threading.Event()
        self.block_ctrl_flag = False
        self._configure_native_startup()
        if self.command_timeout > 0:
            self.auto_enable = False
            self.auto_reenable_on_drop = False
        rospy.loginfo("%s is %s", rospy.resolve_name('~auto_enable_timeout'), self.auto_enable_timeout)
        rospy.loginfo("%s is %s", rospy.resolve_name('~exit_on_enable_timeout'), self.exit_on_enable_timeout)
        rospy.loginfo("%s is %s", rospy.resolve_name('~auto_reenable_on_drop'), self.auto_reenable_on_drop)
        rospy.loginfo("%s is %s", rospy.resolve_name('~auto_reenable_interval'), self.auto_reenable_interval)
        # publish
        self.joint_pub = rospy.Publisher('joint_states_single', JointState, queue_size=1)
        self.arm_status_pub = rospy.Publisher('arm_status', PiperStatusMsg, queue_size=1)
        self.execution_state_pub = (rospy.Publisher('piper_execution_state', PiperExecutionState,
            queue_size=1) if self.command_timeout > 0 else None)
        self.end_pose_pub = rospy.Publisher('end_pose', PoseStamped, queue_size=1)
        self.end_pose_euler_pub = rospy.Publisher('end_pose_euler', PiperEulerPose, queue_size=1)
        self.motor_enable_pub = rospy.Publisher('piper_motor_enable_states', UInt8MultiArray, queue_size=1)
        # service
        self.enable_service = rospy.Service('enable_srv', Enable, self.handle_enable_service)  # 创建enable服务
        self.__enable_flag = False
        self.gripper_service = rospy.Service('gripper_srv', Gripper, self.handle_gripper_service)  # 创建gripper服务
        self.stop_service = rospy.Service('stop_srv', Trigger, self.handle_stop_service)  # 创建stop服务
        self.reset_service = rospy.Service('reset_srv', Trigger, self.handle_reset_service)  # 创建reset服务
        self.go_zero_service = rospy.Service('go_zero_srv', GoZero, self.handle_go_zero_service)  # 创建reset服务
        self.block_arm_service = rospy.Service('block_arm', SetBool, self.handle_block_arm_service)
        self._last_enable_retry_time = rospy.Time(0)
        # joint
        self.joint_states = JointState()
        self.joint_states.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        self.joint_states.position = [0.0] * 7
        self.joint_states.velocity = [0.0] * 7
        self.joint_states.effort = [0.0] * 7
        
        # 创建piper类并打开can接口
        self.piper = C_PiperInterface(can_name=self.can_port)
        if self.command_timeout > 0:
            # Install before the receive thread starts; all decoding and state
            # updates retain the SDK's timestamps, filters and feedback locks.
            self.piper._C_PiperInterface__arm_can.callback_function = FeedbackDispatch(self.piper)
            self.piper.ConnectPort(piper_init=False)
            rospy.loginfo("Piper CAN feedback uses selective SDK dispatch; source freshness checks unchanged.")
            self._configure_command_transport()
            threading.Thread(target=self._command_watchdog, daemon=True).start()
            rospy.on_shutdown(self._shutdown_commands)
        else:
            self.piper.ConnectPort()
            self.piper.MotionCtrl_2(0x01, 0x01, 30, 0)
        # 启动订阅线程
        sub_pos_th = threading.Thread(target=self.SubPosThread)
        sub_pos_th.daemon = True
        sub_pos_th.start()
        sub_joint_th = threading.Thread(target=self.SubJointThread)
        sub_enable_th = threading.Thread(target=self.SubEnableThread)
        
        sub_joint_th.daemon = True
        sub_enable_th.daemon = True
        
        sub_joint_th.start()
        sub_enable_th.start()

    def _configure_command_transport(self):
        """Bound sends and propagate the installed SDK's failure return value."""
        transport = self.piper._C_PiperInterface__arm_can
        send = transport.bus.send

        def bounded_send(message, timeout=None):
            return send(message, timeout=0.005)

        transport.bus.send = bounded_send
        send_frame = transport.SendCanMessage

        def checked_send(*args, **kwargs):
            status = send_frame(*args, **kwargs)
            # This SDK catches every bus.send exception and returns an enum.
            # JointCtrl only logs that enum; raise here before authority commits.
            if status != transport.CAN_STATUS.SEND_MESSAGE_SUCCESS:
                raise RuntimeError('Piper CAN send failed: %s' % status)
            return status

        transport.SendCanMessage = checked_send

    def GetEnableFlag(self):
        return self.__enable_flag

    def _configure_native_startup(self):
        """Validate the original fixed standby target before opening CAN."""
        self.native_startup_to_standby = rospy.get_param('~native_startup_to_standby', False)
        if not isinstance(self.native_startup_to_standby, bool):
            raise ValueError('native_startup_to_standby must be a bool')
        self._native_handoff = False
        self._native_ready = False
        self._native_standby_reached = False
        self._native_started = None
        self._native_last_enable = None
        self._native_last_target = None
        self._native_settled_since = None
        self._native_motors_seen = False
        self._native_target = None
        self._native_target_counts = None
        if not self.native_startup_to_standby:
            return
        if self.command_timeout <= 0:
            raise ValueError('native startup requires a positive command_timeout')
        initial = rospy.get_param('/remani_planner_node/fsm/init_state')
        if (not isinstance(initial, (list, tuple)) or len(initial) != 8 or
                any(isinstance(q, bool) or not isinstance(q, (int, float)) or
                    not math.isfinite(q) for q in initial)):
            raise ValueError('native startup requires the finite 8-value original fsm/init_state')
        target = tuple(math.radians(q) for q in initial[2:])
        robot = ET.fromstring(rospy.get_param('/robot_description'))
        for name, q in zip(ARM_JOINT_NAMES, target):
            joint = robot.find("joint[@name='%s']" % name)
            limit = joint.find('limit') if joint is not None else None
            if limit is None:
                raise ValueError('native startup requires URDF limits for ' + name)
            lower, upper = float(limit.get('lower')), float(limit.get('upper'))
            if not (math.isfinite(lower) and math.isfinite(upper) and lower < upper and lower <= q <= upper):
                raise ValueError('native startup target outside URDF limits: ' + name)
        # Retain the original joint-command bridge's wrist range.
        if not -.35 <= target[5] <= .35:
            raise ValueError('native startup target outside original joint6 command range')
        self._native_target = target
        self._native_target_counts = tuple(round(q * 180000.0 / math.pi) for q in target)

    def _native_pending(self):
        return self.native_startup_to_standby and not self._native_handoff

    def _refresh_native_ready(self):
        """Readiness is revocable and uses complete, source-derived feedback."""
        if not self._native_pending():
            return
        good = (not self._command_latched and self._native_motors_seen and
                self._feedback_fresh() and self._arm_status_normal_fresh() and
                self._motors_enabled_fresh() and self._feedback_velocity is not None and
                all(abs(q - target) < .015 for q, target in
                    zip(self._feedback_position, self._native_target)) and
                all(math.isfinite(v) and abs(v) < .02 for v in self._feedback_velocity))
        if not good:
            self._native_settled_since = None
            self._native_ready = False
        else:
            now = time.monotonic()
            if self._native_settled_since is None:
                self._native_settled_since = now
            ready = now - self._native_settled_since >= .30
            if ready and not self._native_ready:
                rospy.loginfo('Piper native standby reached; holding target until first valid MPPI command.')
            if ready:
                self._native_standby_reached = True
            self._native_ready = ready

    def _native_startup_tick(self):
        """Original 5 Hz fixed target; bounded enable/arrival, no map dependency.

        Called only with _command_lock held. The native sender and external
        JointCtrl callback therefore cannot overlap at handoff.
        """
        if not self._native_pending() or self._command_latched:
            return
        now = time.monotonic()
        # These fixed startup deadlines also run during a temporary feedback
        # hold; alternating short gaps must not keep startup alive forever.
        if self._native_started is not None:
            if not self._native_motors_seen and now - self._native_started >= 10.0:
                self._stop_commands('native startup motor enable timeout')
                return
            if not self._native_standby_reached and now - self._native_started >= 60.0:
                self._stop_commands('native startup standby timeout')
                return
        healthy = self._feedback_control_ready()
        if self.block_ctrl_flag:
            self._stop_commands('native startup blocked')
            return
        if not healthy:
            self._native_ready = False
            self._native_settled_since = None
            if self._native_started is None:
                rospy.logwarn_throttle(2.0,
                    'Piper native startup waiting for fresh normal joint/status/motor '
                    'feedback; no enable sent. A prior stop is never reset automatically.')
            return
        motor_fresh, motors_enabled, _ = self._motor_feedback_sample()
        if not motor_fresh:
            return
        if self._native_started is None:
            self._native_started = now
            self._native_last_enable = now
            # Reject queued external plans from before this startup request;
            # native target sends must never refresh this external timestamp.
            self._last_command_stamp = time.time()
            self.__enable_flag = True
            self.piper.EnableArm(7)
            rospy.loginfo('Piper native startup enable sent; motor timeout 10s, standby timeout 60s.')
        if not self._native_motors_seen:
            if motors_enabled:
                self._native_motors_seen = True
            else:
                if now - self._native_last_enable >= 1.0:
                    self._native_last_enable = now
                    self.piper.EnableArm(7)
                return
        elif not motors_enabled:
            self._stop_commands('native startup motor enable dropped')
            return
        self._refresh_native_ready()
        # The arrival deadline ends at first settled arrival. A later readiness
        # withdrawal while waiting for MPPI must not restart that old deadline.
        if self._native_last_target is None or now - self._native_last_target >= .20:
            self.piper.MotionCtrl_2(0x01, 0x01, 15, 0)
            self.piper.JointCtrl(*self._native_target_counts)
            self._native_last_target = now

    def Pubilsh(self):
        """机械臂消息发布"""
        rate = rospy.Rate(200)  # 200 Hz
        enable_flag = False
        # 设置超时时间（秒）
        timeout = self.auto_enable_timeout
        # 记录进入循环前的时间
        start_time = time.time()
        elapsed_time_flag = False
        while not rospy.is_shutdown():
            if(self.auto_enable):
                while not (enable_flag):
                    elapsed_time = time.time() - start_time
                    print("--------------------")
                    enable_states = self.GetMotorEnableStates()
                    enable_flag = len(enable_states) == 6 and all(enable_states)
                    print("电机使能状态:", enable_states)
                    print("使能状态:", enable_flag)
                    self.piper.EnableArm(7)
                    self.piper.GripperCtrl(0, 1000, 0x01, 0)
                    if(enable_flag):
                        self.__enable_flag = True
                    print("--------------------")
                    # 检查是否超过超时时间
                    if elapsed_time > timeout:
                        print("超时....")
                        elapsed_time_flag = True
                        enable_flag = True
                        break
                    time.sleep(1)
                    pass
            if(elapsed_time_flag):
                if self.exit_on_enable_timeout:
                    print("程序自动使能超时,退出程序")
                    exit(0)
                rospy.logwarn_throttle(
                    5.0,
                    "Piper auto enable timeout. Keep node alive and retry; "
                    "motor_enable_states=%s; check arm power, emergency stop, CAN cable, and driver state.",
                    self.GetMotorEnableStates())
                self.__enable_flag = False
                elapsed_time_flag = False
                enable_flag = False
                start_time = time.time()
            # 发布消息
            self.MaintainAutoEnable()
            self.PublishArmJointAndGripper()
            self.PublishMotorEnableStates()
            self.PublishArmState()
            self.PublishArmEndPose()
            rate.sleep()

    def MaintainAutoEnable(self):
        if not self.auto_enable or not self.auto_reenable_on_drop:
            return

        enable_states = self.GetMotorEnableStates()
        all_enabled = len(enable_states) == 6 and all(enable_states)
        if all_enabled:
            if not self.__enable_flag:
                rospy.logwarn("Piper all motors enabled again: %s", enable_states)
            self.__enable_flag = True
            return

        self.__enable_flag = False
        now = rospy.Time.now()
        if (now - self._last_enable_retry_time).to_sec() < self.auto_reenable_interval:
            return

        self._last_enable_retry_time = now
        rospy.logwarn_throttle(
            2.0,
            "Piper motor enable drop detected: %s. Pause joint commands and retry EnableArm(7).",
            enable_states)
        self.piper.EnableArm(7)
        if self.gripper_exist:
            self.piper.GripperCtrl(0, 1000, 0x01, 0)

    def GetMotorEnableStates(self):
        # The SDK getter also computes six unused FPS values under locks.
        with self.piper._C_PiperInterface__arm_motor_info_low_spd_mtx:
            state = self.piper._C_PiperInterface__arm_motor_info_low_spd
            return [bool(getattr(state, 'motor_%d' % i).foc_status.driver_enable_status)
                    for i in range(1, 7)]

    def PublishMotorEnableStates(self):
        if self.motor_enable_pub.get_num_connections() == 0:
            return
        msg = UInt8MultiArray()
        msg.data = [1 if enabled else 0 for enabled in self.GetMotorEnableStates()]
        self.motor_enable_pub.publish(msg)

    def PublishArmState(self):
        if self.command_timeout > 0:
            message, stamp, count, fresh, _ = self._arm_status_sample()
            if (not fresh or count <= self._last_arm_status_count or
                    stamp <= self._last_arm_status_stamp):
                return
            self._last_arm_status_count = count
            self._last_arm_status_stamp = stamp
            self.arm_status_pub.publish(message)
            return
        # 机械臂状态
        arm_status = PiperStatusMsg()
        arm_status.ctrl_mode = self.piper.GetArmStatus().arm_status.ctrl_mode
        arm_status.arm_status = self.piper.GetArmStatus().arm_status.arm_status
        arm_status.mode_feedback = self.piper.GetArmStatus().arm_status.mode_feed
        arm_status.teach_status = self.piper.GetArmStatus().arm_status.teach_status
        arm_status.motion_status = self.piper.GetArmStatus().arm_status.motion_status
        arm_status.trajectory_num = self.piper.GetArmStatus().arm_status.trajectory_num
        arm_status.err_code = self.piper.GetArmStatus().arm_status.err_code
        arm_status.joint_1_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_1_angle_limit
        arm_status.joint_2_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_2_angle_limit
        arm_status.joint_3_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_3_angle_limit
        arm_status.joint_4_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_4_angle_limit
        arm_status.joint_5_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_5_angle_limit
        arm_status.joint_6_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_6_angle_limit
        arm_status.communication_status_joint_1 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_1
        arm_status.communication_status_joint_2 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_2
        arm_status.communication_status_joint_3 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_3
        arm_status.communication_status_joint_4 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_4
        arm_status.communication_status_joint_5 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_5
        arm_status.communication_status_joint_6 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_6
        self.arm_status_pub.publish(arm_status)

    def _arm_status_sample(self):
        # The installed SDK updates these together only on an ArmStatus CAN
        # frame. Copy scalars under that lock; publishing never refreshes them.
        with self.piper._C_PiperInterface__arm_status_mtx:
            feedback = self.piper._C_PiperInterface__arm_status
            counter = self.piper._C_PiperInterface__fps_counter
            with counter.lock:
                count = counter.fps_data.get('ArmStatus', 0)
                arrival = counter.last_time.get('ArmStatus', 0.0)
            stamp = float(feedback.time_stamp)
            state = feedback.arm_status
            message = PiperStatusMsg()
            for field in ('ctrl_mode', 'arm_status', 'teach_status', 'motion_status',
                          'trajectory_num', 'err_code'):
                setattr(message, field, getattr(state, field))
            message.mode_feedback = state.mode_feed
            for i in range(1, 7):
                for field in ('joint_%d_angle_limit' % i, 'communication_status_joint_%d' % i):
                    setattr(message, field, getattr(state.err_status, field))
        deadline = self._source_deadline(stamp, (arrival,)) if count > 0 else 0.
        return message, stamp, count, time.time() < deadline, deadline

    @staticmethod
    def _source_deadline(stamp, arrivals):
        """Preserve both CAN wall time and each SDK receive-time age limit."""
        wall, steady = time.time(), time.perf_counter()
        if (not math.isfinite(stamp) or stamp <= 0 or stamp > wall or
                not arrivals or any(not math.isfinite(t) or t < 0 or t > steady for t in arrivals)):
            return 0.
        return min(stamp+FEEDBACK_VALID_AGE,
                   wall+FEEDBACK_VALID_AGE-max(steady-t for t in arrivals))

    def _arm_status_normal_fresh(self):
        message, _, _, fresh, _ = self._arm_status_sample()
        return fresh and message.arm_status == 0 and message.err_code == 0

    def PublishExecutionState(self, stopped=False):
        if self.command_timeout <= 0:
            return
        # Serialize positive feedback and stop publication with command writes.
        # A stop report needs no CAN access and keeps the last real source stamp.
        with self._command_lock:
            message = PiperExecutionState()
            message.stop_latched = self._command_latched
            message.command_active = False
            if not stopped and not self._command_latched:
                joint_fresh = self._feedback_fresh()
                status, stamp, _, status_fresh, deadline = self._arm_status_sample()
                self._execution_source_stamp = stamp if math.isfinite(stamp) and stamp > 0 else 0.
                motor_fresh, motors_enabled, motor_deadline = self._motor_feedback_sample()
                counter = self.piper._C_PiperInterface__fps_counter
                with counter.lock:
                    joint_received = all(counter.fps_data.get(k, 0) > 0 for k in SDK_JOINT_GROUPS)
                    joint_arrivals = tuple(counter.last_time.get(k, 0.) for k in SDK_JOINT_GROUPS)
                joint_deadline = self._source_deadline(self._feedback_stamp, joint_arrivals) if joint_received else 0.
                deadline = min(deadline, motor_deadline, joint_deadline)
                message.feedback_valid = bool(status_fresh and status.arm_status == 0 and
                    status.err_code == 0 and joint_fresh and motor_fresh and time.time() < deadline)
                self._refresh_native_ready()
                native_pending = self._native_pending()
                message.enabled = bool(self.__enable_flag and motor_fresh and motors_enabled and
                                       (not native_pending or self._native_ready))
                has_lease = self._command_deadline is not None
                # An old MOVE J lease does not expire healthy permission to
                # accept the next command. Report its activity separately.
                message.command_active = bool(message.enabled and message.feedback_valid and
                    not native_pending and self._command_seen and has_lease and
                    time.monotonic() < self._command_deadline)
                message.valid_until = rospy.Time.from_sec(max(0., deadline))
            message.header.stamp = rospy.Time.from_sec(self._execution_source_stamp)
            self.execution_state_pub.publish(message)
        
    def _read_joint_feedback(self):
        # Installed SDK stores one source CAN timestamp for all three pairs.
        # Snapshot under its joint lock and require new packets for EVERY pair;
        # otherwise one live pair could refresh four stale/default-zero joints.
        with self.piper._C_PiperInterface__arm_joint_msgs_mtx:
            feedback = self.piper._C_PiperInterface__arm_joint_msgs
            counter = self.piper._C_PiperInterface__fps_counter
            with counter.lock:
                counts = tuple(counter.fps_data.get(k, 0) for k in SDK_JOINT_GROUPS)
                arrivals = tuple(counter.last_time.get(k, 0.0) for k in SDK_JOINT_GROUPS)
            stamp = float(feedback.time_stamp)
            positions = [getattr(feedback.joint_state, 'joint_%d' % i) * math.pi / 180000.0
                         for i in range(1, 7)]
        return stamp, counts, arrivals, positions

    def _refresh_joint_feedback(self):
        """Accept SDK feedback independently of ROS publication; caller holds _command_lock."""
        stamp, counts, arrivals, positions = self._read_joint_feedback()
        if (any(n <= old for n, old in zip(counts, self._last_joint_counts)) or
                time.perf_counter() - min(arrivals) > 0.15 or
                max(arrivals) - min(arrivals) > 0.05 or
                not math.isfinite(stamp) or not 0 <= time.time() - stamp < 0.15 or
                stamp <= self._feedback_stamp or not all(math.isfinite(q) for q in positions)):
            return
        self._last_joint_counts = counts
        dt = stamp - self._feedback_stamp
        self._feedback_velocity = (tuple((q - old) / dt for q, old in
            zip(positions, self._feedback_position)) if self._feedback_position is not None and
            0 < dt <= .15 else None)
        self._feedback_stamp = stamp
        self._feedback_monotonic = time.monotonic()
        self._feedback_arrivals = arrivals
        self._feedback_position = tuple(positions)

    def PublishArmJointAndGripper(self):
        with self._command_lock:
            if not self._feedback_fresh():
                return
            stamp, positions = self._feedback_stamp, self._feedback_position
            # A watchdog read may accept a frame before this publishing thread.
            # Publish it once, with its original source time, never ROS time.
            if self._previous_feedback is not None and stamp <= self._previous_feedback[0]:
                return
        msg = JointState()
        msg.header.stamp = rospy.Time.from_sec(stamp)
        msg.name = list(ARM_JOINT_NAMES)
        msg.position = list(positions)
        if self.gripper_exist:
            gripper = self.piper.GetArmGripperMsgs()
            if 0 <= time.time() - gripper.time_stamp < 0.15:
                msg.name.append('gripper')
                msg.position.append(gripper.gripper_state.grippers_angle / 1000000.0)
        # Derive joint velocity from complete, source-stamped position samples.
        # The first sample has unknown velocity, represented by an empty array.
        previous = self._previous_feedback
        if previous is not None:
            old_stamp, old_names, old_q = previous
            dt = stamp - old_stamp
            if old_names == msg.name and 0 < dt <= 0.15:
                msg.velocity = [(q - old) / dt for q, old in zip(msg.position, old_q)]
        self._previous_feedback = (stamp, list(msg.name), list(msg.position))
        self.joint_pub.publish(msg)

    def _feedback_fresh(self):
        with self._command_lock:
            self._refresh_joint_feedback()
            return (self._feedback_position is not None and
                    time.time() < self._source_deadline(self._feedback_stamp, self._feedback_arrivals) and
                    0 <= time.monotonic() - self._feedback_monotonic < 0.15)

    def _joint_feedback_timeout_reason(self):
        stamp, counts, arrivals, _ = self._read_joint_feedback()
        wall, steady = time.time(), time.perf_counter()
        return ('joint feedback expired; accepted_source_age_ms=%.3f '
                'sdk_source_age_ms=%.3f pair_update_age_ms=%s pair_spread_ms=%.3f '
                'pair_advanced=%s' % (
                    (wall-self._feedback_stamp)*1000, (wall-stamp)*1000,
                    tuple(round((steady-t)*1000, 3) for t in arrivals),
                    (max(arrivals)-min(arrivals))*1000,
                    tuple(n > old for n, old in zip(counts, self._last_joint_counts))))

    def _motors_enabled_fresh(self, diagnostics=None):
        fresh, enabled, _ = self._motor_feedback_sample(diagnostics)
        return fresh and enabled

    def _motor_feedback_sample(self, diagnostics=None):
        with self.piper._C_PiperInterface__arm_motor_info_low_spd_mtx:
            state = self.piper._C_PiperInterface__arm_motor_info_low_spd
            counter = self.piper._C_PiperInterface__fps_counter
            with counter.lock:
                keys = ['ArmMotorDriverInfoLowSpd_%d' % i for i in range(1, 7)]
                received = tuple(counter.fps_data.get(k, 0) > 0 for k in keys)
                arrivals = tuple(counter.last_time.get(k, 0.0) for k in keys)
            stamp = float(state.time_stamp)
            enabled = tuple(bool(getattr(state, 'motor_%d' % i).foc_status.driver_enable_status)
                            for i in range(1, 7))
        deadline = self._source_deadline(stamp, arrivals) if all(received) else 0.
        wall = time.time()
        fresh = wall < deadline
        if diagnostics is not None:
            # Preserve this checked sample, not a later SDK read after recovery
            # or quick-stop. Ages are SDK updates; only stamp is CAN source time.
            steady = time.perf_counter()
            diagnostics.update(
                fresh=fresh, received=received, enabled=enabled,
                disabled_fresh=(0 <= wall-stamp < FEEDBACK_VALID_AGE and
                    any(known and not on and 0 <= steady-arrival < FEEDBACK_VALID_AGE
                        for known, on, arrival in zip(received, enabled, arrivals))),
                update_age_ms=tuple(round((steady-t)*1000., 3) for t in arrivals),
                source_age_ms=(wall-stamp)*1000.,
                valid_for_ms=(deadline-wall)*1000. if deadline > 0 else None)
        return fresh, all(enabled), deadline

    def _feedback_control_ready(self):
        """Separate fresh-control permission from a sustained feedback failure.

        Called with _command_lock held. Do not refresh timestamps or resend a
        target during the grace period. Explicit status faults and confirmed
        motor disable still stop immediately; recovery never clears a latch.
        """
        if self._command_latched:
            return False
        joint_fresh = self._feedback_fresh()
        status, _, count, status_fresh, status_deadline = self._arm_status_sample()
        motor_diagnostics = {}
        motor_fresh, motors_enabled, motor_deadline = self._motor_feedback_sample(motor_diagnostics)
        normal = status.arm_status == 0 and status.err_code == 0
        healthy = joint_fresh and status_fresh and normal and motor_fresh
        if not self.GetEnableFlag():
            return healthy
        motor_reason = (
            'motor feedback unavailable/disabled; motor_fresh=%s '
            'received=%s update_age_ms=%s enabled=%s '
            'sdk_source_age_ms=%.3f valid_for_ms=%s' % (
                motor_diagnostics['fresh'], motor_diagnostics['received'],
                motor_diagnostics['update_age_ms'], motor_diagnostics['enabled'],
                motor_diagnostics['source_age_ms'], motor_diagnostics['valid_for_ms']))
        if count > 0 and not normal:
            self._stop_commands('arm status abnormal; arm_status=%s err_code=%s' % (
                status.arm_status, status.err_code))
            return False
        # One delayed motor must not hide another motor's fresh disable bit.
        if motor_diagnostics['disabled_fresh'] and (self._command_seen or self._native_motors_seen):
            self._stop_commands(motor_reason)
            return False
        wall = time.time()
        grace = FEEDBACK_STOP_AGE - FEEDBACK_VALID_AGE
        joint_deadline = self._source_deadline(self._feedback_stamp, self._feedback_arrivals)
        reason = None
        if not joint_fresh and (self._feedback_position is None or
                joint_deadline <= 0 or wall >= joint_deadline+grace or
                not 0 <= time.monotonic()-self._feedback_monotonic < FEEDBACK_STOP_AGE):
            reason = self._joint_feedback_timeout_reason()
        elif not status_fresh and (status_deadline <= 0 or wall >= status_deadline+grace):
            reason = 'arm status feedback expired (300ms hard limit)'
        elif not motor_fresh and (motor_deadline <= 0 or wall >= motor_deadline+grace):
            reason = motor_reason
        if reason is not None:
            self._stop_commands(reason)
            return False
        if not healthy:
            if not self._feedback_hold:
                rospy.logwarn('Piper feedback temporarily stale; no new targets, retaining last '
                              'finite target and motor enable (150ms validity / 300ms stop). '
                              'joint_fresh=%s status_fresh=%s motor_fresh=%s',
                              joint_fresh, status_fresh, motor_fresh)
            self._feedback_hold = True
            return False
        if self._feedback_hold:
            rospy.loginfo('Piper fresh feedback recovered; new source-stamped targets may resume.')
        self._feedback_hold = False
        return True

    def _stop_commands(self, reason):
        with self._command_lock:
            if self._command_latched:
                return
            self._command_latched = True
            self.__enable_flag = False
            self._command_deadline = None
            self._native_ready = False
            self._native_settled_since = None
            rospy.logerr('Piper command stop latched: %s; no automatic reset/re-enable.', reason)
            try:
                self.PublishExecutionState(stopped=True)
            except Exception as exc:
                rospy.logerr('Piper stop state publication failed: %s', exc)
            try:
                self.piper.MotionCtrl_1(0x01, 0, 0)
            except Exception as exc:
                rospy.logerr('Piper quick-stop send failed: %s', exc)

    def _check_command_timeout(self):
        with self._command_lock:
            if self._native_pending():
                self._native_startup_tick()
                return
            if self.__enable_flag and not self._command_latched:
                # A missed planner deadline is distinct from a hardware fault.
                # Keep monitoring feedback even while the command stream waits.
                if not self._feedback_control_ready():
                    return
                if (self._command_deadline is None or
                        time.monotonic() >= self._command_deadline):
                    if not self._command_seen:
                        self._stop_commands('command expired (first_command_received=False)')
                    elif self._command_deadline is not None:
                        # MOVE J already has a finite, bounded position target.
                        # Send nothing: retain that target instead of quick-stop
                        # (which loses holding on this robot). Retain the last
                        # accepted source stamp: a new plan can span this benign
                        # hold if its own age, ordering and feedback checks pass.
                        self._command_deadline = None
                        rospy.logwarn('Piper command stream expired; retaining last bounded joint '
                                      'target, awaiting a new source-stamped command. Motors remain enabled.')

    def _command_watchdog(self):
        while not self._watchdog_done.wait(0.02):
            try:
                self._check_command_timeout()
                self.PublishExecutionState()
            except Exception as exc:
                self._stop_commands('watchdog/status failure: %s' % exc)

    def _shutdown_commands(self):
        with self._command_lock:
            if self.__enable_flag:
                self._stop_commands('node shutdown')
        self._watchdog_done.set()

    def _request_enable(self, enabled):
        with self._command_lock:
            if not enabled:
                self._stop_commands('disable requested')
                return True
            if (self._command_latched or not self._feedback_fresh() or
                    not self._arm_status_normal_fresh()):
                rospy.logwarn('Enable rejected: latched stop or no fresh normal arm/joint feedback.')
                return False
            if self._native_pending():
                try:
                    self._native_startup_tick()
                    return self.__enable_flag and not self._command_latched
                except Exception:
                    self._stop_commands('native startup enable/send failed')
                    return False
            if self.__enable_flag:
                return True
            try:
                self._command_deadline = time.monotonic() + self.command_timeout
                self._last_command_stamp = time.time()
                self.piper.EnableArm(7)
                self.__enable_flag = True
                rospy.loginfo('Piper enable sent; awaiting motor feedback and a fresh command within %.3fs.',
                              self.command_timeout)
                self.PublishExecutionState()
                return True
            except Exception:
                self._stop_commands('enable send failed')
                return False

    def PublishArmEndPose(self):
        pose_requested = self.end_pose_pub.get_num_connections() > 0
        euler_requested = self.end_pose_euler_pub.get_num_connections() > 0
        if not pose_requested and not euler_requested:
            return
        # One SDK read serves both optional telemetry formats.
        pose = self.piper.GetArmEndPoseMsgs().end_pose
        x, y, z = pose.X_axis/1000000, pose.Y_axis/1000000, pose.Z_axis/1000000
        roll = pose.RX_axis/1000
        pitch = pose.RY_axis/1000
        yaw = pose.RZ_axis/1000
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        if pose_requested:
            endpos = PoseStamped()
            endpos.pose.position.x = x
            endpos.pose.position.y = y
            endpos.pose.position.z = z
            quaternion = quaternion_from_euler(roll, pitch, yaw)
            endpos.pose.orientation.x = quaternion[0]
            endpos.pose.orientation.y = quaternion[1]
            endpos.pose.orientation.z = quaternion[2]
            endpos.pose.orientation.w = quaternion[3]
            endpos.header.stamp = rospy.Time.now()
            self.end_pose_pub.publish(endpos)
        if euler_requested:
            end_pose_euler = PiperEulerPose()
            end_pose_euler.header.stamp = rospy.Time.now()
            end_pose_euler.x = x
            end_pose_euler.y = y
            end_pose_euler.z = z
            end_pose_euler.roll = roll
            end_pose_euler.pitch = pitch
            end_pose_euler.yaw = yaw
            self.end_pose_euler_pub.publish(end_pose_euler)
    
    def SubPosThread(self):
        """机械臂末端位姿订阅"""
        rospy.Subscriber('pos_cmd', PosCmd, self.pos_callback, queue_size=1, tcp_nodelay=True)
        rospy.spin()
    
    def SubJointThread(self):
        """机械臂关节订阅"""
        rospy.Subscriber('joint_ctrl_single', JointState, self.joint_callback, queue_size=1, tcp_nodelay=True)
        rospy.spin()
    
    def SubEnableThread(self):
        """机械臂使能"""
        rospy.Subscriber('enable_flag', Bool, self.enable_callback, queue_size=1, tcp_nodelay=True)
        rospy.spin()

    def pos_callback(self, pos_data):
        """机械臂末端位姿订阅回调函数"""
        if self.command_timeout > 0:
            rospy.logwarn_throttle(5.0, 'pos_cmd is disabled when command_timeout is active.')
            return
        if not self.block_ctrl_flag:
            factor = 180 / 3.1415926
            x = round(pos_data.x*1000) * 1000
            y = round(pos_data.y*1000) * 1000
            z = round(pos_data.z*1000) * 1000
            rx = round(pos_data.roll*1000*factor) 
            ry = round(pos_data.pitch*1000*factor)
            rz = round(pos_data.yaw*1000*factor)
            rospy.loginfo("Received PosCmd:")
            rospy.loginfo("x: %f", x)
            rospy.loginfo("y: %f", y)
            rospy.loginfo("z: %f", z)
            rospy.loginfo("roll: %f", rx)
            rospy.loginfo("pitch: %f", ry)
            rospy.loginfo("yaw: %f", rz)
            rospy.loginfo("gripper: %f", pos_data.gripper)
            rospy.loginfo("mode1: %d", pos_data.mode1)
            rospy.loginfo("mode2: %d", pos_data.mode2)
            if(self.GetEnableFlag()):
                self.piper.MotionCtrl_1(0x00, 0x00, 0x00)
                # 末端位姿控制也使用统一的固定速度参数
                self.piper.MotionCtrl_2(0x01, 0x00, self.fixed_vel)
                self.piper.EndPoseCtrl(x, y, z, rx, ry, rz)
                gripper = round(pos_data.gripper*1000*1000)
                if(pos_data.gripper>80000): gripper = 80000
                if(pos_data.gripper<0): gripper = 0
                if(self.gripper_exist):
                    self.piper.GripperCtrl(abs(gripper), 1000, 0x01, 0)
                self.piper.MotionCtrl_2(0x01, 0x00, self.fixed_vel)
    
    def joint_callback(self, joint_data):
        """机械臂关节角回调函数（固定速度核心逻辑）"""
        with self._command_lock:
            try:
                self._joint_command(joint_data)
            except Exception as exc:
                rospy.logerr('Piper joint command failed: %s', exc)
                if self.command_timeout > 0:
                    self._stop_commands('joint command/send failed')

    def _joint_command(self, joint_data):
        if self.command_timeout > 0:
            self._check_command_timeout()
            if self._command_latched or self._feedback_hold:
                return
            if self._native_pending() and not self._native_ready:
                return
        if (len(joint_data.position) not in (6, 7) or
                not all(math.isfinite(q) for q in joint_data.position) or
                (joint_data.name and
                 (len(joint_data.name) != len(joint_data.position) or
                  tuple(joint_data.name[:6]) != ARM_JOINT_NAMES))):
            rospy.logwarn_throttle(2.0, 'Reject incomplete/nonfinite/unordered Piper joint command.')
            return
        if not self._feedback_fresh():
            return
        if self.command_timeout > 0:
            stamp = joint_data.header.stamp.to_sec()
            age = time.time() - stamp
            if (not math.isfinite(stamp) or stamp <= self._last_command_stamp or
                    not 0 <= age < self.max_command_age or
                    not self._motors_enabled_fresh()):
                return
            if any(abs(q-current) > .03+1e-9 for q, current in
                   zip(joint_data.position[:6], self._feedback_position)):
                rospy.logwarn_throttle(2.0, 'Reject Piper joint target exceeding 0.03 rad feedback lead.')
                return
            command_deadline = time.monotonic() + min(
                self.command_timeout, self.max_command_age - age)
        if not self.block_ctrl_flag:
            factor = 180000.0 / math.pi
            joint_0 = round(joint_data.position[0]*factor)
            joint_1 = round(joint_data.position[1]*factor)
            joint_2 = round(joint_data.position[2]*factor)
            joint_3 = round(joint_data.position[3]*factor)
            joint_4 = round(joint_data.position[4]*factor)
            joint_5 = round(joint_data.position[5]*factor)
            if(len(joint_data.position) >= 7):
                joint_6 = round(joint_data.position[6]*1000*1000)
                joint_6 = joint_6 * self.gripper_val_mutiple
                if(joint_6>80000): joint_6 = 80000
                if(joint_6<0): joint_6 = 0
            else: joint_6 = None
            if(self.GetEnableFlag()):
                # 关节角度控制时，使用可配置的固定速度参数
                self.piper.MotionCtrl_2(0x01, 0x01, self.fixed_vel, 0)
                
                # 给定关节角位置
                self.piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
                if self.command_timeout > 0:
                    # Commit only after the SDK send succeeds. Preserve the
                    # pre-send age limit rather than extending it by CAN time.
                    self._command_deadline = command_deadline
                    if time.monotonic() >= command_deadline:
                        self._stop_commands('command expired during joint send')
                        return
                    if self._native_pending():
                        self._native_handoff = True
                        self._native_ready = False
                        self._native_settled_since = None
                        rospy.loginfo('Piper native startup handed off to leased MPPI commands.')
                    if not self._command_seen:
                        rospy.loginfo('Piper first leased joint command sent (source age=%.3fs).', age)
                    self._last_command_stamp = stamp
                    self._command_seen = True
                    # A successfully delivered finite target owns the handoff
                    # even if feedback briefly expires during its send. Do not
                    # resume the native sender over that external target.
                    if not self._feedback_control_ready():
                        return
                
                # 如果末端夹爪存在，则发送末端夹爪控制
                if(self.gripper_exist and joint_6 is not None):
                    if abs(joint_6)<200:
                        joint_6=0
                    if(len(joint_data.effort) >= 7):
                        gripper_effort = joint_data.effort[6]
                        gripper_effort = max(0.5, min(gripper_effort, 3))
                        gripper_effort = round(gripper_effort*1000)
                        self.piper.GripperCtrl(abs(joint_6), gripper_effort, 0x01, 0)
                    else:
                        self.piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)
    
    def enable_callback(self, enable_flag:Bool):
        """机械臂使能回调函数"""
        if self.command_timeout > 0:
            self._request_enable(bool(enable_flag.data))
            return
        rospy.loginfo("Received enable flag: %s", enable_flag.data)
        if(enable_flag.data):
            self.__enable_flag = True
            self.piper.EnableArm(7)
            if(self.gripper_exist):
                self.piper.GripperCtrl(0,1000,0x01, 0)
        else:
            self.__enable_flag = False
            self.piper.DisableArm(7)
            if(self.gripper_exist):
                self.piper.GripperCtrl(0,1000,0x00, 0)
    
    def handle_gripper_service(self,req):
        response = GripperResponse()
        response.code = 15999
        response.status = False
        if self.command_timeout > 0:
            return response
        if(self.gripper_exist):
            rospy.loginfo(f"-----------------------Gripper---------------------------")
            rospy.loginfo(f"Received request:")
            rospy.loginfo(f"PS: Piper should be enable.Please ensure piper is enable")
            rospy.loginfo(f"gripper_angle:{req.gripper_angle}, range is [0m, 0.07m]")
            rospy.loginfo(f"gripper_effort:{req.gripper_effort},range is [0.5N/m, 2N/m]")
            rospy.loginfo(f"gripper_code:{req.gripper_code}, range is [0, 1, 2, 3]\n \
                            0x00: Disable\n \
                            0x01: Enable\n \
                            0x03/0x02: Enable and clear error / Disable and clear error")
            rospy.loginfo(f"set_zero:{req.set_zero}, range is [0, 0xAE] \n \
                            0x00: Invalid value \n \
                            0xAE: Set zero point")
            rospy.loginfo(f"-----------------------Gripper---------------------------")
            gripper_angle = req.gripper_angle
            gripper_angle = round(max(0, min(req.gripper_angle, 0.07)) * 1e6)
            gripper_effort = req.gripper_effort
            gripper_effort = round(max(0.5, min(req.gripper_effort, 2)) * 1e3)
            if req.gripper_code not in [0x00, 0x01, 0x02, 0x03]:
                rospy.logwarn("gripper_code should be in [0, 1, 2, 3], default val is 1")
                gripper_code = 1
                response.code = 15901
            else: gripper_code = req.gripper_code
            if req.set_zero not in [0x00, 0xAE]:
                rospy.logwarn("set_zero should be in [0, 0xAE], default val is 0")
                set_zero = 0
                response.code = 15902
            else: set_zero = req.set_zero
            response.code = 15900
            self.piper.GripperCtrl(abs(gripper_angle), gripper_effort, gripper_code, set_zero)
            response.status = True
        else:
            rospy.logwarn("gripper_exist param is False.")
            response.code = 15903
            response.status = False
        rospy.loginfo(f"Returning GripperResponse: {response.code}, {response.status}")
        return response
    
    def handle_enable_service(self,req):
        if self.command_timeout > 0:
            return EnableResponse(self._request_enable(bool(req.enable_request)))
        rospy.loginfo(f"Received request: {req.enable_request}")
        enable_flag = False
        loop_flag = False
        timeout = 5
        start_time = time.time()
        elapsed_time_flag = False
        while not (loop_flag):
            elapsed_time = time.time() - start_time
            print("--------------------")
            enable_list = self.GetMotorEnableStates()
            print("电机使能状态:", enable_list)
            if(req.enable_request):
                enable_flag = all(enable_list)
                self.piper.EnableArm(7)
                self.piper.GripperCtrl(0,1000,0x01, 0)
            else:
                enable_flag = any(enable_list)
                self.piper.DisableArm(7)
                self.piper.GripperCtrl(0,1000,0x02, 0)
            print("使能状态:", enable_flag)
            self.__enable_flag = enable_flag
            print("--------------------")
            if(enable_flag == req.enable_request):
                loop_flag = True
                enable_flag = True
            else: 
                loop_flag = False
                enable_flag = False
            if elapsed_time > timeout:
                print("超时....")
                elapsed_time_flag = True
                enable_flag = False
                loop_flag = True
                break
            time.sleep(0.5)
        response = enable_flag
        rospy.loginfo(f"Returning response: {response}")
        return EnableResponse(response)

    def handle_stop_service(self,req):
        if self.command_timeout > 0:
            self._stop_commands('stop service')
            return TriggerResponse(success=True, message='Piper stop latched; restart required before further commands.')
        response = TriggerResponse()
        response.success = False
        response.message = "stop piper failed"
        rospy.loginfo(f"-----------------------STOP---------------------------")
        rospy.loginfo(f"Stop piper.")
        rospy.loginfo(f"-----------------------STOP---------------------------")
        self.piper.MotionCtrl_1(0x01,0,0)
        response.success = True
        response.message = "stop piper success"
        rospy.loginfo(f"Returning StopResponse: {response.success}, {response.message}")
        return response

    def handle_reset_service(self,req):
        if self.command_timeout > 0:
            return TriggerResponse(success=False, message='Automatic reset is disabled in command-timeout mode.')
        response = TriggerResponse()
        response.success = False
        response.message = "reset piper failed"
        rospy.loginfo(f"-----------------------RESET---------------------------")
        rospy.loginfo(f"reset piper.")
        rospy.loginfo(f"-----------------------RESET---------------------------")
        self.piper.MotionCtrl_1(0x02,0,0)#恢复
        response.success = True
        response.message = "reset piper success"
        rospy.loginfo(f"Returning resetResponse: {response.success}, {response.message}")
        return response

    def handle_go_zero_service(self,req):
        if self.command_timeout > 0:
            response = GoZeroResponse()
            response.status = False
            response.code = 151000
            return response
        response = GoZeroResponse()
        response.status = False
        response.code = 151000
        rospy.loginfo(f"-----------------------GOZERO---------------------------")
        rospy.loginfo(f"piper go zero .")
        rospy.loginfo(f"-----------------------GOZERO---------------------------")
        if(req.is_mit_mode):
            self.piper.MotionCtrl_2(0x01, 0x01, 30, 0xAD)  # 归零速度也设为30
        else:
            self.piper.MotionCtrl_2(0x01, 0x01, 30, 0)  # 归零速度也设为30
        self.piper.JointCtrl(0, 0, 0, 0, 0, 0)
        response.status = True
        response.code = 151001
        rospy.loginfo(f"Returning GoZeroResponse: {response.status}, {response.code}")
        return response

    def handle_block_arm_service(self,req):
        if self.command_timeout > 0:
            with self._command_lock:
                self.block_ctrl_flag = bool(req.data)
                if req.data:
                    self._stop_commands('arm blocked')
            return SetBoolResponse(success=True, message='Block updated; a latched stop cannot be unblocked.')
        response = SetBoolResponse()
        rospy.loginfo(f"-----------------------BLOCK_ARM---------------------------")
        if req.data:
            response.success = req.data
            response.message = "You will block arm ctrl msg send"
        else:
            response.success = req.data
            response.message = "You will unblock arm ctrl msg send"
        self.block_ctrl_flag = req.data
        rospy.loginfo(f"piper block arm .")
        rospy.loginfo(f"Returning BlockArmResponse: {response.success}, {response.message}")
        rospy.loginfo(f"-----------------------BLOCK_ARM---------------------------")
        return response
    
if __name__ == '__main__':
    try:
        piper_signle = C_PiperRosNode()
        piper_signle.Pubilsh()
    except rospy.ROSInterruptException:
        pass
