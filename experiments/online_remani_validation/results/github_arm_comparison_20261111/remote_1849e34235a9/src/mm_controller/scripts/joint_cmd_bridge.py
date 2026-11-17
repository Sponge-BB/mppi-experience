#!/usr/bin/env python3
import rospy
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

class JointBridge:
    def __init__(self):
        rospy.init_node('joint_cmd_bridge')
        # 发布给 piper 的底层关节控制话题
        self.pub = rospy.Publisher('/joint_ctrl_single', JointState, queue_size=10)
        self.gripper_val = 0.0
        self._last_pub = rospy.Time(0)
        # 限制指令发送频率 ≤ 5Hz，防止 Piper SDK CAN 发送缓冲区溢出
        self._min_interval = rospy.Duration(0.2)
        self.clamp_joint6 = rospy.get_param('~clamp_joint6', True)
        self.joint6_min = rospy.get_param('~joint6_min', -0.35)
        self.joint6_max = rospy.get_param('~joint6_max', 0.35)

        # 订阅 mm_controller 输出的关节轨迹和夹爪状态
        rospy.Subscriber('/mm_controller_node/joint_cmd', JointTrajectoryControllerState, self.joint_cb)
        rospy.Subscriber('/mm_controller_node/gripper_cmd', Bool, self.gripper_cb)

    def gripper_cb(self, msg):
        # 夹爪限位，例如闭合=0.08，打开=0.0，数值根据 piper 需要调节
        self.gripper_val = 0.08 if msg.data else 0.0

    def joint_cb(self, msg):
        now = rospy.Time.now()
        if now - self._last_pub < self._min_interval:
            return  # 限速，丢掉多余的指令
        self._last_pub = now

        pub_msg = JointState()
        pub_msg.header = msg.header

        # 将关节的 6 自由度与 1 个夹爪自由度拼接发送给 piper_ctrl_single_node
        positions = list(msg.desired.positions)
        if self.clamp_joint6 and len(positions) >= 6:
            raw_joint6 = positions[5]
            positions[5] = max(self.joint6_min, min(self.joint6_max, raw_joint6))
            if abs(positions[5] - raw_joint6) > 1e-6:
                rospy.logwarn_throttle(
                    2.0,
                    "Clamp joint6 command %.3f -> %.3f rad to protect Piper wrist",
                    raw_joint6, positions[5])

        pub_msg.position = positions + [self.gripper_val]
        pub_msg.velocity = list(msg.desired.velocities) + [0.0]
        pub_msg.effort = list(msg.desired.effort) + [0.0]
        pub_msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]

        self.pub.publish(pub_msg)

if __name__ == '__main__':
    try:
        JointBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
