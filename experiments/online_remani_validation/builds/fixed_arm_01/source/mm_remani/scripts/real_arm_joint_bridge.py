#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState

class RealArmJointBridge:
    def __init__(self):
        rospy.init_node('real_arm_joint_bridge', anonymous=True)
        
        # 发布给整个系统（包含 TF 和 Planner 过滤器）的关节状态
        self.pub = rospy.Publisher('/gazebo/joint_states', JointState, queue_size=10)
        
        # 订阅实车 Piper 回传的真实状态
        rospy.Subscriber('/joint_states_single', JointState, self.joint_cb)

    def joint_cb(self, msg):
        out_msg = JointState()
        out_msg.header = msg.header
        
        # 遍历修改，将 'gripper' 适配到 URDF 里的 'joint7' 和 'joint8'
        for i, name in enumerate(msg.name):
            if name == "gripper":
                # 实车传出来的 gripper 值是 0 到 80000。0代表张开完全。
                val = msg.position[i]
                
                # URDF里要求 joint7/joint8 的量级通常是米的位移，
                # 上行发下去的是 0.0 ~ 0.08，所以回传需要换算。
                # 由于实车是乘以 1000*1000 变 80000 的，所以除回来：
                gripper_m = val / 1000000.0

                out_msg.name.append("joint7")
                out_msg.position.append(gripper_m)
                
                out_msg.name.append("joint8")
                out_msg.position.append(gripper_m)
            else:
                out_msg.name.append(name)
                out_msg.position.append(msg.position[i])

        self.pub.publish(out_msg)

if __name__ == '__main__':
    try:
        RealArmJointBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
