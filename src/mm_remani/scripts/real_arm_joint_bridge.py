#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from sensor_msgs.msg import JointState

class RealArmJointBridge:
    def __init__(self):
        rospy.init_node('real_arm_joint_bridge', anonymous=True)
        
        # 发布给整个系统（包含 TF 和 Planner 过滤器）的关节状态
        self.pub = rospy.Publisher('/gazebo/joint_states', JointState, queue_size=10)
        
        # 订阅实车 Piper 回传的真实状态
        rospy.Subscriber('/joint_states_single', JointState, self.joint_cb)

    def joint_cb(self, msg):
        if (len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name) or
                not set('joint%d' % i for i in range(1, 7)).issubset(msg.name) or
                not all(math.isfinite(q) for q in msg.position)):
            rospy.logwarn_throttle(2.0, 'Drop incomplete/invalid Piper joint feedback.')
            return
        out_msg = JointState()
        out_msg.header = msg.header
        have_velocity = (len(msg.velocity) == len(msg.name) and
                         all(math.isfinite(v) for v in msg.velocity))
        have_effort = (len(msg.effort) == len(msg.name) and
                       all(math.isfinite(v) for v in msg.effort))
        
        # 遍历修改，将 'gripper' 适配到 URDF 里的 'joint7' 和 'joint8'
        for i, name in enumerate(msg.name):
            if name == "gripper":
                # Piper JointState is already in metres; do not convert twice.
                gripper_m = msg.position[i]

                out_msg.name.append("joint7")
                out_msg.position.append(gripper_m)
                
                out_msg.name.append("joint8")
                out_msg.position.append(gripper_m)
                if have_velocity:
                    out_msg.velocity.extend([msg.velocity[i]] * 2)
                if have_effort:
                    out_msg.effort.extend([msg.effort[i]] * 2)
            else:
                out_msg.name.append(name)
                out_msg.position.append(msg.position[i])
                if have_velocity:
                    out_msg.velocity.append(msg.velocity[i])
                if have_effort:
                    out_msg.effort.append(msg.effort[i])

        self.pub.publish(out_msg)

if __name__ == '__main__':
    try:
        RealArmJointBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
