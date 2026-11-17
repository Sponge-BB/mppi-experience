#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
import math
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros

class RealOdomPublisher:
    def __init__(self):
        rospy.init_node('real_odom_publisher', anonymous=True)
        
        # 里程计积分累计值初始化
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        # 记录上一次收到速度的时间，用于计算 dt
        self.last_time = rospy.Time.now()
        
        # 准备发布 TF 和 Odom 面向底盘的话题
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # 订阅实车传上来的速度话题 (假设你的底层发上来的数据类型是 geometry_msgs/Twist)
        # 如果是别的类型，需要你根据实际情况在这里转换
        rospy.Subscriber('my_car_vel', Twist, self.vel_callback)

        # 订阅指令速度，用于拐弯时替代不可靠的 STM32 vth 反馈
        # 注意：cmd_vw 有过期保护（0.3s 内未收到新指令则回退到实测值）
        self.cmd_vx = 0.0
        self.cmd_vw = 0.0
        self.cmd_vel_stamp = rospy.Time(0)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_cb)

        rospy.loginfo("实车里程计(Odom)计算节点已启动... 正在监听 my_car_vel + cmd_vel 话题")

    def cmd_vel_cb(self, msg):
        self.cmd_vx = msg.linear.x
        self.cmd_vw = msg.angular.z
        self.cmd_vel_stamp = rospy.Time.now()

    def vel_callback(self, msg):
        current_time = rospy.Time.now()

        # 计算距离上次收到消息经过了多少秒
        dt = (current_time - self.last_time).to_sec()
        if dt > 1.0 or dt <= 0.0: # 防止第一次启动时时间差过大或时间戳重复
            self.last_time = current_time
            return

        # 提取实车当前线速度(vx, vy)和角速度(vth)
        vx = msg.linear.x
        vy = msg.linear.y
        vth = msg.angular.z

        # ==============================================================
        # 核心算式：航位推算(Dead Reckoning)
        # ==============================================================
        # vx: 使用 STM32 实测值（比指令值更准，能反映真实位移）
        # vy: 忽略。麦克纳姆轮 vy 来自轮速逆解，地面打滑时严重不可靠
        # vth: 拐弯时用指令值（避免 STM32 增益误差-24%），
        #      直走时用实测值加小死区过滤偏置噪声。
        #      关键：cmd_vw 有 0.3s 超时保护——如果 tracker 停止发指令
        #      （例如从拐弯切换到直走），立即回退到实测值，防止持续积分
        # ==============================================================
        cmd_vw_age = (rospy.Time.now() - self.cmd_vel_stamp).to_sec() if self.cmd_vel_stamp.to_sec() > 0 else 99.0
        if abs(self.cmd_vw) > 0.05 and cmd_vw_age < 0.3:
            # Intentional rotation with fresh command: use commanded vw
            vth_for_odom = self.cmd_vw
        else:
            # Straight / stationary / stale command: use measured vth with deadband
            vth_for_odom = vth if abs(vth) > 0.03 else 0.0

        delta_x = vx * math.cos(self.th) * dt
        delta_y = vx * math.sin(self.th) * dt
        delta_th = vth_for_odom * dt
        
        # 累加到全局累计状态
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        
        # 将偏航角(Yaw)转换为四元数(Quaternion)用于ROS的坐标系发布
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        
        # ---------------------------------------------------------------
        # 1. 广播动态 TF 坐标变换 (map/odom -> base_link)
        # RViz 和避障规划主要靠这个知道小车在哪里
        # ---------------------------------------------------------------
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        # 你的底盘在Gazebo里的z高度补偿是0.115，实车也应保持一致，防止轮子陷进地里
        t.transform.translation.z = 0.115 
        t.transform.rotation.x = odom_quat[0]
        t.transform.rotation.y = odom_quat[1]
        t.transform.rotation.z = odom_quat[2]
        t.transform.rotation.w = odom_quat[3]
        self.tf_broadcaster.sendTransform(t)
        
        # ---------------------------------------------------------------
        # 2. 发布 Odom 数据结构话题包
        # 一些导航包不仅需要TF，还需要底盘的速度信息，所以我们封装出标准的Odom结构发出去
        # ---------------------------------------------------------------
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        
        # 填入位置信息
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.115
        odom.pose.pose.orientation = Quaternion(*odom_quat)
        
        # 填入底盘产生的实际速度信息，子坐标系填自身
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth_for_odom
        
        # 将构造好的odom发出去
        self.odom_pub.publish(odom)
        
        # 更新时间戳以供下一次积分计算使用
        self.last_time = current_time

if __name__ == '__main__':
    try:
        RealOdomPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
