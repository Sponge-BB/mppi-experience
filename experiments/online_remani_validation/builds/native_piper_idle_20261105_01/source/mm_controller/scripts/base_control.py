#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import struct
import threading
from geometry_msgs.msg import Twist

class CmdVelToUartNode:
    def __init__(self):
        # 1. 初始化 ROS 节点
        rospy.init_node('cmd_vel_to_uart_node', anonymous=True)

        # 2. 获取参数
        self.port_name = rospy.get_param('~port', '/dev/ttyUSB_base')
        self.baud_rate = rospy.get_param('~baud', 115200)

        # 3. 尝试打开串口
        try:
            self.ser = serial.Serial(self.port_name, self.baud_rate, timeout=0.5)
            rospy.loginfo("成功打开串口: %s, 波特率: %d", self.port_name, self.baud_rate)
        except serial.SerialException as e:
            rospy.logerr("无法打开串口 %s ! 请检查权限或接线。错误信息: %s", self.port_name, e)
            rospy.signal_shutdown("串口打开失败")
            return

        # 4. 注册节点关闭时的回调函数（用于安全刹车）
        rospy.on_shutdown(self.shutdown_hook)

        # 5. 订阅 /cmd_vel 话题 (发送给单片机)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.loginfo("开始订阅 /cmd_vel 话题，准备下发控制指令...")

        # 6. 新增：发布 /my_car_vel 话题 (从单片机接收)
        self.vel_pub = rospy.Publisher('/my_car_vel', Twist, queue_size=10)
        rospy.loginfo("开始发布 /my_car_vel 话题，提供底盘实际速度反馈...")

        # 7. 新增：开启一个独立线程，专门用于循环读取串口数据
        self.is_running = True
        self.read_thread = threading.Thread(target=self.serial_read_loop)
        self.read_thread.daemon = True # 设置为守护线程，主线程退出时自动销毁
        self.read_thread.start()

    def cmd_vel_callback(self, msg):
        """
        当接收到 /cmd_vel 消息时触发此回调 (ROS -> STM32)
        """
        vx = msg.linear.x
        vy = msg.linear.y
        vw = msg.angular.z

        try:
            # 打包下发数据: 帧头 0xAA, 3个 float, 帧尾 0x55
            pack_data = struct.pack('<B3fB', 0xAA, vx, vy, vw, 0x55)
            self.ser.write(pack_data)
        except Exception as e:
            rospy.logerr("串口发送失败: %s", e)

    def serial_read_loop(self):
        """
        独立线程：不断读取串口数据并解析 (STM32 -> ROS)
        """
        while self.is_running and not rospy.is_shutdown():
            try:
                # 至少要有 14 个字节（1帧的长度）才开始处理
                if self.ser.in_waiting >= 14:
                    # 寻找帧头 0xAB (Python3 中读取的 1 字节前缀是 b'')
                    head = self.ser.read(1)
                    
                    if head == b'\xAB':
                        # 如果帧头对上了，连续读取剩下的 13 个字节（12字节数据 + 1字节帧尾）
                        payload = self.ser.read(13)
                        
                        # 检查长度和帧尾 0xBA (Python3 对 bytes 索引会得到 int，所以用 0xBA)
                        if len(payload) == 13 and payload[12] == 0xBA:
                            
                            # 解析出 3 个 float (<3f 代表按小端序解析 3 个 float)
                            # payload[0:12] 就是那 12 个纯数据字节
                            feedback_vx, feedback_vy, feedback_vw = struct.unpack('<3f', payload[0:12])
                            
                            # 组装成 ROS 的 Twist 消息发布出去
                            fdb_msg = Twist()
                            fdb_msg.linear.x = feedback_vx
                            fdb_msg.linear.y = feedback_vy
                            fdb_msg.angular.z = feedback_vw
                            
                            self.vel_pub.publish(fdb_msg)
                        else:
                            # 帧尾不对，说明发生了数据错位，清空缓冲区重新找帧头
                            self.ser.reset_input_buffer()
            except Exception as e:
                rospy.logwarn_throttle(2, "串口读取异常: %s", e)

    def shutdown_hook(self):
        """
        节点关闭时的安全机制
        """
        rospy.logwarn("节点即将关闭，下发急刹车指令以保护底盘...")
        self.is_running = False # 停止读取线程
        
        try:
            pack_data = struct.pack('<B3fB', 0xAA, 0.0, 0.0, 0.0, 0x55)
            self.ser.write(pack_data)
            self.ser.close()
            rospy.loginfo("串口已安全关闭。")
        except Exception as e:
            rospy.logerr("关闭串口时发生异常: %s", e)

if __name__ == '__main__':
    try:
        node = CmdVelToUartNode()
        rospy.spin() # 阻塞主线程，保持节点存活；读取工作由子线程完成
    except rospy.ROSInterruptException:
        pass