#!/usr/bin/env python3
import rospy
import sys
import math
import signal
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QDoubleSpinBox, QPushButton, QScrollArea)
from PyQt5.QtCore import QTimer
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

class JointAngleGui(QMainWindow):
    def __init__(self):
        super().__init__()

        # 不让 rospy 拦截 SIGINT，由 Qt/进程处理
        rospy.init_node('joint_angle_gui', anonymous=True, disable_signals=True)

        # 参数（默认关节名与 C++ 节点一致）
        self.joint_names = rospy.get_param('~joint_names',
                                          ['joint1','joint2','joint3','joint4','joint5','joint6'])
        self.pose_topic = rospy.get_param('~pose_topic', '/target_pose')
        self.publish_rate = float(rospy.get_param('~publish_rate', 10.0))  # Hz

        # 仅保留姿态发布者
        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=10)

        # 关键改进1：对齐 C++ 节点的关节限位（弧度）
        self.joint_limits = {
            'joint1': (-2.618, 2.618),    # 对应 C++ joint_limits_[0]
            'joint2': (0.0, math.pi),     # 对应 C++ joint_limits_[1]（0~π）
            'joint3': (-math.pi, 0.0),    # 对应 C++ joint_limits_[2]（-π~0）
            'joint4': (-2.967, 2.967),    # 对应 C++ joint_limits_[3]
            'joint5': (-1.2, 1.2),        # 对应 C++ joint_limits_[4]
            'joint6': (-1.22, 1.22)       # 对应 C++ joint_limits_[5]
        }

        # 关键改进2：设置合理初始位置（非零位，避免碰撞，单位：弧度）
        # 初始姿态：机械臂伸直向前，关节2/3呈90°互补，其他关节归零
        self.default_joint_angles = {
            'joint1': 0.0,
            'joint2': math.pi/2,  # 90°
            'joint3': -math.pi/2, # -90°（与joint2互补，使手臂水平）
            'joint4': 0.0,
            'joint5': 0.0,
            'joint6': 0.0
        }
        # 初始化角度数据（优先使用默认初始值，无默认则归零）
        self.joint_angles = {}
        for name in self.joint_names:
            self.joint_angles[name] = self.default_joint_angles.get(name, 0.0)

        # UI（集成限位和初始位置）
        self.init_ui()

        # 定时发布
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.periodic_publish)
        self.timer.start(max(1, int(1000.0 / self.publish_rate)))

        # 日志输出限位和初始位置信息
        limit_info = ", ".join([f"{name}:[{lim[0]:.2f},{lim[1]:.2f}]" for name, lim in self.joint_limits.items()])
        init_info = ", ".join([f"{name}:{val:.2f}" for name, val in self.joint_angles.items()])
        rospy.loginfo("JointAngleGui initialized - 限位:%s | 初始位置:%s | pose_topic:%s | rate:%.1fHz",
                      limit_info, init_info, self.pose_topic, self.publish_rate)

    def init_ui(self):
        self.setWindowTitle("Joint Angle Controller (Aligned with C++ Node)")
        self.setGeometry(100, 100, 450, 500)

        central = QWidget()
        main_layout = QVBoxLayout(central)

        # 可滚动区域（支持任意数量关节）
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        container = QWidget()
        container_layout = QVBoxLayout(container)

        self.spinboxes = {}
        for name in self.joint_names:
            h = QHBoxLayout()
            lbl = QLabel(name)
            spin = QDoubleSpinBox()
            
            # 关键改进：设置自旋框限位（与 C++ 节点一致）
            min_lim = self.joint_limits.get(name, (-math.pi, math.pi))[0]
            max_lim = self.joint_limits.get(name, (-math.pi, math.pi))[1]
            spin.setRange(min_lim, max_lim)
            
            spin.setSingleStep(0.01)  # 步长0.01弧度（≈0.57°）
            spin.setDecimals(4)       # 显示4位小数，精度更高
            spin.setValue(self.joint_angles[name])  # 加载初始位置
            spin.valueChanged.connect(self.make_on_change(name))
            self.spinboxes[name] = spin

            h.addWidget(lbl, 1)
            h.addWidget(spin, 3)
            container_layout.addLayout(h)

        container.setLayout(container_layout)
        scroll.setWidget(container)
        main_layout.addWidget(scroll)

        # 控制按钮（重置按钮改为恢复到默认初始位置，而非归零）
        btn_layout = QHBoxLayout()
        pub_btn = QPushButton("立即发布姿态")
        pub_btn.clicked.connect(self.publish_once)
        btn_layout.addWidget(pub_btn)

        reset_btn = QPushButton("恢复初始位置")  # 按钮文本修改
        reset_btn.clicked.connect(self.reset_to_default)  # 绑定新的重置逻辑
        btn_layout.addWidget(reset_btn)

        main_layout.addLayout(btn_layout)

        self.setCentralWidget(central)

    def make_on_change(self, joint_name):
        def on_change(val):
            self.joint_angles[joint_name] = float(val)
        return on_change

    def publish_pose_stamped(self):
        """将当前关节值映射为 PoseStamped 并发布（逻辑不变）"""
        if rospy.is_shutdown():
            return
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = rospy.get_param('~reference_frame', 'base_link')

        # position 映射：前3个关节 -> x,y,z
        angles = [self.joint_angles.get(n, 0.0) for n in self.joint_names]
        px = angles[0] if len(angles) > 0 else 0.0
        py = angles[1] if len(angles) > 1 else 0.0
        pz = angles[2] if len(angles) > 2 else 0.0
        pose.pose.position.x = px
        pose.pose.position.y = py
        pose.pose.position.z = pz

        # orientation 映射：joint3/joint4/joint5 -> roll/pitch/yaw
        r = angles[3] if len(angles) > 3 else 0.0
        p = angles[4] if len(angles) > 4 else 0.0
        y = angles[5] if len(angles) > 5 else 0.0
        q = quaternion_from_euler(r, p, y)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.pose_pub.publish(pose)

    def periodic_publish(self):
        self.publish_pose_stamped()

    def publish_once(self):
        self.publish_pose_stamped()

    def reset_to_default(self):
        """关键改进：重置为默认初始位置（而非归零）"""
        for name in self.joint_names:
            default_val = self.default_joint_angles.get(name, 0.0)
            self.spinboxes[name].setValue(default_val)
            self.joint_angles[name] = default_val
        rospy.loginfo("Reset to default joint angles: %s", 
                      ", ".join([f"{name}:{val:.2f}" for name, val in self.joint_angles.items()]))

    def closeEvent(self, event):
        self.timer.stop()
        event.accept()

if __name__ == '__main__':
    # 允许 Ctrl-C 正常退出
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QApplication(sys.argv)
    gui = JointAngleGui()
    gui.show()
    sys.exit(app.exec_())