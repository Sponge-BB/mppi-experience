#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import tf
import math
import tkinter as tk
from tkinter import ttk

class InteractivePoseMarkerGUI:
    def __init__(self):
        rospy.init_node('interactive_pose_marker_gui', anonymous=True, disable_signals=True)

        # 参数
        self.marker_name = rospy.get_param('~marker_name', 'target_pose')
        self.reference_frame = rospy.get_param('~reference_frame', 'base_link')
        self.pose_topic = rospy.get_param('~pose_topic', '/target_pose')
        self.joint_topic = rospy.get_param('~joint_topic', '/joint_states')

        # 发布者
        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=5)
        self.joint_pub = rospy.Publisher(self.joint_topic, JointState, queue_size=5)
        self.tf_broadcaster = tf.TransformBroadcaster()

        # 初始值
        self.pose_vals = {'x':0.5,'y':0.0,'z':0.5,'roll':0.0,'pitch':0.0,'yaw':0.0}
        self.joint_names = ["joint1","joint2","joint3","joint4","joint5","joint6","joint7","joint8"]
        self.joint_vals = [0.0]*8

        # 创建 GUI（先创建窗口）
        self.root = tk.Tk()
        self.root.title("Piper Control GUI")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # 在窗口创建后才创建 BooleanVar
        self.gripper_mirror = tk.BooleanVar(value=True)

        nb = ttk.Notebook(self.root)
        nb.pack(fill='both', expand=True, padx=6, pady=6)

        # Pose tab
        pose_frame = ttk.Frame(nb)
        nb.add(pose_frame, text="Pose Control")
        self._build_pose_tab(pose_frame)

        # Joint tab
        joint_frame = ttk.Frame(nb)
        nb.add(joint_frame, text="Joint Control")
        self._build_joint_tab(joint_frame)

        # 启动 tkinter 主循环
        self.root.mainloop()

    def _build_pose_tab(self, frame):
        labels = [('X (m)', 'x', -1.0, 1.0, 0.01),
                  ('Y (m)', 'y', -1.0, 1.0, 0.01),
                  ('Z (m)', 'z', 0.0, 1.5, 0.01),
                  ('Roll (rad)', 'roll', -math.pi, math.pi, 0.01),
                  ('Pitch (rad)', 'pitch', -math.pi/2, math.pi/2, 0.01),
                  ('Yaw (rad)', 'yaw', -math.pi, math.pi, 0.01)]
        row = 0
        self.pose_sliders = {}
        for text, key, mn, mx, step in labels:
            ttk.Label(frame, text=text).grid(column=0, row=row, sticky='w')
            sv = tk.DoubleVar(value=self.pose_vals[key])
            s = tk.Scale(frame, variable=sv, orient='horizontal', from_=mn, to=mx, resolution=step, length=300,
                         command=lambda val, k=key: self._on_pose_change(k, float(val)))
            s.grid(column=1, row=row, padx=4, pady=2)
            self.pose_sliders[key] = sv
            row += 1

        btn_frame = ttk.Frame(frame)
        btn_frame.grid(column=0, row=row, columnspan=2, pady=8)
        ttk.Button(btn_frame, text="Publish Pose", command=self.publish_pose_once).pack(side='left', padx=6)
        ttk.Button(btn_frame, text="Publish & Broadcast TF", command=self.publish_pose_and_tf).pack(side='left', padx=6)

    def _build_joint_tab(self, frame):
        self.joint_sliders = []
        for i in range(8):
            ttk.Label(frame, text=self.joint_names[i]).grid(column=0, row=i, sticky='w')
            sv = tk.DoubleVar(value=0.0)
            s = tk.Scale(frame, variable=sv, orient='horizontal', from_=-3.14, to=3.14, resolution=0.001, length=300,
                         command=lambda val, idx=i: self._on_joint_change(idx, float(val)))
            s.grid(column=1, row=i, padx=4, pady=2)
            self.joint_sliders.append(sv)
        chk = ttk.Checkbutton(frame, text="Mirror gripper (joint8 = -joint7)", variable=self.gripper_mirror)
        chk.grid(column=0, row=9, columnspan=2, pady=6)
        btn_frame = ttk.Frame(frame)
        btn_frame.grid(column=0, row=10, columnspan=2, pady=8)
        ttk.Button(btn_frame, text="Publish JointState", command=self.publish_joint_once).pack(side='left', padx=6)

    def _on_pose_change(self, key, val):
        self.pose_vals[key] = val
        self.publish_pose_once()

    def _on_joint_change(self, idx, val):
        self.joint_vals[idx] = val
        if idx == 6 and self.gripper_mirror.get():
            self.joint_vals[7] = -val
            self.joint_sliders[7].set(-val)
        if idx == 7 and self.gripper_mirror.get():
            self.joint_vals[6] = -val
            self.joint_sliders[6].set(-val)
        self.publish_joint_once()

    def publish_pose_once(self):
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = self.reference_frame
        ps.pose.position.x = self.pose_vals['x']
        ps.pose.position.y = self.pose_vals['y']
        ps.pose.position.z = self.pose_vals['z']
        q = tf.transformations.quaternion_from_euler(self.pose_vals['roll'],
                                                     self.pose_vals['pitch'],
                                                     self.pose_vals['yaw'])
        ps.pose.orientation.x = q[0]
        ps.pose.orientation.y = q[1]
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]
        try:
            self.pose_pub.publish(ps)
        except rospy.ROSInterruptException:
            pass

    def publish_pose_and_tf(self):
        self.publish_pose_once()
        try:
            q = tf.transformations.quaternion_from_euler(self.pose_vals['roll'],
                                                         self.pose_vals['pitch'],
                                                         self.pose_vals['yaw'])
            self.tf_broadcaster.sendTransform(
                (self.pose_vals['x'], self.pose_vals['y'], self.pose_vals['z']),
                (q[0], q[1], q[2], q[3]),
                rospy.Time.now(),
                self.marker_name,
                self.reference_frame)
        except rospy.ROSInterruptException:
            pass

    def publish_joint_once(self):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = self.joint_names
        js.position = [float(v) for v in self.joint_vals]
        try:
            self.joint_pub.publish(js)
        except rospy.ROSInterruptException:
            pass

    def on_close(self):
        try:
            rospy.signal_shutdown("GUI closed")
        except Exception:
            pass
        self.root.destroy()

if __name__ == '__main__':
    try:
        InteractivePoseMarkerGUI()
    except Exception as e:
        print("GUI exit:", e)