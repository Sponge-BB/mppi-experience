#!/usr/bin/env python3
import rospy
import sys
import math
import signal
import tkinter as tk
from tkinter import ttk
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

class JointToPoseGUI:
    """
    GUI: 输入关节弧度（参考 piper_ik_node.cpp 的限位），将关节值映射为 PoseStamped 并发布到 ~pose_topic（默认 /target_pose）。
    关键修改：
      - 启动时默认不发布（避免立刻触发 IK 失败）。
      - 提供 publish_on_start 参数（默认 False）以在需要时启动时发布一次安全默认位姿。
      - 在创建 UI 时抑制初始回调（self._initializing），避免 Scale 初始化触发发布。
      - 使用 piper 的关节限位作为滑块范围，且 load_defaults 支持从 ROS 参数导入默认关节角。
    映射规则（与 piper_ik_node 对接约定）：
      - position.x/y/z <- joint1/2/3
      - orientation <- joint4/5/6 (roll/pitch/yaw -> 四元数)
    """

    def __init__(self):
        rospy.init_node('joint_to_pose_gui', anonymous=True, disable_signals=True)

        # 参数
        self.joint_names = rospy.get_param('~joint_names',
                                          ['joint1','joint2','joint3','joint4','joint5','joint6'])
        self.pose_topic = rospy.get_param('~pose_topic', '/target_pose')
        self.reference_frame = rospy.get_param('~reference_frame', 'base_link')
        # 不在界面变化时自动发布（默认 False），避免启动时立刻发包
        self.publish_on_change = rospy.get_param('~publish_on_change', False)
        # 若需要启动即发布安全默认位姿（默认 False）
        self.publish_on_start = rospy.get_param('~publish_on_start', False)
        # 周期发布频率（仅当 publish_on_change=False 且 publish_on_start=False 时可选择开启周期发布）
        self.publish_rate = float(rospy.get_param('~publish_rate', 0.0))

        # 与 piper_ik_node.cpp 对齐的关节限位（弧度）
        self.joint_limits = {
            'joint1': (-2.618, 2.618),
            'joint2': (0.0, math.pi),
            'joint3': (-math.pi, 0.0),
            'joint4': (-2.967, 2.967),
            'joint5': (-1.2, 1.2),
            'joint6': (-1.22, 1.22)
        }

        # ROS 发布者
        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=10)

        # 状态变量
        self.var_list = []
        self.joint_vals = []
        self._initializing = True  # 在构建 UI 阶段抑制初始回调

        # 创建 GUI 根窗口（BooleanVar 等需要在 root 创建后）
        self.root = tk.Tk()
        self.root.title("Joint -> target_pose (piper)")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self._build_ui()

        # 结束初始化
        self._initializing = False

        # 若需要启动时发布一次安全默认位姿
        if self.publish_on_start:
            # load defaults if provided, else publish current mid values as safe default
            self.load_defaults()  # 尝试从参数加载默认
            self.publish_pose_once()
        else:
            # 若用户不希望自动发布，但又设置了周期发布频率 >0，则开启定时（可选）
            if not self.publish_on_change and self.publish_rate > 0.0:
                self._timer_id = None
                interval_ms = max(10, int(1000.0 / self.publish_rate))
                self._timer_id = self.root.after(interval_ms, self._periodic_publish)

        # 允许 Ctrl-C 退出
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            self.on_close()

    def _build_ui(self):
        frm = ttk.Frame(self.root, padding=8)
        frm.pack(fill='both', expand=True)

        ttk.Label(frm, text="输入关节角（弧度），发布为 PoseStamped 到: {}".format(self.pose_topic)).pack(anchor='w', pady=(0,6))

        # 先尝试读取参数中的默认角度，作为初始值
        defaults = rospy.get_param('~default_joint_angles', None)
        # 格式支持 dict 或 list
        default_list = None
        if isinstance(defaults, dict):
            default_list = [defaults.get(n, 0.0) for n in self.joint_names]
        elif isinstance(defaults, (list, tuple)):
            default_list = list(defaults)[:len(self.joint_names)]

        for i, name in enumerate(self.joint_names):
            mn, mx = self.joint_limits.get(name, (-math.pi, math.pi))
            if default_list is not None and i < len(default_list):
                mid = float(default_list[i])
                # clamp default into limits
                mid = max(mn, min(mx, mid))
            else:
                # 若没有默认则取限位中点或0
                if mn <= 0.0 <= mx:
                    mid = 0.0
                else:
                    mid = (mn + mx) / 2.0

            row = ttk.Frame(frm)
            row.pack(fill='x', pady=2)
            ttk.Label(row, text=name, width=12).pack(side='left')
            var = tk.DoubleVar(value=mid)
            # 创建 Scale 时不主动触发回调发布（在 _on_joint_change 中检查 self._initializing）
            scale = tk.Scale(row, variable=var, orient='horizontal',
                             from_=mn, to=mx, resolution=0.0001, length=360,
                             command=lambda v, idx=i: self._on_joint_change(idx, float(v)))
            scale.pack(side='left', fill='x', expand=True)
            self.var_list.append(var)
            self.joint_vals.append(float(mid))

        btn_row = ttk.Frame(frm)
        btn_row.pack(fill='x', pady=(8,0))
        ttk.Button(btn_row, text="Publish Once", command=self.publish_pose_once).pack(side='left', padx=4)
        ttk.Button(btn_row, text="Reset to defaults", command=self.load_defaults).pack(side='left', padx=4)
        ttk.Button(btn_row, text="Reset to 0", command=self.reset_all).pack(side='left', padx=4)

        # 状态行
        self.status = ttk.Label(frm, text="Ready (no auto-publish)", anchor='w')
        self.status.pack(fill='x', pady=(6,0))

    def _on_joint_change(self, idx, val):
        # 在初始化阶段不自动发布（避免刚启动即触发 IK）
        v = float(val)
        name = self.joint_names[idx]
        mn, mx = self.joint_limits.get(name, (-math.pi, math.pi))
        v = max(mn, min(mx, v))
        self.joint_vals[idx] = v

        if self._initializing:
            return
        if self.publish_on_change:
            self.publish_pose_once()

    def _clamped_angles(self):
        out = []
        for i, name in enumerate(self.joint_names):
            mn, mx = self.joint_limits.get(name, (-math.pi, math.pi))
            v = self.joint_vals[i] if i < len(self.joint_vals) else 0.0
            out.append(max(mn, min(mx, float(v))))
        return out

    def publish_pose_once(self):
        if rospy.is_shutdown():
            self.status.config(text="ROS shutdown")
            return
        angles = self._clamped_angles() + [0.0]*6
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = self.reference_frame

        # 映射：joint1/2/3 -> position x/y/z；joint4/5/6 -> roll/pitch/yaw
        ps.pose.position.x = angles[0] if len(angles) > 0 else 0.0
        ps.pose.position.y = angles[1] if len(angles) > 1 else 0.0
        ps.pose.position.z = angles[2] if len(angles) > 2 else 0.0

        r = angles[3] if len(angles) > 3 else 0.0
        p = angles[4] if len(angles) > 4 else 0.0
        y = angles[5] if len(angles) > 5 else 0.0
        q = quaternion_from_euler(r, p, y)
        ps.pose.orientation.x = q[0]
        ps.pose.orientation.y = q[1]
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]

        try:
            self.pose_pub.publish(ps)
            self.status.config(text="Published target_pose (mapped from joints)")
            rospy.logdebug("Published PoseStamped from joints: %s", angles[:6])
        except Exception as e:
            self.status.config(text="Publish failed: {}".format(e))

    def reset_all(self):
        for i, var in enumerate(self.var_list):
            var.set(0.0)
            self.joint_vals[i] = 0.0
        # 不自动发布，除非用户点击 Publish Once
        self.status.config(text="Reset to 0 (not published)")

    def load_defaults(self):
        defaults = rospy.get_param('~default_joint_angles', None)
        if defaults is None:
            # 若无参数则使用 joint_limits 提供的 mid 或 0
            for i, name in enumerate(self.joint_names):
                mn, mx = self.joint_limits.get(name, (-math.pi, math.pi))
                if mn <= 0.0 <= mx:
                    v = 0.0
                else:
                    v = (mn + mx) / 2.0
                self.var_list[i].set(v)
                self.joint_vals[i] = v
            self.status.config(text="Loaded default = mid of limits")
            return

        if isinstance(defaults, dict):
            for i, name in enumerate(self.joint_names):
                if name in defaults:
                    v = float(defaults[name])
                else:
                    mn, mx = self.joint_limits.get(name, (-math.pi, math.pi))
                    v = 0.0 if mn <= 0.0 <= mx else (mn + mx) / 2.0
                mn, mx = self.joint_limits.get(name, (-math.pi, math.pi))
                v = max(mn, min(mx, v))
                self.var_list[i].set(v)
                self.joint_vals[i] = v
        elif isinstance(defaults, (list, tuple)):
            for i, v in enumerate(defaults):
                if i < len(self.joint_names):
                    mn, mx = self.joint_limits.get(self.joint_names[i], (-math.pi, math.pi))
                    vv = max(mn, min(mx, float(v)))
                    self.var_list[i].set(vv)
                    self.joint_vals[i] = vv
        self.status.config(text="Loaded defaults from params")

    def _periodic_publish(self):
        # 用 after 定时发布（仅在用户显式开启 publish_rate 时运行）
        if not rospy.is_shutdown():
            self.publish_pose_once()
            interval_ms = max(10, int(1000.0 / self.publish_rate))
            self._timer_id = self.root.after(interval_ms, self._periodic_publish)

    def on_close(self):
        try:
            rospy.signal_shutdown("GUI closed")
        except Exception:
            pass
        try:
            if hasattr(self, '_timer_id') and self._timer_id:
                self.root.after_cancel(self._timer_id)
        except Exception:
            pass
        try:
            self.root.quit()
            self.root.destroy()
        except Exception:
            pass
        sys.exit(0)


if __name__ == '__main__':
    JointToPoseGUI()