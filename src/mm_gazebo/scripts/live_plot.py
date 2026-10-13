#!/usr/bin/env python3
"""
Live plot of /tracking_error for trajectory tracking evaluation.

Error definitions (computed in gazebo_base_tracker.control_loop):
  data[0] dist = sqrt(ex_global^2 + ey_global^2)
      全局空间距离误差, 期望位置与当前位置的欧氏距离 (始终 >= 0)
  data[1] ex   = 前向误差 (沿机器人前进方向)
      cos(yaw)*(x_des-x) + sin(yaw)*(y_des-y)
      >0: 机器人落后于期望轨迹
      <0: 机器人超前于期望轨迹
  data[2] ey   = 侧向误差 (垂直于机器人前进方向, 左正右负)
      -sin(yaw)*(x_des-x) + cos(yaw)*(y_des-y)
      >0: 期望轨迹在机器人左侧 → 转弯时外甩表现为正偏
      <0: 期望轨迹在机器人右侧
  data[3] eyaw = 期望航向 - 当前航向 (rad)
      >0: 应逆时针转
      <0: 应顺时针转
"""
import rospy
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from std_msgs.msg import Float64MultiArray
from collections import deque

WINDOW = 500  # points in history window


class LivePlot:
    def __init__(self):
        self.t_data = deque(maxlen=WINDOW)
        self.dist = deque(maxlen=WINDOW)
        self.ex = deque(maxlen=WINDOW)
        self.ey = deque(maxlen=WINDOW)
        self.eyaw = deque(maxlen=WINDOW)
        self.t0 = None

        rospy.init_node('live_plot', anonymous=True)
        rospy.Subscriber('/tracking_error', Float64MultiArray, self.cb)

        plt.ion()
        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = \
            plt.subplots(2, 2, figsize=(14, 9))

        (self.ln1,) = self.ax1.plot([], [], 'r-', lw=1)
        (self.ln2,) = self.ax2.plot([], [], 'b-', lw=1)
        (self.ln3,) = self.ax3.plot([], [], 'g-', lw=1.5)
        (self.ln4,) = self.ax4.plot([], [], 'm-', lw=1)

        # ---- Subplot 1: 全局距离误差 ----
        self.ax1.set_title('global position error dist (m)\n'
                           '期望位置与当前位置的欧氏距离')
        self.ax1.set_ylim(0, 1.5)
        self.ax1.axhline(0.15, color='gray', ls='--', lw=0.5,
                         label='stop threshold 0.15m')
        self.ax1.legend(loc='upper right', fontsize=7)

        # ---- Subplot 2: 前向误差 ----
        self.ax2.set_title('longitudinal error ex (m)\n'
                           '沿机器人前进方向, +退后 -超前')
        self.ax2.set_ylim(-0.6, 0.6)
        self.ax2.axhline(0, color='gray', ls='--', lw=0.5)

        # ---- Subplot 3: 侧向误差 ----
        self.ax3.set_title('lateral error ey (m)  ← 转弯外甩看这里\n'
                           '垂直机器人方向, +左 -右, 转弯时正则=向外甩')
        self.ax3.set_ylim(-0.6, 0.6)
        self.ax3.axhline(0, color='gray', ls='--', lw=0.5)

        # ---- Subplot 4: 航向误差 ----
        self.ax4.set_title('heading error eyaw (rad)\n'
                           '期望航向 - 当前航向, +逆时针 -顺时针')
        self.ax4.set_ylim(-0.4, 0.4)
        self.ax4.axhline(0, color='gray', ls='--', lw=0.5)

        for ax in (self.ax1, self.ax2, self.ax3, self.ax4):
            ax.set_xlabel('time (s)')
            ax.grid(True, alpha=0.3)

        plt.tight_layout()

    def cb(self, msg):
        if self.t0 is None:
            self.t0 = rospy.Time.now().to_sec()
        t = rospy.Time.now().to_sec() - self.t0

        self.t_data.append(t)
        self.dist.append(msg.data[0])
        self.ex.append(msg.data[1])
        self.ey.append(msg.data[2])
        self.eyaw.append(msg.data[3])

    def update(self):
        if not self.t_data:
            return
        t = list(self.t_data)
        self.ln1.set_data(t, list(self.dist))
        self.ln2.set_data(t, list(self.ex))
        self.ln3.set_data(t, list(self.ey))
        self.ln4.set_data(t, list(self.eyaw))
        for ax in (self.ax1, self.ax2, self.ax3, self.ax4):
            if t:
                ax.set_xlim(max(0, t[-1] - 30), max(30, t[-1]))
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()
        plt.close()


if __name__ == '__main__':
    try:
        lp = LivePlot()
        lp.run()
    except rospy.ROSInterruptException:
        pass
