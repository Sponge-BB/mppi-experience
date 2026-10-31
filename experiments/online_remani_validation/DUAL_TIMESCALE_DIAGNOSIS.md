# MPPI 执行链双时间尺度定位实验

日期：2026-10-18，Asia/Shanghai。

## 本轮问题

1. 原生 REMANI 的参考刷新在何处等待：触发、地图就绪、搜索、优化、验收、重试或交付？
2. 不再更新参考时，现有 MPPI 能否在安全条件下从侧偏和绕障中回到参考？
3. 若前两项允许解释，再测试仅增加参考交付延迟对交接的影响。

固定机械臂、差速底盘、完整车体/固定臂投影及原有约束。保留当前 V12 算法，不调代价权重，
不放宽轮速，不启动硬件，不把原 Tracker 的冻结死锁套用到 MPPI 执行组。

## 完整在线实验

`build_dual_timescale_probe.py` 仅生成独立实验构建；`run_dual_timescale.py` 复用已有
真实原生规划器、检测器和 MPPI。localhost:11441，唯一速度发布者应为 `/base_mppi`，
话题 `/sim/cmd_vel`；`/cmd_vel` 不得有发布者。感知输入为合成点云而非真实雷达。

同时保存原生求解事件、ROS 接收/采用事件、原始参考、执行状态、停止原因和 CPU 采样。
正常运行关闭细分代价计时；沿用诊断构建中的候选扫描开销必须在构建清单中声明。
在线运行不与编译、离线控制批次或 GIF 渲染并发。

在线重复使用相同场景设置，但原生搜索继承 `random_device`，感知接收时序也会变化。
所以重复在线运行不是严格相同输入的因果配对，不把两次运行的差异全归于延迟。
区分请求嵌套时长，不能把搜索、优化和总请求时间直接重复相加。

## 独立回线实验

参考来自已经验收的完整原生 REMANI 多项式，不是手绘路径。
复用当前 MPPI 库、实际数值底盘积分和发布前候选/制动检查；不经过 Tracker。
通过正常起步、正负侧偏、可绕过静态障碍隔离局部能力。

这类固定参考实验不是历史 ROS 通信与感知时间重放，也不是完整在线导航优劣对照。
离线场景若使用预设障碍输入，必须明确其绕过了真实检测器，不能据此证明感知可靠。
无障碍回线失败可定位局部执行问题；不能由此直接宣称全局重规划初始化是唯一原因。

## 统计与可视化

- 误差按当前有效参考的连续路段计算，不把不同参考版本的误差混为一条曲线。
- 障碍存在时的允许绕行偏差与障碍清除后的残差分开；不强迫返回被占据的原路段。
- 同时记录停车原因、完整投影净距、轮速边界、截止时间失败与回线持续性。
- GIF 仅离线绘制实际记录；新参考不能提前出现，局部轨迹不能在失效后继续冒充有效。
- 每个实验单独保存 GIF，统一颜色、尺度，标注数值仿真与播放倍率。
- 计时记录使用单调墙钟与 ROS 时刻，不能用客户端日期替代实际运行耗时。

## 实车和后续改进边界

本轮不修改实车 launch、控制律或配置，不自动关机。
若 MPPI 独立回线仍失败，先查局部行为，不立即叠加新初始化与其他算法。
MPPI 绕行初值属于后续算法改动，应在固定外部规划问题下另做对照：相同状态、目标、
地图及预测时刻、相同约束与预算，只改变初始化；搜索和优化收益分开统计。
算法仿真结论不替代 Gazebo、硬件时延标定和低速实车安全验收。

## 正式结果与复现入口

正式数据位于`results/dual_timescale_20261018/`，详见其中TXT、README和GIF_INDEX。
本轮冻结V12不等于`shared_collision`等后续候选的全部实现，后续候选须按各自库哈希另测。

```bash
source /opt/ros/noetic/setup.bash
source /home/l/remani_ws/devel/setup.bash
python3 experiments/online_remani_validation/build_dual_timescale_probe.py --out /tmp/new_dual_build
python3 experiments/online_remani_validation/run_dual_timescale.py --build /tmp/new_dual_build --out /tmp/new_dual_online
python3 experiments/online_remani_validation/run_mppi_rejoin_diagnosis.py --out /tmp/new_mppi_rejoin
```

输出目录必须不存在；在线运行需隔离本机ROS网络权限。绘图在所有计时结束后执行。
生产代码不修改，没有自动关机或自动上传行为。
