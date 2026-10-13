# REMANI + MPPI 多动态障碍物场景

使用当前 V11 构建 08，固定机械臂、差速底盘、三圆盘投影，接入真正的 REMANI 在线参考和重规划。
场景由 `dynamic_scenarios.py` 声明，沿用旧 V8 的圆柱障碍物位置、速度、尺寸及 2 s 出现时间。
出现时间以实测运动触发为原点；V8 对照的运动定义相同，但它使用固定参考和理想 CV，不能直接对比算法成绩。

| 单次启动的 scene | 批次目录 | 内容 |
| --- | --- | --- |
| dynamic_single | 01_single | 单障碍穿越 |
| dynamic_two | 02_two_crossing | 两个反向横穿目标 |
| dynamic_three | 03_three_sequential | 三个先后交会目标 |
| dynamic_four | 04_four_dense | 三个横穿目标＋一个迎面目标 |
| dynamic_head_on | 05_head_on_only | V8 的单独迎面，y=0.30 m |
| dynamic_four_mirrored | 06_four_mirrored | 四障碍上下镜像 |
| dynamic_four_late | 07_four_late | 迎面目标初始 x 再增加 0.30 m |
| dynamic_head_on_center | 08_head_on_center | 补充：迎面目标 y=0，正对中心线 |

障碍物实体半径为 0.20～0.23 m、速度为 0.17～0.26 m/s。每个目标始终按声明的恒速轨迹运动，
部分从场地边界外进入，随后离开场地；没有为了让机器人通过而按时间删除目标。
这些是合成感知的运动场景，圆柱轨迹不会响应机器人或场地墙壁，不是物理人群仿真。

## 数据链路

场景真值 → 合成雷达点云 → 当前动态跟踪器 → 同帧静态/动态碰撞输入 → REMANI 与 MPPI。
没有直接把真值目标速度、身份或轨迹提供给控制器。
真值仅用于生成点云、场景身份显示以及独立投影碰撞审核。

`obstacles.csv` 现在记录每帧所有目标的 `t,x,y,r,id,vx,vy`。
搜索记录保留求解时实际使用的检测目标与观测年龄，灰色箭头按这些感知估计显示 CV 展开，最长 1.5 s，
受当前 2 s 预测有效期限制。场景圆柱编号不是检测器的 track ID。

当前/未来车体、搜索候选、获准展开和真正的 REMANI 参考同时显示。
修正旧绘图器每帧只画最后一个障碍物的问题，使基本参考图和对照图也显示所有目标。

## 感知容量与验收

四目标场景的隔离配置将 `health_maximum_motion_tracks` 从 3 调整为 4。
这项参数原用于检测大范围点云/定位失配；数量上限必须与声明的测试目标数一致。
首次八组测试发现四个圆柱的下采样残差可达 808 点，超过继承的 800 点上限而关闭完整感知输出。
因此四目标配置还按 4/3 的容量比例将 `maximum_residual_points` 调到 1067；一到三个目标仍为 800。
首次结果完整保留，四目标配置复测另存到 `multi_dynamic_capacity_02`，没有更改控制算法来掩盖首次失败。
没有修改实车 launch，也没有解除观测新鲜度、健康状态恢复、固定臂检查、碰撞/制动复核或命令租约。
每个场景的实际覆盖参数都保存在 `parameters.yaml` 与 `manifest.json`。

扩展场景增加了至多 5 ms 积分子步的投影真值审核，遇到碰撞立即结束并保留失败 GIF。
原有 empty/static/crossing 的出现规则保留。控制内核二进制、速度、投影半径、安全余量和预测有效期保持本轮原值。

## 运行与图片

批量运行全部八组并生成分场景 GIF、审核和汇总（新目录）：

```bash
cd /home/l/remani_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 experiments/online_remani_validation/run_native_multi_dynamic.py \
  --out experiments/online_remani_validation/results/my_multi_dynamic
```

单独启动四障碍混合场景：

```bash
python3 experiments/online_remani_validation/run_native_v11.py \
  --scene dynamic_four --duration 52 \
  --out experiments/online_remani_validation/results/my_four_dynamic
```

批次内串行使用独立本机 ROS master；全部控制试验结束后再绘制动画，避免绘图和后续控制试验争用资源。
各场景 `visualization/overview.gif` 与 `search_zoom.gif` 分别为全局和搜索放大视图；
原始点云跟踪诊断、全部参考、实际执行复核和搜索快照独立归档。

本轮统一入口：[八组结果与 GIF](results/native_v11/multi_dynamic_report/README.md)。
首次八组：[multi_dynamic_01](results/native_v11/multi_dynamic_01/README.md)。
容量匹配后的四目标复测：[multi_dynamic_capacity_02](results/native_v11/multi_dynamic_capacity_02/README.md)。
每个场景只有一次运行，保留失败，不由这一批次推断统计成功率。
