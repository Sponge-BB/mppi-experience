# 省重复计算：同输入与新闭环对照

本轮没有解决快速避障：静态两组超期停车，横穿两组到达但仍有停车，迎面两组接触。14份同输入回放通过，但关键重负载只省少量查询。详见下方直白总结。

先看[直白总结](实验总结.txt)和[在线对比图](online_comparison.png)。完整数据见[report.json](report.json)。

修改前与修改后都使用已有空旷提速、160个三维碰撞球与相同安全规则。每场一对种子11，仅故障复现。

| 场景 | 组别 | 结束原因 | 三维采样接触 | 完整回调P95/ms | 超期清零次数 |
|---|---|---|---|---:|---:|
| 静态障碍 | 修改前 | deadline_zero_deadlock | 未检测到 | 124.59 | 54 |
| 静态障碍 | 省重复计算后 | deadline_zero_deadlock | 未检测到 | 193.95 | 78 |
| 横穿障碍 | 修改前 | goal | 未检测到 | 120.71 | 103 |
| 横穿障碍 | 省重复计算后 | goal | 未检测到 | 119.96 | 94 |
| 迎面障碍 | 修改前 | native3d_sample_collision | 有 | 127.57 | 35 |
| 迎面障碍 | 省重复计算后 | native3d_sample_collision | 有 | 127.53 | 41 |

关键GIF：
- [静态障碍](gifs/static/comparison.gif)
- [横穿障碍](gifs/crossing/comparison.gif)
- [迎面障碍](gifs/dynamic_head_on_center/comparison.gif)

仅固定臂ROS数值闭环，非Gazebo或实车。无接触不等于成功，未实际遇到动态障碍不能评价避障。
