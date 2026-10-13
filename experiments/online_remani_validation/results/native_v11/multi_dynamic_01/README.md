# REMANI + MPPI 多动态障碍物实验

固定机械臂，原生 REMANI 在线规划/重规划、点云感知及底盘 MPPI，隔离 ROS 数值仿真。
沿用 V8 的障碍物运动设置，参考来自当前原生规划器；另加中心线迎面场景。
全部场景各运行一次；保留碰撞、超时和进程故障。带搜索记录的运行不作严格实时性能基准。

| 场景 | 障碍物数 | 结果 | 用时/s | 最小投影净距/m | 预测目标峰值 | 全局 GIF | 搜索放大 |
| --- | ---: | --- | ---: | ---: | ---: | --- | --- |
| 单障碍穿越 | 1 | goal | 21.44 | 0.126 | 1 | [全局](01_single/visualization/overview.gif) | [搜索](01_single/visualization/search_zoom.gif) |
| 双障碍交叉 | 2 | goal | 22.43 | 0.094 | 2 | [全局](02_two_crossing/visualization/overview.gif) | [搜索](02_two_crossing/visualization/search_zoom.gif) |
| 三障碍连续交会 | 3 | goal | 25.73 | 0.113 | 3 | [全局](03_three_sequential/visualization/overview.gif) | [搜索](03_three_sequential/visualization/search_zoom.gif) |
| 四障碍混合交会 | 4 | collision | 7.84 | -0.001 | 3 | [全局](04_four_dense/visualization/overview.gif) | [搜索](04_four_dense/visualization/search_zoom.gif) |
| 单独迎面交会 | 1 | collision | 13.12 | -0.002 | 1 | [全局](05_head_on_only/visualization/overview.gif) | [搜索](05_head_on_only/visualization/search_zoom.gif) |
| 四障碍镜像交会 | 4 | collision | 18.71 | -0.001 | 3 | [全局](06_four_mirrored/visualization/overview.gif) | [搜索](06_four_mirrored/visualization/search_zoom.gif) |
| 四障碍延后迎面 | 4 | collision | 7.78 | -0.000 | 3 | [全局](07_four_late/visualization/overview.gif) | [搜索](07_four_late/visualization/search_zoom.gif) |
| 中心线正迎面 | 1 | collision | 12.05 | -0.000 | 1 | [全局](08_head_on_center/visualization/overview.gif) | [搜索](08_head_on_center/visualization/search_zoom.gif) |

图例：橙色 REMANI 参考、蓝色已执行轨迹及车体投影、绿色获准局部展开。
浅蓝/浅红为内核可行/被拒候选，紫色为内核选中。灰色圆柱带场景身份编号；灰箭头使用实际感知估计的 CV 速度，最长展开 1.5 s。
搜索线表示中心展开；当前及获准展开使用底盘＋固定臂三圆盘外形进行碰撞检查。
场景由 scenario.json 声明；obstacles.csv 存全部目标的真值和身份，预测输入来自点云检测，未注入真值速度。
V8 的目标在运动触发后 2 s 出现并持续按恒速运动；部分从边界外进入或离开观测区域。
四障碍场景仅在隔离实验配置中把健康检查的运动目标数量上限从 3 调到 4，其他健康/稳定性条件仍保留。
新增场景按至多 5 ms 的底盘积分子步审核真值投影碰撞，发生碰撞立即终止。净距不包含额外安全余量。
当前构建与速度、车体、安全余量、预测有效期未调优。结果用于暴露多目标交会问题，不宣称所有场景通过或实车可部署。

