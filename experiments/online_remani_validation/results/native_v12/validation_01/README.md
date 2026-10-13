# V12 在线验证与对照

原生 REMANI 在线规划/重规划 + 固定臂底盘 MPPI，隔离 ROS 数值仿真。
主场景均3s有效期，另做2/4s时域、仅改感知、重复运行和参考交付延迟检查。
全部计划运行和失败均保留；不同在线运行的原生搜索与调度存在差异，不能当作统计优势。

| 试验 | 结果 | 运动用时/s | 最小净距/m | 首次CV输入/s | 内核P95/ms | 全局GIF | 搜索GIF |
| --- | --- | ---: | ---: | ---: | ---: | --- | --- |
| 01_single_h3 | collision | 10.57 | -0.002 | 2.47 | 19.9 | [全局](01_single_h3/visualization/overview.gif) | [搜索](01_single_h3/visualization/search_zoom.gif) |
| 02_two_crossing_h3 | collision | 10.58 | -0.001 | 2.45 | 19.8 | [全局](02_two_crossing_h3/visualization/overview.gif) | [搜索](02_two_crossing_h3/visualization/search_zoom.gif) |
| 03_three_sequential_h3 | collision | 10.77 | -0.002 | 2.46 | 19.6 | [全局](03_three_sequential_h3/visualization/overview.gif) | [搜索](03_three_sequential_h3/visualization/search_zoom.gif) |
| 04_four_dense_h3 | collision | 10.52 | -0.002 | 2.41 | 20.2 | [全局](04_four_dense_h3/visualization/overview.gif) | [搜索](04_four_dense_h3/visualization/search_zoom.gif) |
| 05_head_on_only_h3 | timeout | 50.19 | 0.292 | 3.45 | 18.2 | [全局](05_head_on_only_h3/visualization/overview.gif) | [搜索](05_head_on_only_h3/visualization/search_zoom.gif) |
| 06_four_mirrored_h3 | collision | 12.19 | -0.000 | 2.39 | 19.7 | [全局](06_four_mirrored_h3/visualization/overview.gif) | [搜索](06_four_mirrored_h3/visualization/search_zoom.gif) |
| 07_four_late_h3 | collision | 10.93 | -0.003 | 2.43 | 19.1 | [全局](07_four_late_h3/visualization/overview.gif) | [搜索](07_four_late_h3/visualization/search_zoom.gif) |
| 08_head_on_center_h3 | timeout | 50.26 | 0.244 | 4.45 | 18.2 | [全局](08_head_on_center_h3/visualization/overview.gif) | [搜索](08_head_on_center_h3/visualization/search_zoom.gif) |
| empty_h3 | goal | 16.18 | 1.095 | — | 22.1 | [全局](empty_h3/visualization/overview.gif) | [搜索](empty_h3/visualization/search_zoom.gif) |
| static_h3 | goal | 26.82 | 0.134 | — | 31.8 | [全局](static_h3/visualization/overview.gif) | [搜索](static_h3/visualization/search_zoom.gif) |
| center_h2 | goal | 28.93 | 0.244 | 3.36 | 18.7 | [全局](center_h2/visualization/overview.gif) | [搜索](center_h2/visualization/search_zoom.gif) |
| center_h4 | timeout | 50.24 | 0.260 | 3.40 | 21.6 | [全局](center_h4/visualization/overview.gif) | [搜索](center_h4/visualization/search_zoom.gif) |
| offset_h2 | goal | 21.89 | 0.251 | 3.42 | 18.4 | [全局](offset_h2/visualization/overview.gif) | [搜索](offset_h2/visualization/search_zoom.gif) |
| offset_h4 | goal | 26.16 | 0.353 | 3.44 | 18.7 | [全局](offset_h4/visualization/overview.gif) | [搜索](offset_h4/visualization/search_zoom.gif) |
| center_perception_only_h2 | collision | 13.45 | -0.000 | 3.37 | 40.7 | [全局](center_perception_only_h2/visualization/overview.gif) | [搜索](center_perception_only_h2/visualization/search_zoom.gif) |
| center_h3_repeat | goal | 24.11 | 0.257 | 3.34 | 18.8 | [全局](center_h3_repeat/visualization/overview.gif) | [搜索](center_h3_repeat/visualization/search_zoom.gif) |
| center_h3_delay3 | timeout | 50.39 | 0.242 | 3.33 | 19.1 | [全局](center_h3_delay3/visualization/overview.gif) | [搜索](center_h3_delay3/visualization/search_zoom.gif) |

## 主场景 V11 / V12 对照

橙色为实际 REMANI 参考，蓝色为已执行轨迹，车体显示同一固定臂投影。旧失败面板冻结在本次终止状态。

- 单障碍穿越：[对照 GIF](comparison/01_single_h3/comparison.gif)
- 双障碍交叉：[对照 GIF](comparison/02_two_crossing_h3/comparison.gif)
- 三障碍连续交会：[对照 GIF](comparison/03_three_sequential_h3/comparison.gif)
- 四障碍混合交会：[对照 GIF](comparison/04_four_dense_h3/comparison.gif)
- 单独迎面交会：[对照 GIF](comparison/05_head_on_only_h3/comparison.gif)
- 四障碍镜像交会：[对照 GIF](comparison/06_four_mirrored_h3/comparison.gif)
- 四障碍延后迎面：[对照 GIF](comparison/07_four_late_h3/comparison.gif)
- 中心线正迎面：[对照 GIF](comparison/08_head_on_center_h3/comparison.gif)

浅蓝/浅红：内核可行/被拒候选；紫色：内核选中；绿色：发布前复核通过的局部展开。
灰箭头只显示感知CV的最多1.5s，MPPI完整展开长度由本帧搜索记录和limits.json确定，两者不能混淆。
位姿误差、轮速、观测新鲜度和命令租约仍检查。制动后备检查只覆盖制动至停车稳定，不是停车后无限时间安全保证。
仅改感知的试验使用V11控制内核与V12感知，以区分感知修复和局部控制修改的贡献。
带搜索遥测的计算时延不是无干扰硬实时基准；实际执行仍有50ms截止约束。

