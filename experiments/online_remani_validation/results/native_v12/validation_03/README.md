# V12 在线验证与对照

原生 REMANI 在线规划/重规划 + 固定臂底盘 MPPI，隔离 ROS 数值仿真。
主场景均3s有效期，另做2/4s时域、仅改感知、重复运行和参考交付延迟检查。
全部计划运行和失败均保留；不同在线运行的原生搜索与调度存在差异，不能当作统计优势。

| 试验 | 结果 | 运动用时/s | 最小净距/m | 首次CV输入/s | 内核P95/ms | 全局GIF | 搜索GIF |
| --- | --- | ---: | ---: | ---: | ---: | --- | --- |
| 01_single_h3 | goal | 23.62 | 0.281 | 2.45 | 17.9 | [全局](01_single_h3/visualization/overview.gif) | [搜索](01_single_h3/visualization/search_zoom.gif) |
| 02_two_crossing_h3 | goal | 23.71 | 0.302 | 2.44 | 18.2 | [全局](02_two_crossing_h3/visualization/overview.gif) | [搜索](02_two_crossing_h3/visualization/search_zoom.gif) |
| 03_three_sequential_h3 | goal | 24.77 | 0.303 | 2.41 | 18.2 | [全局](03_three_sequential_h3/visualization/overview.gif) | [搜索](03_three_sequential_h3/visualization/search_zoom.gif) |
| 04_four_dense_h3 | goal | 27.74 | 0.322 | 2.40 | 17.9 | [全局](04_four_dense_h3/visualization/overview.gif) | [搜索](04_four_dense_h3/visualization/search_zoom.gif) |
| 05_head_on_only_h3 | goal | 23.45 | 0.282 | 3.44 | 19.1 | [全局](05_head_on_only_h3/visualization/overview.gif) | [搜索](05_head_on_only_h3/visualization/search_zoom.gif) |
| 06_four_mirrored_h3 | goal | 27.70 | 0.319 | 2.42 | 17.9 | [全局](06_four_mirrored_h3/visualization/overview.gif) | [搜索](06_four_mirrored_h3/visualization/search_zoom.gif) |
| 07_four_late_h3 | goal | 32.53 | 0.306 | 2.49 | 18.1 | [全局](07_four_late_h3/visualization/overview.gif) | [搜索](07_four_late_h3/visualization/search_zoom.gif) |
| 08_head_on_center_h3 | goal | 25.14 | 0.279 | 3.36 | 18.3 | [全局](08_head_on_center_h3/visualization/overview.gif) | [搜索](08_head_on_center_h3/visualization/search_zoom.gif) |
| empty_h3 | goal | 16.21 | 1.095 | — | 22.0 | [全局](empty_h3/visualization/overview.gif) | [搜索](empty_h3/visualization/search_zoom.gif) |
| static_h3 | goal | 26.67 | 0.141 | — | 30.4 | [全局](static_h3/visualization/overview.gif) | [搜索](static_h3/visualization/search_zoom.gif) |
| center_h2 | collision | 13.39 | -0.001 | 3.36 | 20.3 | [全局](center_h2/visualization/overview.gif) | [搜索](center_h2/visualization/search_zoom.gif) |
| center_h4 | goal | 24.75 | 0.262 | 3.36 | 19.1 | [全局](center_h4/visualization/overview.gif) | [搜索](center_h4/visualization/search_zoom.gif) |
| offset_h2 | goal | 23.26 | 0.275 | 3.37 | 18.7 | [全局](offset_h2/visualization/overview.gif) | [搜索](offset_h2/visualization/search_zoom.gif) |
| offset_h4 | goal | 23.30 | 0.320 | 3.41 | 18.4 | [全局](offset_h4/visualization/overview.gif) | [搜索](offset_h4/visualization/search_zoom.gif) |
| center_perception_only_h2 | collision | 13.58 | -0.002 | 3.36 | 40.6 | [全局](center_perception_only_h2/visualization/overview.gif) | [搜索](center_perception_only_h2/visualization/search_zoom.gif) |
| center_h3_repeat | goal | 25.72 | 0.245 | 3.52 | 18.8 | [全局](center_h3_repeat/visualization/overview.gif) | [搜索](center_h3_repeat/visualization/search_zoom.gif) |
| center_h3_delay3 | goal | 26.38 | 0.240 | 3.33 | 19.6 | [全局](center_h3_delay3/visualization/overview.gif) | [搜索](center_h3_delay3/visualization/search_zoom.gif) |

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

