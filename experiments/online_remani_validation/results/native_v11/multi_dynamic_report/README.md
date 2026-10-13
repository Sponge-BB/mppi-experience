# 多动态障碍物与迎面交会：统一入口

当前 REMANI 在线规划/重规划 + 底盘 MPPI，固定机械臂，隔离 ROS 数值仿真。
七种障碍物运动设置来自 V8，另加中心线迎面；保留车体投影、搜索候选、获准局部展开及实际 REMANI 参考。

首次运行八组，随后针对残差容量复测三组四目标场景。下表四目标使用容量匹配后的记录，其他使用首次记录。
这些是具体案例，不是多随机种子统计成功率。用时从实测运动触发算起；失败用时是投影碰撞发生时刻。

| 场景 | 目标数 | 结果 | 用时/s | 最小投影净距/m | 全局 GIF | 搜索放大 GIF |
| --- | ---: | --- | ---: | ---: | --- | --- |
| 单障碍穿越 | 1 | 到达 | 21.44 | 0.1261 | [全局](../multi_dynamic_01/01_single/visualization/overview.gif) | [搜索](../multi_dynamic_01/01_single/visualization/search_zoom.gif) |
| 双障碍交叉 | 2 | 到达 | 22.43 | 0.0945 | [全局](../multi_dynamic_01/02_two_crossing/visualization/overview.gif) | [搜索](../multi_dynamic_01/02_two_crossing/visualization/search_zoom.gif) |
| 三障碍连续交会 | 3 | 到达 | 25.73 | 0.1131 | [全局](../multi_dynamic_01/03_three_sequential/visualization/overview.gif) | [搜索](../multi_dynamic_01/03_three_sequential/visualization/search_zoom.gif) |
| 四障碍混合交会 | 4 | 投影碰撞 | 15.45 | -0.0010 | [全局](../multi_dynamic_capacity_02/04_four_dense/visualization/overview.gif) | [搜索](../multi_dynamic_capacity_02/04_four_dense/visualization/search_zoom.gif) |
| 单独迎面，y=0.30 m | 1 | 投影碰撞 | 13.12 | -0.0024 | [全局](../multi_dynamic_01/05_head_on_only/visualization/overview.gif) | [搜索](../multi_dynamic_01/05_head_on_only/visualization/search_zoom.gif) |
| 四障碍镜像交会 | 4 | 投影碰撞 | 15.20 | -0.0020 | [全局](../multi_dynamic_capacity_02/06_four_mirrored/visualization/overview.gif) | [搜索](../multi_dynamic_capacity_02/06_four_mirrored/visualization/search_zoom.gif) |
| 四障碍延后迎面 | 4 | 投影碰撞 | 17.17 | -0.0012 | [全局](../multi_dynamic_capacity_02/07_four_late/visualization/overview.gif) | [搜索](../multi_dynamic_capacity_02/07_four_late/visualization/search_zoom.gif) |
| 中心线正迎面 | 1 | 投影碰撞 | 12.05 | -0.0003 | [全局](../multi_dynamic_01/08_head_on_center/visualization/overview.gif) | [搜索](../multi_dynamic_01/08_head_on_center/visualization/search_zoom.gif) |

“投影碰撞”表示用于实验碰撞检查的底盘＋固定机械臂三圆盘包络与圆柱相交，触发立即终止。
包络是保守近似，没有在本轮进行真实整机外形标定。
双障碍案例虽然到达，最小净距仍低于设定的 0.10 m 安全余量，不能视为全程满足完整余量。

## 怎样读图

橙色：实际消费的 REMANI 参考；蓝色：已执行轨迹及底盘轮廓；灰蓝色：三圆盘车体投影。
浅蓝/浅红：内核可行/被拒候选；紫色：内核选中；绿色：通过发布前复核、获准执行的局部展开。
灰色圆柱上的编号为场景身份；灰箭头来自实际检测速度的 CV 展开，最长 1.5 s。
当前及未来投影随位姿变化，轨迹线仅表示中心运动，碰撞检查仍使用完整投影。

每种场景的原始记录、参数、全部障碍物、参考版本、审核与 GIF 分目录保存。
旧结果、碰撞失败和容量复测都保留：[首次八组](../multi_dynamic_01/README.md)、[四目标容量复测](../multi_dynamic_capacity_02/README.md)。

## 本轮实际暴露的问题

1. **四目标感知容量原本不足。** 数量上限从 3 调到 4 后，四圆柱仍触发了残差点数上限：808 > 800。
   调整隔离配置到 1067 点后，三个四目标复测均未再记录到残差超限或运动期间感知健康关闭。
   这解决了场景配置问题，但三个复测仍发生投影碰撞，不能把失败全部归因于感知门限。
2. **迎面目标动态资格确认滞后。** 两个单独迎面目标均在运动后 2 s 出现；
   y=0.30 m 案例约 7.27 s 才首次进入 MPPI 的 CV 输入，中心线案例约 11.17 s 才进入。
   感知日志此前已有目标点云和聚类，所以问题在运动确认/背景抑制流程，不能将这段延迟解释为雷达一直没有看到目标。
3. **局部避让尚未形成可靠的侧向脱离动作。** 容量复测中仍有多个无可行候选、动作复核拒绝和超时周期；
   机器人后期停住，迎面目标继续接近，最终投影相交。三个复测都只交付了初始 REMANI 参考，
   快层在慢层没有及时交付新参考时未完成迎面避让。

上述原因分别对应配置容量、运动识别和局部避让能力，后两项本轮没有修改算法来修复。
全部参考来源和实际执行契约审核通过；这仅证明来源及命令检查流程符合记录契约，碰撞案例仍然是闭环失败。

最终核查覆盖 11 次闭环记录、22 个可完整解码的 GIF 和 1289 个搜索快照。
所有真值帧均保留声明的完整目标数量，运动位置与场景 CV 定义一致；全部试验使用相同控制内核和车体投影。
1284 个快照的感知输入与独立收集的诊断逐项一致，其余 5 个为启动期间未被诊断订阅器收集的空搜索记录。

## 复现

单次场景、批量运行和参数说明见 [NATIVE_MULTI_DYNAMIC.md](../../../NATIVE_MULTI_DYNAMIC.md)。
当前代码可直接用 `run_native_v11.py --scene dynamic_four` 启动数量扩展场景，或使用
`run_native_multi_dynamic.py` 运行全部八组。运行结束自动生成每个场景的全局和搜索放大 GIF。
