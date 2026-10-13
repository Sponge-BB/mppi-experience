# V11 实验索引

范围：原生 REMANI 在线规划 + 底盘 MPPI，固定机械臂、无 CBF-QP。
**ROS 数值 SIL，不是 Gazebo，也不是实车。** [改动与复现说明](../../NATIVE_V11.md)。
最终构建 `builds/native_v11_08`。旧构建和全部失败记录保留。

新增：[车体投影与真实搜索 GIF](projection_search_01/README.md)，包含静态、动态全局及局部放大视图。
这是记录和显示补充，原碰撞几何与控制内核不变，不能当作新的算法性能改进。

多动态障碍物与迎面交会：[八组场景统一索引与 GIF](multi_dynamic_report/README.md)，
沿用 V8 的运动设置，参考改为当前原生 REMANI 在线输出。[配置与单次启动说明](../../NATIVE_MULTI_DYNAMIC.md)。

最终六次均到达，参考/执行审核均通过，未记录到碰撞或进程故障。
静态三次用时 26.21 / 26.76 / 26.28 s；两次动态 20.68 / 20.39 s；空旷 16.17 s。
仍有短暂停顿，不能据此视为实车放行或全场景成功率。

## 最终闭环：validation_05

每个场景独立保存原始数据、参数、源码哈希、参考审核、执行审核和 GIF。
动画橙色为实际 REMANI 参考，绿色为审核通过的 MPPI 局部轨迹，蓝色为实际运动。

| 场景 | 动画 | 说明 |
| --- | --- | --- |
| 静态首次 | [GIF](validation_05/static/native_reference.gif) | 主对照使用预先安排的首次运行，不挑最好成绩 |
| 静态复测 1 | [GIF](validation_05/static_repeat/native_reference.gif) | 同配置重新启动原生规划器 |
| 静态复测 2 | [GIF](validation_05/static_repeat2/native_reference.gif) | 同配置重新启动原生规划器 |
| 动态，新参考延迟 3 s | [GIF](validation_05/crossing_delayed/native_reference.gif) | 检查不等待慢层更新的局部执行 |
| 动态，正常交付 | [GIF](validation_05/crossing/native_reference.gif) | 无人为参考延迟 |
| 空旷 | [GIF](validation_05/empty/native_reference.gif) | 检查自由行走和终点接续 |

V10 / V11 并排动画分别保存在 [静态对照](comparison/static/comparison.gif) 和
[动态对照](comparison/crossing/comparison.gif)。这里的 V10 是此前完整接入版，不是原生 REMANI-only，
也不是固定直线参考的 V8。单次在线记录包含随机搜索、调度和相遇时刻差异，不能据此声称统计优势。

机器可读汇总：[summary.json](summary.json)。低线速度命令时间包括转弯，不等于机器人完全静止时间。
最小净距为固定臂保守二维投影与仿真真值的离散记录，不是连续时间碰撞证明。

## 开发过程（包括失败）

| 目录 | 构建 | 结果/用途 |
| --- | --- | --- |
| static_01 / crossing_01 | 01 | 均到达，但静态计算超时较多 |
| static_02 | 02 | 静态超时失败；大量超时与慢规划刷新条件问题 |
| validation_01 | 03 | 静态、动态到达，静态仍有超时 |
| validation_02/static | 04 | 原生规划器 SIGSEGV，失败 |
| debug_static_04 | 04 | GDB 复现随机整数分布崩溃；局部最终到达不算整体成功 |
| validation_03/static | 06 | 拦截零节点种子、原生重试成功；局部仍振荡、任务超时 |
| validation_04 | 07 | 空旷/动态/首次静态到达，但静态重复测试停滞失败 |
| validation_05 | 08 | 统一中间备选评分后的最终六次闭环；所有运行列入汇总 |

`builds/native_v11_05` 为未完成的编译目录，不可用于运行。

## 单元及诊断

- `candidate_contracts_08.json`：6 项新内核/候选接口回归。
- `contracts_08.json`：7 项目标/事务 + 8 项执行内核检查。
- `native_seed_06/result.txt`：5 项原生非法种子边界检查；08 原生二进制与 06 相同。
- `pruning_equivalence_04.json`：硬拒绝剪枝的 100 周期同输入比较，仅覆盖该剪枝改动。
- `guide_probe_07`：早期诊断构建，不是完整报告；中断后保留，没有闭环结论。
- `guide_probe_07_complete/probe.json`：补全诊断；该停滞几何状态可生成 143 点可用连接路径，
  重新初始化控制器后可以给出非零动作；不是历史内部状态的精确回放。
- `held_reference/before`、`after`：早期冻结地图局部对照，采用位置/速度终止判据，
  不是完整姿态到达；不能据此替代在线验证。脚本已补齐姿态判据。

失败数据不删除；主文中改动的因果解释与闭环到达结果分别报告，不把未复现问题的简化离线实验当作修复证明。
