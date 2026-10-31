# 预算、轮速验收与冻结恢复诊断

日期：2026-10-16。实验只使用隔离 ROS 数值仿真和离线控制重放，不启动硬件驱动。
固定机械臂、差速底盘和完整碰撞几何；生产源码及实车参数不改。

## 单变量诊断

1. 在新采集的静态/四动态场景中各选取两个首次0.6秒刷新请求；自然在线调用不启用
   昂贵分项计时，也不按失败种类筛选。对得到的候选另做轮速复检，不预设失败类型。
   在同一个序列化规划回调中暂停本次试验自己的传感器/数值底盘和 MPPI 进程，固定
   ROS 时间、地图提交与求解输入；重新初始化优化器，比较 0.6 / 1.5 / 3.0 秒预算。
   预算组保留轮速上限3.28 rad/s，另有3.5 rad/s组；每种预算/轮速两次关闭分项计时，
   另做6次原轮速下开启分项计时的对照。记录输入指纹、首个代价/梯度指纹、地图代次、
   求解耗时、线程 CPU 时间、运行队列等待、代价计算分项、收敛和最终验收结果。
   暂停产生的运动/感知时间差不计入在线性能。
2. 对轮速失败的同一候选先只改变验收上限重新检查，再以相同输入、预算重新优化，
   比较 3.28 / 3.5 rad/s。此改动仅在实验内生效，不代表硬件标定或永久放宽约束。
3. 离线使用实际 Tracker 控制循环（禁止 ROS 发布），重现“已冻结、纯横向残差”状态；
   对照未冻结的相同偏差，以及进入恢复范围的已冻结状态。另用同一已验收原生轨迹
   做固定参考闭环，记录限幅、执行响应和误差，诊断误差来源。
   补充单变量诊断：仅将角速度前馈替换为当前参考朝向按实际参考推进时序的变化率，
   保留0.55rad/s前馈上限及全部Tracker/底盘限幅，用于检查低速前馈抑制与时钟缩放的影响。
4. 重用既有正常在线运行的阶段/参考记录，分别统计搜索优化、验收失败、重试间隔与
   交付后的执行停滞。诊断暂停后的运行不作为导航成功率或延迟对比。

## 判定边界

- 增加预算后仍收敛到约束失败：不能解释为单纯时间不够。
- 增加预算后有效交付：支持当前请求预算不足，不证明所有请求都应采用更长预算。
- 轮速放宽只消除该关卡：不能证明所有其他约束通过，更不能证明实车安全可执行。
- 各层嵌套计时不能直接相加；计时代码存在观测开销。
- 不把输入起点到交付时位置的差直接等同于正确时标下的接续误差。
- 本实验是当前工程冻结 V12 的机制诊断，不是原论文基线或 Gazebo/实车验收。

## 复现

先加载 `/opt/ros/noetic/setup.bash` 和工作空间 `devel/setup.bash`。

```bash
python3 experiments/online_remani_validation/build_failure_diagnosis.py --out /tmp/my_failure_build
python3 experiments/online_remani_validation/run_failure_diagnosis.py --build /tmp/my_failure_build --out /tmp/my_failure_results
python3 experiments/online_remani_validation/run_tracking_diagnosis.py --build /tmp/my_failure_build --out /tmp/my_failure_results/tracking_01
python3 experiments/online_remani_validation/report_failure_diagnosis.py /tmp/my_failure_results
```

所有输出使用新目录，不能覆盖已有结果；运行需要隔离localhost:11441的网络权限。
脚本不会关机，关机是本次用户单独授权的任务收尾操作。

本次开发记录：validation_01/02准备失败，无有效配对；validation_03正常闭环中未遇到预选
轮速拒绝事件，故没有配对矩阵。validation_04保留为细分计时开销研究，不能把其触发超时
都当成未计时原生求解的自然超时。正式主批次为validation_05，各批次不混淆。
