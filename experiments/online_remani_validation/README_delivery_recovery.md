# 2026-10-23：轨迹交付基础验证

这是固定机械臂、完整机器人投影、原生REMANI在线规划与原MPPI执行的数值SIL。不是Gazebo或实车验证。仅使用localhost:11441与`/sim/cmd_vel`，没有改生产src/devel或MPPI库。

## 两个独立改动

1. `build_domain_recovery.py`：将有限的地图域外优化试探作为不可接受步，沿原线搜索二分缩步。初始非法状态、过期/坏快照、非有限数值不因此变成可恢复安全状态。终止负状态先拒绝，避免消费非法试探残留的可变缓存。
2. `build_clearance_recovery.py`：名义求解后静态净距不合格时，只在原优化截止时间内，允许一次固定K8乘子更新与一次全新历史的L-BFGS重启。rho固定1e5；保留40ms验证时间，不重置原来的0.6s重规划预算。恢复后从返回x重建缓存，仍需原生3D/自碰撞/轮速与最新快照验收。

约束项使用每个配点、每个投影圆盘的真实距离导数；没有把已有舒适度代价的梯度错当作最小净距梯度。乘子在一次重启期间不可变，不跨请求保留。使用固定`omega/K`权重，不让AL负常数项随轨迹时长积分而奖励延长轨迹。没有新增膨胀/不确定性半径，没有缩小机器人、放宽10cm硬余量或轮速限制。

## 已完成结果

- 越界开关OFF/ON各1次静态闭环：`results/domain_recovery_20261023_01`。
- 净距恢复OFF/ON各1次静态闭环：`results/clearance_recovery_20261023_01`。
- 四次均到达目标，未记录完整投影碰撞，运行/生产完整性审计通过。
- 越界两次都没有激发域外试探；不能拿它们证明回退效果变快。6条候选独立复查与在线验收一致。
- 净距恢复ON确实触发一次：约120ms内求解器按目标函数相对变化条件停止，但最小物理净距仅从3.574cm到4.410cm，仍不足10cm，正确拒绝。
- 该恢复请求的固定起点净距9.9245cm，本身就不满足全时段10cm约束，单改内部变量无法修复这个固定点；中段K8约束也依然失败，不只是密采样漏检。
- 上一批原始拒收候选起点15.7647cm是可行的，因此不能把所有历史失败都归为固定起点问题。

这是一轮失败机制与兼容性诊断，不是动态快速避障效果已实现的证明，也不是统计性能比较。有限的一次AL更新不保证可行；原预算检查是协作式，不是硬实时抢占。诊断I/O会影响后续请求时机。

## 查看结果

- 根目录摘要：`results/20261023_轨迹交付验证总结.txt`。
- 越界报告与图：`results/domain_recovery_20261023_01/report/`。
- 净距报告、独立复查与图：`results/clearance_recovery_20261023_01/report/`。
- 约束核/原生MINCO梯度/预算测试日志和编译命令：`results/clearance_verification_20261023_01/`。
- 每次试次的`delivery_diagnostics/clearance_recovery/`保存小型名义/恢复系数快照。`failed_last_evaluated_trial_NOT_canonical_x`不是成功返回状态，不可冒充可交付参考。

## 回退与复现

旧构建和数据没有覆盖。生产系统未接入这些模块，不需要回退实车文件。

- 原诊断基线：`builds/delivery_diagnostics_20261022_01`。
- 域回退版本：`builds/domain_recovery_20261023_01`。
- 净距实验版本：`builds/clearance_recovery_20261023_01`。

新实验通过独立构建路径选择；恢复开关默认为关闭。不要将净距实验分支默认为生产可用版本。重跑时给`--out`指定新的、不存在的目录，不能覆盖旧结果：

```bash
python3 experiments/online_remani_validation/run_domain_recovery.py --build experiments/online_remani_validation/builds/domain_recovery_20261023_01 --out <新的结果目录>
python3 experiments/online_remani_validation/run_clearance_recovery.py --build experiments/online_remani_validation/builds/clearance_recovery_20261023_01 --out <另一个新的结果目录>
```

两组净距试次共同启用域回退，并共同增加“原生验证超过原优化截止时间也不能成功交付”的防线；旧批不是新批的严格同配置性能对照。原resource/failure trace的`lbfgs_result/force_stop/final_cost`仍描述名义求解，恢复状态以`clearance_restart_end`与后续资格/验收结果为准。

下一步应先检查真实状态/固定边界/交付时刻的一致性，再用约束残差控制可行性恢复的停止与预算分配；当前不继续叠加罚权，也不提前宣称完成动态快慢层耦合。
