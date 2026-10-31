# MPPI 换向停滞：独立调整实验

日期：2026-10-18（Asia/Shanghai）。本轮只做数值仿真，未接实车。

## 问题与对照

锁定前轮 `dual_timescale_20261018/local_rejoin_01/source_02/dynamic_crossing`：
机器人回到参考线附近，但在原生倒退段末端持续选零动作。完整参考包含倒退和前进，
不能把中间换向位置误认为最终任务目标，也不能以Tracker时钟判断MPPI是否运动。

三组均复用冻结 `native_v12_07`，固定机械臂和完整投影，不改碰撞外形、margin、轮速、
加速度、预测模型、50ms截止或发布前轨迹/制动验收：

- baseline：原版库，重新执行，不以历史曲线代替。
- phase：显式前进/倒退阶段；投影、引导及局部连接点限制在当前阶段；
  到换向点后先减速/对齐，仅实际测量可推进阶段，清理旧阶段warm/绕行关联。
- phase_objective：在phase基础上，将V12动态分支的阶段引导与评分目标统一到当前阶段终点，
  不变更代价权重；完整参考和最终任务目标仍保留，外部终点判据不变。

phase不是一个开关，而是一组有关联的阶段一致性改动；三组对照不能进一步区分其中
“投影限制、候选引导、清理缓存”等每一个子项的独立贡献。
已有`directed_reference`候选属于不同shared构建，不直接拿它替换本轮冻结基准。

## 阶段切换条件

位置距换向点≤4cm、姿态误差≤0.15rad、当前阶段剩余弧长≤5cm、
实际线速度≤0.025m/s、实际角速度≤0.15rad/s，连续两个控制周期。
一次实际更新最多切一个阶段，预测rollout不可修改实际阶段。
阶段切换不等于控制输出放行；候选仍需经过原来的完整投影碰撞/制动检查。
没有强制最低速度，没有取消必要停车，没有放宽安全约束。

## 执行与结果目录

结果根目录：`results/mppi_cusp_20261018/`。

1. `focus_01`为脚本汇总错误留下的开发运行，不计正式汇总，原始数据保留。
2. `focus_02`对唯一失败样本做三组复现，全部使用相同原生参考、初态和种子。
3. `regression_01`对两条参考各9种工况做三组对照，共54次。
4. `gated_validation_01`：增加局部换向点占据门控，36次配对；参考01动态样本仍因姿态对齐与评分不一致停滞，否决。
5. `gated_objective_validation_01`：占据门控与阶段引导/评分一致性合并，36次配对。
6. `dynamic_seeds_01`：最终候选与原版，种子2/3、两条动态参考，共8次。
7. `reset_control_01`：原版持续选零1秒后仅重置一次缓存的两次反事实对照；不是部署看门狗。
8. 计时全部结束后再离线出图，不与计时实验并发渲染。

使用`build_mppi_cusp.py`生成四个独立候选构建，基准/旧结果/生产代码均不覆盖。
`run_mppi_rejoin_diagnosis.py`仅增加可选内核注入和实验范围字段，原判据不改变。
恢复原算法直接选择`builds/native_v12_07`，没有实车配置需要回退。

```bash
python3 experiments/online_remani_validation/build_mppi_cusp.py --variant phase --out /tmp/new_phase_build
python3 experiments/online_remani_validation/build_mppi_cusp.py --variant phase_objective --out /tmp/new_objective_build
python3 experiments/online_remani_validation/run_mppi_cusp_adjustment.py --variants baseline gated_objective --out /tmp/new_cusp_results
```

正式runner默认使用已归档的`builds/native_v12_07`和`builds/cusp_gated_objective_01`，
上面临时目录构建只示范编译；不要误称临时新库已经被默认runner使用。
完整终点、持续回线、独立物理碰撞审计、速度约束必须同时满足才算通过。
少量确定性工况的通过率不等于实际环境导航成功率。

## 回归中暴露的两道条件

严格阶段约束在静态场景失败：原换向点已被完整投影判碰撞，不能强迫回去。
仅加占据门控仍在另一条动态参考失败：候选要求对齐换向点，评分却偏向旧切线，
约0.21rad姿态残差下持续选零。合并候选的目标与评分目标后再完整回归，不能只放宽门槛。
门控仅是参考关联条件，不是可达性或安全证书；实际候选/制动检查仍保留。
