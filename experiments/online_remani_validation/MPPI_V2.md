# MPPI v2：仅底盘双组验证

本轮只比较当前工作空间 REMANI + 原 Tracker 与 REMANI + MPPI，机械臂固定。不是未经修改的论文 baseline，不是 Gazebo 或实车。CBF-QP 不启用，v2 内核显式拒绝 CBF 模式。

## 修改内容

1. 将离散参考点最近距离改为路径线段投影，采用沿路径的进度奖励，减少直接吸引到路径终点造成的弯道切角。
2. 采样候选内部采用反馈转向引导，每个预测步根据预测位姿更新转向，而不是整段维持同一个转弯命令。对齐前视目标而不是障碍前的最近路径切向，避免抵消必要的绕行转向。保留随机采样、停止、倒退和先转后走候选。
3. 加强路径、朝向和速度跟踪；增加相对参考运动方向的倒退代价。负速度参考仍正常执行，没有全局禁止倒车。本版尚未实现有距离/时间上限的受困恢复状态机，不能承诺所有条件下不持续后退。
4. 区分 CV 预测的均值外形与远期不确定性：障碍测量外形、当前观测年龄对应的不确定性及原有 0.10 m 余量继续参与高碰撞代价；随预测时间继续增长的不确定性改为有界软风险。没有改变原规划器预测参数和原生安全限速。
5. 取消距离终点约 0.34 m 时跳出优化器的控制覆盖。终点接近和朝向调整作为候选，经过同样的代价评价。约 2.5 cm 内引导候选仍有最终朝向逻辑，不是宣称所有分段逻辑均消失。
6. 新轨迹切换时清空旧 MPPI 热启动；记录路径、风险、碰撞、倒退分项及停止/倒退候选代价，便于后续归因。
7. 用局部连续搜索、缓存弧长和平方距离比较减少重复运算，没有靠放宽 50 ms 计算截止保护让机器人运动。
8. 将数值仿真被动关闭与正常任务超时区分；报告额外识别 required 原生进程退出，避免崩溃被记作普通超时。
9. 新增只显示两组的简洁路径图、原始数据指标和双栏 GIF，历史三栏绘图仍兼容。
10. 针对静态停滞：无动态目标时使用 3 s / 96 候选，有动态目标时保持 1.5 s / 192 候选，预测滚动步总量近似不变；不超出原有动态预测有效期。
11. 原生参考路径在局部碰撞模型下余量不足时，连续降低贴线代价，离开冲突区域恢复原权重；近终点和终端路径代价采用一致处理。实际碰撞惩罚没有降低。
12. 在前方约 1.2 m 的参考范围内，以测量距离场寻找最多偏移 0.30 m 的法向前视引导点；仅用于生成候选，不替代原生参考路径，也不豁免预测轨迹碰撞检查。这是几何一致的候选引导，不是新增 CBF。0.30 m 是引导点偏移搜索范围，不是实际运动偏离的硬约束。
13. 持续局部停滞自动保存 `local_stall.npz`，包含状态、实际参考、观测、距离场和控制输出，便于完整输入重放。

## 安全与可解释性边界

- 不确定性改为软风险，是风险建模变化，不是已经标定的概率安全约束，也不证明实车安全。原型所有“硬碰撞”措辞均指很高的数值代价，不是严格可行性约束。
- 局部层仍使用保守二维距离场/全身圆包络，原生规划器仍使用三维模型；两者尚未完全统一。不能声称已经完成全部接口问题修复。
- 未改原生移动重规划的发布时间等待和迟到候选校验，未改上层零限速。因此零限速期间仍停车，局部控制器不能自行覆盖它。
- 第一轮近距离突现静态障碍出现原 Tracker 组卡住、MPPI 组原生规划进程退出，保留于 `results/mppi_v2_pair_01/`。失败不能删除或用于有效性宣称。
- 控制单元测试使用理想参考及数值模型，不可替代完整 ROS 闭环；完整闭环每组单次，不能做统计排名。

## 版本与回退

- 旧 MPPI 与原生二进制：`builds/fixed_arm_01/`，原结果 `results/final_report/`，未覆盖。
- v2 初版：`builds/mppi_v2_01/`；空旷试验因计算超时未运动，失败保留。
- 投影优化版：`builds/mppi_v2_02/`；第一轮完整双组试验保留。
- 计算优化版：`builds/mppi_v2_03/`；该版静态停滞，没有作为最终成功版本。
- 中间诊断版：`builds/mppi_v2_04/` 至 `mppi_v2_06/`，分别检查静态预测域、贴线/碰撞冲突及前视引导；失败结果均保留。
- 锁定复测版：`builds/mppi_v2_07/`；加入碰撞一致的前视候选后，静态预检已到达，三场景完整复测另存 `results/mppi_v2_eval_07/`。
- v2 构建复制旧版的原生规划/控制/检测二进制，仅替换实验 MPPI 共享库，构建输出目录必须不存在。原生算法不随组别改变。
- 旧控制器实验可继续指定 `--build experiments/online_remani_validation/builds/fixed_arm_01`。新适配器检测不到 v2 接口时保持旧调用路径；原始脚本快照也保存在各旧结果目录的 `runtime/` 内。
- 实车源码、launch、devel 无须回退：本轮没有部署到它们。停止隔离仿真即可退出实验。

## 复现

在工作空间根目录加载 ROS 环境，使用全新的构建与结果目录。以下命令只面向隔离数值仿真：

```bash
source /opt/ros/noetic/setup.bash
source /home/l/remani_ws/devel/setup.bash
python3 experiments/online_remani_validation/build_mppi_v2.py --base experiments/online_remani_validation/builds/fixed_arm_01 --out experiments/online_remani_validation/builds/my_mppi_v2
python3 experiments/online_remani_validation/test_contracts.py
python3 experiments/online_remani_validation/test_mppi_v2.py --build experiments/online_remani_validation/builds/my_mppi_v2 --fixture experiments/online_remani_validation/results/final_report/empty_mppi --out experiments/online_remani_validation/results/my_unit.json
python3 experiments/online_remani_validation/batch.py --out experiments/online_remani_validation/results/my_pair --build experiments/online_remani_validation/builds/my_mppi_v2 --modes remani,mppi --scenes empty,static --duration 45 --projection-progress --scenario-start goal
python3 experiments/online_remani_validation/run.py --out experiments/online_remani_validation/results/my_pair/crossing_remani --build experiments/online_remani_validation/builds/my_mppi_v2 --mode remani --scene crossing --duration 45 --projection-progress --scenario-start motion
python3 experiments/online_remani_validation/run.py --out experiments/online_remani_validation/results/my_pair/crossing_mppi --build experiments/online_remani_validation/builds/my_mppi_v2 --mode mppi --scene crossing --duration 45 --projection-progress --scenario-start motion
python3 experiments/online_remani_validation/report_pair.py experiments/online_remani_validation/results/my_pair
python3 experiments/online_remani_validation/audit.py experiments/online_remani_validation/results/my_pair
python3 experiments/online_remani_validation/animate_simple.py experiments/online_remani_validation/results/my_pair --modes remani,mppi
```

主对照静态障碍沿用旧实验规则：目标发布后 4 s 出现。动态仍在实测前进建立场景时钟后 1 s 出现。两组同规则不代表出现时机器人姿态严格相同，应结合单元测试和重复试验分析。

双组最终索引：`results/mppi_v2_final/`。Tracker 三组固定复用 `results/mppi_v2_pair_02/`，MPPI 三组统一来自 `results/mppi_v2_eval_07/`，不混用不同 MPPI 版本。`finalize_pair.py` 检查原生二进制哈希、参数、几何和场景触发规则一致后，保存明确来源索引。两种方法不是同一时间并发运行，仍存在原生搜索/调度差异。

以最终目录的 `PAIR_RESULTS.md`、`pair_metrics.json`、原始记录和补充结论为准。`mppi_v2_pair_01/02` 及各静态中间试验均保留，不将早期失败替换成成功记录。

## 最终复测结果

六个主试验均到达，轮速/轮加速度/固定臂/生产源码审计通过。MPPI 内核 P95 约 20～23 ms，无记录到的 50 ms 计算截止超限。

MPPI 动态本轮无倒退，但仍有约 9.94 s 上层零限速，尚未实现连续动态绕行。静态重规划后最大几何偏离约 17.3 cm，较旧 MPPI 记录的 32.0 cm 减少，但仍大于本轮 Tracker 的 2.1 cm；参考路径各自不同，不能当成严格同轨迹排名。

静态任务耗时 Tracker/MPPI 为 28.14/27.87 s，动态为 38.68/30.83 s，单次差异不构成统计性能结论。详细限制与所有失败说明见 `results/mppi_v2_final/结论与未解决问题.txt`。
