# V8 速度上限与预测时域验证

这是固定参考、理想CV的局部数值实验，不是完整REMANI在线重规划、Gazebo或实车。
不改生产速度配置，不启动ROS，不连接底盘，不加入CBF-QP，不更换差速模型。

## 实验与回退

- 冻结基线：`builds/mppi_v8_09/`，原结果未覆盖。
- 新库：`builds/mppi_v8_horizon_01/`，只编译实验动态库，没有复制或安装ROS可执行文件。
- [本轮结果](results/speed_horizon/validation_01/README.md)：原始数据、GIF、汇总及审计。
- [改动与结论TXT](results/speed_horizon/validation_01/改动与结论.txt)：包括增速失败、目标回接超时和长时域计算超时。
- [轨迹叠加可视化](results/speed_horizon/validation_01/trajectory_overlay_01/README.md)：橙色固定参考、绿色已记录MPPI预测、蓝色实际轨迹，含局部放大。不补造REMANI在线路径。
- 新库默认2秒有效期时，必须与原V8的同场景同种子状态、指令、净距逐项完全一致。
- 回退只需重新选择旧实验库与旧fixture；实车从未切换，无需回滚实车代码。

## 固定的对照设置

| 配置 | 车体纵向上限 | 轮缘速度上限 | 参考巡航速度 | 预测有效期 |
|---|---:|---:|---:|---|
| base | 0.30m/s | 0.2296m/s | 0.18m/s | 2、3、4s |
| fast | 0.35m/s | 0.32m/s | 0.24m/s | 2、3、4s |
| capacity | 0.35m/s | 0.32m/s | 0.18m/s | 2s |

前两种配置在单迎面和四障碍场景各跑3个种子；capacity在单迎面场景跑3个种子，共39次。
从起点开始闭环，障碍物始终按原时间线运动，不为了提速组重新调整障碍物出场时间。
因此提速会改变相遇时刻，这组实验不是严格的同相遇状态对照。
参考速度不是实际速度硬上限；capacity即使参考保持0.18m/s，实际速度也可能增加，不能解释成同实际速度对照。

角速度上限0.8rad/s、纵向/角/轮加速度、滞后、车体几何和0.1m固定余量均不变。
预测不确定性仍为零，不能把理想CV成绩解释成有噪声人群中的成绩。
这些提高后的速度上限没有经过实车标定，不能直接复制到真实底盘。

## 改动方式

`build_v8_horizon.py`从冻结09版源码生成独立动态库，将BaseMPPI、V7、V8的三处19步上限统一提高。
实际步数仍受`max_prediction - 观测年龄 - 预留时间`限制，三个档位展开1.9、2.9、3.9秒。
机器人展开与动态碰撞检查同步延长，不只改变预测显示线。
通过单测直接检查导出候选的末端时间，并验证观测过期、CBF禁用、全占据时未验证停车等行为。

有意不改动的部分：

- 控制周期50ms、展开步长100ms，碰撞评价间隔50ms，执行净距审计5ms。
- 代价权重、V8候选原语速度（例如最大±0.22m/s）和0.8秒阶段切换不变。
  提高上限不等于把全部采样动作等比例加速；本轮是当前策略的参数敏感性验证。
- BaseMPPI保留原有按时域减少随机样本的预算策略，外层确定性候选数量不变。
- 继承的局部栅格提案风险前瞻仍为1.5秒；最终候选的完整时间展开和检查已按新时域延长。
- 原生安全权限不变，但本实验并没有运行完整原生FSM，不能称为验证了端到端安全门。

## 验收解释

同时记录到达、实际碰撞、0.1m余量不足、倒退距离、低速时间、路径长度及计算耗时。
达到目标不是唯一验收条件；碰撞后终止时间也不能算成“更快完成”。
内核时间不含Python候选导出、感知、真实ROS调度；实测计算时间没有施加给运动模型。
单次及P95超过50ms都要报告，不把长时域理想轨迹表现等同于20Hz实时表现。
失败样本全部保留，GIF统一seed 1，不挑最好的种子。

## 复现

输出目录必须全新，不覆盖原始实验：

```bash
cd /home/l/remani_ws
python3 experiments/online_remani_validation/build_v8_horizon.py \
  --out experiments/online_remani_validation/builds/my_v8_horizon
python3 experiments/online_remani_validation/suite_speed_horizon.py \
  --build experiments/online_remani_validation/builds/my_v8_horizon \
  --out experiments/online_remani_validation/results/speed_horizon/my_validation
python3 experiments/online_remani_validation/report_speed_horizon.py \
  experiments/online_remani_validation/results/speed_horizon/my_validation
```
