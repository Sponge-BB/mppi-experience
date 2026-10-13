# V4 验证版：完整局部绕行引导与执行权限诊断

这是隔离数值仿真实验，不是 Gazebo，不部署实车，不含 CBF-QP。
最新构建：`builds/mppi_v4_03`。生产源码和实车 launch 未改。
不确定性扩张仍为零，固定余量仍为 0.1 m，原生取消/零限速没有删除。

## 改动的实质

上一版是在被挡的原路径附近偏移少量引导点；静态场景又未接入方向保持。
本版新增小规模局部栅格 A* 搜索，给 MPPI 一个可绕行并回接原路径的完整几何引导。
因此应称“局部几何引导 + MPPI”，不能把效果全部归因于纯 MPPI。

- `mppi_v4.hpp`：77×77、0.1 m 局部栅格，保守全身外接圆加固定余量；障碍来自测量距离场。
- 引导路径缓存并校验；没有被新观测阻塞就保持，不再每个点独立翻转左右方向。
- 引导末端提前回接原路径，避免斜着直接抵达终点导致朝向冲突和停滞。
- MPPI 对局部引导生成采样控制，实际候选仍执行带滞后模型、轮速约束和 CV 碰撞代价。
- 在原有立即前进/减速/停车等候选外，加入等待 0.4 s、0.8 s 后再前进的候选。
- 动态控制预测长度按观测年龄限制在 CV 剩余有效期内，最多 1.9 s；通过减少样本维持近似 rollout 步数预算。
- 原参考的横向误差不再直接误判较宽绕行是跟踪丢失；有局部引导时检查到该引导的距离。

重要边界：局部栅格只有有限范围；其时空风险项使用近似到达时间，路径简化主要检查静态测量几何。
它不是完整时空搜索，也未实现互斥的“障碍前方通过/后方通过”两套完整策略。
真正控制 rollout 才使用时间索引 CV 检查；这些检查是代价而非安全证明。
运动障碍的当前点云仍留在测量距离场里，这个保守占据与未来位置的统一问题未解决。
目前测试覆盖单一前进参考、空旷弯道和倒车；未证明混合换挡、复杂全身运动或任意地图可靠性。

## 上层取消后的影子计算，不是恢复执行

`mppi_node.py` 收到取消后，保留旧参考只用于影子计算，记录局部候选与阻断原因。
`execution_gate` 最终仍强制零输出，原生限速释放也不能让已取消的参考自行恢复运动。
新增单测和日志审计验证这个条件。

这一步用于确认“局部算不出来”还是“局部算出候选但上层不允许执行”。
它没有完成上层接收并独立验证局部执行轨迹的接口，不能宣称动态连续绕障已经实现。
影子候选碰撞代价为零，仅表示当前离散预测内未受该代价惩罚，不是执行安全授权。

## 两层实验必须分开解释

1. `test_local_independence.py`：固定旧直线参考，始终不给新路径，理想合成观测，无 REMANI 上层。
   检查是否能独立绕过障碍，避免把等待上层重规划误算成局部能力。
2. `run.py`：原生在线规划、重规划、点云过滤与目标跟踪，真实保留安全链的数值仿真。
   检查控制器与上层相互作用，不能用第一层成功代替第二层成功。

结果目录：

- `results/local_independence_03`：同观测时间和不变参考下的 v3/v4 对照、路径图与 GIF。
- `results/mppi_v4_online_03`：最新完整图空旷、静态、动态试验；静态障碍早于首条轨迹，不能充当运动中突现障碍证明。
- `results/mppi_v4_motion_static_old` / `mppi_v4_motion_static_new`：运动触发静态障碍的额外新旧对照。
- `results/mppi_v4_comparison`：与原 Tracker 同参数对照索引及验收记录。

首轮局部 v4_01 虽绕过障碍却未完成终端朝向，失败结果保留在 `local_independence_01`。
不能删除失败、混挑不同控制版本中表现最好的场景。

## 复现和回退

```bash
source /opt/ros/noetic/setup.bash
source /home/l/remani_ws/devel/setup.bash
python3 experiments/online_remani_validation/test_contracts.py
python3 experiments/online_remani_validation/test_local_independence.py \
  --builds experiments/online_remani_validation/builds/mppi_v3_02 experiments/online_remani_validation/builds/mppi_v4_03 \
  --fixture experiments/online_remani_validation/results/final_report/empty_mppi \
  --out experiments/online_remani_validation/results/my_local_v4
```

完整图用 `run.py --mode mppi --projection-progress`，指定 v4_03 构建和新的输出目录。
运行器只允许 localhost:11441 与 `/sim/cmd_vel`。
回退仅将 `--build` 改为 `builds/mppi_v3_02`；该库无 v4 标识，节点不会启用局部引导/影子分支。
实车没有修改，无需回退实车。
