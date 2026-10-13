# MPPI v3：取消不确定性扩张、局部绕障与回归

范围：固定机械臂的底盘算法数值仿真，完整接入当前 REMANI 在线规划与重规划。
不是 Gazebo，不是实车部署；没有 CBF-QP。生产源码、实车 launch 和 devel 未修改。

## 取消的是什么

`prepare.py` 根据构建清单的 `experiment_profile=no_uncertainty` 同时覆盖：

- REMANI `dynamic_point_flow/position_uncertainty=0`。
- REMANI `dynamic_point_flow/velocity_uncertainty=0`。
- REMANI `dynamic_point_flow/acceleration_bound=0`。
- MPPI `limits.json` 中 `pos_unc/vel_unc/acc_unc=0`。

所以附加量 `position_uncertainty + velocity_uncertainty*t + acceleration_bound*t*t/2` 为零。
对照组 Tracker 也使用相同的零扩张 REMANI 参数，不能拿之前带扩张的对照直接归因于 MPPI。

保留 CV 均值位置预测、实测障碍物外形、机器人/固定机械臂外形、MPPI 原有 0.1 m 固定余量、
测量栅格的离散补偿和原生地图/时空检查。取消不确定性扩张不等于把障碍物缩成点，
也不等于删除所有可视化圆圈。未经数据标定的旧扩张很保守，但删除它也会失去原有的误差保护。

## 局部控制改动

`mppi_v3.hpp` 继承已有 v2 的采样控制，增加三个状态：

1. TRACK：正常跟踪参考路径的几何形状，不追赶已经过去的绝对时间点。
2. AVOID：用相同碰撞模型检查未来 1.5 s 的参考位姿；根据 CV 预测选择左右绕行引导方向。
   横向引导最大偏移 0.30 m，选择方向有滞回，减少左右来回改变。
3. REJOIN：参考检查连续四个控制周期恢复可行，且横向偏差超过 0.03 m 时增强回归代价。
   吸引回归的每个参考位姿仍要通过同一预测碰撞模型的净距检查。

引导点只生成候选控制，不替换 REMANI 发布的轨迹，不免除实际候选的碰撞评估。
回归项是软代价，不是保证一定回归的定理；障碍仍挡着参考路径时不强迫贴回原线。
合法的倒车参考仍允许负速度，不存在全局倒车禁令。

最新构建 `builds/mppi_v3_02` 去掉零扩张条件下重复的一次预测距离查询：名义几何与所谓
不确定性几何完全相同，直接复用净距，代价数值不变。直行/弯道/倒车单测轨迹与 `_01` 一致。
旧诊断字段 `cost_components.uncertainty_risk` 为兼容保留；在 v3 中只表示原有近障碍代价，
不能据这个字段名认为不确定性扩张仍启用。

## 不可忽略的边界

- 这是带启发式引导的 MPPI 风格采样控制器，不是论文算法的严格复现，也没有安全/最优性证明。
- 超过 50 ms、输入过期、轨迹取消、原生零限速时仍停止，不绕过原有安全链。
- 上层仍评估 REMANI 参考轨迹，局部 MPPI 评估自己的短时候选；两者不是统一的轨迹执行契约。
  因而局部可以找到运动候选，并不意味着上层会准许它执行。
- 实时点云仍进入二维测量距离场，动态跟踪目标也进入 CV 检查；没有假称已解决动态占据残留。
- 本轮首轮动态 Tracker 发生保守外形重叠；MPPI 首轮原生规划进程退出码 -11。
  没有 core/调用栈，不能仅凭最后一条日志断言崩溃代码行；不能把这些试验当作动态避障成功。
- 当前生产地图碰撞模型与局部二维全身包络不完全相同；路径偏离可能来自几何约束差异，
  不能仅用加大路径权重解决。不能承诺“加了 MPPI 必然更好”。

## 复现与回退

加载 ROS 和工作空间环境后，输出目录必须是全新的：

```bash
source /opt/ros/noetic/setup.bash
source /home/l/remani_ws/devel/setup.bash
python3 experiments/online_remani_validation/run.py \
  --build experiments/online_remani_validation/builds/mppi_v3_02 \
  --out experiments/online_remani_validation/results/my_v3_crossing \
  --scene crossing --mode mppi --projection-progress --scenario-start motion
```

用同样参数另开输出目录并指定 `--mode remani` 跑对照。静态与空旷采用 `--scenario-start goal`。
强制使用 localhost:11441 和 `/sim/cmd_vel`，不启动实车驱动。

旧控制器/旧扩张策略：改用保留的 `builds/mppi_v2_07`，其 profile 为原始设置。
首轮结果 `results/mppi_v3_pair_01` 和首轮构建 `builds/mppi_v3_01` 均保留。
第二轮所有 MPPI 结果在 `results/mppi_v3_eval_02`；复用第一轮相同参数/原生二进制的三组 Tracker
进行对照，不挑选不同轮次的最佳 MPPI 场景。不应比较单次耗时就作统计性能结论。
