# 当前 REMANI 完整在线闭环算法验证

这是本机隔离 ROS 数值闭环，不是 Gazebo，也不是实车试验。固定机械臂，只比较底盘局部控制。

[V12 慢速目标感知与迎面/横穿局部策略](NATIVE_V12.md)：隔离候选及多场景验证，
保留完整车体投影、搜索 GIF、失败记录与时域/感知消融；不替换实车或旧版本默认入口。

[V11 静态绕行接续与执行备选复核](NATIVE_V11.md)：修正静态恢复时域、局部进度评价、
原生搜索非法种子边界，并保留所有失败案例。专用入口 `run_native_v11.py`，不替换旧默认版本。

[V10 局部执行权限与统一执行轨迹检查](NATIVE_V10.md)：协同执行接口与三秒参考延迟测试，
历史静态回归失败；V11 以此为对照。

[V9 同帧分类与统一候选搜索实验](NATIVE_V9.md)：局部无解减少，但完整动态复测因上层取消停车更久，
**未通过整体性能验收，不替换默认 V8**。包含失败记录和新旧 GIF。

[当前 V8 MPPI + 实际 REMANI 在线参考](NATIVE_REFERENCE.md)：专用入口 `run_native_mppi.py`，
新增参考来源核验、替换/取消保护及真实参考 GIF。不是固定直线参考实验，也尚非实车放行版本。

[V8 速度上限与预测时域验证](SPEED_HORIZON.md)：独立固定参考实验，分离提高速度上限、提高巡航速度与延长时域；不改实车参数。

[V8 候选覆盖与恢复改进](MPPI_V8.md)：补齐无可行种子时的探索、前后退让与末端风险评价，
仍是固定参考的局部实验版；完整成绩与限制见其对照报告。

[多动态障碍物与真实候选轨迹可视化](MPPI_MULTI_DYNAMIC.md)：V7固定参考局部实验，新增1～4个动态目标，
每场景独立提供全景、搜索放大和纯运动GIF；不改变实车控制链路。

[V7 动作时机与回接实验](MPPI_V7.md)：把风险监测和实际绕行分开，增加有效推进评价、显式回接，
并用 V6/V7 同输入、障碍物持续存在的场景对照。[V7 GIF 与结果导航](results/mppi_v7/validation_01/README.md)。仍不是实车发布版。

新增 [V6 连续运动与动静态分层实验](MPPI_V6.md)：仅理想观测的局部固定参考验证，
**不是实车/完整 REMANI 动态避障已解决**。动画按场景单独整理，入口为
[V6 实验导航](results/mppi_v6/validation_01/README.md)。

缩小引导碰撞外形和提前回接的最新验证版见 [MPPI_V5.md](MPPI_V5.md)。
局部对照已减少绕行距离，但完整动态正式试验出现原生规划器崩溃，不能作为实车发布版本。

局部自主绕行与上层执行权限的最新验证版见 [MPPI_V4.md](MPPI_V4.md)。它是局部栅格引导 + MPPI，
新增固定旧参考对照和取消后影子诊断；尚未放行已被上层取消的轨迹，不是实车上线版本。

最新的无不确定性扩张、无 CBF 双组实验见 [MPPI_V3.md](MPPI_V3.md)。
对比图/GIF 在 `results/mppi_v3_comparison/`，验收限制见其中的 `验收与未解决问题.txt`。
代码与单测已完成，但仍存在动态停车、静态跟踪偏差及首轮原生规划器崩溃，尚未通过实车部署验收。

## 实际接入内容

- 从当前 `src/` 拷贝并编译原生规划器：FSM、运动学搜索、机械臂搜索、轨迹优化、原生地图/异步 ESDF、动态点流检查和轨迹发布。
- 运行原生点云过滤器、目标级动态检测器、原生控制器；对照组实际运行原 Tracker，不再用固定参考线代替重规划。
- MPPI 消费 `/planning/trajectory` 的真实多项式，按原生列主序和降幂系数解析；处理轨迹替换、取消、未来启动时间与多段正反向运动。
- 实际检测输出 `/dynamic_points` 提供位置、速度和目标 ID；局部地图来自过滤后的实时点云和静态地图。仿真障碍物真值不传给控制器，只用于生成测量和评价。
- MPPI 组通过实测位置在参考路径上的连续局部投影，向原生 FSM 回传匹配的轨迹起点时间、当前轨迹时间、进度比例。原控制器进度改发到观察话题，避免两个发布者和旧虚拟时钟干扰。
- 原生安全限速、急停、重规划失败及候选校验均保留。MPPI/CBF 无权越过原生零限速。

## 公平性与边界

三组采用相同实车展开参数、固定机械臂姿态、目标、传感器生成规则、车体几何与带滞后差速模型。模型同时执行底盘及左右轮速度/加速度约束。

固定臂不是仅冻结驱动反馈：隔离源码的优化器将关节坐标梯度清零，搜索关节上下界设为同一初始姿态；输出关节命令和原生多项式同时审计。生产源码没有修改。

MPPI 为 192 个样本、1.5 秒预测域、20 Hz 控制的实验原型。CBF 为带执行滞后的离散约束、有限差分线性化与小规模 QP，可失败；失败时请求停止，不代表已经证明停止安全。

局部碰撞检查使用保守二维全身圆形包络和测量栅格距离场；原规划器仍使用原生三维 ESDF/碰撞模型。两者几何表示不完全相同，这是需要研究的接口，不应把结果解释成严格的算法单因素优劣。

静态障碍在目标后 4 秒出现；横穿障碍在目标后 1 秒开始运动。首次规划延迟和随机搜索差异会改变相遇状态。每组合单次试验用于发现问题，不用于宣称统计成功率。传感器目前无遮挡、噪声或定位误差，底盘无打滑，不能替代 Gazebo 或实车验收。

## 安全隔离与回退

启动器强制 `ROS_MASTER_URI=http://127.0.0.1:11441`，使用独立日志/ROS_HOME，只向 `/sim/cmd_vel` 发布。节点拒绝其他主机及端口。图中不包含 CAN、Piper、真实底盘驱动或实车关节桥。

所有改动在本实验目录，实车 launch、源码、devel 二进制不变。停止实验即可回到原来的实车流程，不需要覆盖或回滚源代码。每轮保存参数、源码哈希、原生二进制哈希、运行脚本副本和原始 CSV/JSON。

## 复现

先加载工作空间的 ROS 环境。输出目录必须不存在。

```bash
cd /home/l/remani_ws
python3 experiments/online_remani_validation/build.py --out experiments/online_remani_validation/builds/my_build
python3 experiments/online_remani_validation/test_contracts.py
python3 experiments/online_remani_validation/batch.py --build experiments/online_remani_validation/builds/my_build --out experiments/online_remani_validation/results/my_runs --projection-progress
python3 experiments/online_remani_validation/report.py experiments/online_remani_validation/results/my_runs
python3 experiments/online_remani_validation/audit.py experiments/online_remani_validation/results/my_runs
python3 experiments/online_remani_validation/animate.py experiments/online_remani_validation/results/my_runs
```

单次测试使用 `run.py --scene static --mode mppi_cbf --projection-progress`，并提供 `--build`、`--out`。

`--planner-variant no_wait` 是单独的时间戳消融：只移除规划管理器发布前的等待，保持未来启动时间、预测拼接状态、候选验证和原生安全保护。不等于已在生产环境修复。当前消费者收到未来轨迹后会保持/停车等待，并未双缓冲继续执行旧轨迹；正式修复还必须处理这个交接问题。

`--scenario-start motion` 在底盘前进至 0.10 m、实测速度达到 0.05 m/s 后建立场景时钟，避免把“初始规划等到障碍离开”误算为运动中避障。最终动态对照使用这一选项。

## 结果目录

- `results/full_online_01/`：旧进度契约的诊断对照，暴露原控制器时钟与 MPPI 几何进度冲突。
- `results/full_online_02/`：补齐进度接口后的主对照。
- `results/no_wait_01/`：发布前等待的单项消融（若已运行）。
- `results/moving_crossing_01/`：底盘确实运动之后的横穿补测。
- `results/final_report/`：最终精选记录的符号链接索引；两张简洁主图和对应 GIF。原始失败试验没有删除。
- `smoke_*`、`remani_static_01`、`online_01_mppi_empty`：接入调试，不纳入主对照。前两类早期调试存在重复更新时间戳导致静态地图重载/检测器重置的问题，已修复。

`online_paths.png`、`online_timeline.png`、`static_online.gif`、`crossing_online.gif` 均从原始记录绘制。`metrics.json` 为指标，`events.json` 为原生 ROS 日志，`trajectories.json` 为实际规划输出，`audit.json` 为一致性和失败现场检查。

简洁版动画为 `results/final_report/static_simple.gif` 与 `crossing_simple.gif`：统一颜色、三栏并排，只保留图例和状态，不显示调试数值。运行 `python3 experiments/online_remani_validation/animate_simple.py experiments/online_remani_validation/results/final_report` 可从已有记录重绘，不会启动 ROS 或重新仿真。

局部执行权限及统一审核见 [V10 接口说明](NATIVE_V10.md) 与 [V10 完整结果](results/native_v10/README.md)。动态参考延迟测试通过，静态回归失败，尚未用于实车。
