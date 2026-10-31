# REMANI / 后台 MPPI / 接入 MPPI 计算资源对照

实验基于冻结的 `native_v12_07`，固定机械臂，在 localhost:11441 运行原生 ROS 数值闭环。
三组使用同一个带观测探针的原生规划器、同一个 V12 检测器和 MPPI 库。

| 组别 | 规划和实际控制 | MPPI |
| --- | --- | --- |
| remani | 当前工程 REMANI + 原 Tracker | 不启动 |
| background | 与 remani 相同 | V12 内核和执行复核持续计算，全部发布到 `/resource_shadow/` |
| coupled | V12 协同重规划 + 局部执行 | 正常计算和执行 |

background 从原生多项式接收参考，原生取消后仍保留最近参考用于测量计算负载。
感知新鲜度和固定机械臂姿态检查仍生效。其原生模式没有协同参考事务，后台适配器使用
同一个 `NativeReference` 解析器组装收到的多项式；这不构成逐周期相同的 MPPI 输入。
每次运行保存 ROS 发布关系，确认后台节点没有向原控制、进度或规划反馈话题发布。
原控制器仍是该组唯一实际底盘控制来源。

CPU 每 200 ms 采样，100% 表示占满一个逻辑 CPU。记录规划器和 MPPI 的进程、线程、
各核负载与 RSS；每次原生求解独立记录墙钟时间、线程 CPU 时间、进程 CPU 时间、
Linux 调度队列等待时间、上下文切换、输入指纹、轨迹分段数和优化变量数。
探针的 `iterations` 字段沿用原生计数器，实际统计代价/梯度回调的完成次数，不能直接当成 L-BFGS 主迭代次数。
REMANI 进程 CPU 包含地图工作线程，求解线程 CPU 单独记录。
探针使用实际单调时钟和 CPU 时钟，不依赖 ROS 模拟时钟。

`smoke_01` 是开发冒烟批次，使用了后来发现开销较大的 100 ms 全进程采样器。
`validation_01` 的优化采样器依赖 `/proc/.../children`，本机内核未提供该入口，导致只采到
启动器、未采到节点进程；该批进程CPU数据无效，求解内部计时保留用于诊断。
正式版本通过隔离 ROS master 查询节点 PID，缓存后每 200 ms 采样，并记录采样器自身开销。
忙算/休眠校准确认能识别单核约 100% 和约 0% 两种状态。冒烟结果不纳入正式统计。
修复后的 `smoke_02` 另核对了全部必需节点 PID；同核双忙算校准验证了线程 CPU 与
运行队列等待时间的区分能力。正式采样的开销随节点和线程数记录，不能用单进程校准的
采样耗时替代完整 ROS 图的监测开销。正式结果保存到 `validation_02`。

## 相同优化请求的补充诊断

三组在线闭环的后续状态、地图接收时序和搜索结果会变化，不能单凭平均耗时判断竞争。
单独的 `paired_static_diagnostic` 在一次静态场景的 0.60 s 参考刷新中，保持同一个
序列化规划回调和优化初始数组，分别以 0.60/1.50 s 预算重新初始化优化器。
每种预算按“MPPI 暂停、运行、运行、暂停”的次序重复，只对本次试验记录的 MPPI PID
使用 SIGSTOP/SIGCONT；不暂停其他进程。输入指纹必须一致。

规划器主队列使用 `ros::spin()`，原生 ESDF 提交由该队列执行，因此补充诊断的递归求解
期间主地图不接受新的提交；场景是持续存在的静态障碍。此诊断只隔离优化阶段，
不代表运动学搜索的严格配对，也不用于动态场景预测精度评价。
额外求解会改变运动时序，该诊断的在线到达时间不纳入三组性能结果。

## 复现

```bash
source /opt/ros/noetic/setup.bash
source /home/l/remani_ws/devel/setup.bash
python3 experiments/online_remani_validation/build_resource_probe.py \
  --out experiments/online_remani_validation/builds/my_resource_probe
python3 experiments/online_remani_validation/run_resource_comparison.py \
  --build experiments/online_remani_validation/builds/my_resource_probe \
  --out experiments/online_remani_validation/results/resource_contention/my_validation \
  --repeats 2 --scenes empty static dynamic_four --duration 48
python3 experiments/online_remani_validation/report_resource_comparison.py \
  experiments/online_remani_validation/results/resource_contention/my_validation
```

所有输出目录必须是新目录。现有构建和失败记录保留。
该构建只增加观测和显式诊断入口，三组正常试验的优化目标、约束、迭代上限和预算策略一致沿用 V12。
三组之间的原生/协同运行分支差异会在请求和预算记录中体现，不解释为单纯 CPU 因果贡献。

原始数据：`cpu_samples.jsonl`、`solver_trace.jsonl`、`publisher_audit.json`、完整状态与轨迹日志。
汇总：`resource_summary.json`、`resource_metrics.csv`、`optimizer_calls.json`、`replan_calls.json`。
图片：CPU 时间曲线、三组资源对比、相同优化请求的暂停/运行 MPPI 对比。
搜索候选的可视化导出关闭；结果绘图在在线试验结束后执行。
