# V8 MPPI 接入 REMANI 在线参考

本入口使用实际运行的 REMANI 规划/重规划输出，不生成直线参考作为替代。
固定机械臂，仅底盘 MPPI，不加 CBF-QP。范围是隔离 ROS 数值闭环，**不是 Gazebo 或实车部署**。
之前 V6/V7/V8 多障碍物、速度/时域实验仍是固定参考实验，原结果没有被改写。

## 数据链路

原生 REMANI `/planning/trajectory`（七阶多项式）→ `native_reference.py` 校验、采样
→ V8 MPPI 的 `online_step` 参考输入 → `/sim/cmd_vel` → 带滞后差速模型
→ `/odom` 实测反馈 → 原生规划器与 MPPI。

- 保留多项式的 x、y、正反向速度、由导数计算的朝向，以及时间轴；采样间隔约 0.04 s。
- MPPI 沿参考几何路径滚动规划，不是强行追赶原规划的绝对时间。实测位置投影出的轨迹时间反馈给原生 FSM。
- id=1 替换参考；相同起始时间、连续 id 的后续消息追加分段。每次追加/替换/取消均使旧计算结果失效。
- 无轨迹、未来启动时间、无效系数/坐标系、漏段或空间不连续时不下发运动；没有固定参考兜底。
- 朝向插值先解缠绕，避免跨 ±π 时错误插到 0。
- 原生取消、零限速、感知新鲜度与 50 ms 计算超时保护保留。取消后的影子计算不能控制机器人。
- 参考切换回调和最后命令发布互斥；不会在已处理取消/换轨后发布用旧参考算出的非零命令。

原生消息没有“总分段数/整包结束”字段。当前按原协议接收连续前缀，在最后一段到达后至少安静 50 ms 再使用；
这不是具有完整性确认的原子轨迹包。迟到段会继续更新版本，漏段会拒绝。
正式部署如需严格的整包确认，应扩展原生发布协议，不能用延时假定完整。
未来起始轨迹暂未做双缓冲交接，因此切换仍可能短暂停顿。

## 复现

```bash
cd /home/l/remani_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 experiments/online_remani_validation/test_contracts.py
python3 experiments/online_remani_validation/run_native_mppi.py \
  --scene static --duration 45 \
  --out experiments/online_remani_validation/results/my_native_static
python3 experiments/online_remani_validation/audit_native_reference.py \
  experiments/online_remani_validation/results/my_native_static
```

输出目录必须不存在。`--scene` 可选 `empty/static/crossing`；`--mode remani` 可运行不加 MPPI 的原生对照。
审计脚本用于 MPPI 组；本次不是充分的多种子性能对照，不得将单次成功理解为优于原版。
端口固定为本机 11441，已有服务占用则拒绝启动；只关闭自己创建的进程组。
实车 launch、硬件控制入口、生产源码、速度/加速度限制、V8 权重和时域均未改动。
默认构建为 `builds/mppi_v8_09`，CV 有效时域 2 s，实际展开受观测年龄限制；不是此前 3 s 时域组。
静态/实时点云和 `/dynamic_points` 来自原生过滤、检测链路，不把仿真目标真值直接作为 MPPI 输入。
`crossing` 沿用既有单目标场景：运动触发后第 1～14 s 存在，不代表多目标持续障碍压力测试。

## 如何确认是真实 REMANI 参考

- `trajectories.json`：独立订阅者记录原始多项式系数、时间戳和序列化消息 SHA256。
- `consumed_references.jsonl`：MPPI 实际传给内核的完整参考数组、时间轴、消息来源哈希和参考哈希。
- `reference_events.jsonl`：替换、分段追加、取消、拒绝原因及版本。
- `mppi_diagnostics.json`：每次计算对应的参考哈希、执行命令和停顿原因。
- `reference_audit.json`：由独立日志中的原始系数重建参考，与内核输入逐项核验；分别报告接入契约和闭环到达结果。
- `native_reference.gif/png`：橙色实际消费的 REMANI 参考，蓝色实际运动；取消后旧参考消失。没有把实际运动轨迹伪装成 MPPI 预测线。
- RViz 添加 `Path`：`/sim/mppi/native_reference`，Fixed Frame=`world`；须连接隔离 master `http://127.0.0.1:11441`。

## 验证记录与边界

见 [本轮测试目录](results/native_reference_v8/README.md)。空旷接入、原生取消后的重规划参考替换均已有闭环记录。
但静态首测原生 `remani_planner_node` 发生 exit -11；调试复测未复现。
这不是已经修好的故障，仍不能作为实车放行版本。测试入口会将必要进程退出明确判为失败，不把 shutdown 当作成功。
本次没有绕过原生取消，也没有解决所有停车/超时问题；接入参考不等于已经实现不中断的快速动态避障。

回退：继续使用此前固定参考实验入口和不可变构建/结果即可；本轮未修改其控制算法，也未改变实车代码。
