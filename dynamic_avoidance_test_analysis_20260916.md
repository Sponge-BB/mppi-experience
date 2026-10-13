# REMANI 实车动态避障延迟分析（2026-09-16）

## 1. 数据来源

- ROS bag：`/tmp/remani_dynamic_test.hW5VnR/dynamic_test.bag`
- 时长：343 s，大小 39 MB，141518 条消息。
- 已记录动态跟踪、预测 TTC、安全限速、规划轨迹、控制输出、轨迹进度和 ROS 日志。

## 2. 实测结论

问题不在底盘执行，也不是动态目标没被检测到，而是规划器内部多个机制叠加，使“提前预测”最终变成“提前停车后同步慢规划”。

### 2.1 感知不是主瓶颈

- 首次动态 track 出现：`1789564262.328 s`。
- Point-flow 状态进入预警：`1789564262.358 s`，差约 30 ms。
- Tracker `processing_ms`：平均 11.31 ms，最大 38.21 ms。
- Tracker 输入消息年龄：平均约 105 ms。

感知可以继续优化，但它无法解释 3–22 s 的停车。

### 2.2 “大老远就停”的直接原因

首次事件中：

- 预测先给出 `TTC=1.30 s`，限速约 `0.18 m/s`。
- 仅 133 ms 后，`waitForRequiredESDF(... capture_planning_boundary=true)` 发现最新占据帧比已提交 ESDF 领先一代，直接发布零限速。
- 安全限速在 `1789564262.491 s` 变成 0，距首次 track 只有 163 ms。

当前实现把“不允许用过期 ESDF 开始规划”和“必须立即停车”绑在了一起。对提前预测事件，ESDF 正常落后一代（130–180 ms 计算时间）也会强制停车，这不是 TTC 紧急停车。

### 2.3 快速形变路径在本次测试中 0/6 成功

六次 warm repair 全部失败，每次消耗 203–231 ms：

- 221.1 ms，失败
- 203.1 ms，失败
- 217.0 ms，失败
- 203.9 ms，失败
- 212.5 ms，失败
- 230.6 ms，失败

原因不只是预算太小：

1. warm repair 保留原轨迹拓扑，人横向进入路径时往往需要改变拓扑，单纯形变成功率本来就低。
2. LBFGS 运行期间，独立 point-flow worker 仍以约 20 Hz 替换 swept overlay。优化器每次查询都会读当时最新 overlay，等于在一次优化中不断改变目标函数。日志中的 `Line search reaches maximum` 和 `machine_prec` 与此一致。
3. 动态 KD-tree 查询每次加锁，同时和 20 Hz overlay 更新竞争。

因此继续把 0.20 s 改成 0.30 s 不会解决根因，只会让停车更久。

### 2.4 局部 A* 本身并没有慢到数秒，但被状态机等待拉长

成功的局部 A* 壁钟时间为：473.7 ms、509.5 ms、374.0 ms、374.3 ms。

但从事件首次检测到输出结果，实测出现：988.8 ms、5.375 s、4.857 s。原因是：

- 动态事件已经 latch 了它需要的 raw ESDF generation，但快速修复和 A* 又分别捕获新的 `planning_boundary_required_esdf_seq_`。
- 新点云持续进来，同步状态机不断等待新 generation。
- FSM timer 和重型 safety callback 共用默认单线程 `ros::spin()`；规划期间安全回调和 FSM 无法并行。

ESDF 本身的实测平均计算约 145–155 ms，年龄约 170–200 ms，通常落后 1 generation。它是毫秒级贡献者，不应被状态机放大成数秒等待。

### 2.5 当前的动态预测空间过度保守

实车参数为：

- position uncertainty = 0.08 m
- velocity uncertainty = 0.25 m/s
- acceleration bound = 0.75 m/s²
- overlay horizon = 1.20 s

代码使用 `r(t)=r0+v_uncertainty*t+0.5*a_bound*t^2`，再加额外半径和采样间隙。实测最终约 1.13–1.15 m。

更关键的是：

- 配置的 `planning_overlay_max_inflation=0.30 m` 被代码明确忽略，日志持续显示 `CAP IGNORED`。
- 最远预测时刻的最大半径被平移到所有时间切片，包括 t=0；因此刚检测到人时就得到约 1.13 m 的包络。

这是“大老远就触发”的第二个直接原因。不能简单硬截断为 0.30 m 并声称仍然等价安全；正确做法是保留按时间增长的不确定度，不要用最终半径覆盖全部时刻。

### 2.6 A* 与动态 overlay 使用不一致

- Kino-A* 内部的底盘 primitive/shot 检查已经使用 `checkCarPlanningObsCollision`，即包含 overlay。
- 但局部目标点选择仍使用 raw `checkcollision`；A* 起点/终点的整机检查也使用 raw `checkcollision`。
- 动态局部目标选定后又把机械臂状态替换为当前值，目标点初检和真正搜索状态不完全一致。

实测已出现 `KinoAstar: goal ... is not free`。这会白白消耗一次 A* 预算。

### 2.7 预测空帧的处理与注释意图相反

`empty_clear_frames=3` 的意图是连续三帧空才清除。但当前第一帧空数据就用 `empty_dropout_unconfirmed` 覆盖上一个有效 snapshot，使 overlay 立即 unavailable。

实测动态点在人减速/遮挡时多次出现单帧空洞，造成：

- overlay 在 ready/unavailable 之间抖动；
- 事件被切碎，重复规划；
- 优化期间障碍场突变。

### 2.8 两次失败后“事件预算耗尽”会造成长时间死停

当前 `global_plan_dynamic_max_replans_per_event=2`把失败的 warm repair 和失败的 A* 都算入总预算。两次失败后，即使后续预测和障碍位置已更新，也只会保持零速，直到整个事件被判定清除。

实测零限速持续时间：

| 次数 | 零速持续 |
|---|---:|
| 1 | 2.905 s |
| 2 | 1.130 s |
| 3 | 4.075 s |
| 4 | 5.098 s |
| 5 | 4.303 s |
| 6 | 1.111 s |
| 7 | 1.913 s |
| 8 | 1.214 s |
| 9 | 0.767 s |
| 10 | **22.364 s** |

22.364 s 这次的直接日志是：warm 失败、A* 后端 0.60 s 超时，随后 `Dynamic event replan budget exhausted (2)`，不再尝试新的障碍 generation。

### 2.9 控制执行层不是本次慢的原因

- `/tracking_fault` 全程为 false。
- `/cmd_vel` 在安全零限速后立即归零。
- 新轨迹发布后仍然不走，是因为规划器的 hazard/ESDF guard 继续持有零限速，不是 Tracker 没有执行轨迹。

## 3. 下一版必须同时修复的内容

不建议再单独调 TTC、A* 时间或 inflation 参数。下一版应按以下顺序实施：

1. **解耦“规划输入新鲜度”和“零速”**
   - raw 障碍事件只等待首次 latch 的 collision generation，同一局部规划不再追逐每一帧新点云。
   - point-flow 事件要求对应 overlay generation ready，不应因 raw ESDF 正常落后一代而自动零速。
   - ESDF 未就绪时不允许启动规划，但是否停车只由制动距离/TTC/真实硬碰撞条件决定。

2. **规划使用不变的快照**
   - 一次 A*/LBFGS 必须固定 `{ESDF generation, raw seq, point-flow seq, overlay}`。
   - worker 可继续生成新 overlay，但不能改变正在优化的目标函数。
   - 候选轨迹返回主线程后，必须用最新 raw occupancy 和最新 point-flow 再验证；不通过绝不发布。

3. **动态事件直接进入局部拓扑搜索**
   - 本次 warm repair 为 0/6，不应再固定浪费 0.20 s。
   - 对“预测包络已穿过当前轨迹”的事件直接运行短视野 A*；只有小幅静态边界侵入才使用 60–80 ms warm repair。

4. **统一局部目标与 A* 的碰撞语义**
   - 局部目标选择、A* start/goal、primitive、shot 和后端必须读同一 planning snapshot。
   - 目标点必须在 raw ESDF 和动态 planning overlay 中都可行，并且使用最终实际的 hold-arm 状态检查。
   - 固定 2 m 改为 1.0–2.5 m 适应性搜索：首个具有足够净空的全局轨迹点才可作为局部终点。

5. **改为时变不确定度，不再把 1.13 m 套到 t=0**
   - 时间参数化后端和安全监督器继续使用 `r(t)`。
   - 空间 A* 的 overlay 只作拓扑引导，使用与 ESDF 一致的基础净空加有界预测余量；最终安全性由时间对齐的 `r(t)` 验证保证。
   - 不能把配置写成 0.30 m 却在代码中静默忽略。

6. **修复空帧保持逻辑**
   - 第 1–2 个空帧保留上一个有效 snapshot，仅增加 age，不产生伪的新 obstacle generation。
   - 连续第 3 个空帧才确认 clear。
   - 如果动态 track 因人停下而消失，raw occupancy/ESDF 必须继续接管静态障碍安全。

7. **用 generation + 限频取代“一个事件最多两次”**
   - 同一 `{raw_seq, point_flow_seq, overlay_seq}` 不重复并发规划。
   - 当新 generation 仍与当前轨迹冲突时，允许有限频率的新局部规划，不允许因历史两次失败而永久死停。
   - 失败原因要区分 `TIMEOUT/NO_PATH/GOAL_COLLISION/STALE_CANDIDATE`，不能全部折算为同一个事件预算。

8. **真正的规划 worker 必须是快照输入 + 候选输出**
   - 不能直接把当前 FSM/优化器丢进 `AsyncSpinner`；它们共享 `traj_container_`、ESDF 和 overlay，会引入数据竞争。
   - 主线程只生成不变 `PlanningSnapshot`，worker 只返回 candidate，轨迹发布和最新安全复核仍由主线程单点负责。
   - queue 容量为 1，新 generation 可取消/废弃旧候选，防止积压。

## 4. 不能破坏的安全不变式

1. 任何候选轨迹在发布前必须通过：静态 PCD/ESDF、最新 live occupancy、最新时间对齐 point-flow 三重检查。
2. 候选的输入 generation 已被新 raw/point-flow generation 否定时，必须丢弃，不得发布。
3. 真实车体进入硬碰撞距离、TTC 小于制动+反应时间时，仍必须立即零速。
4. 规划失败时不得恢复已知碰撞的旧轨迹；只能保持限速/停车并在新 generation 上重试。
5. 感知不可用时，不得用“预测清空”替代 raw occupancy 的安全判断。

## 5. 下一版验收指标

- 动态 track 首次确认到规划请求：P95 < 100 ms。
- 早期预测事件到新候选轨迹：P95 < 600 ms。
- 无紧急 TTC/硬碰撞时，不允许仅因 ESDF 正常落后 1 generation 发布零限速。
- 有效新轨迹发布且安全条件已满足后，底盘恢复不得额外等待数秒。
- 规划 worker 运行时，动态安全监督不中断；不能再出现 4–22 s 的状态机死停。
- 保留每次 generation 的 detect/request/start/result/validate/publish/controller-resume 时间戳，不再仅靠主观观察。

## 6. 总结

本轮已经确定：

- 感知能及时发现人；
- 底盘能及时执行停车；
- 慢和长时间停车的主因是规划器内部的新鲜度零速绑定、变动 overlay 上的同步优化、过度保守的恒定最大膨胀、空帧抖动和两次失败后的死停策略。

因此下一步必须是一次结构性修复，不再是单个阈值调参。
