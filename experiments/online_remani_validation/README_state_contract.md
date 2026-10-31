# 起点—交付一致性：隔离诊断实验

本实验是固定机械臂的原生 ROS 数值 SIL，不是 Gazebo 或实车测试。机器人按完整投影外形检查，保留10cm硬余量和全部原生门限。只使用 localhost:11441 和 `/sim/cmd_vel`。

## 要回答什么

1. REMANI 消费的是哪一帧里程计，低速正则化是否把倒退变成向前？这一数值变化是否实际传到搜索 root 和 MINCO 固定边界？
2. 在同一冻结碰撞场中，原始姿态、选定起点及后端固定起点的净距是否可行？不把数值初始化问题和固定位置不可行混成一类。
3. 起点消息、请求、求解完成、轨迹时间戳、发布及 MPPI 实际消费之间差多少？

## 唯一对照因素

`STATE_CONTRACT_SIGNED_LOW_SPEED=0/1`。OFF 保留原低速分支；ON 对可靠非零低速保留纵向符号，不继承历史方向。数值速度仍使用原最小非奇异速度，仅增加至多16个ULP以跨过原生严格阈值。真实零速仍保留原处理。这不是提高车速，也不是把正则化速度当真实测量。

两组均启用相同记录和域回退，AL恢复关闭。安全余量、轮速、预算、MPPI控制、参考选择、单线程spinner与轨迹定时政策均不改。在线闭环可能产生不同请求，不能把两条轨迹当同一输入的速度因果配对。

## 数据与时间语义

- `state_odom_events.jsonl`：独立 observer 收到的每一帧原始 Odometry。
- `state_mppi_cycles.jsonl`：同锁复制的求解/验证状态与对应身份、实际 kernel 入口和命令发布调用时间上下界。
- `dual_timeline.jsonl`：`state_contract_*` 记录。使用 `capture_mono/capture_sim`，不要用延迟写出的 `mono/sim`。
- `delivery_diagnostics/snapshots/`：后端实际使用的冻结全体投影圆盘与距离场，供独立三端点复核；不等于原生3D碰撞地图或真实物理世界的独立真值。

里程计用完整 ROS1 序列化的标准 FNV1a64、seq、纳秒时间戳和坐标系联合核对，不以最近CSV行替代。planner回调入口不是网络到达；publish调用不是机器人已执行；kernel返回0也不等于候选可行。

新 C++ 快照 I/O 延至请求最终决定后；Python周期记录在本周期命令发布后、锁外。记录仍可能影响后续时序，因此本批不是无扰动性能基准。

## 复现与回退

新目录才可构建/运行，不覆盖旧数据：

```bash
python3 experiments/online_remani_validation/build_state_contract.py --self-test --out <新构建目录> --compile
python3 experiments/online_remani_validation/run_state_contract.py --build <新构建目录> --out <新结果目录>
python3 experiments/online_remani_validation/report_state_contract.py --root <已完成结果目录> --out <新报告目录> --plot
```

编译、绘图不与在线测量并行。绘图前检查全部记录进程退出且11441关闭。实车 `src/devel` 未修改，MPPI库未改，不需要回滚生产代码；选择旧独立构建即可回退。不开新Codex会话，不安排本轮关机。

## 2026-10-23结果入口

两组结果已完成：`results/state_contract_20261023_01/`。简明总结见`results/20261023_起点交付一致性实验总结.txt`，关键图见该批`findings_final/key_findings.png`；`report/summary.json`保留完整精确关联。

旧组实际命中低速倒退改正向问题，新组没有在非零低速区间发起请求，不能声称signed修复已在线贯穿。另确认交付起点过时及快慢层静态余量恢复规则不一致：共用几何/距离场/参数不等于可行性合同已统一。实验仅诊断，没有据此放宽慢层门限或修改生产策略。
