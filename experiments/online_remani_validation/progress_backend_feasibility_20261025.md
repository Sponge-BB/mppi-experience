# 修复后种子：相同输入的后端可行性诊断

## 事实与本轮问题

上轮静态OFF/ON发现：修复后8次重规划均进入后端，但4次budget_timeout、2次left_wheel_acceleration。闭环状态分叉，不能用跨组时间差说明因果；历史完整原生地图内存也未落盘，不能宣称精确重放那6次历史失败。

本轮只诊断，不改算法/限制：原build `seed_parameterization_20261025_01`，固定启用参数化修复。检查原始优化坐标k0候选和已接受迭代的完整双轮速度/加速度、原生IsTrajSafe、冻结全投影20ms检查；看长预算是否得到同时过关的候选，而不只看solver返回ok。

## 预先固定的实验

四个新捕获，次序 seed11 ordinal1、seed12 ordinal2、seed11 ordinal2、seed12 ordinal1。ordinal指“已有reference后第N个符合条件的normal/0.6s后端入口”，不是FSM请求序号或历史request_id；实际FSM、capture时间/场景是否出现须由日志核对。

每次在同进程冻结输入和观测后依次执行0.6、0.6、1.5秒三个预算，轮速/轮加速度/碰撞外形/余量不变，MPPI不改、预测soft weights=0/0、admission与额外clearance recovery关闭。重复0.6用于观察预算抖动，1.5只用于可行性诊断，不是在线预算修改。

只用localhost:11441和模拟控制话题。`local_execution_owner=true`时simulator实际消费 `/sim/local_execution_cmd`（Stamped），旧 `/sim/cmd_vel` 仍记录；两者均必须只有 `/base_mppi` 发布。固定机械臂的原生ROS数值SIL，非Gazebo/实车；诊断期间暂停本试次simulator/MPPI，候选不发布。若未触达目标捕获也保留，不挑选失败/成功重跑；已有结果不改。

## 证据边界

核验同initial x、首cost/gradient、边界、frame、prediction、seq、模拟时间及动态点观测一致。并未逐字节导出所有原生地图/优化器隐状态，不能以此声称完整内部状态冻结。

原生硬检查跳过首0.05秒、使用离散网格；独立轮速扫描和全投影检查将分别标清采样口径。k0指初始MINCO候选，不是A*原折线。首次可行迭代来自求解后重建/检查，不等于在线可交付时刻。所有诊断/画图/编译避免与测量重叠。

MPPI/simulator暂停会改变CPU负载，本轮不是在线算力竞争或性能对照。探针记录已接受x/g的拷贝仍计入L-BFGS预算，canonical事后重建不计入；首次fg相同不等于所有原生地图/优化器隐状态都被证明完全相同。若observed context/first fg检查不通过，该job不得用于相同输入结论，原数据保留且不补挑样。

## 进度

runner/双轮审计/报告已完成，合计52项测试通过后冻结所有根Python源。没有重新编译或改算法。`results/backend_feasibility_20261025_01`四个预定捕获均已完成；12次求解713个canonical候选全部保留，未触发记录截断。

独立核验在 `results/backend_feasibility_verification_20261025_01`：98生产、36隔离源码、二进制/库、4份runtime、24份压缩归档、60个节点PID退出、11441空闲均通过；独立重算输入/首x/g哈希及成本一致，无诊断候选发布。实际两个模拟命令话题所有权核验也通过。正在做完整候选投影及双轮后验扫描，尚不以原生solver返回ok代替完整可行结论。

后验分析已全部完成：`backend_iterate_audit_20261025_01` 和 `backend_kinematics_audit_20261025_01` 都是4/4证据有效；正式报告 `backend_feasibility_report_20261025_02` 的 `all_evidence_valid=true`，TXT/JSON/数据图已生成。四个捕获和其冻结帧均晚于场景出现，重复0.6s的接受数/原生结果一致，细小计时差异保留。

12次求解：6次budget_timeout、3次left_wheel_acceleration、1次left_wheel_speed、1次right_wheel_speed、1次native ok。713候选中41个native通过，全投影20ms通过为0，双过关为0。native-only成功来自seed12 ordinal2长预算，最终投影gap -5.72mm，最佳accepted gap -1.30mm，其K8也违规约0.764mm，不是纯粹密采样才发现。

四个初始MINCO曲线最大双轮约束比(相对包含原有5%容差的硬门)约1.440/1.585/1.399/1.505；长预算末候选为1.346/.952/1.399/1.330。t=0位置投影gap全正，wheel t0也在限内；不能混成固定起点已非法。初始曲线不合格不自动证明初始化是主因，需独立对照。

seed11 ordinal2三预算均LBFGS_STOP、约157ms、相同final x/cost。首个acc拒绝4.51149只超硬门约1.10%，但全程left speed峰值4.81676 vs3.444（超39.86%），left acc峰值5.58628 vs4.4625（超25.18%）；不能只看首个失败点就放宽约束。wheel为软惩罚，delta=1e-3/past=3的总代价改善停止不要求硬可行。仍不能单凭此区分权重/配点/局部最优。

统计口径：failure_trace.iterations由costFunctionCallback计数，含线搜索评估，不等于accepted迭代；本例20次cost回调对应6个accepted步。原生所有状态/算法不改。

测量后唯一root Python修改：报告模板“四轮约束”改为“双轮四项约束”，12项报告测试重跑通过，_01原报告保留，_02为正式文本（数值相同）。runtime原文件不动。该分析文案修正单独列入最终核验说明，不伪称root源码与测量快照仍原文完全相等。简要解释见 `results/20261025_固定输入后端诊断简要结论.txt`。

最终 `backend_feasibility_verification_20261025_01/final_note.json all_checks_passed=true`：四份runtime报告源只施加这一行文案替换即精确等当前root；其余文件无新变化；_01/_02除created和report_source_sha256外的结构/数值证据一致。独立复核初始轮约束峰值、t0位置/轮合格及首个失败≠最坏点均与原始记录一致。
