# 2026-10-26：轮约束梯度与后端早停资格实验

用户授权开展最小新增实验，不重复旧预算/轮速矩阵，不处理Tracker，不接实车。

## 已有证据与本轮范围

- 10/25的713候选审计、四请求预算对照、10/23未成功的静态净距AL恢复均直接复用，不重新包装成新成果。
- 先核验原生轮软罚在真实失败点的局部v/a/j导数；梯度通过后，先单独检验总代价早停资格，不同时引入新目标或联合AL。
- 新停机对照仍是原L-BFGS、目标/梯度/初值/原生安全门/完整投影与双轮限制。只有诊断ON在past/delta停止时额外核验，不合格则继续原求解直到失败或原截止。
- 这叫“停止资格诊断”，不是已经实现完整联合可行性恢复。

## 梯度阶段

- 最终输出 `results/wheel_gradient_20261026_03`：170真实点局部梯度通过、36固定系数主积分时间链通过。
- 713候选、345682真实积分节点最低速度约0.0438454 m/s，未到1e-4截断区。
- 合成截断区确有导数不一致，但不能解释本批实际失败，不顺手更改后宣称交付改善。
- 不涵盖完整原生MINCO反传/触发稠密权重时长链，结论不能越界。
- `_01` JSON序列化失败保留；`_02`全覆盖结果保留；`_03`修正有限差分大数消去，复用哈希覆盖并重算全部差分，未重跑求解器。

## 停机阶段（已完成）

- 新构建 `builds/feasibility_stop_20261026_01`，父构建为 `seed_parameterization_20261025_01`；只在隔离副本改优化器、domain LBFGS及新增gate头，生产src/devel不动。
- 预声明NEW捕获seed11 ordinal2、seed12 ordinal2，各在同进程、同冻结输入下OFF/ON/OFF，全部0.6秒；不按失败种类筛选或补跑。
- `run_feasibility_stop.py`复用旧capture/graph/PID清理逻辑，仅改元数据和诊断环境；不发布私有候选。
- 开关 `FEASIBILITY_STOP_GATE=1` 还必须满足replay running与精确middle phase；正常前置规划始终OFF。
- 三臂共同增加最终原deadline资格复核，不把这个共同防线当成ON算法收益；检查为协作式，不能声称硬实时抢占。
- gate内原生警告、完整验收开销计入原预算；canonical后验审计仍在求解后预算外，不把首次后验合格当在线交付。
- 旧数据未保存完整原生地图/KD-tree，故只用历史数据审计局部导数；完整native硬门结论依赖本轮NEW同进程捕获，不宣传精确历史重放。

## 本轮文件所有权

- root：runner、runner契约测试、进度与总报告。
- wheel_gradient_audit：轮梯度数值审计及NEW stop报告脚本。
- feasibility_recovery_design：gate头、exact patch与源码契约测试（已冻结）。
- stop_gate_builder：NEW构建器及实际C++ gate/domain单元验证。

编译和计时仿真不重叠。所有旧构建、测量、失败和中间报告保留。本轮没有安排关机。

## 实测结果与结论

- 两个NEW捕获均成功，三臂首x/f/g一致、冻结上下文与原生动态观察一致；原生capture request ID均为3，但不是历史同一请求。
- seed11 OFF/ON/OFF最终判定163.87/602.31/160.62 ms；accepted7/54/7。OFF右轮加速度拒绝，ON预算超时。
- seed12对应160.82/602.25/161.01 ms；accepted6/58/6；同样OFF右轮加速度拒绝、ON超时。
- ON否决代价停止1/5次；gate开销0.60/9.38 ms，计入原预算。没有fatal/cache不一致或新增私有候选发布。
- 144个canonical候选，native/projected/both通过均0；部分轮违规减小，但继续原目标仍没有可用轨迹。
- 两组固定k0 gap+6.69/+9.03 mm，曲线内部K8已明显违规；不是旧AL那个起点先天不合格反例，也不只是密检查漏检。
- 9源码+4runner+18真实C+++11梯度测试通过。新build/source/parent/reused产物及98production完整性复查通过。
- 两launch均exit0、无required进程异常；主机侧核查记录PID全部退出，11441连接拒绝(已释放)。
- 正式图/报告 `results/feasibility_stop_report_20261026_01`，`evidence_valid=True`，输入/snapshot/全部canonical hash二次复核一致。
- 总摘要 `results/20261026_轮梯度与早停对照简要结论.txt`。

本轮证据只支持“所测真实局部梯度未见错误，单纯阻止cost-stop不足以提升交付”，不支持声称已完成联合恢复或稳定交付。
