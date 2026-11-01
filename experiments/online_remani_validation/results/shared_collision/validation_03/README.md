# REMANI / V12：共用碰撞模型对照

本次为追加验证：6次新在线试验 + 18项同一当前研究批次已经完成的对照记录复用。不是24次全部重新运行，也不是从旧研究中选择历史成功案例。
复用对照来自[完整原始对照批次](<../validation_02/README.md>)；原始文件只读，新的派生分析保存在本目录 reused_analysis 中。每项 record_origin 和 original_record_path 均保留。
MPPI采样种子固定但原生搜索与调度没有配对随机种子；复用记录不能被说成新运行或统计配对实验。

固定臂、原生 REMANI 在线规划和重规划、真实检测器、滞后差速数值底盘。不是 Gazebo 或实车验收。
四组区分局部执行器与原生碰撞几何变化：原 REMANI、原 V12、统一模型 REMANI、统一模型 V12。
这里的 REMANI baseline 是当前工程使用原生规划器、控制器和 Tracker 并关闭 MPPI；不是未经修改的上游 REMANI-Planner。
统一版共用同一投影、同一测量快照、0.10 m 硬余量及三次碰撞代价。原生优化仍使用冻结当前帧；局部控制按候选时刻做CV外推，时域策略没有被强行变成一样。
V12 统一版是碰撞成本、暖启动清空、回接权重及终点模式的联合改进；本四组对照不能把全部差异单独归因于碰撞代价。
局部参考改变后清空旧绕行暖启动；安全清空路线恢复参考约束，实际预测冲突仍可暂时绕行。

[对照摘要图](comparison_summary.png) · [静态参考误差](static_reference_error.png)

| 场景/版本 | 结果 | 实际静止/s | 新参考后误差P95/m | 最小物理净距/m |
| --- | --- | ---: | ---: | ---: |
| empty/remani | goal | 0.000 | — | 1.086 |
| empty/v12 | goal | 0.000 | — | 1.095 |
| empty/remani_shared | goal | 0.000 | — | 1.086 |
| empty/v12_shared | goal | 0.000 | — | 1.095 |
| static/v12_shared | goal | 0.672 | 0.050 | 0.112 |
| static/remani_shared | collision | 44.117 | — | -0.014 |
| static/v12 | goal | 0.403 | 0.224 | 0.170 |
| static/remani | timeout | 34.687 | 0.096 | 0.058 |
| dynamic_single/remani | collision | 0.000 | — | -0.001 |
| dynamic_single/v12 | goal | 0.201 | 0.004 | 0.313 |
| dynamic_single/remani_shared | collision | 0.301 | — | -0.002 |
| dynamic_single/v12_shared | goal | 0.663 | 0.038 | 0.252 |
| dynamic_head_on_center/v12_shared | goal | 0.000 | 0.075 | 0.244 |
| dynamic_head_on_center/remani_shared | collision | 1.146 | — | -0.001 |
| dynamic_head_on_center/v12 | goal | 0.000 | — | 0.262 |
| dynamic_head_on_center/remani | collision | 0.000 | — | -0.003 |
| dynamic_four/remani | collision | 0.000 | — | -0.000 |
| dynamic_four/v12 | goal | 0.068 | 0.041 | 0.319 |
| dynamic_four/remani_shared | collision | 0.366 | — | -0.004 |
| dynamic_four/v12_shared | goal | 0.560 | 0.014 | 0.251 |
| static_repeat/v12_shared | timeout | 9.136 | 0.179 | 0.120 |
| static_repeat/remani_shared | timeout | 43.831 | — | 0.002 |
| static_repeat/v12 | goal | 0.523 | 0.217 | 0.134 |
| static_repeat/remani | timeout | 35.560 | 0.100 | 0.062 |

## 每个场景的独立 GIF

- empty：[原版对照](comparison/empty/original/comparison.gif) · [统一碰撞对照](comparison/empty/shared/comparison.gif) · [V12改动前后](comparison/empty/v12_change/comparison.gif) · [MPPI搜索](empty/v12_shared/visualization/search_zoom.gif)
- static：[原版对照](comparison/static/original/comparison.gif) · [统一碰撞对照](comparison/static/shared/comparison.gif) · [V12改动前后](comparison/static/v12_change/comparison.gif) · [MPPI搜索](static/v12_shared/visualization/search_zoom.gif)
- dynamic_single：[原版对照](comparison/dynamic_single/original/comparison.gif) · [统一碰撞对照](comparison/dynamic_single/shared/comparison.gif) · [V12改动前后](comparison/dynamic_single/v12_change/comparison.gif) · [MPPI搜索](dynamic_single/v12_shared/visualization/search_zoom.gif)
- dynamic_head_on_center：[原版对照](comparison/dynamic_head_on_center/original/comparison.gif) · [统一碰撞对照](comparison/dynamic_head_on_center/shared/comparison.gif) · [V12改动前后](comparison/dynamic_head_on_center/v12_change/comparison.gif) · [MPPI搜索](dynamic_head_on_center/v12_shared/visualization/search_zoom.gif)
- dynamic_four：[原版对照](comparison/dynamic_four/original/comparison.gif) · [统一碰撞对照](comparison/dynamic_four/shared/comparison.gif) · [V12改动前后](comparison/dynamic_four/v12_change/comparison.gif) · [MPPI搜索](dynamic_four/v12_shared/visualization/search_zoom.gif)

## 指标和限制

静止指标同时要求平移和角速度低，且距离目标>0.35m，避免把原地转向当作停车。
新参考后误差剔除交付后0.5s和近终点阶段；额外记录无预测冲突时的误差。最近几何距离与连续分支距离均保留，不能把“贴住某条旧分支”当作按序跟踪。
首个障碍出现后的新参考时差，不自动等同障碍触发重规划时延；需结合原生事件日志解释。
全部失败、超时、投影碰撞和进程故障均保留；原生随机搜索未做配对种子，不作统计优势或任意场景安全声明。
原生-only参考图来自实际发布多项式的回放，局部组来自实际接收事务；不伪造控制器接收确认。
静态软碰撞代价作用区是0.12m，硬余量是0.10m。运动目标软代价另加 min(0.12m, 0.8s×目标速度)，且两层使用同一公式；该速度相关舒适余量不改变硬碰撞外形或引入CV不确定性膨胀。
共同的0.02m软/硬边界差用于减少优化器在硬边界内收敛后被拒绝；动态软区通常大于0.12m，不能表述成所有目标恒定0.12m。
原生规划从一次新鲜快照启动并在完整优化期间冻结输入，交付参考时输入年龄可能超过0.35s。这是指导轨迹的冻结输入，不是当下执行批准；原生非优化安全查询使用最新快照，局部执行批准必须通过当前帧健康和≤0.35s新鲜度检查。
继承的静态余量恢复存在明确例外：无预测障碍目标、已经进入0.10m余量带但物理净距仍为正时，可批准净距不减、最终恢复完整余量的退出轨迹；备用制动只要求保持正物理净距，不要求制动过程中退出余量带。不是允许物理碰撞，也不能宣称所有获准轨迹每一点始终≥0.10m。
共享输入审计仅核对已记录的传输、模型、序号、健康和年龄，不重算碰撞或制动，也不构成安全证书。没有原生交付或局部获准执行时明确记为覆盖不足，不能把“无执行且无错误”说成执行验证通过；原生-only不验证局部执行。
控制、绘图和原始数据均按场景/版本独立归档；重绘不参与控制决策。

## 共享输入审计覆盖

| 场景/版本 | 审计状态 | 输入契约 | 原生参考段 | 局部获准记录 |
| --- | --- | --- | ---: | ---: |
| empty/remani_shared | checked_native_input_only | 通过（仅上述输入覆盖） | 1 | 0 |
| empty/v12_shared | checked_local_and_native_inputs | 通过（仅上述输入覆盖） | 1 | 342 |
| static/v12_shared | checked_local_and_native_inputs | 通过（仅上述输入覆盖） | 3 | 640 |
| static/remani_shared | checked_native_input_only | 通过（仅上述输入覆盖） | 1 | 0 |
| dynamic_single/remani_shared | checked_native_input_only | 通过（仅上述输入覆盖） | 1 | 0 |
| dynamic_single/v12_shared | checked_local_and_native_inputs | 通过（仅上述输入覆盖） | 3 | 581 |
| dynamic_head_on_center/v12_shared | checked_local_and_native_inputs | 通过（仅上述输入覆盖） | 2 | 557 |
| dynamic_head_on_center/remani_shared | checked_native_input_only | 通过（仅上述输入覆盖） | 1 | 0 |
| dynamic_four/remani_shared | checked_native_input_only | 通过（仅上述输入覆盖） | 1 | 0 |
| dynamic_four/v12_shared | checked_local_and_native_inputs | 通过（仅上述输入覆盖） | 2 | 562 |
| static_repeat/v12_shared | checked_local_and_native_inputs | 通过（仅上述输入覆盖） | 3 | 994 |
| static_repeat/remani_shared | checked_native_input_only | 通过（仅上述输入覆盖） | 1 | 0 |

## 每项记录来源

| 场景/版本 | 来源 | 原始记录 |
| --- | --- | --- |
| empty/remani | reused_current_campaign_control | [原始结果](<../validation_02/empty/remani/result.json>) |
| empty/v12 | reused_current_campaign_control | [原始结果](<../validation_02/empty/v12/result.json>) |
| empty/remani_shared | reused_current_campaign_control | [原始结果](<../validation_02/empty/remani_shared/result.json>) |
| empty/v12_shared | new_online_trial | [原始结果](<empty/v12_shared/result.json>) |
| static/v12_shared | new_online_trial | [原始结果](<static/v12_shared/result.json>) |
| static/remani_shared | reused_current_campaign_control | [原始结果](<../validation_02/static/remani_shared/result.json>) |
| static/v12 | reused_current_campaign_control | [原始结果](<../validation_02/static/v12/result.json>) |
| static/remani | reused_current_campaign_control | [原始结果](<../validation_02/static/remani/result.json>) |
| dynamic_single/remani | reused_current_campaign_control | [原始结果](<../validation_02/dynamic_single/remani/result.json>) |
| dynamic_single/v12 | reused_current_campaign_control | [原始结果](<../validation_02/dynamic_single/v12/result.json>) |
| dynamic_single/remani_shared | reused_current_campaign_control | [原始结果](<../validation_02/dynamic_single/remani_shared/result.json>) |
| dynamic_single/v12_shared | new_online_trial | [原始结果](<dynamic_single/v12_shared/result.json>) |
| dynamic_head_on_center/v12_shared | new_online_trial | [原始结果](<dynamic_head_on_center/v12_shared/result.json>) |
| dynamic_head_on_center/remani_shared | reused_current_campaign_control | [原始结果](<../validation_02/dynamic_head_on_center/remani_shared/result.json>) |
| dynamic_head_on_center/v12 | reused_current_campaign_control | [原始结果](<../validation_02/dynamic_head_on_center/v12/result.json>) |
| dynamic_head_on_center/remani | reused_current_campaign_control | [原始结果](<../validation_02/dynamic_head_on_center/remani/result.json>) |
| dynamic_four/remani | reused_current_campaign_control | [原始结果](<../validation_02/dynamic_four/remani/result.json>) |
| dynamic_four/v12 | reused_current_campaign_control | [原始结果](<../validation_02/dynamic_four/v12/result.json>) |
| dynamic_four/remani_shared | reused_current_campaign_control | [原始结果](<../validation_02/dynamic_four/remani_shared/result.json>) |
| dynamic_four/v12_shared | new_online_trial | [原始结果](<dynamic_four/v12_shared/result.json>) |
| static_repeat/v12_shared | new_online_trial | [原始结果](<static_repeat/v12_shared/result.json>) |
| static_repeat/remani_shared | reused_current_campaign_control | [原始结果](<../validation_02/static_repeat/remani_shared/result.json>) |
| static_repeat/v12 | reused_current_campaign_control | [原始结果](<../validation_02/static_repeat/v12/result.json>) |
| static_repeat/remani | reused_current_campaign_control | [原始结果](<../validation_02/static_repeat/remani/result.json>) |

