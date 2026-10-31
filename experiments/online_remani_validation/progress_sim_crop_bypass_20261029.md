# 2026-10-29 仿真自车过滤单因素调整

用户授权：按已提出方案实施实验。只调整当前固定臂ROS数值仿真，不改实车、生产src/devel；不含上传、关机或实车控制。

## 本轮唯一变化

当前synthetic cloud只有墙与障碍圆柱，没有机器人自身回波。新trial删除`crop_box_filter`节点，让`passthrough_z`直接输出到`/velodyne_points`。参数YAML中保留的crop配置已无运行消费者，不能据此说crop仍启用。

完整REMANI/MPPI、160球、CV预测、速度/轮约束、后备刹停检查、200ms期限/250ms租约和seed11均不变。复用`builds/execution_timing_20261028_01`，无重编译；保留与旧诊断相同的分段计时和卡顿栈。

脚本：`run_sim_crop_bypass.py`。单次新闭环：`results/sim_crop_bypass_20261029_01/head_on_bypass_seed11`。历史对照为`results/execution_timing_diagnosis_20261028_01/online_probe`，不是同期重复试验或精确同输入重放。

## 结果

- 唯一新闭环完成，接口审计有效，生产源码/构建哈希未变，进程正常回收；仍在21.316s发生三维采样接触，不能称避障成功。
- 实际launch/图确认crop未运行、输入旁路生效；原先那一层误删已消除，不等于整个感知链完美。
- 新终点快照421有93个动态点，CV约(-0.18,0,0)m/s。全机dynamic检查净距-4.34cm，说明这次已看到危险；物理采样接触-3.18mm。不可套用旧终点“全机仍+5.35cm”的误判叙述。
- 新actor窗口：5个控制周期出现code5后备刹停拒绝、共38次候选；之后10轮无可行候选。1个deadline标签也已经验收失败，没有安全非零提案仅因超时被丢弃。
- 发目标后普通Twist发布至下轮开始最大约61.88ms，没有复现2.82s空档；不能因此宣称偶发等待已修复。
- REMANI的3次刷新在目标预检拒绝，优化器调用0次，只用到初始参考。这一轮不能把无新轨迹归因于优化器计算超时。

## 复核和产物

`audits/audit_sim_crop_bypass.py`独立计算新终点真值/动态点云几何，输出`results/sim_crop_bypass_report_20261029_01/perception_endpoint_audit.json`，不做全流水线重放。

`report_sim_crop_bypass.py`输出同目录TXT、JSON与唯一关键并排GIF，包含整机投影、真实REMANI参考和真实MPPI搜索子集。计时节点退出后才绘图；新记录380个cycle对379条diag，不能直接zip，缺失首条已记入，按时间/原因/验收码关联并排除可识别接触后命令。

6项针对性测试通过：仿真路由单因素、probe AST不变、内存阶段记录、原timing配置、报告缺行/重复时刻/终点筛选、验收失败与纯超期分类。

下一项应围绕可执行候选/后备动作与目标选择入口处理，不能用继续加预算或删除安全检查冒充避障修复。本轮未加入这些新因素。
