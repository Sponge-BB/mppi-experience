# V12 实验归档

原生 REMANI 在线参考 + 点云动态检测 + 固定臂底盘 MPPI，隔离 ROS 数值仿真；不是 Gazebo/实车。
完整方案、边界和复现方法见 [NATIVE_V12.md](../../NATIVE_V12.md)。

## 正式候选

[validation_03](validation_03/README.md)：构建 `native_v12_07`，预先声明 17 项在线试验。
各场景分别保存 `visualization/overview.gif`、`search_zoom.gif`、`search_snapshot.png`，
V11 对照在该批次 `comparison/`，没有删除旧碰撞/超时案例。
以实际汇总和 `suite_audit.json` 为准；主场景、短时域失败消融、仅感知消融不能混在一起报成功率。
本轮8个主场景均到达；17项中15项到达、2项消融投影碰撞。全部参考/执行审核通过、无原生必需进程故障。
2616个搜索快照、34个场景GIF与8个对照GIF均完成检查；98项生产源码未变。

## 开发及失败

- [validation_01](validation_01/README.md)：构建 03，错误地给横穿目标持续侧向奖励；多个投影碰撞和终点超时，保留。
- [validation_02](validation_02/README.md)：构建 05，8个主场景中7个到达，偏置迎面绕过后终点停滞；
  中心迎面4 s时域也终点超时，2 s及仅感知消融投影碰撞，保留完整 GIF。
- `development_01/02`：单迎面开发记录，不代表全场景验收。
- `closed_loop_contracts_06_02.json`：构建 06 的理想控制器回归，19 cm终点后纠偏失败。
- `closed_loop_contracts_06.json`：测试报告序列化错误留下的不完整文件，不是有效结果。

## 独立回归检查

- [kernel_contracts_07.json](kernel_contracts_07.json)：2/3/4 s的实际控制展开、远处无冲突目标、合法倒车、全占据拒绝。
- [execution_contracts_07_02.json](execution_contracts_07_02.json)：7项目标事务检查及8项执行/制动内核检查，反馈前导出并核对真实首命令。
- [closed_loop_contracts_07_02.json](closed_loop_contracts_07_02.json)：7项理想 CV/冻结参考控制回归，包含真实失败参考和终点13/19 cm纠偏。
  明确执行顺序为求解→导出原始序列→首命令一致性断言→验证→一次反馈→执行。
- 较早 `closed_loop_contracts_06_02/07.json` 导出发生在反馈后，可能验证了已平移的热启动；
  保留为开发诊断，不再引用为精确执行序列验证。原生在线执行的反馈顺序本来正确，在线试验不受影响。
- 感知回归由 `test_perception_v12.py --show-original-regression` 复现：原驻留背景错误先失败，修正版30项C++与2项Python检查通过。

理想回归不含在线点云、真实 REMANI 重规划或调度延迟，不能代替在线试验或实车安全验证。
