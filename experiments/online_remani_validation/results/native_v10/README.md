# V10 执行权限实验记录

固定机械臂，底盘 MPPI，无 CBF-QP；真实 REMANI 在线参考 + 隔离 ROS 数值被控对象。
**动态验证通过，静态到达验证失败，未推广到实车或默认版本。**

## 最终构建 native_v10_06

| 案例 | 到达 | 碰撞记录 | 最小保守净间距 | 参考交付等待期间执行 |
| --- | --- | --- | --- | --- |
| `crossing_delayed_06` | 是 | 无 | 0.147 m | 延迟约 3 s，60/60 周期前进 |
| `static_06` | 否，超时 | 无 | 0.141 m | 新参考已收到，仍局部停滞 |
| `static_delayed_06` | 否，超时 | 无 | 0.139 m | 延迟约 3 s，42/61 周期前进；随后停滞 |
| `empty_06` | 是 | 无 | 1.095 m | 正常空旷跟踪 |
| `crossing_runtime_06` | 是 | 无 | 0.127 m | 最终运行时：延迟約 3 s，60/60 周期前进 |
| `static_runtime_06` | 否，超时 | 无 | 0.157 m | 最终运行时：新参考已收到，仍停滞 |

“无碰撞记录”针对该数值场景的保守投影判据，不是实车安全结论。
上述完成审核的案例均通过原生参考审核、局部执行审核，但审核通过不等于到达目标。
单次试验、在线线程非确定，不作为统计显著性能提升结论。
空旷回归在 `empty_06` 保存。

## 图像

- [最终动态 V9/V10 对照 GIF](comparison/crossing_runtime/comparison.gif)
- [实际运动与参考更新等待关系](comparison/crossing_runtime/execution_timeline.png)
- [最终动态执行过程](crossing_runtime_06/native_reference.gif)
- [静态失败过程](static_06/native_reference.gif)
- [静态三秒延迟实验](static_delayed_06/native_reference.gif)

单案例图中橙为实际 REMANI 参考，绿为审核通过的局部轨迹，蓝为实际运动。
每个实验的 GIF 位于自己的文件夹，原始轨迹、指令、日志、失败状态均保留。

## 过程记录

- `crossing_04`：初版到达，取消停车消失；发现末尾 CV 时域随求解时间失效，后续已修复。
- `crossing_delayed_05`：到达，但只有初始参考，没有发生延迟更新；**不能算三秒延迟验收**。
- `static_05`：超时；定位并修复协同重规划入口重复捕获 ESDF 边界。
- `static_delayed_06`：真实延迟已触发，执行接管有效，但静态绕行未完成。
- `static_06`：不注入参考延迟也失败；留有 `local_stall.npz`，排除了只由三秒注入造成的问题。
- `crossing_delayed_06`：最终动态延迟验收，到达，60 个延迟周期均前进。
- `crossing_runtime_06`：参考解码加速及交接时间戳修复后的最终运行时复测；到达，延迟期间 60/60 周期前进。
- `static_runtime_06`：相同最终运行时仍静态停滞，保留失败快照及 GIF。
- `contracts_final_runtime_06.json`：7 个任务/事务测试、8 个内核检查；既有 `test_contracts.py` 的 13 项也通过。

`builds/native_v10_01` 为标记匹配失败的未完成构建；02/03 为开发过程构建；
04/05 修复局部校验有效期；06 修复协同 ESDF 等待和实测状态接续。
构建/试验没有覆盖旧结果，也没有替换生产文件。

实现及启动/回退见 [NATIVE_V10.md](../../NATIVE_V10.md)。
