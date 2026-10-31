# 候选B接入

- 用户选择47球候选；仅接入独立数值实验，未修改生产源码、devel、实车参数或历史构建。
- 子构建：`builds/urdf_spheres_b_20261029_01`；从`execution_timing_20261028_01`继承实验算法，MMConfig原始输入来自`fixed_arm_01`。
- 模型、优化器、所有search/planner消费者重新编译；每个固定球使用自己的半径，臂20球不改。
- 仅custom Piper软base-arm检查与现有硬检查的连杆起点对齐；不改变外部检查或旧140球模式。
- 15项Python检查、实际C++位置/梯度/自碰/legacy回退测试通过。
- 同输入碰撞查询微基准已完成，不把查询提速说成完整MPPI或REMANI提速。
- 25秒发目标后短闭环已完成，实际导出47+20球，接口审计通过，设定时间结束，未完成6米全程。
- 结果及说明：`results/urdf_spheres_b_validation_20261029_01/候选B接入总结.txt`。
- 运行入口：`run_urdf_spheres_b.py`；默认65秒，重复运行需指定新的`--out`目录。
- 旧构建与`run_watchdog_off.py`保留；未安排关机或上传。

## 后续：用户要求同步实车

- 已将模型及对应半径消费者最小移植到生产src；没有拷贝SIL固定臂优化器或实验执行链。
- 实车仍为REMANI＋Tracker，`exp0_real.launch`默认启用`use_urdf_collision_b:=true`；显式false恢复旧模型并抑制残留参数。
- 回退包：`backups/real_urdf_spheres_b_20261029_01`；包含修改前源码、运行ELF及目标对象，不依赖Git。
- 部署状态与操作说明以该目录`实车启用与回退.txt`、`deployed.json`为准；未启动实车运动。
