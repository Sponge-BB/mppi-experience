# REMANI + MPPI 实验工程

当前 REMANI 移动机械臂工程，以及固定机械臂、底盘 MPPI / CBF-QP 的历次算法实验归档。
这是工作空间当前文件快照，不是原版 REMANI 仓库，也不表示所有历史版本都通过验收。

## 2026-11-01 更新

补充轨迹交付窗口、中段横穿和短前缀回接等后续实验，保留脚本、隔离源码快照、原始数值记录、报告和 GIF。

- [交付窗口：结论](experiments/online_remani_validation/results/delivery_window_20261101_01/report/本轮结论.txt) · [迎面对照 GIF](experiments/online_remani_validation/results/delivery_window_20261101_01/report/迎面对照.gif)
- [中段横穿：结论](experiments/online_remani_validation/results/delivery_midcross_20261101_01/report/本轮结论.txt) · [对照 GIF](experiments/online_remani_validation/results/delivery_midcross_20261101_01/report/中段横穿对照.gif)
- [短前缀回接：结论](experiments/online_remani_validation/results/short_prefix_rejoin_20261101_01/report/本轮结论.txt) · [独立核查](experiments/online_remani_validation/results/short_prefix_rejoin_20261101_01/report/独立核查.txt) · [对照 GIF](experiments/online_remani_validation/results/short_prefix_rejoin_20261101_01/report/短前缀衔接对照.gif)

迎面单次对照中，仅延长慢层冻结预测窗口后，首次反馈到实际有效新控制为 4.406→1.910 s。
中段横穿为 6.057→4.271 s，但实际路程和运动时间增加，仍不能认为协同问题已经解决。
最新短前缀实验的新策略两组均未触发，不能据此宣称策略有效；核查结果和失败记录一并保留。
以上是固定机械臂的隔离 ROS 数值闭环，不是 Gazebo 或实车验收，也不是总体成功率结论。
本次未替换实车代码或生产二进制。私人会话导出和 Windows 开题材料包不在公开上传范围。

## 2026-10-31 更新

本次同步当前工程源码，以及上次归档后新增的轨迹交付、碰撞球简化、计算耗时和 MPPI 协同实验。
实车默认入口与隔离实验版本分别保留，历史 V12 结果仍可查阅。

- [最新一轮实验总结（TXT）](experiments/online_remani_validation/results/delivery_next_20261031_01/本轮实验总结.txt)
- [静态挡路对照 GIF](experiments/online_remani_validation/results/delivery_next_20261031_01/report/静态挡路.gif)
- [中段横穿对照 GIF](experiments/online_remani_validation/results/delivery_next_20261031_01/report/中段横穿.gif)
- [轨迹交付时间对照图](experiments/online_remani_validation/results/delivery_next_20261031_01/report/轨迹交付对照.png)
- [最新隔离构建与运行入口](experiments/online_remani_validation/build_delivery_next.py)
- [在线对照实验脚本](experiments/online_remani_validation/run_delivery_next_online.py)

本轮在两组均修正 MPPI 换向段关联的条件下，对比优化器原退出方式与合格候选提前退出。
重规划触发至 MPPI 实际采用新参考，静态为 1.53→0.66 s，横穿为 4.60→1.04 s。
四次 ROS 数值闭环均到达目标，球体采样检查未发现碰撞；每场景每组只有一次运行，
改进组动态场景仍有最长约 0.8 s 的低速爬行。它不是 Gazebo 或实车验收，完整限制见本轮总结。

## 快速入口

- [完整在线实验与版本索引](experiments/online_remani_validation/README.md)
- [V12 改动、限制与复现说明](experiments/online_remani_validation/NATIVE_V12.md)
- [V12 全部结果和分场景 GIF](experiments/online_remani_validation/results/native_v12/validation_03/README.md)
- [V12 正迎面：V11/V12 对照 GIF](experiments/online_remani_validation/results/native_v12/validation_03/comparison/08_head_on_center_h3/comparison.gif)
- [V12 四障碍：MPPI 搜索过程 GIF](experiments/online_remani_validation/results/native_v12/validation_03/04_four_dense_h3/visualization/search_zoom.gif)
- [V11 在线规划与局部执行说明](experiments/online_remani_validation/NATIVE_V11.md)
- [早期底盘 MPPI + CBF-QP 比较](experiments/base_mppi_cbf_compare/README.md)
- [底盘控制验证实验](experiments/mppi_cbf_validation/README.md)
- [上传范围与使用注意](PUBLICATION_SCOPE.md)
- [文件完整性清单（SHA-256）](UPLOAD_MANIFEST.json)

`experiments/` 保留各版本实验脚本、参数、状态/障碍物/参考记录、检查报告、搜索快照、PNG/GIF 和实验规划器输出日志，
包括中途失败和被否决的开发版本。请从各版本 README 进入，不要只看成功动画。

## 目录

```text
src/                         当前 ROS 工程源码、launch、配置和机器人模型
Utils/                       工具与 ROS 消息包
NEXTE_Sentry_Nav/             感知/导航相关源码快照
piper_kinematics/             机械臂运动学相关源码
REMANI-Planner/               原生 baseline 参考代码
NeuPAN/                      思想参考项目的本地源码快照
docs/                        工程文档
experiments/                 各类 MPPI / CBF / REMANI 联合实验及完整结果
```

嵌套项目按普通目录保存，原有 LICENSE 和作者说明保留；没有把它们变成不可直接获取内容的 submodule。

## 实验范围

最终 V12 使用原生 REMANI 在线规划/重规划、点云检测、固定臂底盘 MPPI，
但属于隔离 ROS 数值仿真，不是 Gazebo 或实车安全验收。
V12 主场景默认局部 CV 外推 3 s，原生规划层配置仍为 2 s；无 CBF-QP、无不确定性半径。
保留同一车体投影、轮速限制、0.10 m 余量及发布前轨迹/制动检查。

V12 本轮 8 个动态主场景到达，空旷/静态、重复迎面和参考延迟试验也到达；
2 s 正迎面及仅改感知的两项消融仍投影碰撞。结果只支持这些有限场景，不代表任意场景、实车或完全不停顿。
规划与执行的碰撞几何、预测触发和新参考回接仍有待改进，具体限制在版本说明中列出。

## 在另一台设备使用

```bash
git clone https://github.com/Sponge-BB/mppi-experience.git
cd mppi-experience
source /opt/ros/noetic/setup.bash
```

这是 ROS Noetic / catkin 工作空间快照；需自行安装 ROS、各包依赖并重新编译。
`build/`、`devel/`、机器本地大型地图和历史编译二进制没有上传。
历史隔离构建的 `source/` 和 manifest 保留，但构建链仍须先重建所需父构建，不能假设已有可执行文件。
历史参数、JSON 审核和文档可能保留原机器绝对路径，迁移时应调整路径，而非改写原始证据。

不要未经硬件标定、影子验证和安全检查直接启动实车控制。
