# REMANI + MPPI 实验工程

当前 REMANI 移动机械臂工程，以及固定机械臂、底盘 MPPI / CBF-QP 的历次算法实验归档。
这是工作空间当前文件快照，不是原版 REMANI 仓库，也不表示所有历史版本都通过验收。

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

`experiments/` 保留各版本实验脚本、参数、状态/障碍物/参考记录、检查报告、搜索快照、PNG/GIF，
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
