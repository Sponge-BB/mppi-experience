# 固定机械臂、底盘 MPPI + CBF-QP：三组算法对照

本目录是**不接管机器人**的局部控制层数值闭环。没有 ROS 节点启动、网络连接、话题发布、串口访问或 Gazebo 运动命令。原实车代码和启动方式不变。

三组：原工程 `RealBaseTracker` / MPPI / MPPI+CBF-QP，均使用同一实验保护、参考路径、外部障碍运动、观测时间、执行模型和物理约束。

重要：原 Tracker 被直接调用，但**没有运行 REMANI 的搜索、轨迹优化、动态重规划和生产 TTC 状态机**。因此这不是完整 REMANI 的性能对照，不能用其静态障碍前停车声称原项目不会绕行。

## 运行

在已配置当前工作空间的终端执行（无需启动机器人或 roscore）：

```bash
cd /home/l/remani_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 experiments/base_mppi_cbf_compare/run.py --seeds 3 --samples 192
```

输出到新的带时间戳目录，绝不覆盖旧结果。依赖已有 ROS Noetic 开发头文件、g++、NumPy、Matplotlib；ROS 只用于离线解析 launch、消息结构和链接现有 CV 函数。

小规模检查：

```bash
python3 experiments/base_mppi_cbf_compare/run.py --seeds 1 --samples 96 \
  --scenarios empty,curve,reverse,crossing --profiles workspace
```

独立校验与重新绘图：

```bash
python3 experiments/base_mppi_cbf_compare/verify.py <结果目录>
python3 experiments/base_mppi_cbf_compare/run.py --render-only <结果目录>
```

## 与当前工程的联系

- 解析 `exp0_real.launch` 的**最终参数**，导出完整参数、采用值和来源哈希。
- 直接调用 `real_base_tracker.py` 的原类，以内存 `rospy` 适配替代所有发布/订阅/定时器。
- 直接编译 `dynamic_point_buffer.cpp` 的 CV 与预测不确定度静态函数，不启动该节点。
- 读取当前固定关节角、底盘尺寸、Piper 安装变换，复用已有 DH 采样导出器，校验最终参数未被覆盖成另一套几何。
- 整机二维覆盖包含固定机械臂，额外工具余量 0.10 m 未经 mesh 标定；不是“把机械臂删掉”。
- 同时限制车体速度/加速度和左右轮速度/加速度，保留倒退。

## 场景与统计

十个场景：空旷、弯曲参考、后方同朝向目标、新静态障碍、横穿、延迟与掉帧、封闭通道、出现即重叠的边界测试、留有净距的近距离出现、执行滞后失配。

`workspace` 十场景使用当前实车误差参数。`sensitivity` 只针对三个移动障碍场景，假设更小的误差包络，独立成图，**不能冒充实车参数或感知精度**。默认三随机种子共 117 次运行；确定性 Tracker 的重复种子仅用于配对，并非独立统计样本。

首批 `results/full_01` 为原先九场景共 108 次；发现 `late_intrusion` 出现即重叠后，保留该批原始数据并补测 `close_clearance`。不能把瞬间生成在机器人内部的障碍用来证明控制器反应太慢。

三组均使用 20 Hz 数值控制周期（实车 Tracker 定时器实际为 50 Hz），MPPI 192 候选、1.5 s 预测时域；执行滞后假设 0.12 s，失配场景真值为 0.28 s。没有复现实车线程、地图更新、激光检测和底盘通信延迟。

共同保护是实验定义的反应段＋制动预测检查，不是复制生产 TTC。QP 不可行、观测过期、计算超过 50 ms 请求制动；**制动仍可能不安全**，真值碰撞照常统计。

MPPI 为启发候选引导的采样加权原型，含终点位置/朝向分阶段收敛。CBF 为顺序线性化离散屏障 QP，有非线性与周期内采样复验，不宣称连续时间或鲁棒安全保证。仅末端串联过滤，不宣称已实现 CBF-aware MPPI。

终点切换半径使用一个预测时域的可用直行距离（至少 0.20 m），先消除位置误差再对齐朝向。两个 MPPI 模式共用，不通过放宽到点容差宣称成功。

## 图片和数据

- `trajectories_workspace.png`：当前参数三组全部轨迹，失败也保留。
- `summary_workspace.png`：到达、低速停留、最小净距、计算耗时。
- `crossing_snapshots.png`：同一时刻三组机器人外形与障碍物位置。
- `detail_*.png`：固定种子 0 的速度、净距、耗时、保护介入和 QP 失败。
- `prediction_envelopes.png`：当前参数与小误差假设的预测外形比较。
- `geometry.png`：固定机械臂和底盘占据。
- `REPORT.md` / `runs.csv` / `metrics.json` / 逐周期 CSV：完整可追溯统计。
- `verification.txt`：约束、几何、统计一致性和生产文件未改动检查。
- `independent_audit.json`：独立 Python 实现按 5 ms 重积分全部种子 0 的动作，复核状态和同时间真值净距。

三组同步动画（固定种子 0，可选 `--scenario new_static`）：

```bash
python3 experiments/base_mppi_cbf_compare/animate.py <结果目录> --scenario crossing
```

每条 CSV 状态在区间末端，控制量作用于刚结束的区间。图片来自实际执行的计算结果，不是 AI 生成示意图。耗时只涵盖离线局部计算，不能据此宣称实车达到该频率。

## 回退

只新增本目录。移走 `experiments/base_mppi_cbf_compare/` 即撤回本轮实验；没有实车配置需要恢复。历史 `experiments/mppi_cbf_validation/` 结果不覆盖。
