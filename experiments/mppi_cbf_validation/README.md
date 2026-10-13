# MPPI + CBF-QP 独立验证（不接管实车）

这是用于判断是否值得继续集成的最小实验，不是已部署的安全控制器。
所有新增代码、可执行文件、数据和图片都在本目录。没有修改 `src/`、launch、原控制器或 ROS 参数。
程序无 ROS 发布器、无网络连接、无底盘驱动接口；可在机器人不开机时运行。

## 一条命令运行

在 `/home/l/remani_ws` 下：

```bash
python3 experiments/mppi_cbf_validation/run.py --seeds 3 --samples 256
```

依赖本机已有的 g++、Python 3、NumPy、Matplotlib、PyYAML。读取历史 bag 时另需 ROS 的 rosbag Python 模块。
不需要 catkin 编译，不需要 roscore，不需要 GPU。默认新建带时间戳的结果目录；显式 `--out` 指定的目录若已存在会拒绝覆盖。

同时生成历史实车时间线（只读文件，**不是** `rosbag play`）：

```bash
python3 experiments/mppi_cbf_validation/run.py --seeds 3 --samples 256 \
  --bag test_logs/final_validation_20260916_144826.bag
```

快速冒烟测试：

```bash
python3 experiments/mppi_cbf_validation/run.py --seeds 1 --samples 128 \
  --scenarios empty,new_static,crossing
```

结果包含 `REPORT.md`、`trajectories.png`、`summary.png`、`detail_*.png`、`geometry.png`、CSV、JSON 与本次编译的独立可执行文件。
图中的蓝线是 MPPI，橙线是 MPPI + CBF-QP；详情图使用固定种子 0，不筛选成功样例。
轨迹图红色圆形是障碍物在标注时刻的位置；线条交叉不等于同一时刻发生碰撞，应结合详情图或动画判断。

已有实验只重新绘图：

```bash
python3 experiments/mppi_cbf_validation/run.py --render-only experiments/mppi_cbf_validation/results/full_01
python3 experiments/mppi_cbf_validation/test_results.py experiments/mppi_cbf_validation/results/full_01
python3 experiments/mppi_cbf_validation/animate.py experiments/mppi_cbf_validation/results/full_01 --scenario crossing
```

## 实验定义

同一场景下，两种方法使用相同的初始状态、障碍物运动、观测时序、物理限制和随机种子。
**没有把一个简化跟踪器称为原版 REMANI，也没有声称完成原版 A/B 对照。**

|场景|用途|
|---|---|
|empty|空旷直线到达；检查新增走停和摆动|
|near_miss|障碍在路旁，不构成碰撞；检查多余动作|
|new_static|1.5 s 后突然出现静态障碍，左右可绕行|
|crossing|已知匀速横穿障碍，CV 模型准确|
|delayed_crossing|横穿 + 150 ms 传输延迟 + 速度误差 + 短时掉帧|
|blocked|通道被封闭，预期停止而非强行到达|
|workspace_uncertainty|横穿，使用现有 exp0 的大预测不确定度参数|
|late_intrusion|3.5 s 后才出现的近路障碍，测试制动/绕行选择|

控制周期 0.05 s，MPPI 时域 18 × 0.1 s = 1.8 s。观测频率 10 Hz。
v、ω、a、角加速度上限分别为 0.30 m/s、0.80 rad/s、0.35 m/s²、1.00 rad/s²，来自当前 `real_speed.yaml` 的 Tracker 限制。
入口检查这些值是否漂移，若配置变更则拒绝悄悄沿用旧上限。

底盘运动模型：x_dot=v*cos(yaw)，y_dot=v*sin(yaw)，yaw_dot=ω。
速度目标通过一阶滞后与加速度限幅后执行，滞后时间常数暂设 0.12 s，**未经实车辨识**。
控制器 rollout 采用 10 ms 积分；独立仿真状态和碰撞检查采用 5 ms 积分。
用实际执行动作更新热启动，不以未执行的 MPPI 名义指令代替。

### 固定机械臂碰撞几何

底盘按 0.5 × 0.5 m 矩形的两个覆盖圆建模。机械臂采用 `exp0_param.yaml` 的初始姿态，
读取 `piper_param.yaml` 的安装位置，DH 和碰撞点与 `mm_config.cpp` 的 Piper 分支一致。
把固定机械臂碰撞点投影到 XY 后取覆盖圆，包含连杆采样半径，并额外加 0.10 m 未标定工具余量。
见 `geometry.json` 和 `geometry.png`。

这保留了固定姿态机械臂的平面占据，不是把机械臂删掉，但仍然不是经过 mesh 核验的实车整机模型。
不处理机械臂运动、高度分层、自碰撞和末端任务。因为采用投影，低矮障碍也会约束上方连杆，可能保守。

### MPPI 原型

相关高斯扰动 + 前向/左转/右转/制动/倒退候选，逐条推演速度目标序列，以温度加权得到动作序列。
这是带启发候选和约束裁剪的实用采样加权 MPPI 原型，不宣称严格的重要性采样无偏性。
代价包含路线偏离、终点距离、动作变化、时变障碍净距和碰撞罚项。
对加权后的序列重新计算代价，必要时使用最佳候选，避免安全候选的均值反而切过障碍。
**碰撞软代价不构成安全保证。**

CV 查询显式计入观测年龄：p(k)=p_obs+v_obs*(now-stamp+k*dt)。
不确定度为 e_p+e_v*tau+0.5*a_bound*tau²。
理想场景参数为 0.02、0.03、0.02；workspace_uncertainty 为 0.08、0.25、0.75。
后者单独列出，不能只展示较小误差包络下的好结果。

### CBF-QP 原型与边界

对于覆盖圆与障碍物，h=||p_robot-p_obs||²-(r_robot+r_obs+margin)²。
墙壁采用有符号距离约束。要求 h(x_next,t+dt) >= exp(-alpha*dt)*h(x,t)，alpha=1.2。
`x_next` 显式包含当前速度、加速度限制和执行滞后，因此不是对差速底盘生硬套用全向速度模型。

把下一步 h 对速度目标 (v_cmd,ω_cmd) 有限差分线性化，解两变量严格凸 QP：
min (u-u_nom)' diag(1,0.20) (u-u_nom)，约束线性化屏障及指令范围。
二维 QP 通过枚举内部点、边界投影、边界交点求解，不依赖外部 QP 库。
最多迭代 4 次，每次验证原始非线性屏障和周期内采样净距。

**这只是数值线性化离散屏障实验，不是鲁棒 CBF 或全时域安全证明。**
当状态已经不具备制动余量、预测误差超出包络、线性化退化或约束不可行时，过滤器仍可能失败。
失败、过期、计算超时请求有加速度限制的制动，并明确记录 infeasible/unverified；不会把制动自动标成安全。
CBF 没有被提前塞进每条 MPPI rollout，这是有意保留的最简单串联消融，便于测出两者不一致导致的停车。

### 统计不能混淆

- `reached`：24 s 内到达 0.10 m 位置范围且 |v|<0.04 m/s；本轮不考核终点精确航向。
- `collision`：任意 5 ms 真值检查净距小于 0。真值使用障碍真实位置，不使用 CV 预测位置。
- `stop_seconds`：启动 1 s 后、距目标超过 0.15 m、|v|<0.025 m/s 的累计时间，包含原地转向、必要等待，不全部等于误停。
- `qp_infeasible`：QP 无解或迭代后非线性验证仍失败，不能全部解释成数学 QP 无解。
- `verified`：该步通过原型检查，不代表实车安全证书；MPPI-only 不具备此认证。
- `total_ms`：合成观测 + MPPI + CBF 求解/验证；不含 ROS 感知、地图更新、串口和图形绘制。
- 超过 50 ms 的结果丢弃并请求制动。此为简化期限违约故障模型，非完整真实时间异步队列仿真。
- 所有种子都计入结果，不剔除碰撞或超时试验；不得仅以平均时间声称实时保证。

## 历史 bag 的边界

历史录包中 `/odom` 的 frame 是 odom，动态点 frame 是 world，且所查录包没有完整关节状态。
因此当前只生成原记录的动态点、速度、TTC、限速和轨迹消息时间线，不伪造 TF 对齐或固定真实关节姿态。
未完成真实数据的新控制器影子推演。影子推演本身也无法证明闭环成功。

## 尚未完成的验证

1. 接入真实 ESDF 和点云，测量实际查询负载。
2. 同一闭环环境中的当前 REMANI / MPPI / MPPI+CBF 公平 A/B/C。
3. CBF 与 MPPI 内部安全模型一致性、可靠的执行模型辨识。
4. 受控低速实车试验和全身协同。前述条件未通过前，不接管 `/cmd_vel`。

## 回退与可追溯性

当前工作空间不是可用的 Git 仓库，因此没有假装创建回退提交。
本轮只新增 `experiments/mppi_cbf_validation/`；把此目录移走即可撤回实验，对原启动方式无影响。
每批 `manifest.json` 保留源码/读取配置的 SHA-256、编译命令、运行命令和环境；旧结果不覆盖。

## 方法背景

- GS-MPPI: https://arxiv.org/abs/2410.02154 （说明 MPPI 与 CBF 结合已有研究；本原型不是其复现）
- Sampled-data CBF with delays: https://arxiv.org/abs/2005.06418 （说明连续时间保证不能直接外推到有延迟的实车）
