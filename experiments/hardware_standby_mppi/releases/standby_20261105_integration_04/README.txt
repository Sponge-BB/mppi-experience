standby_20261105_integration_04 — MPPI 到原驱动的离线迁移

状态：核心代码接入与离线验证完成；没有进行实车运动或避障验收。
原项目、冻结版本 01/02/03、原 SDK 保持不变。默认仅记录控制，设备连接与运动关闭。

核心链路
  已有 MPPI 的首个 [v,w,dq1..dq6] → 限速/限加速度 → 67 球碰撞复检 →
  六轴短位置目标 + 底盘 v/w → 原机械臂 SDK 和原底盘 UART。
  底盘自由度保留；没有固定底盘。不是把整段预测轨迹直接发给机械臂。
  计划、实测状态、障碍数据均保留源时间，取最早截止，转发和 watchdog 不续期。
  机械臂沿用原编码与单位换算，外部适配器限制每帧 CAN 发送至多 5ms。
  底盘沿用原 UART 编码与接收方法，50Hz 发送前检查同一绝对截止，写超时 5ms。
  /mm_controller_node/car_cmd 的位姿字段不作为底盘速度输入。

已验证
  reports/tests.log：95 项纯逻辑、mock 驱动与接线检查通过，峰值约 83 MiB。
  reports/offline_validation/report.json 与 overview.svg：原研究参数的单次合成 MPPI
  到 mock SDK 链路，包含限幅后复检、短位置偏移及超时停止。采用确定性测试时钟。
  reports/wall_clock_pipeline.json：当前低速配置、真实主机时钟的单次合成链路通过，
  规划约 84ms、复检及记录约 6ms。此结果不证明持续 20Hz 或实车响应。
  reports/preservation.json：13 个原项目关键文件和父版本 367 个冻结文件保持一致。
  reports/launch_parse.json：三个节点配置和依赖可解析，未启动 ROS 节点或硬件。

参数与边界
  当前低速接口配置：v<=0.05m/s，w<=0.15rad/s，关节速度<=0.15rad/s。
  源命令有效期最多 150ms；规划预算 95ms，复检预算 20ms，过期直接拒绝。
  为降低单次计算量，迁移配置为 8×0.1s 时域、4 个随机样本；研究原版不改。
  config/hardware_shadow.json 的姿态候选沿用已做实验的收臂姿态，未经实车验证。
  20Hz 是调度目标；当前单次测量未达到该持续频率，必须做连续反馈时序验收。
  速度模型与短位置路径的离散复检不等价于真实伺服响应或连续时间安全证明。
  实车定位故障、当前收臂姿态碰撞近似、近臂障碍可见性尚未解决，因此
  config/hardware_readiness.json 保持未通过；不能手工改为 true 代替验证。

运行已完成的离线检查（每次一个进程，1GiB 地址空间上限、单线程数学库）
  cd /home/l/remani_ws/experiments/hardware_standby_mppi/releases/standby_20261105_integration_04
  /usr/bin/python3 tools/memory_bounded_run.py --label tests -- -m unittest discover -s tests -p 'test_*.py'
  /usr/bin/python3 tools/memory_bounded_run.py --label synthetic -- tools/run_offline_validation.py
  /usr/bin/python3 tools/memory_bounded_run.py --label wall_clock -- reports/wall_clock_case.py

ROS 入口（本轮只解析，未启动）
  source bin/environment.sh
  roslaunch config/integration.launch
  默认 execute=false，只消费反馈并记录；不自动使能。反馈沿用冻结 02 的私有话题。
  在有效输入下，gateway/enable 是显式控制开关；旧计划不能在使能时复用。
  arm/command_lease + arm/joint_ctrl_single 是配对接口，base/leased_command 是
  底盘完整带期限接口。正常接入使用 gateway，不直接旁路发布。

下一项实车工作：验证定位/近臂障碍输入与模型，再做原生项目对当前项目的
移动障碍干扰机械臂实验，测底盘位移和转角。此次报告不包含实车避障成功结论。
