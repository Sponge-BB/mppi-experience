standby_20261105_control_03 — 原机械臂驱动隔离复用，只读连通已验证

当前状态：已在实车短时仅RX连接验证，测试进程已退出；未使能、未发送
CAN数据帧或运动指令，运动功能尚未验证。默认连接和运动参数仍均为false。
8秒验证收到原驱动297条关节反馈，六个电机使能均为0，SDK发送调用为0，
CAN TX包计数增量为0。完整记录位于上级目录reused_driver_readonly_actual_audit.json。
当前实车模型自碰撞/感知定位问题由主任务处理，本版本不能绕过它们直接运动。

复用内容：
  driver_reference/piper_ctrl_single_node.py 和原 launch 为原文件逐字节副本。
  python/piper_sdk 为已安装 SDK 0.6.1 的原样副本；协议编码完全复用。
  新节点继承原反馈发布方法，保留 SDK JointCtrl、EnableArm、MotionCtrl_1 stop。
  原构造函数、初始化 MotionCtrl_2、PiperInit 请求、自动 enable/re-enable 不运行。
  pos/go-zero/gripper/reset/unblock 旁路均未注册，并有覆盖方法阻断直接调用。
  不修改原 src、devel、系统 Python、.bashrc，也不修改冻结版本 01/02。

默认状态与私有接口：
  config/reused_piper.launch 的 connect_transport=false、allow_motion=false。
  默认连 CAN socket 都不打开；独立 allow_motion=false 可用于以后只收反馈核对。
  所有服务/话题明确 remap 到 /standby_trial_20261105_control_03/arm/。
  node wrapper保留该arm命名空间，并校验节点全名，确保私有launch参数有效。
  唯一跨版本输入是 02 的 /standby_trial_20261105_feedback_02/sensors/joint_states。
  该输入必须来自已核查的只读 CAN 三对反馈完整拼接器，保留最旧帧时间戳。
  原 SDK joint_states_single 被放到 sdk_feedback_unvalidated 私有话题：原方法
  角度换算 0.017444 rad/degree 存在约 -0.053% 比例误差；原速度单位仍未验证。
  因 SDK aggregate stamp 只反映最近一对反馈，不能把此话题回接门控或 MPPI。

门控：
  只有 connect_transport=true、allow_motion=true、实际新鲜反馈和显式 enable 才能开门。
  JointState 命令必须严格 joint1..joint6 共 6 个位置，rad，无夹爪控制；header.seq
  严格递增，header.stamp 为真实墙钟 ROS 时间；租约固定 150 ms，按源 stamp 到期。
  命令不续过期租约，enable 重复消息也不续租；超时/反馈失效/非法值/重放/时钟跳变
  先锁存关闭，再调用原 MotionCtrl_1(0x01,0,0) 停止。迟到命令不能重新启动。
  恢复需再次显式 enable，并使用 enable 后生成的新 stamp 和更大 seq；seq floor
  不因 re-enable 清零。六关节限位保持模型值，joint6 在 [-0.35,0.35] rad，
  目标距实测角最多 0.03 rad；原固定速度参数默认 5，允许 [1,10]。
  所有 actuator SDK 调用共用同一 RLock，独立单调时钟 watchdog 每 10 ms 检查。
  此门控不等价于硬件急停；进程挂死/SDK 阻塞仍需独立硬件保障。
  已确认原 SDK 的 bus.send(message) 没有传 timeout；其阻塞可占住同一控制锁，
  导致进程 watchdog 无法及时 stop。启用实际运动前必须补齐有界发送和独立停止
  路径，并验证多帧 JointCtrl 的总发送时长小于源命令剩余租约；目前尚未解决。
  SDK 自身部分发送错误只记录日志，因此真实 TX 成功/停止效果尚需上机检查。

离线测试（不会启动 ROS 节点、不会构造 SDK 实例、不会打开设备）：
  source /home/l/remani_ws/experiments/hardware_standby_mppi/releases/standby_20261105_control_03/bin/environment.sh
  /usr/bin/python3 -m unittest discover -s "$STANDBY_CONTROL_RELEASE/tests" -v
  /usr/bin/python3 "$STANDBY_CONTROL_RELEASE/tests/audit_sdk_receive_path.py"
  测试涵盖默认零 TX、初始化参数、源时间租约、迟到命令、重放、反馈失效、
  re-enable 新命令要求、转换/限位、传输异常、同锁命令/stop 顺序和私有 launch。

上线前仍需主任务核查：
  真实 CAN RX-only 启动前后 TX 计数、私有 ROS 图；不能与原驱动同时控制。
  真实机械臂模型与定位/点云对齐、碰撞判定、反馈时效及实际停止效果。
  MPPI 到该 JointState 接口的受限位置网关尚未接入，底盘传输也尚未接入。
  因此本版本交付的是可审查驱动复用层，不代表实车主动避障实验完成。

来源/校验：driver_reference/provenance.json、config/release_manifest.json、
config/sdk_receive_path_audit.json、config/offline_validation.json。
