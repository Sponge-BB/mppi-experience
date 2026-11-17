实车只读反馈与待机 MPPI 旁路版本 standby_20261105_feedback_02

本版本接收真实机械臂关节反馈、激光雷达和定位输入，显示实测状态及
尚未执行的 MPPI 建议。它不向底盘或机械臂执行器发布命令，不使能机器人，
不导入 Piper SDK，不启动原版 exp0_real.launch 或其他整机控制入口。
底盘/机械臂运动控制和完整实车避障仍未放行。

2026-11-05 实机核查结果：
- can1 的六关节反馈约200Hz，MID360原始云约10Hz/每帧1.9万点，IMU约200Hz。
- 启用硬件ACK后关节反馈正常；应用CAN发送数据帧为0。未使能或控制机械臂。
- 独立FASTLIO曾输出约10Hz定位；20:22主机内存耗尽后输入处理中断55.8秒，
  恢复后位置发生不可能的跳变。此定位进程已隔离停止，不能用它测量底盘位移。
- 新增发布桥与shadow两级定位连续性检查。超时、过期、重放或位置跳变锁存，
  不通过裁剪速度掩盖位置异常，也不自动把异常位置当作新的待机锚点。
- 当前应使用 --raw-view 查看真实雷达帧；该视图不显示失效定位下的机器人模型。
- 当前机械臂球模型对折叠姿态报自碰；原项目Piper自碰检查被关闭，不能据此
  取消本版本检查。本帧多数臂球也超出原始点云上视场，空回波不代表可通行。
- 原Piper驱动的隔离复用在相邻control_03准备，不能将本观察版直接切为执行器。
故障、几何与输入证据见上级目录feedback02_odometry_failure_audit.json、
geometry_alignment_audit.json、actual_feedback_integration.json。

与 standby_20261105_01 的区别：
- 单独复制 core/model/config/runtime/bin/lib/tests；不继承日志、PID或run。
- ROS 命名空间 /standby_trial_20261105_feedback_02。
- 机械臂通过 CAN_RAW 接收反馈，要求已配置 UP + LISTEN-ONLY；代码不调用
  CAN send，不改变接口配置。can0曾被内核重命名为can1，请按现场检查明确
  传 --interface can1，不能只凭旧名称判断设备。
  若现场明确允许控制器发送协议ACK，可显式加 --allow-hardware-ack，允许
  已经配置的normal CAN模式；应用仍不发送任何CAN数据帧或运动/配置命令。
  硬件ACK位与应用发送数据帧必须分别记录，不能用应用TX=0否认硬件ACK。
- 原始雷达、IMU、点云与里程计使用 sensors/ 下单独配置的只感知链；
  不借用旧环境的盲区裁剪、固定地图配准或假定机器人位姿。

原版 src、devel、.bashrc、launch、系统 Python 和冻结_01均不修改。
仅 source /opt/ros/noetic/setup.bash；使用 /usr/bin/python3；禁用用户site，
清除继承的旧 overlay 路径。所有日志、ROS_HOME和PID都在本release内。
运行前先停止_01自己的监视器；_02不会抢占或停止已占用11461的master。

操作（在本release目录运行，由现场操作者按设备状态选择）：
1. bin/preflight.sh
   仅检查依赖、哈希、CAN/串口存在情况；受限沙箱可能看不到宿主设备，
   真实硬件清单以实际宿主权限检查为准。不会打开UART或发送CAN。
2. bin/start_monitor.sh
   启动本release的 localhost:11461 roscore 和shadow，只绑定回环地址。
   无真实输入显示等待；不伪造底盘、机械臂、障碍物或避障成功结果。
3. bin/start_feedback.sh --interface can1 --duration-s 120
   接口必须已经UP并确认LISTEN-ONLY，否则拒绝启动且不改接口。
   显式 --allow-hardware-ack 可接收已配置normal CAN模式；默认不放开。
   仅在本release拥有的master上接收指定时长；输出为
   /standby_trial_20261105_feedback_02/sensors/joint_states。
   关节按joint1..6、弧度、接收时间戳排列；速度由真实位置差分估计。
   非新鲜/不完整反馈不能成为可执行控制依据。
4. bin/start_sensors.sh
   纳管启动原始Livox点云与IMU，只连接本release已有master。
   读取真实传感器，不启动任何底盘/机械臂执行器或机器人使能。
5. bin/start_odometry.sh
   只有独立FASTLIO构建完成、传感器数据新鲜且实际外参核对后才启动。
   独立ROS话题和TF前缀；该状态估计不构成已经完成机械臂几何标定。
6. bin/view_shadow.sh
   打开本release的RViz。固定帧 standby_feedback_02_odom；显示私有球模型
   MarkerArray，可勾选真实原始点云；无导航目标/初始位姿发布工具。
   未有真实odom/TF时显示等待，不补造静态TF或机器人位姿。
   FASTLIO尚未启动时可用 bin/view_shadow.sh --raw-view，以实际雷达帧
   standby_feedback_02_livox_frame直接看实时原始点云，不伪造odom/TF。
7. bin/status.sh / bin/stop.sh
   检查/停止本release纳管的master、shadow、feedback、raw sensors、
   odometry、可选tracker和RViz进程组。PID、启动时刻和release身份均匹配
   才发送SIGINT；不killall，不停止别人的驱动或其他版本。
8. bin/restart_shadow.sh
   更新observer后，只替换本release身份验证通过的shadow进程；独立master、
   真实反馈、传感器、定位和RViz保持运行。旧shadow日志保留，新日志单独
   记录；PID/启动时刻/进程组/环境身份无法验证时拒绝发信号或替换。

所有纳管组件也可用 bin/session.py 的 start-feedback、start-sensors、
start-odometry、start-tracker 操作。Shell入口会先设置本release环境。
底层 start_raw_sensors.sh / start_sensor_odometry.sh / start_private_tracker.sh
只供上述session调用，保持前台exec；应优先使用纳管入口统一退出。
start_tracker.sh 是可选私有感知，静态地图未配准前不要启动来伪造健康。

当前输入配置见 config/hardware_shadow.json：
odom、joint_states、lidar_pcl2均来自本release的sensors命名空间；world/base
分别为 standby_feedback_02_odom 与 standby_feedback_02_base_link。
尚无已验收的filtered/static/dynamic安全输入时，shadow严格保留等待或拒绝。
真实关节与里程计齐备后才显示实测姿态球模型；紫色轨迹即使出现也仅是
未执行的候选，不等于机器人已经完成避障。

几何仍为native Piper DH球模型，真实连杆/工具/安装外参尚未验收。
冻结URDF仅保留运动学和关节限位，删除未使用的visual/collision网格和
Gazebo元素，不依赖原环境mesh。joint6在URDF、model和kernel中均收窄
到[-0.35,0.35]弧度；参数来源、源文件哈希和转换记录在model/provenance.json。
待机锚点和机械臂参考从新鲜、静止的实测状态捕获；不直接套仿真初值。

版本集成与对照：
config/packaging_origin.json记录冻结_01来源及其文件哈希。此版本集成阶段
不沿用_01的release_manifest，也不在未完成协作时假装已经冻结。
集成和检查完成后显式运行 /usr/bin/python3 bin/release_manifest.py --write；
之后 /usr/bin/python3 bin/release_manifest.py --verify 验证本release内容。
清单排除日志、run和pycache，包含sensors及私有lib。A组原入口保持原样；
切回A前先bin/stop.sh，再按现场流程恢复CAN接口与原版启动，避免两套
感知/控制链同时争夺设备。停止本release本身不会更改CAN状态。
