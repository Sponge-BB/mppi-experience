实车待机 MPPI 独立版本 standby_20261105_01

这是 shadow（旁路观察）部署：读取真实状态和传感器，显示建议避障动作，
不会向 /cmd_vel、/joint_ctrl_single 或其他执行器发送指令，不启动底盘、
机械臂驱动，不自动使能。它不是已经放行的实车运动控制版本。

原环境保留：不修改 src、devel、.bashrc、原 launch 或系统 Python。
仅 source /opt/ros/noetic/setup.bash；使用 /usr/bin/python3，禁用用户 site。
core/model/config 冻结于本目录；运行日志、ROS_HOME 和 PID 文件只写本目录。
命名空间 /standby_trial_20261105_01。部署不需要 catkin_make 或 pip install。

操作（从本目录运行）：
1. bin/preflight.sh
   只读检查依赖、版本哈希、CAN/串口存在情况；不会打开设备。
2. bin/start_monitor.sh
   无现有 ROS 时，在 localhost:11461 启独立 roscore + shadow。
   没有真实输入时应显示等待状态，不伪造机器人、障碍或运动结果。
3. bin/start_shadow.sh --master-uri http://127.0.0.1:11311
   有真实传感器 ROS 图时，连接明确指定的现有 master；不启动任何驱动。
   远程 master 还需 --advertise-host <本机可达地址>。
4. bin/preflight.sh --master-uri http://127.0.0.1:11311
   额外保存真实 ROS 图、话题类型、/use_sim_time。全程只读。
5. bin/status.sh / bin/stop.sh
   仅检查/停止本 release 记录且 PID 身份匹配的进程组；不 killall，
   不停止别人的 master、驱动或原版控制器。
6. bin/view_shadow.sh
   打开本release的RViz。fixed frame为odom，只显示独立MarkerArray；
   原始/滤波实际点云可在Displays勾选，不提供导航目标或姿态发布工具。
   无传感器时只有等待提示，不生成虚构的实车运动。

A/B 对照：A 继续使用原来的启动入口和参数，不需要本目录。
B 目前是旁路影子组，适合对照建议轨迹/风险检测/计算时延，不能据此
声称完成真实避障。关闭 B 仅需 bin/stop.sh；原始文件与原环境不受改写。
执行模式还需独立网关、唯一控制权、真实时序/几何标定、现场确认后放行。

传感器配置见 config/hardware_shadow.json。待机锚点与机械臂保持姿态由
新鲜、静止的实测状态捕获。仿真姿态仅存在 config/simulation_source.json
用于溯源，默认不作为实车目标。机械臂关节速度从真实位置差分估计。
任何输出均在独立命名空间；输出可视化不等于已向真实机器人执行。
动态点云默认订阅本release的私有感知输出。已有真实传感器和定位数据后，
才可显式使用 bin/start_private_tracker.sh；该入口不自动启动任何硬件驱动。
私有slow-target参数用于观察0.08 m/s慢目标，仍未实车验收，不能据此实控。
显式入口：bin/start_private_tracker.sh --master-uri http://127.0.0.1:11311
该入口只连接已存在的master，前台运行；Ctrl-C仅停止这个私有感知进程。
远程master需另加 --advertise-host <本机可达地址>。它不启动任何底盘/机械臂
驱动、不使能机器人、不发布执行器命令；没有真实传感器时仍无实车数据。

几何仍为 native Piper DH 球模型，URDF 仅作冻结运动学与关节限位依据。
与真实连杆/工具/安装高度对齐尚未验收。model/mm.urdf 删除未使用的
visual/collision 网格与 Gazebo 元素，不依赖原环境mesh；转换来源有哈希。
joint6 上下界在冻结URDF与配置中收窄到[-0.35,0.35]弧度，与旧桥一致。
默认不加载 URDF 到全局 robot_description，不覆盖 TF 或传感器参数。

版本校验：config/release_manifest.json 记录冻结文件 SHA256。
deploy_local.py --refresh-manifest 仅在 runtime 编写完成后更新本 release
清单，不修改原代码。preflight 输出存 logs/preflight_*.json。
