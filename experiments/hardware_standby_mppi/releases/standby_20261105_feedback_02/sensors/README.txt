本目录只包含 MID360、点云转换、FASTLIO 与底盘位姿换算。
不加载原 real.launch；不包含底盘驱动、机械臂驱动、控制桥、REMANI、ICP、自动使能或运动发布器。

启动由本版本 session 统一记录并停止。底层前台脚本为：
  bin/start_raw_sensors.sh --master-uri http://HOST:PORT
  bin/start_sensor_odometry.sh --master-uri http://HOST:PORT
远程 master 额外需要 --advertise-host 本机地址。两个脚本都要求 master 已存在，不创建新的 master。
第二个脚本仅启动 odometry，不重复启动雷达。

第一阶段输出（真实测量，全部私有）：
  /standby_trial_20261105_feedback_02/sensors/lidar_custom
  /standby_trial_20261105_feedback_02/sensors/imu
  /standby_trial_20261105_feedback_02/sensors/lidar_pcl2
雷达 frame 为 standby_feedback_02_livox_frame。点云转换保留原 tag 置信标志过滤，未使用旧 CropBox，
不把机械臂附近的一整块空间删除；点云是否充分覆盖近臂区域仍需现场观察。

第二阶段输出：
  /standby_trial_20261105_feedback_02/sensors/odometry_raw
  /standby_trial_20261105_feedback_02/sensors/odom
  /standby_trial_20261105_feedback_02/sensors/cloud_registered
TF frame 均带 standby_feedback_02_ 前缀：camera_init、body、odom、base_link、livox_frame。
IMU/LiDAR内部外参沿用原 mid360 参数；base_link 到雷达沿用 [0.28,0,0.002] 米。
机械臂碰撞模型仍按地面 z=0，不能重复加 odom 中 base_link 的 0.115 米高度。

FASTLIO 独立源码在 source/，单进程 -O1 -g0 构建，ROOT_DIR 仅指向本版本 run/fastlio_runtime/。
旧版本 Log 文件校验基线保存在 old_fastlio_log_baseline.json；执行 audit_sensor_bundle.py 可以只读核验。
构建命令、源码哈希、编译日志和动态库依赖保存在 build_manifest.json 和 build/。

这些节点只提供新局部坐标系中的感知和测量；不宣称与旧静态 PCD 地图已经配准。
没有加载旧地图，没有用 identity TF 伪装已配准，没有用未验收感知放行控制。
没有图像、点云或关节反馈时应显示 waiting，不能把空输入当作成功避障。
