standby_20261106_alignment_05 — 沿用原项目安装参数的独立版本

状态：离线代码修正与100项回归检查通过。没有进行实车避障运动。
原项目、冻结01–04均保持原样；本版本的控制话题单独使用05命名空间。

本次改动
  保留原REMANI的安装参数：机械臂地面系安装高度0.680m，雷达由
  base_z=0.115m和相对高度0.002m得到0.117m。URDF链得到0.688m，
  这8mm差异原项目本来就存在；沿用原约定，不添加重复高度补偿。
  用户已明确暂按原项目参数推进。config/original_geometry_assumptions.json
  保存来源和假设；这些尺寸未被宣称经过现场精确测量。

  修正一处自碰撞邻接映射：DH采样组1与3分别对应上臂link2、前臂link3，
  原SRDF明确将二者列为Adjacent。保留FK、全部67个碰撞球及半径、外障、
  地面和底盘检查；其余89对自碰撞检查仍有效。model/piper_camera.srdf
  是原文件的冻结副本，不引入新的碰撞框架。

当前限制
  修正后零关节姿态可通过球模型；当前捕获的真实收拢姿态仍因肩部/腕部
  球0/18重叠约7.4mm而拒绝。这是近似模型的判定，不能直接当成实物碰撞，
  也没有删除这对或缩小球半径来放行。仍需处理这处模型冲突才能进入动作验证。
  机械臂附近约44cm处的高目标缺少上部回波；移到约76cm后20帧均有高处回波。
  此结果只验证相应位置的可见性，未解决近距离高障碍覆盖。
  原定位链在内存限制下完成30秒静止检查、约10Hz、无跳变或中断，进程树
  RSS峰值约276MiB。静止底盘位置估计最大变化约3.6cm，尚不是动态/长期验收。
  定位测试结束后已停止本次所有定位进程。硬件readiness仍未放行。

证据
  reports/tests.log：100项通过，含5项邻接回归；测试峰值约84MiB。
  reports/preservation.json：原环境及01–04完整性核对。
  reports/alignment_check.json：当前真实关节反馈的模型判定。
  ../../../readonly_localization_20261106/report.json：实际只读定位记录。
  ../../../front_target_distance_comparison_20261106.png：泡沫柱近远对比。

入口
  source bin/environment.sh
  roslaunch config/integration.launch
  默认execute=false，驱动不连接、不自动使能；读取02的私有反馈话题。
  can_port默认can0，可在launch参数中显式选择；重启后接口名可能改变。
  速度/加速度、源数据截止期限和驱动实现沿用04。本次没有改动MPPI评分。

离线复查（单进程、数学库单线程、1GiB地址空间上限）
  /usr/bin/python3 tools/memory_bounded_run.py --label tests -- -m unittest discover -s tests -p 'test_*.py'
