standby_20261106_mesh_06 — 原网格支持的局部自碰撞复检

本步结果
  之前采集的真实收拢关节姿态可以进入MPPI规划。05中两处粗球负间隙
  (0,18)=-7.398mm、(7,18)=-7.387mm，均有原网格保守复检依据。
  110项离线测试通过；测试峰值RSS约85.2MiB。没有启动运动或发送硬件命令。
  原项目13个关键文件、冻结01—05清单均无变化。本版本使用独立06命名空间。

最小改动
  保留89对自碰撞检查、全部67个外部避障球、原FK、半径、限位、底盘和地面策略。
  仅两对粗球间隙为负时复检：肩部/机械臂底座对腕部/末端使用完整网格凸包；
  上臂叉形空间按原link2本地z=0拆成两片凸包，跨截面的完整三角同时放入两片。
  原上臂73166个三角全部覆盖，283个跨面三角保留于两边，不丢掉几何。
  各片随候选关节姿态重新计算支持间隙；只有每片均正才解除对应粗球拒绝。
  所有间隙统一扣0.1mm数值余量，该余量不代表实物标定误差。
  起姿、采样轨迹、加权合成、制动和最终10ms间隔验收共用self_gaps。
  不同底盘候选中完全相同的关节状态仅计算一次证书，无额外缓存框架。
  运行时只用NumPy，不加载原三角网格、不依赖FCL或新增驱动。

验证结果
  当前姿态肩腕原三角表面间隙1.586mm；保守凸包证书扣数值余量后0.523mm。
  上臂两片证书最小10.458mm。q5增加0.03rad会使原肩腕网格相交，06仍拒绝。
  相邻21个离线姿态中8个网格相交样本全部拒绝。原有其他球判定保持保守，
  因此也有原肩腕网格不相交但整机模型仍拒绝的点；没有声称所有近似误差已消除。
  当前关节姿态+明确合成点云的3次规划耗时83.1/86.2/88.0ms（均小于95ms预算），
  最终复检约7.9—8.1ms（小于20ms预算）。这是单次离线计时，未证明持续20Hz。
  该合成场景对收拢姿态无威胁，规划正确保持静止；不冒充实车已做避障动作。
  另保留伸臂+移动点障碍的合成MPPI回归：有机械臂避让，预测底盘位移0。
  这些结果都不是实车运动对比数据。

下一步
  接入真实输入做有界影子运行，检查规划连续性及超时。仍需处理近处高障碍
  可见性缺口，再验证低速实物响应，最后做原生REMANI与MPPI的整机运动对照。
  当前hardware_readiness各项仍false，execute=false、自动使能false。
  原安装高度沿用用户认可的参数，不要求本步补做精确测量。

文件
  reports/mesh_refinement.png：当前关节附近的粗模型/细复检/原网格对照图。
  reports/mesh_refinement.json：21个姿态、3次规划、最终复检和合成避障结果。
  reports/tests.log、reports/independent_geometry_review.json、reports/preservation.json。
  model/shoulder_wrist_proxy.json：原网格来源、哈希、关节链和凸包极点。
  ../../../mesh_audit_20261106：原FCL网格结果及离线资产生成脚本。

复现（单进程、数学库单线程、1GiB地址空间上限，无硬件访问）
  /usr/bin/python3 tools/memory_bounded_run.py --label tests -- -m unittest discover -s tests -p 'test_*.py'
  /usr/bin/python3 tools/memory_bounded_run.py --label geometry -- tools/validate_mesh_refinement.py

观察入口
  source bin/environment.sh
  roslaunch config/integration.launch
  默认execute=false，沿用02私有反馈输入。此入口仍会等待必要的真实感知输入。
