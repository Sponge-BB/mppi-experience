# V5：减少引导外形保守性与不必要绕行

适用范围：固定机械臂的隔离底盘数值仿真。不是 Gazebo，不修改实车，不含 CBF-QP。
最新成功构建 `builds/mppi_v5_02`，对照为保留的 `builds/mppi_v4_03`。

## 碰撞模型改动

取消局部引导额外使用的约 0.405 m 大外接圆，位姿净距改用 MPPI 相同的原有组合圆，
各圆心随朝向旋转。几何外形和固定机械臂包络本身没有缩小或删除。
原有 0.1 m 固定安全余量、测量栅格离散补偿、轮速及加速度限制仍保留。
不确定性附加半径仍为零。

栅格节点代价暂以当前朝向估计，连接线段按行进朝向做组合圆检查；这不是完整 SE(2) 搜索。
转向和执行可行性仍交给 MPPI rollout 检查，不能将几何候选称作安全证书。
继承的动态搜索风险排序仍使用保守包络，但它不是新几何通行约束；最终候选使用组合圆评价。

## 路径与策略改动

- 查找近处受阻参考段后的可行回接位置，不再固定选参考末端前 0.45 m。
- 缓存绕行时也检查是否有更短的安全连接；连续四周期可行且至少缩短 0.12 m 才切换。
- 原参考恢复畅通且已贴回 0.04 m 内时，清空绕行缓存，退出绕行。
- 将引导搜索中远离障碍的软代价系数从 0.08 降到 0.02；没有同步减少碰撞安全余量。
- 对动态场景分别生成原参考/等待候选与绕行候选，使用同一预测碰撞、偏差、进度和动作代价比较。
  不能直接比较两条不同参考下的跟踪代价，所以新增共同评价函数。
- 两组候选各分配一半采样预算；动态预测仍受观测年龄和 CV 最大有效期约束。
- 保留等待 0.4/0.8 s 后通行候选，不强制朝障碍速度反方向运动。

这仍是“局部几何引导 + MPPI”验证原型。它不是完整时空最优规划，也不保证某策略总优。
当前动态点云仍同时进入测量距离场和 CV 检查；没有未经验证地删除目标周围的静态点。

## 执行权限未放松

上层取消、原生零限速、过期感知和超时保护保留。取消后影子计算不执行。
本轮未修改 REMANI 接受局部执行轨迹的协议，完整动态停车仍可能存在。
上一轮曾有原生规划进程 -11，本轮没有修改该原生二进制，也不能宣称已修复该问题。

## 验证与复现

`test_mppi_v5.py` 专门检查：

1. 同一距离场里，原大圆误判不通的姿态，组合圆可通过；机器人旋转后真实占用过宽仍判不通。
2. 5 m 参考中的近处障碍，回接点在近处而非末端。
3. 障碍消失后，缓存会退出，不继续保留多余绕行。

`test_local_independence.py` 始终不给新参考，隔离检验局部能力：

```bash
source /opt/ros/noetic/setup.bash
source /home/l/remani_ws/devel/setup.bash
python3 experiments/online_remani_validation/test_local_independence.py \
  --builds experiments/online_remani_validation/builds/mppi_v4_03 experiments/online_remani_validation/builds/mppi_v5_02 \
  --fixture experiments/online_remani_validation/results/final_report/empty_mppi \
  --out experiments/online_remani_validation/results/my_v5_local
```

已有记录 `results/local_independence_v5_02`；完整在线图 `results/mppi_v5_online_02`；
运动后出现静态障碍补测 `results/mppi_v5_motion_static_02`。
所有测试均是单次、理想或合成观测，不构成统计性能或实车安全结论。

回退只需将 `--build` 指向 `builds/mppi_v4_03`，使用新的输出目录；实车没有改动。
最初 `mppi_v5_01` 是构建脚本索引错误导致的失败构建，不能运行；已保留，不作为测试成功记录。
