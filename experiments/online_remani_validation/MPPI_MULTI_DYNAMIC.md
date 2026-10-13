# V7 多动态障碍物与候选轨迹可视化

本轮只扩展实验与诊断，不改变 V7 决策代价、候选生成策略、0.1m固定余量、底盘模型或实车代码。
当前是固定参考的局部数值闭环，不是完整 REMANI 在线规划、Gazebo 或实车验证。

## 入口

- [正式实验：障碍物互不相交](results/mppi_v7/multi_dynamic_03/README.md)
- [首轮压力测试：含障碍物相互重叠，保留失败结果](results/mppi_v7/multi_dynamic_02/README.md)
- [候选导出接口单测](results/mppi_v7/search_export_tests_02.json)
- [原 V7 决策回归](results/mppi_v7/search_unit_tests_01.json)
- [正式轮失败分析与下一步建议](results/mppi_v7/multi_dynamic_03/失败分析.txt)

最终结果：单、双、三障碍组各3次到达；四障碍组3次均碰撞，未通过避障验收。
四障碍首次失去可行候选后，零速兜底不能阻止迎面物体继续靠近。
离线检查在同状态找到其他短时可行的退让速度组合，支持候选覆盖不足的判断；没有把这些离线动作当作修复后的闭环成绩。

正式测试分别使用1、2、3、4个动态障碍，每组3个种子。动画固定展示seed 1。
第二、三目标上下交叉，第四目标迎面接近。障碍物从2秒开始持续运动，出画面后不删除。
各目标独立CV运动、速度估计准确、静动态点归属理想，没有真实雷达遮挡或定位误差。

首轮`multi_dynamic_02`发现部分障碍物之间会重叠；正式轮调整第二目标初始y和第三目标x，
用解析相对运动计算整个45秒内每一对障碍物的最小净距，确保障碍物本身互不相交。
这是场景有效性修正，不是为改善成绩而调整MPPI；两轮数据分别保存。
`multi_dynamic_01`是首次JSON序列化失败目录，没有可用正式结果，保留不混入统计。

## 可视化内容

每组目录独立保存：

```text
data/                  所有种子、逐周期指令/状态/障碍物、实际候选轨迹
gif/search.gif         全景：候选、所选预测、已经执行的轨迹
gif/search_zoom.gif    跟随机器人放大：看清短时域候选束
gif/motion.gif         不画候选的简洁运动图
plots/search_snapshot.png
plots/search_zoom.png
plots/metrics.png      全部重复的轨迹、速度、净距、内核耗时
```

- 蓝色是实际轨迹和车体组合圆。
- 绿色是实际选中的预测中心轨迹；浅蓝/浅红分别为通过/未通过碰撞检查的候选。
- 灰色圆及箭头是障碍物当前位置和1.5秒CV方向。
- 候选来自V7外层真实评估，不是伪造的扇形线；不包含内层V5全部采样或栅格搜索节点。
- 候选按未来时间与障碍物预测对应检查，不能仅看它是否穿过障碍物的当前圆形。
- 无动态目标时不记录V7动态候选；监测/回接分支只有少量候选，不能解读为丢帧。
- 没有新增RViz话题。本轮交付为离线GIF/PNG和原始JSON，之后可复用数据接口做ROS Marker发布。

## 实现与检查

- `mppi_v7.hpp`：增加默认关闭、可按周期开启的只读候选记录；不额外调用随机数或改变评分。
- `build_mppi_v2.py`：新增`online_search_enable`、`online_search_trace`实验C接口。
- `suite_multi_dynamic.py`：多目标持久观测、5ms子步净距审计、数据记录及简洁渲染。
- `test_mppi_search.py`：导出容量、只读性、轨迹有效期、重置/非法输入清理、执行模型一致性检查。
- `report_multi_dynamic.py`：结果汇总、数据与源文件审计、全部GIF可读性检查。

记录只复制已评估的控制序列、代价和可行性；轨迹几何在控制内核计时外重建，导出耗时单列。
每组seed 1与原`mppi_v7_05`完整闭环复跑，逐项比较指令和状态，防止可视化影响随机采样与运动结果。
5ms净距检查覆盖本步起点、中间子步及终点；报告碰撞和低于固定安全余量两种情况。
算法无可行候选时的零速输出仍标记“未验证制动”，不会因为输出零速就计为安全。

实验按固定仿真时间前进，不将内核运算时间自动施加为物理执行延迟；所以耗时统计不能当作实时性认证。
所有试验均未连接ROS master、底盘、机械臂或真实控制话题。

## 复现与回退

```bash
cd /home/l/remani_ws
python3 experiments/online_remani_validation/suite_multi_dynamic.py \
  --build experiments/online_remani_validation/builds/mppi_v7_search_01 \
  --original experiments/online_remani_validation/builds/mppi_v7_05 \
  --fixture experiments/online_remani_validation/results/final_report/empty_mppi \
  --out experiments/online_remani_validation/results/mppi_v7/my_multi_run
python3 experiments/online_remani_validation/report_multi_dynamic.py \
  experiments/online_remani_validation/results/mppi_v7/my_multi_run
```

使用新的输出目录。原V7/V6库和结果未覆盖；选择原构建即可回退实验控制器，不涉及生产源码回滚。
