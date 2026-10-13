# 车体投影 + MPPI 搜索可视化

这是 V11 构建 08 的**记录与显示补充**，不是 V12 算法改进。
范围仍是 REMANI 原生在线规划/重规划 + MPPI 的隔离 ROS 数值仿真，固定机械臂。
没有启动实车，也不是 Gazebo；没有修改碰撞模型、速度、余量或控制内核二进制。

## 新录制的完整闭环

| 场景 | 全局 GIF | 搜索放大 GIF | 静态图片 |
| --- | --- | --- | --- |
| 静态障碍物 | [全局](static/visualization/overview.gif) | [局部搜索](static/visualization/search_zoom.gif) | [搜索快照](static/visualization/search_snapshot.png) |
| 动态穿越 | [全局](crossing/visualization/overview.gif) | [局部搜索](crossing/visualization/search_zoom.gif) | [搜索快照](crossing/visualization/search_snapshot.png) |

图例：橙色是真正被消费的 REMANI 参考，蓝色是已执行轨迹。
灰蓝色三圆盘是与 `geometry.csv` 一致的底盘＋固定臂保守投影，随实际位姿旋转和平移；
内部方框是 0.50 × 0.50 m 底盘，深色小点是机械臂碰撞采样点的 XY 投影。
圆盘不包含额外 0.10 m 安全余量，也不代表精确的实物表面模型。

搜索浅蓝色是内核可行候选，浅红色是被拒候选，紫色是内核选择；
**绿色才是经过发布前检查、获准执行的局部展开**。
绿色沿线稀疏绘制未来车体投影。轨迹线表示中心的运动，不表示碰撞检查按质点进行。
末帧、缺失记录、过期记录不补画预测；获准展开只在记录的命令租约内显示。
被拒的搜索曲线可能穿过障碍物，这是拒绝结果的展示，不是允许碰撞。

## 记录与验证

- 保持已有内核 trace 开启状态不变，读取已经用于候选选择的轨迹，约 5 Hz 保存到各场景 `search/`。
- 这是内核导出的候选，包括引导、运动原语、扰动及选中展开；不声称覆盖内部每一个采样步骤。
- 在控制指令发布后只读导出；有界后台队列写二进制记录，队列拥塞只丢遥测，不修改控制结果。
- 静态 168 帧、动态 138 帧；两次记录均无队列丢帧和写入错误。
- 导出有开销：本次单次导出最大约 7.89 / 10.10 ms，不包含在原 `kernel_ms` 中。
  因此这些带可视化记录的试验不是无干扰实时性能基准；测速可用 `--no-record-search`。
- 两次均到达目标；运动到结束约 25.89 / 20.32 s，记录的最小投影净距约 0.156 / 0.132 m。
- 两次参考来源审核、实际执行审核均通过，未记录到保守投影碰撞或必需进程故障。
- 仍有拒绝动作和超时周期，不能把“到达”解读为全程不停顿，也未修复此前讨论的慢规划失败及回接偏离。
- `test_native_visuals.py` 的 5 项测试通过：投影几何一致及旋转、缺少几何禁止退化为点、记录新鲜度和终止清理、
  真实内核 getter 不改变后续控制输出、未调用内核时不复用旧搜索。
- 每套图的 `visualization_audit.json` 保存输入哈希；重绘验证原始记录不变。

## 历史记录补画投影

[原静态 V10/V11 对照](historical_comparison/static/comparison.gif)：与此前讨论的两条静态记录一致，只改显示。

[另一组动态历史对照](historical_comparison/crossing/comparison.gif)：V10 `crossing_delayed_06` 对
V11 `validation_05/crossing_delayed`，不是此前首页使用的 `crossing_runtime_06` 配对。
历史记录没有完整搜索快照，不能事后伪造搜索线；以上图仅补车体投影。
原来的 GIF 和原始数据均保留。

## 后续使用

`run_native_v11.py` 现在默认记录搜索，运行结束自动在新实验目录的 `visualization/` 生成全局和放大 GIF。
单独重绘（必须指定新的输出目录）：

```bash
python3 experiments/online_remani_validation/render_native_search.py \
  --source <实验目录> --out <新的图片目录>
```

无需回滚实车工程。停用新增记录可传 `--no-record-search`；回退显示则直接查看仍保留的旧 GIF。
