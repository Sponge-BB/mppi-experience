# MPPI预测 / 参考路径 / 实际轨迹

橙色虚线：固定参考路径；绿色：本帧记录的选中MPPI预测；蓝色：已执行轨迹。灰色：障碍物及CV预测。

**上一轮没有运行REMANI在线规划器，所以不存在可回放的REMANI在线路径。橙线是脚本生成的固定参考，不能作为REMANI重规划结果。**

只重绘原实验seed 1；不重跑算法，不更改参数、成绩、原始JSON或旧GIF。
预测记录为5Hz，显示时严格匹配记录时间；缺失、无选中候选及终止帧不补画，不用实际未来运动冒充MPPI预测。

## 时域对照

- 单迎面，原速：[三时域对照](comparison/05_head_on_only/base_horizons.gif)
- 单迎面，提速：[三时域对照](comparison/05_head_on_only/fast_horizons.gif)
- 四障碍，原速：[三时域对照](comparison/04_four_dense/base_horizons.gif)
- 四障碍，提速：[三时域对照](comparison/04_four_dense/fast_horizons.gif)

## 各配置单独查看

- base_h2 / 单迎面：[轨迹GIF](base_h2/05_head_on_only/paths.gif) · [t=10秒快照或终止帧](base_h2/05_head_on_only/paths_snapshot.png)
- base_h2 / 四障碍：[轨迹GIF](base_h2/04_four_dense/paths.gif) · [t=10秒快照或终止帧](base_h2/04_four_dense/paths_snapshot.png)
- base_h3 / 单迎面：[轨迹GIF](base_h3/05_head_on_only/paths.gif) · [t=10秒快照或终止帧](base_h3/05_head_on_only/paths_snapshot.png) · [局部放大GIF](base_h3/05_head_on_only/paths_zoom.gif)
- base_h3 / 四障碍：[轨迹GIF](base_h3/04_four_dense/paths.gif) · [t=10秒快照或终止帧](base_h3/04_four_dense/paths_snapshot.png) · [局部放大GIF](base_h3/04_four_dense/paths_zoom.gif)
- base_h4 / 单迎面：[轨迹GIF](base_h4/05_head_on_only/paths.gif) · [t=10秒快照或终止帧](base_h4/05_head_on_only/paths_snapshot.png)
- base_h4 / 四障碍：[轨迹GIF](base_h4/04_four_dense/paths.gif) · [t=10秒快照或终止帧](base_h4/04_four_dense/paths_snapshot.png)
- fast_h2 / 单迎面：[轨迹GIF](fast_h2/05_head_on_only/paths.gif) · [t=10秒快照或终止帧](fast_h2/05_head_on_only/paths_snapshot.png)
- fast_h2 / 四障碍：[轨迹GIF](fast_h2/04_four_dense/paths.gif) · [t=10秒快照或终止帧](fast_h2/04_four_dense/paths_snapshot.png)
- fast_h3 / 单迎面：[轨迹GIF](fast_h3/05_head_on_only/paths.gif) · [t=10秒快照或终止帧](fast_h3/05_head_on_only/paths_snapshot.png)
- fast_h3 / 四障碍：[轨迹GIF](fast_h3/04_four_dense/paths.gif) · [t=10秒快照或终止帧](fast_h3/04_four_dense/paths_snapshot.png)
- fast_h4 / 单迎面：[轨迹GIF](fast_h4/05_head_on_only/paths.gif) · [t=10秒快照或终止帧](fast_h4/05_head_on_only/paths_snapshot.png)
- fast_h4 / 四障碍：[轨迹GIF](fast_h4/04_four_dense/paths.gif) · [t=10秒快照或终止帧](fast_h4/04_four_dense/paths_snapshot.png)
- capacity_h2 / 单迎面：[轨迹GIF](capacity_h2/05_head_on_only/paths.gif) · [t=10秒快照或终止帧](capacity_h2/05_head_on_only/paths_snapshot.png)

本次无法补出不存在的REMANI路径。要同时查看真正的REMANI在线重规划与MPPI，需要另跑完整在线实验并记录规划轨迹。

