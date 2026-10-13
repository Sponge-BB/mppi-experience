# V7 实验文件分类

- [多动态障碍物正式实验与搜索 GIF](multi_dynamic_03/README.md)：1～4个动态目标，各3次重复，障碍物彼此不相交。
- [多障碍实现、可视化图例与边界](../../MPPI_MULTI_DYNAMIC.md)
- [首轮多障碍压力测试](multi_dynamic_02/README.md)：含障碍物之间互相重叠，原始成功与失败结果均保留，不与正式轮混算。

- [正式对照及全部 GIF](validation_01/README.md)：7 场景，每版本3个种子；V6/V7同理想分层输入，障碍物持续存在。
- [结果汇总](validation_01/结果汇总.txt)：包含较慢、停留和净距缩小的取舍，不只展示改善指标。
- [完整改动说明](../../MPPI_V7.md)
- [开发失败记录](DEVELOPMENT_LOG.md)：01、03、04构建未通过相应开发检查，不能当作最终版本。
- `development_01/`、`development_02/`、`development_05/`：开发对照原始数据及各自GIF，未覆盖。
- `unit_tests/`：V7动作决策、持续目标回接、合法倒车、分类地图接口和跟踪回归。

每个场景独立保存 `data/`、`gif/`、`plots/`；GIF分别为 `v6.gif`、`v7.gif`、`comparison.gif`。
曲线除车体线速度之外还包含 `progress.png`，用于区分“在移动”和“向目标有效前进”。
当前实验构建：`builds/mppi_v7_05`；回退构建：`builds/mppi_v6_07`。实车未改。
