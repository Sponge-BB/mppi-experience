# V6 实验分类

- [正式对照 validation_01](validation_01/README.md)：6 个场景，V5/V6 各 3 个随机种子；不是完整 REMANI / Gazebo / 实车。
- [源代码与假设](../../MPPI_V6.md)：必须先了解“理想动态点归属分层”假设。
- `unit_tests/`：协议、碰撞分层、直线/曲线/倒车跟踪回归。
- `ablation_union_01/`：最终 V6 参数但不提供分层输入的对照，禁止只展示分层后的有利结果。
- `development_02/`：最初连续候选，停留增加，且报告序列化中断；原始记录保留并重绘。
- `development_03/`：早期风险代价，仍未通过性能验收。
- `development_04/`：横穿终端软偏好，仍停留。
- `development_05/`：扩展转向候选，但联合距离场仍导致退化。
- `development_06/`：首次理想点归属分层，不停但绕行过长。
- `development_07/`：提前触发后的单种子开发检查；正式结论使用 validation_01 全部种子。
- 旧目录 `../mppi_v6_development_01/`：最初 V5/V6 四次检查，未移动、未覆盖。

每个正式场景单独存放 `data/`、`gif/`、`plots/`、`summary.json`。
每个 `gif/` 有旧版、新版、并排三张动画。GIF 使用种子 1，其余种子全部保存在 data 中。
失败的中间构建和结果不作为推荐版本；当前实验构建是 `builds/mppi_v6_07`，V5 可随时复现。
