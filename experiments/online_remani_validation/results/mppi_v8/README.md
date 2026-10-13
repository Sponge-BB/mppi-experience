# V8 实验分类与入口

当前实验构建：`../../builds/mppi_v8_09/`。
仅固定参考数值闭环；没有部署实车，没有加入CBF-QP。

- [最终结果与全部GIF](validation_03/README.md)：7场景，每场景V7/V8各3次。
- [中文改进总结](validation_03/改进总结.txt)。
- [实现与验证边界](../../MPPI_V8.md)。
- [独立审计](validation_03/independent_audit.json)：记录完整性通过；安全余量和耗时问题仍如实保留。
- [开发记录](DEVELOPMENT_LOG.md)：development_*及validation_01、validation_02是开发中间结果，含碰撞、超时，非最终成绩。

最终新版21/21到达且无几何碰撞；但有1周期略低于0.1m余量、2次内核超过50ms，仍有明显倒退和低速。
V7指上一实验控制器，不代表原生REMANI基线。场景参与过开发，结果不构成独立泛化证明。

每个最终场景独立分目录：

- `data/`：两版3个种子的原始记录。
- `gif/comparison.gif`：V7/V8并列对比。
- `gif/search.gif`、`gif/search_zoom.gif`：V8实际候选及局部放大。
- `gif/motion.gif`：简洁运动动画。
- `plots/`：轨迹、速度、净距和搜索快照。

GIF统一seed 1，碰撞后的旧版画面冻结，不代表仍在正常运行。
原始记录、旧构建及中间失败结果均未删除。回退选择`mppi_v7_search_01`原构建即可，不涉及生产源码回退。

附加回归：`recovery_contracts_09.json`、`search_contracts_09.json`、`v7_contracts_09.json`、
`tracking_contracts_09.json`、`static_empty_regression_09/summary.json`。
`optimization_equivalence_08_09.json`记录08/09六组已有轨迹的精确一致性，非全输入等价证明。
`../../finalize_v8.py`从冻结记录整理对照PNG，不修改原始数据或指标。
