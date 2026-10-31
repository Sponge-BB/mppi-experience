# 双时间尺度耦合优化：2026-10-22先导验证

已完成15次在线数值仿真；不是Gazebo/实车。**机制接通，但未证明性能提升；静态双项组退化。**

- [先读验证结论](验证结论.txt)
- [简明数据图](key_figure_01/验证要点.png)
- [静态GIF](visuals/static_key/overview.gif) / [四动态GIF](visuals/dynamic_four_key/overview.gif)
- [逐次数据](report_01/逐次结果.csv) / [分组汇总](report_01/分组汇总.csv)
- [机器可读报告](report_01/summary.json)
- [瓶颈源码与日志核对](bottleneck_audit_01/)
- [数学与原生梯度检查](verification/native_01/verification.json)
- [完整先导原记录](pilot_01/)

串联组也含MPPI；耦合组额外含投影补项，分离耦合贡献要与engineering组比较。
三场景×五组，每格仅一个种子；不得将“都到达”当成“慢层都成功交付”。

`smoke_01`为空场接线检查；`qualification_endpoint`为历史状态末端资格复现；
`report_partial_01`保留空间中断时的临时报告，最终只用`report_01`。
`visuals/oversize_attempts`为超过初始5MiB存储目标的首次GIF输出，不是额外算法试次；
最终GIF为`*_key`，对应独立审计清单。未记录搜索候选，不补画搜索。

碰撞快照使用无损归档，逐文件哈希在各trial的`collision_snapshot_archive.json`。
读取具体快照前用`archive_coupling_snapshots.py restore --trial <trial> --restore-to <新目录>`恢复。
不得把已归档的原NPZ目录当成仍在原位。生产源码和devel保持不变。

存储余量约290MiB；更多实验前需要整理空间。当前不建议把本原型直接替换实车模块。
