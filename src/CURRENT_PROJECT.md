# 当前工程与验证状态（2026-11-17）

本仓库使用工作空间布局：`src/` 对应 l 端 `/home/l/remani_ws/src`；`experiments/` 包含实验代码、报告及保留的图/GIF。当前运行代码版本为 `20261117-125232-planar-motion-surface`，父版本 `20261117-125138-exact-esdf-scratch`。本次上传本身不修改实车控制逻辑。

本轮修复与限制：动态表面运动验收改为与二维平移估计一致的XY距离，完整XYZ碰撞点保留；45项测试通过，4836帧历史输入回放中，动态输出由全空变为51帧非空，仍非连续跟踪或实车避障验收。ESDF精确整数范围内缩减临时存储，完整库99项测试通过，历史裁剪数据仅约1.52%收益，未证明在线卡顿已解决。机械臂反馈新增阶段诊断，秒级主机反馈空档具体阻塞位置仍待定位；11mm局部自滤候选没有默认启用。

[完整结果与版本链](../experiments/online_remani_validation/results/repair_20261117-120602/RESULTS.txt)
[长方形障碍影响机械臂的仿真对照](../experiments/online_remani_validation/results/arm_box_sim_20261114-210347/report/comparison.html)
[后续前缀保留对照动画](../experiments/online_remani_validation/results/arm_box_refresh_20261115-160051/report/comparison.gif)

本轮保留远端原有文件与历史，在相同路径同步当前本机文件、补充缺失内容。新上传材料省略原始大体积bag、逐帧地图数组、重复runtime副本、build/devel与缓存，原件仍在本机。已有历史远端文件未因本次筛选而删除。增量文件及SHA256见 `../UPLOAD_MANIFEST_20261117.json`；旧 `UPLOAD_MANIFEST.json` 对应旧发布快照。

本机源码与实际部署产物的恢复记录在 `/home/l/remani_ws/rollback/VERSIONS.tsv`，GitHub源码不能恢复外部SDK、操作系统、固件或控制器状态。新电脑须准备ROS Noetic、Piper SDK、Livox外部工作空间等依赖，在干净catkin构建中重新编译；不要复用别的版本build。
