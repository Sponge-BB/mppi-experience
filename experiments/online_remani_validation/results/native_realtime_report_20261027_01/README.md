# 原生3D按需计算实验

两组都使用原生三维碰撞；差异仅空旷分支候选调用。单种子案例，不是统计成功率。

| 场景/组 | 结束原因 | 实际运动触发 | 障碍实际出现 | 有效3D动态输入 | 3D采样接触 | 最小3D净距/m | 内核P50/ms | deadline周期 |
|---|---|---|---|---|---|---:|---:|---:|
| static/original | deadline_zero_deadlock | False | False | False | False | 1.2326 | 74.39 | 103 |
| static/demand | timeout | True | True | False | False | 0.0926 | 0.77 | 40 |
| crossing/original | deadline_zero_deadlock | False | False | False | False | 1.2326 | 74.42 | 101 |
| crossing/demand | goal | True | True | True | False | 0.5468 | 2.78 | 105 |
| dynamic_head_on_center/original | deadline_zero_deadlock | False | False | False | False | 1.2326 | 72.71 | 106 |
| dynamic_head_on_center/demand | native3d_sample_collision | True | True | True | True | -0.0011 | 12.31 | 40 |

deadline_zero_deadlock是共同早停失败，不是到达或避障通过。没有实际运动/障碍暴露的场次，不能被当作动态安全样本。
真实3D采样接触/净距与旧二维投影指标分别保存在JSON；不把投影相交等同三维接触。失败不删除、不替换种子。
