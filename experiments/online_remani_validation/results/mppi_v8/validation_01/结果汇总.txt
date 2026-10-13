# V8 候选覆盖与恢复实验

固定参考、理想CV与点归属、固定机械臂几何；不是完整REMANI、Gazebo或实车。
V7/V8使用相同场景、车体、0.1m余量、执行模型及2秒预测有效期。GIF固定seed 1，不挑最好结果。
前四场景V7使用此前冻结原始记录，新增场景重新运行V7；耗时不作严格跨批性能对比。

| 场景 | 版本 | 到达 | 碰撞 | 平均终止时间/s | 平均路径/m | 平均低速/s | 最小净距/m | 内核P95均值/ms |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| 单障碍对照 | V7 | 3/3 | 0 | 20.62 | 3.13 | 0.80 | 0.1001 | 25.72 |
| 单障碍对照 | V8 | 3/3 | 0 | 20.62 | 3.13 | 0.80 | 0.1001 | 26.20 |
| 双障碍交叉 | V7 | 3/3 | 0 | 20.62 | 3.13 | 0.80 | 0.1001 | 26.43 |
| 双障碍交叉 | V8 | 3/3 | 0 | 20.62 | 3.13 | 0.80 | 0.1001 | 26.79 |
| 三障碍连续交会 | V7 | 3/3 | 0 | 21.67 | 3.13 | 0.80 | 0.1001 | 26.98 |
| 三障碍连续交会 | V8 | 3/3 | 0 | 21.67 | 3.13 | 0.80 | 0.1001 | 27.47 |
| 四障碍密集交会 | V7 | 0/3 | 3 | 15.68 | 1.62 | 2.82 | -0.0037 | 27.72 |
| 四障碍密集交会 | V8 | 3/3 | 0 | 30.63 | 4.28 | 0.93 | 0.1000 | 41.68 |
| 单独迎面交会 | V7 | 0/3 | 3 | 12.95 | 1.95 | 1.68 | -0.0049 | 25.70 |
| 单独迎面交会 | V8 | 3/3 | 0 | 21.98 | 3.26 | 0.00 | 0.1000 | 37.59 |
| 四障碍镜像交会 | V7 | 0/3 | 3 | 15.77 | 1.59 | 2.67 | -0.0059 | 27.91 |
| 四障碍镜像交会 | V8 | 3/3 | 0 | 28.77 | 4.04 | 0.87 | 0.1000 | 41.66 |
| 四障碍延后交会 | V7 | 0/3 | 3 | 16.50 | 1.77 | 2.72 | -0.0070 | 27.82 |
| 四障碍延后交会 | V8 | 2/3 | 1 | 22.15 | 2.90 | 1.17 | -0.0003 | 40.22 |

碰撞组的终止时间是碰撞时刻，不是成功通行时间。净距不含额外安全余量，负值表示几何碰撞。

## 动画与数据

- 单障碍对照：[对照GIF](01_single/gif/comparison.gif) · [V8搜索](01_single/gif/search.gif) · [搜索放大](01_single/gif/search_zoom.gif) · [对照曲线](01_single/plots/comparison.png)
- 双障碍交叉：[对照GIF](02_two_crossing/gif/comparison.gif) · [V8搜索](02_two_crossing/gif/search.gif) · [搜索放大](02_two_crossing/gif/search_zoom.gif) · [对照曲线](02_two_crossing/plots/comparison.png)
- 三障碍连续交会：[对照GIF](03_three_sequential/gif/comparison.gif) · [V8搜索](03_three_sequential/gif/search.gif) · [搜索放大](03_three_sequential/gif/search_zoom.gif) · [对照曲线](03_three_sequential/plots/comparison.png)
- 四障碍密集交会：[对照GIF](04_four_dense/gif/comparison.gif) · [V8搜索](04_four_dense/gif/search.gif) · [搜索放大](04_four_dense/gif/search_zoom.gif) · [对照曲线](04_four_dense/plots/comparison.png)
- 单独迎面交会：[对照GIF](05_head_on_only/gif/comparison.gif) · [V8搜索](05_head_on_only/gif/search.gif) · [搜索放大](05_head_on_only/gif/search_zoom.gif) · [对照曲线](05_head_on_only/plots/comparison.png)
- 四障碍镜像交会：[对照GIF](06_four_mirrored/gif/comparison.gif) · [V8搜索](06_four_mirrored/gif/search.gif) · [搜索放大](06_four_mirrored/gif/search_zoom.gif) · [对照曲线](06_four_mirrored/plots/comparison.png)
- 四障碍延后交会：[对照GIF](07_four_late/gif/comparison.gif) · [V8搜索](07_four_late/gif/search.gif) · [搜索放大](07_four_late/gif/search_zoom.gif) · [对照曲线](07_four_late/plots/comparison.png)

浅蓝/浅红是通过/未通过碰撞检查的外层候选，绿色是所选预测，蓝色是实际轨迹及当前车体。
没有加入额外不确定性膨胀；没有解除原生安全门；没有在实车接口部署。
后续动作检查只在原2秒CV有效期内，不代表递归可行或长期安全证明。
内核计时不包含Python导出、真实感知及执行延迟；离线绘图不在控制循环内。

