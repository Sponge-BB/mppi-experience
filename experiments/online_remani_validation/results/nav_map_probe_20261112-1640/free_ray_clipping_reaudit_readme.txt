本轮仅重算已保存的73帧，不连接实车、不修改运行代码。

旧 continuous_cloud_static_voxel_audit.json 的方法描述只检查范围4.5米内回波，这与原生处理不等价：原生对于更远的有效回波，将终点截断到4.5米，仍保留近场清空射线。

新 continuous_cloud_native_clipping_reaudit.json 已修正，并包含原生投影窗口 |sensor xyz|<[8,8,3]、world z>=0.15、地图范围(-7.5,-5,0)+(15,10,3)、closetPointInMap 的 first-positive-boundary-minus-.001 裁剪，以及随后的4.5米长度裁剪。
73帧130217点中120636条通过投影检查，其中4221条原始回波超过4.5米，没有超地图边界的合格回波。修正后，9个阻塞体素与有效线段的三维几何相交次数仍全部为0。

本次检测的是欧氏射线段与5厘米体素盒子的相交，未复现原生DDA离散步进及去重。输入是最终 /velodyne_points_mapping 点云，缺少同时段原始/高度过滤前点云，因而不能断言真实光束没有扫过后墙，也不能证明未经过是由离散光束稀疏造成的。
具体检查脚本 recheck_free_rays_native_clip.py；结果文件包含每帧计数、9体素计数和所有输入SHA256。原报告未覆盖。
