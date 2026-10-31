# 2026-10-25：底盘种子参数化修复实验

## 本轮边界

用户要求执行下一步。先修已构造复现的反向/换向路径参数化问题，再验证是否改善完整 REMANI 的轨迹交付；不把原生 A* 成功等同于有效种子或最终轨迹成功。

仅隔离 source/build；生产 src/devel、旧 build/result 均不修改。固定机械臂，原生 ROS 数值 SIL，非 Gazebo/实车。ROS master 只用 localhost:11441，命令只在 /sim/cmd_vel，MPPI 为唯一模拟执行者。

原父版本：`builds/delivery_stability_20261024_02`。将使用同一新二进制，通过 `SEED_PARAMETERIZATION_FIX=0/1` 比较；默认关闭。关闭分支保留原函数语义，并验证源码精确反投影。

修复声明：相对坐标计算正反向距离；紧曲率度量不得小于真实平移弦长；时长与位移共用原有 0.7 倍加速度；有效输出整体提交，失败不保留半成品；同步位置求值与短段末端采样，禁止空种子继续传入机械臂采样。保持优化器、MPPI、碰撞外形/余量、轮速限制、0.6 秒求解预算不变。

## 预先固定的步骤

1. 六个实际替换函数提取回归 + 独立 helper 性质测试 + 源码开关/精确逆投影测试。
2. 独立审查修复范围、边界速度/换向、非法输入失败传播。
3. 新建隔离 build；原优化器 archive、MPPI library 字节不变。
4. 冻结所有根实验 Python 源码后，依次跑静态 seed11 OFF、11 ON、12 ON、12 OFF，各最多48秒场景时长。两组均启用相同诊断，关闭 prediction soft weights、admission 和额外 clearance recovery。保留所有失败，不挑成功重跑。
5. 全部测量结束、确认节点/端口退出后，输出 TXT、JSON 和简洁数据图。初始规划、重规划分开；区分原生 A*、参数化/前端、后端优化、硬检查、发布、实际采用。

闭环运行状态会分叉，不声称 ON/OFF 每次请求输入相同，不以两 seed 推导统计显著性或真实实时性能。

## 关机状态

2026-10-25 02:00 的既有授权关机已执行；今天12:29主机重新开机。本轮用户没有新关机要求，不重复设置。

## 进度

实现和独立代码审查完成；24项Python测试通过。实际六函数提取的C++性质回归已在 `results/seed_parameterization_geometry_20261025_02` 通过，`_01` 初始测试保留。

隔离 `builds/seed_parameterization_20261025_01` 编译完成。优化器 archive、MPPI library及其它执行节点与父版本字节相同；只替换search归档并重编规划器节点TUs。审查后覆盖继承的 `initialization_unchanged=true` 声明，明确本轮只改种子参数化；scope attestation不改二进制。

四组闭环已完成，接口审计全通过。`results/seed_parameterization_20261025_01/complete.json` 记录全部固定job，未删失败/未重跑。完整runtime202份Python两臂相同，只有显式修复开关不同。

结果：OFF 9次重规划有2次空种子，ON 8次全部产生非空正时长种子并进入后端；ON仍4次budget_timeout、2次left_wheel_acceleration。每组仅各1条重规划参考实际采用。交付时间 seed11 OFF/ON=7.773/6.087秒，seed12 OFF/ON=5.531/6.384秒。四组goal、记录无保守投影碰撞。不能据两seed/闭环分叉声称整体稳定或因果提速。

生成逐组TXT/JSON和 `seed_parameterization_report_20261025_01/seed_delivery.png`，配对/接口/构建契约全通过。简要解释在 `results/20261025_参数化修复与静态对照简要结论.txt`。

边界：原getFlatState非奇异端速度替代未改；seed有效不等于完整非完整轨迹可行；历史两条失败路径未做完整精确重放。当前仍需后端轮加速度/碰撞约束及预算内可行解诊断，不能宣称稳定交付任务完成。

最终独立核验：`results/seed_parameterization_verification_20261025_01/independent_audit_runtime_aware.json` 的 `all_checks_passed=true`：98生产文件、36隔离source、3原生binary+MPPI/archives、4份完整runtime、24份gzip原内容、全部图所有权/失败记录核查通过；60个记录节点PID已退出，11441空闲。首版直接比较root原文与runtime的检查忽略既有机械patch，原文件保留并由 `audit_correction_note.json` 标为superseded；最终版显式重建确定性patch后通过，没有放宽数据/执行证据检查。
