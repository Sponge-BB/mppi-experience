#!/usr/bin/env python3
"""Draw only recorded numerical output; never invent trajectories or omit failures."""
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Circle
import numpy as np

FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
if Path(FONT).exists():
    if hasattr(font_manager.fontManager, 'addfont'):
        font_manager.fontManager.addfont(FONT)
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=FONT).get_name()
    else:
        font_manager.fontManager.ttflist.append(font_manager.FontEntry(fname=FONT, name='ValidationCJK'))
        plt.rcParams['font.family'] = 'ValidationCJK'
plt.rcParams['axes.unicode_minus'] = False
MODES = {'tracker': ('原 Tracker＋统一实验保护', '#606b77'),
         'mppi': ('MPPI＋统一实验保护', '#2678b2'),
         'mppi_cbf': ('MPPI＋CBF-QP＋统一实验保护', '#e87924')}
NAMES = {'empty':'空旷直行','curve':'弯曲参考路径','reverse':'后方同朝向目标',
         'new_static':'新出现静态障碍','crossing':'移动障碍横穿','delayed_crossing':'横穿＋延迟／掉帧',
         'blocked':'封闭通道（预期不通过）','late_intrusion':'近距离突然出现','lag_mismatch':'执行滞后失配'}
PROFILES = {'workspace':'当前实车预测参数','sensitivity':'小误差包络敏感性（非实车参数）'}


def read(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def trace(out, row):
    stem='__'.join(row[k] for k in ('scenario','profile','mode','seed'))
    return np.atleast_1d(np.genfromtxt(out/(stem+'.csv'),delimiter=',',names=True))


def render(out):
    rows=read(out/'runs.csv');refs=read(out/'references.csv');objects=read(out/'obstacles.csv')
    forecasts=read(out/'predictions.csv')
    limits=json.loads((out/'limits.json').read_text());manifest=json.loads((out/'manifest.json').read_text())
    geometry=json.loads((out/'geometry.json').read_text())
    metrics=[]
    for profile in dict.fromkeys(r['profile'] for r in rows):
        names=list(dict.fromkeys(r['scenario'] for r in rows if r['profile']==profile))
        fig,axes=plt.subplots(math.ceil(len(names)/3),3,figsize=(15,4.3*math.ceil(len(names)/3)),squeeze=False)
        for name,ax in zip(names,axes.flat):
            ref=np.array([(float(r['x']),float(r['y'])) for r in refs if r['scenario']==name])
            ax.plot(ref[:,0],ref[:,1],'k--',alpha=.45,lw=1,label='公共参考路径')
            for mode,(label,color) in MODES.items():
                group=[r for r in rows if r['scenario']==name and r['profile']==profile and r['mode']==mode]
                ds=[trace(out,r) for r in group]
                latency=np.concatenate([d['total_ms'] for d in ds])
                for r,d in zip(group,ds):
                    ax.plot(d['x'],d['y'],color=color,alpha=.9 if r['seed']=='0' else .35,lw=1.7 if r['seed']=='0' else 1,
                            label=label if r['seed']=='0' else None)
                    ax.scatter(d['x'][-1],d['y'][-1],s=32,marker='x' if int(r['collision']) else ('o' if int(r['reached']) else 's'),
                               color='red' if int(r['collision']) else color)
                metrics.append({'scenario':name,'profile':profile,'mode':mode,'runs':len(group),
                    'reached':sum(int(r['reached']) for r in group),'collisions':sum(int(r['collision']) for r in group),
                    'stop_mean_s':float(np.mean([float(r['stop_seconds']) for r in group])),
                    'reverse_mean_s':float(np.mean([float(r['reverse_seconds']) for r in group])),
                    'min_clearance':min(float(r['min_clearance']) for r in group),
                    'p95_ms':float(np.percentile(latency,95)),'p99_ms':float(np.percentile(latency,99)),
                    'max_ms':float(latency.max()),
                    **{key:sum(int(r[key]) for r in group) for key in ('qp_failed','unverified','interventions','guard_cycles','stale_cycles','deadline_misses','tracker_faults','cycles')}})
            static=name in ('new_static','late_intrusion','blocked')
            for stamp in ([5] if static else [3,5,7,9]):
                for o in objects:
                    if o['scenario']!=name or abs(float(o['t'])-stamp)>1e-7:continue
                    x,y,r=(float(o[k]) for k in ('x','y','r'))
                    ax.add_patch(Circle((x,y),r,color='#bd4540',alpha=.18))
                    if not static:ax.text(x+.25,y,f'{stamp}s',fontsize=7)
            ax.scatter(ref[0,0],ref[0,1],c='black',s=35,marker='o',zorder=5)
            ax.scatter(ref[-1,0],ref[-1,1],c='green',s=65,marker='*',zorder=5)
            ax.axhline(1.8,color='gray',lw=.8);ax.axhline(-1.8,color='gray',lw=.8)
            ax.set(title=NAMES[name],xlabel='X / m',ylabel='Y / m')
            ax.set_aspect('equal',adjustable='datalim');ax.grid(alpha=.2);ax.legend(fontsize=6.5,loc='best')
        for ax in list(axes.flat)[len(names):]:ax.axis('off')
        fig.suptitle(PROFILES[profile]+'｜局部控制层离线闭环，全部种子\n圆点＝到达，方块＝超时未到达，红叉＝碰撞；不是完整 REMANI／实车结果',fontsize=13)
        fig.tight_layout(rect=[0,0,1,.94]);fig.savefig(out/f'trajectories_{profile}.png',dpi=145);plt.close(fig)
        fig,axes=plt.subplots(2,2,figsize=(15,10));x=np.arange(len(names));width=.25
        for i,(mode,(label,color)) in enumerate(MODES.items()):
            group=[next(m for m in metrics if m['scenario']==n and m['profile']==profile and m['mode']==mode) for n in names]
            for ax,key in zip(axes.flat,['reached','stop_mean_s','min_clearance','p95_ms']):
                vals=[m[key]/m['runs'] if key=='reached' else m[key] for m in group]
                ax.bar(x+(i-1)*width,vals,width,color=color,label=label)
        for ax,title in zip(axes.flat,['到达比例（封闭通道预期为 0）','低速／停留时间均值 / s','所有种子的最小真值净距 / m','局部计算 P95 / ms']):
            ax.set_title(title);ax.set_xticks(x);ax.set_xticklabels([NAMES[n] for n in names],rotation=35,ha='right',fontsize=9)
            ax.grid(axis='y',alpha=.2);ax.legend(fontsize=7)
        axes[0,0].set_ylim(0,1.12);axes[1,0].axhline(0,color='red',ls='--')
        axes[1,1].axhline(50,color='red',ls='--',label='50 ms 实验周期')
        fig.suptitle(PROFILES[profile]+'｜三组对照：差异不等同于整套 REMANI 性能提升')
        fig.tight_layout(rect=[0,0,1,.96]);fig.savefig(out/f'summary_{profile}.png',dpi=145);plt.close(fig)
    for name in dict.fromkeys(r['scenario'] for r in rows):
        profile='workspace'
        chosen=[r for r in rows if r['scenario']==name and r['profile']==profile and r['seed']=='0']
        if not chosen:continue
        fig,axes=plt.subplots(6,1,figsize=(12,13),sharex=True)
        for r in chosen:
            d=trace(out,r);label,color=MODES[r['mode']]
            for ax,key in zip(axes[:4],['v','w','clearance','total_ms']):ax.plot(d['t'],d[key],color=color,label=label,lw=1.1)
            axes[4].step(d['t'],d['guard']+list(MODES).index(r['mode'])*1.3,where='post',color=color,label=label)
            if r['mode']=='mppi_cbf':
                axes[0].plot(d['t'],d['nominal_v'],ls=':',color=color,alpha=.7,label='MPPI 名义 v')
                axes[0].plot(d['t'],d['command_v'],ls='--',color=color,alpha=.7,label='最终目标 v')
                axes[5].step(d['t'],d['intervention'],where='post',label='CBF 介入',color=color)
                axes[5].step(d['t'],1.3+d['qp_failed'],where='post',label='QP／非线性验证失败（＋1.3）',color='red')
        for ax,label in zip(axes,['实际线速度 / m/s','实际角速度 / rad/s','真值净距 / m','完整局部计算 / ms','统一保护介入（错位显示）','CBF 状态（错位显示）']):
            ax.set_ylabel(label);ax.grid(alpha=.2);ax.legend(fontsize=7,loc='upper right')
        axes[2].axhline(0,color='red',ls='--');axes[2].axhline(limits['margin'],color='gray',ls=':')
        axes[3].axhline(50,color='red',ls='--');axes[-1].set_xlabel('仿真时间 / s')
        fig.suptitle(NAMES[name]+'｜当前参数，固定种子 0（不按成功结果选图）\n统一保护是实验装置，不是原 REMANI TTC 实现')
        fig.tight_layout(rect=[0,0,1,.955]);fig.savefig(out/f'detail_{name}.png',dpi=135);plt.close(fig)
    # Time-aligned snapshots make simultaneous physical separation observable.
    if any(r['scenario']=='crossing' and r['profile']=='workspace' for r in rows):
        fig,axes=plt.subplots(3,4,figsize=(16,11),sharex=True,sharey=True)
        for i,(mode,(label,color)) in enumerate(MODES.items()):
            r=next(r for r in rows if r['scenario']=='crossing' and r['profile']=='workspace' and r['mode']==mode and r['seed']=='0')
            d=trace(out,r)
            for j,stamp in enumerate((3.,5.,7.,9.)):
                ax=axes[i,j];idx=max(0,np.searchsorted(d['t'],stamp,side='right')-1);p=d[idx]
                ax.plot(d['x'][:idx+1],d['y'][:idx+1],color=color,lw=2)
                for bx,by,rad in geometry['body_disks_xy_r']:
                    x=p['x']+math.cos(p['yaw'])*bx-math.sin(p['yaw'])*by;y=p['y']+math.sin(p['yaw'])*bx+math.cos(p['yaw'])*by
                    ax.add_patch(Circle((x,y),rad,fill=False,color=color,alpha=.65))
                ax.arrow(p['x'],p['y'],.23*math.cos(p['yaw']),.23*math.sin(p['yaw']),width=.015,color=color)
                for o in objects:
                    if o['scenario']=='crossing' and abs(float(o['t'])-stamp)<1e-8:
                        ax.add_patch(Circle((float(o['x']),float(o['y'])),float(o['r']),color='#bd4540',alpha=.6))
                ax.plot([0,3],[0,0],'k--',alpha=.3);ax.scatter(3,0,c='green',marker='*')
                ax.set(title=f'{label}\nt={stamp:g} s',xlim=(-1,3.5),ylim=(-1.8,1.8));ax.set_aspect('equal');ax.grid(alpha=.2)
                if stamp>d['t'][-1]:ax.text(-.8,1.3,'该次运行已结束',fontsize=8,color='red')
        fig.suptitle('同一时刻的位置与整机固定外形｜当前参数，横穿，种子 0\n红色＝实际障碍物；彩色圆＝机器人固定姿态覆盖，不是预测膨胀区')
        fig.tight_layout(rect=[0,0,1,.94]);fig.savefig(out/'crossing_snapshots.png',dpi=145);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(13,5))
    for ax,profile in zip(axes,('workspace','sensitivity')):
        selected=[r for r in forecasts if r['scenario']=='crossing' and r['profile']==profile and abs(float(r['t'])-5)<1e-8]
        for row in sorted(selected,key=lambda r:-float(r['ahead'])):
            ahead=float(row['ahead']);x,y,r=(float(row[k]) for k in ('x','y','r'))
            ax.add_patch(Circle((x,y),r,fill=False,color=plt.cm.viridis(ahead/1.5),lw=2,label=f'+{ahead:g}s，障碍外形＋误差半径={r:.2f}m'))
            ax.scatter(x,y,s=20,color='black')
        ax.set(title=PROFILES[profile],xlim=(-.2,3.2),ylim=(-1.8,1.8),xlabel='X / m',ylabel='Y / m');ax.set_aspect('equal');ax.grid(alpha=.2)
        if selected:ax.legend(fontsize=8)
    fig.suptitle('直接调用工程 CV 预测函数：t=5s 的时变外形与误差包络\n不含机器人半径；右侧仅为敏感性假设，未修改实车参数')
    fig.tight_layout(rect=[0,0,1,.90]);fig.savefig(out/'prediction_envelopes.png',dpi=145);plt.close(fig)
    fig,ax=plt.subplots(figsize=(7,6));length,width=geometry['base_length'],geometry['base_width']
    ax.plot([-.5*length,.5*length,.5*length,-.5*length,-.5*length],[-.5*width,-.5*width,.5*width,.5*width,-.5*width],'k-',label='工程底盘矩形')
    for i,(x,y,r) in enumerate(geometry['body_disks_xy_r']):
        ax.add_patch(Circle((x,y),r,fill=False,color='#2678b2' if i<2 else '#e87924',label='底盘覆盖圆' if i==0 else ('固定机械臂投影覆盖' if i==2 else None)))
    pts=np.asarray(geometry['arm_collision_points_xyz']);ax.scatter(pts[:,0],pts[:,1],s=15,label='Piper DH 碰撞采样点')
    ax.set(xlim=(-.8,.8),ylim=(-.7,.7),xlabel='车体 X / m',ylabel='车体 Y / m',title='机械臂不运动，但不删除其碰撞占据')
    ax.set_aspect('equal');ax.legend(fontsize=8);ax.grid(alpha=.2);fig.tight_layout();fig.savefig(out/'geometry.png',dpi=145);plt.close(fig)
    (out/'metrics.json').write_text(json.dumps(metrics,indent=2,ensure_ascii=False)+'\n')
    report(out,metrics,limits,manifest)
    (out/'render_manifest.json').write_text(json.dumps({
        'plotter_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'runs_csv_sha256':hashlib.sha256((out/'runs.csv').read_bytes()).hexdigest(),
        'scope':'render only; benchmark traces and original source manifest unchanged'},indent=2)+'\n')


def report(out,metrics,limits,manifest):
    text=['# 固定机械臂：底盘 MPPI / CBF-QP 三组对照','',
      '这是算法层离线数值闭环，不是 Gazebo、实车或完整 REMANI 的 A/B 测试。全部图片来自本批 CSV。','',
      '## 对照口径','',
      '- 无 MPPI：直接执行工程 RealBaseTracker 类，ROS 传输替换为内存；参考点由统一几何路径适配器生成。',
      '- 有 MPPI：同一路径作为软参考，允许局部偏离；不是重新运行 REMANI 搜索/轨迹优化。',
      '- MPPI+CBF-QP：同一 MPPI 后接离散数值线性化屏障 QP，原始非线性约束复验。',
      '- 两个 MPPI 模式均包含终点位置／朝向分阶段收敛控制；切换半径为 max(0.20 m, 可用直行上限×预测时域)，仍通过各自过滤与统一保护，不是纯采样控制包办所有状态。',
      '- 三组均经过同一个实验保护：0.15 s 反应段＋制动预测检查，过期/超时请求制动。它不是生产 TTC 代码。',
      '- 原 Tracker 没有配上 REMANI 在线重规划；其绕不过静态障碍不能解释为原 REMANI 绕不过。',
      '- 封闭通道不应通过；必要等待、原地转向均计入低线速度时长，不能全部算误停。','',
      '## 工程关联与未复用部分','',
      '- 读取 exp0_real.launch 的最终解析参数，导出 effective_real_params.json，而非只读 YAML 默认值。',
      '- 直接编译调用 DynamicPointBuffer 的 CV 与误差包络函数；未启动节点，未运行检测/聚类/关联或 ESDF。',
      '- 固定机械臂使用当前初始关节角和 Piper DH 采样投影；额外工具余量 0.10 m 未经实车 mesh 标定。',
      '- 合成障碍用圆形完整外形，静态地图用墙/圆几何；并非实际点云，更不是把每个雷达点都当独立目标。',
      f"- 同时满足 Tracker 速度上限 {limits['vmax']:.3f} m/s、{limits['wmax']:.3f} rad/s，以及左右轮线速度 {limits['wheel_v']:.4f} m/s、轮线加速度 {limits['wheel_a']:.4f} m/s²；因此直行实际可用上限不是 0.30 m/s。",
      f"- 统一安全余量取工程底盘/机械臂余量的较大值 {limits['margin']:.3f} m（二维投影保守近似）。",
      f"- 三组统一按 20 Hz 评估；生产 Tracker 定时器是 {manifest['production_tracker_rate_hz']:g} Hz。原类控制循环被同步调用，没有复现真实线程调度。",
      '- 控制器执行滞后假设 0.12 s；lag_mismatch 真值滞后 0.28 s。都不是实车辨识结果。',
      '- workspace 全部使用当前误差参数；sensitivity 仅修改离线假设，用来分析保守性，不能当作实车效果。',
      '- 真值碰撞独立按 5 ms 检查；到达同时检查位置、朝向和速度。CSV 每条记录是区间末状态，控制量作用于刚结束的区间。',
      '- 计算耗时含合成观测、预测/采样/过滤/共同保护，未包含 ROS、ESDF、点云处理、串口；不是实车实时保证。',
      '- QP 失败或下游保护改变动作后，不把原 CBF 检查当作最终动作的安全认证。','',
      '## 图片','', '![当前参数全部轨迹](trajectories_workspace.png)','',
      '![当前参数三组统计](summary_workspace.png)','',
      '![同一时刻外形](crossing_snapshots.png)','',
      '![预测包络](prediction_envelopes.png)','',
      '## 完整统计（不删除失败种子）','',
      '|参数组|场景|方法|到达/次数|碰撞|停留均值 s|最小净距 m|P95/P99/max ms|QP失败|保护周期|超时|',
      '|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for m in metrics:
        text.append(f"|{m['profile']}|{NAMES[m['scenario']]}|{m['mode']}|{m['reached']}/{m['runs']}|{m['collisions']}|{m['stop_mean_s']:.2f}|{m['min_clearance']:.3f}|{m['p95_ms']:.2f}/{m['p99_ms']:.2f}/{m['max_ms']:.2f}|{m['qp_failed']}|{m['guard_cycles']}|{m['deadline_misses']}|")
    text+=['','## 能与不能得出的结论','',
      '可以比较：固定参考任务下，增加局部采样控制/安全过滤后，本批场景的到达、净距、停留、计算时间如何变化。',
      '不能宣称：超过完整 REMANI baseline、实车能立即避让、CBF 保证无碰撞、预测器精度已验证。',
      '当前是启发候选引导的采样加权 MPPI 原型；CBF 是顺序线性化的离散屏障实验，不是全时域鲁棒安全证明。',
      '如果 MPPI+CBF 比 MPPI 更慢或更容易停车，需分析模型一致性、约束可行性和预测包络，而不能隐藏该结果。','',
      '## 可复现与回退','',
      'manifest.json 保留代码与生产配置哈希、编译命令、运行参数。verification.txt 为独立数值一致性检查。',
      '只新增 experiments/base_mppi_cbf_compare/；不修改 src/、实车 launch 或驱动，不连接 ROS master。移走新增目录即可撤回本轮实验。']
    (out/'REPORT.md').write_text('\n'.join(text)+'\n')


if __name__=='__main__':
    render(Path(sys.argv[1]).resolve())
