#!/usr/bin/env python3
"""Read-only audit/plots and optional GIFs for the cusp adjustment fixtures."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from run_mppi_cusp_adjustment import HERE, BUILDS, verify_build
from render_dual_timescale import render, FONT

FINAL='gated_objective'
FINAL_SUITE='gated_objective_validation_01'
NAMES=dict(baseline='原版V12',phase='强制阶段',phase_objective='强制阶段＋动态评分',
           gated_phase='仅门控接续',gated_objective='门控＋一致阶段目标')
COLORS=dict(baseline='#909090',phase='#b99069',phase_objective='#9b808e',gated_phase='#8b9f83',gated_objective='#3f7f9f')


def read(path):return json.loads(path.read_text())


def longest(mask,dt=.05):
    best=current=0.
    for v in mask:
        current=current+dt if v else 0.;best=max(best,current)
    return best


def audit_case(path):
    result,meta=read(path/'result.json'),read(path/'metadata.json')
    state=np.atleast_1d(np.genfromtxt(path/'state.csv',delimiter=',',names=True,encoding='utf8',dtype=None))
    ref=np.atleast_1d(np.genfromtxt(path/'reference.csv',delimiter=',',names=True))
    decisions=[json.loads(s) for s in (path/'decisions.jsonl').read_text().splitlines()]
    if len(state)!=result['steps'] or len(state)!=len(decisions):raise ValueError('record length mismatch')
    if not np.isfinite(np.column_stack([state[k] for k in ('t','x','y','yaw','v','w','cmd_v','cmd_w')])).all():
        raise ValueError('nonfinite execution')
    unfinished=(state['goal_distance']>.05)|(abs(np.arctan2(np.sin(state['yaw']-ref['yaw'][-1]),
        np.cos(state['yaw']-ref['yaw'][-1])))>.1)
    active=(state['reason']=='ok')&(abs(state['cmd_v'])<1e-6)&(abs(state['cmd_w'])<1e-6)&unfinished
    events=[]
    for index,decision in enumerate(decisions):
        phase=decision.get('cusp_debug')
        if phase and phase[10]==2:
            previous=decisions[index-1]['cusp_debug']
            # At most one measured step advance; gates must already have held
            # on the preceding actual measurement (confirmation count == 1).
            if index<1 or previous[11]!=1 or phase[1]!=previous[1]+1:raise ValueError('invalid cusp transition')
            s=state[index]
            old_end=int(previous[4]);distance=np.hypot(s['x']-ref['x'][old_end],s['y']-ref['y'][old_end])
            yaw=abs(np.arctan2(np.sin(s['yaw']-ref['yaw'][old_end]),np.cos(s['yaw']-ref['yaw'][old_end])))
            if distance>.04+1e-9 or yaw>.15+1e-9 or abs(s['v'])>.025+1e-9 or abs(s['w'])>.15+1e-9:
                raise ValueError('transition violates measured gate')
            events.append(dict(t=decision['t'],phase=int(phase[1]),distance_m=float(distance),
                yaw_error_rad=float(yaw),v_mps=float(s['v']),w_radps=float(s['w']),
                association_active=decision.get('cusp_bound',True)))
    velocity=np.column_stack((state['v'],state['w']))
    acceleration=np.diff(velocity,axis=0)/.05
    limits=meta['limits'];half=.5*limits['track']
    ratios=[np.max(abs(acceleration[:,0]))/limits['accel'],
            np.max(abs(acceleration[:,1]))/limits['angular_accel'],
            np.max(abs(acceleration[:,0]-half*acceleration[:,1]))/limits['wheel_a'],
            np.max(abs(acceleration[:,0]+half*acceleration[:,1]))/limits['wheel_a']]
    if max(ratios)>1+1e-7:raise ValueError('plant acceleration/wheel bound violation')
    if result['max_command_bound_ratio']>1+1e-8:raise ValueError('command bound violation')
    return dict(path=str(path),suite=path.parts[-5],case=result['case'],source=path.parts[-4],seed=meta['seed'],
        variant=meta['experiment_context']['variant'],passed=result['passed'],collision=result['collision'],
        duration_s=result['simulated_duration_s'],endpoint_reached=result['endpoint_reached'],
        first_rejoin_s=result['first_sustained_rejoin_s'],min_gap_m=result['min_physical_gap_m'],
        callback_p95_ms=result['callback_p95_ms'],deadline_misses=result['deadline_misses'],
        reason_counts=result['reason_counts'],longest_unfinished_active_zero_s=longest(active),
        total_unfinished_active_zero_s=float(np.count_nonzero(active)*.05),
        final_goal_distance_m=float(state['goal_distance'][-1]),
        max_command_bound_ratio=result['max_command_bound_ratio'],
        bound_cycles=sum(bool(d.get('cusp_bound')) for d in decisions),
        blocked_cusp_cycles=sum(d.get('cusp_estimated_gap_m',1.)<0 for d in decisions),
        reset_events_s=[d['t'] for d in decisions if d.get('one_time_reset')],
        max_average_plant_acceleration_bound_ratio=max(ratios),phase_events=events,
        reference_sha256=meta['reference_sha256'],geometry_sha256=meta['geometry_sha256'],
        library_sha256=meta['library_sha256'],plant_sha256=meta['plant_method_sha256'],limits=limits,
        initial_state=meta['initial_state'],world_criteria=meta['criteria'],obstacle_center=meta['obstacle_center'])


def replay_equal(now,old):
    a=np.atleast_1d(np.genfromtxt(now/'state.csv',delimiter=',',names=True,encoding='utf8',dtype=None))
    b=np.atleast_1d(np.genfromtxt(old/'state.csv',delimiter=',',names=True,encoding='utf8',dtype=None))
    keys=('t','x','y','yaw','v','w','cmd_v','cmd_w')
    return bool(len(a)==len(b) and all(np.array_equal(a[k],b[k]) for k in keys))


def plot(root,records):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    font=FontProperties(fname=FONT,size=10)
    focus=root/'focus_02/source_02/seed_01'
    fig,axes=plt.subplots(2,1,figsize=(9,6),sharex=True)
    for variant in ('baseline',FINAL):
        path=root/FINAL_SUITE/'source_02/seed_01'/variant/'dynamic_crossing'
        state=np.genfromtxt(path/'state.csv',delimiter=',',names=True,encoding='utf8',dtype=None)
        axes[0].plot(state['t'],state['goal_distance'],color=COLORS[variant],label=NAMES[variant])
        axes[1].plot(state['t'],state['v'],color=COLORS[variant],label=NAMES[variant])
    axes[0].set_ylabel('距最终目标 / m',fontproperties=font)
    axes[1].set_ylabel('实际线速度 / m/s',fontproperties=font)
    axes[1].set_xlabel('仿真时间 / s',fontproperties=font)
    for axis in axes:axis.grid(alpha=.2);axis.legend(prop=font,frameon=False)
    fig.suptitle('同一失败样本：阶段接续能否解除停滞',fontproperties=font)
    fig.tight_layout(rect=[0,0,1,.94]);fig.savefig(root/'01_stall_comparison.png',dpi=160);plt.close(fig)
    formal=[r for r in records if '/'+FINAL_SUITE+'/' in r['path']]
    cases=['native_turn','offset_p010','offset_n010','offset_p020','offset_n020',
           'offset_p030','offset_n030','static_rejoin','dynamic_crossing']
    case_names=dict(native_turn='无障碍',offset_p010='+10cm',offset_n010='−10cm',
        offset_p020='+20cm',offset_n020='−20cm',offset_p030='+30cm',offset_n030='−30cm',
        static_rejoin='静态绕障',dynamic_crossing='动态横穿')
    fig,axes=plt.subplots(1,2,figsize=(12,6),sharex=True)
    for axis,source in zip(axes,('source_01','source_02')):
        for vi,variant in enumerate(('baseline',FINAL)):
            chosen=[next(r for r in formal if r['source']==source and r['variant']==variant and r['case']==case) for case in cases]
            y=np.arange(len(cases))+(vi-.5)*.30
            bars=axis.barh(y,[r['duration_s'] for r in chosen],height=.28,color=COLORS[variant],label=NAMES[variant])
            for bar,r in zip(bars,chosen):
                if not r['passed']:
                    bar.set_hatch('//');axis.text(r['duration_s']+.2,bar.get_y()+bar.get_height()/2,'未完成',va='center',fontproperties=font,color='#ab493f')
        axis.set_yticks(np.arange(len(cases)));axis.set_yticklabels([case_names[x] for x in cases],fontproperties=font)
        axis.invert_yaxis();axis.set_xlim(0,65);axis.grid(axis='x',alpha=.2)
        axis.set_title('原生参考 '+source[-2:],fontproperties=font);axis.set_xlabel('运行的仿真时间 / s',fontproperties=font)
    axes[0].legend(prop=font,frameon=False,loc='upper center',bbox_to_anchor=(1.05,1.17),ncol=2)
    fig.tight_layout(rect=[0,0,1,.95]);fig.savefig(root/'02_regression_results.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(13,4.5))
    for vi,variant in enumerate(NAMES):
        suite=FINAL_SUITE if variant in ('baseline',FINAL) else 'gated_validation_01' if variant=='gated_phase' else 'regression_01'
        chosen=[r for r in records if '/'+suite+'/' in r['path'] and r['variant']==variant]
        axes[0].bar(vi,max(r['callback_p95_ms'] for r in chosen),color=COLORS[variant])
        axes[1].bar(vi,max(r['longest_unfinished_active_zero_s'] for r in chosen),color=COLORS[variant])
    for axis in axes:
        axis.set_xticks(range(5));axis.set_xticklabels(list(NAMES.values()),fontproperties=font,rotation=18)
        axis.grid(axis='y',alpha=.2)
    axes[0].axhline(50,color='#ba6556',linestyle='--',linewidth=1)
    axes[0].set_ylabel('各工况P95的最大值 / ms',fontproperties=font)
    axes[0].set_title('完整决策与发布前验收耗时',fontproperties=font)
    axes[1].set_ylabel('最长连续主动零指令 / s',fontproperties=font)
    axes[1].set_title('排除已到最终位姿后的停车',fontproperties=font)
    fig.tight_layout();fig.savefig(root/'03_timing_and_stops.png',dpi=160);plt.close(fig)


def write_report(root,summary):
    records=summary['records'];selected=[r for r in records if '/'+FINAL_SUITE+'/' in r['path']]
    def one(variant,case,source='source_02'):
        return next(r for r in selected if r['variant']==variant and r['case']==case and r['source']==source)
    old,new=one('baseline','dynamic_crossing'),one(FINAL,'dynamic_crossing')
    lines=['MPPI换向停滞：调整实验结果与结论','日期：2026-10-18（Asia/Shanghai）','',
      '一、范围',
      '本轮为固定真实原生REMANI完整参考的数值闭环，固定机械臂、差速底盘和完整机器人投影。',
      'MPPI与底盘重新计算，不是播放历史记录；观测是理想同步输入，不经过真实检测器。',
      '世界按固定0.05s步进；实测决策耗时参与50ms截止验收，但没有重放真实ROS排队、感知与定位时延。',
      '基准冻结native_v12_07，不等于后续shared_collision等候选，更不是Gazebo或实车。',
      '没有修改98项生产源码/配置，没有启动ROS/硬件，没有发送实车指令或关机。','',
      '二、五组对照（两条参考，各9工况）']
    for variant,data in summary['variants'].items():
        lines.append('%s：%d/%d通过；碰撞%d；50ms截止失败累计%d。'%(NAMES[variant],data['passed'],data['count'],data['collisions'],data['deadline_misses']))
    lines+=['',
      '通过必须同时满足：持续安全回线2秒、最终位置/朝向/速度稳定1秒、完整投影无碰撞及命令不超限。',
      '样本数很小，不把这组工况通过比例宣传成真实环境导航成功率。','',
      '三、先前停滞样本',
      '原版：运行%.2fs仍未到最终目标，剩余%.3fm，未完成任务时最长连续主动零指令%.2fs。'%(old['duration_s'],old['final_goal_distance_m'],old['longest_unfinished_active_zero_s']),
      '门控接续：运行%.2fs，完整判据%s；同类最长连续主动零指令%.2fs。'%(new['duration_s'],'通过' if new['passed'] else '未通过',new['longest_unfinished_active_zero_s'])]
    for event in new['phase_events']:
        lines.append('实际阶段切换t=%.2fs：距换向点%.2fcm，姿态误差%.3frad，实际v=%.5fm/s、w=%.3frad/s。'%(event['t'],event['distance_m']*100,event['yaw_error_rad'],event['v_mps'],event['w_radps']))
    lines+=['',
      '四、为什么否决第一版',
      '强制全程跟随当前前进/倒退段虽然解除了动态定向停滞，但两条静态绕障均未完成。',
      '静态障碍出现后，原换向位置的完整投影最小净距约-0.42m，即已被判碰撞。',
      '强制先返回该位置再进入下一段，会阻止原来允许的安全绕行和后续接回。',
      '另外调整动态阶段评分也不能解决这个静态几何问题，不能只展示定向成功GIF。','',
      '五、保留的调整：门控局部换向接续',
      '1. 保存完整原生参考，显式记录当前方向段；预测rollout无权推进实际阶段。',
      '2. 距当前换向点和剩余弧长均在20cm内，且换向点的完整投影在当前估计碰撞层未被占据，才接管关联。',
      '   这只是关联门控，不是路径可达性或安全证书；所选候选仍需原来的完整轨迹及制动验收。',
      '3. 距换向点<=4cm、剩余弧长<=5cm、姿态误差<=0.15rad、|v|<=0.025m/s、|w|<=0.15rad/s，',
      '   连续两个实际控制周期满足条件，才进入下一方向段。正常换向需要减速，不承诺完全不停。',
      '4. 切换时清理旧阶段的warm和局部绕行关联，但不清零实际状态或上次下发的执行指令。',
      '5. 换向点被挡时，不强制它作为必经目标，保留原版绕行关联；没有强制最小速度或取消停车。',
      '   此时cusp_bound=false，cusp_debug只是影子阶段记录，不能用它判断实际参考冻结。',
      '6. 换向附近的候选引导和评分都使用当前阶段终点，而不是引导要求对齐、评分仍奖励旧切线。',
      '   最终任务目标仍是完整参考终点，不能把中间换向点误判为任务完成；原权重保持不变。',
      '仅加入门控但不统一阶段目标的版本，在参考01动态样本中出现约0.21rad姿态残差选停，已否决。','',
      '六、静态表现与代价']
    for source in ('source_01','source_02'):
        a,b=one('baseline','static_rejoin',source),one(FINAL,'static_rejoin',source)
        lines.append('%s静态：原版%.2fs，门控版%.2fs；门控接管%d周期，换向点被判占据%d周期。'%(source,a['duration_s'],b['duration_s'],b['bound_cycles'],b['blocked_cusp_cycles']))
    differences=[one(FINAL,case,source)['duration_s']-one('baseline',case,source)['duration_s']
                 for source in ('source_01','source_02') for case in ('native_turn','offset_p010','offset_n010','offset_p020','offset_n020','offset_p030','offset_n030')]
    lines+=['正常/侧移工况的完成时间差（门控－原版）范围%.2f～%.2fs；并非每个场景都更快。'%(min(differences),max(differences)),
      '本轮修复目标是可靠接续，不能据此宣称整体避障速度或REMANI求解时间全面改善。','',
      '七、额外动态种子与仅重置缓存对照']
    extra=[r for r in records if '/dynamic_seeds_01/' in r['path']]
    for variant in ('baseline',FINAL):
        values=[r for r in extra if r['variant']==variant]
        lines.append('额外种子2、3，%s：%d/%d完整通过，碰撞%d，截止失败%d。'%(NAMES[variant],sum(r['passed'] for r in values),len(values),sum(r['collision'] for r in values),sum(r['deadline_misses'] for r in values)))
    for r in summary['reset_control']:
        reset_times='、'.join('%.2fs'%t for t in r['reset_events_s']) or '未触发'
        lines.append('原版持续选零1秒后重置一次的对照，%s：重置时刻%s；%s，运行%.2fs，剩余目标距离%.3fm。'%(r['source'],reset_times,'通过' if r['passed'] else '未通过',r['duration_s'],r['final_goal_distance_m']))
    lines+=['重置对照仅检验这一重置策略，不证明任何形式的缓存处理都无效。','',
      '八、仍未解决/未验证',
      '本轮不改REMANI的目标选择、失败重试、预算或初始化；不能宣称慢层交付慢已解决。',
      '不涉及Tracker执行链的横向残差恢复死锁；不能把本轮替代Tracker控制律修复。',
      '未验证新参考频繁替换、复杂多段往返、自交/环形路线、多动态目标及感知/定位误差。',
      '本轮没有修改mppi_node/progress.py的对外进度反馈；后续在线接入需使其与内核实际阶段一致，',
      '不能继续维护两套互相矛盾的参考进度，也不能仅替换库就宣称全局—局部协同已修好。',
      '完整在线REMANI、Gazebo和实车仍需另外接入回归；门控不构成全局活性或安全证明。','',
      '九、文件与回退',
      '01_stall_comparison.png：原停滞样本的最终目标距离与实际速度。',
      '02_regression_results.png：原版与最终候选的全部18工况结果，未完成样本明确标出。',
      '03_timing_and_stops.png：各工况P95的最大值及未完成时最长主动零指令。',
      'GIF_INDEX.md：分类GIF（包含被否决的第一版静态失败）。',
      'metrics.csv与summary_audit.json：原始指标与审计；各运行目录保留状态、候选、选择和阶段事件。',
      '原版builds/native_v12_07与旧数据均未覆盖；最终候选builds/cusp_gated_objective_01。',
      '当前未替换实车默认入口，回退只需继续选择原版实验构建，无生产配置需要回滚。']
    (root/'实验结果与改进总结.txt').write_text('\n'.join(lines)+'\n')
    (root/'README.md').write_text('# MPPI换向接续调整实验\n\n'
        '数值仿真，非Gazebo/实车。基准为冻结native_v12_07。\n\n'
        '[完整总结](实验结果与改进总结.txt)、[数据](metrics.csv)、[审计](summary_audit.json)、[GIF分类](GIF_INDEX.md)。\n\n'
        '![停滞对照](01_stall_comparison.png)\n\n![全工况回归](02_regression_results.png)\n')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True)
    p.add_argument('--render',action='store_true');a=p.parse_args();root=a.root.resolve()
    manifest=read(HERE/'builds/native_v12_07/build_manifest.json');workspace=HERE.parents[1]
    for name,digest in manifest['production_sources_sha256'].items():
        if hashlib.sha256((workspace/name).read_bytes()).hexdigest()!=digest:raise ValueError('production changed '+name)
    records=[]
    for suite in ('focus_02','regression_01','gated_validation_01',FINAL_SUITE,'dynamic_seeds_01','reset_control_01'):
        if not (root/suite).exists():continue
        for path in sorted((root/suite).glob('source_*/seed_*/*/*/result.json')):records.append(audit_case(path.parent))
    expected=54
    formal=[r for r in records if '/regression_01/' in r['path']]
    if len(formal)!=expected:raise ValueError('formal regression incomplete')
    selected=[r for r in records if '/'+FINAL_SUITE+'/' in r['path']]
    if len(selected)!=36:raise ValueError('selected gated regression incomplete')
    replay=[]
    old=HERE/'results/dual_timescale_20261018/local_rejoin_01'
    for r in formal:
        if r['variant']=='baseline':
            replay.append(dict(source=r['source'],case=r['case'],
                identical_state_and_command=replay_equal(Path(r['path']),old/r['source']/r['case'])))
    groups={}
    for r in records:groups.setdefault((r['source'],r['seed'],r['case']),[]).append(r)
    for key,values in groups.items():
        for field in ('reference_sha256','geometry_sha256','plant_sha256','limits','initial_state','world_criteria','obstacle_center'):
            if any(v[field]!=values[0][field] for v in values):raise ValueError('unpaired fixture '+str(key))
    summary=dict(scope='Paired fixed-native-reference numerical adjustment experiment, not Gazebo or hardware',
        production_sources_unchanged=98,builds={v:verify_build(b) for v,b in BUILDS.items()},
        strict_screening_trials=len(formal),selected_validation_trials=len(selected),records=records,baseline_replay=replay,
        reset_control=[r for r in records if r['variant']=='reset_only'],
        variants={v:dict(count=len(chosen),passed=sum(r['passed'] for r in chosen),
            collisions=sum(r['collision'] for r in chosen),deadline_misses=sum(r['deadline_misses'] for r in chosen),
            min_gap_m=min(r['min_gap_m'] for r in chosen),
            longest_active_zero_s=max(r['longest_unfinished_active_zero_s'] for r in chosen))
            for v in NAMES for chosen in [[r for r in
                (selected if v in ('baseline',FINAL) else [x for x in records if '/gated_validation_01/' in x['path']] if v=='gated_phase' else formal)
                if r['variant']==v]]})
    (root/'summary_audit.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
    with (root/'metrics.csv').open('w') as f:
        fields=('suite','source','seed','variant','case','passed','duration_s','min_gap_m','callback_p95_ms',
            'deadline_misses','longest_unfinished_active_zero_s','final_goal_distance_m','path')
        writer=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(records)
    plot(root,records)
    write_report(root,summary)
    if a.render:
        fixtures=[(FINAL_SUITE,'source_02','dynamic_crossing',FINAL,'01_dynamic_stall'),
                  ('focus_02','source_02','dynamic_crossing','phase_objective','02_dynamic_stage_objective'),
                  (FINAL_SUITE,'source_01','static_rejoin',FINAL,'03_static_reference01'),
                  (FINAL_SUITE,'source_02','static_rejoin',FINAL,'04_static_reference02'),
                  (FINAL_SUITE,'source_02','offset_p030',FINAL,'05_offset30'),
                  ('regression_01','source_01','static_rejoin','phase','06_rejected_strict_phase'),
                  ('gated_validation_01','source_01','dynamic_crossing','gated_phase','07_rejected_yaw_stall'),
                  (FINAL_SUITE,'source_01','dynamic_crossing',FINAL,'08_dynamic_reference01')]
        for suite,source,case,variant,name in fixtures:
            base=root/suite/source/'seed_01';out=root/'figures'/name
            if not (out/'overview.gif').exists():
                render([base/'baseline'/case,base/variant/case],out,
                    [NAMES['baseline'],NAMES[variant]],playback=2.,step=.2)
            print('CUSP_GIF',name,flush=True)
        (root/'GIF_INDEX.md').write_text('# 换向接续实验GIF\n\n固定原生完整参考，无在线重规划；重新运行MPPI而非播放历史轨迹。\n\n2倍播放；完整投影、实际参考、获准局部轨迹及真实搜索抽样。\n\n'+
            '\n'.join('- [%s](figures/%s/overview.gif)'%(name,name) for *_,name in fixtures)+'\n')
    print('CUSP_AUDIT',summary['variants'],flush=True)


if __name__=='__main__':main()
