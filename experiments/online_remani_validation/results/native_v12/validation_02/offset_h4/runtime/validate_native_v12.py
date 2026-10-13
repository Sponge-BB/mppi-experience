#!/usr/bin/env python3
"""Predeclared serial V12 online validation, horizon and perception ablations."""
import argparse
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from dynamic_scenarios import CASES
from run import run
from compare_native_v9 import load,metrics,compare
from audit_native_reference import audit as reference_audit
from audit_execution import audit as execution_audit
from render_native_search import render

HERE=Path(__file__).resolve().parent


def declared(build,perception_build):
    specs=[dict(key=case+'_h3',scene=scene,prediction=3.,build=str(build),delay=0.,group='main')
           for scene,(case,_,_) in CASES.items()]
    specs+=[dict(key=scene+'_h3',scene=scene,prediction=3.,build=str(build),delay=0.,group='regression')
            for scene in ('empty','static')]
    for scene,name in (('dynamic_head_on_center','center'),('dynamic_head_on','offset')):
        specs+=[dict(key=name+'_h'+str(int(h)),scene=scene,prediction=h,build=str(build),delay=0.,group='horizon')
                for h in (2.,4.)]
    specs+=[dict(key='center_perception_only_h2',scene='dynamic_head_on_center',prediction=2.,
                 build=str(perception_build),delay=0.,group='perception_ablation'),
            dict(key='center_h3_repeat',scene='dynamic_head_on_center',prediction=3.,build=str(build),delay=0.,group='repeat'),
            dict(key='center_h3_delay3',scene='dynamic_head_on_center',prediction=3.,build=str(build),delay=3.,group='reference_delay')]
    return specs


def summary(out,specs):
    records=[];lines=['# V12 在线验证与对照','',
        '原生 REMANI 在线规划/重规划 + 固定臂底盘 MPPI，隔离 ROS 数值仿真。',
        '主场景均3s有效期，另做2/4s时域、仅改感知、重复运行和参考交付延迟检查。',
        '全部计划运行和失败均保留；不同在线运行的原生搜索与调度存在差异，不能当作统计优势。','',
        '| 试验 | 结果 | 运动用时/s | 最小净距/m | 首次CV输入/s | 内核P95/ms | 全局GIF | 搜索GIF |',
        '| --- | --- | ---: | ---: | ---: | ---: | --- | --- |']
    for spec in specs:
        p=out/spec['key'];row=dict(spec)
        if not (p/'result.json').exists():row['result']='incomplete';records.append(row);continue
        trial=load(p);r=trial['result'];m=metrics(trial);row.update(m)
        anchor=r['motion_anchor'] or r['goal_sent'] or 0.;first=next((x for x in trial['diag'] if x.get('observed_obstacles')),None)
        row['first_dynamic_input_s']=first['t']-anchor if first else None
        s=trial['state'];dt=np.diff(s['t']);reverse=np.maximum(0.,-s['v'])
        row['reverse_m']=float(np.sum(dt*(reverse[:-1]+reverse[1:])/2))
        row['max_lateral_m']=float(abs(s['y']).max())
        row['collision']=r['collision_conservative_projection'];row['native_bundles']=r['trajectory_bundles']
        row['motion_deadline_miss']=sum(x['reason']=='deadline_miss' for x in trial['diag'] if x['t']>=anchor)
        row['peak_dynamic_tracks']=max((x['obstacle_tracks'] for x in trial['diag']),default=0)
        for name,key in (('reference_audit','reference_contract_pass'),('execution_audit','execution_contract_pass')):
            f=p/(name+'.json');row[key]=json.loads(f.read_text())[key] if f.exists() else None
        row['required_process_failed']=json.loads((p/'run_status.json').read_text())['required_process_failed']
        if row['required_process_failed']:row['result']='process_failed'
        records.append(row);first_text='—' if first is None else f'{row["first_dynamic_input_s"]:.2f}'
        kernel_text='—' if m['kernel_p95_ms'] is None else f'{m["kernel_p95_ms"]:.1f}'
        lines.append(f'| {spec["key"]} | {row["result"]} | {m["motion_to_end_s"]:.2f} | {m["min_clearance_m"]:.3f} | {first_text} | {kernel_text} | [全局]({spec["key"]}/visualization/overview.gif) | [搜索]({spec["key"]}/visualization/search_zoom.gif) |')
    lines+=['','## 主场景 V11 / V12 对照','',
        '橙色为实际 REMANI 参考，蓝色为已执行轨迹，车体显示同一固定臂投影。旧失败面板冻结在本次终止状态。','']
    for spec in specs:
        if spec['group']=='main':lines.append(f'- {CASES[spec["scene"]][1]}：[对照 GIF](comparison/{spec["key"]}/comparison.gif)')
    lines+=['','浅蓝/浅红：内核可行/被拒候选；紫色：内核选中；绿色：发布前复核通过的局部展开。',
        '灰箭头只显示感知CV的最多1.5s，MPPI完整展开长度由本帧搜索记录和limits.json确定，两者不能混淆。',
        '位姿误差、轮速、观测新鲜度和命令租约仍检查。制动后备检查只覆盖制动至停车稳定，不是停车后无限时间安全保证。',
        '仅改感知的试验使用V11控制内核与V12感知，以区分感知修复和局部控制修改的贡献。',
        '带搜索遥测的计算时延不是无干扰硬实时基准；实际执行仍有50ms截止约束。','']
    (out/'summary.json').write_text(json.dumps(records,ensure_ascii=False,indent=2)+'\n')
    (out/'README.md').write_text('\n'.join(lines)+'\n');return records


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--build',type=Path,default=HERE/'builds/native_v12_05')
    p.add_argument('--perception-build',type=Path,default=HERE/'builds/native_v12_perception_01')
    p.add_argument('--duration',type=float,default=52.)
    p.add_argument('--render-only',action='store_true')
    a=p.parse_args();out=a.out.resolve();specs=declared(a.build.resolve(),a.perception_build.resolve())
    if not a.render_only:
        out.mkdir(parents=True,exist_ok=False)
        manifest=dict(specs=specs,duration=a.duration,
            kernels={str(b.resolve()):hashlib.sha256((b/'build/libonline_mppi.so').read_bytes()).hexdigest() for b in (a.build,a.perception_build)},
            timing='serial ROS trials; heavy plotting after all trials; fixed MPPI seed1, native search is not seeded/paired')
        (out/'suite_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        for spec in specs:
            print('START',spec['key'],flush=True);dest=out/spec['key']
            args=SimpleNamespace(out=dest,build=Path(spec['build']),scene=spec['scene'],duration=a.duration,
                mode='mppi',perception='classified',execution='local',projection_progress=True,
                planner_variant='current',scenario_start='motion',debug_native=False,
                reference_delay=spec['delay'],record_search=True,prediction_horizon=spec['prediction'])
            try:run(args)
            except Exception as exc:
                dest.mkdir(parents=True,exist_ok=True)
                (dest/'suite_error.json').write_text(json.dumps(dict(error=str(exc)))+'\n')
                print('FAILED',spec['key'],str(exc),flush=True)
            summary(out,specs)
    else:specs=json.loads((out/'suite_manifest.json').read_text())['specs']
    for spec in specs:
        dest=out/spec['key']
        if not (dest/'result.json').exists():continue
        reference_audit(dest);execution_audit(dest)
        if not (dest/'visualization').exists():render(dest,dest/'visualization')
    for spec in specs:
        if spec['group']!='main':continue
        case=CASES[spec['scene']][0]
        base=HERE/'results/native_v11'/('multi_dynamic_capacity_02' if len(CASES[spec['scene']][2])==4 else 'multi_dynamic_01')/case
        target=out/'comparison'/spec['key']
        if (out/spec['key']/'result.json').exists() and not target.exists():
            compare(base,out/spec['key'],target,'V11（2s）','V12（3s）')
    rows=summary(out,specs);print('COMPLETE',out,flush=True)
    print(json.dumps([(r['key'],r['result']) for r in rows],ensure_ascii=False),flush=True)


if __name__=='__main__':main()
