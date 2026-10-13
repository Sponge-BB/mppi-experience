#!/usr/bin/env python3
"""V8-style multi-actor suite through actual native ROS perception/planning."""
import argparse
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
from dynamic_scenarios import CASES,scene_definition
from run import run
from compare_native_v9 import load,metrics
from render_native_search import render
from audit_native_reference import audit as reference_audit
from audit_execution import audit as execution_audit

HERE=Path(__file__).resolve().parent


def report(out, scenes):
    rows=[];lines=['# REMANI + MPPI 多动态障碍物实验','',
        '固定机械臂，原生 REMANI 在线规划/重规划、点云感知及底盘 MPPI，隔离 ROS 数值仿真。',
        '沿用 V8 的障碍物运动设置，参考来自当前原生规划器；另加中心线迎面场景。',
        '全部场景各运行一次；保留碰撞、超时和进程故障。带搜索记录的运行不作严格实时性能基准。','',
        '| 场景 | 障碍物数 | 结果 | 用时/s | 最小投影净距/m | 预测目标峰值 | 全局 GIF | 搜索放大 |',
        '| --- | ---: | --- | ---: | ---: | ---: | --- | --- |']
    for scene in scenes:
        case,name,actors=CASES[scene];p=out/case
        if not (p/'result.json').exists():
            rows.append(dict(scene=scene,result='incomplete'));continue
        a=load(p);m=metrics(a);r=a['result']
        peak=max((d['obstacle_tracks'] for d in a['diag']),default=0)
        row=dict(scene=scene,case=case,actors=len(actors),**m,collision=r['collision_conservative_projection'],
                 peak_predicted_tracks=peak,reference_bundles=r['trajectory_bundles'])
        for audit_name,key in (('reference_audit','reference_contract_pass'),('execution_audit','execution_contract_pass')):
            f=p/(audit_name+'.json');row[key]=json.loads(f.read_text())[key] if f.exists() else False
        status_file=p/'run_status.json'
        row['required_process_failed']=json.loads(status_file.read_text())['required_process_failed'] if status_file.exists() else True
        if row['required_process_failed']:row['result']='process_failed'
        rows.append(row)
        links=(f'[全局]({case}/visualization/overview.gif)',f'[搜索]({case}/visualization/search_zoom.gif)')
        lines.append(f'| {name} | {len(actors)} | {row["result"]} | {m["motion_to_end_s"]:.2f} | {r["min_clearance"]:.3f} | {peak} | {links[0]} | {links[1]} |')
    lines+=['','图例：橙色 REMANI 参考、蓝色已执行轨迹及车体投影、绿色获准局部展开。',
        '浅蓝/浅红为内核可行/被拒候选，紫色为内核选中。灰色圆柱带场景身份编号；灰箭头使用实际感知估计的 CV 速度，最长展开 1.5 s。',
        '搜索线表示中心展开；当前及获准展开使用底盘＋固定臂三圆盘外形进行碰撞检查。',
        '场景由 scenario.json 声明；obstacles.csv 存全部目标的真值和身份，预测输入来自点云检测，未注入真值速度。',
        'V8 的目标在运动触发后 2 s 出现并持续按恒速运动；部分从边界外进入或离开观测区域。',
        '四障碍场景仅在隔离实验配置中把健康检查的运动目标数量上限从 3 调到 4，其他健康/稳定性条件仍保留。',
        '新增场景按至多 5 ms 的底盘积分子步审核真值投影碰撞，发生碰撞立即终止。净距不包含额外安全余量。',
        '当前构建与速度、车体、安全余量、预测有效期未调优。结果用于暴露多目标交会问题，不宣称所有场景通过或实车可部署。','']
    (out/'summary.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2)+'\n')
    (out/'README.md').write_text('\n'.join(lines)+'\n')
    return rows


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--build',type=Path,default=HERE/'builds/native_v11_08')
    p.add_argument('--scenes',choices=list(CASES),nargs='+',default=list(CASES))
    p.add_argument('--duration',type=float,default=52.)
    p.add_argument('--render-only',action='store_true')
    args=p.parse_args();out=args.out.resolve()
    if not args.render_only:
        out.mkdir(parents=True,exist_ok=False)
        manifest=dict(build=str(args.build.resolve()),scenes=[scene_definition(s) for s in args.scenes],
                      duration=args.duration,scope='native online ROS numerical SIL, single trial per scene',
                      library_sha256=hashlib.sha256((args.build/'build/libonline_mppi.so').read_bytes()).hexdigest())
        (out/'suite_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
        for scene in args.scenes:
            dest=out/CASES[scene][0];print('START',scene,flush=True)
            config=SimpleNamespace(out=dest,build=args.build,mode='mppi',scene=scene,duration=args.duration,
                projection_progress=True,planner_variant='current',scenario_start='motion',debug_native=False,
                perception='classified',execution='local',reference_delay=0.,record_search=True)
            try:run(config)
            except Exception as exc:
                # Preserve failed trials and continue the declared suite.
                print('TRIAL FAILED',scene,str(exc),flush=True)
                dest.mkdir(parents=True,exist_ok=True)
                (dest/'suite_error.json').write_text(json.dumps(dict(error=str(exc)))+'\n')
            report(out,args.scenes)
    # Heavy drawing runs after ALL timing-sensitive ROS trials have ended.
    for scene in args.scenes:
        dest=out/CASES[scene][0]
        if not (dest/'result.json').exists():continue
        reference_audit(dest);execution_audit(dest)
        if not (dest/'visualization').exists():render(dest,dest/'visualization')
    rows=report(out,args.scenes)
    print('COMPLETE',out,flush=True)
    print(json.dumps([{k:r[k] for k in ('scene','result')} for r in rows],ensure_ascii=False),flush=True)


if __name__=='__main__':main()
