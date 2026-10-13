#!/usr/bin/env python3
"""Fair V6/V7 persistent-obstacle comparison; both get ideal classified input."""
import argparse
import hashlib
import json
import shutil
from pathlib import Path
import numpy as np
from test_local_independence import trial
from suite_v6 import metrics,plots,gif

HERE=Path(__file__).resolve().parent
CASES=[('01_crossing_persistent','crossing',{}),('02_static','static',{}),
       ('03_mirrored','crossing',{'mirror':True}),('04_fast','crossing',{'crossing_speed':.38}),
       ('05_late_encounter','crossing',{'crossing_offset':-.35}),
       ('06_nonconflicting','crossing',{'crossing_offset':2.}),('07_empty','empty',{})]

def progress_plot(dest,records):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axs=plt.subplots(2,1,figsize=(8,5),sharex=True)
    for r,label,color in zip(records,('V6','V7'),('#888888','#287ca0')):
        a=np.array([row[:9] for row in r['trace']])
        axs[0].plot(a[:,0],a[:,4]*np.cos(a[:,3]),label=label,color=color)
        axs[1].plot(a[:,0],a[:,2],label=label,color=color)
    axs[0].axhline(.03,color='.7',ls='--',lw=1);axs[0].set(ylabel='Progress / m/s',title='Progress toward goal (not just body speed)')
    axs[1].axhline(0,color='.7',lw=1);axs[1].set(ylabel='Lateral offset / m',xlabel='Time / s')
    for ax in axs:ax.grid(alpha=.15);ax.legend()
    fig.tight_layout();fig.savefig(dest/'progress.png',dpi=140);plt.close(fig)

def extended_metrics(r):
    m=metrics(r);x=np.array([row[:9] for row in r['trace']]);live=(x[:,0]>1)&(x[:,1]<2.85)
    progress=x[:,4]*np.cos(x[:,3]) # This suite's reference is the +x straight line.
    shifted=np.flatnonzero(abs(x[:,2])>.05)
    peak=int(np.argmax(abs(x[:,2])));rejoined=np.flatnonzero((np.arange(len(x))>peak)&(abs(x[:,2])<.04)&(abs(x[:,3])<.15))
    m.update(first_lateral_5cm_s=float(x[shifted[0],0]) if len(shifted) else None,
             max_lateral_m=float(abs(x[:,2]).max()),
             low_path_progress_s=float(np.sum(live&(progress<.03))*.05),
             moving_without_progress_s=float(np.sum(live&(x[:,4]>.06)&(abs(progress)<.03))*.05),
             rejoin_s=float(x[rejoined[0],0]) if len(shifted) and len(rejoined) else None)
    return m

def render(out):
    summaries={};lines=['# V7 实验导航','',
      'V6/V7 使用相同的理想分层输入、固定参考、车体与约束。障碍物不在14秒时移除。',
      '不是完整 REMANI、Gazebo 或实车；灰箭头来自理想 CV，非实测预测准确率。',
      '每场景 data 为全部种子；gif 为种子1（或最小种子），不挑最好一次。','']
    txt=['V6 / V7 同输入对照。理想分层、固定参考、持续障碍物；非实车。','']
    for case in sorted(out.iterdir()):
        if not (case/'data').is_dir():continue
        rows=[];first=[]
        for label in ('v6','v7'):
            records=[json.loads(f.read_text()) for f in sorted((case/'data').glob(label+'_*.json'))]
            if not records:continue
            first.append(min(records,key=lambda r:r['seed']))
            ms=[{'version':label,'seed':r['seed'],**extended_metrics(r)} for r in records];rows+=ms
            txt.append(f"{case.name} {label}: 到达 {sum(m['reached'] for m in ms)}/{len(ms)}, "
              f"平均通行 {np.mean([m['duration_s'] for m in ms]):.2f}s, 路径 {np.mean([m['path_length_m'] for m in ms]):.2f}m, "
              f"低速 {np.mean([m['low_speed_away_from_goal_s'] for m in ms]):.2f}s, "
              f"低推进 {np.mean([m['low_path_progress_s'] for m in ms]):.2f}s, "
              f"三次最小净距 {min(m['min_clearance_m'] for m in ms):.3f}m, "
              f"未验证制动 {sum(m['unverified_brake_cycles'] for m in ms)} 周期")
        summaries[case.name]=rows;(case/'summary.json').write_text(json.dumps(rows,indent=2)+'\n')
        if len(first)==2:
            plots(case/'plots',first,labels=('V6','V7'));gif(case/'gif'/'comparison.gif',first,['旧版 V6','新版 V7'])
            progress_plot(case/'plots',first)
            for r,label in zip(first,('v6','v7')):gif(case/'gif'/f'{label}.gif',[r],[label.upper()])
        lines.append(f'- {case.name}: [并排 GIF]({case.name}/gif/comparison.gif) · [V6]({case.name}/gif/v6.gif) · [V7]({case.name}/gif/v7.gif) · [曲线]({case.name}/plots/comparison.png) · [有效推进]({case.name}/plots/progress.png)')
    (out/'summary.json').write_text(json.dumps(summaries,indent=2)+'\n')
    (out/'README.md').write_text('\n'.join(lines)+'\n');(out/'结果汇总.txt').write_text('\n'.join(txt)+'\n');print('\n'.join(txt),flush=True)

def main():
    p=argparse.ArgumentParser();p.add_argument('--old',type=Path);p.add_argument('--new',type=Path)
    p.add_argument('--fixture',type=Path);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--seeds',type=int,nargs='+',default=[1,2,3]);p.add_argument('--cases',nargs='+');p.add_argument('--render-only',action='store_true');a=p.parse_args()
    if a.render_only:render(a.out);return
    assert a.old and a.new and a.fixture
    a.out.mkdir(parents=True,exist_ok=False);(a.out/'source_snapshot').mkdir();(a.out/'fixture').mkdir()
    for name in ('mppi_v7.hpp','build_mppi_v2.py','test_local_independence.py','suite_v7.py','suite_v6.py','test_mppi_v2.py','test_mppi_v7.py','mppi_node.py'):
        shutil.copy2(HERE/name,a.out/'source_snapshot'/name)
    for name in ('geometry.csv','limits.json'):shutil.copy2(a.fixture/name,a.out/'fixture'/name)
    manifest={'scope':'local fixed-reference ideal classified SIL; persistent obstacle in BOTH versions','builds':{},'seeds':a.seeds}
    for label,build in (('v6',a.old),('v7',a.new)):
        manifest['builds'][label]={'path':str(build.resolve()),'manifest':json.loads((build/'build_manifest.json').read_text()),'sha256':hashlib.sha256((build/'build/libonline_mppi.so').read_bytes()).hexdigest()}
    native=manifest['builds']['v7']['manifest']['production_sources_sha256']
    assert all(hashlib.sha256((HERE.parents[1]/name).read_bytes()).hexdigest()==digest for name,digest in native.items())
    manifest['production_files_unchanged']=len(native)
    manifest['source_snapshot_sha256']={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in (a.out/'source_snapshot').iterdir()}
    (a.out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    for name,scene,kwargs in CASES:
        if a.cases and name not in a.cases:continue
        dest=a.out/name
        for sub in ('data','gif','plots'):(dest/sub).mkdir(parents=True)
        for seed in a.seeds:
            for label,build in (('v6',a.old),('v7',a.new)):
                r=trial(build,a.fixture,scene,seed=seed,separate_dynamic=True,persistent_obstacle=True,**kwargs)
                (dest/'data'/f'{label}_seed{seed}.json').write_text(json.dumps(r,indent=2)+'\n')
                print(name,label,seed,extended_metrics(r),flush=True)
    render(a.out)

if __name__=='__main__':main()
