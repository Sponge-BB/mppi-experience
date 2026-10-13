#!/usr/bin/env python3
"""V7/V8 regression and holdout comparisons, fixed-reference local SIL only."""
import argparse
import hashlib
import json
import shutil
from pathlib import Path
import numpy as np
from PIL import Image
import suite_multi_dynamic as multi

HERE=Path(__file__).resolve().parent
EXTRA={'05_head_on_only':[multi.D],
       '06_four_mirrored':[[a[0],-a[1],a[2],-a[3],a[4],a[5]] for a in multi.CASES['04_four_dense']],
       '07_four_late':[multi.A,multi.B,multi.C,multi.D[:]]}
EXTRA['07_four_late'][-1][0]+=.3
multi.CASES.update(EXTRA)
multi.NAMES.update({'05_head_on_only':'单独迎面交会','06_four_mirrored':'四障碍镜像交会','07_four_late':'四障碍延后交会'})


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def paired_gif(path,old,new):
    frames=[];duration=max(len(old['trace']),len(new['trace']))
    ids=list(range(0,duration,4))
    if ids[-1]!=duration-1:ids.append(duration-1)
    for k in ids:
        # Freeze the failed/finished panel at its own terminal state, not imply
        # a collision is subsequently recovered by that version.
        left=multi.frame(old,min(k,len(old['trace'])-1),search=False)
        right=multi.frame(new,min(k,len(new['trace'])-1),search=False)
        canvas=Image.new('RGB',(1600,640),'white');canvas.paste(left,(0,0));canvas.paste(right,(800,0));frames.append(canvas)
    frames[0].save(path,save_all=True,append_images=frames[1:],duration=100,loop=0,disposal=2)


def render(out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    summary={};lines=['# V8 候选覆盖与恢复实验','',
      '固定参考、理想CV与点归属、固定机械臂几何；不是完整REMANI、Gazebo或实车。',
      'V7/V8使用相同场景、车体、0.1m余量、执行模型及2秒预测有效期。GIF固定seed 1，不挑最好结果。',
      '前四场景V7使用此前冻结原始记录，新增场景重新运行V7；耗时不作严格跨批性能对比。','',
      '| 场景 | 版本 | 到达 | 碰撞 | 平均终止时间/s | 平均路径/m | 平均低速/s | 最小净距/m | 内核P95均值/ms |',
      '|---|---|---:|---:|---:|---:|---:|---:|---:|']
    for case in sorted(out.iterdir()):
        if not (case/'data').exists():continue
        records={v:[json.loads(f.read_text()) for f in sorted((case/'data').glob(v+'_seed*.json'))] for v in ('v7','v8')}
        summary[case.name]={}
        for v in ('v7','v8'):
            rows=[r['metrics'] for r in records[v]]
            mean=lambda key:float(np.mean([r[key] for r in rows]))
            m={'n':len(rows),'reached':sum(r['reached'] for r in rows),'collisions':sum(r['termination']=='collision' for r in rows),
               'mean_duration_s':mean('duration_s'),'mean_path_m':mean('path_m'),'mean_low_speed_s':mean('low_speed_s'),
               'mean_reverse_m':mean('reverse_m'),'min_gap_m':min(r['min_gap_m'] for r in rows),
               'mean_kernel_p95_ms':mean('kernel_p95_ms'),'max_kernel_ms':max(r['kernel_max_ms'] for r in rows),
               'kernel_over_50ms':sum(r['kernel_over_50ms'] for r in rows),
               'margin_violation_cycles':sum(r['margin_violation_cycles'] for r in rows),
               'unverified_brake_cycles':sum(r['unverified_brake_cycles'] for r in rows)}
            summary[case.name][v]=m
            lines.append(f'| {multi.NAMES[case.name]} | {v.upper()} | {m["reached"]}/{m["n"]} | {m["collisions"]} | {m["mean_duration_s"]:.2f} | {m["mean_path_m"]:.2f} | {m["mean_low_speed_s"]:.2f} | {m["min_gap_m"]:.4f} | {m["mean_kernel_p95_ms"]:.2f} |')
        multi.render_case(case,records['v8'])
        old,new=records['v7'][0],records['v8'][0]
        paired_gif(case/'gif/comparison.gif',old,new)
        fig,axs=plt.subplots(1,3,figsize=(11,3.4))
        for r,color in ((old,'#888888'),(new,'#287ca0')):
            t=np.array([x['t'] for x in r['trace']]);s=np.array([x['state'] for x in r['trace']]);lab=r.get('version','V7')
            axs[0].plot(s[:,0],s[:,1],color=color,label=lab)
            axs[1].plot(t,s[:,3],color=color,label=lab)
            axs[2].plot(t,[x['gap'] for x in r['trace']],color=color,label=lab)
        axs[0].plot([0,3],[0,0],'--',color='.8');axs[0].set(title='Executed center path',xlabel='x / m',ylabel='y / m',aspect='equal')
        axs[1].set(title='Actual speed',xlabel='time / s',ylabel='m/s')
        axs[2].axhline(.1,ls='--',color='.7');axs[2].set(title='Body clearance',xlabel='time / s',ylabel='m')
        for ax in axs:ax.grid(alpha=.15);ax.legend()
        fig.suptitle('V7 / V8 | seed 1 | fixed-reference SIL');fig.tight_layout();fig.savefig(case/'plots/comparison.png',dpi=140);plt.close(fig)
    lines+=['','碰撞组的终止时间是碰撞时刻，不是成功通行时间。净距不含额外安全余量，负值表示几何碰撞。','',
            '## 动画与数据','']
    for case in summary:
        lines.append(f'- {multi.NAMES[case]}：[对照GIF]({case}/gif/comparison.gif) · [V8搜索]({case}/gif/search.gif) · [搜索放大]({case}/gif/search_zoom.gif) · [对照曲线]({case}/plots/comparison.png)')
    lines+=['','浅蓝/浅红是通过/未通过碰撞检查的外层候选，绿色是所选预测，蓝色是实际轨迹及当前车体。',
            '没有加入额外不确定性膨胀；没有解除原生安全门；没有在实车接口部署。',
            '后续动作检查只在原2秒CV有效期内，不代表递归可行或长期安全证明。',
            '内核计时不包含Python导出、真实感知及执行延迟；离线绘图不在控制循环内。','']
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    (out/'README.md').write_text('\n'.join(lines)+'\n');(out/'结果汇总.txt').write_text('\n'.join(lines)+'\n')
    print(json.dumps(summary,indent=2),flush=True)


def main():
    p=argparse.ArgumentParser();p.add_argument('--new',type=Path);p.add_argument('--old',type=Path,default=HERE/'builds/mppi_v7_search_01')
    p.add_argument('--out',type=Path,required=True);p.add_argument('--seeds',type=int,nargs='+',default=[1,2,3]);p.add_argument('--cases',nargs='+')
    p.add_argument('--render-only',action='store_true');a=p.parse_args()
    if a.render_only:render(a.out);return
    a.out.mkdir(parents=True,exist_ok=False);(a.out/'source_snapshot').mkdir();(a.out/'fixture').mkdir()
    fixture=HERE/'results/final_report/empty_mppi';baseline=HERE/'results/mppi_v7/multi_dynamic_03'
    baseline_manifest=json.loads((baseline/'manifest.json').read_text())
    for name in ('mppi_v7.hpp','mppi_v8.hpp','build_mppi_v2.py','suite_v8.py','suite_multi_dynamic.py','test_mppi_v2.py','test_mppi_v8.py'):
        shutil.copy2(HERE/name,a.out/'source_snapshot'/name)
    for name in ('geometry.csv','limits.json'):
        assert sha(fixture/name)==sha(baseline/'fixture'/name)
        shutil.copy2(fixture/name,a.out/'fixture'/name)
    cases={k:v for k,v in multi.CASES.items() if not a.cases or k in a.cases}
    manifest={'scope':'fixed-reference ideal local SIL, not ROS/Gazebo/real','cases':cases,'seeds':a.seeds,
              'builds':{v:{'path':str(b.resolve()),'sha256':sha(b/'build/libonline_mppi.so'),'manifest':json.loads((b/'build_manifest.json').read_text())} for v,b in (('v7',a.old),('v8',a.new))},
              'source_snapshot_sha256':{f.name:sha(f) for f in (a.out/'source_snapshot').iterdir()},'baseline_source_files':{}}
    assert manifest['builds']['v7']['sha256']==baseline_manifest['library_sha256']
    for case,actors in cases.items():
        assert all(v['gap']>=0 for v in multi.actor_pair_gaps(actors))
        dest=a.out/case
        for sub in ('data','gif','plots'):(dest/sub).mkdir(parents=True)
        for seed in a.seeds:
            cached=baseline/case/'data'/f'seed{seed}.json'
            if cached.exists():
                assert actors==baseline_manifest['cases'][case]
                shutil.copy2(cached,dest/'data'/f'v7_seed{seed}.json');manifest['baseline_source_files'][str(cached)]=sha(cached)
                r=json.loads(cached.read_text())
            else:
                r=multi.trial(a.old,fixture,case,seed,capture=False)
                (dest/'data'/f'v7_seed{seed}.json').write_text(json.dumps(r,separators=(',',':'))+'\n')
            print(case,'V7',seed,r['metrics'],flush=True)
            r=multi.trial(a.new,fixture,case,seed,capture=True)
            (dest/'data'/f'v8_seed{seed}.json').write_text(json.dumps(r,separators=(',',':'))+'\n')
            print(case,'V8',seed,r['metrics'],flush=True)
        (a.out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    native=manifest['builds']['v8']['manifest']['production_sources_sha256']
    assert all(sha(HERE.parents[1]/name)==digest for name,digest in native.items())
    render(a.out)
    gifs=list(a.out.glob('*/gif/*.gif'))
    for gif in gifs:
        with Image.open(gif) as im:assert im.n_frames>1;im.seek(im.n_frames-1);im.load()
    (a.out/'audit.json').write_text(json.dumps({'production_files_unchanged':len(native),'gifs_readable':len(gifs),'source_and_baseline_hashes_saved':True},indent=2)+'\n')

if __name__=='__main__':main()
