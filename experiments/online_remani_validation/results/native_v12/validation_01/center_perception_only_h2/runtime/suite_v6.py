#!/usr/bin/env python3
"""Organized V5/V6 local-only comparison. Never starts ROS or commands a robot."""
import argparse
import hashlib
import json
import shutil
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from test_local_independence import trial

HERE=Path(__file__).resolve().parent
CASES=[('01_dynamic_crossing','crossing',{}),
       ('02_static_obstacle','static',{}),
       ('03_dynamic_mirrored','crossing',{'mirror':True}),
       ('04_dynamic_faster','crossing',{'crossing_speed':.38}),
       ('05_dynamic_later_encounter','crossing',{'crossing_offset':-.35}),
       ('06_empty','empty',{})]

def metrics(r):
    a=np.array([row[:9] for row in r['trace']]);moving=(a[:,0]>1)&(np.linalg.norm(a[:,1:3]-[3,0],axis=1)>.15)
    return {'reached':r['reached'],'duration_s':r['duration_s'],
            'path_length_m':float(np.linalg.norm(np.diff(a[:,1:3],axis=0),axis=1).sum()),
            'min_clearance_m':r['min_clearance_m'],'reverse_distance_m':r['reverse_distance_m'],
            'low_speed_away_from_goal_s':float(np.sum(moving&(np.abs(a[:,4])<.02))*.05),
            'kernel_p95_ms':r['kernel_p95_ms'],'kernel_max_ms':max(r['kernel_ms']),
            'kernel_over_50ms':int(sum(t>50 for t in r['kernel_ms'])),
            'unverified_brake_cycles':sum(row[8]>0 for row in r['diagnostics'])}

def plots(dest,rs,labels=('V5','V6')):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(2,2,figsize=(10,7));colors=['#888888','#287ca0']
    for r,color,label in zip(rs,colors,labels):
        a=np.array([row[:9] for row in r['trace']]);ax[0,0].plot(a[:,1],a[:,2],color=color,label=label)
        ax[0,1].plot(a[:,0],a[:,4],color=color,label=label)
        ax[1,0].plot(a[:,0],a[:,8],color=color,label=label)
        ax[1,1].plot(a[:,0],r['kernel_ms'],color=color,label=label,alpha=.7)
    ax[0,0].plot([0,3],[0,0],'--',color='.8',lw=1);ax[0,0].set(title='Actual center path',xlabel='x / m',ylabel='y / m',aspect='equal')
    ax[0,1].set(title='Actual linear speed',xlabel='time / s',ylabel='m/s');ax[0,1].axhline(0,color='.8',lw=1)
    ax[1,0].set(title='Body-to-obstacle/wall gap (no added margin)',xlabel='time / s',ylabel='m');ax[1,0].axhline(.1,color='.8',ls='--',lw=1)
    ax[1,1].set(title='Kernel computation (not full callback)',xlabel='time / s',ylabel='ms');ax[1,1].axhline(50,color='.8',ls='--',lw=1)
    for a in ax.flat:a.legend();a.grid(alpha=.15)
    fig.suptitle('Fixed reference / ideal measurements / no native replanner',fontsize=12)
    fig.tight_layout(rect=(0,0,1,.94));fig.savefig(dest/'comparison.png',dpi=140);plt.close(fig)

def gif(dest,rs,labels):
    font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',16)
    small=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',13)
    frames=[];duration=max(r['duration_s'] for r in rs);scale=100
    for t in np.arange(0,duration+.2,.2):
        im=Image.new('RGB',(480*len(rs),460),'white');draw=ImageDraw.Draw(im)
        for panel,(r,label) in enumerate(zip(rs,labels)):
            def xy(x,y):return (panel*480+50+scale*(x+.4),225-scale*y)
            draw.text((panel*480+30,12),label+'  |  '+r['scene'],fill='#333333',font=font)
            draw.text((panel*480+30,37),'蓝：实际轨迹  绿：所选局部预测/引导',fill='#555555',font=small)
            draw.rectangle((panel*480+20,65,panel*480+460,400),outline='#dddddd')
            draw.line([xy(0,0),xy(3,0)],fill='#bbbbbb',width=1)
            rows=r['trace'];k=min(len(rows)-1,int(round(t/.05)));z=rows[k]
            guides=[g for g in r['routes'] if g['t']<=z[0]]
            if guides and len(guides[-1]['points'])>1:draw.line([xy(*p) for p in guides[-1]['points']],fill='#99b59f',width=1)
            if z[9] is not None:
                cx,cy=xy(z[9],z[10]);rad=.23*scale
                visible=cy-rad>=65 and cy+rad<=400
                if visible:draw.ellipse((cx-rad,cy-rad,cx+rad,cy+rad),fill='#bbbbbb')
                elif r.get('persistent_obstacle'):
                    draw.text((panel*480+30,378),'障碍物已出画面，仍持续被观测',fill='#777777',font=small)
                if visible and r['scene']=='crossing':
                    sign=-1 if r['mirror'] else 1
                    end=xy(z[9],z[10]+sign*r['crossing_speed']*1.5)
                    end=(end[0],max(70,min(395,end[1])))
                    draw.line([(cx,cy),end],fill='#777777',width=2)
                    draw.line([(end[0]-4,end[1]+sign*6),end,(end[0]+4,end[1]+sign*6)],fill='#777777',width=2)
            if k:draw.line([xy(row[1],row[2]) for row in rows[:k+1:2]],fill='#287ca0',width=2)
            # Draw the actual disk-union model: center icons previously hid size.
            for bx,by,rad in r['body']:
                px=z[1]+np.cos(z[3])*bx-np.sin(z[3])*by;py=z[2]+np.sin(z[3])*bx+np.cos(z[3])*by
                cx,cy=xy(px,py);rr=rad*scale;draw.ellipse((cx-rr,cy-rr,cx+rr,cy+rr),outline='#287ca0',width=1)
            cx,cy=xy(z[1],z[2]);draw.ellipse((cx-3,cy-3,cx+3,cy+3),fill='#287ca0')
            draw.line([(cx,cy),xy(z[1]+.25*np.cos(z[3]),z[2]+.25*np.sin(z[3]))],fill='#287ca0',width=2)
            finished=t>=r['duration_s']
            state=('到达' if r['reached'] else '超时') if finished else ('转向' if abs(z[4])<.02 and abs(z[5])>.05 else '停留' if abs(z[4])<.02 else '倒车' if z[4]<0 else '前进')
            draw.text((panel*480+30,408),f'{state}    t={z[0]:.1f}s    v={z[4]:.2f}m/s',fill='#333333',font=font)
        scope='理想点归属分层 · 固定参考 · 非实车 · 2 倍速' if any(r.get('separate_dynamic') for r in rs) else '固定参考数值仿真 · 非 Gazebo / 实车 · 2 倍速'
        draw.text((30,437),scope,fill='#666666',font=small)
        frames.append(im)
    frames[0].save(dest,save_all=True,append_images=frames[1:],duration=100,loop=0,optimize=False,disposal=2)
    with Image.open(dest) as im:
        assert im.n_frames==len(frames)

def main():
    p=argparse.ArgumentParser();p.add_argument('--old',type=Path,required=True);p.add_argument('--new',type=Path,required=True)
    p.add_argument('--fixture',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--seeds',type=int,nargs='+',default=[1,2,3]);p.add_argument('--cases',nargs='+');p.add_argument('--separate-dynamic',action='store_true')
    args=p.parse_args();args.out.mkdir(parents=True,exist_ok=False);(args.out/'source_snapshot').mkdir()
    for name in ('mppi_v6.hpp','build_mppi_v2.py','test_local_independence.py','test_mppi_v2.py','test_mppi_v6.py','suite_v6.py'):
        shutil.copy2(HERE/name,args.out/'source_snapshot'/name)
    manifest={'scope':'local fixed-reference SIL, not native REMANI/Gazebo/real robot','builds':{},'seeds':args.seeds,'v6_ideal_measurement_separation':args.separate_dynamic}
    for tag,b in (('v5',args.old),('v6',args.new)):
        manifest['builds'][tag]={'path':str(b.resolve()),'manifest':json.loads((b/'build_manifest.json').read_text()),'sha256':hashlib.sha256((b/'build/libonline_mppi.so').read_bytes()).hexdigest()}
    (args.out/'fixture').mkdir()
    for name in ('geometry.csv','limits.json'):shutil.copy2(args.fixture/name,args.out/'fixture'/name)
    manifest['source_snapshot_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (args.out/'source_snapshot').iterdir()}
    native=manifest['builds']['v6']['manifest']['production_sources_sha256'];workspace=HERE.parents[1]
    changed=[name for name,digest in native.items() if hashlib.sha256((workspace/name).read_bytes()).hexdigest()!=digest]
    assert not changed,changed
    manifest['production_files_unchanged']=len(native)
    (args.out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    summaries={}
    for name,scene,kwargs in CASES:
        if args.cases and name not in args.cases:continue
        dest=args.out/name
        for sub in ('data','gif','plots'): (dest/sub).mkdir(parents=True)
        summary=[];first=[]
        for seed in args.seeds:
            for label,b in (('v5',args.old),('v6',args.new)):
                r=trial(b,args.fixture,scene,seed=seed,separate_dynamic=args.separate_dynamic and label=='v6',**kwargs)
                (dest/'data'/f'{label}_seed{seed}.json').write_text(json.dumps(r,indent=2)+'\n')
                m={'version':label,'seed':seed,**metrics(r)};summary.append(m);print(name,m,flush=True)
                if seed==args.seeds[0]:first.append(r)
        (dest/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');summaries[name]=summary
        plots(dest/'plots',first)
        gif(dest/'gif'/'comparison.gif',first,['旧版 V5','新版 V6'])
        for r,label in zip(first,('v5','v6')):gif(dest/'gif'/f'{label}.gif',[r],[label.upper()])
        (dest/'README.txt').write_text('固定旧参考的局部控制实验，不运行 REMANI 重规划。\n'
          f'场景参数: {kwargs}; GIF/曲线对应种子 {args.seeds[0]}，不是挑选最好结果。\n'
          f'V6 理想动态点归属分层: {args.separate_dynamic}。此归属能力未经实车感知验证；V5 保持原有联合距离场。\n'
          'gif/comparison.gif：两版本并排；gif/v5.gif、v6.gif：各自独立。\n'
          'data/：所有种子原始逐周期记录；summary.json：全部指标。\n'
          '低速指标排除起步前 1 秒和距目标 0.15m 内；车体净距不含额外安全余量。\n')
    (args.out/'summary.json').write_text(json.dumps(summaries,indent=2)+'\n')

if __name__=='__main__':main()
