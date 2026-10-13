#!/usr/bin/env python3
"""Render native reference, measured body projection and recorded search only.

No re-running a controller to invent missing historical candidates. New output
directory required; original records and old GIFs are never overwritten.
"""
import argparse
import hashlib
import json
import shutil
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from compare_native_v9 import load
from native_visuals import geometry, footprint, SearchReplay, draw_search

FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'


def render(source, out):
    a=load(source);body,meta=geometry(source);search=SearchReplay(source)
    execution=[json.loads(s) for s in (source/'execution_rollouts.jsonl').read_text().splitlines()]
    etimes=[v['t'] for v in execution]
    r=a['result'];anchor=r['motion_anchor'] or r['goal_sent'] or 0
    # Record exactly which bytes the visualization is derived from.
    inputs=[source/n for n in ('result.json','state.csv','obstacles.csv','geometry.csv','geometry.json',
                               'consumed_references.jsonl','reference_events.jsonl','execution_rollouts.jsonl')]
    inputs+=sorted((source/'search').glob('*.npz'))
    hashes={str(p.relative_to(source)):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    out.mkdir(parents=True,exist_ok=False)
    title=ImageFont.truetype(FONT,20);small=ImageFont.truetype(FONT,16)
    frames=[];search_frames=0;best=None;best_count=-1
    def frame(t, zoom):
        nonlocal search_frames,best,best_count
        terminal=t>=r['elapsed'];s=a['state'];j=max(0,np.searchsorted(s['t'],t,side='right')-1);state=s[j]
        im=Image.new('RGB',(850,650),'white');d=ImageDraw.Draw(im)
        d.text((22,12),f'REMANI 在线参考 + MPPI | {r["scene"]} | t={t-anchor:.2f}s',font=title,fill='#333333')
        d.text((22,46),'橙：REMANI  蓝：实际轨迹  灰蓝外形：车体投影  绿：获准局部轨迹',font=small,fill='#333333')
        d.text((22,71),'搜索：浅蓝可行 / 浅红被拒 / 紫色内核选中（不等于获准执行）',font=small,fill='#555555')
        scale=190 if zoom else 110
        def xy(x,y):return (420+(x-state['x'])*scale,345-(y-state['y'])*scale) if zoom else (195+x*scale,345-y*scale)
        layer=Image.new('RGB',im.size,'white');g=ImageDraw.Draw(layer)
        for y in (-2.,2.):g.line([xy(-1.5,y),xy(4.5,y)],fill='#e5e5e5')
        for x in (-1.5,4.5):g.line([xy(x,-2),xy(x,2)],fill='#e5e5e5')
        o=a['obs'];oi=np.searchsorted(o['t'],t,side='right')-1 if len(o) else -1
        if oi>=0 and t-o['t'][oi]<.1:
            stamp=o['t'][oi]
            for ob in o[o['t']==stamp]:
                x,y=xy(ob['x'],ob['y']);rad=ob['r']*scale;g.ellipse((x-rad,y-rad,x+rad,y+rad),fill='#aaaaaa')
        refs=[v for v in a['refs'] if v['t']<=t];events=[v for v in a['events'] if v['t']<=t]
        if refs and events and events[-1]['event'] not in ('cancel','invalid') and refs[-1]['revision']==events[-1]['revision']:
            g.line([xy(*v[:2]) for v in refs[-1]['reference']],fill='#d78224',width=3)
        if j:g.line([xy(x,y) for x,y in zip(s['x'][:j+1],s['y'][:j+1])],fill='#2874a6',width=3)
        footprint(g,xy,scale,body,(state['x'],state['y'],state['yaw']),meta)
        snap=search.at(t,terminal=terminal);feasible,rejected=draw_search(g,xy,snap)
        ei=np.searchsorted(etimes,t,side='right')-1
        ex=execution[ei] if ei>=0 and t-etimes[ei]<.15 and not terminal else None
        if ex and ex['reason']=='ok' and len(ex['rollout'])>1:
            points=np.asarray(ex['rollout'])
            # Sparse full-body projection along the approved rollout. The
            # line represents its center, not a point-robot collision model.
            for k in sorted(set([len(points)//3,2*len(points)//3,len(points)-1])):
                footprint(g,xy,scale,body,points[k,1:4],ghost=True)
            g.line([xy(x,y) for x,y in points[:,1:3]],fill='#248f45',width=3)
        im.paste(layer.crop((18,105,832,550)),(18,105));d=ImageDraw.Draw(im)
        d.rectangle((18,105,832,550),outline='#dddddd')
        if snap is not None:
            note=f'已记录候选 {feasible+rejected} 条（可行 {feasible}）；记录年龄 {t-float(snap["t"]):.2f}s'
            search_frames+=1
        else:note='结束，不保留旧预测' if terminal else '此帧无新鲜搜索记录，不补画'
        d.text((22,564),note,font=small,fill='#333333')
        d.text((22,591),'投影与碰撞模型一致；另有固定安全余量，未把余量画成实体',font=small,fill='#555555')
        d.text((22,616),'隔离数值仿真 / 固定机械臂 / '+('局部放大' if zoom else '全局视图')+f' / 结果：{r["reason"]}',font=small,fill='#555555')
        if zoom and feasible+rejected>best_count:best_count=feasible+rejected;best=im.copy()
        return im
    times=list(np.arange(anchor,r['elapsed'],.2))+[r['elapsed']]
    for zoom,name in ((False,'overview'),(True,'search_zoom')):
        frames=[frame(t,zoom).convert('P',palette=Image.ADAPTIVE,colors=128) for t in times]
        frames[0].save(out/(name+'.gif'),save_all=True,append_images=frames[1:],duration=200,loop=0,disposal=2)
    best.save(out/'search_snapshot.png')
    for p in inputs:assert hashlib.sha256(p.read_bytes()).hexdigest()==hashes[str(p.relative_to(source))]
    for name in ('render_native_search.py','native_visuals.py','compare_native_v9.py'):
        shutil.copy2(Path(__file__).parent/name,out/name)
    report=dict(source=str(source.resolve()),source_sha256=hashes,search_snapshots=len(search.entries),
                rendered_search_frames=search_frames,visualization_only=True,source_unchanged=True,
                no_missing_search_fabricated=True,body_disks=body.tolist(),result=r['reason'])
    (out/'visualization_audit.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='source_sha256'},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);args=p.parse_args();render(args.source,args.out)
