#!/usr/bin/env python3
"""Two recorded trial panels, measured-state metrics, and no synthetic paths."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from compare_native_v9 import load
from native_visuals import geometry,footprint

FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
SOURCE_NAMES=('result.json','run_status.json','state.csv','obstacles.csv','geometry.csv',
              'geometry.json','mppi_diagnostics.json','consumed_references.jsonl',
              'reference_events.jsonl','manifest.json')


def measured_status(trial,state,t):
    """Native-only has no MPPI diagnostics: report motion, not 'waiting'."""
    if trial['manifest'].get('mode')=='remani':
        v,w=float(state['v']),float(state['w'])
        mode='静止' if abs(v)<.02 and abs(w)<.06 else '转向' if abs(v)<.02 else '行进'
        return f'原生{mode}  v={v:.2f}  w={w:.2f}  参考{int(state["trajectory_count"])}'
    diagnostics=[item for item in trial['diag'] if item['t']<=t]
    if not diagnostics or t-diagnostics[-1]['t']>.2:return '无新鲜局部诊断'
    return diagnostics[-1]['reason']


def source_hashes(path):
    return {name:hashlib.sha256((path/name).read_bytes()).hexdigest()
            for name in SOURCE_NAMES if (path/name).is_file()}


def compare(old,new,out,old_label='REMANI（当前工程，无MPPI）',new_label='V12'):
    # Lazy import keeps the report's comparator dependency acyclic.
    from report_shared_collision import active_reference,measured_anchor,trial_metrics
    trials=[load(Path(old)),load(Path(new))]
    metrics=[]
    for trial in trials:
        trial['geometry']=geometry(trial['path'])
        trial['manifest']=json.loads((trial['path']/'manifest.json').read_text())
        metrics.append(trial_metrics(trial['path'],write=False))
    originals={str(t['path']):source_hashes(t['path']) for t in trials}
    out=Path(out);out.mkdir(parents=True,exist_ok=False)
    font=ImageFont.truetype(FONT,18);small=ImageFont.truetype(FONT,15)
    duration=max(max(0.,t['result']['elapsed']-measured_anchor(t['result'])) for t in trials)
    frames=[]
    for elapsed in list(np.arange(0.,duration,.2))+[duration]:
        im=Image.new('RGB',(1100,505),'white');draw=ImageDraw.Draw(im)
        draw.text((24,12),'橙：在线 REMANI 参考   蓝：实际运动及车体投影   灰：障碍物',font=font,fill='#333333')
        draw.text((955,15),f't={elapsed:.1f}s',font=small,fill='#333333')
        for panel,(trial,label) in enumerate(zip(trials,(old_label,new_label))):
            result=trial['result'];t=min(measured_anchor(result)+elapsed,result['elapsed'])
            states=trial['state'];i=max(0,np.searchsorted(states['t'],t,side='right')-1);state=states[i]
            def xy(x,y):return (panel*550+35+(x+1.5)*80,95+(2-y)*80)
            draw.text((panel*550+90,57),label,font=font,fill='#333333')
            draw.rectangle((*xy(-1.5,2),*xy(4.5,-2)),outline='#dddddd',width=2)
            reference=active_reference(trial['refs'],trial['events'],t)
            if reference is not None and len(reference['reference'])>1:
                draw.line([xy(*row[:2]) for row in reference['reference']],fill='#d78224',width=2)
            if i:draw.line([xy(x,y) for x,y in zip(states['x'][:i+1],states['y'][:i+1])],fill='#2874a6',width=3)
            footprint(draw,xy,80,trial['geometry'][0],(state['x'],state['y'],state['yaw']),trial['geometry'][1])
            obstacles=trial['obs'];oi=np.searchsorted(obstacles['t'],t,side='right')-1 if len(obstacles) else -1
            if oi>=0 and 0.<=t-obstacles['t'][oi]<.1:
                for obstacle in obstacles[obstacles['t']==obstacles['t'][oi]]:
                    x,y=xy(obstacle['x'],obstacle['y']);radius=obstacle['r']*80
                    draw.ellipse((x-radius,y-radius,x+radius,y+radius),fill='#aaaaaa')
            status='结束：'+metrics[panel]['result'] if t>=result['elapsed'] else measured_status(trial,state,t)
            draw.text((panel*550+35,439),status,font=small,fill='#333333')
            scope='发布参考回放，非控制器接收确认' if trial['manifest'].get('mode')=='remani' else '实际接收的完整参考事务'
            draw.text((panel*550+35,467),scope,font=small,fill='#666666')
        frames.append(im.convert('P',palette=Image.ADAPTIVE,colors=128))
    frames[0].save(out/'comparison.gif',save_all=True,append_images=frames[1:],duration=200,loop=0,disposal=2)
    frames[-1].save(out/'paths.png')
    for trial in trials:
        if source_hashes(trial['path'])!=originals[str(trial['path'])]:
            raise RuntimeError('recorded source changed during comparison rendering: '+str(trial['path']))
    report=dict(old=metrics[0],new=metrics[1],labels=[old_label,new_label],
        alignment='measured motion anchor; fallback to goal only if no measured motion, explicitly marked in metrics',
        metrics_source='report_shared_collision.trial_metrics; measured state stop definition shared by all variants',
        baseline_scope='current工程 native REMANI/controller/Tracker, MPPI disabled; NOT unmodified upstream REMANI-Planner',
        attribution='V12 shared combines common collision model and reference reacquisition changes; no collision-cost-only causality claim',
        sources=originals,source_unchanged=True,visualization_only=True)
    (out/'comparison.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print('COMPARISON',out,flush=True)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--old',type=Path,required=True);p.add_argument('--new',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--old-label',default='REMANI（当前工程，无MPPI）');p.add_argument('--new-label',default='V12')
    a=p.parse_args();compare(a.old,a.new,a.out,a.old_label,a.new_label)
