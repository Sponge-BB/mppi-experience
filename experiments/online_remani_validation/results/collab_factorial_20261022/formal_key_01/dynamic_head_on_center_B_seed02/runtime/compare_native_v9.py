#!/usr/bin/env python3
"""Recorded native-reference A/B comparison; never synthesize controller paths."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from native_visuals import geometry, footprint


def load(p):
    r=json.loads((p/'result.json').read_text())
    return dict(path=p,result=r,state=np.atleast_1d(np.genfromtxt(p/'state.csv',delimiter=',',names=True)),
        obs=np.atleast_1d(np.genfromtxt(p/'obstacles.csv',delimiter=',',names=True)),
        refs=[json.loads(x) for x in (p/'consumed_references.jsonl').read_text().splitlines()],
        events=[json.loads(x) for x in (p/'reference_events.jsonl').read_text().splitlines()],
        diag=json.loads((p/'mppi_diagnostics.json').read_text()))


def metrics(a):
    s=a['state'];r=a['result'];anchor=r['motion_anchor'] or r['goal_sent'] or 0
    # Actual execution includes time spent stopped after a native abort.
    # Do not exclude shadow-only cycles from motion/stop metrics.
    ds=[d for d in a['diag'] if d['t']>=anchor]
    kernel_times=[d['kernel_ms'] for d in ds if d['code']==0]
    low=0.;longest=0.;start=None
    for i,d in enumerate(ds):
        j=min(len(s)-1,np.searchsorted(s['t'],d['t']));away=np.hypot(s['x'][j]-3,s['y'][j])>.35
        islow=abs(d['applied_v'])<.02 and away
        if islow:
            if start is None:start=d['t']
            if i+1<len(ds):low+=ds[i+1]['t']-d['t']
            longest=max(longest,d['t']-start)
        else:start=None
    return {'result':r['reason'],'motion_to_end_s':r['elapsed']-anchor,'min_clearance_m':r['min_clearance'],
        'unverified_cycles':sum(d['stop_unverified'] for d in ds if not d['shadow_only']),'reason_counts':dict(Counter(d['reason'] for d in ds)),
        'native_abort_stopped_s':sum(ds[i+1]['t']-d['t'] for i,d in enumerate(ds[:-1]) if d['shadow_only']),
        'low_linear_command_away_goal_s':low,'longest_low_linear_command_s':longest,
        'path_length_m':float(np.linalg.norm(np.diff(np.column_stack((s['x'],s['y'])),axis=0),axis=1).sum()),
        'kernel_p95_ms':float(np.percentile(kernel_times,95)) if kernel_times else None,
        'classification_errors':dict(Counter(str(d.get('classified_error')) for d in ds)),
        'scope':'single closed-loop trial; low linear speed includes turning; not statistical superiority'}


def compare(old,new,out,old_label='接入版 V8',new_label='改进版 V9'):
    out.mkdir(parents=True,exist_ok=False);trials=[load(old),load(new)]
    for a in trials:a['geometry']=geometry(a['path'])
    font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',18)
    small=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',15)
    duration=max(a['result']['elapsed']-(a['result']['motion_anchor'] or a['result']['goal_sent']) for a in trials)
    frames=[]
    for elapsed in np.arange(0,duration+.2,.2):
        im=Image.new('RGB',(1100,505),'white');d=ImageDraw.Draw(im)
        d.text((24,12),'橙：REMANI 参考  蓝：实际运动及车体投影  灰：障碍物',font=font,fill='#333333')
        d.text((925,15),f't={elapsed:.1f}s',font=small,fill='#333333')
        for panel,(a,label) in enumerate(zip(trials,[old_label,new_label])):
            r=a['result'];anchor=r['motion_anchor'] or r['goal_sent'];t=min(anchor+elapsed,r['elapsed'])
            s=a['state'];j=max(0,np.searchsorted(s['t'],t,side='right')-1);state=s[j]
            def xy(x,y):return (panel*550+35+(x+1.5)*80,95+(2-y)*80)
            d.text((panel*550+200,57),label,font=font,fill='#333333')
            d.rectangle((*xy(-1.5,2),*xy(4.5,-2)),outline='#dddddd',width=2)
            refs=[v for v in a['refs'] if v['t']<=t];events=[v for v in a['events'] if v['t']<=t]
            if refs and events and events[-1]['event'] not in ('cancel','invalid') and refs[-1]['revision']==events[-1]['revision']:
                d.line([xy(v[0],v[1]) for v in refs[-1]['reference']],fill='#d78224',width=2)
            if j>0:d.line([xy(x,y) for x,y in zip(s['x'][:j+1],s['y'][:j+1])],fill='#2874a6',width=3)
            footprint(d,xy,80,a['geometry'][0],(state['x'],state['y'],state['yaw']),a['geometry'][1])
            o=a['obs']
            if len(o):
                oi=np.searchsorted(o['t'],t,side='right')-1
                if oi>=0 and t-o['t'][oi]<.1:
                    for ob in o[o['t']==o['t'][oi]]:
                        x,y=xy(ob['x'],ob['y']);rad=ob['r']*80;d.ellipse((x-rad,y-rad,x+rad,y+rad),fill='#aaaaaa')
            ds=[v for v in a['diag'] if v['t']<=t];reason=ds[-1]['reason'] if ds else 'waiting'
            d.text((panel*550+35,445),reason if t<r['elapsed'] else '结束：'+r['reason'],font=small,fill='#333333')
        frames.append(im)
    frames[0].save(out/'comparison.gif',save_all=True,append_images=frames[1:],duration=200,loop=0)
    frames[-1].save(out/'paths.png')
    report={'old':metrics(trials[0]),'new':metrics(trials[1]),'alignment':'measured motion anchor, not absolute launch time',
        'sources':{str(a['path']):{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in a['path'].iterdir() if p.name in ('state.csv','obstacles.csv','geometry.csv','geometry.json','mppi_diagnostics.json','consumed_references.jsonl','manifest.json')} for a in trials}}
    (out/'comparison.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--old',type=Path,required=True);p.add_argument('--new',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--old-label',default='接入版 V8');p.add_argument('--new-label',default='改进版 V9')
    a=p.parse_args();compare(a.old,a.new,a.out,a.old_label,a.new_label)
