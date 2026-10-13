#!/usr/bin/env python3
"""Verify exact online reference provenance; draw recorded native paths only."""
import argparse
from collections import Counter
import hashlib
import io
import json
import math
import re
from pathlib import Path
import numpy as np
import rospy
from quadrotor_msgs.msg import PolynomialTraj,PolynomialMatrix
from native_reference import sampled_reference,fingerprint
from simulator import polynomial


def recorded_message(record):
    m=PolynomialTraj();m.action=record['action'];m.singul=record['singul'];m.trajectory_id=record['id']
    seconds=math.floor(record['stamp']);nanoseconds=round((record['stamp']-seconds)*1e9)
    m.header.stamp=rospy.Time(seconds,nanoseconds);m.header.frame_id=record.get('frame','')
    for item in record['polynomials']:
        piece=PolynomialMatrix()
        for k,v in item.items():setattr(piece,k,v)
        m.trajectory.append(piece)
    return m


def audit(out):
    out=Path(out)
    raw=json.loads((out/'trajectories.json').read_text())
    diagnostics=json.loads((out/'mppi_diagnostics.json').read_text())
    refs=[json.loads(s) for s in (out/'consumed_references.jsonl').read_text().splitlines()]
    records={r['message_sha256']:r for r in raw}
    canonical_records={r['canonical_message_sha256']:r for r in raw if 'canonical_message_sha256' in r}
    for record in raw:
        if 'canonical_message_sha256' not in record:
            stream=io.BytesIO();recorded_message(record).serialize(stream)
            canonical_records[hashlib.sha256(stream.getvalue()).hexdigest()]=record
    failures=[]
    for r in refs:
        parts=[];times=[];offset=0.
        for h in r['message_hashes']:
            # roscpp assigns a transport header.seq on the public topic. The
            # atomic wrapper carries the original message; all other bytes,
            # including every native coefficient, must match independently.
            record=records.get(h)
            if record is None and r.get('transport')=='complete goal-keyed transaction':record=canonical_records.get(h)
            if record is None:
                failures.append('consumed message missing from independent simulator log');continue
            m=recorded_message(record)
            rows=polynomial(m,step=.04);parts.append(rows[:,1:5]);times.extend((rows[:,0]+offset).tolist());offset=times[-1]
        if not parts:failures.append('no native source');continue
        decoded=sampled_reference(parts,r['reference'][0][2])
        if fingerprint(decoded)!=r['sha256'] or fingerprint(r['reference'])!=r['sha256']:
            failures.append('kernel input does not match decoded native coefficients')
        if not np.array_equal(np.asarray(times),r['times']):failures.append('native time vector mismatch')
    identities={(r['revision'],r['sha256']) for r in refs}
    calls=[d for d in diagnostics if d['code']==0 and not d['shadow_only']]
    if not refs or not calls:failures.append('no live native reference consumed by kernel')
    if any((d['bundle'],d['reference_sha256']) not in identities for d in calls):failures.append('kernel call missing exact reference record')
    blocked=('native_safety_stop','trajectory_changed','native_abort_shadow','deadline_miss','invalid_native_trajectory','waiting_native_trajectory')
    if any(d['reason'] in blocked and (abs(d['applied_v'])>1e-12 or abs(d['applied_w'])>1e-12) for d in diagnostics):failures.append('nonzero command while blocked')
    result=json.loads((out/'result.json').read_text())
    process_failed=bool(re.search(r'REQUIRED process \[[^\]]+\] has died',(out/'launch_stdout.log').read_text(errors='replace')))
    report={'reference_contract_pass':not failures,'closed_loop_reached':result['reason']=='goal' and not process_failed,'required_process_failed':process_failed,'failures':failures,'source':'actual /planning/trajectory; independently recorded polynomial coefficients','native_bundles':result['trajectory_bundles'],'consumed_versions':len(refs),'live_kernel_calls':len(calls),'stop_reasons':dict(Counter(d['reason'] for d in diagnostics)),'trial_result':result,'scope':'reference integration verification, NOT a performance comparison or hardware acceptance'}
    (out/'reference_audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    render(out,refs,diagnostics,result)
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return not failures


def render(out,refs,diagnostics,result):
    from PIL import Image,ImageDraw,ImageFont
    font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',18)
    small=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',15)
    state=np.atleast_1d(np.genfromtxt(out/'state.csv',delimiter=',',names=True))
    obs=np.atleast_1d(np.genfromtxt(out/'obstacles.csv',delimiter=',',names=True))
    events=[json.loads(s) for s in (out/'reference_events.jsonl').read_text().splitlines()]
    def xy(x,y):return (70+(x+1.5)*105,105+(2-y)*105)
    frames=[]
    for t in np.arange(result['goal_sent'] or 0,result['elapsed']+.2,.2):
        t=min(t,result['elapsed']);im=Image.new('RGB',(760,580),'white');d=ImageDraw.Draw(im)
        d.text((25,12),'REMANI 在线参考 + MPPI | '+result['scene'],font=font,fill='#222222')
        d.text((25,43),'橙：REMANI 参考   绿：审核通过的局部轨迹   蓝：实际运动',font=small,fill='#333333')
        d.rectangle((*xy(-1.5,2),*xy(4.5,-2)),outline='#dddddd',width=2)
        j=max(0,np.searchsorted(state['t'],t,side='right')-1);s=state[j]
        current=[r for r in refs if r['t']<=t]
        ev=[e for e in events if e['t']<=t]
        if current and ev and ev[-1]['event'] not in ('cancel','invalid') and current[-1]['revision']==ev[-1]['revision']:
            p=current[-1]['reference'];d.line([xy(r[0],r[1]) for r in p],fill='#d78224',width=3)
        if j>0:d.line([xy(x,y) for x,y in zip(state['x'][:j+1],state['y'][:j+1])],fill='#2874a6',width=3)
        x,y=xy(s['x'],s['y']);d.ellipse((x-7,y-7,x+7,y+7),fill='#2874a6')
        if len(obs):
            oi=np.searchsorted(obs['t'],t,side='right')-1
            if oi>=0 and t-obs['t'][oi]<.1:
                o=obs[oi];x,y=xy(o['x'],o['y']);r=o['r']*105;d.ellipse((x-r,y-r,x+r,y+r),fill='#aaaaaa')
        ds=[v for v in diagnostics if v['t']<=t];reason=ds[-1]['reason'] if ds else 'waiting'
        if ds and ds[-1].get('authorized') and t-ds[-1]['t']<.15 and len(ds[-1].get('local_route',[]))>1:
            d.line([xy(*r) for r in ds[-1]['local_route']],fill='#2b9348',width=2)
        d.text((25,540),f't={t:.1f}s   {reason}   最终结果：{result["reason"]}',font=small,fill='#333333')
        frames.append(im)
    frames[0].save(out/'native_reference.gif',save_all=True,append_images=frames[1:],duration=200,loop=0)
    frames[-1].save(out/'native_reference.png')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('out',type=Path)
    raise SystemExit(0 if audit(p.parse_args().out) else 1)
