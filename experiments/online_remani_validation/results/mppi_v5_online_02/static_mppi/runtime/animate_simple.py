#!/usr/bin/env python3
"""Minimal GIFs redrawn from recorded trials; no ROS or new simulation."""
import argparse
import hashlib
import json
import math
import re
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont

FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
METHODS=[('remani','当前 REMANI'),('mppi','REMANI + MPPI'),('mppi_cbf','REMANI + MPPI + CBF-QP')]
INK='#333333';BLUE='#367594';PLAN='#909B91';OBSTACLE='#C4C4C4'


def centered(draw,x,y,text,font,fill=INK):
    if hasattr(draw,'textbbox'):
        box=draw.textbbox((0,0),text,font=font);width=box[2]-box[0]
    else:width=draw.textsize(text,font=font)[0]
    draw.text((x-width/2,y),text,font=font,fill=fill)


def render(out,scene,methods=None):
    methods=METHODS if methods is None else methods
    pair=len(methods)==2
    panel_width=540 if pair else 360
    scale=100. if pair else 75.
    canvas_height=580 if pair else 470
    trials=[];source_hashes={}
    for mode,label in methods:
        p=out/(scene+'_'+mode)
        manifest=json.loads((p/'manifest.json').read_text())
        build=json.loads((Path(manifest['build'])/'build_manifest.json').read_text())
        if mode=='mppi' and build.get('controller_revision','').endswith('v4'):label='REMANI + 局部引导 + MPPI'
        for name in ('state.csv','result.json','trajectories.json','obstacles.csv','geometry.csv'):
            source_hashes[mode+'/'+name]=hashlib.sha256((p/name).read_bytes()).hexdigest()
        state=np.atleast_1d(np.genfromtxt(p/'state.csv',delimiter=',',names=True))
        result=json.loads((p/'result.json').read_text())
        if re.search(r'REQUIRED process \[[^\]]+\] has died',(p/'launch_stdout.log').read_text(errors='replace')):
            result['reason']='process_failure'
        trajectories=json.loads((p/'trajectories.json').read_text())
        obstacles=np.atleast_1d(np.genfromtxt(p/'obstacles.csv',delimiter=',',names=True))
        body=np.atleast_2d(np.loadtxt(str(p/'geometry.csv'),delimiter=','))
        trials.append((state,result,trajectories,obstacles,body,label))
    title_font=ImageFont.truetype(FONT,21);font=ImageFont.truetype(FONT,17);small=ImageFont.truetype(FONT,15)
    duration=max(r['elapsed']-r['goal_sent'] for _,r,*_ in trials)
    frames=[]
    for elapsed in np.arange(0,duration+.2,.2):
        im=Image.new('RGB',(1080,canvas_height),'white');draw=ImageDraw.Draw(im)
        draw.text((20,9),'静态障碍对比' if scene=='static' else '动态横穿对比',font=title_font,fill=INK)
        draw.text((945,15),f'{elapsed:.1f} s',font=small,fill='#666666')
        draw.line((290,27,321,27),fill=BLUE,width=3);draw.text((330,15),'实际运动',font=small,fill=INK)
        for x in range(455,487,10):draw.line((x,27,x+5,27),fill=PLAN,width=2)
        draw.text((493,15),'规划路线',font=small,fill=INK)
        draw.ellipse((620,20,634,34),fill=OBSTACLE);draw.text((644,15),'障碍物',font=small,fill=INK)
        draw.line((765,22,775,32),fill=INK,width=2);draw.line((765,32,775,22),fill=INK,width=2);draw.text((785,15),'目标',font=small,fill=INK)
        for panel,(state,result,trajectories,obstacles,body,label) in enumerate(trials):
            left=panel*panel_width+(60 if pair else 22);top=92
            def xy(x,y):return (left+(x+.6)*scale,top+(2.1-y)*scale)
            centered(draw,panel*panel_width+panel_width/2,58,label,font)
            # Same scale and world bounds for all three panels.
            draw.rectangle((left,top,left+4.2*scale,top+4.2*scale),outline='#E7E7E7')
            for y in (-2,2):draw.line([xy(-.6,y),xy(3.6,y)],fill='#D8D8D8')
            # Completed trials freeze at their recorded final frame, including
            # the environment; never invent later obstacle/robot motion.
            t=min(result['goal_sent']+elapsed,result['elapsed'])
            i=min(len(state)-1,max(0,int(np.searchsorted(state['t'],t,side='right')-1)));s=state[i]
            if len(obstacles):
                j=int(np.searchsorted(obstacles['t'],t,side='right')-1)
                if j>=0 and t-obstacles['t'][j]<.06:
                    o=obstacles[j];cx,cy=xy(o['x'],o['y']);rad=o['r']*scale
                    draw.ellipse((cx-rad,cy-rad,cx+rad,cy+rad),fill=OBSTACLE)
            current=[]
            for trajectory in trajectories:
                if trajectory['t']>t or trajectory['stamp']>1000+t:continue
                if trajectory['action']!=1:current=[]
                else:
                    if trajectory['id']==1:current=[]
                    current.extend(trajectory['points'])
            for k in range(0,len(current)-1,2):draw.line([xy(current[k][1],current[k][2]),xy(current[k+1][1],current[k+1][2])],fill=PLAN,width=2)
            indices=list(range(0,i+1,2))
            if indices[-1]!=i:indices.append(i)
            if len(indices)>1:draw.line([xy(state[k]['x'],state[k]['y']) for k in indices],fill=BLUE,width=3)
            # The union of the recorded body disks: one quiet silhouette,
            # without the three overlapping debug-circle outlines.
            c,sine=math.cos(s['yaw']),math.sin(s['yaw'])
            for bx,by,radius in body:
                cx,cy=xy(s['x']+c*bx-sine*by,s['y']+sine*bx+c*by);rr=radius*scale
                draw.ellipse((cx-rr,cy-rr,cx+rr,cy+rr),fill='#DAE4E9')
            cx,cy=xy(s['x'],s['y']);draw.ellipse((cx-3,cy-3,cx+3,cy+3),fill=BLUE)
            draw.line([(cx,cy),xy(s['x']+.25*c,s['y']+.25*sine)],fill=BLUE,width=3)
            gx,gy=xy(3,0);draw.line((gx-5,gy-5,gx+5,gy+5),fill=INK,width=2);draw.line((gx-5,gy+5,gx+5,gy-5),fill=INK,width=2)
            finished=result['goal_sent']+elapsed>=result['elapsed']
            if finished:status='已到达' if result['reason']=='goal' else ('超时未完成' if result['reason']=='timeout' else '异常结束')
            elif s['trajectory_count']==0:status='等待轨迹'
            elif abs(s['v'])<.02 and abs(s['w'])<.05:status='停留'
            else:status='运动中'
            if np.any(state['clearance'][:i+1]<0):status+=' · 已发生外形重叠'
            centered(draw,panel*panel_width+panel_width/2,523 if pair else 413,status,small)
        centered(draw,540,canvas_height-25,'数值仿真，非实车  ·  2 倍速',small,fill='#777777')
        frames.append(im)
    name=scene+'_simple'
    # A single explicit palette avoids the old Pillow RGB/GIF palette reuse
    # artifacts (dithered fills, stale labels and ghost robot silhouettes).
    palette_colors=[(255,255,255),(51,51,51),(54,117,148),(144,155,145),
                    (196,196,196),(218,228,233),(231,231,231),(216,216,216)]
    palette_colors += [(v,)*3 for v in range(248)]
    palette=Image.new('P',(1,1));palette.putpalette([v for color in palette_colors for v in color])
    encoded=[frame.quantize(palette=palette,dither=Image.NONE) for frame in frames]
    encoded[0].save(str(out/(name+'.gif')),save_all=True,append_images=encoded[1:],duration=100,loop=0,optimize=False,disposal=2,background=0)
    # Validate actual playback, not just the pre-encoding drawing. Some old
    # Pillow releases retain stale pixels when optimizing inter-frame deltas.
    with Image.open(out/(name+'.gif')) as decoded:
        assert decoded.n_frames==len(encoded)
        for index in (0,min(95,len(encoded)-1),len(encoded)-1):
            decoded.seek(index)
            np.testing.assert_array_equal(np.asarray(decoded.convert('RGB')),np.asarray(encoded[index].convert('RGB')))
    frames[min(len(frames)-1,int(19/.2))].save(out/(name+'_preview.png'))
    (out/(name+'.json')).write_text(json.dumps({'scene':scene,'methods':[m for m,_ in methods],'source_sha256':source_hashes,'new_trials_run':False,'sample_interval_s':.2,'frame_duration_ms':100,'frames':len(frames),'presentation':'same-scale panels, minimal legend, body-disk union silhouette, completed panels frozen'},indent=2)+'\n')
    for mode,_ in methods:
        for name in ('state.csv','result.json','trajectories.json','obstacles.csv','geometry.csv'):
            assert hashlib.sha256((out/(scene+'_'+mode)/name).read_bytes()).hexdigest()==source_hashes[mode+'/'+name]
    print(out/(scene+'_simple.gif'),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('out',type=Path);p.add_argument('--scene',choices=['static','crossing','both'],default='both');p.add_argument('--modes',default='remani,mppi,mppi_cbf');args=p.parse_args()
    methods=[(m,label) for m,label in METHODS if m in args.modes.split(',')]
    if len(methods) not in (2,3):p.error('select two or three valid modes')
    for scene in (['static','crossing'] if args.scene=='both' else [args.scene]):render(args.out,scene,methods)
