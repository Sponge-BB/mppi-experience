#!/usr/bin/env python3
"""GIFs from recorded native ROS trajectory messages and plant feedback only."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont

MODES=[('remani','当前 REMANI＋原 Tracker',(85,99,115)),('mppi','当前 REMANI＋MPPI',(40,124,177)),('mppi_cbf','当前 REMANI＋MPPI＋CBF',(211,139,34))]
FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'


def render(out,scene):
    trials=[]
    for mode,label,color in MODES:
        p=out/(scene+'_'+mode);a=np.atleast_1d(np.genfromtxt(p/'state.csv',delimiter=',',names=True));r=json.loads((p/'result.json').read_text());tr=json.loads((p/'trajectories.json').read_text());obs=np.atleast_1d(np.genfromtxt(p/'obstacles.csv',delimiter=',',names=True));body=np.loadtxt(str(p/'geometry.csv'),delimiter=',');trials.append((a,r,tr,obs,body,label,color))
    frames=[];font=ImageFont.truetype(FONT,16);small=ImageFont.truetype(FONT,13);duration=max(r['elapsed']-r['goal_sent'] for _,r,*_ in trials)
    for elapsed in np.arange(0,duration+.15,.2):
        im=Image.new('RGB',(1080,450),'white');d=ImageDraw.Draw(im)
        d.text((12,4),f'完整原生 REMANI 在线规划／重规划｜{"突现静态障碍" if scene=="static" else "移动障碍横穿"}｜目标后 {elapsed:.1f}s',font=font,fill='black')
        d.text((12,28),'绿色虚线＝原生规划器当前轨迹；彩线＝实测运动；圆形包络＝固定底盘＋机械臂；数值仿真，非实车',font=small,fill='#555555')
        for panel,(a,r,tr,obs,body,label,color) in enumerate(trials):
            left=panel*360+12;top=80;scale=73
            def xy(x,y):return (left+(x+.55)*scale,top+(2.1-y)*scale)
            d.text((left,54),label,font=font,fill=color)
            for y in [-2,-1,0,1,2]:d.line([xy(-.4,y),xy(4.1,y)],fill='#aaaaaa' if abs(y)==2 else '#eeeeee')
            t=r['goal_sent']+elapsed;i=min(len(a)-1,max(0,int(np.searchsorted(a['t'],t,side='right')-1)));s=a[i]
            if len(obs):
                j=int(np.searchsorted(obs['t'],t,side='right')-1)
                if j>=0 and abs(obs['t'][j]-t)<.2:
                    o=obs[j];cx,cy=xy(o['x'],o['y']);rad=o['r']*scale;d.ellipse((cx-rad,cy-rad,cx+rad,cy+rad),fill='#e79797',outline='#af3838')
            current=[]
            for traj in tr:
                if traj['t']>t or traj['stamp']>1000+t:continue
                if traj['action']!=1:current=[]
                else:
                    if traj['id']==1:current=[]
                    current.extend(traj['points'])
            for k in range(0,len(current)-1,2):d.line([xy(current[k][1],current[k][2]),xy(current[k+1][1],current[k+1][2])],fill='#719466',width=2)
            if i>0:d.line([xy(row['x'],row['y']) for row in a[:i+1:2]],fill=color,width=3)
            c,ss=math.cos(s['yaw']),math.sin(s['yaw'])
            for bx,by,rad in body:
                cx,cy=xy(s['x']+c*bx-ss*by,s['y']+ss*bx+c*by);rr=rad*scale;d.ellipse((cx-rr,cy-rr,cx+rr,cy+rr),outline=color,width=1)
            center=xy(s['x'],s['y']);d.line([center,xy(s['x']+.28*c,s['y']+.28*ss)],fill=color,width=3)
            gx,gy=xy(3,0);d.ellipse((gx-4,gy-4,gx+4,gy+4),fill='green')
            state='到达' if t>=r['elapsed'] and r['reason']=='goal' else '超时未到达' if t>=r['elapsed'] else '运行中'
            d.text((left,400),f'v={s["v"]:.2f} m/s　新轨迹={int(s["trajectory_count"])-1}　{state}',font=small,fill='black')
            guard='不限速' if s['safety_limit']<0 else f'{s["safety_limit"]:.2f} m/s'
            d.text((left,422),f'原生限速：{guard}　净距：{s["clearance"]:.2f} m',font=small,fill='black')
        frames.append(im)
    frames[0].save(str(out/(scene+'_online.gif')),save_all=True,append_images=frames[1:],duration=100,loop=0,optimize=False)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('out',type=Path);p.add_argument('--scene',choices=['static','crossing','both'],default='both');a=p.parse_args()
    for scene in (['static','crossing'] if a.scene=='both' else [a.scene]):render(a.out,scene)
