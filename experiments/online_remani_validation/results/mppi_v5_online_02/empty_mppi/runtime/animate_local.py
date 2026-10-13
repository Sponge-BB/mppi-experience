#!/usr/bin/env python3
"""Minimal local-only GIF; never imply these are native-REMANI runs."""
import argparse
import json
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont

def render(out,scene,old,new):
    data=[json.loads((out/(scene+'_'+name+'.json')).read_text()) for name in (old,new)]
    font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',18)
    small=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',15)
    frames=[];duration=max(r['duration_s'] for r in data)
    for t in np.arange(0,duration+.2,.2):
        im=Image.new('RGB',(1000,500),'white');draw=ImageDraw.Draw(im)
        draw.text((22,10),('静态' if scene=='static' else '动态')+'自主避障：不给新参考',fill='#333333',font=font)
        draw.text((660,12),'蓝：实际运动  绿：局部引导',fill='#555555',font=small)
        for i,r in enumerate(data):
            left=35+500*i
            def xy(x,y):return (left+95*(x+.4),255-95*y)
            draw.text((left+110,48),'旧版 MPPI' if i==0 else '局部引导 + MPPI',fill='#333333',font=font)
            draw.rectangle((left,80,left+400,427),outline='#dddddd')
            for x in np.arange(0,3,.13):draw.line([xy(x,0),xy(min(3,x+.07),0)],fill='#999999')
            rows=r['trace'];k=min(len(rows)-1,int(t/.05));z=rows[k]
            guides=[p for p in r['routes'] if p['t']<=z[0]]
            if guides and len(guides[-1]['points'])>1:draw.line([xy(*p) for p in guides[-1]['points']],fill='#a7bca8',width=1)
            if z[9] is not None:
                cx,cy=xy(z[9],z[10]);rad=.23*95;draw.ellipse((cx-rad,cy-rad,cx+rad,cy+rad),fill='#c7c7c7')
                if scene=='crossing':draw.line((cx,cy,cx,cy-28),fill='#777777',width=2);draw.line((cx-4,cy-22,cx,cy-28,cx+4,cy-22),fill='#777777',width=2)
            if k>0:draw.line([xy(row[1],row[2]) for row in rows[:k+1:2]],fill='#367594',width=3)
            cx,cy=xy(z[1],z[2]);draw.ellipse((cx-7,cy-7,cx+7,cy+7),fill='#367594')
            draw.line([(cx,cy),xy(z[1]+.22*np.cos(z[3]),z[2]+.22*np.sin(z[3]))],fill='#367594',width=2)
            gx,gy=xy(3,0);draw.line((gx-4,gy-4,gx+4,gy+4),fill='black',width=2);draw.line((gx-4,gy+4,gx+4,gy-4),fill='black',width=2)
            finished=t>=r['duration_s'];label=('到达' if r['reached'] else '超时') if finished else ('停留' if abs(z[4])<.02 else '运动中')
            draw.text((left+140,435),label,fill='#333333',font=small)
        draw.text((80,474),'数值仿真 · 理想观测 · 固定旧参考 · 不运行上层重规划 · 2 倍速',fill='#666666',font=small)
        frames.append(im)
    palette=frames[0].quantize(colors=128).getpalette()
    # Explicit common palette includes plot colors even absent in first frame.
    colors=[(255,255,255),(54,117,148),(167,188,168),(199,199,199),(0,0,0)]+[(k,k,k) for k in range(251)]
    pal=Image.new('P',(1,1));pal.putpalette([v for c in colors for v in c])
    encoded=[im.quantize(palette=pal,dither=Image.NONE) for im in frames]
    dest=out/(scene+'_local.gif');encoded[0].save(dest,save_all=True,append_images=encoded[1:],duration=100,loop=0,optimize=False,disposal=2)
    with Image.open(dest) as im:
        assert im.n_frames==len(encoded)
        for k in (0,len(encoded)//2,len(encoded)-1):
            im.seek(k);np.testing.assert_array_equal(np.array(im.convert('RGB')),np.array(encoded[k].convert('RGB')))
    print(dest)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('out',type=Path);p.add_argument('--old',default='mppi_v3_02');p.add_argument('--new',default='mppi_v4_03');a=p.parse_args()
    for scene in ('static','crossing'):render(a.out,scene,a.old,a.new)
