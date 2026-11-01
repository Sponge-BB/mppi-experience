#!/usr/bin/env python3
"""Small read-only native cusp figure, without a simulation or rendering loop."""
import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE=Path(__file__).resolve().parent
FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,default=HERE/'results/cusp_phase_native_20261031_01')
    args=parser.parse_args();out=args.input/'report';out.mkdir(exist_ok=True)
    summary=json.loads((args.input/'summary.json').read_text())
    records={name:[json.loads(line) for line in (args.input/(name+'.jsonl')).read_text().splitlines()]
             for name in ('late_off','late_on','reverse_to_forward_on')}
    with np.load(args.input/'common_inputs.npz',allow_pickle=False) as archive:
        reference=archive['reference']
    image=Image.new('RGB',(1120,670),'white');draw=ImageDraw.Draw(image)
    font=lambda size:ImageFont.truetype(FONT,size)
    draw.text((34,18),'同一停点：仅修正换向段关联，能否重新出发？',font=font(29),fill='#222222')
    draw.text((34,65),'903点REMANI参考、67碰撞球、原生3D检查与刹停检查保留',font=font(21),fill='#555555')
    bounds=(5.60,6.08,-.055,.215);box=(108,142,1006,474)
    def point(x,y):
        return (box[0]+(x-bounds[0])/(bounds[1]-bounds[0])*(box[2]-box[0]),
                box[3]-(y-bounds[2])/(bounds[3]-bounds[2])*(box[3]-box[1]))
    for x in (5.6,5.7,5.8,5.9,6.0):
        a,b=point(x,bounds[2]),point(x,bounds[3]);draw.line((a,b),fill='#eeeeee',width=1)
        draw.text((a[0]-19,box[3]+9),'%.1f'%x,font=font(18),fill='#666666')
    for y in (0.,.1,.2):
        a,b=point(bounds[0],y),point(bounds[1],y);draw.line((a,b),fill='#eeeeee',width=1)
        draw.text((51,a[1]-10),'%.1f'%y,font=font(18),fill='#666666')
    draw.text((1018,477),'x / m',font=font(17),fill='#666666')
    draw.text((39,116),'y / m',font=font(17),fill='#666666')
    draw.line([point(*r[:2]) for r in reference[818:]],fill='#d79d45',width=5)
    path=[point(*r['state'][:2]) for r in records['late_on']]
    draw.line(path,fill='#397ca6',width=5)
    x,y=path[0];draw.ellipse((x-9,y-9,x+9,y+9),fill='#be5757',outline='white',width=2)
    draw.text((x+14,y-17),'A停住 / 共同起点',font=font(19),fill='#a94444')
    x,y=path[-1];draw.ellipse((x-7,y-7,x+7,y+7),fill='#397ca6')
    x,y=point(*reference[-1,:2]);draw.line((x-10,y,x+10,y),fill='#333333',width=3)
    draw.line((x,y-10,x,y+10),fill='#333333',width=3)
    draw.text((x-20,y+16),'目标',font=font(19),fill='#333333')
    draw.line((117,110,158,110),fill='#d79d45',width=5)
    draw.text((166,97),'最后前进段参考',font=font(20),fill='#555555')
    draw.line((442,110,483,110),fill='#397ca6',width=5)
    draw.text((491,97),'B修正后的实走轨迹',font=font(20),fill='#555555')
    draw.text((45,539),'A 原关联：25秒仍原地，距目标36.4厘米',font=font(24),fill='#a94444')
    draw.text((45,579),'B 修正关联：4.75秒达到原位置/朝向门限，距目标1.6厘米',font=font(24),fill='#397ca6')
    draw.text((45,624),'新构造共同3D场；数值闭环，非原场景全输入复现，非实车。结束时仍有角速度。',font=font(18),fill='#777777')
    image.save(out/'换向停滞定点对照.png')
    text=[
        '1. 这次把旧停点和903点参考固定下来，使用完整原生MPPI内核。原关联连续25秒仍不动；只修正当前前进/倒车段的关联，4.75秒进入原有到达门限。原版库和新库关开关时的动作、代价、执行及刹停验收完全一致。原先不是碰撞检查挡住了前进，而是跨换向段关联让前进候选得分比停车更差。',
        '2. 没放宽球形外形、轮速、加速度或碰撞验收，也没强制前进。67球的完整3D轨迹检查和刹停检查都通过。另从合法倒车状态出发，保留了5个控制周期的倒车，低速确认后才换到前进段；没有把倒车一律禁掉。',
        '3. 边界要说清楚：旧实验没有保存完整3D场，本次为两组共同新构造的开阔3D场，不是旧实测感知的精确重播，也没有模拟在线计算延迟。B结束时距目标约1.64厘米、朝向误差约0.092弧度，但角速度仍约0.172弧度/秒；达到的是原位置/朝向/线速度门限，不代表已经完全静止。该修复在新在线障碍物场景是否仍有效，要看随后闭环对照。'
    ]
    (out/'实验结论.txt').write_text('\n\n'.join(text)+'\n')
    print(out)


if __name__=='__main__':main()
