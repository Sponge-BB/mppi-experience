#!/usr/bin/env python3
"""Extract first/middle/last recorded frames for a compact visual check."""
from pathlib import Path
from PIL import Image, ImageDraw

OUT=Path(__file__).resolve().parent.parent/'report'
names=('静态挡路AB.gif','中段横穿AB.gif')
sheet=Image.new('RGB',(1740,550),'white')
draw=ImageDraw.Draw(sheet)
for row,name in enumerate(names):
    with Image.open(OUT/name) as gif:
        count=gif.n_frames
        print(name,'frames',count,'size',gif.size)
        for column,index in enumerate((0,count//2,count-1)):
            gif.seek(index)
            frame=gif.convert('RGB');frame.thumbnail((580,255))
            sheet.paste(frame,(580*column,275*row+20))
            draw.text((580*column+5,275*row+3),'frame %d / %d'%(index,count),fill='black')
sheet.save(OUT/'GIF关键帧.png')
