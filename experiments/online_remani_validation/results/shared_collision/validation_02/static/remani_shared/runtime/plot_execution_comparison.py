#!/usr/bin/env python3
"""Small recorded-position plot showing motion during reference unavailability."""
import argparse
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from compare_native_v9 import load


def plot(old,new,out,replace=False):
    if out.exists() and not replace:raise FileExistsError(out)
    font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    trials=[load(old),load(new)];fig,axes=plt.subplots(2,1,figsize=(8,4.8),sharex=True,sharey=True)
    for ax,trial,label in zip(axes,trials,['V9：原生取消后等待','V10：局部执行权限']):
        anchor=trial['result']['motion_anchor'];s=trial['state'];mask=s['t']>=anchor
        ax.plot(s['t'][mask]-anchor,s['x'][mask],color='#2874a6',linewidth=2)
        ax.axhline(3,color='#aaaaaa',linestyle=':',linewidth=1)
        if trial is trials[0]:
            events=trial['events']
            for i,event in enumerate(events):
                if event['event']=='cancel':
                    end=next((r['t'] for r in events[i+1:] if r['event']=='replace'),trial['result']['elapsed'])
                    ax.axvspan(event['t']-anchor,end-anchor,color='#dddddd',alpha=.8)
        else:
            pending=[d for d in trial['diag'] if d.get('pending_reference_updates',0)>0]
            if pending:ax.axvspan(pending[0]['t']-anchor,pending[-1]['t']-anchor,color='#dddddd',alpha=.8)
        ax.set_title(label,fontproperties=font,fontsize=12,loc='left')
        ax.set_ylabel('前进位置 x / m',fontproperties=font);ax.set_ylim(0,3.2)
        ax.spines['top'].set_visible(False);ax.spines['right'].set_visible(False)
    axes[-1].set_xlabel('运动触发后的时间 / s',fontproperties=font)
    fig.suptitle('灰色：等待参考更新；蓝线：实测位置（单次案例）',fontproperties=font,fontsize=12)
    fig.tight_layout(rect=(0,0,1,.92));out.parent.mkdir(parents=True,exist_ok=True);fig.savefig(out,dpi=160);plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--old',type=Path,required=True)
    p.add_argument('--new',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--replace',action='store_true',help='replace a previously generated plot at the exact output path')
    a=p.parse_args();plot(a.old,a.new,a.out,a.replace)
