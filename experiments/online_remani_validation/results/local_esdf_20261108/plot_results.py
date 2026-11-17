from pathlib import Path
import json,os
for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path(__file__).resolve().parent
d=json.loads((p/'benchmark.json').read_text()); s=json.loads((p/'benchmark_summary.json').read_text())
f=[x for x in d['frames'] if x['name'].startswith('nearby_')]
fig,axes=plt.subplots(1,2,figsize=(11.8,4.4),gridspec_kw={'width_ratios':[1.6,1]})
ax=axes[0]
ax.plot(range(1,len(f)+1),[x['full']['pipeline_wall_ms'] for x in f],'o-',color='#546778',label='Full-map EDT')
ax.plot(range(1,len(f)+1),[x['local']['pipeline_wall_ms'] for x in f],'o-',color='#198b77',label='Local incremental EDT')
ax.set(xlabel='Synthetic local obstacle frame',ylabel='Map-update pipeline time (ms)',title='Same real PCD background, same 3-thread worker',ylim=(0,max(x[mode]["pipeline_wall_ms"] for x in f for mode in ("full","local"))*1.1))
ax.grid(alpha=.18); ax.legend(frameon=False)
ax=axes[1]; labels=['Nearby changes','Unchanged','Distant changes']; full=[s['nearby']['full']['pipeline_wall_ms']['median'],s['unchanged_occupancy']['full']['pipeline_wall_ms'],s['widely_separated_changes']['full']['pipeline_wall_ms']]; local=[s['nearby']['local']['pipeline_wall_ms']['median'],s['unchanged_occupancy']['local']['pipeline_wall_ms'],s['widely_separated_changes']['local']['pipeline_wall_ms']]
for xs,values,color,label in [([x-.18 for x in range(3)],full,'#546778','Full'),([x+.18 for x in range(3)],local,'#198b77','Local')]:
 bars=ax.bar(xs,values,width=.34,color=color,label=label)
 for bar,v in zip(bars,values):ax.text(bar.get_x()+bar.get_width()/2,v+3,f'{v:.1f}',ha='center',fontsize=9)
ax.set_xticks(range(3));ax.set_xticklabels(labels,fontsize=9,rotation=12);ax.set(ylabel='Time (ms)',ylim=(0,180),title='Full fallback retained for widespread changes')
ax.spines['top'].set_visible(False);ax.spines['right'].set_visible(False)
fig.suptitle('Offline local ESDF: 14 frames, zero mismatch against capped full EDT',fontsize=14)
fig.text(.5,.015,'Compared after +/-0.6 m truncation. EDT pipeline excludes sensor fusion, ROS transport and MPPI.',ha='center',fontsize=9,color='#555555')
fig.tight_layout(rect=(0,.055,1,.94));fig.savefig(p/'offline_comparison.png',dpi=160);fig.savefig(p/'offline_comparison.svg');plt.close(fig)
