import json,math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
A=Path('/home/l/remani_ws/experiments/online_remani_validation/results/dynamic_acceptance_20261117-103634/analysis')
def read(n):return [json.loads(l) for l in (A/(n+'.jsonl')).open()]
cmd=read('cmd_vel');status=read('mppi_status');tr=read('planning_trajectory')
windows=[(1794883101.6072,1794883155.9394603,'Goal 1: transfer / turn / reverse'),(1794883271.6000993,1794883291.6000993,'Goal 2: first 20 seconds')]
fig,axes=plt.subplots(3,2,figsize=(14,8),sharex='col');fig.patch.set_facecolor('white')
for col,(lo,hi,title) in enumerate(windows):
 cs=[r for r in cmd if lo<=r['t']<=hi];ss=[]
 for r in status:
  d=r['d'];c=d.get('candidate_diagnostics',{});stamp=c.get('plan_time',c.get('capture_inputs',{}).get('checked_at',r['t']-d.get('cycle_ms',0)/1000))
  if lo<=stamp<=hi:ss.append((stamp,d,c))
 ax=axes[0,col];ax.step([r['t']-lo for r in cs],[r['v'] for r in cs],where='post',color='#006fad',lw=1,label='Published /cmd_vel');ax.axhline(0,color='#444444',lw=.5);ax.set_ylabel('Linear command (m/s)');ax.set_title(title);ax.legend(loc='lower left',fontsize=8)
 ax=axes[1,col];ax.step([r['t']-lo for r in cs],[r['w'] for r in cs],where='post',color='#006fad',lw=1);ax.set_ylabel('Angular command (rad/s)');ax.axhline(0,color='#444444',lw=.5)
 ax=axes[2,col];good=[];bad=[]
 for stamp,d,c in ss:
  ck=c.get('checked_inputs',c.get('capture_inputs',{}));age=ck.get('map_age_ms')
  if age is None:continue
  (good if d['state']=='command_delivered' else bad).append((stamp-lo,age))
 for v,color,label in [(good,'#178041','Delivered cycle'),(bad,'#d14636','Rejected cycle')]:
  if v:ax.scatter(*zip(*v),s=10,c=color,label=label,alpha=.8)
 ax.axhline(350,color='#222222',ls='--',lw=1,label='Map age limit: 350 ms');ax.set_ylabel('Checked map age (ms)');ax.set_xlabel('Seconds after goal');ax.set_ylim(0,850);ax.legend(loc='upper right',fontsize=8)
 for row in range(3):
  axes[row,col].grid(alpha=.17);axes[row,col].set_xlim(0,hi-lo)
  for r in tr:
   if r['action']==2 and lo<=r['t']<=hi:axes[row,col].axvline(r['t']-lo,color='#8c4ab8',ls=':',lw=1.2)
fig.suptitle('Recorded command interruptions and map-validity checks',fontsize=13,y=.985)
fig.text(.5,.012,'Blue curves use recorder receipt time; map checks use MPPI source time. Purple dotted lines: slow-planner trajectory cancellation. Curves are commands, not wheel-speed measurements.',ha='center',fontsize=8)
fig.tight_layout(rect=(0,.035,1,.955));fig.savefig(A/'navigation_stop_evidence.png',dpi=160);plt.close(fig)
print(A/'navigation_stop_evidence.png')
