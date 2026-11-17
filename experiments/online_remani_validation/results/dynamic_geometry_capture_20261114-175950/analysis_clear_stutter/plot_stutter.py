import json, collections, datetime
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
P=Path('/home/l/remani_ws/experiments/online_remani_validation/results/dynamic_geometry_capture_20261114-175950/analysis_clear_stutter')
def rows(name):
 with (P/(name+'.jsonl')).open() as f: return [json.loads(x) for x in f]
s=rows('status'); c=rows('commands'); p=rows('progress'); logs=rows('logs'); a=1794650950.; b=1794650980.
ss=[r for r in s if a<=r['bag']<=b]; cc=[r for r in c if a<=r['bag']<=b]; pp=[r for r in p if a<=r['bag']<=b]
fig,axs=plt.subplots(3,1,figsize=(12,7),sharex=True,constrained_layout=True)
axs[0].step([r['bag']-a for r in cc],[r['v'] for r in cc],where='post',color='#1864ab',label='Published linear command')
axs[0].set_ylabel('m/s');axs[0].legend(loc='upper left')
valid=[r for r in ss if 'map_age_ms' in r['checked']]
axs[1].plot([r['bag']-a for r in valid],[r['checked']['map_age_ms'] for r in valid],'.-',markersize=3,linewidth=.7,color='#343a40')
axs[1].axhline(350,color='#c92a2a',linestyle='--',label='350 ms validity limit');axs[1].set_ylabel('Map age (ms)');axs[1].legend(loc='upper left')
def group(r):
 if r['state']=='command_delivered': return 0
 if r['state']=='native_reference_cancelled': return 2
 if r['checked'].get('map_age_ok') is False: return 1
 return 3
colors=['#2b8a3e','#e67700','#c92a2a','#7048e8']
for k,label in enumerate(['Delivered','Map expired','Reference cancelled','Other rejection']):
 rr=[r for r in ss if group(r)==k]
 axs[2].scatter([r['bag']-a for r in rr],[k]*len(rr),color=colors[k],s=12,label=label)
axs[2].set_yticks(range(4));axs[2].set_yticklabels(['Delivered','Map expired','Ref cancelled','Other rejection'])
for ax in axs:
 ax.grid(alpha=.2); ax.set_xlim(0,30)
 for r in logs:
  if 'stop before replanning' in r['msg'] and a<=r['source']<=b:
   ax.axvline(r['source']-a,color='#c92a2a',linewidth=1,alpha=.6)
axs[2].set_xlabel('Seconds after 18:09:10 (UTC+8); command/status receipt time')
fig.suptitle('Recorded stop-go commands before a deliberate dynamic obstacle was introduced')
fig.savefig(str(P/'stutter_chain.png'),dpi=140)
pa=[r for r in rows('piper') if r['bag']>=1794650911.]
summary={'plot_window':[a,b],'piper_samples':len(pa),'piper_enabled_false':sum(not r['enabled'] for r in pa),'piper_stop_latched':sum(r['latched'] for r in pa),'piper_feedback_invalid':sum(not r['feedback_valid'] for r in pa),'navigation_live_input_timeouts':[r for r in logs if r['source']>=1794650911. and 'Live input timed out' in r['msg']],'base_tracking_freeze_logs':[r for r in logs if 'freeze reference progress' in r['msg']],'arm_hold_logs':[r for r in logs if 'pause base progress' in r['msg']]}
(P/'additional_chain_checks.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({k:v for k,v in summary.items() if k not in ['base_tracking_freeze_logs','arm_hold_logs']},ensure_ascii=False))
