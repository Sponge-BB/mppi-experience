import json,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path('/home/l/remani_ws/experiments/online_remani_validation/results/frozen_guidance_capture_20261114-161009/analysis_rootcause')
r=[json.loads(l) for l in (p/'guide_replay_rows.jsonl').open()]
r=[x for x in r if 4.8<=x['rel']<=13]
t=np.array([x['rel'] for x in r]);scale=np.array([x['scale'] for x in r]);yaw=np.unwrap([x['yaw'] for x in r]);ref=yaw+np.array([x['exact_yaw_error'] for x in r]);cmd=np.array([x['observed'][1] if x['state']=='command_delivered' else np.nan for x in r]);distance=np.array([x['end_distance'] for x in r])*1000
fig,axs=plt.subplots(4,1,figsize=(11,8),sharex=True,constrained_layout=True)
axs[0].plot(t,np.degrees(yaw),label='Measured yaw');axs[0].plot(t,np.degrees(ref),label='Current native reference yaw');axs[0].set_ylabel('Yaw (deg)');axs[0].legend(loc='upper left',ncol=2)
axs[1].plot(t,cmd,'.-',label='Delivered angular command');axs[1].axhline(0,color='.6',lw=.7);axs[1].set_ylabel('Angular speed\n(rad/s)');axs[1].legend(loc='upper left')
axs[2].plot(t,scale,'.-',color='tab:green');axs[2].set_ylabel('Native clock scale');axs[2].set_ylim(-.05,1.1)
axs[3].plot(t,distance,color='tab:purple',label='Distance to guide endpoint');axs[3].axhline(25,ls='--',color='tab:red',label='Old active-branch threshold: 25 mm');axs[3].set_ylabel('Distance (mm)');axs[3].set_ylim(0,120);axs[3].legend(loc='upper left');axs[3].set_xlabel('Seconds after recorded goal')
for ax in axs:
 ax.grid(alpha=.25)
 for lo,hi in [(5.877,9.147),(10.167,11.547)]:ax.axvspan(lo,hi,color='orange',alpha=.15)
 ax.axvline(9.189,color='.4',ls=':',lw=1)
 ax.axvline(10.187,color='.4',ls=':',lw=1)
fig.suptitle('Recorded turning oscillation: frozen pose vs advancing lookahead\nOrange bands: native progress frozen; missing command points: rejected delivery',fontsize=13)
fig.savefig(p/'turn_rootcause.png',dpi=150)
print(p/'turn_rootcause.png')
