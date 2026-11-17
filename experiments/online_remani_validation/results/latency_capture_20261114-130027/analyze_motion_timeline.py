import bisect, collections, json, math
from pathlib import Path
import rosbag

out=Path(__file__).resolve().parent
goals=[]; trajectories=[]; commands=[]; odom=[]; statuses=[]; finishes=[]
with rosbag.Bag(str(out/'latency.bag')) as bag:
    for topic,m,t in bag.read_messages(topics=['/move_base_simple/goal','/planning/trajectory','/cmd_vel','/odom','/mppi/status','/planning/finish']):
        at=t.to_sec()
        if topic=='/move_base_simple/goal':
            goals.append(dict(t=at,frame=m.header.frame_id,x=m.pose.position.x,y=m.pose.position.y))
        elif topic=='/planning/trajectory':
            trajectories.append(dict(t=at,action=m.action,id=m.trajectory_id))
        elif topic=='/cmd_vel': commands.append((at,m.linear.x,m.angular.z))
        elif topic=='/odom':
            odom.append((at,m.pose.pose.position.x,m.pose.pose.position.y,m.twist.twist.linear.x,m.twist.twist.angular.z,m.header.frame_id))
        elif topic=='/planning/finish': finishes.append((at,m.data))
        else:
            d=json.loads(m.data);c=d.get('candidate_diagnostics',{});check=c.get('checked_inputs',c.get('capture_inputs',{}))
            statuses.append(dict(t=at,state=d.get('state'),cycle=d.get('cycle_ms'),v=d.get('base_v'),w=d.get('base_w'),map_age=check.get('map_age_ms'),stage=check.get('stage')))
assert len(goals)==1
start=goals[0]['t']; commands=[v for v in commands if v[0]>=start]; odom=[v for v in odom if v[0]>=start]
statuses=[s for s in statuses if s['t']>=start]; trajectories=[s for s in trajectories if s['t']>=start]
aborts=[s['t'] for s in trajectories if s['action'] in (2,5)]
stop=aborts[-1] if aborts else commands[-1][0]
live=[s for s in statuses if s['t']<stop]
status_times=[s['t'] for s in statuses]; zero_runs=[]; begin=None
for i,(at,v,w) in enumerate(commands):
    if at>stop: break
    zero=abs(v)+abs(w)<1e-6
    if zero and begin is None: begin=at
    if not zero and begin is not None:
        lo=bisect.bisect_left(status_times,begin-.04); hi=bisect.bisect_right(status_times,begin+.04)
        zero_runs.append(dict(begin_s=begin-start,end_s=at-start,duration_s=at-begin,
                              start_statuses=[s['state'] for s in statuses[lo:hi]]))
        begin=None
if begin is not None:zero_runs.append(dict(begin_s=begin-start,end_s=stop-start,duration_s=stop-begin,start_statuses=['open_until_abort']))
duration_zero=sum(max(0.,min(commands[i+1][0],stop)-at) for i,(at,v,w) in enumerate(commands[:-1]) if at<stop and abs(v)+abs(w)<1e-6)
active_odom=[v for v in odom if v[0]<=stop]
r=dict(goal=goals[0],trajectory_events=[dict(s,relative_s=s['t']-start) for s in trajectories],
       finish_events=finishes,window_until_last_abort_s=stop-start,command_zero_duration_s=duration_zero,
       zero_runs=zero_runs,statuses_before_last_abort=dict(collections.Counter(s['state'] for s in live)),
       odom_frames=sorted(set(v[5] for v in active_odom)),
       odom_net_displacement_m=math.hypot(active_odom[-1][1]-active_odom[0][1],active_odom[-1][2]-active_odom[0][2]) if active_odom else None,
       caveat='命令为零时长不是车轮实际停住时长；时间为bag接收时间。图中map age来自实际gate，末次取消后的超时不用于解释首次停车。')
(out/'motion_timeline.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axes=plt.subplots(2,1,figsize=(10,5.8),sharex=True)
axes[0].step([v[0]-start for v in commands],[v[1] for v in commands],where='post',label='Command linear x',lw=1.2)
axes[0].plot([v[0]-start for v in odom],[v[3] for v in odom],label='Odometry linear x',lw=.8,alpha=.75)
axes[0].set_ylabel('Velocity (m/s)');axes[0].legend(loc='upper right',fontsize=8)
for name,color in [('command_delivered','#3977b7'),('delivery_deadline_exceeded','#df9b00'),('collision_snapshot_unavailable','#cc1111'),('native_reference_cancelled','#777777')]:
    values=[s for s in statuses if s['state']==name and s['map_age'] is not None]
    axes[1].scatter([s['t']-start for s in values],[s['map_age'] for s in values],s=10,c=color,label=name)
axes[1].axhline(350,c='#cc1111',ls='--',lw=.8,label='Map age limit: 350 ms')
axes[1].set_ylabel('Map age at input check (ms)');axes[1].set_xlabel('Seconds after navigation goal')
axes[1].legend(loc='upper right',fontsize=7,ncol=2)
for ax in axes:
    for at in aborts: ax.axvline(at-start,c='#7f2388',ls='--',lw=1)
    ax.axvspan(stop-start,25,color='#aaaaaa',alpha=.12);ax.grid(alpha=.2);ax.set_xlim(0,25)
axes[0].set_title('Actual run: planner ABORTs precede persistent cancellation')
for i,at in enumerate(aborts):axes[0].annotate('ABORT '+str(i+1),(at-start,0),xytext=(at-start+.15,.17),fontsize=8,color='#7f2388')
fig.tight_layout();fig.savefig(out/'motion_timeline.png',dpi=150);fig.savefig(out/'motion_timeline.svg');plt.close(fig)
print(json.dumps(r,ensure_ascii=False,indent=2))
