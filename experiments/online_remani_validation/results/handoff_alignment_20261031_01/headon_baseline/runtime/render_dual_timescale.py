#!/usr/bin/env python3
"""Small, read-only GIF/plot renderer for dual-time-scale diagnosis records.

Online inputs are native recorder files. Offline inputs are the neutral
run_mppi_rejoin_diagnosis interface. References appear only after their recorded
consumption time; selected trajectories are never reconstructed by re-solving.
No ROS imports, publishers, controllers, or source-record modifications.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from native_visuals import geometry, footprint, draw_search, SearchReplay

FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
COLORS = dict(reference='#d78224', actual='#2874a6', selected='#248f45',
              obstacle='#a6a6a6', search='#d7e3ec', rejected='#f0dede')


def read_json(path, default=None):
    return json.loads(path.read_text()) if path.exists() else default


def read_jsonl(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()] if path.exists() else []


def read_csv(path):
    if not path.exists():
        return np.empty(0, dtype=[('t', float)])
    return np.atleast_1d(np.genfromtxt(path, names=True, delimiter=',', encoding='utf8'))


def latest(items, t):
    """Recorded timestamps only; never use an event that happened in the future."""
    if not items:
        return None
    index = np.searchsorted([row['t'] for row in items], t, side='right') - 1
    return items[index] if index >= 0 else None


def geometric_lateral(pose, reference):
    """Signed nearest-polyline lateral distance, NOT controller internal error.

    A segment projection avoids falsely counting sparse sample spacing as
    lateral error. Sign is with respect to reference traversal direction.
    """
    reference = np.asarray(reference, dtype=float)
    if len(reference) < 2:
        return float('nan')
    starts, ends = reference[:-1, :2], reference[1:, :2]
    vector = ends - starts
    length2 = np.einsum('ij,ij->i', vector, vector)
    valid = length2 > 1e-16
    if not np.any(valid):
        return float('nan')
    starts, vector, length2 = starts[valid], vector[valid], length2[valid]
    displacement = np.asarray(pose[:2]) - starts
    fraction = np.clip(np.einsum('ij,ij->i', displacement, vector) / length2, 0., 1.)
    residual = displacement - fraction[:, None] * vector
    index = np.argmin(np.einsum('ij,ij->i', residual, residual))
    # Report signed perpendicular residual, not endpoint longitudinal distance.
    return float((vector[index, 0]*residual[index, 1] - vector[index, 1]*residual[index, 0]) / math.sqrt(length2[index]))


class Trial:
    def __init__(self, source, title=None):
        self.source = Path(source).resolve()
        self.state = read_csv(self.source/'state.csv')
        if len(self.state) < 2 or not {'t', 'x', 'y', 'yaw'}.issubset(self.state.dtype.names):
            raise ValueError('Need at least two finite recorded robot poses')
        if not np.isfinite(np.column_stack([self.state[k] for k in ('t','x','y','yaw')])).all():
            raise ValueError('nonfinite recorded state')
        if np.any(np.diff(self.state['t']) < 0):
            raise ValueError('state timestamps must be monotonic')
        self.result = read_json(self.source/'result.json', {})
        self.metadata = read_json(self.source/'metadata.json', {})
        self.offline = (self.source/'reference.csv').exists()
        self.title = title or self.metadata.get('title') or self.source.name
        self.anchor = float(self.result.get('motion_anchor') or self.result.get('goal_sent') or self.state['t'][0])
        self.end = float(min(self.state['t'][-1], self.result.get('elapsed', self.state['t'][-1])))
        if self.end <= self.anchor:
            raise ValueError('No measured interval after the alignment anchor')
        if (self.source/'geometry.csv').exists():
            self.body, self.geometry = geometry(self.source)
        else:
            self.geometry = self.metadata.get('geometry', {})
            self.body = np.asarray(self.geometry.get('body_disks_xy_r', []), dtype=float)
            if self.body.ndim != 2 or self.body.shape[1] != 3 or not len(self.body) or not np.isfinite(self.body).all() or np.any(self.body[:,2] <= 0):
                raise ValueError('Full recorded body geometry is required; point fallback forbidden')
        self.refs = sorted(read_jsonl(self.source/'consumed_references.jsonl'), key=lambda v:v['t'])
        self.ref_events = sorted(read_jsonl(self.source/'reference_events.jsonl'), key=lambda v:v['t'])
        if self.offline:
            ref = read_csv(self.source/'reference.csv')
            self.fixed_ref = np.column_stack([ref[k] for k in ('x','y')])
        else:
            self.fixed_ref = None
        self.diagnostics = read_json(self.source/'mppi_diagnostics.json', [])
        self.diagnostics += read_jsonl(self.source/'diagnostics.jsonl')
        self.diagnostics.sort(key=lambda v:v['t'])
        self.execution = read_jsonl(self.source/'execution_rollouts.jsonl')
        self.execution.sort(key=lambda v:v['t'])
        self.selected = sorted(read_jsonl(self.source/'selected_rollouts.jsonl'), key=lambda v:v['t'])
        self.control_decisions = sorted(read_jsonl(self.source/'decisions.jsonl'), key=lambda v:v['t'])
        self.search = SearchReplay(self.source)
        self.search_json = sorted(read_jsonl(self.source/'search_trace.jsonl'), key=lambda v:v['t'])
        self.obs = read_csv(self.source/'obstacles.csv')
        self.obs_json = sorted(read_jsonl(self.source/'obstacles.jsonl'), key=lambda v:v['t'])
        self.inputs = [p for p in self.source.iterdir() if p.is_file() and p.suffix in ('.csv','.json','.jsonl')]
        self.inputs += sorted((self.source/'search').glob('*.npz'))
        self.hashes = {str(p.relative_to(self.source)):hashlib.sha256(p.read_bytes()).hexdigest() for p in self.inputs}

    def state_at(self, t):
        index = max(0, np.searchsorted(self.state['t'], t, side='right')-1)
        return self.state[min(index,len(self.state)-1)], index

    def reference_at(self, t):
        if self.fixed_ref is not None:
            # This is a deliberately held accepted native reference, not online
            # replanning. Provenance is saved in the independent metadata.
            return self.fixed_ref
        ref, event = latest(self.refs,t), latest(self.ref_events,t)
        if ref is None:
            return None
        if event and (event.get('event') in ('cancel','invalid') or ref.get('revision') != event.get('revision')):
            return None
        return np.asarray(ref['reference'], dtype=float)

    def selected_at(self, t):
        if t >= self.end-1e-9:
            return None
        if self.selected:
            row = latest(self.selected,t)
            if row and 0 <= t-row['t'] < float(row.get('lease_s',.075)) and row.get('reason') == 'ok':
                path = np.asarray(row['path'], dtype=float)
                return path if path.ndim == 2 and len(path)>1 else None
            return None
        row = latest(self.execution,t)
        if row and row.get('reason') == 'ok' and 0 <= t-row['t'] < row.get('lease_s',.075):
            path = np.asarray(row['rollout'], dtype=float)
            return path[:,1:] if path.ndim == 2 and len(path)>1 else None
        return None

    def search_at(self, t):
        if t >= self.end-1e-9:
            return None
        if self.search_json:
            row = latest(self.search_json,t)
            if row and 0 <= t-row['t'] < .251:
                return dict(rows=np.asarray(row['rows'],dtype=float))
            return None
        return self.search.at(t)

    def obstacles_at(self, t):
        if self.obs_json:
            row=latest(self.obs_json,t)
            return row['obstacles'] if row and 0<=t-row['t']<.251 else []
        if len(self.obs):
            index=np.searchsorted(self.obs['t'],t,side='right')-1
            if index>=0 and 0<=t-self.obs['t'][index]<.12:
                rows=self.obs[self.obs['t']==self.obs['t'][index]]
                return [dict(x=float(v['x']),y=float(v['y']),radius=float(v['r'])) for v in rows]
        return []

    def observed_arrows_at(self,t):
        row=latest(self.diagnostics,t)
        if row and 0<=t-row['t']<.251 and 0<=row.get('observed_obstacle_age',100)<.35:
            age=row['observed_obstacle_age']
            return [(x+age*vx,y+age*vy,vx,vy) for x,y,vx,vy,r,moving in row.get('observed_obstacles',[]) if moving]
        return []

    def status_at(self,t):
        if t>=self.end-1e-9:
            if self.offline:
                if self.result.get('collision'):
                    return '结束：碰撞'
                if self.result.get('passed'):
                    return '结束：回线并到达终点'
                if not self.result.get('endpoint_reached'):
                    return '结束：未到达终点'
                return '结束：未满足持续回线判据'
            return '结束：'+self.result.get('reason',self.metadata.get('termination','记录结束'))
        if self.offline:
            decision=latest(self.control_decisions,t)
            if decision and decision.get('cusp_debug'):
                if decision.get('cusp_bound') is False:
                    return f"原绕行关联 | {decision['reason']}"
                phase=decision['cusp_debug']
                gear='前进' if phase[5]>0 else '倒退'
                return f"{gear}段 {int(phase[1])+1}/{int(phase[2])} | {decision['reason']}"
        row=latest(self.execution,t) or latest(self.selected,t) or latest(self.diagnostics,t)
        return row.get('reason','等待记录') if row else '等待记录'

    def verify_unchanged(self):
        for name,digest in self.hashes.items():
            if hashlib.sha256((self.source/name).read_bytes()).hexdigest()!=digest:
                raise RuntimeError('Source changed during rendering: '+name)


def bounds(trials):
    arrays=[]
    def visible_actor_positions(points):
        # Actors continue their prescribed motion outside the bounded SIL arena.
        # Do not let their late, remote positions shrink the robot into a dot.
        # This affects ONLY camera framing: no sensor, actor or audit is changed.
        points=np.asarray(points,dtype=float).reshape(-1,2)
        inside=(points[:,0]>=-1.5)&(points[:,0]<=4.5)&(points[:,1]>=-2.)&(points[:,1]<=2.)
        return points[inside]
    for trial in trials:
        arrays.append(np.column_stack((trial.state['x'],trial.state['y'])))
        arrays += [np.asarray(r['reference'])[:,:2] for r in trial.refs]
        if trial.fixed_ref is not None:arrays.append(trial.fixed_ref[:,:2])
        if not trial.offline:
            if len(trial.obs):arrays.append(visible_actor_positions(np.column_stack((trial.obs['x'],trial.obs['y']))))
            for row in trial.obs_json:
                if row['obstacles']:arrays.append(visible_actor_positions([[o['x'],o['y']] for o in row['obstacles']]))
    xy=np.concatenate(arrays)
    low,high=xy.min(axis=0)-.7,xy.max(axis=0)+.7
    # Don't squash an empty straight scene into an unreadable narrow stripe.
    center=(low+high)/2;span=np.maximum(high-low,[3.,2.2])
    return center-span/2,center+span/2


def arrow(draw, start, end, color='#888888'):
    if np.linalg.norm(np.asarray(end)-start)<3:return
    draw.line([start,end],fill=color,width=2)
    angle=math.atan2(end[1]-start[1],end[0]-start[0])
    draw.line([(end[0]-6*math.cos(angle-.5),end[1]-6*math.sin(angle-.5)),end,
               (end[0]-6*math.cos(angle+.5),end[1]-6*math.sin(angle+.5))],fill=color,width=2)


def draw_panel(image, trial, t, rect, limits, show_search=True):
    draw=ImageDraw.Draw(image)
    left,top,right,bottom=rect
    low,high=limits
    scale=min((right-left-12)/(high[0]-low[0]),(bottom-top-12)/(high[1]-low[1]))
    center=(low+high)/2
    def xy(x,y):return ((left+right)/2+(x-center[0])*scale,(top+bottom)/2-(y-center[1])*scale)
    # Clip all scene drawing to its panel, including search curves and body.
    layer=Image.new('RGB',image.size,'white');g=ImageDraw.Draw(layer)
    for ob in trial.obstacles_at(t):
        x,y=xy(ob['x'],ob['y']);r=ob.get('radius',ob.get('r',0))*scale
        g.ellipse((x-r,y-r,x+r,y+r),fill=COLORS['obstacle'])
        if trial.offline:
            arrow(g,(x,y),xy(ob['x']+ob.get('vx',0),ob['y']+ob.get('vy',0)))
    if not trial.offline:
        for x,y,vx,vy in trial.observed_arrows_at(t):arrow(g,xy(x,y),xy(x+vx,y+vy))
    ref=trial.reference_at(t)
    if ref is not None and len(ref)>1:g.line([xy(*p[:2]) for p in ref],fill=COLORS['reference'],width=3)
    row,index=trial.state_at(t)
    if index:g.line([xy(x,y) for x,y in zip(trial.state['x'][:index+1],trial.state['y'][:index+1])],fill=COLORS['actual'],width=3)
    meta=trial.geometry if {'base_length','base_width','arm_collision_points_xyz'}.issubset(trial.geometry) else None
    footprint(g,xy,scale,trial.body,(row['x'],row['y'],row['yaw']),meta)
    # A real short rollout can lie inside the footprint. Draw recorded search
    # and the accepted path last so the filled body does not conceal them.
    if show_search:
        search=trial.search_at(t)
        if search is not None and np.asarray(search['rows']).ndim==2 and len(search['rows']):draw_search(g,xy,search)
    selected=trial.selected_at(t)
    if selected is not None:g.line([xy(*p[:2]) for p in selected],fill=COLORS['selected'],width=3)
    image.paste(layer.crop(rect),(left,top));draw=ImageDraw.Draw(image)
    draw.rectangle(rect,outline='#dddddd')
    return row,index


def render_gif(trials, out, playback=2., step=.2, titles=None):
    limits=bounds(trials);panel_width=620;width=panel_width*len(trials);height=540
    titlefont=ImageFont.truetype(FONT,19);small=ImageFont.truetype(FONT,15)
    duration=max(trial.end-trial.anchor for trial in trials)
    elapsed=list(np.arange(0.,duration,step))+[duration]
    frame_ms=max(20,10*round(1000*step/playback/10))
    actual_playback=1000*step/frame_ms
    frames=[];key=None;key_score=-1
    for age in elapsed:
        im=Image.new('RGB',(width,height),'white');draw=ImageDraw.Draw(im)
        draw.text((18,9),f'仿真时间 {age:.2f} s  |  {actual_playback:.3g} 倍播放',font=titlefont,fill='#333333')
        draw.text((18,38),'橙：REMANI参考   蓝：实际路线与完整投影   绿：获准MPPI轨迹',font=small,fill='#444444')
        draw.text((18,61),'浅色：抽样搜索候选（若有）；灰箭头：1秒运动示意，非完整预测时域',font=small,fill='#666666')
        score=0
        for i,trial in enumerate(trials):
            t=min(trial.anchor+age,trial.end)
            draw.text((i*panel_width+18,88),trial.title,font=titlefont,fill='#333333')
            row,_=draw_panel(im,trial,t,(i*panel_width+18,119,(i+1)*panel_width-18,452),limits)
            draw=ImageDraw.Draw(im)
            ref=trial.reference_at(t)
            lat=geometric_lateral([row['x'],row['y']],ref) if ref is not None else float('nan')
            speed=float(row['v']) if 'v' in row.dtype.names else float('nan')
            draw.text((i*panel_width+18,463),f'几何侧偏 {100*lat:+.1f} cm  |  速度 {speed:+.2f} m/s',font=small,fill='#444444')
            draw.text((i*panel_width+18,487),trial.status_at(t)[:68],font=small,fill='#555555')
            score+=abs(lat) if np.isfinite(lat) else 0
        note=('固定原生参考 / 无在线重规划 / 固定机械臂；数值仿真，非实车'
              if all(trial.offline for trial in trials) else
              '固定机械臂 / 数值仿真，非实车；不补画缺失或过期的MPPI轨迹')
        draw.text((18,514),note,font=small,fill='#777777')
        if score>key_score:key_score=score;key=im.copy()
        frames.append(im.convert('P',palette=Image.ADAPTIVE,colors=128))
    # The last sample is a terminal hold, not a further prediction or motion.
    durations=[frame_ms]*len(frames);durations[-1]=max(frame_ms,500)
    frames[0].save(out/'overview.gif',save_all=True,append_images=frames[1:],duration=durations,loop=0,disposal=2)
    frames[-1].convert('RGB').save(out/'paths.png');key.save(out/'largest_deviation.png')
    return dict(frames=len(frames),sample_sim_s=step,playback_multiplier_requested=playback,
                actual_regular_playback_multiplier=actual_playback,
                terminal_hold_ms=durations[-1],shared_world_limits=[v.tolist() for v in limits])


def plot_tracking(trials,out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    font=FontProperties(fname=FONT,size=10)
    fig,axes=plt.subplots(2,1,figsize=(8,5.5),sharex=True)
    use_return_distance=all(trial.offline and 'geometric_reference_error' in trial.state.dtype.names for trial in trials)
    for trial in trials:
        s=trial.state;valid=s['t']>=trial.anchor
        # Every 5th state is sufficient for a derived plotting curve and avoids
        # re-running nearest-segment work thousands of times per long replay.
        indices=np.flatnonzero(valid)[::5]
        values=[]
        for index in indices:
            row=s[index];ref=trial.reference_at(row['t'])
            if use_return_distance:values.append(100*row['geometric_reference_error'])
            else:values.append(100*geometric_lateral([row['x'],row['y']],ref) if ref is not None else float('nan'))
        axes[0].plot(s['t'][indices]-trial.anchor,values,label=trial.title,linewidth=1.5)
        if 'v' in s.dtype.names:axes[1].plot(s['t'][valid]-trial.anchor,s['v'][valid],label=trial.title,linewidth=1.5)
    axes[0].axhline(0,color='#999999',linewidth=.8)
    for axis in axes:axis.grid(alpha=.2);axis.legend(prop=font,frameon=False)
    axes[0].set_ylabel('回线距离 / cm' if use_return_distance else '几何侧偏 / cm',fontproperties=font)
    axes[1].set_ylabel('实际线速度 / m/s',fontproperties=font)
    axes[1].set_xlabel('自运动开始的仿真时间 / s',fontproperties=font)
    fig.suptitle('同朝向参考的几何距离；记录诊断值，不是MPPI内部代价' if use_return_distance else
                 '已消费参考的最近线段侧偏；不是控制器内部跟踪误差',fontproperties=font)
    fig.tight_layout(rect=[0,0,1,.94]);fig.savefig(out/'tracking.png',dpi=150);plt.close(fig)


def plot_timeline(trials,out):
    """Draw source simulation timestamps; show wall cost only as an annotation."""
    if not any((trial.source/'dual_timeline.jsonl').exists() for trial in trials):return
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    from report_dual_timescale import pair_events
    font=FontProperties(fname=FONT,size=9)
    fig,axes=plt.subplots(len(trials),1,figsize=(9,3.1*len(trials)),squeeze=False)
    names=['请求','搜索','优化','最终验收','显式等待','参考发布/采用']
    for axis,trial in zip(axes[:,0],trials):
        records=read_jsonl(trial.source/'dual_timeline.jsonl')
        requests,_=pair_events(records,'request_begin','request_end',lambda r:r['request_id'])
        stages,_=pair_events(records,'stage_begin','stage_end',lambda r:(r.get('request_id'),r.get('stage')))
        waits,_=pair_events(records,'wait_begin','wait_end',lambda r:r['wait_key'])
        origin=float(trial.metadata.get('sim_time_origin',1000.))+trial.anchor
        for begin,end in requests:
            x0,x1=begin['sim']-origin,end['sim']-origin
            if x1<0:continue
            axis.broken_barh([(x0,max(.005,x1-x0))],(-.27,.54),facecolors='#839db4')
            axis.text((x0+x1)/2,.38,f"{end.get('wall_ms',1000*(end['mono']-begin['mono'])):.0f}ms",ha='center',fontsize=7)
        for begin,end in stages:
            stage=begin.get('stage');lane={'search':1,'optimizer':2,'live_acceptance':3}.get(stage)
            if lane is None:continue
            x0,x1=begin['sim']-origin,end['sim']-origin
            if x1<0:continue
            color='#6ca67c' if end.get('success') else '#d08a7d'
            axis.broken_barh([(x0,max(.005,x1-x0))],(lane-.27,.54),facecolors=color)
        for begin,end in waits:
            x0,x1=begin['sim']-origin,end['sim']-origin
            if x1<0:continue
            axis.broken_barh([(x0,max(.005,x1-x0))],(3.73,.54),facecolors='#e4bf72')
        publications=[r['sim']-origin for r in records if r.get('event')=='reference_publish' and r['sim']>=origin]
        consumptions=[r['t']-trial.anchor for r in trial.refs if r['t']>=trial.anchor]
        if publications:axis.scatter(publications,[4.9]*len(publications),marker='|',s=80,c=COLORS['reference'],label='发布')
        if consumptions:axis.scatter(consumptions,[5.1]*len(consumptions),marker='|',s=80,c=COLORS['selected'],label='MPPI采用')
        if len(trial.obs):
            onset=float(trial.obs['t'].min()-trial.anchor)
            axis.axvline(onset,color='#888888',linestyle='--',linewidth=1,label='障碍真值出现，非感知识别')
        axis.set_yticks(range(6));axis.set_yticklabels(names,fontproperties=font)
        axis.set_xlim(0,trial.end-trial.anchor);axis.set_ylim(5.6,-.8)
        axis.set_title(trial.title,fontproperties=font,pad=38);axis.grid(axis='x',alpha=.2)
        axis.legend(prop=font,frameon=False,loc='lower center',bbox_to_anchor=(.5,1.02),ncol=3)
        axis.set_xlabel('自运动开始的仿真时间 / s；请求旁数字为墙钟耗时，嵌套阶段不相加',fontproperties=font)
    fig.tight_layout();fig.savefig(out/'timeline.png',dpi=160);plt.close(fig)


def render(sources,out,titles=None,playback=2.,step=.2,refresh=False):
    if not 0<playback<=20 or not .05<=step<=2:raise ValueError('invalid playback/step')
    out=Path(out);trials=[Trial(source,titles[i] if titles else None) for i,source in enumerate(sources)]
    out.mkdir(parents=True,exist_ok=refresh)
    report=render_gif(trials,out,playback,step)
    plot_tracking(trials,out)
    plot_timeline(trials,out)
    for trial in trials:trial.verify_unchanged()
    report.update(visualization_only=True,no_controller_reexecution=True,source_unchanged=True,
                  no_future_reference=True,no_missing_rollout_fabricated=True,
                  scope='fixed-arm numerical diagnosis; not Gazebo or real vehicle',
                  lateral_definition='signed perpendicular residual of nearest active-reference line segment; not internal controller error',
                  alignment='recorded motion_anchor, else goal_sent, else first state; each panel aligned independently',
                  arrow_definition='one-second estimated CV velocity online; one-second prescribed actor velocity offline; not full prediction horizon',
                  camera_note='Offline camera fits recorded robot/reference only; off-panel actors remain in the experiment and collision audit. Online camera also includes in-arena actors.',
                  sources=[dict(path=str(t.source),hashes=t.hashes,body_disks_xy_r=t.body.tolist(),
                                held_reference=t.offline,reference_provenance=t.metadata.get('reference_source')) for t in trials])
    (out/'visualization_audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,action='append',required=True,help='one source per panel (one or two)')
    parser.add_argument('--title',action='append',help='one concise title per source')
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--playback',type=float,default=2.)
    parser.add_argument('--step',type=float,default=.2,help='simulation seconds between displayed recorded frames')
    args=parser.parse_args()
    if len(args.source)>2:parser.error('at most two panels')
    if args.title and len(args.title)!=len(args.source):parser.error('one --title per --source')
    report=render(args.source,args.out,args.title,args.playback,args.step)
    print(json.dumps({k:v for k,v in report.items() if k!='sources'},ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
