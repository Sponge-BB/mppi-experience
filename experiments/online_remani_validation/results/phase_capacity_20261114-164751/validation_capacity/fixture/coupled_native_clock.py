#!/usr/bin/python3
"""Offline native-clock/production-guide coupling. No ROS node or publications."""
import ast, hashlib, json, math, sys, time
from pathlib import Path
import numpy as np
import rosbag, yaml
ROOT=Path('/home/l/remani_ws')
HERE=Path(__file__).resolve().parent
CASE=HERE.parent
sys.path.insert(0,str(ROOT/'src/mm_remani/scripts/mppi'))
from active_arm_mppi import WholeBodyMPPI, wrap
from native_reference_adapter import NativeReferenceAdapter

def definitions(file,names,namespace):
    tree=ast.parse(file.read_text())
    nodes=[n for n in tree.body if (isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name in names) or (isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id in names for t in n.targets))]
    exec(compile(ast.Module(body=nodes,type_ignores=[]),str(file),'exec'),namespace)
    return namespace

PLANT_SOURCE=ROOT/'src/mm_remani/scripts/standby_simulator.py'
Plant=definitions(PLANT_SOURCE,{'Plant','Q_START','STEP'},{'np':np,'math':math})['Plant']
base_command=definitions(ROOT/'src/mm_remani/scripts/whole_body_mppi.py',{'base_command'},{})['base_command']
CFG=yaml.safe_load((ROOT/'src/mm_remani/config/mppi.yaml').read_text())
PARAMS=dict(yaw_hold_speed=.04,virtual_time_scale_rise_rate=.8,virtual_time_scale_fall_rate=2.,base_tracking_slow_distance=.08,base_tracking_stop_distance=.16,base_tracking_resume_distance=.05,base_tracking_slow_yaw=.25,base_tracking_stop_yaw=.55,base_tracking_resume_yaw=.15,base_tracking_min_time_scale=.10)
PARAMS.update({k:v for k,v in yaml.safe_load((ROOT/'src/mm_remani/config/real_speed.yaml').read_text())['mm_controller_node']['controller'].items() if k in PARAMS})

class Model:
    lower,upper=np.full(6,-4.),np.full(6,4.)
    velocity_limits,acceleration_limits=np.ones(6),np.full(6,2.)
    def check(self,q): return {'valid':True}

def load_fixture():
    """Keep exact recorded polynomial coefficients and source-time start pose."""
    file=HERE/'coupled_clock_fixture.json'
    if file.exists(): return json.loads(file.read_text())
    msgs=[]; odoms=[]; joints=[]; goal=None
    with rosbag.Bag(str(CASE/'latency.bag')) as b:
        for topic,m,t in b.read_messages(topics=['/planning/trajectory','/odom','/mm/joint_states','/move_base_simple/goal']):
            if topic=='/planning/trajectory':
                if m.action==1: msgs.append((t.to_sec(),m))
            elif topic=='/move_base_simple/goal':
                q=m.pose.orientation;goal=[m.pose.position.x,m.pose.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
            elif topic=='/odom':
                p,q=m.pose.pose.position,m.pose.pose.orientation
                odoms.append((m.header.stamp.to_sec(),[p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),m.twist.twist.linear.x,m.twist.twist.angular.z]))
            else:
                ids=[m.name.index('joint%d'%i) for i in range(1,7)]
                joints.append((m.header.stamp.to_sec(),[m.position[i] for i in ids]+[m.velocity[i] for i in ids]))
    assert len(msgs)==2 and [m.trajectory_id for _,m in msgs]==[1,2]
    start=msgs[0][1].header.stamp.to_sec()
    assert all(m.header.stamp.to_sec()==start for _,m in msgs)
    base=max((x for x in odoms if x[0]<=start),key=lambda x:x[0]);joint=max((x for x in joints if x[0]<=start),key=lambda x:x[0])
    pieces=[];blocks=[]
    for _,m in msgs:
        block=[]
        for p in m.trajectory:
            row=dict(duration=p.duration,coefficients=np.asarray(p.data).reshape((8,8),order='F').tolist(),gear=m.singul)
            pieces.append(row);block.append(row)
        blocks.append(block)
    fixture=dict(scope='Original recorded 2-gear, 5-piece degree-7 trajectory. Initial pose is latest source-stamped odometry before native start.',native_start=start,base_stamp=base[0],joint_stamp=joint[0],state=base[1]+joint[1],goal=goal,blocks=blocks,pieces=pieces)
    file.write_text(json.dumps(fixture,indent=2)+'\n')
    return fixture

class NativePolynomial:
    def __init__(self,fixture):
        self.pieces=fixture['pieces'];self.ends=np.cumsum([r['duration'] for r in self.pieces]);self.total=float(self.ends[-1])
        self.coeff=[np.asarray(r['coefficients']) for r in self.pieces]
        self.velocity=[np.array([np.polyder(c) for c in a[:2]]) for a in self.coeff]
    def sample(self,phase):
        # Native C++ locates the previous piece at equality (t > duration).
        i=min(len(self.ends)-1,int(np.searchsorted(self.ends,phase,side='left')))
        t=phase-(0. if i==0 else self.ends[i-1])
        p=np.array([np.polyval(c,t) for c in self.coeff[i][:2]])
        v=np.array([np.polyval(c,t) for c in self.velocity[i]])
        speed=float(np.linalg.norm(v));gear=self.pieces[i]['gear']
        return p,v,speed,math.atan2(gear*v[1],gear*v[0]),gear

class NativeBaseClock:
    """Numerical transcription of advanceTrajectoryClock base-only branch.

    Source-fresh odom/joints and no safety limit assumed. Arm tracking is ideal.
    No artificial delay/noise, no controller thresholds or gains are tuned.
    """
    def __init__(self,poly): self.poly=poly;self.phase=0.;self.scale=1.;self.hold=False;self.events=[]
    def step(self,state,dt,t):
        p,v,speed,yaw,gear=self.poly.sample(self.phase)
        dist=float(np.linalg.norm(p-state[:2]))
        err=abs(float(wrap(yaw-state[2]))) if speed>PARAMS['yaw_hold_speed'] else 0.
        prev=self.hold
        if self.hold and dist<=PARAMS['base_tracking_resume_distance'] and err<=PARAMS['base_tracking_resume_yaw']: self.hold=False
        elif not self.hold and (dist>=PARAMS['base_tracking_stop_distance'] or err>=PARAMS['base_tracking_stop_yaw']): self.hold=True
        if prev!=self.hold:self.events.append(dict(t=t,event='freeze' if self.hold else 'resume',phase=self.phase,position_error=dist,yaw_error=err,gear=gear))
        if self.hold:self.scale=0.
        else:
            scales=[]
            for error,stem in [(dist,'distance'),(err,'yaw')]:
                slow=PARAMS['base_tracking_slow_'+stem];stop=PARAMS['base_tracking_stop_'+stem]
                ratio=max(0.,min(1.,(stop-error)/(stop-slow)))
                scales.append(1. if error<=slow else PARAMS['base_tracking_min_time_scale']+ratio*(1.-PARAMS['base_tracking_min_time_scale']))
            target=min(scales);rate=PARAMS['virtual_time_scale_rise_rate'] if target>self.scale else PARAMS['virtual_time_scale_fall_rate']
            self.scale=max(self.scale-rate*dt,min(self.scale+rate*dt,target))
        self.phase=min(self.poly.total,self.phase+dt*self.scale)
        return dist,err

def make_adapter(fixture,k):
    a=NativeReferenceAdapter(k,k,resume_distance=PARAMS['base_tracking_resume_distance'])
    offset=0.;a.arm_pieces=[]
    # Match production reference_polynomial sampling exactly, without ROS objects.
    for block in fixture['blocks']:
        parts=[];local=0.
        for piece in block:
            dur=piece['duration'];c=np.asarray(piece['coefficients']);ts=np.linspace(0,dur,max(2,int(dur/.04)+1))
            x,y=[np.polyval(z,ts) for z in c[:2]];vx,vy=[np.polyval(np.polyder(z),ts) for z in c[:2]]
            speed=np.array([np.linalg.norm([x,y])*piece['gear'] for x,y in zip(vx,vy)])
            yaw=np.array([math.atan2(y*piece['gear'],x*piece['gear']) if abs(s)>1e-5 else float('nan') for x,y,s in zip(vx,vy,speed)])
            parts.append(np.c_[local+ts,x,y,yaw,speed]);local+=dur
            a.arm_pieces.append((dur,c[2:]))
        rows=np.concatenate(parts);a.native.times.append(rows[:,0]+offset);a.native.refs.append(rows[:,1:]);a.native.gears.append(block[0]['gear']);offset+=local
    return a

def simulate(fixture=None,guide=None,max_seconds=120.,delivery_events=None):
    fixture=load_fixture() if fixture is None else fixture
    m=Model();k=WholeBodyMPPI(m,dict(CFG['base_limits']),(m.lower,m.upper,np.asarray(CFG['arm_velocity']),np.asarray(CFG['arm_acceleration'])),[],enabled=False,horizon=CFG['horizon'],step=1./CFG['rate_hz'],random_samples=0)
    a=make_adapter(fixture,k);poly=NativePolynomial(fixture);clock=NativeBaseClock(poly)
    plant=Plant(m,k.limits,k.vmax,k.amax);plant.state=np.asarray(fixture['state']).copy();plant.state[11:]=0.
    u=np.zeros(2);trace=[];clock_trace=[];initial=plant.state.copy();done=False;event_index=0
    for tick in range(int(max_seconds/.005)+1):
        t=tick*.005
        if tick%2==0:
            if tick>0: dist,err=clock.step(plant.state,.01,t)
            else:
                p,_,speed,yaw,_=poly.sample(0.);dist=float(np.linalg.norm(p-plant.state[:2]));err=abs(float(wrap(yaw-plant.state[2]))) if speed>.04 else 0.
            clock_trace.append((t,clock.phase,clock.scale,clock.hold,dist,err))
            if clock.phase>=poly.total-1e-6:done=True;break
        event=None
        if delivery_events is not None:
            while event_index<len(delivery_events) and delivery_events[event_index]['t']<=t+1e-10:
                event=delivery_events[event_index];event_index+=1
        run_control=(tick%20==0) if delivery_events is None else event is not None
        if run_control:
            path,block,end=a._base_reference(clock.phase,clock.scale,plant.state[2]);gear=a.native.gears[block]
            if guide is None:
                arm=a._arm_samples(np.minimum(end,clock.phase+clock.scale*k.dt*np.arange(1,k.H+1)))
                context=dict(gear=gear,pause_key=(fixture['native_start'],block,clock.phase) if clock.scale==0 else None,resume_distance=PARAMS['base_tracking_resume_distance'])
                controls,_=k.proposals(plant.state.copy(),path,arm,[(1.,0.)],tracking=context)
                u=np.array(base_command(controls[0,0],k.limits))
            else: u=np.asarray(guide(plant.state.copy(),a,poly,clock.phase,clock.scale,t,k),dtype=float)
            if event is not None and not event['allow']: u=np.zeros(2)
            trace.append(dict(delivery_event=event,t=t,phase=clock.phase,scale=clock.scale,hold=clock.hold,block=block,gear=gear,position=plant.state[:2].tolist(),yaw=float(plant.state[2]),command=u.tolist(),reference_position_error=dist,reference_yaw_error=err,current_distance=float(np.linalg.norm(path[0,:2]-plant.state[:2])),end_distance=float(np.linalg.norm(path[-1,:2]-plant.state[:2])),pause_align=bool(k._pause_align)))
        plant.step(u,plant.state[5:11].copy(),.005)
    flips=[]
    for prev,row in zip(trace,trace[1:]):
        if prev['command'][1]*row['command'][1]<0 and min(abs(prev['command'][1]),abs(row['command'][1]))>=.05:
            flips.append(dict(before=prev,after=row,scale_mode_changed=(prev['scale']==0)!=(row['scale']==0),same_gear=prev['gear']==row['gear'],endpoint_2_5cm_crossed=(prev['end_distance']<.025)!=(row['end_distance']<.025)))
    nonzero_rows=[r for r in trace if abs(r['command'][1])>=.05]
    nonzero_flips=[dict(before=p,after=r,scale_mode_changed=(p['scale']==0)!=(r['scale']==0),same_gear=p['gear']==r['gear'],endpoint_2_5cm_crossed=(p['end_distance']<.025)!=(r['end_distance']<.025)) for p,r in zip(nonzero_rows,nonzero_rows[1:]) if p['command'][1]*r['command'][1]<0]
    transitions=[dict(before=p,after=r,command_flip=p['command'][1]*r['command'][1]<0 and min(abs(p['command'][1]),abs(r['command'][1]))>=.05) for p,r in zip(trace,trace[1:]) if (p['scale']==0)!=(r['scale']==0)]
    goal=np.array(fixture['goal']);endpos=poly.sample(poly.total)[0]
    return dict(scope='Production proposal first control + production base_command, exact recorded polynomial; transcription of native 100Hz base clock; independent existing 5ms lagged Plant. Fresh ideal feedback, ideal arm tracking, no map/collision checks/latency/replanning/terminal-yaw FSM. Numerical compatibility experiment, not hardware prediction.',params=PARAMS,base_limits=k.limits,initial_state=initial.tolist(),final_state=plant.state.tolist(),native_duration=poly.total,elapsed=t,clock_completed=done,final_phase=clock.phase,final_scale=clock.scale,final_goal_position_error=float(np.linalg.norm(plant.state[:2]-goal[:2])),final_endpoint_position_error=float(np.linalg.norm(plant.state[:2]-endpos)),final_goal_yaw_error=abs(float(wrap(goal[2]-plant.state[2]))),freeze_events=sum(e['event']=='freeze' for e in clock.events),resume_events=sum(e['event']=='resume' for e in clock.events),frozen_seconds=.01*sum(r[3] for r in clock_trace),max_reference_position_error=max(r[4] for r in clock_trace),max_reference_yaw_error=max(r[5] for r in clock_trace),angular_flips_ge_005=len(flips),angular_flips_scale_mode=sum(r['scale_mode_changed'] for r in flips),angular_flips_endpoint_2_5cm=sum(r['endpoint_2_5cm_crossed'] for r in flips),scale_transitions=len(transitions),scale_transition_command_flips=sum(r['command_flip'] for r in transitions),events=clock.events,flips=flips,transitions=transitions,nonzero_sign_changes=len(nonzero_flips),nonzero_sign_changes_scale_mode=sum(r['scale_mode_changed'] for r in nonzero_flips),nonzero_sign_changes_endpoint_2_5cm=sum(r['endpoint_2_5cm_crossed'] for r in nonzero_flips),nonzero_sign_change_definition='Consecutive controls with abs(w)>=.05, skipping smaller/zero commands; counts can include ordinary smooth sign changes over long intervals. Inspect attached rows for root cause.',nonzero_sign_change_rows=nonzero_flips,trace=trace)

if __name__=='__main__':
    began=time.monotonic();result=simulate()
    trace=result.pop('trace');(HERE/'coupled_clock_trace.jsonl').write_text(''.join(json.dumps(r,separators=(',',':'))+'\n' for r in trace))
    result['wall_seconds']=time.monotonic()-began
    result['source_sha256']={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [ROOT/'src/mm_controller/src/mm_controller_fsm.cpp',ROOT/'src/mm_remani/scripts/mppi/active_arm_mppi.py',ROOT/'src/mm_remani/scripts/mppi/native_reference_adapter.py',PLANT_SOURCE]}
    (HERE/'coupled_clock_report.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('flips','transitions','nonzero_sign_change_rows','source_sha256','initial_state','final_state')},indent=2))
