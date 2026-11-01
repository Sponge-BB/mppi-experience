#!/usr/bin/env python3
"""Three-group native ROS CPU experiment, with an independent paired probe."""
import argparse
import collections
import csv
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from xmlrpc.client import ServerProxy
import yaml
from prepare import prepare, HERE, ROOT, PORT

GROUPS=('remani','background','coupled')


def process_stats(pid):
    base=Path('/proc')/str(pid)
    raw=(base/'stat').read_text();tail=raw[raw.rfind(')')+2:].split()
    return dict(pid=pid,ppid=int(tail[1]),ticks=int(tail[11])+int(tail[12]),
                birth=int(tail[19]),rss_mb=int(tail[21])*os.sysconf('SC_PAGE_SIZE')/1048576,
                core=int(tail[36]))


class Monitor:
    def __init__(self,root,out):
        self.root=root;self.out=out;self.stop=threading.Event();self.hz=os.sysconf('SC_CLK_TCK')
        self.previous={};self.previous_cores={};self.previous_wall=None;self.samples=0
        self.thread=threading.Thread(target=self.run,daemon=True)
    def run(self):
        with (self.out/'cpu_samples.jsonl').open('w',buffering=1) as log:
            while not self.stop.is_set():
                started=time.monotonic();processes={}
                for p in Path('/proc').iterdir():
                    if not p.name.isdigit():continue
                    try:processes[int(p.name)]=process_stats(int(p.name))
                    except (FileNotFoundError,PermissionError,ProcessLookupError):pass
                selected={self.root};changed=True
                while changed:
                    extra={pid for pid,s in processes.items() if s['ppid'] in selected}-selected
                    changed=bool(extra);selected.update(extra)
                rows=[];previous={};dt=started-self.previous_wall if self.previous_wall else None
                for pid in sorted(selected):
                    if pid not in processes:continue
                    info=processes[pid]
                    try:
                        args=(Path('/proc')/str(pid)/'cmdline').read_bytes().decode(errors='replace').split('\0')
                        name=next((a.split(':=',1)[1] for a in args if a.startswith('__name:=')),
                                  Path(args[0]).name if args[0] else 'unknown')
                        key=(pid,info['birth']);old=self.previous.get(key)
                        info.update(name=name,cpu_pct=(info['ticks']-old['ticks'])/self.hz/dt*100 if old and dt else None)
                        previous[key]={'ticks':info['ticks']}
                        if name in ('base_mppi','background_mppi'):(self.out/'mppi_pid.txt').write_text(str(pid)+'\n')
                        threads=[]
                        if name in ('remani_planner_node','base_mppi','background_mppi'):
                            for tp in (Path('/proc')/str(pid)/'task').iterdir():
                                try:
                                    t=process_stats_thread(tp);tk=(pid,int(tp.name),info['birth']);old=self.previous.get(tk)
                                    t['cpu_pct']=(t['ticks']-old['ticks'])/self.hz/dt*100 if old and dt else None
                                    t['wait_ms']=(t['wait_ns']-old['wait_ns'])/1e6 if old else None
                                    previous[tk]={'ticks':t['ticks'],'wait_ns':t['wait_ns']};threads.append(t)
                                except (FileNotFoundError,ProcessLookupError):pass
                        info['threads']=threads;rows.append(info)
                    except (FileNotFoundError,ProcessLookupError):pass
                cores=[];core_previous={}
                for line in Path('/proc/stat').read_text().splitlines():
                    key,*values=line.split()
                    if not key.startswith('cpu'):continue
                    numbers=list(map(int,values));total=sum(numbers[:8]);idle=numbers[3]+numbers[4]
                    old=self.previous_cores.get(key);core_previous[key]=(total,idle)
                    if old and total>old[0]:cores.append({'core':key,'busy_pct':100*(1-(idle-old[1])/(total-old[0]))})
                log.write(json.dumps({'mono':started,'sample_ms':(time.monotonic()-started)*1000,
                                     'processes':rows,'cores':cores})+'\n')
                self.previous=previous;self.previous_wall=started;self.previous_cores=core_previous;self.samples+=1
                self.stop.wait(max(0.,.1-(time.monotonic()-started)))
    def start(self):self.thread.start()
    def close(self):self.stop.set();self.thread.join(timeout=3)


def process_stats_thread(path):
    text=(path/'stat').read_text();tail=text[text.rfind(')')+2:].split()
    sched=list(map(int,(path/'schedstat').read_text().split()))
    return dict(tid=int(path.name),name=text[text.find('(')+1:text.rfind(')')],
                ticks=int(tail[11])+int(tail[12]),wait_ns=sched[1],core=int(tail[36]))


def prepare_trial(out,build,group,scene,duration,paired=False):
    mode='mppi' if group=='coupled' else 'remani'
    prepare(out,build,mode,scene,duration,projection_progress=group=='coupled',
            scenario_start='motion',perception='classified',execution='local' if group=='coupled' else 'native',
            record_search=False,prediction_horizon=3.)
    tree=ET.parse(out/'experiment.launch');launch=tree.getroot()
    if group=='background':
        (out/'ros_pkgs/online_remani_experiment/resource_shadow.py').symlink_to(out/'runtime/resource_shadow.py')
        node=ET.SubElement(launch,'node',pkg='online_remani_experiment',type='resource_shadow.py',
                           name='background_mppi',output='log',required='true')
        for key,value in dict(out=str(out),mode='mppi',library=str(build/'build/libonline_mppi.so'),
                              projection_progress='false',classified_cloud='true',local_execution_owner='false',
                              record_search='false',shared_collision='false').items():
            ET.SubElement(node,'param',name=key,value=value)
    for node in launch.findall('node'):
        if node.get('name') in ('base_mppi','background_mppi'):
            ET.SubElement(node,'env',name='RESOURCE_MPPI_PID_PATH',value=str(out/'mppi_pid.txt'))
    tree.write(out/'experiment.launch',encoding='unicode')
    manifest=json.loads((out/'manifest.json').read_text())
    manifest.update(resource_group=group,resource_paired_probe=paired,cpu_sample_interval_s=.1,
        background_scope='same V12 kernel and publication-time validator; all publishers isolated; retain last native guide across aborts' if group=='background' else None)
    (out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')


def run_trial(out,build,group,scene,duration,paired=False):
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1',PORT))==0:raise RuntimeError('isolated experiment master already active')
    prepare_trial(out,build,group,scene,duration,paired)
    env={k:v for k,v in os.environ.items() if not re.search('TOKEN|SECRET|PASSWORD|CREDENTIAL|AUTH|API_KEY',k,re.I)}
    env.update(ROS_MASTER_URI='http://127.0.0.1:%d'%PORT,ROS_IP='127.0.0.1',ROS_HOSTNAME='localhost',
               ROS_HOME=str(out/'ros_home'),ROS_LOG_DIR=str(out/'roslogs'),RESOURCE_TRACE_PATH=str(out/'solver_trace.jsonl'))
    env['ROS_PACKAGE_PATH']=str(out/'ros_pkgs')+':'+env.get('ROS_PACKAGE_PATH','')
    if paired:env['RESOURCE_PAIRED_PID_FILE']=str(out/'mppi_pid.txt')
    started=time.monotonic();audit=None
    with (out/'launch_stdout.log').open('w') as log:
        proc=subprocess.Popen(['roslaunch','-p',str(PORT),str(out/'experiment.launch')],env=env,
                              stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        monitor=Monitor(proc.pid,out);monitor.start()
        try:
            while proc.poll() is None and not (out/'result.json').exists() and time.monotonic()-started<duration+45:
                if audit is None and time.monotonic()-started>14:
                    try:
                        _,_,state=ServerProxy('http://127.0.0.1:%d'%PORT).getSystemState('/resource_audit')
                        publishers={topic:nodes for topic,nodes in state[0]}
                        illegal=[topic for topic,nodes in publishers.items() if '/background_mppi' in nodes and not topic.startswith('/resource_shadow/') and topic!='/rosout']
                        audit={'publishers':publishers,'background_nonisolated_publishers':illegal}
                        (out/'publisher_audit.json').write_text(json.dumps(audit,indent=2)+'\n')
                        if illegal:raise RuntimeError('background output isolation failed: '+str(illegal))
                    except OSError:pass
                time.sleep(.2)
        finally:
            # A diagnostic may have paused its OWN MPPI child. Always resume it
            # before terminating this trial's process group.
            if paired and (out/'mppi_pid.txt').exists():
                try:os.kill(int((out/'mppi_pid.txt').read_text()),signal.SIGCONT)
                except ProcessLookupError:pass
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=8)
            monitor.close()
    text=(out/'launch_stdout.log').read_text(errors='replace')
    status={'returncode':proc.returncode,'result_written':(out/'result.json').exists(),
            'required_process_failed':bool(re.search(r'REQUIRED process \[[^\]]+\] has died',text)),
            'cpu_samples':monitor.samples,'elapsed_wall_s':time.monotonic()-started}
    (out/'run_status.json').write_text(json.dumps(status,indent=2)+'\n')
    if not status['result_written'] or status['required_process_failed']:
        raise RuntimeError('trial failed, inspect '+str(out/'launch_stdout.log'))
    print('FINISHED',out.name,json.loads((out/'result.json').read_text())['reason'],flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--build',type=Path,default=HERE/'builds/resource_probe_v12_01')
    parser.add_argument('--repeats',type=int,default=2)
    parser.add_argument('--scenes',nargs='+',default=['empty','static','dynamic_four'])
    parser.add_argument('--duration',type=float,default=48.)
    parser.add_argument('--smoke',action='store_true')
    args=parser.parse_args();out=args.out.resolve();build=args.build.resolve();out.mkdir(parents=True,exist_ok=False)
    metadata={'build':str(build),'affinity':sorted(os.sched_getaffinity(0)),
              'cpu_percent_convention':'100 percent is one fully occupied logical CPU',
              'cpu_count':os.cpu_count(),'kernel':os.uname().release,'date':'2026-10-15 Asia/Shanghai',
              'scope':'isolated native ROS numerical simulation, fixed arm; not Gazebo or physical robot',
              'groups':list(GROUPS),'repeats':args.repeats,'scenes':args.scenes,
              'source_hashes':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in HERE.glob('resource*.*')}}
    for path in ('/proc/self/cgroup','/sys/fs/cgroup/cpu/user.slice/cpu.cfs_quota_us','/sys/fs/cgroup/cpu/user.slice/cpu.cfs_period_us'):
        try:metadata[path]=Path(path).read_text().strip()
        except (FileNotFoundError,PermissionError):pass
    (out/'experiment_plan.json').write_text(json.dumps(metadata,indent=2,ensure_ascii=False)+'\n')
    for repeat in range(args.repeats):
        # Rotate order across scenes/repeats to reduce order/temperature bias.
        for scene_index,scene in enumerate(args.scenes):
            shift=(repeat+scene_index)%3;order=GROUPS[shift:]+GROUPS[:shift]
            for group in order:
                run_trial(out/f'{scene}_{group}_{repeat+1:02d}',build,group,scene,args.duration)
    if not args.smoke:run_trial(out/'paired_static_diagnostic',build,'coupled','static',75.,paired=True)
    print('COMPLETE',out,flush=True)


if __name__=='__main__':main()
