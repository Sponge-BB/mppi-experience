#!/usr/bin/python3
"""Open only this trial's RViz and probe, then stop both after a bounded run."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration',type=float,default=60.)
    parser.add_argument('--label',required=True)
    parser.add_argument('--publish-goal',action='store_true')
    args=parser.parse_args()
    if not 20<=args.duration<=120 or not args.label.replace('_','').isalnum():raise ValueError('bounded label/duration required')
    out=ROOT/'reports'/args.label;out.mkdir(parents=True,exist_ok=False)
    env=os.environ.copy();env.update(ROS_MASTER_URI='http://192.168.137.128:11462',ROS_IP='192.168.137.107',DISPLAY=':0',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    env.pop('ROS_HOSTNAME',None);env.pop('ROS_NAMESPACE',None)
    children=[];logs=[];start=time.monotonic();reason='complete'
    try:
        commands=[('rviz',[str(ROOT/'bin/start_hai_rviz.sh')]),('probe',['/usr/bin/python3',str(ROOT/'tools/hai_probe.py'),'--duration',str(args.duration-2.),'--output',str(out/'probe.json')])]
        if args.publish_goal:commands[-1][1].append('--publish-goal')
        for name,cmd in commands:
            log=(out/(name+'.log')).open('w');logs.append(log)
            children.append((name,subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)))
        while time.monotonic()-start<args.duration:
            available=int(next(s.split()[1] for s in Path('/proc/meminfo').read_text().splitlines() if s.startswith('MemAvailable:')))
            if available<2*1024**2:reason='hai_memory_floor';break
            if children[0][1].poll() is not None:reason='rviz_exited';break
            if children[1][1].poll() is not None:reason='probe_complete' if children[1][1].returncode==0 else 'probe_failed';break
            time.sleep(.5)
    except KeyboardInterrupt:
        reason='interrupted_after_l_trial_stop'
    finally:
        for _,child in children:
            if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
        for _,child in children:
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=3)
        for log in logs:log.close()
        result=dict(reason=reason,elapsed_s=time.monotonic()-start,children=[{'name':name,'pid':p.pid,'exit_code':p.returncode} for name,p in children],only_own_children_stopped=True)
        (out/'view_report.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)


if __name__=='__main__':main()
