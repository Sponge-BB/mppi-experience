#!/usr/bin/env python3
"""Run one isolated native-ROS numerical simulation, never remote ROS."""
import argparse
import json
import os
import re
import signal
import socket
import subprocess
import time
from pathlib import Path
from prepare import prepare,PORT


def run(args):
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1',PORT))==0:raise RuntimeError('experiment master port already in use')
    out=prepare(args.out,args.build,args.mode,args.scene,args.duration,args.projection_progress,args.planner_variant)
    env={k:v for k,v in os.environ.items() if not re.search('TOKEN|SECRET|PASSWORD|CREDENTIAL|AUTH|API_KEY',k,re.I)}
    env.update(ROS_MASTER_URI='http://127.0.0.1:%d'%PORT,ROS_IP='127.0.0.1',ROS_HOSTNAME='localhost',ROS_HOME=str(out/'ros_home'),ROS_LOG_DIR=str(out/'roslogs'))
    env['ROS_PACKAGE_PATH']=str(out/'ros_pkgs')+':'+env.get('ROS_PACKAGE_PATH','')
    started=time.monotonic()
    with (out/'launch_stdout.log').open('w') as log:
        proc=subprocess.Popen(['roslaunch','-p',str(PORT),str(out/'experiment.launch')],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            while proc.poll() is None and not (out/'result.json').exists() and time.monotonic()-started<args.duration+45:time.sleep(.5)
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=12)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=8)
    if not (out/'result.json').exists():
        raise RuntimeError('Trial did not finish; inspect '+str(out/'launch_stdout.log'))
    print((out/'result.json').read_text(),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True);p.add_argument('--build',type=Path,required=True)
    p.add_argument('--mode',choices=['remani','mppi','mppi_cbf'],default='remani');p.add_argument('--scene',choices=['empty','static','crossing'],default='empty');p.add_argument('--duration',type=float,default=45)
    p.add_argument('--projection-progress',action='store_true')
    p.add_argument('--planner-variant',choices=['current','no_wait'],default='current')
    run(p.parse_args())
