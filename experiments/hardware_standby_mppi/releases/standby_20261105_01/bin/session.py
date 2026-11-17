#!/usr/bin/python3
"""Own only this release's process groups; never send actuator messages."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
from urllib.parse import urlparse
import xmlrpc.client

ROOT=Path(__file__).resolve().parents[1]
STATE=ROOT/'run/session.json'
NAMESPACE='/standby_trial_20261105_01'


def save(data):
    STATE.parent.mkdir(parents=True,exist_ok=True)
    temp=STATE.with_suffix('.tmp')
    temp.write_text(json.dumps(data,indent=2)+'\n'); temp.replace(STATE)


def ticks(pid):
    try:
        return Path('/proc/'+str(pid)+'/stat').read_text().rsplit(')',1)[1].split()[19]
    except (OSError,IndexError):
        return None


def owned(entry):
    pid=int(entry['pid'])
    try:
        env=Path('/proc/'+str(pid)+'/environ').read_bytes().split(b'\0')
        return ticks(pid)==entry['start_ticks'] and os.getpgid(pid)==pid and \
            ('STANDBY_RELEASE_DIR='+str(ROOT)).encode() in env
    except OSError:
        return False


def launch(command,log,env):
    with log.open('ab',buffering=0) as stream:
        process=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,
                                 cwd=ROOT,env=env,start_new_session=True)
    return dict(pid=process.pid,start_ticks=ticks(process.pid),command=command,log=str(log))


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self,host):
        connection=super().make_connection(host); connection.timeout=1.; return connection


def master_ok(uri):
    try:
        result=xmlrpc.client.ServerProxy(uri,transport=TimeoutTransport()).getPid('/standby_release_check')
        return result[0]==1
    except (OSError,xmlrpc.client.Error):
        return False


def stop(data):
    stopped=[]
    for entry in reversed(data.get('processes',[])):
        if not owned(entry):
            continue
        pid=entry['pid']; os.killpg(pid,signal.SIGINT)
        deadline=time.monotonic()+8.
        while owned(entry) and time.monotonic()<deadline:
            time.sleep(.1)
        if owned(entry):
            os.killpg(pid,signal.SIGTERM)
        stopped.append(pid)
    data.update(status='stopped',stopped_pids=stopped,stopped_wall=time.time()); save(data)
    return stopped


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['start-shadow','start-monitor','stop','status','view'])
    parser.add_argument('--master-uri')
    parser.add_argument('--advertise-host')
    args=parser.parse_args()
    (ROOT/'run').mkdir(exist_ok=True)
    with (ROOT/'run/session.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        data=json.loads(STATE.read_text()) if STATE.exists() else {}
        if args.action=='stop':
            print(json.dumps(dict(stopped_pids=stop(data),release=str(ROOT)))); return
        live=[entry for entry in data.get('processes',[]) if owned(entry)]
        if args.action=='status':
            print(json.dumps(dict(record=data,live_processes=live),indent=2)); return
        if args.action=='view':
            uri=args.master_uri or data.get('master_uri')
            parsed=urlparse(uri or '')
            if parsed.scheme!='http' or not parsed.hostname or not parsed.port or not master_ok(uri):
                raise SystemExit('The explicit ROS master or this release monitor must be reachable before RViz.')
            rviz=Path('/opt/ros/noetic/bin/rviz')
            if not rviz.is_file():
                raise SystemExit('System ROS RViz is not installed; no dependency was changed.')
            env=os.environ.copy(); env['ROS_MASTER_URI']=uri
            if parsed.hostname in ('127.0.0.1','localhost'):
                env['ROS_IP']='127.0.0.1'; env['ROS_HOSTNAME']='localhost'
            elif args.advertise_host:
                env['ROS_HOSTNAME']=args.advertise_host
            else:
                raise SystemExit('A remote master also requires --advertise-host <this-host-address>.')
            log_dir=Path(data['run_dir']) if data.get('run_dir') else ROOT/'logs'
            log_dir.mkdir(parents=True,exist_ok=True)
            entry=launch([str(rviz),'-d',str(ROOT/'config/shadow.rviz'),
                          '__name:=standby_shadow_view','__ns:='+NAMESPACE],log_dir/'rviz.log',env)
            data.setdefault('processes',[]).append(entry); data['master_uri']=uri; save(data)
            print(json.dumps(entry,indent=2)); return
        if live:
            raise SystemExit('This release already has live processes; inspect bin/status.sh or bin/stop.sh.')
        node=ROOT/'runtime/hardware_shadow_node.py'
        if not node.is_file():
            raise SystemExit('Missing frozen runtime/hardware_shadow_node.py; do not start an incomplete release.')
        monitor=args.action=='start-monitor'
        uri='http://127.0.0.1:11461' if monitor else args.master_uri
        parsed=urlparse(uri or '')
        if parsed.scheme!='http' or not parsed.hostname or not parsed.port or parsed.username or parsed.password:
            raise SystemExit('Provide an explicit HTTP ROS master URI, e.g. --master-uri http://127.0.0.1:11311')
        env=os.environ.copy(); env['ROS_MASTER_URI']=uri
        if parsed.hostname in ('127.0.0.1','localhost'):
            env['ROS_IP']='127.0.0.1'; env['ROS_HOSTNAME']='localhost'
        elif args.advertise_host:
            env['ROS_HOSTNAME']=args.advertise_host
        else:
            raise SystemExit('A remote master also requires --advertise-host <this-host-address>.')
        stamp=time.strftime('%Y%m%d_%H%M%S',time.gmtime())
        run=ROOT/'logs'/stamp; run.mkdir(parents=True,exist_ok=False)
        env['STANDBY_RUN_DIR']=str(run)
        data=dict(release=str(ROOT),namespace=NAMESPACE,mode='shadow',hardware_publishers=False,
                  master_uri=uri,run_dir=str(run),status='starting',processes=[])
        try:
            if monitor:
                with socket.socket() as probe:
                    if probe.connect_ex(('127.0.0.1',11461))==0:
                        raise RuntimeError('Port 11461 is occupied; refusing to reuse or stop another master.')
                entry=launch(['/opt/ros/noetic/bin/roscore','-p','11461'],run/'roscore.log',env)
                data['processes'].append(entry); save(data)
                deadline=time.monotonic()+12.
                while not master_ok(uri) and time.monotonic()<deadline:
                    if not owned(entry):
                        raise RuntimeError('The private roscore exited; inspect '+str(run/'roscore.log'))
                    time.sleep(.15)
            if not master_ok(uri):
                raise RuntimeError('ROS master is not reachable; no shadow node was launched.')
            entry=launch(['/usr/bin/python3',str(node),'--release-dir',str(ROOT),
                          '--namespace',NAMESPACE],run/'shadow.log',env)
            data['processes'].append(entry); save(data)
            time.sleep(.6)
            if not owned(entry):
                raise RuntimeError('Shadow process exited; inspect '+str(run/'shadow.log'))
            data.update(status='running',started_wall=time.time()); save(data)
            print(json.dumps(data,indent=2))
        except BaseException:
            stop(data)
            raise


if __name__=='__main__':
    main()
