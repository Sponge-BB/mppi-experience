#!/usr/bin/python3
"""Own only this release's process groups; never send actuator messages."""
import argparse
import fcntl
import json
import math
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
NAMESPACE='/standby_trial_20261105_feedback_02'


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


def verify_receive_only_can(interface,allow_hardware_ack=False):
    if not interface.startswith('can') or not interface[3:].isdigit():
        raise RuntimeError('Select an existing canN interface; this release never configures it.')
    details=json.loads(subprocess.check_output(['/usr/sbin/ip','-json','-details','link','show',interface]))
    if len(details)!=1 or details[0].get('linkinfo',{}).get('info_kind')!='can' or 'UP' not in details[0].get('flags',[]):
        raise RuntimeError('CAN interface is not an existing UP CAN device; no receiver started.')
    text=subprocess.check_output(['/usr/sbin/ip','-details','link','show',interface]).decode()
    listen_only='LISTEN-ONLY' in text
    if not listen_only and not allow_hardware_ack:
        raise RuntimeError('CAN must already report LISTEN-ONLY; no interface change or receiver start was attempted.')
    return dict(interface=details[0],detail=text,listen_only=listen_only,
        hardware_ack_allowed=bool(allow_hardware_ack),application_can_tx_enabled=False,
        controller_mode='listen_only' if listen_only else 'normal_ack_application_receive_only')


def start_component(args,data,live):
    uri=args.master_uri or data.get('master_uri')
    parsed=urlparse(uri or '')
    if parsed.scheme!='http' or parsed.hostname not in ('127.0.0.1','localhost') or parsed.port!=11461:
        raise RuntimeError('Real feedback and sensors require this release private localhost master on port 11461.')
    if not any(entry.get('component')=='master' for entry in live) or not master_ok(uri):
        raise RuntimeError('Start this release bin/start_monitor.sh first. An unowned master is never reused.')
    component=args.action[len('start-'):]
    if any(entry.get('component')==component for entry in live):
        raise RuntimeError('This release already owns a live '+component+' process.')
    env=os.environ.copy(); env.update(ROS_MASTER_URI=uri,ROS_IP='127.0.0.1',ROS_HOSTNAME='localhost')
    log_dir=Path(data['run_dir']); log_dir.mkdir(parents=True,exist_ok=True)
    suffix=str(time.time_ns())
    can_state=None
    if component=='feedback':
        if not math.isfinite(args.duration_s) or args.duration_s<=0.:
            raise RuntimeError('--duration-s must be a positive finite observation interval.')
        can_state=verify_receive_only_can(args.can_interface,args.allow_hardware_ack)
        receiver=ROOT/'runtime/piper_can_receive_only.py'
        if not receiver.is_file():
            raise RuntimeError('Missing frozen receive-only CAN entry.')
        command=['/usr/bin/python3',str(receiver),'--interface',args.can_interface,
            '--duration',str(args.duration_s),'--output-jsonl',str(log_dir/('feedback_'+suffix+'.jsonl'))]
    else:
        filename={'sensors':'start_raw_sensors.sh','odometry':'start_sensor_odometry.sh',
                  'tracker':'start_private_tracker.sh'}[component]
        wrapper=ROOT/'bin'/filename
        if not wrapper.is_file():
            raise RuntimeError('Missing integrated sensor wrapper: '+str(wrapper))
        command=['/bin/bash',str(wrapper),'--master-uri',uri]
    entry=launch(command,log_dir/(component+'_'+suffix+'.log'),env)
    entry.update(component=component,hardware_actuator_publishers=False)
    if can_state is not None:
        entry['can_interface_verification']=can_state
    data.setdefault('processes',[]).append(entry); save(data)
    time.sleep(.6)
    if not owned(entry):
        entry['startup_failed']=True; save(data)
        raise RuntimeError('The '+component+' process exited; inspect '+entry['log'])
    print(json.dumps(dict(component=component,entry=entry,status='running'),indent=2))


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


def restart_shadow(data,live):
    """Replace one owned observer without interrupting any other component."""
    uri=data.get('master_uri')
    parsed=urlparse(uri or '')
    if parsed.scheme!='http' or parsed.hostname not in ('127.0.0.1','localhost') or parsed.port!=11461:
        raise RuntimeError('Shadow restart requires this release private localhost master 11461.')
    masters=[entry for entry in live if entry.get('component')=='master']
    shadows=[entry for entry in live if entry.get('component')=='shadow']
    if len(masters)!=1 or not owned(masters[0]) or not master_ok(uri):
        raise RuntimeError('The owned private master could not be verified; no process was signalled.')
    if len(shadows)!=1 or not owned(shadows[0]):
        raise RuntimeError('Exactly one owned shadow must be verified; no process was signalled or replaced.')
    old=shadows[0]
    node=ROOT/'runtime/hardware_shadow_node.py'
    if not node.is_file():
        raise RuntimeError('Missing integrated observer entry; existing processes were preserved.')
    # Recheck PID/start ticks, PGID and release environment at every signal.
    if not owned(old):
        raise RuntimeError('Shadow ownership changed before restart; no signal was sent.')
    os.killpg(old['pid'],signal.SIGINT)
    deadline=time.monotonic()+8.
    while owned(old) and time.monotonic()<deadline:
        time.sleep(.1)
    if owned(old):
        os.killpg(old['pid'],signal.SIGTERM)
        deadline=time.monotonic()+2.
        while owned(old) and time.monotonic()<deadline:
            time.sleep(.1)
    if owned(old):
        raise RuntimeError('Owned shadow has not exited; refusing to start a duplicate observer.')
    old.update(replaced_by_restart=True,stopped_wall=time.time())
    save(data)
    env=os.environ.copy()
    env.update(ROS_MASTER_URI=uri,ROS_IP='127.0.0.1',ROS_HOSTNAME='localhost',
               STANDBY_RUN_DIR=data['run_dir'])
    log=Path(data['run_dir'])/('shadow_restart_'+str(time.time_ns())+'.log')
    entry=launch(['/usr/bin/python3',str(node),'--release-dir',str(ROOT),
                  '--namespace',NAMESPACE],log,env)
    entry.update(component='shadow',hardware_actuator_publishers=False,replaces_pid=old['pid'])
    data['processes'].append(entry); save(data)
    time.sleep(.6)
    if not owned(entry):
        entry['startup_failed']=True; save(data)
        raise RuntimeError('Replacement observer exited; other components were preserved. Inspect '+str(log))
    data.update(status='running',shadow_restarted_wall=time.time()); save(data)
    print(json.dumps(dict(status='shadow_restarted',old_pid=old['pid'],entry=entry,
                          other_components_preserved=True),indent=2))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['start-shadow','start-monitor','stop','status','view',
        'start-feedback','start-sensors','start-odometry','start-tracker','restart-shadow'])
    parser.add_argument('--master-uri')
    parser.add_argument('--advertise-host')
    parser.add_argument('--can-interface','--interface',dest='can_interface',default='can1')
    parser.add_argument('--duration-s',type=float,default=120.)
    parser.add_argument('--allow-hardware-ack',action='store_true',
        help='Permit an already configured normal CAN controller to ACK; the application still never transmits CAN frames')
    parser.add_argument('--raw-view',action='store_true',help='View live sensor cloud in its real lidar frame before odometry exists')
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
        if args.action in ('start-feedback','start-sensors','start-odometry','start-tracker'):
            start_component(args,data,live); return
        if args.action=='restart-shadow':
            restart_shadow(data,live); return
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
            view_config=ROOT/'config'/('raw_sensor_view.rviz' if args.raw_view else 'shadow.rviz')
            entry=launch([str(rviz),'-d',str(view_config),
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
                entry['component']='master'
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
            entry['component']='shadow'
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
