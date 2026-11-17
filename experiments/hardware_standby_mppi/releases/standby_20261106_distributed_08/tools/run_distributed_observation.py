#!/usr/bin/python3
"""Bounded original-sensor/native-planner observation. No actuator process."""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import resource
import signal
import socket
import subprocess
import sys
import time
import xmlrpc.client

RELEASE = Path(__file__).resolve().parents[1]
FEEDBACK = RELEASE.parent/'standby_20261105_feedback_02'
PARENT = RELEASE.parent/'standby_20261106_stationary_07'
sys.path.insert(0, str(PARENT/'tools'))
from run_stationary_preparation import process_rows, memory_available, actuator_publishers

NS = '/standby_trial_20261106_distributed_08'
FLOOR_MIB = 2560
TREE_MIB = 850


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=90.)
    parser.add_argument('--label', required=True)
    parser.add_argument('--recording', action='store_true', help='Exercise existing07 checks with RecordingSink only')
    args = parser.parse_args()
    if not 20 <= args.duration <= 180 or not args.label.replace('_', '').isalnum():
        raise ValueError('Duration20..180 and unique simple label required')
    if os.environ.get('ROS_MASTER_URI') != 'http://192.168.137.128:11462' or os.environ.get('ROS_IP') != '192.168.137.128':
        raise ValueError('Use08 environment on l')
    if memory_available() < (FLOOR_MIB+TREE_MIB)*1024:
        raise RuntimeError('Insufficient free memory for full trial reservation')
    # A second LiDAR process would claim the same device even on another master.
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():
            continue
        try:
            command = (p/'cmdline').read_bytes().split(b'\0')[0].decode()
        except (OSError, UnicodeError):
            continue
        if command.endswith('/feedback02_livox_driver'):
            raise RuntimeError('Existing raw LiDAR driver must be stopped before this trial')
    probe = socket.socket(); probe.settimeout(.3)
    if probe.connect_ex(('127.0.0.1', 11462)) == 0:
        raise RuntimeError('Private11462 already running; refuse duplicate trial')
    probe.close()
    out = RELEASE/'reports'/args.label; out.mkdir(exist_ok=False)
    os.environ['STANDBY_DISTRIBUTED_RUN']=str(out)
    tx_path = Path('/sys/class/net/can0/statistics/tx_packets'); tx_before = int(tx_path.read_text())
    children=[]; logs=[]; owned={}; statuses=[]; counts=Counter(); events=[]
    peak=0; peak_processes=[]; minimum=memory_available(); reason='observation_complete'; start=time.monotonic()
    master=None; rospy=None; pubs=[]; subscriptions=[]
    def limits():
        resource.setrlimit(resource.RLIMIT_AS, (1536*1024**2,)*2)
        resource.setrlimit(resource.RLIMIT_CORE, (0,0))
    def launch(name, cmd):
        log=(out/(name+'.log')).open('w');logs.append(log)
        child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,
            preexec_fn=limits,start_new_session=True,env=os.environ.copy())
        children.append((name,child))
        return child
    def receive_status(msg):
        counts['status']+=1;statuses.append(json.loads(msg.data))
        if len(statuses)>1200:del statuses[0]
    def count(key):
        def callback(msg):counts[key]+=1
        return callback
    try:
        launch('master',['/opt/ros/noetic/bin/rosmaster','--core','-p','11462'])
        socket.setdefaulttimeout(2.)
        master=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        for _ in range(40):
            try:
                if master.getPid('/distributed08_start')[0]==1:break
            except OSError:pass
            time.sleep(.1)
        else:raise RuntimeError('Private master did not start')
        import rospy
        from sensor_msgs.msg import PointCloud2,JointState
        from nav_msgs.msg import Odometry,Path as RosPath
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import String,Bool
        from visualization_msgs.msg import MarkerArray
        rospy.init_node('distributed08_probe',disable_signals=True)
        recording_enable = rospy.Publisher(NS+'/gateway/enable',Bool,queue_size=1) if args.recording else None
        recording_requested = False
        sensor='/standby_trial_20261105_feedback_02/sensors'
        for topic,kind,key in [(sensor+'/lidar_pcl2',PointCloud2,'raw'),(sensor+'/odom',Odometry,'odom'),
                (sensor+'/joint_states',JointState,'joints'),(NS+'/perception/filtered_points',PointCloud2,'filtered'),
                (NS+'/goal',PoseStamped,'goal'),(NS+'/markers',MarkerArray,'markers'),
                (NS+'/native/path',RosPath,'path')]:
            subscriptions.append(rospy.Subscriber(topic,kind,count(key),queue_size=1))
        subscriptions.append(rospy.Subscriber(NS+'/status',String,receive_status,queue_size=1))
        launch('observation',['/opt/ros/noetic/bin/roslaunch',str(RELEASE/'config/observation.launch'),
                              'duration:='+str(args.duration+10.),
                              'record_gateway:='+str(args.recording).lower()])
        last_print=-10.
        while time.monotonic()-start<args.duration:
            elapsed=time.monotonic()-start
            if args.recording and statuses and not recording_requested:
                latest=statuses[-1]
                if latest.get('mode') != 'integration_recording' or latest.get('commands_executed'):
                    raise RuntimeError('Recording-only mode required')
                if latest.get('proposed_controls') is not None and recording_enable.get_num_connections():
                    recording_enable.publish(Bool(data=True))
                    recording_requested=True  # Never re-enable after a failure.
            rows=process_rows(); selected={p.pid for _,p in children}
            while True:
                more={pid for pid,(parent,_,_) in rows.items() if parent in selected}
                if more<=selected:break
                selected.update(more)
            for pid in selected:
                if pid in rows:owned[pid]=rows[pid][1]
            rss=sum(rows[p][2] for p in selected if p in rows)
            if rss>peak:
                peak=rss
                peak_processes=[{'pid':p,'rss_mib':round(rows[p][2]/1024,2),
                    'name':(Path('/proc')/str(p)/'comm').read_text().strip()}
                    for p in selected if p in rows and (Path('/proc')/str(p)/'comm').exists()]
            available=memory_available();minimum=min(minimum,available)
            if available<FLOOR_MIB*1024 or rss>TREE_MIB*1024:
                reason='memory_threshold_reached';break
            if any(p.poll() is not None for _,p in children):
                reason='owned_process_exited';break
            if elapsed-last_print>=5.:
                print(json.dumps({'elapsed_s':round(elapsed,1),'tree_rss_mib':round(rss/1024,1),
                    'available_mib':round(available/1024,1),'counts':dict(counts),
                    'last_status':statuses[-1]['status'] if statuses else None}),flush=True)
                last_print=elapsed
            time.sleep(.25)
    except KeyboardInterrupt:
        reason='interrupted'
    except Exception as exc:
        reason=type(exc).__name__+': '+str(exc)
    finally:
        if master is not None:
            try:pubs=actuator_publishers(master)
            except Exception as exc:events.append('final graph query: '+str(exc))
        for name,child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                if name=='observation':
                    try:child.wait(timeout=4.)
                    except subprocess.TimeoutExpired:pass
        deadline=time.monotonic()+6.
        while any(p.poll() is None for _,p in children) and time.monotonic()<deadline:time.sleep(.1)
        rows=process_rows()
        for pid,stamp in owned.items():
            if pid in rows and rows[pid][1]==stamp:
                try:os.kill(pid,signal.SIGTERM)
                except ProcessLookupError:pass
        for log in logs:log.close()
        time.sleep(.2);rows=process_rows()
        remaining=[pid for pid,stamp in owned.items() if pid in rows and rows[pid][1]==stamp]
        report=dict(reason=reason,counts=dict(counts),duration_s=time.monotonic()-start,
            status_counts=dict(Counter(r['status'] for r in statuses)),
            maximum_tree_rss_mib=peak/1024,minimum_available_mib=minimum/1024,
            memory_floor_mib=FLOOR_MIB,owned_tree_limit_mib=TREE_MIB,
            native_map_size_m=[6.,6.,2.5],native_map_resolution_m=.05,peak_processes=peak_processes,
            scope='local stationary distributed interface check; not full-map navigation acceptance',
            actuator_publishers=pubs,can_tx_before=tx_before,can_tx_after=int(tx_path.read_text()),
            owned_processes_remaining=remaining,hardware_commands_sent=False,notes=events)
        report['recording_only']=args.recording
        report['final_gateway']=statuses[-1].get('command_gateway') if statuses else None
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        (out/'shadow_status.json').write_text(json.dumps(statuses,indent=2)+'\n')
        print(json.dumps(report),flush=True)
        if rospy is not None:rospy.signal_shutdown('bounded08 observation ended')
    return 0 if reason=='observation_complete' and not remaining and not pubs and int(tx_path.read_text())==tx_before else 1


if __name__=='__main__':
    sys.exit(main())
