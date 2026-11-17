#!/usr/bin/python3
"""One bounded stationary observation; starts no actuator node or SDK."""
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
sys.path.insert(0, str(RELEASE/'runtime'))
MEMORY_FLOOR_MIB = 2816
TREE_LIMIT_MIB = 650


def memory_available():
    return int(next(line.split()[1] for line in Path('/proc/meminfo').read_text().splitlines()
                    if line.startswith('MemAvailable:')))


def process_rows():
    rows = {}
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():
            continue
        try:
            fields = (p/'stat').read_text().rsplit(')', 1)[1].split()
            rss = int(next((line.split()[1] for line in (p/'status').read_text().splitlines()
                            if line.startswith('VmRSS:')), '0'))
            rows[int(p.name)] = (int(fields[1]), fields[19], rss)
        except (OSError, ValueError):
            pass
    return rows


def actuator_publishers(master):
    topics = master.getSystemState('/stationary_preparation_audit')[2][0]
    suffixes = ('/cmd_vel', '/car_cmd', '/joint_ctrl_single', '/leased_command', '/enable_flag', '/command_lease')
    return [(name, pubs) for name, pubs in topics if name.endswith(suffixes) and pubs]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=30.)
    parser.add_argument('--label', default='stationary_01')
    parser.add_argument('--recording', action='store_true',
                        help='Exercise the command gateway with an in-memory RecordingSink only')
    args = parser.parse_args()
    if not 15 <= args.duration <= 90 or not args.label.replace('_', '').isalnum():
        raise ValueError('bounded duration 15..90 and simple label required')
    if memory_available() < (MEMORY_FLOOR_MIB+TREE_LIMIT_MIB)*1024:
        raise RuntimeError('Insufficient memory for the reserved floor plus the entire trial budget')
    if os.environ.get('ROS_MASTER_URI') != 'http://127.0.0.1:11461':
        raise RuntimeError('Private existing master 11461 required')
    socket.setdefaulttimeout(3.)
    master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
    if actuator_publishers(master):
        raise RuntimeError('Existing actuator publishers; observation not started')
    published = dict(master.getSystemState('/stationary_preparation_audit')[2][0])
    if published.get('/standby_trial_20261105_feedback_02/sensors/odom'):
        raise RuntimeError('Existing odometry publisher; refuse duplicate localization')
    if not published.get('/standby_trial_20261105_feedback_02/sensors/lidar_pcl2'):
        raise RuntimeError('Existing raw LiDAR input required')
    out = RELEASE/'reports'/args.label; out.mkdir(exist_ok=False)
    can_tx = Path('/sys/class/net/can0/statistics/tx_packets')
    tx_before = int(can_tx.read_text())
    import numpy as np
    import rospy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState, PointCloud2
    from std_msgs.msg import String, Bool
    from pointcloud_numpy import read_points_numpy
    rospy.init_node('stationary_preparation_probe', anonymous=True, disable_signals=True)
    prefix = '/standby_trial_20261106_stationary_07'
    sensor = '/standby_trial_20261105_feedback_02/sensors'
    counts = Counter(); statuses = []; odometry = []; clouds = {}; joint = {}; health = []
    recording_enable = (rospy.Publisher(prefix+'/gateway/enable', Bool, queue_size=1)
                        if args.recording else None)

    def status(msg):
        counts['status'] += 1
        value = json.loads(msg.data); statuses.append(value)
        if len(statuses) > 1200:
            del statuses[0]

    def odom(msg):
        counts['odom'] += 1
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        odometry.append([msg.header.stamp.to_sec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                         msg.twist.twist.linear.x, msg.twist.twist.angular.z,
                         rospy.get_time()-msg.header.stamp.to_sec()])

    def joints(msg):
        counts['joints'] += 1
        joint.update(stamp=msg.header.stamp.to_sec(), q=list(msg.position[:6]))

    def cloud(msg, key):
        counts[key] += 1
        points = np.asarray(read_points_numpy(msg, ('x','y','z')), dtype=np.float32)
        if len(points) > 20000:
            points = points[np.linspace(0, len(points)-1, 20000, dtype=int)]
        clouds[key] = (points.copy(), msg.header.stamp.to_sec(), msg.header.frame_id)

    def on_health(msg):
        counts['healthy' if msg.data else 'unhealthy'] += 1
        health.append((rospy.get_time(), bool(msg.data)))

    subs = [rospy.Subscriber(prefix+'/status', String, status, queue_size=1),
            rospy.Subscriber(sensor+'/odom', Odometry, odom, queue_size=1),
            rospy.Subscriber(sensor+'/joint_states', JointState, joints, queue_size=1),
            rospy.Subscriber(prefix+'/perception/dynamic_obstacle_tracker/perception_healthy', Bool, on_health, queue_size=1)]
    for key, topic in [('raw',sensor+'/lidar_pcl2'), ('filtered',prefix+'/perception/filtered_points'),
                       ('background',prefix+'/perception/background')]:
        subs.append(rospy.Subscriber(topic, PointCloud2, lambda msg, k=key: cloud(msg,k), queue_size=1,
                                     buff_size=4*1024**2))
    children = []; owned = {}; logs = []; peak = 0; min_available = memory_available()
    reason = 'observation_complete'; start = time.monotonic()

    def limits():
        resource.setrlimit(resource.RLIMIT_AS, (768*1024**2, 768*1024**2))
        resource.setrlimit(resource.RLIMIT_CORE, (0,0))

    def launch(name, cmd):
        log = (out/(name+'.log')).open('w'); logs.append(log)
        child = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, env=os.environ.copy(),
                                 preexec_fn=limits, start_new_session=True)
        children.append((name, child))

    try:
        launch('receive_only_can', ['/usr/bin/python3',str(FEEDBACK/'runtime/piper_can_receive_only.py'),
                                    '--interface','can0','--duration',str(args.duration+10.)])
        launch('odometry', ['/opt/ros/noetic/bin/roslaunch',str(FEEDBACK/'sensors/sensor_odometry.launch')])
        perception_started = False; last_print = -10.; recording_requested = False
        while time.monotonic()-start < args.duration:
            elapsed = time.monotonic()-start
            if elapsed >= 3. and not perception_started:
                launch('perception', ['/opt/ros/noetic/bin/roslaunch',str(RELEASE/'config/stationary_perception.launch')])
                if args.recording:
                    launch('shadow', ['/usr/bin/python3',str(RELEASE/'runtime/standby_integration_node.py'),
                                      '_execute:=false', '_log_dir:='+str(out)])
                else:
                    launch('shadow', ['/usr/bin/python3',str(RELEASE/'runtime/hardware_shadow_node.py'),
                                      '--release-dir',str(RELEASE),'--namespace',prefix,
                                      '_log_dir:='+str(out)])
                perception_started = True
            if args.recording and statuses and not recording_requested:
                latest = statuses[-1]
                if latest.get('mode') != 'integration_recording' or latest.get('commands_executed'):
                    raise RuntimeError('Recording-only mode required')
                if latest.get('proposed_controls') is not None and recording_enable.get_num_connections():
                    # Software recording gate only. No actuator driver is
                    # started, and execute=false constructs no RosCommandSink.
                    recording_enable.publish(Bool(data=True))
                    recording_requested = True
            rows = process_rows(); selected = {child.pid for _,child in children}
            while True:
                more = {pid for pid,(parent,_,_) in rows.items() if parent in selected}
                if more <= selected:
                    break
                selected.update(more)
            for pid in selected:
                if pid in rows:
                    owned[pid] = rows[pid][1]
            rss = sum(rows[p][2] for p in selected if p in rows)
            peak = max(peak,rss); available = memory_available(); min_available = min(min_available,available)
            if available < MEMORY_FLOOR_MIB*1024 or rss > TREE_LIMIT_MIB*1024:
                reason = 'memory_threshold_reached'; break
            if any(child.poll() is not None for _,child in children):
                reason = 'owned_process_exited'; break
            if elapsed-last_print >= 5.:
                print(json.dumps({'elapsed_s':round(elapsed,1),'tree_rss_mib':round(rss/1024,1),
                                  'available_mib':round(available/1024,1),'counts':dict(counts),
                                  'last_status':statuses[-1].get('status') if statuses else None}),flush=True)
                last_print = elapsed
            time.sleep(.5)
    finally:
        pubs = actuator_publishers(master)
        for _,child in children:
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
        deadline = time.monotonic()+6.
        while any(child.poll() is None for _,child in children) and time.monotonic()<deadline:
            time.sleep(.1)
        rows = process_rows()
        for pid,stamp in owned.items():
            if pid in rows and rows[pid][1]==stamp:
                try: os.kill(pid,signal.SIGTERM)
                except ProcessLookupError: pass
        for log in logs: log.close()
        time.sleep(.2)
        rows = process_rows()
        remaining = [pid for pid,stamp in owned.items() if pid in rows and rows[pid][1]==stamp]
        snapshot = {key:value[0] for key,value in clouds.items()}
        snapshot.update(joints=np.asarray(joint.get('q',[])), odometry=np.asarray(odometry))
        np.savez_compressed(out/'snapshot.npz',**snapshot)
        (out/'shadow_status.json').write_text(json.dumps(statuses,indent=2)+'\n')
        result = {'scope':'actual stationary shadow only; no actuator nodes or commands',
                  'reason':reason,'duration_s':time.monotonic()-start,'counts':dict(counts),
                  'status_counts':dict(Counter(s.get('status') for s in statuses)),
                  'last_status':statuses[-1] if statuses else None,
                  'clouds':{k:{'count':len(v[0]),'stamp':v[1],'frame':v[2]} for k,v in clouds.items()},
                  'joints':joint,'maximum_tree_rss_mib':peak/1024,'minimum_available_mib':min_available/1024,
                  'per_process_address_space_limit_mib':768,'memory_floor_mib':MEMORY_FLOOR_MIB,
                  'owned_tree_limit_mib':TREE_LIMIT_MIB,
                  'can_tx_before':tx_before,'can_tx_after':int(can_tx.read_text()),
                  'actuator_publishers':pubs,'owned_processes_remaining':remaining,
                  'hardware_commands_sent':False, 'recording_only':args.recording}
        (out/'report.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({k:result[k] for k in ('reason','counts','status_counts','maximum_tree_rss_mib',
                                               'minimum_available_mib','can_tx_after','owned_processes_remaining')}),flush=True)
        rospy.signal_shutdown('bounded observation finished')
    return 0 if reason=='observation_complete' and not remaining and not pubs and int(can_tx.read_text())==tx_before else 1


if __name__ == '__main__':
    sys.exit(main())
