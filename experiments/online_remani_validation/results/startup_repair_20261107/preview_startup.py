#!/usr/bin/python3
"""Subscribe to current inputs and preview startup; no control publishers."""
import json
import math
from pathlib import Path
import sys
import threading
import time

import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

ROOT = Path(__file__).resolve().parents[4]
sys.path[:0] = [str(ROOT/'src/mm_remani/scripts'), str(ROOT/'src/mm_remani/scripts/mppi')]
from whole_body_mppi import startup_plan, checked_first_step
from active_arm_mppi import Snapshot
from active_arm_model import PiperArmModel
from standby_whole_body_mppi import StandbyWholeBodyMPPI

rospy.init_node('startup_readonly_preview', anonymous=True, disable_signals=True)
config = rospy.get_param('/whole_body_mppi')
params = rospy.get_param('/remani_planner_node')
model = PiperArmModel.from_parameters(params)
model.lower[5] = max(model.lower[5], -.35)
model.upper[5] = min(model.upper[5], .35)
kernel = StandbyWholeBodyMPPI(model, config['base_limits'],
    (model.lower, model.upper, config['arm_velocity'], config['arm_acceleration']),
    config['postures'], horizon=config['horizon'], step=1/config['rate_hz'],
    random_samples=config['random_samples'], config=config.get('standby', {}))
target = np.deg2rad(params['fsm']['init_state'][2:])
latest = {}; lock = threading.Lock(); headers = []

def receive(message, kind):
    with lock:
        if kind == 'snapshot':
            try:
                latest[kind] = Snapshot.decode(message.data, latest.get(kind))
                headers.append([rospy.get_time(), *latest[kind].header.tolist()])
            except ValueError as exc:
                latest['snapshot_decode_error'] = str(exc)
        else:
            latest[kind] = message

subscribers = [rospy.Subscriber(topic, cls, receive, callback_args=kind, queue_size=1,
                                buff_size=8*1024**2) for topic, cls, kind in [
    ('/mppi/collision_snapshot', numpy_msg(Float64MultiArray), 'snapshot'),
    ('/odom', Odometry, 'odom'), ('/joint_states_single', JointState, 'joints')]]
deadline = time.monotonic()+5
while time.monotonic() < deadline:
    with lock:
        ready = all(k in latest for k in ('snapshot', 'odom', 'joints')) and latest['snapshot'].fresh(rospy.get_time())
    if ready:
        break
    time.sleep(.01)
report = {'scope': 'Live-input read-only preview; no publishers, services or hardware control'}
with lock:
    try:
        now = rospy.get_time(); odom = latest['odom']; joints = latest['joints']; snapshot = latest['snapshot']
        ages = [now-odom.header.stamp.to_sec(), now-joints.header.stamp.to_sec(), now-snapshot.header[3]]
        if not snapshot.fresh(now) or not all(-.005 <= x < .15 for x in ages[:2]):
            raise ValueError('inputs_not_fresh')
        p, q, v = odom.pose.pose.position, odom.pose.pose.orientation, odom.twist.twist
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        ids = [joints.name.index('joint'+str(i)) for i in range(1, 7)]
        state = np.r_[[p.x,p.y,yaw,v.linear.x,v.angular.z],
                      [joints.position[i] for i in ids], [joints.velocity[i] for i in ids]]
        began = time.monotonic()
        result, info = startup_plan(kernel, state, target, snapshot, now, preflight=True)
        report.update(plan=info, input_ages_s=ages, current_q=state[5:11].tolist(), target_q=target.tolist())
        if result is not None:
            checked = checked_first_step(kernel, state, result[0][0], snapshot, now)
            report.update(checked_target=checked[5:11].tolist(), command=result[0][0].tolist())
        report['preview_ms'] = (time.monotonic()-began)*1000
    except (ValueError, KeyError) as exc:
        report['rejected'] = str(exc)
        report['snapshot_decode_error'] = latest.get('snapshot_decode_error')
        if 'snapshot' in latest:
            report['snapshot_header'] = latest['snapshot'].header.tolist()
report['sampled_snapshot_headers'] = headers
Path(__file__).with_suffix('.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
report.pop('sampled_snapshot_headers')
print(json.dumps(report, ensure_ascii=False))
rospy.signal_shutdown('preview complete')
