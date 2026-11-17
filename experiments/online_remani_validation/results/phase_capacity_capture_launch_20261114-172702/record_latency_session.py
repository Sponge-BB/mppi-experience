#!/usr/bin/python3
"""Bounded small-message capture; only ROS logger levels are temporarily changed.

No control, goal, enable or reset publishers. Waits for the user's normal launch.
Full clouds/maps are deliberately excluded so the navigation tail is retained.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import threading
import time
import xmlrpc.client

import rosbag
import roslib.message
import rospy
from roscpp.srv import GetLoggers, SetLoggerLevel

TOPICS = ('/mppi/status', '/cmd_vel', '/odom', '/move_base_simple/goal',
          '/planning/trajectory', '/planning/finish', '/planning/safety_speed_limit',
          '/planning/mppi_refresh_request',
          '/piper_execution_state', '/tracking_fault', '/rosout_agg',
          '/joint_states_single', '/mm/joint_states',
          '/mm_controller_node/trajectory_progress')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output'); parser.add_argument('--duration', type=float, default=900.)
    parser.add_argument('--after-goal', type=float, default=120.)
    parser.add_argument('--max-mib', type=float, default=96.)
    args = parser.parse_args()
    output = Path(args.output); output.mkdir(parents=True, exist_ok=False)
    began = time.monotonic(); counts = {}; first_goal = None; payload_bytes = 0
    stop = threading.Event(); lock = threading.Lock(); logger_levels = {}
    logger_rpc = dict(unavailable=False, error='', restoration={})
    evidence = dict(map_trace=False, export_trace=False, mppi_gate=False)
    phase = 'waiting_for_master'; reason = ''; classes = {}
    def save():
        with lock:
            value = dict(pid=os.getpid(), phase=phase, elapsed_s=round(time.monotonic()-began, 2),
                         counts=dict(counts), evidence=dict(evidence), reason=reason,
                         payload_bytes=payload_bytes, first_goal=first_goal,
                         logger_levels=dict(logger_levels), logger_rpc=dict(logger_rpc), topics=list(TOPICS),
                         duration_limit_s=args.duration, after_goal_limit_s=args.after_goal,
                         payload_limit_mib=args.max_mib, control_publications=False)
        temp=output/'status.tmp'; temp.write_text(json.dumps(value, indent=2)+'\n')
        temp.replace(output/'status.json')
    def logger_call(service, service_type, *values):
        result = []
        def invoke():
            try:
                rospy.wait_for_service(service, timeout=.15)
                result.append((True, rospy.ServiceProxy(service, service_type)(*values)))
            except Exception as exc:
                result.append((False, exc))
        worker = threading.Thread(target=invoke, daemon=True)
        worker.start(); worker.join(.75)
        if worker.is_alive():
            logger_rpc['unavailable'] = True
            logger_rpc['error'] = 'RPC timed out; logger state unconfirmed: '+service
            raise TimeoutError(logger_rpc['error'])
        success, value = result[0]
        if not success: raise value
        return value
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stop.set())
    # XMLRPC must not stall indefinitely when the remote master is not yet up.
    socket.setdefaulttimeout(2.)
    master = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
    while not stop.is_set() and time.monotonic()-began < args.duration:
        save()
        try:
            if xmlrpc.client.ServerProxy(master).getPid('/latency_capture_ready')[0] == 1:
                break
        except (OSError, xmlrpc.client.Error):
            pass
        stop.wait(1.)
    else:
        reason='master_wait_cancelled_or_expired'; phase='closed'; save(); return
    socket.setdefaulttimeout(None)
    rospy.init_node('remani_latency_capture', anonymous=True, disable_signals=True, disable_rosout=True)
    bag = rosbag.Bag(str(output/'latency.bag'), 'w', chunk_threshold=256*1024)
    phase='waiting_for_instrumented_nodes'
    def callback(message, topic):
        nonlocal payload_bytes, first_goal, phase, reason
        received=rospy.Time.now()
        with lock:
            if stop.is_set(): return
            if payload_bytes+len(message._buff) > args.max_mib*1024**2:
                reason='payload_budget'; stop.set(); return
            try:
                name=message._connection_header['type']
                cls=classes.get(name)
                if cls is None:
                    cls=roslib.message.get_message_class(name)
                    if cls is None: raise ValueError('message class unavailable: '+name)
                    classes[name]=cls
                decoded=cls().deserialize(message._buff)
                bag.write(topic, decoded, received, connection_header=message._connection_header)
                payload_bytes += len(message._buff); counts[topic]=counts.get(topic,0)+1
                if topic=='/move_base_simple/goal' and first_goal is None:
                    first_goal=time.monotonic(); phase='capturing_navigation'
                if topic=='/rosout_agg':
                    evidence['map_trace'] |= '[MapTrace]' in decoded.msg
                    evidence['export_trace'] |= '[LatencyTrace]' in decoded.msg
                if topic=='/mppi/status':
                    d=json.loads(decoded.data).get('candidate_diagnostics', {})
                    evidence['mppi_gate'] |= 'checked_inputs' in d or 'capture_inputs' in d
                if all(evidence.values()) and first_goal is None:
                    phase='ready_for_user_goal'
            except Exception as exc:
                reason='record_error: '+str(exc); stop.set()
    subscribers=[rospy.Subscriber(t, rospy.AnyMsg, callback, callback_args=t,
                                  queue_size=256, buff_size=256*1024) for t in TOPICS]
    last_logger_check=0.
    try:
        while not stop.is_set() and not rospy.is_shutdown():
            now=time.monotonic()
            if now-began >= args.duration:
                reason='duration_limit'; break
            if first_goal is not None and now-first_goal >= args.after_goal:
                reason='navigation_window_complete'; break
            if now-last_logger_check >= 2. and not logger_rpc['unavailable']:
                last_logger_check=now
                try:
                    loggers=logger_call('/remani_planner_node/get_loggers', GetLoggers).loggers
                    for logger in loggers:
                        if logger.name.endswith('.latency'):
                            logger_levels.setdefault(logger.name, logger.level)
                            if logger.level.lower() != 'debug':
                                logger_call('/remani_planner_node/set_logger_level', SetLoggerLevel,
                                            logger.name, 'debug')
                except (rospy.ROSException, rospy.ServiceException, TimeoutError):
                    pass
            save(); stop.wait(.5)
    finally:
        stop.set()
        for subscriber in subscribers: subscriber.unregister()
        with lock: bag.close()
        for name, level in logger_levels.items():
            try:
                logger_call('/remani_planner_node/set_logger_level', SetLoggerLevel, name, level)
                logger_rpc['restoration'][name] = ('unconfirmed_after_rpc_timeout'
                    if logger_rpc['unavailable'] else 'confirmed')
            except (rospy.ROSException, rospy.ServiceException, TimeoutError) as exc:
                logger_rpc['restoration'][name] = 'unconfirmed: '+str(exc)
        phase='closed'; reason=reason or 'requested_stop'; save()
        rospy.signal_shutdown(reason)


if __name__=='__main__':
    main()
