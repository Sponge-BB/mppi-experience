#!/usr/bin/python3
"""Recorder smoke on a dedicated loopback ROS graph; no hardware master contact."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import xmlrpc.client

CASE = Path(__file__).resolve().parent
URI = 'http://127.0.0.1:11471'
os.environ.update(ROS_MASTER_URI=URI, ROS_IP='127.0.0.1', ROS_HOME=str(CASE/'smoke_ros_home'),
                  ROS_LOG_DIR=str(CASE/'smoke_ros_logs'), OPENBLAS_NUM_THREADS='1',
                  OMP_NUM_THREADS='1', PYTHONDONTWRITEBYTECODE='1')
os.environ.pop('ROS_HOSTNAME', None)

import rosbag
import rospy
from geometry_msgs.msg import PoseStamped
from roscpp.msg import Logger
from roscpp.srv import GetLoggers, GetLoggersResponse, SetLoggerLevel, SetLoggerLevelResponse
from rosgraph_msgs.msg import Log
from std_msgs.msg import String

class TimedTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 1.
        return connection

def main():
    processes = []; streams = []; result = dict(master=URI, passed=False)
    capture = CASE/('recorder_smoke_capture_'+time.strftime('%H%M%S'))
    try:
        with socket.socket() as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            probe.bind(('127.0.0.1', 11471))
        def start(argv, logfile):
            stream = open(CASE/logfile, 'w'); streams.append(stream)
            proc = subprocess.Popen(argv, stdout=stream, stderr=subprocess.STDOUT,
                                    env=os.environ.copy(), start_new_session=True)
            processes.append(proc); return proc
        core = start(['roscore', '-p', '11471'], 'recorder_smoke_roscore.log')
        master = xmlrpc.client.ServerProxy(URI, transport=TimedTransport())
        deadline = time.monotonic()+8.
        while True:
            try:
                assert master.getPid('/isolated_smoke')[0] == 1
                break
            except (OSError, xmlrpc.client.Error):
                if core.poll() is not None or time.monotonic()>deadline: raise
                time.sleep(.1)
        rospy.init_node('remani_planner_node', disable_signals=True, disable_rosout=True)
        from rospy.impl.registration import get_service_manager
        for service in ('get_loggers', 'set_logger_level'):
            get_service_manager().get_service('/remani_planner_node/'+service).shutdown('replace isolated mock')
        levels = {'ros.plan_env.latency': 'INFO', 'ros.remani_planner.latency': 'INFO'}
        calls = []
        def get_loggers(_request):
            return GetLoggersResponse(loggers=[Logger(name=k, level=v) for k,v in levels.items()])
        def set_logger(request):
            levels[request.logger] = request.level.upper()
            calls.append(dict(at=time.monotonic(), name=request.logger, level=request.level.upper()))
            return SetLoggerLevelResponse()
        services = [rospy.Service('/remani_planner_node/get_loggers', GetLoggers, get_loggers),
                    rospy.Service('/remani_planner_node/set_logger_level', SetLoggerLevel, set_logger)]
        log_pub = rospy.Publisher('/rosout_agg', Log, queue_size=8)
        status_pub = rospy.Publisher('/mppi/status', String, queue_size=8)
        # Synthetic goal only exists on this newly created loopback graph.
        goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        recorder = start(['/usr/bin/python3', '/home/l/remani_ws/experiments/online_remani_validation/record_latency_session.py',
                          str(capture), '--duration', '20', '--after-goal', '1.5', '--max-mib', '2'],
                         'recorder_smoke_recorder.log')
        started = time.monotonic(); reset_at = None; sent_at = None; ready_seen = False
        while recorder.poll() is None and time.monotonic()-started < 18.:
            if all(v == 'DEBUG' for v in levels.values()):
                for prefix in ('[MapTrace]', '[LatencyTrace]'):
                    message=Log(name='/remani_planner_node', level=Log.DEBUG,
                                msg=prefix+' stage=isolated_smoke obs=1')
                    message.header.stamp=rospy.Time.now(); log_pub.publish(message)
            status_pub.publish(String(data=json.dumps(dict(state='candidate_only',
                candidate_diagnostics=dict(checked_inputs=dict(stage='capture', snapshot_valid=True))))))
            status_path=capture/'status.json'
            status=json.loads(status_path.read_text()) if status_path.exists() else {}
            ready_seen |= status.get('phase') == 'ready_for_user_goal'
            if ready_seen and reset_at is None:
                for name in levels: levels[name] = 'INFO'
                reset_at=time.monotonic()
            elif reset_at is not None and sent_at is None and all(v=='DEBUG' for v in levels.values()):
                assert goal_pub.get_num_connections() == 1
                graph=master.getSystemState('/isolated_smoke')[2]
                nodes=sorted({n for section in graph for _topic, names in section for n in names})
                assert all(n in ('/rosout', '/remani_planner_node') or n.startswith('/remani_latency_capture_') for n in nodes), nodes
                result['graph_nodes']=nodes
                result['graph_publishers']={name: publishers for name,publishers in graph[0]}
                assert '/cmd_vel' not in result['graph_publishers']
                assert '/joint_ctrl_single' not in result['graph_publishers']
                goal=PoseStamped(); goal.header.stamp=rospy.Time.now(); goal.header.frame_id='world'
                goal.pose.orientation.w=1.; goal_pub.publish(goal); sent_at=time.monotonic()
            time.sleep(.05)
        assert recorder.poll() == 0, 'recorder did not exit normally within isolated test deadline'
        status=json.loads((capture/'status.json').read_text())
        assert ready_seen and all(status['evidence'].values()), status
        assert status['reason']=='navigation_window_complete' and status['phase']=='closed', status
        assert 1.4 <= time.monotonic()-sent_at <= 4., 'after-goal cutoff failed'
        assert all(v=='INFO' for v in levels.values()), levels
        assert all(v=='confirmed' for v in status['logger_rpc']['restoration'].values()), status
        assert sum(c['level']=='DEBUG' for c in calls) == 4, calls
        with rosbag.Bag(str(capture/'latency.bag'),'r') as bag:
            info=bag.get_type_and_topic_info()
            assert all(t in info.topics for t in ('/rosout_agg','/mppi/status','/move_base_simple/goal'))
            result['bag_counts']={k:v.message_count for k,v in info.topics.items()}
            result['bag_messages']=bag.get_message_count()
        result.update(passed=True, ready_seen=ready_seen, logger_reset_reenabled=True,
                      logger_calls=calls, final_levels=levels, recorder_status=status,
                      capture=str(capture), publications='Only synthetic logs/status/goal on loopback isolated graph')
    except Exception as error:
        result['error']=repr(error)
        raise
    finally:
        if rospy.core.is_initialized(): rospy.signal_shutdown('isolated smoke finished')
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try: proc.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL); proc.wait(timeout=3.)
        for stream in streams: stream.close()
        result['process_exit_codes']=[p.returncode for p in processes]
        (CASE/'recorder_smoke_result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(passed=result['passed'], capture=str(capture), bag_messages=result['bag_messages'])))

if __name__=='__main__': main()
