"""Isolated test master only; no robot launch or motion publishers."""
import os
import signal
import socket
import subprocess
import sys
import time
import xmlrpc.client
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
env = os.environ.copy()
env.pop('ROS_HOSTNAME', None)
env.update(ROS_MASTER_URI='http://127.0.0.1:11459', ROS_IP='127.0.0.1',
           ROS_LOG_DIR='/tmp/remani_map_compute_test_logs',
           ROS_HOME='/tmp/remani_map_compute_ros_home')
with socket.socket() as probe:
    probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    probe.bind(('127.0.0.1', 11459))  # Refuse an occupied port.
with (HERE / 'test_master.log').open('w') as master_log:
    master = subprocess.Popen(['/opt/ros/noetic/bin/roscore', '-p', '11459'],
        env=env, stdout=master_log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        for attempt in range(100):
            if master.poll() is not None:
                raise RuntimeError('Isolated master exited')
            try:
                if xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/map_compute_test')[0] == 1:
                    break
            except OSError:
                pass
            time.sleep(.1)
        else:
            raise RuntimeError('Isolated master did not become ready')
        commands = [
            ('map_tests', 'after_test', HERE, []),
            ('benchmark_before', 'before_test', HERE / 'before',
             ['--gtest_filter=GridMapBenchmark.Synthetic300x200x60AsyncThreeThreads']),
            ('benchmark_after', 'after_test', HERE,
             ['--gtest_filter=GridMapBenchmark.Synthetic300x200x60AsyncThreeThreads']),
        ]
        if '--benchmark-only' in sys.argv:
            before, after = commands[-2:]
            commands = [(item[0] + '_repeat' + str(index), *item[1:])
                        for index, order in [(2, (after, before)), (3, (before, after))]
                        for item in order]
        for name, executable, library, arguments in commands:
            test_env = env.copy()
            test_env['LD_LIBRARY_PATH'] = str(library) + ':' + env.get('LD_LIBRARY_PATH', '')
            if name.startswith('benchmark_'):
                test_env['REMANI_MAP_BENCHMARK'] = '1'
            else:
                test_env.pop('REMANI_MAP_BENCHMARK', None)
            print('Running ' + name + ' on isolated localhost master', flush=True)
            with (HERE / (name + '.log')).open('w') as log:
                started = time.monotonic()
                child = subprocess.Popen([str(HERE / executable)] + arguments, env=test_env,
                    stdout=log, stderr=subprocess.STDOUT)
                peak_rss = 0
                while child.poll() is None:
                    try:
                        for line in Path('/proc/' + str(child.pid) + '/status').read_text().splitlines():
                            if line.startswith('VmHWM:'):
                                peak_rss = max(peak_rss, int(line.split()[1]))
                    except FileNotFoundError:
                        pass
                    if time.monotonic() - started > 180:
                        child.kill()
                        child.wait()
                        raise RuntimeError(name + ' timed out')
                    time.sleep(.05)
                log.write('\n[map-check-resources] sampled_peak_rss_kib=%d elapsed_s=%.3f\n' %
                          (peak_rss, time.monotonic() - started))
                if child.returncode:
                    raise RuntimeError(name + ' failed; see its log')
            print(name + ' passed', flush=True)
    finally:
        os.killpg(master.pid, signal.SIGTERM)
        try:
            master.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(master.pid, signal.SIGKILL)
            master.wait()
