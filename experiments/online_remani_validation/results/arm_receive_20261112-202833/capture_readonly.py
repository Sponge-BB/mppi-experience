#!/usr/bin/python3
"""One-run passive CAN/ROS/process capture. Never sends CAN or ROS control."""
import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

ROOT = Path('/home/l/remani_ws')
out = ROOT/'experiments/online_remani_validation/results'/(
    'arm_receive_'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
out.mkdir(exist_ok=False)
shutil.copyfile(__file__, out/'capture_readonly.py')
halt = threading.Event()
for number in (signal.SIGINT, signal.SIGTERM):
    signal.signal(number, lambda *_: halt.set())
ids = [0x150, 0x151, 0x155, 0x156, 0x157, 0x2A1,
       0x2A5, 0x2A6, 0x2A7, *range(0x261, 0x267)]
sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                b''.join(struct.pack('=II', i, 0xE00007FF) for i in ids))
sock.setsockopt(socket.SOL_SOCKET, 35, 1)  # SO_TIMESTAMPNS on this Linux host.
sock.setsockopt(socket.SOL_SOCKET, 40, 1)  # SO_RXQ_OVFL, this socket only.
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)
sock.settimeout(.1)
sock.bind(('can1',))
topics = ['/joint_states_single', '/arm_status', '/piper_motor_enable_states',
          '/piper_execution_state', '/joint_ctrl_single', '/mppi/status',
          '/rosout_agg', '/move_base_simple/goal']
roslog = (out/'rosbag.log').open('w')
bag = subprocess.Popen(['/opt/ros/noetic/bin/rosbag', 'record', '--quiet',
    '--buffsize=8', '--chunksize=256', '--duration=600', '-O', str(out/'feedback.bag'),
    *topics], stdout=roslog, stderr=subprocess.STDOUT, start_new_session=True)
began = time.monotonic()
status = {'output': str(out), 'pid': os.getpid(), 'rosbag_pid': bag.pid,
          'started_wall': time.time(), 'can_frames': 0, 'can_socket_drops': 0,
          'ids': [hex(i) for i in ids], 'can_tx_calls': 0,
          'format': '=QQQIB3x8s: kernel_ns, read_wall_ns, read_monotonic_ns, id, dlc, data',
          'record_bytes': struct.calcsize('=QQQIB3x8s'),
          'limit': '600s/64MiB CAN; small ROS topics; no point clouds; no driver edits',
          'thread_caveat': 'All driver TIDs; names do not identify SDK receive thread.'}
errors = []
sources = ['src/piper_ros/src/piper/scripts/piper_ctrl_single_node.py',
           'src/piper_ros/src/piper/scripts/piper_feedback_dispatch.py',
           'src/mm_remani/launch/exp0_real.launch', 'rollback/VERSIONS.tsv']
status['source_sha256'] = {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sources}

def sample_processes():
    tracked = []
    discover_at = 0.
    with (out/'threads.jsonl').open('w', buffering=65536) as stream:
        while not halt.wait(.2):
            now = time.monotonic()
            if now >= discover_at:
                tracked = []
                for entry in Path('/proc').iterdir():
                    if not entry.name.isdigit():
                        continue
                    try:
                        argv = (entry/'cmdline').read_bytes().split(b'\0')
                        if any(x.endswith(b'/piper_ctrl_single_node.py') for x in argv[:3]):
                            tracked.append(entry)
                    except (OSError, ProcessLookupError):
                        pass
                discover_at = now+1.
            row = {'wall': time.time(), 'monotonic': now,
                   'clock_offset': time.time()-time.monotonic(), 'drivers': []}
            for proc in tracked:
                try:
                    item = {'pid': int(proc.name), 'argv': (proc/'cmdline').read_bytes().replace(b'\0', b' ').decode(), 'threads': []}
                    for task in (proc/'task').iterdir():
                        try:
                            stat = (task/'stat').read_text()
                            item['threads'].append({'tid': int(task.name),
                                'state': stat[stat.rfind(')')+2:].split()[0],
                                'schedstat': [int(v) for v in (task/'schedstat').read_text().split()],
                                'wchan': (task/'wchan').read_text().strip()})
                        except (OSError, ProcessLookupError):
                            pass
                    row['drivers'].append(item)
                except (OSError, ProcessLookupError):
                    pass
            stream.write(json.dumps(row, separators=(',', ':'))+'\n')
            stream.flush()
            if bag.poll() is not None:
                errors.append('rosbag_exited:'+str(bag.returncode))
                halt.set()
        stream.flush()

thread = threading.Thread(target=sample_processes, daemon=True)
thread.start()
print('READY '+json.dumps(status, ensure_ascii=False), flush=True)
next_status = began
reason = 'interrupted'
try:
    with (out/'can_frames.bin').open('wb', buffering=262144) as stream:
        while not halt.is_set():
            now = time.monotonic()
            if now-began > 600:
                reason = '600s_limit'
                break
            if status['can_frames']*status['record_bytes'] >= 64*1024**2:
                reason = 'CAN_64MiB_limit'
                break
            try:
                frame, ancillary, flags, address = sock.recvmsg(16, 128)
                wall = time.time_ns()
                steady = time.monotonic_ns()
            except socket.timeout:
                continue
            kernel = None
            for level, kind, payload in ancillary:
                if level == socket.SOL_SOCKET and kind == 35:
                    sec, nsec = struct.unpack_from('@ll', payload)
                    kernel = sec*1000000000+nsec
                elif level == socket.SOL_SOCKET and kind == 40:
                    status['can_socket_drops'] = struct.unpack_from('=I', payload)[0]
            if kernel is None or len(frame) != 16 or flags & socket.MSG_CTRUNC:
                errors.append('missing_or_truncated_kernel_timestamp_or_CAN_frame')
                halt.set()
                break
            can_id, dlc, data = struct.unpack('=IB3x8s', frame)
            stream.write(struct.pack('=QQQIB3x8s', kernel, wall, steady, can_id, dlc, data))
            status['can_frames'] += 1
            if now >= next_status:
                stream.flush()
                status.update(elapsed_s=now-began, running=True, errors=list(errors))
                (out/'status.json').write_text(json.dumps(status, indent=2)+'\n')
                next_status = now+1.
except BaseException as exc:
    errors.append(type(exc).__name__+': '+str(exc))
    reason = 'capture_error'
finally:
    halt.set()
    thread.join(timeout=2.)
    sock.close()
    if bag.poll() is None:
        bag.send_signal(signal.SIGINT)
        try:
            bag.wait(timeout=12.)
        except subprocess.TimeoutExpired:
            errors.append('rosbag_close_timeout_not_killed')
    roslog.close()
    status.update(elapsed_s=time.monotonic()-began, running=False, reason=reason,
                  errors=errors, rosbag_exit=bag.poll())
    (out/'summary.json').write_text(json.dumps(status, ensure_ascii=False, indent=2)+'\n')
    print('SAVED '+str(out), flush=True)
