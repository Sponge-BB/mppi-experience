#!/usr/bin/python3
"""One-off passive CAN capture. No SDK, transmit, or interface configuration."""
import json
import os
from pathlib import Path
import signal
import socket
import struct
import time

out = Path(__file__).with_name('can_startup_%d.jsonl' % time.time())
running = True

def stop(*_):
    global running
    running = False

signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGINT, stop)
commands = set(range(0x150, 0x160)) | {0x471}
feedback = {0x2A1, 0x2A5, 0x2A6, 0x2A7}
sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                b''.join(struct.pack('=II', i, 0x800007FF)
                         for i in sorted(commands | feedback)))
sock.bind(('can1',))
sock.settimeout(.5)
last = {}
deadline = time.monotonic() + 600
with out.open('w', buffering=1) as log:
    print(json.dumps({'ready': True, 'pid': os.getpid(), 'output': str(out),
                      'mode': 'CAN recv only; no SDK, send or reconfiguration'}), flush=True)
    while running and time.monotonic() < deadline:
        try:
            raw = sock.recv(16)
        except socket.timeout:
            continue
        can_id, size, data = struct.unpack('=IB3x8s', raw)
        now = time.time()
        if can_id in feedback and now - last.get(can_id, 0) < .05:
            continue
        last[can_id] = now
        item = {'received': now, 'can_id': hex(can_id), 'data': data[:size].hex()}
        log.write(json.dumps(item) + '\n')
        if can_id in commands:
            print(json.dumps(item), flush=True)
sock.close()
