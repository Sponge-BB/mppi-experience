#!/usr/bin/python3
"""Read installed SDK's documented parameters; 13 query frames only."""
import json
from pathlib import Path
import socket
import struct
import time

requests = [(0x4AF, bytes([1, 0, 0, 0, 0, 0, 0, 0]))]
requests += [(0x472, bytes([joint, kind, 0, 0, 0, 0, 0, 0]))
             for kind in (1, 2) for joint in range(1, 7)]
assert len(requests) == 13
assert all(i in (0x472, 0x4AF) and len(data) == 8 for i, data in requests)
sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                b''.join(struct.pack('=II', i, 0x800007FF)
                         for i in (0x2A1, 0x473, 0x47C, 0x4AF)))
sock.bind(('can1',))
sock.settimeout(.02)
sent = []
received = []
next_request = time.monotonic() + .20
deadline = time.monotonic() + 4.0
while time.monotonic() < deadline:
    if len(sent) < len(requests) and time.monotonic() >= next_request:
        can_id, data = requests[len(sent)]
        sock.send(struct.pack('=IB3x8s', can_id, len(data), data))
        sent.append({'time': time.time(), 'can_id': hex(can_id), 'data': data.hex()})
        next_request = time.monotonic() + .08
    try:
        raw = sock.recv(16)
    except socket.timeout:
        continue
    can_id, size, data = struct.unpack('=IB3x8s', raw)
    received.append({'time': time.time(), 'can_id': hex(can_id), 'data': data[:size].hex()})
sock.close()
report = {'scope': '13 query frames only; no SDK, enable, position, reset or configuration writes',
          'sent': sent, 'received': received}
out = Path(__file__).with_name('motor_parameter_query_%d.json' % time.time())
out.write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps({'output': str(out), 'sent_count': len(sent),
                  'parameters': [r for r in received if r['can_id'] != '0x2a1'],
                  'status_values': sorted(set(r['data'] for r in received if r['can_id'] == '0x2a1'))}))
