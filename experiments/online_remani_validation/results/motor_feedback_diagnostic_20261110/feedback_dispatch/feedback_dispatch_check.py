#!/usr/bin/env python3
"""Offline SDK replay only: no interface init, socket, thread start, or ROS."""
import ast
import enum
import hashlib
import inspect
import json
from pathlib import Path
import socket
import statistics
import struct
import sys
import threading
import time

import can
import piper_sdk.interface.piper_interface as sdk_module

HERE = Path(__file__).resolve().parent
BASE = HERE.parent
ROOT = BASE.parents[3]
MODULE = ROOT / 'src/piper_ros/src/piper/scripts/piper_feedback_dispatch.py'
sys.path.insert(0, str(MODULE.parent))
from piper_feedback_dispatch import FeedbackDispatch


def forbidden(*args, **kwargs):
    raise AssertionError('Offline check attempted socket/CAN/thread access')


socket.socket = forbidden
threading.Thread.start = forbidden
sdk_module.C_STD_CAN = forbidden
SDK = sdk_module.C_PiperInterface
source = Path(inspect.getfile(SDK))
tree = ast.parse(source.read_text())
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'C_PiperInterface')
init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
# Only the original pure-memory initializer after the parser assignment. Never
# execute SDK __new__/__init__, transport construction, logging, or ConnectPort.
first = next(i for i, n in enumerate(init.body) if isinstance(n, ast.AnnAssign)
             and isinstance(n.target, ast.Attribute) and n.target.attr == '__parser')
init.body = init.body[first:]
cls.body = [init]
namespace = dict(vars(sdk_module))
exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])),
             str(source), 'exec'), namespace)
memory_init = namespace['C_PiperInterface'].__init__


def make_sdk():
    sdk = object.__new__(SDK)
    memory_init(sdk)
    sdk._C_PiperInterface__start_sdk_joint_limit = False
    sdk._C_PiperInterface__start_sdk_gripper_limit = False
    sdk._C_PiperInterface__start_sdk_fk_cal = False
    sdk._C_PiperInterface__abnormal_data_filter = True
    sdk._C_PiperInterface__piper_fk = sdk_module.C_PiperForwardKinematics(1)
    return sdk


def plain(value):
    if isinstance(value, enum.Enum):
        return value.value
    if isinstance(value, (str, int, float, bool, type(None))):
        return value
    if isinstance(value, (bytes, bytearray)):
        return bytes(value).hex()
    if isinstance(value, (tuple, list)):
        return [plain(v) for v in value]
    if isinstance(value, dict):
        return {str(k): plain(v) for k, v in value.items()}
    return plain(vars(value))


def snapshot(sdk):
    # Every SDK data wrapper, including less-frequent parameter/control replies.
    result = {k: plain(v) for k, v in vars(sdk).items()
              if hasattr(v, 'time_stamp') or k.endswith(('__firmware_data', '__link_feedback_fk', '__link_ctrl_fk'))}
    fps = sdk._C_PiperInterface__fps_counter
    result['frame_counts'] = dict(fps.fps_data)
    # FPS last_time is parser execution time, necessarily different per replay;
    # source timestamps and every data-wrapper field are compared exactly.
    return result


def pair():
    old, new = make_sdk(), make_sdk()
    return old, new, FeedbackDispatch(new)


def compare_frames(frames, old, new, fast):
    for index, frame in enumerate(frames):
        old.ParseCANFrame(frame)
        fast(frame)
        a, b = snapshot(old), snapshot(new)
        if a != b:
            raise AssertionError('frame %d ID %s differs: %s' % (
                index, hex(frame.arbitration_id), [k for k in a if a[k] != b[k]]))


raw = json.loads((BASE / 'feedback_frames.json').read_text())
frames = [can.Message(arbitration_id=r['arbitration_id'], is_extended_id=r['is_extended_id'],
                      data=bytes.fromhex(r['data']), timestamp=1000. + i / 3040.)
          for i, r in enumerate(raw)]
old, new, fast = pair()
compare_frames(frames, old, new, fast)
checks = {'recorded_frames_all_state_fields': len(frames)}

# Fault bits, stale/duplicate source stamps and SDK-rejected payloads must retain
# original semantics, including counts only for accepted per-category updates.
special = [can.Message(arbitration_id=can_id, data=data, timestamp=stamp,
                       is_extended_id=False) for can_id, data, stamp in (
    (0x2A1, bytes.fromhex('010200000000ffff'), 900.),
    (0x261, bytes.fromhex('012c000a0aff0001'), 900.),
    (0x2A8, bytes.fromhex('000000640001ff00'), 900.),
    (0x2A5, struct.pack('>ii', 4000000, 1), 2000.),
    (0x2A2, struct.pack('>ii', 2000000, 1), 2000.),
    (0x2A8, struct.pack('>ihBB', 160000, 1, 0, 0), 2000.),
    (0x473, bytes.fromhex('010001ffff000200'), 2001.),
    (0x47C, bytes.fromhex('0100030000000000'), 2002.),
    (0x151, bytes.fromhex('0101010000000000'), 2003.),
    (0x777, bytes(8), 2004.),
)]
compare_frames(special, old, new, fast)
checks['fault_filter_old_stamp_and_other_id_frames'] = len(special)
for sdk in (old, new):
    sdk._C_PiperInterface__start_sdk_joint_limit = True
    sdk._C_PiperInterface__start_sdk_gripper_limit = True
    sdk.GetSDKJointLimitParam = lambda name: (-.01, .01)
    sdk.GetSDKGripperRangeParam = lambda: (0., .001)
compare_frames(frames[:40], old, new, fast)
checks['sdk_limit_enabled_frames'] = 40
for sdk in (old, new):
    sdk._C_PiperInterface__start_sdk_fk_cal = True
compare_frames(frames[:20], old, new, fast)
checks['sdk_fk_original_path_frames'] = 20

# Measure actual installed decoder + SDK updater execution (not fake counters).
# Alternate ordering to reduce one-direction warmup bias; no per-frame snapshots
# in these timed loops. Six passes, about 36k calls per implementation.
timings = {'original': [], 'selective': []}
for repeat in range(6):
    old, new, fast = pair()
    for name, callback in ([('original', old.ParseCANFrame), ('selective', fast)]
                           if repeat % 2 == 0 else [('selective', fast), ('original', old.ParseCANFrame)]):
        began = time.process_time()
        for frame in frames:
            callback(frame)
        timings[name].append(time.process_time() - began)
    assert snapshot(old) == snapshot(new)
medians = {k: statistics.median(v) for k, v in timings.items()}
report = {
    'scope': 'Offline recorded-frame replay; real installed SDK DecodeMessage/Update. '
             'SDK object.__new__ plus AST-selected pure-memory initialization; sockets, CAN constructor '
             'and thread start forbidden. Not a live receive-latency result.',
    'checks': checks,
    'recorded_ids': [hex(i) for i in sorted({f.arbitration_id for f in frames})],
    'source_time': 'Synthetic unchanged per-frame stamps; capture did not store kernel timestamps. '
                   'Explicit old/repeated stamps checked; FPS parser execution times excluded from equality.',
    'cpu_seconds_per_replay': timings,
    'median_cpu_seconds': medians,
    'median_microseconds_per_frame': {k: v * 1e6 / len(frames) for k, v in medians.items()},
    'speedup': medians['original'] / medians['selective'],
    'cpu_reduction_percent': 100. * (1. - medians['selective'] / medians['original']),
    'source_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                      for p in (MODULE, source, Path(__file__), BASE / 'feedback_frames.json')},
}
(HERE / 'feedback_dispatch_report.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps({k: report[k] for k in ('checks', 'median_microseconds_per_frame', 'speedup', 'cpu_reduction_percent')}, indent=2))
