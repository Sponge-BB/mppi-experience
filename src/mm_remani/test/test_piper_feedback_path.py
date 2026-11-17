#!/usr/bin/env python3
"""Feedback diagnostic boundaries; no ROS master, CAN socket, SDK or motors."""
import contextlib
import importlib.util
import io
from pathlib import Path
import runpy
import sys
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parents[3]
with contextlib.redirect_stdout(io.StringIO()):
    original = runpy.run_path(str(ROOT/'src/mm_remani/test/test_piper_feedback_command.py'))
feed = original['feed']
def node():
    n = original['node']()
    n._feedback_path_pending = {}
    n._feedback_path_last_log = 0.
    return n
clock, namespace = original['clock'], original['namespace']
logs = []
namespace['rospy'].logwarn = lambda *args: logs.append(args)

n = node()
n.feedback_path_diagnostics = True
n._feedback_path_last = {}
feed(n)
stamp = n._feedback_stamp
n._feedback_path_call('probe', lambda: None)
clock.value += .01
n._feedback_path_call('probe', lambda: None)
assert not logs
clock.value += .2
n._feedback_path_call('probe', lambda: None)
assert not logs  # Observing a delay under a command lock performs no ROS I/O.
assert n._feedback_path_pending['probe', 'entry_gap']['elapsed_ms'] >= 200.
assert n._feedback_stamp == stamp  # Observation cannot renew source freshness.

logs.clear()
def slow_failure():
    clock.value += .2
    raise RuntimeError('original failure')
try:
    n._feedback_path_call('slow_call', slow_failure)
except RuntimeError as exc:
    assert str(exc) == 'original failure'
else:
    raise AssertionError('diagnostic swallowed the failure')
assert n._feedback_path_pending['slow_call', 'call_elapsed']['elapsed_ms'] >= 200.

class SlowLock:
    def __enter__(self):
        clock.value += .2
    def __exit__(self, *args):
        return False
n = node()
n.feedback_path_diagnostics = True
n._feedback_path_last = {}
feed(n)
before = len(n.joint_pub.messages)
n._command_lock = SlowLock()
logs.clear()
n.PublishArmJointAndGripper()
assert ('joint_command_lock', 'wait') in n._feedback_path_pending
assert not logs
assert len(n.joint_pub.messages) == before  # Delayed old feedback remains stale.
n._feedback_path_flush()
assert len(logs) == 1 and not n._feedback_path_pending
n._feedback_path_event('joint_command_lock', 'wait', .2)
n._feedback_path_flush()
assert len(logs) == 1  # At most one batch per second.
clock.value += 1.01
def broken_logger(*args):
    raise RuntimeError('diagnostic sink failed')
namespace['rospy'].logwarn = broken_logger
n._feedback_path_flush()  # Diagnostic sink failure never becomes a stop fault.
namespace['rospy'].logwarn = lambda *args: logs.append(args)

for enabled in (False, True):
    n = node()
    n.feedback_path_diagnostics = enabled
    n._feedback_path_last = {}
    feed(n)
    stamp = n._feedback_stamp
    clock.value += .04
    feed(n, pairs=(0, 1))
    assert n._feedback_stamp == stamp
    assert n._feedback_reject_reason == 'triplet_not_advanced'
    clock.value += .01
    feed(n, pairs=(2,))
    assert n._feedback_stamp > stamp
    assert n._feedback_reject_reason == 'accepted'

# Load the dispatch with a fake decoded message, preserving its original API.
module_name = 'piper_sdk.piper_msgs.msg_v2'
saved = sys.modules.get(module_name)
sys.modules[module_name] = NS(PiperMessage=NS)
try:
    path = ROOT/'src/piper_ros/src/piper/scripts/piper_feedback_dispatch.py'
    spec = importlib.util.spec_from_file_location('path_dispatch_under_test', path)
    dispatch_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(dispatch_module)
finally:
    if saved is None:
        del sys.modules[module_name]
    else:
        sys.modules[module_name] = saved
dispatch_module.time = clock

def fake_sdk():
    calls = []
    p = NS(ParseCANFrame=lambda frame: calls.append(('original', frame.arbitration_id)))
    p._C_PiperInterface__start_sdk_fk_cal = False
    p._C_PiperInterface__parser = NS(DecodeMessage=lambda f, m: f.valid)
    p._C_PiperInterface__fps_counter = NS(increment=lambda key: calls.append(('counter', key)))
    for method in ('UpdateArmStatus', 'UpdateArmEndPoseState', 'UpdateArmJointState',
                   'UpdateArmGripperState', 'UpdateDriverInfoHighSpdFeedback',
                   'UpdateDriverInfoLowSpdFeedback'):
        setattr(p, '_C_PiperInterface__'+method,
                lambda message, name=method: calls.append(('update', name)))
    return p, calls
outputs = []
for enabled in (False, True):
    p, calls = fake_sdk()
    dispatch = dispatch_module.FeedbackDispatch(p, enabled)
    for ident, valid in ((0x2a5, True), (0x2a6, False), (0x261, True), (0x123, True)):
        dispatch(NS(arbitration_id=ident, timestamp=1000., valid=valid))
    outputs.append(calls)
    if enabled:
        assert dispatch.path_diagnostics()['stage'] == 'idle'
        assert dispatch.path_diagnostics()['last_joint_dispatch_source'] == 1000.
assert outputs[0] == outputs[1]
print('Feedback path checks passed: threshold, wait/call distinction, no freshness renewal, '
      'exception preservation, complete-triplet acceptance and dispatch parity; no ROS/CAN/SDK.')
