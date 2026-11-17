#!/usr/bin/env python3
"""Actual installed SDK encode/decode check; no socket, CAN, ROS or thread start."""
import hashlib
import json
from pathlib import Path
import struct
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
REPLAY = HERE.parent / 'motor_feedback_diagnostic_20261110/feedback_dispatch/feedback_dispatch_check.py'
# Reuse the existing explicitly guarded object.__new__ + pure-memory AST setup.
# Stop before its replay/benchmark so this only defines SDK objects and helpers.
namespace = {'__file__': str(REPLAY), '__name__': 'offline_sdk_setup'}
exec(compile(REPLAY.read_text().split('\nraw = json.loads(')[0], str(REPLAY), 'exec'), namespace)
SDK, make_sdk = namespace['SDK'], namespace['make_sdk']
can, FeedbackDispatch = namespace['can'], namespace['FeedbackDispatch']
PiperMessage = namespace['sdk_module'].PiperMessage
TARGET = (0, 29850, -60098, -2, 54276, 3)
EXPECTED = [(0x151, bytes.fromhex('01010f0000000000'))] + [
    (0x155 + pair, struct.pack('>ii', *TARGET[pair*2:pair*2+2])) for pair in range(3)]

class Transport:
    CAN_STATUS = SimpleNamespace(SEND_MESSAGE_SUCCESS=object())
    def __init__(self):
        self.frames = []
    def SendCanMessage(self, arbitration_id, data):
        self.frames.append((arbitration_id, bytes(data)))
        return self.CAN_STATUS.SEND_MESSAGE_SUCCESS

def sdk_with_transport():
    sdk = make_sdk()
    sdk._C_PiperInterface__arm_can = Transport()
    sdk.logger = SimpleNamespace(error=lambda *args: None)
    return sdk

def send(sdk):
    transport = sdk._C_PiperInterface__arm_can
    transport.frames.clear()
    sdk.MotionCtrl_2(1, 1, 15, 0)
    sdk.JointCtrl(*TARGET)
    assert transport.frames == EXPECTED, transport.frames
    return list(transport.frames)

old, fast_sdk = sdk_with_transport(), sdk_with_transport()
fast = FeedbackDispatch(fast_sdk)
send(old)
send(fast_sdk)
raw = json.loads((REPLAY.parent.parent / 'feedback_frames.json').read_text())
for index, item in enumerate(raw):
    frame = can.Message(arbitration_id=item['arbitration_id'],
                        is_extended_id=item['is_extended_id'],
                        data=bytes.fromhex(item['data']), timestamp=1000.+index/3040.)
    old.ParseCANFrame(frame)
    fast(frame)
    assert send(old) == send(fast_sdk)

a, b = PiperMessage(), PiperMessage()
for field in ('arm_joint_feedback', 'arm_joint_ctrl', 'arm_motion_ctrl_2'):
    assert getattr(a, field) is not getattr(b, field)
scratch = fast._messages.message
assert scratch.arm_joint_ctrl is not a.arm_joint_ctrl
report = {
    'scope': 'Installed real SDK JointCtrl/MotionCtrl_2/EncodeMessage/DecodeMessage/updaters, '
             'object.__new__ with guarded pure-memory initialization and fake transport. '
             'No sockets, CAN construction, threads, ROS or real transmission.',
    'target_counts': TARGET,
    'encoded_frames': [{'id': hex(i), 'data_hex': data.hex()} for i, data in EXPECTED],
    'interleaved_recorded_rx_frames': len(raw),
    'each_rx_then_original_and_selective_tx_equal_to_expected': True,
    'independent_message_children': True,
    'source_sha256': {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                      for path in (Path(namespace['source']), namespace['MODULE'], Path(__file__), REPLAY)},
    'limitation': 'Proves host encoding and feedback dispatch isolation for replayed inputs; '
                  'does not prove actual bus bytes, firmware acceptance, or absence of another CAN writer.'
}
(HERE / 'offline_joint_encoding_check.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps({key: report[key] for key in ('encoded_frames', 'interleaved_recorded_rx_frames',
    'each_rx_then_original_and_selective_tx_equal_to_expected', 'independent_message_children')}, indent=2))
