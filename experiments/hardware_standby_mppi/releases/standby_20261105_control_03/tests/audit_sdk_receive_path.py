"""Offline AST/source inventory. Does not import or instantiate the Piper SDK.

This records the exact SDK method closure reached by the fork's receive path.
PiperInit is the sole excluded call: the fork explicitly passes piper_init=False.
External python-can/socket construction is separately source reviewed below.
"""
import ast
import hashlib
import json
from pathlib import Path

RELEASE = Path(__file__).resolve().parents[1]


def method_closure(path, classname, roots, excluded=()):
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == classname)
    methods = {n.name: n for n in cls.body if isinstance(n, ast.FunctionDef)}
    pending = list(roots)
    visited = set()
    report = []
    while pending:
        name = pending.pop()
        if name in visited:
            continue
        visited.add(name)
        node = methods[name]
        direct = set()
        transmit = []
        for sub in ast.walk(node):
            if not isinstance(sub, ast.Call) or not isinstance(sub.func, ast.Attribute):
                continue
            if sub.func.attr in ("SendCanMessage", "send", "sendto", "sendmsg", "send_periodic"):
                transmit.append(dict(line=sub.lineno, method=sub.func.attr))
            if isinstance(sub.func.value, ast.Name) and sub.func.value.id == "self":
                callee = sub.func.attr
                direct.add(callee)
                if callee in methods and callee not in excluded:
                    pending.append(callee)
        report.append(dict(method=name, line=node.lineno, self_calls=sorted(direct),
                           transmit_calls=transmit))
        if transmit:
            raise AssertionError("transmit call reachable from read path: %s" % name)
    return dict(path=str(path.relative_to(RELEASE)), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                excluded_calls=list(excluded), reachable_methods=sorted(report, key=lambda x:x['line']))


def audit():
    sdk = RELEASE/'python/piper_sdk/interface/piper_interface.py'
    backend = RELEASE/'python/piper_sdk/hardware_port/can_encapsulation.py'
    reports = [method_closure(sdk, 'C_PiperInterface', [
        '__new__', '__init__', 'ConnectPort', 'ParseCANFrame', '__CanMonitor',
        'GetArmJointMsgs', 'GetArmStatus', 'GetArmEndPoseMsgs', 'GetArmHighSpdInfoMsgs',
        'GetArmLowSpdInfoMsgs', 'GetArmGripperMsgs', 'DisconnectPort'], ['PiperInit']),
        method_closure(backend, 'C_STD_CAN', ['__init__', 'Init', 'JudgeCanInfo',
                                            'ReadCanMessage', 'Close', '__del__'])]
    return dict(
        status='offline_source_review_not_live_CAN_validation',
        SDK_imported=False, SDK_instantiated=False, device_opened=False,
        excluded_branch_proof='runtime/piper_lease_gate.py open_existing_sdk calls ConnectPort(piper_init=False)',
        source_paths=reports,
        external_review=[
            'C_STD_CAN.Init only creates can.interface.Bus(channel=..., bustype=socketcan); it sends no frames.',
            'Frozen python-can socketcan.SocketcanBus.__init__ creates/binds a CAN_RAW socket and configures receive options; no send, periodic TX or BCM task starts.',
            'C_STD_CAN.JudgeCanInfo checks sysfs/interface state and ip -details link show; it does not configure bitrate or bring interfaces up.',
            'SDK ParseCANFrame calls decoder and cached feedback updates. ReadCan and CanMonitor thread functions are included by ast.walk of ConnectPort.',
            'C_FPSCounter.start launches only its counter thread. No CAN handle is passed to it.',
            'Original driver constructor is not invoked. No original subscription/automatic enable thread is started.',
            'Gate allow_motion=false rejects all enable and command requests; disabled watchdog/shutdown has zero actuator calls.',
        ],
        limits=[
            'Static method closure is not a formal proof of every dynamically dispatched dependency.',
            'Actual driver RX-only startup still requires host TX counters and ROS publisher audit before and after.',
            'Any connect_transport=true starts a real CAN socket; this audit itself never does so.',
        ])


if __name__ == '__main__':
    print(json.dumps(audit(), indent=2))
