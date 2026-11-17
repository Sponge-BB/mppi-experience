#!/usr/bin/env python3
"""RX by default; --execute requests one <=1 s current-position hold.

No ROS, base, gripper, reset, disable, automatic resume or re-enable. A software
stop is not a physical emergency stop and may permit gravity settling. Execute
only after the operator has confirmed support/clearance and reviewed this file.
"""
import os
import sys
import resource
sys.dont_write_bytecode = True
for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'
resource.setrlimit(resource.RLIMIT_AS, (512 * 1024 ** 2, 512 * 1024 ** 2))

import argparse
from collections import deque
from copy import copy
from hashlib import sha256
import json
import math
from pathlib import Path
import signal
import socket
import struct
import threading
import time
import numpy as np

BASE = Path(__file__).resolve().parent
RELEASES = BASE.parent / 'releases'
OLD = RELEASES / 'standby_20261106_stationary_07'
sys.path[:0] = [str(RELEASES/'standby_20261106_distributed_08/runtime'),
               str(RELEASES/'standby_20261105_feedback_02/runtime'),
               str(OLD/'runtime'), str(OLD/'core'), str(OLD/'python')]
from piper_can_receive_only import ReceiveOnlySocket, SOL_CAN_RAW, CAN_RAW_FILTER
from piper_feedback_decode import JointFeedbackAssembler, PAIR_IDS, CAN_SFF_MASK, CAN_EFF_FLAG, CAN_RTR_FLAG
from piper_lease_gate import PiperLeaseGate, JOINT_NAMES, LOWER, UPPER
from bounded_can_transport import install_bounded_can_transport
from active_arm_model import PiperArmModel

CONTROL_IDS = set(range(0x150, 0x180)) | {0x470, 0x471, 0x472, 0x474, 0x475, 0x477, 0x479, 0x47A, 0x47D}
MOTOR_IDS = tuple(range(0x261, 0x267))
STATUS_IDS = {0x2A1, *MOTOR_IDS}
CONTROL_PROGRAMS = {'piper_ctrl_single_node.py', 'piper_reused_driver', 'piper_reused_driver_node.py',
                    'mm_controller_node', 'leased_base_driver', 'leased_base_driver.py',
                    'base_control.py', 'joint_cmd_bridge.py', 'arm_hold_probe.py'}


def probe_model():
    model = PiperArmModel.from_parameters(OLD/'model/parameters.yaml', OLD/'model/mm.urdf')
    data = json.loads((BASE/'hold_pose_separation.json').read_text())
    asset = OLD/'model/shoulder_wrist_proxy.json'
    if data['proxy_sha256'] != sha256(asset.read_bytes()).hexdigest():
        raise ValueError('probe_separation_asset_mismatch')
    original = model.shoulder_wrist_proxy
    extra = copy(original)
    axis = np.asarray(data['axis_local_link5'], dtype=float)
    if (axis.shape != (3,) or not np.isfinite(axis).all() or
            abs(float(np.linalg.norm(axis))-1.) > 1e-8 or
            data['padding_m'] != original.numerical_tolerance):
        raise ValueError('invalid_probe_separation_axis')
    extra.axis = axis.copy(); extra.axis.setflags(write=False)
    projection = extra.vertices['link5'] @ extra.axis
    extra.link5_interval = (float(projection.min()), float(projection.max()))
    original_gaps = original.pair_gaps
    def combined_gaps(q):
        gaps = original_gaps(q)
        # Either complete-hull separating direction proves this pair disjoint.
        # Evaluate actual q every time; retain the original upper-arm result.
        gaps[:, 0] = np.maximum(gaps[:, 0], extra.pair_gaps(q)[:, 0])
        return gaps
    original.pair_gaps = combined_gaps
    return model


def decode_status(can_id, payload, stamp):
    # Frozen SDK protocol_v2/piper_protocol_v2.py:74-80, 165-216;
    # msg_v2/feedback/arm_feedback_low_spd.py: bit6=enable; other bits=faults.
    if can_id not in STATUS_IDS or len(payload) != 8 or not math.isfinite(stamp):
        raise ValueError('invalid_hardware_status_frame')
    row = dict(can_id=can_id, kernel_stamp_monotonic=stamp)
    if can_id == 0x2A1:
        row.update(ctrl_mode=payload[0], arm_status=payload[1], mode_feed=payload[2],
                   teach_status=payload[3], motion_status=payload[4],
                   err_code=int.from_bytes(payload[6:8], 'big'))
    else:
        row.update(motor=can_id-0x260, foc_status=payload[5],
                   enabled=bool(payload[5] & 0x40), fault_bits=payload[5] & 0xBF)
    return row


def status_problem(frames, now, require_disabled=False):
    if any(i not in frames for i in STATUS_IDS): return 'hardware_status_incomplete'
    if any(not 0. <= now-frames[i]['kernel_stamp_monotonic'] < .15 for i in STATUS_IDS):
        return 'hardware_status_stale'
    arm = frames[0x2A1]
    if (arm['arm_status'] != 0 or arm['err_code'] != 0 or arm['teach_status'] != 0
            or arm['ctrl_mode'] not in (0, 1)):
        return 'arm_status_not_normal'
    if any(frames[i]['fault_bits'] for i in MOTOR_IDS): return 'motor_fault_bits_set'
    if require_disabled and any(frames[i]['enabled'] for i in MOTOR_IDS):
        return 'motor_already_enabled'
    return None


def reject_controllers():
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            with (entry/'cmdline').open('rb') as stream:
                words = stream.read(8192).split(b'\0')
            names = [Path(w.decode(errors='replace')).name for w in words if w]
            candidates = names[:3] if names and names[0].startswith('python') else names[:1]
            if any(name in CONTROL_PROGRAMS for name in candidates):
                raise RuntimeError('existing_control_process_pid_'+entry.name)
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue


def check_pose(model, q, target=None):
    if len(q) != 6 or any(not math.isfinite(v) or v < lo or v > hi
                          for v, lo, hi in zip(q, LOWER, UPPER)):
        raise RuntimeError('six_joint_limits_failed')
    drift = 0. if target is None else max(abs(v-t) for v, t in zip(q, target))
    if drift > .005:
        raise RuntimeError('position_deviation_over_0.005rad')
    result = model.check(q)
    if not result['valid']:
        raise RuntimeError('fine_geometry_rejected:'+','.join(result['violations']))
    return drift, result


def self_test():
    class FakeSDK:
        set_transport_deadline = None
        def __init__(self): self.calls = []
        def __getattr__(self, name):
            return lambda *args: self.calls.append((name, args))
    q = (-.23212879051524585, -.03330088212805181, .006754424205218055,
         .02741912254883092, .4934220328313169, -.020629791758572977)
    model = probe_model()
    check_pose(model, q)
    current = tuple(json.loads((BASE/'hold_pose_separation.json').read_text())['q_radians'])
    check_pose(model, current)
    assert not model.check(tuple(q[:4])+(q[4]+.03, q[5]))['valid']
    sdk = FakeSDK(); gate = PiperLeaseGate(sdk, allow_motion=True, fixed_vel=1)
    assert gate.update_feedback(JOINT_NAMES, q, 100., 10., 100.)
    assert gate.enable(True, 10.01, 100.01)
    assert not gate.enable(True, 10.02, 100.02)
    assert gate.command(JOINT_NAMES, q, 1, 100.03, 10.03, 100.03, expires_at=100.13)
    gate.tick(10.14, 100.14)
    assert [x[0] for x in sdk.calls] == ['EnableArm', 'MotionCtrl_2', 'JointCtrl', 'MotionCtrl_1']
    assert sdk.calls[1][1] == (1, 1, 1, 0) and sdk.calls[-1][1] == (1, 0, 0)
    try: check_pose(model, (q[0]+.006,)+q[1:], q)
    except RuntimeError as error: assert str(error) == 'position_deviation_over_0.005rad'
    else: raise AssertionError('drift was accepted')
    decoded = decode_status(0x2A1, bytes([1, 7, 1, 0, 0, 0, 0x12, 0x34]), 10.)
    assert (decoded['ctrl_mode'], decoded['arm_status'], decoded['err_code']) == (1, 7, 0x1234)
    assert decode_status(0x261, bytes([0, 240, 0, 30, 25, 0x40, 0, 0]), 10.)['enabled']
    frames = {i: decode_status(i, bytes(8), 10.) for i in STATUS_IDS}
    assert status_problem(frames, 10.01, True) is None
    assert status_problem(frames, 10.16, True) == 'hardware_status_stale'
    frames[0x261] = decode_status(0x261, bytes([0, 0, 0, 0, 0, 0x40, 0, 0]), 10.)
    assert status_problem(frames, 10.01, True) == 'motor_already_enabled'
    frames[0x261] = decode_status(0x261, bytes([0, 0, 0, 0, 0, 0x80, 0, 0]), 10.)
    assert status_problem(frames, 10.01) == 'motor_fault_bits_set'
    assert decode_status(0x2A1, bytes([1, 1, 1, 0, 0, 0, 0, 0]), 10.)['arm_status'] == 1
    print(json.dumps({'fake_checks_passed': True, 'hardware_opened': False,
                      'checks': ['measured_folded_geometry', 'one_enable_only', 'fixed_target_mode_speed',
                                 'lease_timeout_stop', 'drift_rejection', 'SDK_status_byte_fields',
                                 'SDK_motor_enable_bit6', 'status_freshness', 'already_enabled_rejection',
                                 'motor_fault_rejection', 'quickstop_status1_decode']}))


def run(args):
    directory = BASE/('inspect_' if not args.execute else 'hold_')
    directory = directory.with_name(directory.name+str(time.time_ns()))
    directory.mkdir(parents=True)
    log = (directory/'events.jsonl').open('x', buffering=1)
    write_lock, expected_lock = threading.Lock(), threading.Lock()
    expected, halt, finished = deque(maxlen=256), threading.Event(), threading.Event()
    result = dict(execute_requested=args.execute, hardware_motion_validated=False,
                  physical_motion_braking_validated=False, sdk_api_operations_completed=0,
                  external_obstacles_checked=False, reason='starting', max_drift_rad=0.,
                  hold_target=None, source_stamps_changed=False, tx_frames=0,
                  six_motor_enable_observed=False, quickstop_status_observed=False)
    hardware_status = {}
    gate, sdk, watchdog = None, None, None
    hold_until, stop_at, last_snapshot = math.inf, None, None
    started = time.monotonic()
    receiver = ReceiveOnlySocket(args.interface, timeout=.01)
    assembler = JointFeedbackAssembler()
    model = probe_model()

    def emit(event, **values):
        with write_lock:
            log.write(json.dumps(dict(event=event, monotonic=time.monotonic(), wall=time.time(), **values))+'\n')

    def status_received(can_id, payload, stamp):
        row = decode_status(can_id, payload, stamp)
        hardware_status[can_id] = row
        emit('hardware_status', **row)
        enable_stamp = result.get('enable_request_monotonic', math.inf)
        if (not result['six_motor_enable_observed'] and
                status_problem(hardware_status, time.monotonic()) is None and
                all(hardware_status[i]['enabled'] and
                    hardware_status[i]['kernel_stamp_monotonic'] >= enable_stamp for i in MOTOR_IDS)):
            result['six_motor_enable_observed'] = True
            result['six_motor_enable_confirmation'] = {hex(i): dict(hardware_status[i]) for i in MOTOR_IDS}
            emit('six_motor_enable_observed')
        stop_stamp = result.get('stop_request_monotonic', math.inf)
        if (can_id == 0x2A1 and row['arm_status'] == 1 and stamp >= stop_stamp
                and not result['quickstop_status_observed']):
            result['quickstop_status_observed'] = True
            result['quickstop_confirmation'] = dict(row)
            emit('quickstop_status_observed', kernel_stamp_monotonic=stamp)

    def interrupted(signum, _frame):
        result['reason'] = 'signal_'+str(signum); result['interrupted'] = True; halt.set()
    for number in (signal.SIGINT, signal.SIGTERM): signal.signal(number, interrupted)

    def watch():
        try:
            while not finished.wait(.01):
                with gate.lock:
                    now, wall = time.monotonic(), time.time()
                    problem = status_problem(hardware_status, now)
                    if halt.is_set() or now >= hold_until: gate.stop('hold_deadline_or_signal')
                    elif gate.enabled and problem: gate.stop(problem)
                    else: gate.tick(now, wall)
        except BaseException as error:
            halt.set(); gate.stop('watchdog_error:'+type(error).__name__)

    def installer(native):
        wrapped = install_bounded_can_transport(native)
        raw_send = wrapped.transport._raw_send
        target = tuple(round(v*180000/math.pi) for v in result['hold_target'])
        allowed = {0x471: bytes([7, 2, 0, 0, 0, 0, 0, 0]),
                   0x151: bytes([1, 1, 1, 0, 0, 0, 0, 0]),
                   0x150: bytes([1, 0, 0, 0, 0, 0, 0, 0])}
        for i in range(3): allowed[0x155+i] = struct.pack('>ii', *target[2*i:2*i+2])
        def send(message, timeout=None):
            item = (message.arbitration_id, bytes(message.data))
            if message.is_extended_id or allowed.get(item[0]) != item[1]:
                raise RuntimeError('unexpected_outgoing_CAN_frame')
            if item[0] == 0x471: result['enable_request_monotonic'] = time.monotonic()
            if item[0] == 0x150: result['stop_request_monotonic'] = time.monotonic()
            with expected_lock: expected.append((item, time.monotonic()))
            raw_send(message, timeout=timeout)
            result['tx_frames'] += 1
            emit('tx', can_id=hex(item[0]), data=item[1].hex())
        wrapped.transport._raw_send = send
        return wrapped

    try:
        reject_controllers(); receiver.open()
        mask = CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG
        filters = b''.join(struct.pack('=II', can_id, mask)
                           for can_id in sorted(CONTROL_IDS | set(PAIR_IDS) | STATUS_IDS))
        receiver._socket.setsockopt(SOL_CAN_RAW, CAN_RAW_FILTER, filters)
        stable_since, baseline, last_command, sequence, last_source = None, None, -math.inf, 0, -math.inf
        last_log, last_process_check = -math.inf, -math.inf
        while True:
            now = time.monotonic()
            if halt.is_set() and stop_at is None: stop_at = now
            if gate is not None and not gate.status()['enabled'] and stop_at is None:
                stop_at = now; result['reason'] = gate.status()['reason']; emit('stopped', reason=result['reason'])
            if stop_at is not None and now-stop_at >= 2.: break
            if now-started > 10.: raise RuntimeError('probe_total_deadline')
            if now-last_process_check >= .2:
                reject_controllers(); last_process_check = now
            try:
                can_id, payload, stamp = receiver.receive()
                if can_id in CONTROL_IDS:
                    with expected_lock:
                        while expected and now-expected[0][1] > .15: expected.popleft()
                        match = next((x for x in expected if x[0] == (can_id, payload)), None)
                        if match is None: raise RuntimeError('foreign_CAN_control_frame_'+hex(can_id))
                        expected.remove(match)
                elif can_id in STATUS_IDS: status_received(can_id, payload, stamp)
                elif can_id in PAIR_IDS and not assembler.ingest(can_id, payload, stamp):
                    raise RuntimeError('joint_feedback_frame_rejected')
            except socket.timeout: pass
            snap = assembler.snapshot(time.monotonic())
            if snap is None:
                if last_snapshot is not None and time.monotonic()-last_snapshot.stamp_monotonic >= .15:
                    raise RuntimeError('feedback_stale')
                continue
            last_snapshot = snap
            try:
                drift, geometry = check_pose(model, snap.positions, result['hold_target'])
            except RuntimeError:
                result['rejected_positions'] = snap.positions
                result['geometry'] = model.check(snap.positions)
                raise
            result['max_drift_rad'] = max(result['max_drift_rad'], drift)
            if now-last_log >= .05:
                emit('feedback', q=snap.positions, dq=snap.velocities, kernel_oldest_monotonic=snap.stamp_monotonic,
                     max_drift_rad=drift); last_log = now
            if gate is not None:
                with gate.lock:
                    current, wall = time.monotonic(), time.time()
                    source = wall-(current-snap.stamp_monotonic)
                    if not gate.update_feedback(JOINT_NAMES, snap.positions, source, current, wall):
                        raise RuntimeError('feedback_gate_rejected:'+gate.reason)
                    if gate.enabled and current < hold_until and current-last_command >= .04:
                        sequence += 1; source = time.time()
                        if source <= last_source: raise RuntimeError('command_source_not_unique')
                        expiry = source+min(.12, hold_until-current)
                        if not gate.command(JOINT_NAMES, result['hold_target'], sequence, source, current,
                                            source, expires_at=expiry): raise RuntimeError(gate.reason)
                        emit('hold_command', sequence=sequence, source_stamp=source, expires_at=expiry)
                        last_source, last_command = source, current
                continue
            if stop_at is not None: continue
            if now-started > 5.: raise RuntimeError('stationary_feedback_not_ready')
            problem = status_problem(hardware_status, time.monotonic(), require_disabled=True)
            if problem == 'hardware_status_incomplete':
                stable_since, baseline = None, None; continue
            if problem: raise RuntimeError(problem)
            if not snap.velocity_valid or max(abs(v) for v in snap.velocities) > .01:
                stable_since, baseline = None, None; continue
            if baseline is None: baseline, stable_since = snap.positions, now
            if max(abs(q-b) for q, b in zip(snap.positions, baseline)) > .001:
                baseline, stable_since = snap.positions, now; continue
            if now-stable_since < 1.: continue
            if result['hold_target'] is not None and tuple(snap.positions) != result['hold_target']:
                raise RuntimeError('position_changed_during_SDK_startup')
            result['hold_target'] = tuple(snap.positions); result['geometry'] = geometry
            result['preflight_hardware_status'] = {hex(i): dict(hardware_status[i]) for i in STATUS_IDS}
            check_pose(model, tuple(round(v*180000/math.pi)*math.pi/180000
                                    for v in result['hold_target']), result['hold_target'])
            emit('stationary_preflight_passed', q=result['hold_target'], geometry=geometry)
            if not args.execute:
                result['reason'] = 'inspect_passed_no_TX'; break
            reject_controllers()
            if sdk is None:
                from piper_sdk import C_PiperInterface
                sdk = installer(C_PiperInterface(can_name=args.interface))
                # Verified RX already runs here. The original SDK's optional
                # second decoder competes for the GIL and is not needed for TX.
                sdk.ConnectPort(piper_init=False, start_thread=False)
                # SDK startup has no TX. Obtain another full stable RX window
                # afterwards; never relabel feedback collected before startup.
                stable_since, baseline = None, None
                assembler, last_snapshot = JointFeedbackAssembler(), None
                hardware_status.clear()
                # Discard the startup backlog before any enable request.
                receiver.close(); receiver.open()
                receiver._socket.setsockopt(SOL_CAN_RAW, CAN_RAW_FILTER, filters)
                continue
            gate = PiperLeaseGate(sdk, allow_motion=True, fixed_vel=1)
            with gate.lock:
                current, wall = time.monotonic(), time.time()
                problem = status_problem(hardware_status, current, require_disabled=True)
                if problem: raise RuntimeError(problem+'_after_SDK_open')
                source = wall-(current-snap.stamp_monotonic)
                if not gate.update_feedback(JOINT_NAMES, snap.positions, source, current, wall):
                    raise RuntimeError('feedback_stale_after_SDK_open')
                hold_until = current+1.
                if not gate.enable(True, current, wall): raise RuntimeError(gate.reason)
            watchdog = threading.Thread(target=watch, daemon=True); watchdog.start()
            emit('enable_requested_once', hold_until_monotonic=hold_until)
    except BaseException as error:
        result['reason'] = type(error).__name__+':'+str(error); emit('error', reason=result['reason'])
        if gate is not None:
            gate.stop('probe_error'); post_stop_end = time.monotonic()+2.
            while time.monotonic() < post_stop_end:
                try:
                    can_id, payload, stamp = receiver.receive()
                    if can_id in STATUS_IDS: status_received(can_id, payload, stamp)
                    elif can_id in PAIR_IDS: assembler.ingest(can_id, payload, stamp)
                    snap = assembler.snapshot(time.monotonic())
                    if snap is not None:
                        drift = max(abs(q-t) for q, t in zip(snap.positions, result['hold_target']))
                        result['max_drift_rad'] = max(result['max_drift_rad'], drift)
                        if time.monotonic()-last_log >= .05:
                            emit('post_stop_feedback', q=snap.positions, max_drift_rad=drift,
                                 kernel_oldest_monotonic=snap.stamp_monotonic)
                            last_log = time.monotonic()
                except socket.timeout: pass
                except Exception as observation_error:
                    emit('post_stop_feedback_error', reason=str(observation_error)); break
    finally:
        if gate is not None: gate.stop('probe_finally')
        finished.set()
        if watchdog is not None: watchdog.join(timeout=.1)
        try:
            if sdk is not None: sdk.DisconnectPort()
        except Exception as error: result['disconnect_error'] = str(error)
        finally: receiver.close()
        result['duration_s'] = time.monotonic()-started
        if gate is not None:
            result['sdk_api_operations_completed'] = gate.status()['sdk_tx_operations']
        result['last_hardware_status'] = {hex(i): dict(v) for i, v in hardware_status.items()}
        if args.execute and result['reason'] == 'hold_deadline_or_signal':
            if not result['six_motor_enable_observed']: result['reason'] = 'six_motor_enable_not_confirmed'
            elif not result['quickstop_status_observed']: result['reason'] = 'hardware_quickstop_not_confirmed'
        result['peak_rss_mib'] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024.
        result['process_memory'] = {line.split(':', 1)[0]: line.split(':', 1)[1].strip()
                                   for line in Path('/proc/self/status').read_text().splitlines()
                                   if line.startswith(('VmHWM:', 'VmRSS:', 'VmPeak:'))}
        # VmHWM belongs to this executable image; ru_maxrss may retain the
        # launching process's pre-exec high-water mark on this host.
        result['peak_rss_mib'] = int(result['process_memory']['VmHWM'].split()[0])/1024.
        result['limitations'] = ['no physical response certification', 'identical CAN frames have no sender authentication',
                                 'fine model check excludes external obstacles', 'software stop may allow gravity settling']
        (directory/'result.json').write_text(json.dumps(result, indent=2)+'\n'); log.close()
    print(json.dumps({'report': str(directory/'result.json'), **result}, indent=2))
    return 0 if (not result.get('interrupted') and
                 result['reason'] in ('inspect_passed_no_TX', 'hold_deadline_or_signal')) else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', default='can0')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--self-test', action='store_true')
    arguments = parser.parse_args()
    if arguments.self_test:
        self_test()
    else:
        sys.exit(run(arguments))
