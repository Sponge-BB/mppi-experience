#!/usr/bin/python3
"""Receive-only CAN/SDK timing. No ROS node, enable, reset or motion command."""
import argparse
from array import array
from collections import Counter, defaultdict
import json
from pathlib import Path
import resource
import time


def describe(values):
    if not values:
        return {'n': 0}
    ordered = sorted(values)
    return {'n': len(ordered), 'p50_ms': ordered[len(ordered)//2],
            'p95_ms': ordered[int((len(ordered)-1)*.95)],
            'p99_ms': ordered[int((len(ordered)-1)*.99)],
            'max_ms': ordered[-1],
            'over_150_ms': sum(v >= 150 for v in ordered)}


def interface_counters(interface):
    root = Path('/sys/class/net') / interface / 'statistics'
    return {key: int((root/key).read_text()) for key in
            ('rx_packets', 'rx_dropped', 'rx_errors', 'tx_packets', 'tx_errors')}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', default='can1')
    parser.add_argument('--seconds', type=float, default=30.)
    parser.add_argument('--stage', choices=('both', 'raw_receive', 'sdk_parse'), default='both')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 5 <= args.seconds <= 60:
        parser.error('seconds must be between 5 and 60 per stage')
    resource.setrlimit(resource.RLIMIT_AS, (768*1024**2, 768*1024**2))
    from can.interfaces.socketcan.socketcan import SocketcanBus
    from piper_sdk import C_PiperInterface

    sends = []

    def refuse_send(*unused, **unused_kw):
        sends.append(time.time())
        raise RuntimeError('Receive-only timing probe refuses every CAN send')

    # Block outgoing paths before the SDK constructor opens its socket.
    SocketcanBus.send = refuse_send
    SocketcanBus._send_once = refuse_send
    SocketcanBus._send_periodic_internal = refuse_send
    args.output.mkdir(parents=True, exist_ok=False)
    before = interface_counters(args.interface)
    report = {'interface': args.interface, 'mode': 'receive only',
              'started': time.time(), 'counters_before': before, 'stages': []}
    arm = C_PiperInterface(can_name=args.interface)
    transport = arm._C_PiperInterface__arm_can
    transport.SendCanMessage = refuse_send
    transport.bus.send = refuse_send
    arm.ConnectPort(start_thread=False, piper_init=False)
    original = arm.ParseCANFrame
    labels = {0x2A1: 'arm_status', 0x2A5: 'joint_12', 0x2A6: 'joint_34',
              0x2A7: 'joint_56', **{0x260+i: 'motor_%d' % i for i in range(1, 7)}}
    try:
        stages = (False, True) if args.stage == 'both' else (args.stage == 'sdk_parse',)
        for parse_sdk in stages:
            label = 'sdk_parse' if parse_sdk else 'raw_receive'
            start, cpu_start = time.monotonic(), time.process_time()
            received_age, parse_cost, boot_age = array('f'), array('f'), array('f')
            by_id = defaultdict(lambda: {'age': array('f'), 'gap': array('f')})
            last_stamp, counts, read_results = {}, Counter(), Counter()

            def receive(frame):
                wall, steady = time.time(), time.perf_counter()
                stamp, can_id = frame.timestamp, frame.arbitration_id
                age = (wall-stamp)*1000
                if parse_sdk:
                    original(frame)
                cost = (time.perf_counter()-steady)*1000
                counts[can_id] += 1
                if time.monotonic()-start < .5:
                    boot_age.append(age)
                    return
                received_age.append(age)
                if parse_sdk:
                    parse_cost.append(cost)
                if can_id in labels:
                    values = by_id[can_id]
                    values['age'].append(age)
                    if can_id in last_stamp:
                        values['gap'].append((stamp-last_stamp[can_id])*1000)
                    last_stamp[can_id] = stamp

            transport.callback_function = receive
            print(json.dumps({'stage': label, 'seconds': args.seconds,
                              'output': str(args.output)}), flush=True)
            with (args.output/(label+'.jsonl')).open('w') as log:
                next_report = start+1
                while time.monotonic()-start < args.seconds and not sends:
                    read_status = transport.ReadCanMessage()
                    read_results[read_status.name] += 1
                    if read_status == transport.CAN_STATUS.READ_CAN_MSG_FAILED:
                        raise RuntimeError('CAN receive/SDK parse failed; measurement is invalid')
                    now = time.monotonic()
                    if now >= next_report:
                        row = {'elapsed_s': now-start, 'frames': sum(counts.values()),
                               'last_receive_age_ms': received_age[-1] if received_age else None,
                               'last_parse_ms': parse_cost[-1] if parse_cost else None}
                        log.write(json.dumps(row)+'\n')
                        log.flush()
                        next_report = now+1
            elapsed = time.monotonic()-start
            report['stages'].append({
                'stage': label, 'elapsed_s': elapsed,
                'cpu_one_core_percent': (time.process_time()-cpu_start)/elapsed*100,
                'frames': sum(counts.values()), 'receive_age': describe(received_age),
                'read_results': dict(read_results),
                'initial_half_second_age': describe(boot_age),
                'sdk_parse_cost': describe(parse_cost),
                'per_id_counts': {hex(k): v for k, v in counts.items()},
                'feedback': {labels[k]: {'receive_age': describe(v['age']),
                                         'source_gap': describe(v['gap'])}
                             for k, v in by_id.items()}})
            print(json.dumps(report['stages'][-1]), flush=True)
            if not counts:
                raise RuntimeError('No CAN frames received; stop the comparison')
        report['arm_status'] = int(arm.GetArmStatus().arm_status.arm_status)
        report['arm_error_code'] = arm.GetArmStatus().arm_status.err_code
        motors = arm.GetArmLowSpdInfoMsgs()
        report['motors_enabled'] = [bool(getattr(motors, 'motor_%d' % i)
                                        .foc_status.driver_enable_status) for i in range(1, 7)]
    finally:
        arm.DisconnectPort()
        report['finished'] = time.time()
        report['send_attempts_blocked'] = len(sends)
        report['counters_after'] = interface_counters(args.interface)
        report['counter_delta'] = {k: report['counters_after'][k]-v for k, v in before.items()}
        report['current_rss_kib'] = next((int(line.split()[1]) for line in
            Path('/proc/self/status').read_text().splitlines() if line.startswith('VmRSS:')), None)
        (args.output/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    if sends:
        raise RuntimeError('Unexpected SDK send attempt was blocked')


if __name__ == '__main__':
    main()
