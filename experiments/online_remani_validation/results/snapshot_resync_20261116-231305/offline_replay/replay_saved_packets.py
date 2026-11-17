#!/usr/bin/python3
"""Replay saved ROS snapshots through production receive/reject code, offline.

No ROS initialization, publishers, subscriptions, sockets or robot commands.
The simulated response is the recorded full packet; the C++ exporter is not run.
"""
import argparse
import hashlib
import io
import json
from pathlib import Path
import resource
import struct
import sys
import threading
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import numpy as np
from rospy.msg import deserialize_messages
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float64MultiArray

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT/'src/mm_remani/scripts'))
from whole_body_mppi import WholeBodyNode


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def node():
    item = WholeBodyNode.__new__(WholeBodyNode)
    item.lock = threading.RLock()
    item.snapshot = item.snapshot_cache = None
    item.input_errors = {}
    item.base, item.joints = np.zeros(5), np.zeros(12)
    item.base_stamp = item.joint_stamp = 0.
    item.anchor = item.arm_anchor = None
    item.execute = item.startup_complete = item.sent_command = True
    item.sent_navigation_goal = 99.
    item.reference = NS(lock=threading.RLock(), active=True, goal_stamp=99., revision=7)
    item.cycle_began = 0.
    for name in ('base_pub', 'arm_pub', 'fault_pub', 'info_pub', 'snapshot_resync_pub'):
        setattr(item, name, Mock())
    return item


def feed(item, message, now, clock=1.):
    with patch('whole_body_mppi.rospy.get_time', return_value=now), \
            patch('whole_body_mppi.time.monotonic', return_value=clock):
        item.on_snapshot(message)


def metadata(message):
    h = message.data
    return dict(wire_seq=int(h[2]), field=int(h[14]), full=bool(h[20]),
                fusion_stamp=float(h[3]), copy_stamp=float(h[17]))


def state(item):
    return dict(executable_snapshot=item.snapshot is not None,
                cache_field=None if item.snapshot_cache is None else int(item.snapshot_cache.header[14]),
                error=item.input_errors.get('collision_snapshot'),
                detail=getattr(item, 'snapshot_error_detail', None),
                resync_requests=item.snapshot_resync_pub.publish.call_count,
                tracking_fault_messages=[bool(c.args[0].data) for c in item.fault_pub.publish.call_args_list])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--navigation-segment', type=Path, default=ROOT/
        'experiments/online_remani_validation/results/dynamic_crossing_20261116-195203/navigation_segment')
    args = parser.parse_args()
    paths = [args.navigation_segment/'map_events'/name for name in (
        'event08_status_referenced_full.msg', 'event09_status_referenced_full.msg',
        'event09_status_referenced_packet.msg')]
    raw = [path.read_bytes() for path in paths]
    cls = numpy_msg(Float64MultiArray)
    old, full, delta = [cls().deserialize(value) for value in raw]
    assert old.data[20] == full.data[20] == 1 and delta.data[20] == 0
    assert full.data[14] == delta.data[14] != old.data[14]
    now = float(max(delta.data[3], delta.data[17], delta.data[18]))+.01
    results = []
    for queue_size in (1, None):
        buffer = io.BytesIO(b''.join(struct.pack('<I', len(value))+value for value in raw[1:]))
        buffer.seek(0, 2)
        received = []
        deserialize_messages(buffer, received, cls, queue_size=queue_size)
        item = node()
        feed(item, old, now)
        steps = []
        for message in received:
            feed(item, message, now)
            steps.append(dict(packet=metadata(message), state=state(item)))
        results.append(dict(queue_size=queue_size, received=[metadata(m) for m in received], steps=steps))
        if queue_size == 1:
            assert item.snapshot is None
            assert item.input_errors['collision_snapshot'] == 'collision_snapshot_decode_failed'
        else:
            assert item.snapshot is not None and item.snapshot.header[2] == delta.data[2]
            assert item.snapshot_resync_pub.publish.call_count == 0

    item = node()
    feed(item, old, now)
    original_field = item.snapshot_cache.distance
    feed(item, delta, now, clock=1.)  # The prerequisite full was omitted.
    after_miss = state(item)
    assert item.snapshot is None and item.snapshot_cache.distance is original_field
    item.base_stamp = item.joint_stamp = now
    try:
        item.capture(now)
        raise AssertionError('A cache miss must not authorize a command')
    except ValueError as error:
        assert str(error) == 'collision_snapshot_decode_failed'
        with patch('whole_body_mppi.rospy.logwarn'):
            item.reject(str(error))
    zeros = [c.args[0] for c in item.base_pub.publish.call_args_list]
    assert zeros and all(c.linear.x == c.linear.y == c.angular.z == 0. for c in zeros)
    item.arm_pub.publish.assert_not_called()
    item.fault_pub.publish.assert_not_called()
    after_reject = state(item)
    feed(item, delta, now, clock=1.1)
    assert item.snapshot_resync_pub.publish.call_count == 1
    feed(item, delta, now, clock=1.251)
    assert item.snapshot_resync_pub.publish.call_count == 2
    after_dropped_response = state(item)
    # Simulate the requested resend using the actual saved matching full.
    feed(item, full, now, clock=1.26)
    restored_field = item.snapshot.distance
    feed(item, delta, now, clock=1.27)
    assert item.snapshot.distance is restored_field
    assert item.snapshot.header[14] == full.data[14]
    assert item.snapshot_error_detail is None and not item.input_errors
    assert item.reference.active and item.reference.revision == 7
    after_recovery = state(item)
    for index in (3, 17, 18):
        assert item.snapshot.header[index] == delta.data[index]
    expires = float(item.snapshot.header[3]+item.snapshot.header[15])+.001
    item.base_stamp = item.joint_stamp = expires
    try:
        item.capture(expires)
        raise AssertionError('A resend must not renew the original field TTL')
    except ValueError as error:
        assert str(error) == 'collision_snapshot_unavailable'
    source_paths = [ROOT/'src/mm_remani/scripts/whole_body_mppi.py',
        ROOT/'src/mm_remani/scripts/mppi/active_arm_mppi.py',
        ROOT/'src/mm_remani_planner/plan_manage/include/plan_manage/mppi_collision_export.hpp',
        Path(__file__), Path(sys.modules['rospy.msg'].__file__)]
    report = dict(result='PASS', inputs=[dict(path=str(path), sha256=sha(path),
        bytes=len(value), packet=metadata(message)) for path, value, message in zip(paths, raw, (old, full, delta))],
        source_hashes=[dict(path=str(path), sha256=sha(path)) for path in source_paths],
        ros_batch_replay=results,
        resync_replay=dict(replay_receive_time=now, after_miss=after_miss, after_zero_reject=after_reject,
            after_response_dropped_and_retry=after_dropped_response,
            after_recorded_full_then_delta=after_recovery,
            reused_exact_distance_array=True, zero_base_command_count=len(zeros),
            arm_command_count=0, tracking_fault_count=0,
            original_fusion_stamp=float(delta.data[3]), original_copy_stamp=float(delta.data[17]),
            rejected_after_original_ttl_at=expires),
        boundaries=[
            'Actual saved field1164/1177 payload replay; not an exact payload replay of first field1175 fault.',
            'The full response is simulated by replaying a recorded packet; no C++ exporter or publisher queue integration is exercised.',
            'Publishers are mocks; zero-command and fault assertions verify production Python behavior without robot I/O.',
            'Single producer/ordered connection only; no concurrent multi-publisher ordering guarantee is tested.',
            'Passing offline replay does not establish real-robot validation or eliminate independent 95ms budget failures.'],
        peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024.)
    destination = Path(__file__).with_name('saved_packet_replay.json')
    destination.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(dict(result='PASS', report=str(destination), peak_rss_mib=report['peak_rss_mib'],
                         report_sha256=sha(destination))))


if __name__ == '__main__':
    main()
