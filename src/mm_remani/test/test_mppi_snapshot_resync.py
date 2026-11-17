"""Dependent snapshot transport/recovery; no ROS master, sockets or hardware.

The ROS batch test calls the installed rospy deserializer. Receiver tests call
the production methods; returning a full response is simulated, so these do
not claim to test roscpp publisher queues or physical command delivery.
"""
import ast
import io
import json
from pathlib import Path
import struct
import sys
import threading
from types import MethodType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np
from rospy.msg import deserialize_messages
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Empty, Float64MultiArray

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'scripts'))
import whole_body_mppi as production
from active_arm_mppi import Snapshot


def packet(sequence=1, field=1, full=True, fusion=100., copy_stamp=100.02,
           origin=(0., 0., 0.), value=1., dynamic=False):
    """Small valid schema 3.1 packet; dimensions/ESDF are real decoder inputs."""
    h = np.zeros(32)
    h[:5] = [3., 1., sequence, fusion, 1.]
    h[5:8], h[8:11], h[11] = (4, 4, 4), origin, .1
    h[12:21] = [1, 0, field, .35, 2., copy_stamp, fusion, int(dynamic), int(full)]
    h[22:25], h[28:32] = [1., 1., fusion], [int(dynamic), field, sequence, sequence]
    return np.r_[h, [0., 0., 0., .1, .2, 1., 0.],
                 np.full(64, value) if full else []]


def ros_batch(packets, queue_size):
    """Serialize actual ROS messages into one TCPROS-style receive batch."""
    buffer = io.BytesIO()
    for wire in packets:
        payload = io.BytesIO()
        Float64MultiArray(data=wire.tolist()).serialize(payload)
        raw = payload.getvalue()
        buffer.write(struct.pack('<I', len(raw)))
        buffer.write(raw)
    result = []
    deserialize_messages(buffer, result, numpy_msg(Float64MultiArray),
                         queue_size=queue_size)
    return result


def receiver(simple=False):
    node = NS() if simple else production.WholeBodyNode.__new__(production.WholeBodyNode)
    node.lock = threading.RLock()
    node.snapshot = None
    if not simple:
        node.snapshot_cache = None
    node.input_errors = {}
    node.snapshot_resync_pub = Mock()
    node.base, node.joints = np.zeros(5), np.zeros(12)
    node.base_stamp = node.joint_stamp = 100.
    node.anchor = node.arm_anchor = None
    node.execute = node.startup_complete = True
    node.sent_command = True
    node.sent_navigation_goal = 99.
    node.reference = NS(lock=threading.RLock(), active=True, goal_stamp=99., revision=7)
    node.cycle_began = 0.
    for name in ('base_pub', 'arm_pub', 'fault_pub', 'info_pub'):
        setattr(node, name, Mock())
    if simple:
        for name in ('request_snapshot_resync', 'stop_base'):
            setattr(node, name, MethodType(getattr(production.WholeBodyNode, name), node))
    return node


class SnapshotResyncTests(unittest.TestCase):
    def feed(self, node, wire, monotonic=1., now=100.1):
        with patch('whole_body_mppi.rospy.get_time', return_value=now), \
                patch('whole_body_mppi.time.monotonic', return_value=monotonic):
            production.WholeBodyNode.on_snapshot(node, NS(data=wire))

    def test_configured_ros_batch_keeps_full_before_arbitrarily_many_deltas(self):
        # Evaluate just the real constructor's subscription call with I/O
        # mocked, then use its actual queue setting in the real deserializer.
        tree = ast.parse((PACKAGE/'scripts/whole_body_mppi.py').read_text())
        call, = [item for item in ast.walk(tree) if isinstance(item, ast.Call)
                 and isinstance(item.func, ast.Attribute) and item.func.attr == 'Subscriber'
                 and item.args and isinstance(item.args[0], ast.Constant)
                 and item.args[0].value == '/mppi/collision_snapshot']
        subscriber = Mock()
        scope = dict(vars(production), self=NS(on_snapshot=Mock()))
        with patch('whole_body_mppi.rospy.Subscriber', subscriber):
            eval(compile(ast.Expression(body=call), '<production snapshot subscription>', 'eval'), scope)
        configured = subscriber.call_args.kwargs
        self.assertIsNone(configured['queue_size'])
        wires = [packet(sequence=10, field=2)] + [
            packet(sequence=i, field=2, full=False) for i in range(11, 20)]
        old = Snapshot.decode(packet(field=1))
        latest_only = ros_batch(wires, queue_size=1)
        self.assertEqual(len(latest_only), 1)
        with self.assertRaisesRegex(ValueError, 'delta without identical cached ESDF'):
            Snapshot.decode(latest_only[0].data, old)
        received = ros_batch(wires, queue_size=configured['queue_size'])
        self.assertEqual([int(m.data[2]) for m in received], list(range(10, 20)))
        node = receiver()
        node.snapshot = node.snapshot_cache = old
        for message in received:
            self.feed(node, message.data)
        self.assertEqual(node.snapshot.header[2], 19)
        self.assertEqual(node.snapshot.header[14], 2)
        node.snapshot_resync_pub.publish.assert_not_called()

    def test_dropped_full_pauses_retries_then_recovers_with_matching_full_and_delta(self):
        node = receiver()
        self.feed(node, packet())
        old_field = node.snapshot_cache.distance
        missing = packet(sequence=3, field=2, full=False, value=2.)
        self.feed(node, missing, monotonic=1.)
        self.assertIsNone(node.snapshot)
        self.assertIs(node.snapshot_cache.distance, old_field)
        self.assertEqual(node.input_errors['collision_snapshot'], 'collision_snapshot_decode_failed')
        self.assertIsInstance(node.snapshot_resync_pub.publish.call_args.args[0], Empty)
        self.feed(node, missing, monotonic=1.1)
        self.assertEqual(node.snapshot_resync_pub.publish.call_count, 1)
        # A response can itself be dropped by the latest-only publishing end.
        self.feed(node, missing, monotonic=1.251)
        self.assertEqual(node.snapshot_resync_pub.publish.call_count, 2)
        self.feed(node, packet(sequence=4, field=2, value=2.), monotonic=1.26)
        new_field = node.snapshot.distance
        self.assertIsNot(new_field, old_field)
        self.feed(node, packet(sequence=5, field=2, full=False), monotonic=1.27)
        self.assertIs(node.snapshot.distance, new_field)
        self.assertIs(node.snapshot_cache, node.snapshot)
        self.assertNotIn('collision_snapshot', node.input_errors)
        self.assertIsNone(node.snapshot_error_detail)
        self.assertEqual(node.snapshot_resync_pub.publish.call_count, 2)

    def test_version_crop_resolution_shape_and_copy_stamp_must_match(self):
        full = Snapshot.decode(packet())
        delta = packet(sequence=2, full=False)
        for index in (5, 6, 7, 8, 9, 10, 11, 14, 17):
            changed = delta.copy()
            changed[index] += 1. if index in (5, 6, 7, 14) else .01
            with self.subTest(header_index=index), self.assertRaises(ValueError):
                Snapshot.decode(changed, full)
        self.assertIs(Snapshot.decode(delta, full).distance, full.distance)

    def test_malformed_message_does_not_destroy_the_only_field_cache(self):
        node = receiver()
        self.feed(node, packet())
        cached = node.snapshot_cache
        self.feed(node, packet(sequence=2)[:-1])
        self.assertIsNone(node.snapshot)
        self.assertIs(node.snapshot_cache, cached)
        self.assertEqual(node.input_errors['collision_snapshot'], 'collision_snapshot_decode_failed')
        self.feed(node, packet(sequence=3, full=False))
        self.assertIs(node.snapshot.distance, cached.distance)
        self.assertNotIn('collision_snapshot', node.input_errors)

    def test_resync_is_throttled_by_monotonic_time_at_250_ms(self):
        node = receiver()
        for clock, expected in ((10., 1), (10.1, 1), (10.249, 1),
                                (10.25, 2), (10.499, 2), (10.5, 3)):
            with self.subTest(monotonic=clock), \
                    patch('whole_body_mppi.time.monotonic', return_value=clock):
                node.request_snapshot_resync()
                self.assertEqual(node.snapshot_resync_pub.publish.call_count, expected)

    def test_partial_simple_namespace_fixture_initializes_cache_on_first_full(self):
        node = receiver(simple=True)
        self.feed(node, packet())
        self.assertIs(node.snapshot_cache, node.snapshot)
        self.feed(node, packet(sequence=2, full=False))
        self.assertEqual(node.snapshot.header[2], 2)

    def test_decode_failure_capture_and_reject_stop_without_setting_or_clearing_fault(self):
        for existing_fault in (False, True):
            with self.subTest(existing_fault=existing_fault):
                node = receiver()
                if existing_fault:
                    with patch('whole_body_mppi.rospy.logwarn'):
                        node.reject('no_feasible_candidate')
                calls_before = list(node.fault_pub.publish.call_args_list)
                node.base_pub.reset_mock()
                self.feed(node, packet(field=2, full=False))
                with self.assertRaisesRegex(ValueError, '^collision_snapshot_decode_failed$'):
                    node.capture(100.1)
                with patch('whole_body_mppi.rospy.logwarn'):
                    node.reject(node.input_errors['collision_snapshot'])
                self.assertEqual(node.fault_pub.publish.call_args_list, calls_before)
                self.assertEqual(node.reference.revision, 7)
                self.assertTrue(node.reference.active)
                node.arm_pub.publish.assert_not_called()
                command = node.base_pub.publish.call_args.args[0]
                self.assertEqual((command.linear.x, command.linear.y, command.angular.z), (0., 0., 0.))
                status = json.loads(node.info_pub.publish.call_args.args[0].data)
                self.assertEqual(status['state'], 'collision_snapshot_decode_failed')
                self.assertIn('delta without identical cached ESDF', status['snapshot_error'])

    def test_full_resync_and_cached_delta_do_not_extend_original_map_ttl(self):
        node = receiver()
        self.feed(node, packet(), now=100.1)
        fusion, copied = node.snapshot.header[[3, 17]]
        self.feed(node, packet(sequence=2, field=2, full=False), now=100.3)
        self.feed(node, packet(sequence=3), now=100.4)
        self.feed(node, packet(sequence=4, full=False), now=100.4)
        self.assertEqual(tuple(node.snapshot.header[[3, 17]]), (fusion, copied))
        self.assertTrue(node.snapshot.fresh(100.349))
        self.assertFalse(node.snapshot.fresh(100.351))
        node.base_stamp = node.joint_stamp = 100.4
        with self.assertRaisesRegex(ValueError, '^collision_snapshot_unavailable$'):
            node.capture(100.4)

    def test_full_resync_does_not_make_stale_dynamic_observations_fresh(self):
        node = receiver()
        data = packet(dynamic=True, fusion=100.3)
        data[18] = 99.9
        self.feed(node, data, now=100.31)
        self.assertEqual(node.snapshot.header[18], 99.9)
        self.assertFalse(node.snapshot.fresh(100.31))
        node.base_stamp = node.joint_stamp = 100.31
        with self.assertRaisesRegex(ValueError, '^collision_snapshot_unavailable$'):
            node.capture(100.31)


if __name__ == '__main__':
    unittest.main(verbosity=2)
