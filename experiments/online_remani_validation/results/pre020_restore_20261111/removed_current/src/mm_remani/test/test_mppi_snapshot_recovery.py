"""Collision transport recovery through the real callback; no ROS master/CAN."""
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from whole_body_mppi import WholeBodyNode


def packet(version=7, stamp=100., full=True):
    h = np.zeros(32)
    h[:2] = [3., 1.]
    h[3:5] = [stamp, 1.]
    h[5:8] = [8, 8, 8]
    h[11:17] = [.1, 47, 0, version, .35, 2.]
    h[18:21] = [stamp, 1., float(full)]
    h[28] = 1.  # Enabled but observed empty dynamic layer.
    return np.r_[h, np.zeros(47*7), np.ones(512) if full else []]


class SnapshotRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.node = WholeBodyNode.__new__(WholeBodyNode)
        n = self.node
        n.lock = threading.RLock()
        n.base = np.zeros(5); n.joints = np.zeros(12)
        n.base_stamp = n.joint_stamp = 100.
        n.anchor = n.arm_anchor = n.snapshot = n.snapshot_cache = None
        n.input_errors = {}

    def feed(self, data):
        self.node.on_snapshot(SimpleNamespace(data=data))

    def capture(self, now):
        self.node.base_stamp = self.node.joint_stamp = now
        with patch('whole_body_mppi.rospy.get_time', return_value=now):
            return self.node.capture(now)

    def assert_unavailable(self, now=100.):
        with self.assertRaisesRegex(ValueError, '^collision_snapshot_unavailable$'):
            self.capture(now)

    def test_malformed_packet_revokes_execution_but_fresh_matching_delta_recovers(self):
        self.feed(packet())
        cached = self.node.snapshot
        self.feed(packet()[:-1])
        self.assertIsNone(self.node.snapshot)
        self.assertIs(self.node.snapshot_cache, cached)
        self.assert_unavailable()
        self.feed(packet(stamp=100.1, full=False))
        self.assertIs(self.node.snapshot.distance, cached.distance)
        self.assertNotIn('collision_snapshot', self.node.input_errors)
        self.assertIs(self.capture(100.1)[1], self.node.snapshot)

    def test_missed_new_full_waits_for_that_full_not_old_cached_geometry(self):
        self.feed(packet())
        cached = self.node.snapshot
        for stamp in (100.05, 100.1, 100.15):
            self.feed(packet(version=8, stamp=stamp, full=False))
            self.assertIsNone(self.node.snapshot)
            self.assertIs(self.node.snapshot_cache, cached)
            self.assert_unavailable(stamp)
        self.feed(packet(version=8, stamp=100.2))
        self.assertIsNot(self.node.snapshot_cache.distance, cached.distance)
        self.assertEqual(self.capture(100.2)[1].header[14], 8)

    def test_grid_change_never_borrows_same_version_cached_field(self):
        self.feed(packet())
        changed = packet(stamp=100.1, full=False)
        changed[8] += .1
        self.feed(changed)
        self.assert_unavailable(100.1)

    def test_decoded_cache_does_not_extend_source_age_or_override_invalid_health(self):
        self.feed(packet())
        self.feed(packet()[:-1])
        self.feed(packet(full=False))
        self.assert_unavailable(100.36)
        invalid = packet(stamp=100.4, full=False)
        invalid[4] = 0.
        self.feed(invalid)
        self.assert_unavailable(100.4)

    def test_missing_field_stops_locally_without_cancelling_valid_reference(self):
        self.feed(packet(version=8, full=False))
        n = self.node
        n.stop_base = Mock(); n.fault_pub = Mock(); n.info_pub = Mock()
        n.execute = True; n.sent_command = True; n.sent_navigation_goal = 99.
        n.reference = SimpleNamespace(lock=threading.RLock(), active=True, goal_stamp=99.)
        n.cycle_began = time.monotonic()
        with patch('whole_body_mppi.rospy.logwarn'):
            try:
                self.capture(100.)
            except ValueError as exc:
                n.reject(str(exc))
            else:
                self.fail('A missing collision field must not authorize execution')
        n.stop_base.assert_called_once_with()
        n.fault_pub.publish.assert_not_called()
        report = json.loads(n.info_pub.publish.call_args[0][0].data)
        self.assertEqual(report['state'], 'collision_snapshot_unavailable')
        self.assertEqual(report['input_error'], 'delta without identical cached ESDF')


if __name__ == '__main__':
    unittest.main()
