"""Exercise the copied bridge's publication boundary with offline ROS stubs."""
import importlib.util
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

RELEASE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RELEASE/'runtime'))
from odometry_continuity import OdometryContinuityGate


class Stamp:
    def __init__(self, value): self.value = value
    def to_sec(self): return self.value
    def __sub__(self, other): return Stamp(self.value-other.value)


def vector(): return NS(x=0., y=0., z=0.)
def quaternion(): return NS(x=0., y=0., z=0., w=1.)
def odometry(stamp=100., x=0., y=0.):
    p = vector(); p.x = x; p.y = y
    return NS(header=NS(stamp=Stamp(stamp), frame_id=''), child_frame_id='',
              pose=NS(pose=NS(position=p, orientation=quaternion())),
              twist=NS(covariance=[0.]*36, twist=NS(linear=vector(), angular=vector())))
def transform():
    return NS(header=NS(stamp=None, frame_id=''), child_frame_id='',
              transform=NS(translation=vector(), rotation=quaternion()))


class BridgeGateTests(unittest.TestCase):
    def setUp(self):
        self.ros = NS(get_time=Mock(return_value=100.02), logerr_throttle=Mock())
        stubs = {'rospy':self.ros, 'geometry_msgs':NS(),
            'geometry_msgs.msg':NS(TransformStamped=transform), 'nav_msgs':NS(),
            'nav_msgs.msg':NS(Odometry=odometry), 'tf2_ros':NS(),
            'std_msgs':NS(), 'std_msgs.msg':NS(String=lambda **kwargs: NS(**kwargs))}
        with patch.dict(sys.modules, stubs):
            spec = importlib.util.spec_from_file_location('offline_feedback02_bridge',
                RELEASE/'sensors/bin/feedback02_fastlio_to_odom.py')
            self.module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(self.module)
        self.bridge = self.module.FastLIOToOdomBridge.__new__(self.module.FastLIOToOdomBridge)
        b = self.bridge
        b._gate_lock = threading.Lock(); b._gate = OdometryContinuityGate()
        b._validation_pub = Mock(); b.odom_pub = Mock(); b.tf_broadcaster = Mock()
        b._last_stamp = Stamp(0.); b._last_base_pose = None
        b._filtered_twist = [0.,0.,0.]; b.planar_base_link = True
        b.base_z = .115; b.lidar_offset = (.28,0.,.002)
        b.max_derived_linear_speed = 1.; b.max_derived_angular_speed = 3.
        b.twist_filter_alpha = .35

    def feed(self, stamp, x=0., y=0., receipt=10.):
        self.ros.get_time.return_value = stamp+.02
        with patch.object(self.module.time, 'monotonic', return_value=receipt):
            self.bridge.cb(odometry(stamp,x,y))

    def assert_output_count(self, count):
        self.assertEqual(self.bridge.odom_pub.publish.call_count, count)
        self.assertEqual(self.bridge.tf_broadcaster.sendTransform.call_count, count)

    def test_fresh_clipped_twist_cannot_hide_pose_jump_or_resume(self):
        self.feed(100.)
        self.assert_output_count(1)
        self.feed(100.1, 200., 137., 10.1)
        self.feed(100.2, receipt=10.2)
        self.assert_output_count(1)
        self.assertEqual(self.bridge._gate.fault, 'position_jump')

    def test_55_second_input_gap_blocks_all_new_odom_and_tf(self):
        self.feed(100.)
        self.feed(155., 190., 137., 65.)
        self.assert_output_count(1)
        self.assertEqual(self.bridge._gate.fault, 'measurement_or_receipt_gap')

    def test_fresh_initial_diverged_source_never_publishes(self):
        self.feed(100., 22205., 31933.)
        self.assert_output_count(0)
        self.assertTrue(self.bridge._gate.status()['requires_restart'])


if __name__ == '__main__':
    unittest.main()
