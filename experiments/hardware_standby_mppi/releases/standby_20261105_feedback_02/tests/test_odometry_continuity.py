import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'runtime'))
from odometry_continuity import OdometryContinuityGate


def yaw(angle):
    return (0.0, 0.0, math.sin(angle/2), math.cos(angle/2))


class OdometryContinuityTests(unittest.TestCase):
    def sample(self, gate, stamp=100.0, position=(0., 0., 0.), angle=0., receipt=10.):
        return gate.validate(stamp, stamp+.02, receipt, position, yaw(angle))

    def test_stationary_noise_and_slow_motion(self):
        g = OdometryContinuityGate()
        for i in range(100):
            self.sample(g, 100+i*.1, (i*.005, .002*math.sin(i), 0.),
                        .002*math.cos(i), 10+i*.1)
        self.assertEqual(g.accepted, 100)

    def test_observed_55_second_gap_and_200_metre_jump_latches(self):
        g = OdometryContinuityGate()
        self.sample(g)
        with self.assertRaisesRegex(ValueError, 'measurement_or_receipt_gap'):
            self.sample(g, 155., (190., 137., 0.), receipt=65.)
        with self.assertRaisesRegex(ValueError, 'requires_restart'):
            self.sample(g, 155.1, receipt=65.1)
        self.assertEqual(g.accepted, 1)

    def test_fresh_huge_pose_rejected_even_without_velocity_input(self):
        g = OdometryContinuityGate()
        self.sample(g)
        with self.assertRaisesRegex(ValueError, 'position_jump'):
            self.sample(g, 100.1, (200., 0., 0.), receipt=10.1)

    def test_new_observer_cannot_accept_old_diverged_source(self):
        with self.assertRaisesRegex(ValueError, 'initial_pose_outside_local_origin'):
            self.sample(OdometryContinuityGate(), position=(22205., 31933., .115))

    def test_receipt_pause_rejected_even_for_adjacent_measurement_stamps(self):
        g = OdometryContinuityGate()
        self.sample(g)
        with self.assertRaisesRegex(ValueError, 'measurement_or_receipt_gap'):
            self.sample(g, 100.1, receipt=11.)

    def test_quaternion_sign_and_yaw_wrap_are_continuous(self):
        g = OdometryContinuityGate()
        self.sample(g, angle=math.pi-.01)
        self.sample(g, 100.1, angle=-math.pi+.01, receipt=10.1)
        q = tuple(-v for v in yaw(-math.pi+.01))
        self.assertTrue(g.validate(100.2, 100.22, 10.2, (0, 0, 0), q))

    def test_orientation_jump_latches(self):
        g = OdometryContinuityGate()
        self.sample(g)
        with self.assertRaisesRegex(ValueError, 'orientation_jump'):
            self.sample(g, 100.1, angle=1., receipt=10.1)

    def test_replay_and_out_of_order_latch(self):
        for stamp in (100., 99.9):
            g = OdometryContinuityGate()
            self.sample(g)
            with self.assertRaisesRegex(ValueError, 'nonmonotonic_measurement_stamp'):
                self.sample(g, stamp, receipt=10.1)

    def test_stale_future_and_invalid_pose_latch(self):
        for stamp, now, position, q in ((100, 100.3, (0,0,0), yaw(0)),
                (100, 99.9, (0,0,0), yaw(0)),
                (100, 100.02, (float('nan'),0,0), yaw(0)),
                (100, 100.02, (0,0,0), (0,0,0,0))):
            g = OdometryContinuityGate()
            with self.assertRaisesRegex(ValueError, 'requires_restart'):
                g.validate(stamp, now, 10, position, q)

    def test_monotonic_clock_regression_latches(self):
        g = OdometryContinuityGate()
        self.sample(g)
        with self.assertRaisesRegex(ValueError, 'monotonic_clock_regression'):
            self.sample(g, 100.1, receipt=9.9)

    def test_watchdog_latches_silence_but_allows_startup_waiting(self):
        g = OdometryContinuityGate()
        g.expire(200.)
        self.sample(g)
        g.expire(10.4)
        with self.assertRaisesRegex(ValueError, 'measurement_receipt_timeout'):
            g.expire(10.6)


if __name__ == '__main__':
    unittest.main()
