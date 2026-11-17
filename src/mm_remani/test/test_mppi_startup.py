"""Startup authorization/geometry regressions; no ROS master, CAN or motion."""
from functools import lru_cache
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'scripts'))
from whole_body_mppi import WholeBodyNode, startup_plan
from active_arm_model import PiperArmModel
from active_arm_mppi import WholeBodyMPPI
from native_reference_adapter import NativeReferenceAdapter

Q = np.array([0., .520979, -1.048914, 0., .947297, 0.])


@lru_cache(maxsize=1)
def geometry():
    def merge(dst, source):
        for key, value in source.items():
            if isinstance(value, dict) and isinstance(dst.get(key), dict):
                merge(dst[key], value)
            else:
                dst[key] = value
    parameters = {}
    for name in ('exp0_param.yaml', 'piper_param.yaml', 'remani_planner_param.yaml', 'fixed_collision_spheres_b.yaml'):
        merge(parameters, yaml.safe_load((PACKAGE.parent/'mm_remani_planner/plan_manage/config'/name).read_text()))
    model = PiperArmModel.from_parameters(parameters)
    model.lower[5], model.upper[5] = -.35, .35
    return model, yaml.safe_load((PACKAGE/'config/mppi.yaml').read_text())


def kernel():
    model, config = geometry()
    return WholeBodyMPPI(model, config['base_limits'],
        (model.lower, model.upper, config['arm_velocity'], config['arm_acceleration']), [],
        horizon=8, step=.1, random_samples=4, arm_position_servo=False)


class Environment:
    def __init__(self, point=None):
        self.point = point
        self.valid = True

    def fresh(self, now):
        return self.valid

    def query(self, spheres, times):
        gap = np.ones(len(spheres)) if self.point is None else (
            np.linalg.norm(spheres[..., :3]-self.point, axis=-1)-spheres[..., 3]-.01).min(axis=-1)
        return gap, np.zeros(len(spheres))


class StartupGeometryTests(unittest.TestCase):
    def setUp(self):
        self.kernel = kernel()
        self.state = np.r_[np.zeros(5), Q, np.zeros(6)]
        self.target = Q.copy(); self.target[0] = .6

    def plan(self, environment):
        return startup_plan(self.kernel, self.state, self.target, environment, 100., preflight=True)

    def test_clear_path_uses_actual_geometry_and_zero_base_controls(self):
        answer, info = self.plan(Environment())
        self.assertEqual(len(self.kernel.model.fixed_spheres), 47)
        self.assertEqual(info['feasible'], 1)
        controls, states = answer
        np.testing.assert_array_equal(controls[:, :2], 0.)
        np.testing.assert_array_equal(states[:, :5], 0.)
        self.assertGreater(states[-1, 5], self.state[5])

    def test_external_obstacle_at_current_pose_rejects_preflight(self):
        point = self.kernel.spheres(self.state[None])[0, 61, :3]
        with self.assertRaisesRegex(ValueError, 'startup_joint_path_obstacle_rejected'):
            self.plan(Environment(point))

    def test_clear_endpoints_do_not_hide_an_obstacle_inside_joint_path(self):
        middle = self.state.copy(); middle[5] = .3
        point = self.kernel.spheres(middle[None])[0, 61, :3]
        endpoint = self.state.copy(); endpoint[5:11] = self.target
        environment = Environment(point)
        gap, _ = environment.query(self.kernel.spheres(np.array([self.state, endpoint])), [100., 100.])
        self.assertTrue(np.all(gap > .02))
        with self.assertRaisesRegex(ValueError, 'startup_joint_path_obstacle_rejected'):
            self.plan(environment)

    def test_configuration_collision_at_current_pose_is_rejected(self):
        self.state[5:11] = [0., 1., 0., 0., 1., 0.]
        self.assertLess(self.kernel.model.configuration_gaps_batch(self.state[None, 5:11])[0], 0.)
        with self.assertRaisesRegex(ValueError, 'startup_joint_path_configuration_rejected'):
            self.plan(Environment())

    def test_moving_base_does_not_start_arm_initialization(self):
        self.state[3] = .02
        with self.assertRaisesRegex(ValueError, 'startup_waiting_stationary_base'):
            self.plan(Environment())

    def test_recorded_folded_pose_can_start_but_external_wrist_obstacle_still_blocks(self):
        self.state[5:11] = np.deg2rad([-2.447, -1.909, .943, 5.992, 26.168, -5.879])
        self.target = self.kernel.model.q_nominal.copy()
        answer, info = self.plan(Environment())
        self.assertIsNotNone(answer)
        self.assertEqual(info['feasible'], 1)
        controls, _ = answer
        np.testing.assert_array_equal(controls[:, :2], 0.)
        wrist = self.kernel.spheres(self.state[None])[0, 47+18, :3]
        with self.assertRaisesRegex(ValueError, 'startup_joint_path_obstacle_rejected'):
            self.plan(Environment(wrist))


class StartupCycleTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        node = self.node = WholeBodyNode.__new__(WholeBodyNode)
        node.lock = threading.RLock(); node.execute = True; node.simulation = False
        node.kernel = kernel(); node.model = node.kernel.model; node.limits = node.kernel.limits
        node.base = np.zeros(5); node.joints = np.r_[Q, np.zeros(6)]
        node.base_stamp = node.joint_stamp = self.now
        node.snapshot = Environment(); node.anchor = np.zeros(3); node.arm_anchor = Q.copy()
        node.startup_complete = False; node.startup_target = Q.copy(); node.startup_target[0] = .1
        node.startup_enable_requested = False; node.startup_settled_since = None
        node.enable_feedback_stamp = node.enable_feedback_until = 0.
        node.execution_stamp = node.execution_valid_until = 0.
        node.execution_enabled = node.execution_latched = False; node.execution_revision = 0
        node.safety_speed_limit = None; node.deadline = 1.; node.last_visual = float('inf')
        node.arm_enabled = True; node.seed = 11; node.sent_command = False; node.sent_navigation_goal = None
        node.input_errors = {}
        for name in ('base_pub', 'arm_pub', 'enable_pub', 'fault_pub', 'info_pub'):
            setattr(node, name, Mock())
        node.enable_pub.get_num_connections.return_value = 1
        node.reference = NativeReferenceAdapter(node.kernel, navigation_kernel=node.kernel)
        state = np.r_[node.base, node.joints]
        controls = np.tile(np.r_[.02, np.zeros(7)], (node.kernel.H, 1))
        node.reference.plan = Mock(return_value=((controls, np.tile(state, (2, 1))), {'feasible': 1}))

    def feed(self, now, enabled=False, valid=True, stopped=False, stamp=None, command_active=False):
        self.now = now; self.node.base_stamp = self.node.joint_stamp = now
        source = now if stamp is None else stamp
        msg = NS(header=NS(stamp=NS(to_sec=lambda: source)),
                 valid_until=NS(to_sec=lambda: source+.15),
                 enabled=enabled, feedback_valid=valid, stop_latched=stopped,
                 command_active=command_active)
        with patch('whole_body_mppi.rospy.get_time', return_value=now):
            self.node.on_execution_state(msg)

    def cycle(self, hook=None):
        def plan(*args, **kwargs):
            answer = startup_plan(*args, **kwargs)
            if hook is not None:
                hook()
            return answer
        with patch('whole_body_mppi.rospy.get_time', side_effect=lambda: self.now), \
                patch('whole_body_mppi.rospy.loginfo'), \
                patch('whole_body_mppi.startup_plan', side_effect=plan) as planned:
            self.node.cycle()
        return planned

    def status(self):
        return json.loads(self.node.info_pub.publish.call_args.args[0].data)

    def assert_zero_base(self):
        for call in self.node.base_pub.publish.call_args_list:
            command = call.args[0]
            self.assertEqual((command.linear.x, command.linear.y, command.angular.z), (0., 0., 0.))

    def test_first_joint_sample_cannot_hide_missing_odometry_after_feedback_recovers(self):
        from sensor_msgs.msg import JointState
        import rospy
        msg = JointState()
        msg.header.stamp = rospy.Time.from_sec(self.now)
        msg.name = ['joint%d' % i for i in range(1, 7)]
        msg.position = Q.tolist()
        self.node.joint_stamp = 0.
        with patch('whole_body_mppi.rospy.get_time', return_value=self.now):
            self.node.on_joints(msg)
            self.assertIsNone(self.node.joints)
            self.assertEqual(self.node.input_errors['joint_feedback'], 'joint_feedback_incomplete')
            msg.velocity = [0.]*6
            self.node.on_joints(msg)
        self.assertNotIn('joint_feedback', self.node.input_errors)
        self.node.base = None
        with self.assertRaisesRegex(ValueError, '^waiting_odometry$'):
            self.node.capture(self.now)
        self.node.base = np.zeros(5)
        self.node.snapshot = None
        with self.assertRaisesRegex(ValueError, '^waiting_collision_snapshot$'):
            self.node.capture(self.now)

    def test_preflight_requests_enable_once_without_sending_an_arm_command(self):
        self.feed(100.)
        planned = self.cycle()
        self.assertTrue(planned.call_args.kwargs['preflight'])
        self.node.enable_pub.publish.assert_called_once()
        self.assertTrue(self.node.enable_pub.publish.call_args.args[0].data)
        self.node.arm_pub.publish.assert_not_called()
        self.assertEqual(self.status()['state'], 'startup_enable_requested')
        for now in (100.05, 100.1):
            self.feed(now); self.cycle()
        self.node.enable_pub.publish.assert_called_once()
        self.node.arm_pub.publish.assert_not_called()
        self.assert_zero_base()

    def test_latched_startup_reports_stop_without_replanning_or_enabling(self):
        self.feed(100., stopped=True)
        # The stop must remain the reported cause even if another input expires.
        self.node.snapshot.valid = False
        planned = self.cycle()
        planned.assert_not_called()
        self.assertEqual(self.status()['state'], 'arm_stop_latched')
        self.node.arm_pub.publish.assert_not_called()
        self.node.enable_pub.publish.assert_not_called()
        self.assert_zero_base()

    def test_rejected_startup_reports_external_collision_diagnostics(self):
        self.feed(100., enabled=True)
        self.node.startup_enable_requested = True
        state = np.r_[self.node.base, self.node.joints]
        self.node.snapshot.point = self.node.kernel.spheres(state[None])[0, 61, :3]
        self.cycle()
        status = self.status()
        self.assertEqual(status['state'], 'no_feasible_candidate')
        self.assertEqual(status['candidate_diagnostics']['first_external_violation_s'], [0.])
        self.assertEqual(status['candidate_diagnostics']['feasible'], 0)
        self.node.arm_pub.publish.assert_not_called()
        self.assert_zero_base()

    def test_unhealthy_stale_latched_feedback_or_disconnected_driver_never_enables(self):
        for case in ('stale', 'abnormal', 'latched', 'disconnected'):
            with self.subTest(case=case):
                self.setUp()
                self.feed(100., valid=case != 'abnormal', stopped=case == 'latched',
                          stamp=99. if case == 'stale' else 100.)
                if case == 'disconnected': self.node.enable_pub.get_num_connections.return_value = 0
                self.cycle()
                self.node.enable_pub.publish.assert_not_called()
                self.node.arm_pub.publish.assert_not_called()
                self.assert_zero_base()

    def test_failed_collision_or_freshness_preflight_never_enables(self):
        for case in ('obstacle', 'map_stale', 'joint_stale'):
            with self.subTest(case=case):
                self.setUp(); self.feed(100.)
                if case == 'obstacle':
                    state = np.r_[self.node.base, self.node.joints]
                    self.node.snapshot.point = self.node.kernel.spheres(state[None])[0, 61, :3]
                elif case == 'map_stale': self.node.snapshot.valid = False
                else: self.node.joint_stamp = 99.
                self.cycle()
                self.node.enable_pub.publish.assert_not_called()
                self.node.arm_pub.publish.assert_not_called()
                self.assert_zero_base()

    def test_enabled_feedback_requires_a_later_cycle_and_new_command_stamp(self):
        self.feed(100.); self.cycle()
        self.node.arm_pub.publish.assert_not_called()
        self.feed(100.01, enabled=True)
        self.now = 100.02; self.cycle()
        self.node.arm_pub.publish.assert_called_once()
        stamp = self.node.arm_pub.publish.call_args.args[0].header.stamp.to_sec()
        self.assertAlmostEqual(stamp, 100.02, places=8)
        self.assertGreater(stamp, 100.01)
        self.node.enable_pub.publish.assert_called_once()
        self.assert_zero_base()

    def test_older_healthy_feedback_cannot_restore_pre_enable_permission(self):
        self.feed(100., enabled=False)
        self.feed(100.04, enabled=False, valid=False)
        self.feed(100.05, enabled=False, stamp=100.02)
        self.assertEqual(self.node.enable_feedback_stamp, 100.04)
        self.assertEqual(self.node.enable_feedback_until, 0.)
        self.cycle()
        self.node.enable_pub.publish.assert_not_called()
        self.node.arm_pub.publish.assert_not_called()
        self.feed(100.06, enabled=False)
        self.cycle()
        self.node.enable_pub.publish.assert_called_once()
        self.node.arm_pub.publish.assert_not_called()
        self.assert_zero_base()

    def test_enable_arriving_during_preflight_cannot_authorize_that_plan(self):
        self.feed(100.)
        self.cycle(hook=lambda: self.feed(100.01, enabled=True))
        self.node.arm_pub.publish.assert_not_called()
        self.now = 100.02; self.cycle()
        self.node.arm_pub.publish.assert_called_once()
        self.assert_zero_base()

    def test_startup_ignores_navigation_goal_and_finish_messages(self):
        self.node.reference.feed_goal = Mock()
        with patch('whole_body_mppi.rospy.logwarn_throttle'):
            self.node.on_goal(NS())
        self.node.on_finish(NS(data=True))
        self.node.reference.feed_goal.assert_not_called()
        self.assertFalse(self.node.startup_complete)
        self.feed(100., enabled=True); self.cycle()
        self.node.reference.plan.assert_not_called()
        self.assert_zero_base()

    def test_measured_pose_and_speed_settle_continuously_before_reset_and_normal_mppi(self):
        node = self.node
        node.reference.active = True; node.reference.goal_stamp = 99.
        node.reference.native.refs = [np.zeros((2, 4))]
        node.base[:3] = [.03, -.01, .02]
        node.joints[:6] = node.startup_target
        self.feed(100., enabled=True); self.cycle()
        self.assertFalse(node.startup_complete)
        node.joints[6] = .03
        self.feed(100.2, enabled=True); self.cycle()
        self.assertIsNone(node.startup_settled_since)
        node.joints[6:] = 0.
        for now in (100.21, 100.50):
            self.feed(now, enabled=True); self.cycle(); self.assertFalse(node.startup_complete)
        self.feed(100.52, enabled=True); self.cycle()
        self.assertTrue(node.startup_complete)
        self.assertFalse(node.reference.active); self.assertEqual(node.reference.native.refs, [])
        np.testing.assert_array_equal(node.anchor, node.base[:3])
        np.testing.assert_array_equal(node.arm_anchor, node.joints[:6])
        self.assert_zero_base()
        self.feed(100.54, enabled=True); self.cycle()
        node.reference.plan.assert_called_once()
        self.assertEqual(node.base_pub.publish.call_args.args[0].linear.x, .02)

    def test_rejection_resets_the_measured_settling_interval(self):
        self.node.joints[:6] = self.node.startup_target
        self.feed(100., enabled=True); self.cycle()
        self.node.snapshot.valid = False
        self.feed(100.2, enabled=True); self.cycle()
        self.assertIsNone(self.node.startup_settled_since)
        self.node.snapshot.valid = True
        self.feed(100.4, enabled=True); self.cycle()
        self.assertFalse(self.node.startup_complete)
        self.feed(100.71, enabled=True); self.cycle()
        self.assertTrue(self.node.startup_complete)

    def test_observe_only_neither_enables_nor_completes_startup(self):
        self.node.execute = False; self.node.joints[:6] = self.node.startup_target
        for now in (100., 100.4):
            self.feed(now); self.cycle()
        self.node.enable_pub.publish.assert_not_called()
        self.node.arm_pub.publish.assert_not_called()
        self.assertFalse(self.node.startup_complete)
        self.assertEqual(self.status()['state'], 'candidate_only')

    def test_already_normal_mode_does_not_run_startup_or_publish_enable(self):
        self.node.startup_complete = True
        self.feed(100., enabled=True)
        planned = self.cycle()
        planned.assert_not_called()
        self.node.enable_pub.publish.assert_not_called()
        self.node.reference.plan.assert_called_once()
        self.node.arm_pub.publish.assert_called_once()
        self.assertEqual(self.node.base_pub.publish.call_args.args[0].linear.x, .02)

    def test_map_pause_and_fresh_driver_hold_resume_without_reenabling(self):
        self.node.startup_complete = True
        self.feed(100., enabled=True, command_active=True)
        self.cycle()
        self.assertEqual(self.node.arm_pub.publish.call_count, 1)
        self.node.snapshot.valid = False
        self.feed(100.4, enabled=True, command_active=False)
        self.cycle()
        self.assertEqual(self.status()['state'], 'collision_snapshot_unavailable')
        self.assertEqual(self.node.arm_pub.publish.call_count, 1)
        self.assertFalse(self.node.execution_latched)
        self.assertTrue(self.node.execution_ready(self.now))
        self.assertEqual(self.node.base_pub.publish.call_args.args[0].linear.x, 0.)
        self.node.snapshot.valid = True
        self.feed(100.5, enabled=True, command_active=False)
        self.cycle()
        self.assertEqual(self.status()['state'], 'command_delivered')
        self.assertEqual(self.node.arm_pub.publish.call_count, 2)
        self.node.enable_pub.publish.assert_not_called()


class NativeStartupCycleTests(unittest.TestCase):
    feed = StartupCycleTests.feed
    cycle = StartupCycleTests.cycle
    status = StartupCycleTests.status
    assert_zero_base = StartupCycleTests.assert_zero_base

    def setUp(self):
        StartupCycleTests.setUp(self)
        node = self.node
        node.native_arm_startup = True
        node.native_handoff_sent = False
        node.native_handoff_stamp = 0.
        node.execution_command_active = False
        node.startup_target = Q.copy()
        node.enable_pub = None

    def test_native_driver_can_start_without_map_or_mppi_plan(self):
        self.node.snapshot = None
        self.node.joints[:6] = np.deg2rad([-2.343, -1.960, 1.256, 5.970, 26.698, -5.864])
        self.feed(100., enabled=False)
        planned = self.cycle()
        planned.assert_not_called()
        self.node.reference.plan.assert_not_called()
        self.node.arm_pub.publish.assert_not_called()
        self.assertEqual(self.status()['state'], 'waiting_native_arm_startup')
        self.assert_zero_base()

    def test_ready_report_does_not_override_actual_pose_or_velocity(self):
        for part in ('position', 'velocity'):
            with self.subTest(part=part):
                self.setUp(); self.feed(100., enabled=True)
                self.node.joints[0 if part == 'position' else 6] += .05
                self.cycle()
                self.assertEqual(self.status()['state'], 'native_startup_standby_not_reached')
                self.node.reference.plan.assert_not_called()
                self.node.arm_pub.publish.assert_not_called()
                self.assert_zero_base()

    def test_first_mppi_command_waits_for_a_fresh_map(self):
        self.feed(100., enabled=True)
        self.node.snapshot.valid = False
        self.cycle()
        self.assertEqual(self.status()['state'], 'collision_snapshot_unavailable')
        self.node.arm_pub.publish.assert_not_called()
        self.assertFalse(self.node.native_handoff_sent)
        self.assertFalse(self.node.startup_complete)
        self.assert_zero_base()

    def test_first_command_is_checked_and_zero_base_until_driver_ack(self):
        self.feed(100., enabled=True)
        planned = self.cycle()
        planned.assert_not_called()
        self.node.reference.plan.assert_called_once()
        self.node.arm_pub.publish.assert_called_once()
        self.assertTrue(self.node.native_handoff_sent)
        self.assertFalse(self.node.startup_complete)
        self.feed(100.02, enabled=True, command_active=False)
        self.cycle()
        self.assertFalse(self.node.startup_complete)
        self.assert_zero_base()
        self.feed(100.04, enabled=True, command_active=True)
        self.cycle()
        self.assertTrue(self.node.startup_complete)
        self.assertEqual(self.node.base_pub.publish.call_args.args[0].linear.x, .02)

    def test_obstacle_prevents_handoff_even_if_native_driver_is_ready(self):
        self.feed(100., enabled=True)
        state = np.r_[self.node.base, self.node.joints]
        self.node.snapshot.point = self.node.kernel.spheres(state[None])[0, 61, :3]
        self.cycle()
        self.node.arm_pub.publish.assert_not_called()
        self.assertFalse(self.node.native_handoff_sent)
        self.assertIn('command_hold_or_braking_rejected', self.status()['state'])
        self.assert_zero_base()

    def test_stop_or_stale_ack_cannot_complete_startup(self):
        self.feed(100., enabled=True); self.cycle()
        self.feed(100.02, enabled=True, command_active=True, stamp=99.99)
        self.cycle()
        self.assertFalse(self.node.startup_complete)
        self.feed(100.04, enabled=True, command_active=True, stopped=True)
        planned = self.cycle()
        planned.assert_not_called()
        self.assertFalse(self.node.startup_complete)
        self.assertEqual(self.status()['state'], 'arm_stop_latched')
        self.assert_zero_base()

    def test_enable_status_change_during_handoff_rejects_delivery(self):
        self.feed(100., enabled=True)
        result = self.node.reference.plan.return_value
        def plan(*args, **kwargs):
            self.feed(100.01, enabled=False)
            return result
        self.node.reference.plan.side_effect = plan
        self.cycle()
        self.node.arm_pub.publish.assert_not_called()
        self.assertFalse(self.node.native_handoff_sent)
        self.assertFalse(self.node.startup_complete)
        self.assert_zero_base()

    def test_navigation_goals_wait_for_confirmed_handoff(self):
        self.feed(100., enabled=True); self.cycle()
        self.node.reference.feed_goal = Mock()
        with patch('whole_body_mppi.rospy.logwarn_throttle'):
            self.node.on_goal(NS())
        self.node.reference.feed_goal.assert_not_called()

    def test_preexisting_or_pre_delivery_active_status_is_not_a_new_ack(self):
        self.feed(100., enabled=True, command_active=True)
        self.cycle()
        self.node.arm_pub.publish.assert_not_called()
        self.assertEqual(self.status()['state'], 'native_startup_driver_already_claimed')
        self.setUp(); self.feed(100., enabled=True); self.cycle()
        self.node.native_handoff_stamp = 100.03
        self.feed(100.02, enabled=True, command_active=True)
        self.cycle()
        self.assertFalse(self.node.startup_complete)
        self.assert_zero_base()
        self.feed(100.04, enabled=True, command_active=True)
        self.cycle()
        self.assertTrue(self.node.startup_complete)


if __name__ == '__main__':
    unittest.main()
