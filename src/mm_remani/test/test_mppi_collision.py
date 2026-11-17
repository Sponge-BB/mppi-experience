"""Offline native snapshot/model checks; no ROS master, sockets or hardware."""
from pathlib import Path
import json
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'scripts'))
sys.path.insert(0, str(PACKAGE/'scripts'/'mppi'))
from active_arm_mppi import Snapshot, WholeBodyMPPI
from active_arm_model import PiperArmModel
from standby_whole_body_mppi import StandbyWholeBodyMPPI
from whole_body_mppi import checked_first_step, WholeBodyNode
import test_mppi_execution_gate as gates
import test_mppi_native_phase as phases
from native_reference_adapter import NativeReferenceAdapter


def wire(shape=(8, 8, 8), resolution=.1, dynamic=(), value=1., origin=(0., 0., 0.)):
    h = np.zeros(32)
    h[:2] = [3., 1.]
    h[3:5] = [100., 1.]
    h[5:8], h[8:11], h[11] = shape, origin, resolution
    h[12:17] = [47, len(dynamic), 7, .3, 2.]
    h[18:21] = [100., 1., 1.]
    h[28] = bool(len(dynamic))
    return np.r_[h, np.zeros(47*7), np.asarray(dynamic).ravel(),
                 np.full(int(np.prod(shape)), value)]


class CollisionTests(unittest.TestCase):
    def test_standby_rejection_stops_locally_without_latching_navigation_recovery(self):
        for active, delivered_goal, current_goal, fault_expected in (
                (False, None, None, False), (True, None, 10., False),
                (True, 9., 10., False), (True, 10., 10., True),
                (True, None, None, False)):
            node = SimpleNamespace(stop_base=Mock(), sent_command=True, fault_pub=Mock(),
                sent_navigation_goal=delivered_goal,
                reference=SimpleNamespace(active=active, goal_stamp=current_goal,
                                          lock=threading.RLock()), execute=True, info_pub=Mock(),
                cycle_began=time.monotonic())
            WholeBodyNode.reject(node, 'no_feasible_candidate')
            node.stop_base.assert_called_once_with()
            if fault_expected:
                self.assertTrue(node.fault_pub.publish.call_args[0][0].data)
            else:
                node.fault_pub.publish.assert_not_called()
            self.assertEqual(json.loads(node.info_pub.publish.call_args[0][0].data)['state'],
                             'no_feasible_candidate')

    def test_reference_transitions_stop_without_restarting_navigation_recovery(self):
        for reason in ('reference_changed_during_plan', 'reference_changed_before_delivery',
                       'native_reference_quiet_interval', 'native_reference_not_started',
                       'waiting_native_reference_after_goal', 'waiting_native_progress',
                       'waiting_native_progress_during_plan', 'waiting_native_progress_before_delivery',
                       'native_gear_changed_during_plan', 'native_gear_changed_before_delivery',
                       'native_reference_cancelled', 'native_reference_cancelled_before_delivery',
                       'native_reference_quiet_interval_before_delivery',
                       'native_reference_not_started_before_delivery',
                       'native_reference_expired', 'native_reference_expired_during_plan',
                       'native_reference_expired_before_delivery'):
            with self.subTest(reason=reason):
                node = SimpleNamespace(stop_base=Mock(), sent_command=True, fault_pub=Mock(),
                    arm_pub=Mock(), sent_navigation_goal=10.,
                    reference=SimpleNamespace(active=True, goal_stamp=10., lock=threading.RLock()),
                    execute=True, info_pub=Mock(), cycle_began=time.monotonic())
                WholeBodyNode.reject(node, reason)
                node.stop_base.assert_called_once_with()
                node.fault_pub.publish.assert_not_called()
                node.arm_pub.publish.assert_not_called()  # Existing watchdog must expire normally.
                self.assertEqual(json.loads(node.info_pub.publish.call_args[0][0].data)['state'], reason)

    def test_same_goal_real_failures_still_request_navigation_recovery(self):
        for reason in ('no_feasible_candidate', 'command_hold_or_braking_rejected_by_latest_state_or_map',
                       'arm_target_exceeds_current_feedback_lead', 'invalid_selected_arm_target',
                       'native_progress_stale', 'native_progress_stale_during_plan',
                       'arm_stop_latched', 'feedback_invalidated_during_plan',
                       'joint_feedback_outside_limits', 'native_world_odom_alignment_changed',
                       'unexpected_error'):
            with self.subTest(reason=reason):
                node = SimpleNamespace(stop_base=Mock(), sent_command=True, fault_pub=Mock(),
                    sent_navigation_goal=10.,
                    reference=SimpleNamespace(active=True, goal_stamp=10., lock=threading.RLock()),
                    execute=True, info_pub=Mock(), cycle_began=time.monotonic())
                WholeBodyNode.reject(node, reason)
                node.stop_base.assert_called_once_with()
                node.fault_pub.publish.assert_called_once()
                self.assertTrue(node.fault_pub.publish.call_args[0][0].data)

    def test_transient_rejections_stop_without_setting_or_clearing_an_existing_fault(self):
        reasons = ('planning_deadline_exceeded', 'delivery_deadline_exceeded',
                   'collision_snapshot_unavailable', 'collision_snapshot_expired_during_plan',
                   'state_feedback_stale', 'feedback_expired_during_plan',
                   'inputs_expired_before_delivery', 'waiting_explicit_arm_enable_or_normal_status')
        for existing_fault in (False, True):
            node = SimpleNamespace(stop_base=Mock(), sent_command=True, fault_pub=Mock(),
                arm_pub=Mock(), sent_navigation_goal=10.,
                reference=SimpleNamespace(active=True, goal_stamp=10., revision=7,
                                          lock=threading.RLock()),
                execute=True, info_pub=Mock(), cycle_began=time.monotonic())
            if existing_fault:
                WholeBodyNode.reject(node, 'no_feasible_candidate')
            previous_calls = list(node.fault_pub.publish.call_args_list)
            for reason in reasons:
                with self.subTest(existing_fault=existing_fault, reason=reason):
                    node.stop_base.reset_mock()
                    WholeBodyNode.reject(node, reason)
                    node.stop_base.assert_called_once_with()
                    node.arm_pub.publish.assert_not_called()
                    self.assertEqual(node.fault_pub.publish.call_args_list, previous_calls)
                    self.assertEqual(node.reference.revision, 7)
                    self.assertTrue(node.reference.active)
                    self.assertEqual(json.loads(node.info_pub.publish.call_args[0][0].data)['state'], reason)

    @classmethod
    def setUpClass(cls):
        import rospkg
        from roslaunch.config import ROSLaunchConfig
        from roslaunch.xmlloader import XmlLoader
        config = ROSLaunchConfig()
        # Parse only: no ROSLaunchParent, master, machine/network discovery or nodes.
        XmlLoader().load(str(PACKAGE/'launch'/'exp0_real.launch'), config,
                        argv=['use_mppi:=true', 'hardware_enabled:=false'], verbose=False)
        p = {key: value.value for key, value in config.params.items()}
        urdf = Path(rospkg.RosPack().get_path('my_description'))/'robot'/'mm.urdf'
        model = PiperArmModel.from_parameters(p, urdf, PACKAGE/'config'/'mppi_shoulder_wrist_proxy.json')
        model.lower[5] = max(model.lower[5], -.35)
        model.upper[5] = min(model.upper[5], .35)
        prefix = '/whole_body_mppi/'
        limits = {key.rsplit('/', 1)[-1]: value for key, value in p.items()
                  if key.startswith(prefix+'base_limits/')}
        kernel = StandbyWholeBodyMPPI(model, limits,
            (model.lower, model.upper, p[prefix+'arm_velocity'], p[prefix+'arm_acceleration']),
            p[prefix+'postures'], horizon=p[prefix+'horizon'], step=1./p[prefix+'rate_hz'],
            random_samples=p[prefix+'random_samples'])
        q = np.array([0., .520979, -1.048914, 0., .947297, 0.])
        state = np.r_[np.zeros(5), q, np.zeros(6)]
        cls.model, cls.kernel, cls.q, cls.state = model, kernel, q, state

    def test_dynamic_motion_and_quadratic_uncertainty(self):
        data = wire(resolution=.5, dynamic=[[2., .75, .75, .2, 0., 0., 1.]], value=10.)
        data[25:28] = [.08, .25, .75]
        snapshot = Snapshot.decode(data)
        age = np.array([0., .5, 1.])
        spheres = np.tile([[[.75, .75, .75, .1]]], (3, 1, 1))
        gap, _ = snapshot.query(spheres, 100.+age)
        uncertainty = .08 + .25*age + .5*.75*age**2
        np.testing.assert_allclose(gap, 1.25+.2*age-uncertainty-.1-.5)
        self.assertTrue(snapshot.fresh(100.1))
        self.assertFalse(snapshot.fresh(100.31))
        beyond, _ = snapshot.query(spheres[:1], 102.1)
        self.assertLess(beyond[0], -100.)

    def test_delta_reuses_only_identical_version_and_grid(self):
        data = wire()
        full = Snapshot.decode(data)
        delta = data[:32+47*7].copy()
        delta[3], delta[20] = 100.1, 0.
        self.assertIs(Snapshot.decode(delta, full).distance, full.distance)
        with self.assertRaises(ValueError):
            Snapshot.decode(delta)
        for index in (8, 11, 14):
            changed = delta.copy(); changed[index] += .01
            with self.subTest(changed_header=index), self.assertRaises(ValueError):
                Snapshot.decode(changed, full)

    def test_bad_dimensions_and_incomplete_wire_rejected(self):
        data = wire()
        for index, value in ((5, 1.), (5, 2.5), (5, 500001.), (11, 0.),
                             (12, 0.), (13, -1.), (25, -.1), (3, np.nan)):
            changed = data.copy(); changed[index] = value
            with self.subTest(index=index, value=value), self.assertRaises(ValueError):
                Snapshot.decode(changed)
        with self.assertRaises(ValueError):
            Snapshot.decode(data[:-1])

    def test_disabled_dynamic_layer_accepts_fresh_static_snapshot_without_fake_stamp(self):
        data = wire()
        data[18], data[19], data[28] = 0., 0., 0.
        snapshot = Snapshot.decode(data)
        self.assertTrue(snapshot.fresh(100.1))
        self.assertFalse(snapshot.fresh(100.31))
        # A stale enabled producer cannot use the disabled-layer exception.
        data[28] = 1.
        self.assertFalse(Snapshot.decode(data).fresh(100.1))
        data[19] = 1.
        self.assertFalse(Snapshot.decode(data).fresh(100.1))

    def test_fixed_radius_has_exactly_one_resolution_margin(self):
        snapshot = Snapshot.decode(wire(value=1.))
        spheres = np.tile([.35, .35, .35, .2], (1, 48, 1))
        gap, _ = snapshot.query(spheres, 100.)
        self.assertAlmostEqual(gap[0], .7)  # 1 - physical radius .2 - resolution .1
        spheres[:, 47, 3] = .8
        gap, _ = snapshot.query(spheres, 100.)
        self.assertAlmostEqual(gap[0], .2)  # Arm radius receives no base margin.
        np.testing.assert_allclose(spheres[:, :47, 3], .2)  # No in-place inflation.

    def test_delivered_command_hold_detects_obstacle_after_first_step(self):
        class LaterObstacle:
            def query(self, spheres, times):
                self.times = np.asarray(times)
                gap = np.where(self.times <= 100.15+1e-9, 1., -.01)
                return gap, np.zeros_like(gap)
        snapshot = LaterObstacle()
        with self.assertRaisesRegex(ValueError, 'command_hold_or_braking'):
            checked_first_step(self.kernel, self.state.copy(), np.zeros(8), snapshot, 100.)
        self.assertGreaterEqual(snapshot.times.max(), 100.30-1e-9)

    def test_real_launch_nominal_pose_has_eight_dimensional_candidates(self):
        model, kernel, q, state = self.model, self.kernel, self.q, self.state.copy()
        self.assertEqual(len(model.fixed_spheres), 47)
        reference = np.zeros((2, 4))
        snapshot = Snapshot.decode(wire(shape=(20, 20, 20), resolution=.2,
                                        origin=(-2., -2., -.2), value=5.))
        result, info = kernel.plan(state, reference, q, snapshot, 100.)
        self.assertIsNotNone(result)
        controls, prediction = result
        self.assertEqual(controls.shape, (kernel.H, 8))
        self.assertEqual(prediction.shape, (kernel.H+1, 17))
        self.assertTrue(np.isfinite(prediction).all())
        self.assertGreater(info['feasible'], 0)
        self.assertGreaterEqual(info['min_gap'], 0.)
        self.assertEqual(info['sample_dimensions'], 8)
        self.assertTrue(np.all(np.asarray(info['noise_rms_per_coordinate']) > 0.))
        base_candidates = np.asarray(info['structured_first_base_commands'])
        self.assertTrue(np.any(base_candidates[:, 0] != 0.))
        self.assertTrue(np.any(base_candidates[:, 1] != 0.))

    def test_recorded_mesh_intersection_cannot_be_released_as_a_safe_command(self):
        report = json.loads((PACKAGE/'test/data/arm_pose_mesh_check_20261106.json').read_text())
        case = report['cases']['last_recorded_stationary']
        shoulder_wrist = next(p for p in case['mesh_pairs'] if p['links'] == ['link1', 'link5'])
        self.assertTrue(shoulder_wrist['surface_intersection'])
        q = np.asarray(case['q_radians'])
        self.assertIn('arm_self_collision', self.model.check(q)['violations'])
        state = np.r_[np.zeros(5), q, np.zeros(6)]
        clear_external_map = Snapshot.decode(wire(shape=(20, 20, 20), resolution=.2,
                                                  origin=(-2., -2., -.2), value=5.))
        result, _ = self.kernel.plan(state, np.zeros((2, 4)), q, clear_external_map, 100.)
        self.assertIsNone(result)
        with self.assertRaisesRegex(ValueError, 'command_hold_or_braking'):
            checked_first_step(self.kernel, state, np.zeros(8), clear_external_map, 100.)


class MoveJIntentTests(unittest.TestCase):
    """Real velocity rollout and delivery checks with one moving test sphere.

    The small geometry isolates interface/time coverage, not Piper calibration.
    """
    class Model:
        configuration_floor = -float('inf')

        def body_spheres_batch(self, q, base):
            spheres = np.zeros(q.shape[:-1]+(1, 4))
            spheres[..., 0, 0] = q[..., 0]
            spheres[..., 0, 1] = base[..., 0]
            spheres[..., 0, 3] = .0001
            return spheres

        def configuration_gaps_batch(self, q):
            return np.where(q[..., 0] < self.configuration_floor, -.01, 1.)

    class Scene:
        def __init__(self, obstacle=None):
            self.obstacle = obstacle

        def fresh(self, now):
            return True

        def query(self, spheres, times):
            self.spheres, self.times = spheres.copy(), np.asarray(times).copy()
            gap = np.ones(len(spheres))
            if self.obstacle is not None:
                x, crossing_time = self.obstacle
                # An obstacle crosses the sphere's line at the selected time.
                center = np.c_[np.full(len(times), x),
                               np.asarray(times)-crossing_time, np.zeros(len(times))]
                gap = np.linalg.norm(spheres[:, 0, :3]-center, axis=1)-spheres[:, 0, 3]-.0001
            return gap, np.zeros_like(gap)

    def setUp(self):
        self.model = self.Model()
        self.kernel = WholeBodyMPPI(self.model,
            dict(vmax=.05, wmax=.15, accel=.1, angular_accel=.3,
                 track=.37, wheel_v=.2296, wheel_a=.2975),
            ([-2.]*6, [2.]*6, [.15]*6, [.35]*6), [], horizon=8)
        self.state, self.control = np.zeros(17), np.zeros(8)

    def test_measured_velocity_spike_cannot_create_an_inertial_movej_target(self):
        self.state[11] = -.534  # Recorded finite-difference feedback peak.
        original = self.state.copy()
        for intent in (0., -.15, .15, -2., 2.):
            with self.subTest(intent=intent):
                self.control[2] = intent
                scene = self.Scene()
                end = checked_first_step(self.kernel, self.state, self.control, scene, 100.)
                self.assertAlmostEqual(end[5], np.clip(intent, -.15, .15)*.1)
                self.assertLess(scene.spheres[:, 0, 0].min(), -.03)
                # The actual dq remains in the checked inertia/braking path.
                np.testing.assert_array_equal(self.state, original)

    def test_dynamic_target_midpoint_and_early_arrival_are_checked(self):
        self.control[2] = .15
        for label, position, when in (('target_hold', .015, .55),
                                     ('midpoint', .0075, .15),
                                     ('early_target', .015, .05)):
            with self.subTest(case=label):
                with self.assertRaisesRegex(ValueError, 'command_hold_or_braking'):
                    checked_first_step(self.kernel, self.state, self.control,
                                       self.Scene((position, 100.+when)), 100.)

    def test_measured_velocity_collision_is_still_rejected(self):
        self.state[11] = -.534
        inertial = self.kernel.advance(self.state, self.control, .05)[5]
        with self.assertRaisesRegex(ValueError, 'command_hold_or_braking'):
            checked_first_step(self.kernel, self.state, self.control,
                               self.Scene((inertial, 100.05)), 100.)

    def test_measured_velocity_self_collision_and_limits_are_still_rejected(self):
        self.state[11] = -.534
        self.model.configuration_floor = -.02
        with self.assertRaisesRegex(ValueError, 'command_hold_or_braking'):
            checked_first_step(self.kernel, self.state, self.control, self.Scene(), 100.)
        self.model.configuration_floor = -float('inf')
        self.state[5] = -1.99
        with self.assertRaisesRegex(ValueError, 'command_hold_or_braking'):
            checked_first_step(self.kernel, self.state, self.control, self.Scene(), 100.)

    def test_lead_cap_and_joint_clipping_are_checked_before_returning(self):
        self.kernel.vmax[:] = 1.
        self.control[2] = 2.
        scene = self.Scene()
        end = checked_first_step(self.kernel, self.state, self.control, scene, 100.)
        self.assertAlmostEqual(end[5], .03)
        self.assertTrue(np.any(np.isclose(scene.spheres[:, 0, 0], .03)))
        # Clipping a target must not hide the original velocity path's violation.
        self.kernel.qmax[0] = .006
        with self.assertRaisesRegex(ValueError, 'command_hold_or_braking'):
            checked_first_step(self.kernel, self.state, self.control, scene, 100.)
        self.assertTrue(np.any(np.isclose(scene.spheres[:, 0, 0], .006)))

    def test_cycle_publishes_the_same_target_that_was_collision_checked(self):
        fixture = gates.ExecutionDeliveryTests(methodName='runTest')
        fixture.setUp()
        fixture.node.kernel = self.kernel
        scene = fixture.node.snapshot = self.Scene()
        fixture.node.joints[6] = -.534
        fixture.feed(gates.message(100.))
        with patch('whole_body_mppi.rospy.get_time', return_value=100.):
            fixture.node.cycle()  # Actual checked_first_step, fake publishers only.
        fixture.node.arm_pub.publish.assert_called_once()
        target = fixture.node.arm_pub.publish.call_args.args[0].position
        np.testing.assert_allclose(target, [.005, 0., 0., 0., 0., 0.])
        self.assertTrue(np.any(np.isclose(scene.spheres[:, 0, 0], target[0])))


class LocalRetryReferenceTests(unittest.TestCase):
    """Real cycle and reference adapter; isolated solver/publications only."""
    solve = gates.ExecutionDeliveryTests.solve
    feed = gates.ExecutionDeliveryTests.feed
    cycle = gates.ExecutionDeliveryTests.cycle

    def setUp(self):
        gates.ExecutionDeliveryTests.setUp(self)
        navigation = phases.Kernel()
        navigation.arm_position_servo, navigation._servo_native = False, None
        navigation.plan = Mock(side_effect=lambda *args, **kwargs: self.solve(*args))
        reference = NativeReferenceAdapter(phases.Kernel(), navigation, clock=lambda: self.now)
        self.assertEqual(reference.feed_goal(phases.goal(98.), 98.), 'accepted')
        self.assertEqual(reference.feed_trajectory(phases.trajectory(start=99.), 99.), 'replace')
        self.assertEqual(reference.feed_progress(phases.progress(100., start=99.), 100.), 'accepted')
        self.node.reference = reference
        self.node.sent_command = True
        self.node.sent_navigation_goal = reference.goal_stamp
        self.node.deadline = 1.
        self.feed(gates.message(100.))

    def test_expired_input_stops_then_fresh_input_resumes_the_same_native_reference(self):
        for stale in ('map', 'state'):
            with self.subTest(stale=stale):
                self.setUp()
                node, reference = self.node, self.node.reference
                native, revision = reference.native, reference.revision
                reference_hash = reference.status(100.)['reference_hash']
                if stale == 'map':
                    node.snapshot.fresh = lambda now: False
                else:
                    node.joint_stamp = 99.
                self.cycle()
                reference.navigation.plan.assert_not_called()
                node.arm_pub.publish.assert_not_called()
                node.fault_pub.publish.assert_not_called()
                for call in node.base_pub.publish.call_args_list:
                    self.assertEqual((call.args[0].linear.x, call.args[0].angular.z), (0., 0.))
                self.assertIs(reference.native, native)
                self.assertEqual(reference.revision, revision)
                self.assertEqual(reference.status(100.)['reference_hash'], reference_hash)

                # Only new healthy source data permit the next cycle. No goal,
                # trajectory replacement, fault reset, or stale command replay.
                node.snapshot.fresh = lambda now: True
                node.base_stamp = node.joint_stamp = 100.01
                self.feed(gates.message(100.01), now=100.01)
                self.cycle()
                reference.navigation.plan.assert_called_once()
                node.arm_pub.publish.assert_called_once()
                self.assertAlmostEqual(node.arm_pub.publish.call_args.args[0].header.stamp.to_sec(), 100.01)
                self.assertEqual(node.base_pub.publish.call_args.args[0].linear.x, .02)
                self.assertEqual(json.loads(node.info_pub.publish.call_args.args[0].data)['state'],
                                 'command_delivered')
                self.assertIs(reference.native, native)
                self.assertEqual(reference.revision, revision)
                self.assertFalse(node.fault_pub.publish.call_args.args[0].data)


if __name__ == '__main__':
    unittest.main()
