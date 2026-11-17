"""Complete-hull support certificates only; no ROS, devices or mesh runtime."""
import copy
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

import numpy as np
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'scripts/mppi'))
from active_arm_model import PiperArmModel

# Recorded /mm/joint_states, hardware_startup_check_20261107/runtime_feedback.json.
# This is an offline regression pose, never an assertion about current hardware.
RECORDED_Q = np.array([-.10072295113259275, -.03183480555637657,
                      .01345648853287628, .10487683475233926,
                      .46677085515336353, -.10257300013970674])
PRIOR_SEPARATED_Q = np.array([-.10108947027551157, -.03330088212805181,
                            .010088003076527225, .0976337183565628,
                            .48288024414927117, -.11770500475449759])
# Actual disabled feedback from the 2026-11-08 hardware startup recording.
# Original triangle surfaces are separate; the whole-link convex hulls overlap.
FOLDED_Q = np.deg2rad([-2.447, -1.909, .943, 5.992, 26.168, -5.879])
# 2026-11-09 post-enable feedback: original link1/link5 triangle surfaces
# intersect in the URDF (triangles 8604/15784). This is model evidence only.
POST_ENABLE_Q = np.array([-.04089306437422714, -.03420845333908886,
                         .021921335405048776, .10419615634406147,
                         .4659680036974461, -.10234610733694748])


def original_vertex_support(proxy, q, axis, wrist_vertices=None, direction_frame='link5'):
    """Independent scalar world-vertex projection, not the batched runtime path."""
    transform = np.eye(4)
    frames = {'arm_base': transform.copy()}
    for i, origin in enumerate(proxy.origins):
        c, s = np.cos(q[i]), np.sin(q[i])
        turn = np.eye(4)
        turn[:2, :2] = [[c, -s], [s, c]]
        transform = transform @ origin @ turn
        frames['link'+str(i+1)] = transform.copy()
    direction = frames[direction_frame][:3, :3] @ axis
    vertices = {}
    for name, points in proxy.vertices.items():
        if name == 'link5' and wrist_vertices is not None:
            points = wrist_vertices
        frame = frames[name]
        vertices[name] = points @ frame[:3, :3].T + frame[:3, 3]
    shoulder = np.concatenate([vertices['arm_base'], vertices['link1']]) @ direction
    wrist = np.concatenate([vertices['link5'], vertices['link6']]) @ direction
    return max(shoulder.min()-wrist.max(), wrist.min()-shoulder.max()) - proxy.numerical_tolerance


class ShoulderSupportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        parameters = {}
        config = PACKAGE.parent/'mm_remani_planner/plan_manage/config'
        for name in ('piper_param.yaml', 'remani_planner_param.yaml', 'fixed_collision_spheres_b.yaml'):
            for group, values in yaml.safe_load((config/name).read_text()).items():
                if isinstance(values, dict):
                    parameters.setdefault(group, {}).update(values)
                else:
                    parameters[group] = values
        cls.model = PiperArmModel.from_parameters(parameters)
        cls.original = copy.copy(cls.model)
        cls.original.shoulder_wrist_proxy = copy.copy(cls.model.shoulder_wrist_proxy)
        cls.original.shoulder_wrist_proxy.shoulder_axes = ()
        cls.original.shoulder_wrist_proxy.wrist_parts = ()

    def test_recorded_pose_has_complete_hull_positive_certificate(self):
        old = self.original.check(RECORDED_Q)
        new = self.model.check(RECORDED_Q)
        self.assertFalse(old['valid'])
        self.assertEqual(old['self_pair'], [0, 18])
        self.assertTrue(new['valid'])
        self.assertAlmostEqual(new['min_self_gap'], .0001350678761031677, places=12)
        self.assertEqual(old['min_base_arm_gap'], new['min_base_arm_gap'])
        self.assertEqual(old['min_ground_gap'], new['min_ground_gap'])
        proxy = self.model.shoulder_wrist_proxy
        # The historical second axis alone still does NOT prove this pose safe.
        self.assertLess(original_vertex_support(proxy, RECORDED_Q, proxy.shoulder_axes[0][0]), 0.)
        independent = original_vertex_support(proxy, RECORDED_Q, proxy.shoulder_axes[1][0])
        self.assertAlmostEqual(new['min_self_gap'], independent, places=12)

    def test_post_enable_pose_with_original_mesh_intersection_remains_rejected(self):
        report = self.model.check(POST_ENABLE_Q)
        self.assertFalse(report['valid'])
        self.assertIn('arm_self_collision', report['violations'])
        self.assertEqual(report['self_pair'], [0, 18])
        self.assertLess(self.model.shoulder_wrist_proxy.gaps(POST_ENABLE_Q[None])[0], 0.)
        self.assertLess(report['min_self_gap'], 0.)

    def test_prior_safe_pose_nominal_and_other_pairs_preserved(self):
        self.assertTrue(self.model.check(PRIOR_SEPARATED_Q)['valid'])
        self.assertEqual(self.model.check(self.model.q_nominal), self.original.check(self.model.q_nominal))
        rng = np.random.default_rng(11)
        q = np.vstack([RECORDED_Q, PRIOR_SEPARATED_Q, self.model.q_nominal,
                       RECORDED_Q+rng.uniform(-.08, .08, (24, 6))])
        old = self.original.self_gaps(q)
        new = self.model.self_gaps(q)
        shoulder = self.model._shoulder_wrist_pairs[0]
        other = np.arange(old.shape[1]) != shoulder
        np.testing.assert_array_equal(old[:, other], new[:, other])
        self.assertTrue(np.all(new[:, shoulder] >= old[:, shoulder]))
        np.testing.assert_array_equal(self.original.shoulder_wrist_proxy.pair_gaps(q)[:, 1],
                                      self.model.shoulder_wrist_proxy.pair_gaps(q)[:, 1])

    def test_every_released_pose_has_independent_whole_hull_separation(self):
        rng = np.random.default_rng(23)
        q = np.vstack([RECORDED_Q, PRIOR_SEPARATED_Q, FOLDED_Q,
                       RECORDED_Q+rng.uniform(-.04, .04, (32, 6)),
                       FOLDED_Q+rng.uniform(-.005, .005, (32, 6))])
        proxy = self.model.shoulder_wrist_proxy
        old = self.original.shoulder_wrist_proxy.pair_gaps(q)[:, 0]
        new = proxy.pair_gaps(q)[:, 0]
        released = np.flatnonzero((old <= 0.) & (new > 0.))
        self.assertGreater(len(released), 0)
        for row in released:
            independent = [original_vertex_support(proxy, q[row], axis)
                           for axis, _ in proxy.shoulder_axes]
            independent.append(min(original_vertex_support(proxy, q[row], axis,
                wrist_vertices=vertices, direction_frame='link'+str(frame))
                for vertices, axis, frame in proxy.wrist_parts))
            self.assertGreater(max(independent), 0.)
            self.assertTrue(any(abs(gap-new[row]) < 1e-12 for gap in independent))

    def test_known_original_mesh_intersection_remains_rejected(self):
        report = json.loads((PACKAGE/'test/data/arm_pose_mesh_check_20261106.json').read_text())
        case = report['cases']['last_recorded_stationary']
        pair = next(p for p in case['mesh_pairs'] if p['links'] == ['link1', 'link5'])
        self.assertTrue(pair['surface_intersection'])
        q = np.asarray(case['q_radians'])
        self.assertLess(self.model.shoulder_wrist_proxy.gaps(q[None])[0], 0.)
        self.assertIn('arm_self_collision', self.model.check(q)['violations'])

    def test_folded_feedback_and_standby_path_pass_without_moving_spheres(self):
        legacy = copy.copy(self.model)
        legacy.shoulder_wrist_proxy = copy.copy(self.model.shoulder_wrist_proxy)
        legacy.shoulder_wrist_proxy.wrist_parts = ()
        self.assertFalse(legacy.check(FOLDED_Q)['valid'])
        self.assertEqual(legacy.check(FOLDED_Q)['self_pair'], [0, 18])
        self.assertTrue(self.model.check(FOLDED_Q)['valid'])
        count = int(np.ceil(np.max(np.abs(self.model.q_nominal-FOLDED_Q))/.02))+1
        path = np.linspace(FOLDED_Q, self.model.q_nominal, count)
        self.assertTrue(np.all(self.model.configuration_gaps_batch(path) > 0.))
        np.testing.assert_array_equal(self.model.arm_spheres(path), legacy.arm_spheres(path))
        np.testing.assert_array_equal(self.model.fixed_spheres, legacy.fixed_spheres)

    def test_mask_scalar_batch_and_exact_q_deduplication(self):
        proxy = self.model.shoulder_wrist_proxy
        q = np.vstack([RECORDED_Q, PRIOR_SEPARATED_Q, RECORDED_Q])
        np.testing.assert_array_equal(proxy.pair_gaps(q, np.zeros(3, bool)),
                                      self.original.shoulder_wrist_proxy.pair_gaps(q))
        np.testing.assert_allclose(self.model.self_gaps(q),
                                   np.vstack([self.model.self_gaps(row) for row in q]),
                                   rtol=0., atol=1e-12)
        model = copy.copy(self.model)
        model.shoulder_wrist_proxy = copy.copy(proxy)
        model.shoulder_wrist_proxy.pair_gaps = Mock(wraps=proxy.pair_gaps)
        model.self_gaps(q)
        unique, mask = model.shoulder_wrist_proxy.pair_gaps.call_args[0]
        self.assertEqual(unique.shape, (2, 6))
        self.assertTrue(mask.all())


if __name__ == '__main__':
    unittest.main()
