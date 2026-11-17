"""Regression against captured joints and independently transformed envelopes."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

RELEASE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RELEASE / 'core'))
sys.path.insert(0, str(RELEASE / 'runtime'))
from active_arm_model import PiperArmModel, ShoulderWristProxy
from active_arm_mppi import WholeBodyMPPI
from standby_whole_body_mppi import StandbyWholeBodyMPPI
from mppi_command_gateway import NativeLeaseValidator

Q_CAPTURED = np.array([-.23212879051524585, -.03330088212805181,
                      .006754424205218055, .02741912254883092,
                      .4934220328313169, -.020629791758572977])


class SyntheticClearSnapshot:
    """Explicit test geometry, never a statement about unseen real space."""
    def fresh(self, wall):
        return True

    def query(self, spheres, times):
        return np.ones(len(spheres)), np.zeros(len(spheres))


class ShoulderWristMeshTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = PiperArmModel.from_parameters(RELEASE/'model/parameters.yaml',
                                                  RELEASE/'model/mm.urdf')
        previous = RELEASE.parent/'standby_20261106_alignment_05'
        spec = importlib.util.spec_from_file_location('coarse05', previous/'core/active_arm_model.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        cls.coarse = module.PiperArmModel.from_parameters(RELEASE/'model/parameters.yaml',
                                                         RELEASE/'model/mm.urdf')
        cls.asset = json.loads((RELEASE/'model/shoulder_wrist_proxy.json').read_text())
        cls.config = json.loads((RELEASE/'config/hardware_shadow.json').read_text())
        cls.qs = np.vstack([Q_CAPTURED, Q_CAPTURED+[0, 0, 0, 0, .03, 0],
                           np.zeros(6), np.random.RandomState(607).uniform(
                               cls.model.lower, cls.model.upper, (12, 6))])

    def kernel(self, cls=StandbyWholeBodyMPPI):
        a = self.config['arm_limits']
        return cls(self.model, self.config['limits'],
                   [a[k] for k in ('lower', 'upper', 'velocity', 'acceleration')],
                   self.config['postures'], horizon=8, step=.1, random_samples=4)

    def test_captured_pose_resolves_only_the_two_supported_coarse_pairs(self):
        old, new = self.coarse.check(Q_CAPTURED), self.model.check(Q_CAPTURED)
        self.assertFalse(old['valid'])
        self.assertTrue(new['valid'], new)
        self.assertEqual(old['self_pair'], [0, 18])
        self.assertEqual(new['self_pair'], [0, 18])
        self.assertAlmostEqual(new['min_self_gap'], .000622771232690722-.0001, places=11)
        old_gaps = self.coarse.arm_spheres(Q_CAPTURED)
        i, j = self.coarse._self_pairs.T
        expected = np.linalg.norm(old_gaps[i, :3]-old_gaps[j, :3], axis=1)-.068
        actual = np.asarray(self.model.self_gaps(Q_CAPTURED)).reshape(-1)
        changed = np.flatnonzero(np.abs(actual-expected) > 1e-12)
        self.assertEqual(set(map(tuple, self.model._self_pairs[changed])), {(0, 18), (7, 18)})

    def test_original_fcl_intersection_is_still_rejected(self):
        # Recorded original triangle-mesh query: q5 +0.03 rad intersects.
        # See ../../../mesh_audit_20261106/nearby_mesh_cases.json.
        result = self.model.check(self.qs[1])
        self.assertFalse(result['valid'])
        self.assertIn('arm_self_collision', result['violations'])

    def test_external_spheres_and_all_other_checks_unchanged(self):
        poses = np.tile([.2, -.1, .7], (len(self.qs), 1))
        np.testing.assert_array_equal(self.model.body_spheres(self.qs, poses),
                                      self.coarse.body_spheres(self.qs, poses))
        np.testing.assert_array_equal(self.model._self_pairs, self.coarse._self_pairs)
        self.assertEqual(len(self.model._self_pairs), 89)
        for q in self.qs:
            fine, coarse = self.model.check(q), self.coarse.check(q)
            for key in ('min_base_arm_gap', 'min_ground_gap', 'joint_gaps'):
                self.assertEqual(fine[key], coarse[key])

    def test_scalar_batch_and_both_kernels_agree_in_transformed_base_frame(self):
        expected = np.array([min(v['min_self_gap'], v['min_base_arm_gap'],
                                 v['min_ground_gap']) for v in self.model.check_batch(self.qs)])
        np.testing.assert_allclose(self.model.configuration_gaps_batch(self.qs), expected, atol=1e-14)
        states = np.zeros((len(self.qs), 17)); states[:, 5:11] = self.qs
        states[:, :3] = [.4, -.3, .8]
        for cls in (WholeBodyMPPI, StandbyWholeBodyMPPI):
            kernel = self.kernel(cls)
            np.testing.assert_allclose(kernel.configuration_gap(kernel.spheres(states), self.qs),
                                       expected, atol=1e-14)

    def test_rollout_brake_and_final_validator_accept_and_reject_consistently(self):
        snapshot = SyntheticClearSnapshot()
        for cls in (WholeBodyMPPI, StandbyWholeBodyMPPI):
            kernel = self.kernel(cls)
            reference = np.zeros((1, 4))
            for q, expected in [(Q_CAPTURED, True), (self.qs[1], False)]:
                state = np.r_[np.zeros(5), q, np.zeros(6)]
                _, valid, _, _ = kernel.evaluate(state, np.zeros((1, 8, 8)),
                                                 reference, q, snapshot, 1000.)
                self.assertEqual(bool(valid[0]), expected)
                self.assertEqual(kernel.lease_brake(state, None, snapshot, 1000.)[0], expected)
                validator = NativeLeaseValidator(kernel)
                self.assertEqual(validator(state, np.zeros(8), snapshot, 1000., .1), expected)

    def test_repeated_joint_states_with_different_base_poses_keep_their_verdicts(self):
        qs = self.qs[[0, 1, 0, 2, 1, 0]]
        states = np.zeros((len(qs), 17)); states[:, 5:11] = qs
        states[:, :3] = np.random.RandomState(66).uniform(-1., 1., (len(qs), 3))
        kernel = self.kernel()
        np.testing.assert_allclose(kernel.configuration_gap(kernel.spheres(states), qs),
                                   self.model.configuration_gaps_batch(qs), atol=1e-14)

    def test_direct_constructor_retains_coarse_rejection(self):
        coarse = PiperArmModel(self.model.fixed_spheres, urdf_path=RELEASE/'model/mm.urdf')
        self.assertFalse(coarse.check(Q_CAPTURED)['valid'])

    def test_support_projection_matches_independent_original_urdf_chain(self):
        proxy = self.model.shoulder_wrist_proxy
        expected = []
        for q in self.qs:
            frames = self.model.urdf_link_transforms(q)
            axis = frames['link5'][:3, :3] @ proxy.axis
            projections = {}
            for name, vertices in proxy.vertices.items():
                frame = frames[name]
                projections[name] = (vertices @ frame[:3, :3].T+frame[:3, 3]) @ axis
            a = np.concatenate([v for k, v in projections.items() if k not in ('link5', 'link6')])
            b = np.concatenate([projections['link5'], projections['link6']])
            expected.append(max(a.min()-b.max(), b.min()-a.max())-proxy.numerical_tolerance)
        np.testing.assert_allclose(proxy.gaps(self.qs), expected, rtol=0., atol=1e-14)

    def test_both_upper_arm_parts_match_independent_full_vertex_projection(self):
        proxy = self.model.shoulder_wrist_proxy
        expected = []
        parts = self.asset['upper_arm_certificate']['parts']
        for q in self.qs:
            frames = self.model.urdf_link_transforms(q)
            wrist = np.concatenate([proxy.vertices[name] @ frames[name][:3, :3].T+frames[name][:3, 3]
                                    for name in ('link5', 'link6')])
            gaps = []
            for part in parts.values():
                axis = frames['link5'][:3, :3] @ np.asarray(part['axis_local_link5'])
                shoulder = np.asarray(part['vertices']) @ frames['link2'][:3, :3].T+frames['link2'][:3, 3]
                a, b = shoulder @ axis, wrist @ axis
                gaps.append(max(a.min()-b.max(), b.min()-a.max())-proxy.numerical_tolerance)
            expected.append(min(gaps))
        np.testing.assert_allclose(proxy.pair_gaps(self.qs)[:, 1], expected, rtol=0., atol=1e-14)

    def test_invalid_asset_cannot_grant_clearance(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'proxy.json'
            with self.assertRaises(FileNotFoundError):
                ShoulderWristProxy(path, self.model._urdf_root)
            bad = json.loads(json.dumps(self.asset))
            bad['proxies']['link5']['vertices'][0][0] = float('nan')
            path.write_text(json.dumps(bad))
            with self.assertRaises(ValueError):
                ShoulderWristProxy(path, self.model._urdf_root)
            bad = json.loads(json.dumps(self.asset))
            bad['joint_chain_arm_base_to_link6'][4]['origin']['xyz'] = '0 0 .01'
            path.write_text(json.dumps(bad))
            with self.assertRaises(ValueError):
                ShoulderWristProxy(path, self.model._urdf_root)


if __name__ == '__main__':
    unittest.main()
