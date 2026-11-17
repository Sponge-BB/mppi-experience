"""Original physical adjacency without removing non-adjacent sphere checks."""

import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np


RELEASE = Path(__file__).resolve().parents[1]
PREVIOUS = RELEASE.parent / 'standby_20261105_integration_04'
MEASURED_Q = np.deg2rad([-13.299, -1.907, .388, 1.569, 28.269, -1.179])


def load_model(release):
    spec = importlib.util.spec_from_file_location(
        'adjacency_' + release.name, release / 'core/active_arm_model.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # Identical inputs isolate the policy change from any deployment settings.
    return module.PiperArmModel.from_parameters(
        RELEASE / 'model/parameters.yaml', RELEASE / 'model/mm.urdf')


class OriginalArmAdjacencyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = load_model(RELEASE)
        cls.previous = load_model(PREVIOUS)
        rng = np.random.RandomState(613)
        cls.qs = np.vstack([MEASURED_Q, np.zeros(6), cls.model.q_nominal,
                            rng.uniform(cls.model.lower, cls.model.upper, (16, 6))])

    def test_original_adjacency_and_physical_link_ownership(self):
        srdf = ET.parse(RELEASE / 'model/piper_camera.srdf').getroot()
        rules = {(r.get('link1'), r.get('link2')): r.get('reason')
                 for r in srdf.findall('disable_collisions')}
        self.assertEqual(rules[('link2', 'link3')], 'Adjacent')
        self.assertNotIn(('link1', 'link5'), rules)
        # The native elbow connector and forearm samples share URDF link3.
        for ids, link in [(np.arange(3, 8), 'link2'), (np.arange(8, 15), 'link3')]:
            local_points = []
            for q in self.qs:
                frames = self.model.urdf_link_transforms(q)
                frame = np.linalg.inv(frames['arm_base']) @ frames[link]
                points = self.model.arm_spheres(q)[ids, :3]
                points = (points - self.model.mount[:3, 3]) @ self.model.mount[:3, :3]
                local_points.append((points - frame[:3, 3]) @ frame[:3, :3])
            # Original URDF rotations/translations have rounded constants.
            scatter = np.linalg.norm(np.asarray(local_points) - local_points[0], axis=2)
            self.assertLess(float(scatter.max()), 1e-4)

    def test_only_supported_self_pairs_change_and_all_67_spheres_stay(self):
        poses = np.tile([.2, -.1, .7], (len(self.qs), 1))
        body = self.model.body_spheres_batch(self.qs, poses)
        self.assertEqual(body.shape, (len(self.qs), 67, 4))
        np.testing.assert_array_equal(body, self.previous.body_spheres_batch(self.qs, poses))
        np.testing.assert_array_equal(self.model.base_arm_checked_indices,
                                      self.previous.base_arm_checked_indices)
        old = set(map(tuple, self.previous.collision_policy()['checked_self_pairs']))
        policy = self.model.collision_policy()
        new = set(map(tuple, policy['checked_self_pairs']))
        self.assertEqual(new - old, set())
        self.assertEqual(old - new, {(i, j) for i in range(3, 8)
                                    for j in range(10, 15)} - {(3, 14)})
        reasons = {(r['sphere_i'], r['sphere_j']): r['reason']
                   for r in policy['excluded_self_pairs']}
        for pair in old - new:
            self.assertEqual(reasons[pair], 'physical_adjacent_urdf_link2_link3')
        for q in self.qs:
            current, previous = self.model.check(q), self.previous.check(q)
            for key in ['min_base_arm_gap', 'min_ground_gap', 'joint_gaps']:
                self.assertEqual(current[key], previous[key])

    def test_zero_pose_passes_but_measured_nonadjacent_overlap_still_rejects(self):
        self.assertFalse(self.previous.check(np.zeros(6))['valid'])
        zero = self.model.check(np.zeros(6))
        self.assertTrue(zero['valid'], zero)
        self.assertAlmostEqual(zero['min_self_gap'], .003, places=10)
        measured = self.model.check(MEASURED_Q)
        self.assertFalse(measured['valid'])
        self.assertEqual(measured['violations'], ['arm_self_collision'])
        self.assertEqual(measured['self_pair'], [0, 18])
        self.assertAlmostEqual(measured['min_self_gap'], -.0074005751875568584, places=10)

    def test_nonadjacent_collision_is_not_hidden_by_base_rejection(self):
        # An in-bounds sphere-model counterexample, not mesh certification.
        q = np.deg2rad([100.59939439175184, 136.75775756919737,
                       -12.492276236379057, 2.8589187401900937,
                       55.52823537956905, -15.24086704244293])
        result = self.model.check(q)
        self.assertGreater(result['joint_gaps']['position'], 0.)
        self.assertIn('arm_self_collision', result['violations'])
        self.assertEqual(result['self_pair'], [1, 17])
        self.assertLess(result['min_self_gap'], -.04)

    def test_scalar_and_rollout_collision_gaps_agree(self):
        scalar = [min(r['min_self_gap'], r['min_base_arm_gap'], r['min_ground_gap'])
                  for r in self.model.check_batch(self.qs)]
        np.testing.assert_allclose(self.model.configuration_gaps_batch(self.qs),
                                   scalar, rtol=0., atol=1e-14)


if __name__ == '__main__':
    unittest.main()
