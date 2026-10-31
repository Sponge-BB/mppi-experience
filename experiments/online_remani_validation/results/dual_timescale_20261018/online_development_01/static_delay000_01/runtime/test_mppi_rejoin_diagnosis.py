#!/usr/bin/env python3
"""Protocol/geometry tests; no ROS, no vehicle commands or long kernel batch."""
import ast
import hashlib
from pathlib import Path
import unittest

import numpy as np

from run_mppi_rejoin_diagnosis import (
    HERE, CRITERIA, DEFAULT_CASES, PARAMETERS, Kernel, World, extract_plant,
    load_reference, project_continuous, project_geometric, sha,
)


class RejoinProtocolTests(unittest.TestCase):
    source = HERE/'results/resource_contention/validation_02/static_remani_01'

    def setUp(self):
        self.ref, self.times, self.arcs, self.initial, self.bundle = load_reference(self.source)

    def test_complete_native_transaction_including_reverse(self):
        self.assertEqual([m['id'] for m in self.bundle], [1, 2])
        self.assertEqual([m['singul'] for m in self.bundle], [-1, 1])
        self.assertGreater(self.times[-1], 20.)
        self.assertLess(np.min(self.ref[:, 3]), -.04)
        self.assertGreater(np.max(self.ref[:, 3]), .1)
        np.testing.assert_allclose(self.ref[-1, :2], [3., 0.], atol=1e-6)

    def test_projection_detects_lateral_offset(self):
        p = self.initial.copy()
        p[1] += .2
        projected = project_continuous(p, self.ref, self.times, self.arcs, 0.)
        self.assertTrue(projected['valid'])
        self.assertAlmostEqual(projected['error'], .2, delta=.01)
        self.assertAlmostEqual(abs(projected['lateral']), .2, delta=.01)

    def test_projection_does_not_jump_far_ahead(self):
        p = np.array([*self.ref[-1, :3], 0., 0.])
        projected = project_continuous(p, self.ref, self.times, self.arcs, 0.)
        self.assertLessEqual(projected['time'], 2.)

    def test_projection_does_not_skip_reverse_phase(self):
        p = self.initial.copy()
        p[1] -= .2
        projected = project_continuous(p, self.ref, self.times, self.arcs, 0.)
        phase_end = int(np.flatnonzero(self.ref[:, 3] > 0)[0])
        self.assertLessEqual(projected['time'], self.times[phase_end])

    def test_geometric_metric_can_recognize_safe_forward_rejoin(self):
        p = np.array([*self.ref[-10, :3], 0., 0.])
        geometric = project_geometric(p, self.ref, self.times, self.arcs)
        self.assertTrue(geometric['valid'])
        self.assertLess(geometric['error'], 1e-9)
        self.assertGreater(geometric['s'], .9*self.arcs[-1])

    def test_wrong_heading_projection_not_reported_as_success(self):
        p = self.initial.copy()
        p[2] = np.pi
        projected = project_continuous(p, self.ref, self.times, self.arcs, 0.)
        self.assertFalse(projected['valid'])

    def test_scene_appearance_and_ideal_observations(self):
        world = World('static_rejoin', self.ref, self.arcs)
        before, obs = world.inputs(3.99)
        after, obs2 = world.inputs(4.)
        self.assertEqual(obs.shape, (0, 6))
        self.assertEqual(obs2.shape, (0, 6))
        self.assertIs(before, world.wall_field)
        self.assertIs(after, world.static_field)
        self.assertEqual(len(world.actors(4.)), 1)
        crossing = World('dynamic_crossing', self.ref, self.arcs)
        _, tracks = crossing.inputs(9.)
        np.testing.assert_allclose(tracks[0, :2]+.04*tracks[0, 2:4], crossing.center)
        self.assertAlmostEqual(np.linalg.norm(tracks[0, 2:4]), .24)

    def test_exact_plant_ast_no_modification_or_ros(self):
        integrate, digest = extract_plant()
        tree = ast.parse((HERE/'simulator.py').read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Simulator')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'integrate')
        self.assertEqual(digest, hashlib.sha256(ast.get_source_segment((HERE/'simulator.py').read_text(), method).encode()).hexdigest())
        self.assertEqual(integrate.__name__, 'integrate')

    def test_prespecified_protocol(self):
        self.assertEqual(len(DEFAULT_CASES), 9)
        self.assertEqual(CRITERIA['rejoin_continuous_s'], 2.)
        self.assertEqual(CRITERIA['deadline_ms'], 50.)
        self.assertEqual(len(PARAMETERS), 16)

    def test_frozen_binary_identity(self):
        self.assertEqual(sha(HERE/'builds/failure_diagnosis_v12_04/build/libonline_mppi.so'),
                         sha(HERE/'builds/native_v12_07/build/libonline_mppi.so'))


if __name__ == '__main__':
    unittest.main()
