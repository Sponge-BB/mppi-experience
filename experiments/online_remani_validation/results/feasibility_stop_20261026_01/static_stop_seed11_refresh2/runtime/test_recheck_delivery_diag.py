#!/usr/bin/env python3
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

import recheck_delivery_diag as d


def frame():
    # Analytic plane distance d(x,y)=x, with origin x=0.
    return dict(nx=5, ny=5, gx=0., gy=-2., res=1., margin=.1,
                comfort=.12, weight=5e4, max_age=.35, max_prediction=3.,
                healthy=True, sequence=4, stamp=1000., body=[[0., 0., .2]],
                obstacles=[], field=np.tile(np.arange(5.), (5, 1)).ravel().tolist())


def piece(T, x, y=0., vx=0., vy=0.):
    c = np.zeros((8, 8))
    c[0, -1], c[1, -1] = x, y
    c[0, -2], c[1, -2] = vx, vy
    return dict(T=T, coeffs=c.tolist(), basis='descending')


class RecheckTests(unittest.TestCase):
    def test_static_categories_and_bilinear(self):
        m = d.snapshot_arrays(frame())
        for x, category, physical in ((.5, 'safe', .3), (.25, 'margin_only', .05), (.1, 'collision', -.1)):
            e = d.evaluate_pose(m, x, 0., 0., 0.)
            self.assertEqual(e['category'], category)
            self.assertAlmostEqual(e['physical'], physical)
        self.assertEqual(d.evaluate_pose(m, -.1, 0., 0., 0.)['category'], 'invalid')

    def test_body_rotation_not_point(self):
        raw = frame()
        raw['body'] = [[.3, 0., .2]]
        m = d.snapshot_arrays(raw)
        self.assertAlmostEqual(d.evaluate_pose(m, .5, 0., 0., 0.)['physical'], .6)
        self.assertAlmostEqual(d.evaluate_pose(m, .5, 0., np.pi, 0.)['physical'], 0.)

    def test_dynamic_cv_and_unknown(self):
        raw = frame()
        raw['obstacles'] = [[2., 0., -1., 0., .3, 1.]]
        m = d.snapshot_arrays(raw)
        self.assertAlmostEqual(d.evaluate_pose(m, 1., 0., 0., 0., True)['physical'], .5)
        self.assertEqual(d.evaluate_pose(m, 1., 0., 0., 1., True)['category'], 'collision')
        self.assertEqual(d.evaluate_pose(m, 1., 0., 0., 3.1, True)['category'], 'unknown')

    def test_polynomial_and_boundary(self):
        c = dict(start_time=1001., parts=[dict(duration=2., gear=-1, pieces=[piece(1., .5, vx=.2), piece(1., .7, vx=.2)])])
        samples = list(d.candidate_samples(c, 'dense', .5))
        self.assertEqual(samples[2]['piece'], 0)
        self.assertAlmostEqual(samples[2]['x'], .7)
        self.assertAlmostEqual(samples[-1]['query_time'], 1003.)
        self.assertAlmostEqual(abs(samples[-1]['yaw']), np.pi)
        self.assertEqual(len(list(d.candidate_samples(c, 'K8'))), 18)

    def test_sampling_difference_is_detected(self):
        # Narrow quadratic dip between K8 samples .0 and .125.
        p = piece(1., .19+40*.0625**2, vx=-80*.0625)
        p['coeffs'][0][-3] = 40.
        c = dict(start_time=0., parts=[dict(duration=1., gear=1, pieces=[p])])
        dense = min(s['x'] for s in d.candidate_samples(c, 'dense'))
        coarse = min(s['x'] for s in d.candidate_samples(c, 'K8'))
        self.assertLess(dense, .2)  # collision with .2 m body radius
        self.assertGreater(coarse, .3)  # coarse grid even clears .1 m margin

    def test_no_fabricated_dynamic_when_past(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root/'snapshots').mkdir()
            (root/'snapshots/a.json').write_text(json.dumps(frame()))
            c = dict(schema='delivery_candidate_v1', start_time=998.,
                     solve_frame=dict(snapshot='snapshots/a.json', now=1000.1),
                     accept_frame=dict(snapshot='snapshots/a.json', now=1000.2),
                     parts=[dict(duration=1., gear=1, pieces=[piece(1., .8, vx=.1)])], failure=None)
            path = root/'candidate.json'
            path.write_text(json.dumps(c))
            audit, rows = d.audit_candidate(root, path)
            self.assertTrue(audit['replay_gate_status_agrees'])
            self.assertTrue(all(r['dynamic_category']=='past' for r in rows))

    def test_stale_distinct_from_geometric_failure(self):
        self.assertTrue(d.fresh(d.snapshot_arrays(frame()), 1000.1))
        self.assertFalse(d.fresh(d.snapshot_arrays(frame()), 1000.4))

    def test_no_path_traversal(self):
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaises(ValueError):
                d.resolve_snapshot(Path(temp), dict(snapshot='../escape.json'))

    def test_nested_actual_frame_schema(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root/'snapshots').mkdir()
            (root/'snapshots/a.json').write_text(json.dumps(frame()))
            reference = dict(reference=dict(snapshot='snapshots/a.json', snapshot_payload_hash='hash'), now=1000.1)
            self.assertEqual(d.resolve_snapshot(root, reference)['sequence'], 4)
            self.assertIsNone(d.resolve_snapshot(root, dict(reference=None, now=0.)))

    def test_geometry_error_independent_reconstruction(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root/'snapshots').mkdir()
            (root/'snapshots/a.json').write_text(json.dumps(frame()))
            c = np.zeros((8, 8))
            c[0, 0], c[1, 0] = -.5, 0.
            e = dict(schema='delivery_geometry_error_v1', frame=dict(now=1000.1, reference=dict(snapshot='snapshots/a.json')),
                     p=[-.5, 0.], v=[0., 0.], yaw=0., forecast=.1, s=0., block=0, piece=0, sample=0,
                     blocks=[dict(basis='ascending_rows_per_8', coeffs=c.tolist(), times=[1.], gear=1)],
                     leaf=dict(reason='static_field_out_of_bounds'))
            path = root/'error.json'
            path.write_text(json.dumps(e))
            result = d.audit_geometry_error(root, path)
            self.assertEqual(result['independent_cause'], 'body_outside_grid')
            self.assertEqual(result['reconstructed_parameter_deltas'], dict(p_max_abs=0., v_max_abs=0.))

    def test_duration_mismatch_rejected(self):
        with self.assertRaises(ValueError):
            list(d.candidate_samples(dict(start_time=1., parts=[dict(gear=1, duration=2., pieces=[piece(1., 1.)])]), 'dense'))

    def test_invalid_field_does_not_become_collision(self):
        raw = frame()
        raw['field'][2*5+1] = float('nan')
        m = d.snapshot_arrays(raw)
        self.assertEqual(d.evaluate_pose(m, 1.5, 0., 0., 0.)['category'], 'invalid')


if __name__ == '__main__':
    unittest.main()
