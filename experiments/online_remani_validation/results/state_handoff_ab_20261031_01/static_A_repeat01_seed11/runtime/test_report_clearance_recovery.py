#!/usr/bin/env python3
"""Small synthetic fixtures only; no ROS, compilation, models or plotting."""
import copy
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

import report_clearance_recovery as report


def snapshot():
    return dict(schema='delivery_snapshot_v1', nx=61, ny=7, gx=0., gy=-3., res=1., margin=.1,
                comfort=.12, weight=5e4, max_age=.35, max_prediction=3., healthy=True,
                sequence=4, stamp=1000., body=[[0., 0., .2]], obstacles=[],
                field=np.tile(np.arange(61.), (7, 1)).ravel().tolist())


def phase(name='nominal', x=.25, times=(.1,), gear=1):
    c = np.zeros((8 * len(times), 2))
    for piece in range(len(times)):
        c[8 * piece, 0] = x
    return dict(phase=name, blocks=[dict(gear=gear, times=list(times), basis='ascending_rows_per_8', coeffs=c.tolist())])


def write_json(path, value):
    path.write_text(json.dumps(value), encoding='utf-8')


def events(enabled=True, force=0, code=1):
    common = dict(pid=100, request_id=1)
    return [dict(common, event='request_begin', mono=1.),
            dict(common, event='coupling_scope', mono=1.1, scope_id=3),
            dict(common, event='domain_solver_status', mono=1.2, lbfgs_result=1),
            dict(common, event='clearance_nominal', mono=1.5, enabled=int(enabled), valid=True,
                 min_gap=-.05, min_physical=.05, samples=6, elapsed_s=.1),
            dict(common, event='clearance_restart_begin', mono=2., remaining_s=.1),
            dict(common, event='clearance_restart_end', mono=2.1, lbfgs_result=code, force_stop=force),
            dict(common, event='clearance_restored', mono=2.2, enabled=int(enabled), valid=True,
                 min_gap=.1, min_physical=.2, samples=6, elapsed_s=.2),
            dict(common, event='request_end', mono=3.)]


def fixture(root, name='on', enabled=True, phases=None, rows=None, outcome='native_qualified_pending_latest_gate'):
    trial = root / name
    trial.mkdir()
    write_json(trial / 'manifest.json', dict(coupling_experiment=dict(clearance_recovery=enabled, seed=11)))
    write_json(trial / 'run_status.json', dict(returncode=0, result_written=True))
    write_json(trial / 'trial_audit.json', dict(valid_experiment=True, errors=[]))
    write_json(trial / 'result.json', dict(reason='goal', collision_conservative_projection=False))
    (trial / 'dual_timeline.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in (rows if rows is not None else events(enabled))), encoding='utf-8')
    diagnostic = trial / 'delivery_diagnostics'
    (diagnostic / 'snapshots').mkdir(parents=True)
    (diagnostic / 'candidates').mkdir()
    (diagnostic / 'clearance_recovery').mkdir()
    write_json(diagnostic / 'snapshots/a.json', snapshot())
    payload = dict(schema='clearance_recovery_v1', request_id=1, scope_id=3, enabled=enabled, rho=1e5,
                   outcome=outcome, frame=dict(now=1000.1, reference=dict(snapshot='snapshots/a.json')),
                   phases=phases if phases is not None else [phase(), phase('restored_canonical_x', .4)])
    write_json(diagnostic / 'clearance_recovery/100_1.json', payload)
    return trial


class ClearanceReportTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def state(self, trial):
        return report.summarize_trial(trial)['state_records'][0]

    def test_asymmetric_coefficients_transpose_reverse_piece_order_and_gears(self):
        blocks = []
        for gear, times in ((1, [.04, .07]), (-1, [.06])):
            c = np.arange(8 * len(times) * 3, dtype=float).reshape(8 * len(times), 3) / 100.
            blocks.append(dict(gear=gear, times=times, basis='ascending_rows_per_8', coeffs=c.tolist()))
        view = report.trajectory_view(dict(blocks=blocks))
        self.assertNotIn('accepted', view)
        self.assertNotIn('schema', view)
        for block, part in zip(blocks, view['parts']):
            c = np.asarray(block['coeffs'])
            for i, piece in enumerate(part['pieces']):
                for t in (0., piece['T'] * .37, piece['T']):
                    expected_p = sum(c[8*i+k] * t**k for k in range(8))
                    expected_v = sum(k*c[8*i+k] * t**(k-1) for k in range(1, 8))
                    p, v = report.replay.polynomial(piece, t)
                    np.testing.assert_allclose(p, expected_p, rtol=1e-14)
                    np.testing.assert_allclose(v, expected_v, rtol=1e-14)
        dense = list(report.replay.candidate_samples(view, 'dense', .02))
        self.assertEqual(len(dense), sum(math.ceil(sum(b['times']) / .02) + 1 for b in blocks))
        self.assertEqual(len(list(report.replay.candidate_samples(view, 'K8'))), 27)
        reverse = next(s for s in dense if s['part'] == 1)
        self.assertAlmostEqual(reverse['yaw'], math.atan2(-reverse['vy'], -reverse['vx']))

    def test_full_body_and_reverse_gear_not_center_only(self):
        raw = snapshot(); raw['body'] = [[.2, 0., .2]]
        forward = phase(x=.4)
        forward['blocks'][0]['coeffs'][1][0] = .1
        backward = copy.deepcopy(forward); backward['blocks'][0]['gear'] = -1
        a = report.replay_phase(forward, raw, 1000.1)
        b = report.replay_phase(backward, raw, 1000.1)
        self.assertGreater(a['sampling']['dense']['min_gap'], 0.)
        self.assertLess(b['sampling']['dense']['min_gap'], 0.)
        self.assertEqual(b['sampling']['dense']['worst']['body_index'], 0)

    def test_K8_and_dense_are_independent_evidence(self):
        p = phase(times=(1.,), x=.19 + 40 * .0625**2)
        p['blocks'][0]['coeffs'][1][0] = -80 * .0625
        p['blocks'][0]['coeffs'][2][0] = 40
        result = report.replay_phase(p, snapshot(), 1000.1)
        self.assertLess(result['sampling']['dense']['min_gap'], 0.)
        self.assertGreater(result['sampling']['K8']['min_gap'], 0.)

    def test_malformed_coefficients_durations_basis_and_gear_are_null(self):
        cases = []
        for field, value in [('basis', 'descending'), ('times', [0.]), ('times', [float('nan')]), ('gear', 1.5), ('gear', True)]:
            p = phase(); p['blocks'][0][field] = value; cases.append(p)
        p = phase(); p['blocks'][0]['coeffs'].pop(); cases.append(p)
        p = phase(); p['blocks'][0]['coeffs'][0][0] = float('nan'); cases.append(p)
        for p in cases + [None, dict(blocks=[1])]:
            result = report.replay_phase(p, snapshot(), 1000.1)
            self.assertIsNone(result['sampling'])
            self.assertIsNotNone(result['error'])

    def test_scope_proof_and_completed_dense_comparison(self):
        result = self.state(fixture(self.root))
        self.assertTrue(result['identity_certified'])
        self.assertTrue(result['nominal_comparison']['agrees'])
        self.assertTrue(result['restored_comparison']['agrees'])
        self.assertAlmostEqual(result['independent_canonical_gap_delta'], .15)
        self.assertTrue(result['canonical_status_certified'])
        self.assertEqual(result['latest_hard_accepted'], 0)

    def test_failed_trial_even_geometrically_safe_is_never_canonical_after(self):
        phases = [phase(), phase('failed_last_evaluated_trial_NOT_canonical_x', 2.)]
        result = self.state(fixture(self.root, phases=phases, rows=events(code=-1007), outcome='recovery_solver_failure'))
        self.assertEqual(result['phases'][1]['state_kind'], 'noncanonical_failed_trial')
        self.assertGreater(result['phases'][1]['sampling']['dense']['min_gap'], 0.)
        self.assertFalse(result['canonical_x_recorded'])
        self.assertIsNone(result['independent_post_gap'])
        self.assertIsNone(result['independent_canonical_gap_delta'])
        self.assertFalse(result['recorded_native_qualified'])

    def test_timeout_overrides_nonnegative_code_and_canonical_claim(self):
        result = self.state(fixture(self.root, rows=events(force=3, code=1), outcome='recovery_solver_failure'))
        self.assertEqual(result['restart_status'], 'timeout')
        self.assertFalse(result['canonical_status_certified'])
        self.assertIsNone(result['independent_post_gap'])
        self.assertIsNone(result['independent_canonical_gap_delta'])
        for row, expected in [(dict(lbfgs_result=1), 'unknown'), (dict(force_stop=0), 'unknown'),
                              (dict(force_stop=2, lbfgs_result=1), 'forced_stop'),
                              (dict(force_stop=0, lbfgs_result=-1), 'negative_result')]:
            self.assertEqual(report.restart_status(row), expected)

    def test_missing_after_is_not_zero_or_improvement(self):
        result = self.state(fixture(self.root, phases=[phase()]))
        self.assertIsNone(result['independent_post_gap'])
        self.assertIsNone(result['independent_canonical_gap_delta'])
        self.assertFalse(result['canonical_x_recorded'])

    def test_duplicate_scope_or_scan_never_certifies_comparison(self):
        for name, event in [('scope', 'coupling_scope'), ('scan', 'clearance_nominal')]:
            rows = events(); rows.insert(-1, copy.deepcopy(next(r for r in rows if r['event'] == event)))
            result = self.state(fixture(self.root, name=name, rows=rows))
            if name == 'scope':
                self.assertFalse(result['identity_certified'])
                self.assertIsNone(result['nominal_comparison'])
            else:
                self.assertIsNone(result['nominal_comparison']['agrees'])

    def test_duplicate_record_bad_filename_wrong_scope_and_pid_do_not_join(self):
        for case in ('duplicate', 'bad_name', 'wrong_scope', 'wrong_pid'):
            trial = fixture(self.root, name=case)
            path = trial / 'delivery_diagnostics/clearance_recovery/100_1.json'
            payload = json.loads(path.read_text())
            if case == 'duplicate':
                write_json(path.with_name('100_2.json'), payload)
            elif case == 'bad_name':
                path.rename(path.with_name('unknown.json'))
            elif case == 'wrong_pid':
                path.rename(path.with_name('101_1.json'))
            else:
                payload['scope_id'] = 999; write_json(path, payload)
            result = self.state(trial)
            self.assertFalse(result['identity_certified'])
            self.assertIsNone(result['independent_canonical_gap_delta'])

    def test_deadline_failure_not_overridden_by_safe_offline_geometry(self):
        rows = events(); rows[-2].update(valid=False, reason='deadline')
        result = self.state(fixture(self.root, rows=rows, outcome='original_budget_exhausted_in_native_validation'))
        self.assertGreater(result['independent_post_gap'], 0.)
        self.assertIsNone(result['restored_comparison']['agrees'])
        self.assertFalse(result['recorded_native_qualified'])

    def test_fixed_boundary_violation_is_reported_only_with_unchanged_pv(self):
        before, after = phase(x=.29), phase('restored_canonical_x', .29)
        result = self.state(fixture(self.root, phases=[before, after], outcome='restored_static_margin_not_feasible'))
        self.assertEqual(len(result['fixed_boundary_obstructions']), 2)
        self.assertIn('K8 配点仍违反', result['failure_explanation'])
        self.assertIn('基于拓扑的推断', result['failure_explanation'])
        self.assertFalse(result['phases'][0]['boundary_feasibility']['initial'])
        after['blocks'][0]['coeffs'][0][0] += .001
        a = report.replay_phase(before, snapshot(), 1000.1)
        b = report.replay_phase(after, snapshot(), 1000.1)
        self.assertEqual(report.boundary_obstruction(a, b), [])

    def test_actual_latest_gate_kept_separate_from_solver_records(self):
        trial = fixture(self.root)
        write_json(trial / 'delivery_diagnostics/candidates/100_9.json', dict(schema='delivery_candidate_v1',
                   request_id=1, scope_id=3, clock=dict(pid=100), finished=True, accepted=False,
                   failure=dict(reason='projected_static_collision')))
        result = report.summarize_trial(trial)
        self.assertEqual(result['counts']['recorded_native_qualified'], 1)
        self.assertEqual(result['counts']['latest_hard_rejected'], 1)
        self.assertEqual(result['counts']['latest_hard_accepted'], 0)
        self.assertEqual(result['state_records'][0]['latest_hard_accepted'], 0)

    def test_missing_snapshot_malformed_json_and_stale_snapshot(self):
        trial = fixture(self.root, name='missing')
        (trial / 'delivery_diagnostics/snapshots/a.json').unlink()
        self.assertIsNone(self.state(trial)['independent_pre_gap'])
        trial = fixture(self.root, name='malformed')
        (trial / 'delivery_diagnostics/clearance_recovery/100_1.json').write_text('{broken')
        result = self.state(trial)
        self.assertTrue(result['errors'])
        self.assertIsNone(result['independent_pre_gap'])
        raw = snapshot(); raw['stamp'] = 998.
        self.assertFalse(report.replay_phase(phase(), raw, 1000.1)['snapshot_fresh'])

    def test_completed_batch_only_and_no_plot_by_default(self):
        fixture(self.root, name='off', enabled=False)
        fixture(self.root, name='on')
        write_json(self.root / 'plan.json', dict(seed=11, modes=[False, True]))
        write_json(self.root / 'complete.json', dict(complete=True, completed=[dict(trial='off'), dict(trial='on')]))
        out = self.root / 'report'
        with patch.object(report, 'plot_report') as plot:
            self.assertEqual(report.main(['--root', str(self.root), '--out', str(out)]), 0)
            plot.assert_not_called()
        self.assertTrue((out / 'summary.json').is_file())
        self.assertIn('相对代价停止', (out / '实验结论.txt').read_text())
        write_json(self.root / 'complete.json', dict(complete=False, completed=[]))
        with self.assertRaises(ValueError):
            report.build_report(self.root)


if __name__ == '__main__':
    unittest.main()
