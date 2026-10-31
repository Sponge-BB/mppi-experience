#!/usr/bin/env python3
"""Contract tests: no ROS, plotting, solver or production source mutations."""
import copy
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import report_budget_scaling as report


def successful_fixture():
    marker = dict(schema='delivery_replay_v1', validation_schema='budget_final_validation_v1', valid=True, success=True,
        elapsed_s=.4, budget_s=.6, solver_result_known=True, solver_result=1, force_stop=0,
        final_x_hash='42', last_canonical_x_hash='42', last_accepted_x_hash='42',
        accepted_identity_match=True, cache_identity_known=True,
        cache_identity_match=True, output_constructed=True, output_hash='98',
        native_known=True, native_pass=True, projected_known=True, projected_pass=True,
        in_budget=True, allowed=True, reason='ok')
    failure = dict(success=True, lbfgs_result=1, force_stop=0)
    audit = dict(solve_summary=dict(native_return_success=True), recursive_return=dict(success=True),
        candidates=[dict(candidate_x_hash='42', native_IsTrajSafe_pass=True,
                         frozen_projected_pass=True)])
    return marker, failure, audit


class PrimaryTests(unittest.TestCase):
    def test_requires_full_final_marker(self):
        marker, failure, audit = successful_fixture()
        good = report.primary_endpoint(marker, failure, audit, .6, True)
        self.assertIs(good['paused_backend_deliverable'], True)
        for absent in (None, {}):
            got = report.primary_endpoint(absent, failure, audit, .6, True)
            self.assertIsNone(got['paused_backend_deliverable'])
        # Posterior accepted candidates can be good even when the final solve
        # was not usable. They do not establish delivery without its marker.
        self.assertTrue(audit['candidates'][0]['frozen_projected_pass'])

    def test_positive_needs_every_hard_proof(self):
        marker, failure, audit = successful_fixture()
        changes = dict(schema='budget_final_validation_v1', validation_schema=None,
            valid=False, solver_result_known=False, solver_result=-1,
            final_x_hash='other', last_canonical_x_hash='other', cache_identity_known=False,
            last_accepted_x_hash='other', accepted_identity_match=False,
            cache_identity_match=False, output_constructed=False, output_hash=None,
            native_known=False, native_pass=False, projected_known=False,
            projected_pass=False, in_budget=False, allowed=False, elapsed_s=.6)
        for name, value in changes.items():
            mutated = dict(marker, **{name:value})
            with self.subTest(field=name):
                got = report.primary_endpoint(mutated, failure, audit, .6, True)
                self.assertIsNone(got['paused_backend_deliverable'])
        self.assertIsNone(report.primary_endpoint(marker, failure, audit, .6, False)['paused_backend_deliverable'])

    def test_success_requires_exact_last_canonical(self):
        marker, failure, audit = successful_fixture()
        audit['candidates'][-1]['candidate_x_hash'] = 'new'
        self.assertIsNone(report.primary_endpoint(marker, failure, audit, .6, True)['paused_backend_deliverable'])

    def test_timeout_is_known_failure_not_posterior_delivery(self):
        marker, failure, audit = successful_fixture()
        marker.update(success=False, allowed=False, in_budget=False, elapsed_s=.602,
                      force_stop=3, solver_result=-1021, native_known=False, projected_known=False,
                      reason='budget_timeout')
        failure.update(success=False, lbfgs_result=-1021, force_stop=3)
        audit['solve_summary']['native_return_success'] = False
        audit['recursive_return']['success'] = False
        got = report.primary_endpoint(marker, failure, audit, .6, True)
        self.assertIs(got['paused_backend_deliverable'], False)
        self.assertTrue(got['evidence_valid'])
        self.assertTrue(audit['candidates'][0]['frozen_projected_pass'])

    def test_return_disagreement_is_unknown(self):
        marker, failure, audit = successful_fixture()
        audit['recursive_return']['success'] = False
        self.assertIsNone(report.primary_endpoint(marker, failure, audit, .6, True)['paused_backend_deliverable'])


class AggregateTests(unittest.TestCase):
    def trial(self, values, validity=True):
        return dict(phases=[dict(evidence_valid=validity, paused_backend_deliverable=v)
                            for v in values])

    def test_unknown_retained_in_fixed_denominator(self):
        trials = [self.trial([True]*5), self.trial([False]*5), self.trial([None]*5),
                  self.trial([True]*5, False)]
        for group in report.aggregate(trials):
            self.assertEqual((group['passed'], group['failed'], group['unknown']), (1, 1, 2))
            self.assertEqual(group['scheduled_requests'], 4)
            self.assertIsNone(group['success_rate'])
            self.assertEqual((group['lower_bound'], group['upper_bound']), (.25, .75))

    def test_complete_rate_requires_no_unknown(self):
        groups = report.aggregate([self.trial([False]*5) for _ in range(4)])
        self.assertTrue(all(g['success_rate'] == 0. and g['complete_rate_known'] for g in groups))

    def test_margin_sensitivity_does_not_change_native_result(self):
        candidate = dict(frozen_projected_20ms=dict(worst_static=dict(physical=.07)))
        got = report.margin_sensitivity(candidate, False)
        self.assertEqual([r['sampled_projection_pass'] for r in got['cases']], [True, True, False])
        self.assertTrue(all(r['native_return_success'] is False for r in got['cases']))
        self.assertFalse(got['changes_primary'])
        self.assertIsNone(report.margin_sensitivity({}, False).get('min_physical_clearance_m'))

    def test_missing_capture_explicit_not_normal_success(self):
        with tempfile.TemporaryDirectory() as directory:
            got = report.trial_report(Path(directory)/'missing', dict(seed=11, ordinal=1, captured=False))
        self.assertFalse(got['captured'])
        self.assertEqual(len(got['phases']), 5)
        self.assertTrue(all(p['paused_backend_deliverable'] is None for p in got['phases']))

    def test_gate_phase_specific_and_fixed_budget(self):
        gate = dict(kind='feasibility_stop_summary', phase=report.PHASES[2], enabled=True,
            checks=3, blocked_count=3, allowed_count=0, native_pass_count=0,
            projected_pass_count=0, budget_expired_count=1, fatal=False,
            native_wall_s=.01, projected_wall_s=0., gate_wall_s=.01,
            feasibility_decision_elapsed_s=3.002, original_budget_s=3.,
            effective_variant='cost_stop_veto')
        self.assertTrue(report.gate_evidence([gate], report.PHASES[2], 3.)['evidence_valid'])
        self.assertFalse(report.gate_evidence([gate], report.PHASES[2], .6)['evidence_valid'])
        self.assertFalse(report.gate_evidence([gate, gate], report.PHASES[2], 3.)['evidence_valid'])


if __name__ == '__main__':
    unittest.main()
