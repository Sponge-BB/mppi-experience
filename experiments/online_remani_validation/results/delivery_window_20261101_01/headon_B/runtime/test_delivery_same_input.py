#!/usr/bin/env python3
import unittest
from unittest import mock

import report_delivery_same_input as report
import run_delivery_same_input as runner


class SameInputTests(unittest.TestCase):
    def test_scope_restores_lifecycle_after_error(self):
        original = runner.lifecycle.prepare_capture
        with mock.patch.object(runner.lifecycle, 'run_capture', side_effect=RuntimeError('test')):
            with self.assertRaises(RuntimeError):
                runner.run_capture(None, None)
        self.assertIs(runner.lifecycle.prepare_capture, original)

    def test_recovered_timeout_is_backend_success(self):
        summary = dict(valid=True, phase=runner.PHASES[1], budget_s=.6, success=True,
            force_stop=3, reason='ok', first_signature={}, candidates=[],
            solve_summary=dict(input_hash='42'))
        resources = [dict(event='end', stage='optimizer', label=runner.PHASES[1],
            success=True, input_hash=42, candidate_mode=1, force_stop=3,
            salvage_attempt=1, salvage_native_result=1)]
        with mock.patch.object(report, 'summarize_phase', return_value=summary):
            row = report.phase_report(runner.PHASES[1], 1, [], [], resources)
        self.assertTrue(row['success'])
        self.assertEqual(row['force_stop'], 3)
        self.assertTrue(all(row['checks'].values()))
        self.assertTrue(row['native_candidate_not_live_delivery'])

    def test_all_four_outcomes_are_not_always_improvement(self):
        def result(a, b):
            return report.conclusion(dict(captured=True, valid=True,
                phases=[dict(success=a), dict(success=b)]))
        self.assertIn('不能说修好了', result(True, True))
        self.assertIn('原组失败', result(False, True))
        self.assertIn('退化', result(True, False))
        self.assertIn('没有解决', result(False, False))

    def test_invalid_or_missing_capture_cannot_claim_effect(self):
        self.assertIn('没有有效', report.conclusion(dict(captured=False, valid=False)))
        self.assertIn('暂不能', report.conclusion(dict(captured=True, valid=False)))


if __name__ == '__main__':
    unittest.main()
