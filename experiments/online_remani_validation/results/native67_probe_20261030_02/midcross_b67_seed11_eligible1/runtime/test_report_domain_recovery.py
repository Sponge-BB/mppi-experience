#!/usr/bin/env python3
"""Tiny synthetic fixtures only: no ROS, models, bulk logs, or plots."""
import copy
import gzip
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import report_domain_recovery as report


PID, TID, REQUEST, REPLAN, OPTIMIZER, HASH = 101, 101, 1, 7, 8, 17999999999999999999


def native_rows(enabled=True, rejects=2, accepted=1, result=1):
    common = dict(pid=PID, request_id=REQUEST)
    return [dict(common, event='request_begin', mono=1.),
            dict(common, event='resource_link', mono=1.1, resource_replan_id=REPLAN),
            dict(common, event='coupling_scope', mono=2.1, scope_id=13, input_hash=HASH),
            dict(common, event='domain_solver_status', mono=3., enabled=int(enabled),
                 rejected_trials=rejects, accepted_steps_after_rejection=accepted, lbfgs_result=result),
            dict(common, event='request_end', mono=6.)]


def trace_rows(result=1, force=0, success=True):
    common = dict(pid=PID, tid=TID, label='normal')
    return [dict(common, event='begin', stage='replan', id=REPLAN, mono=1.05),
            dict(common, event='begin', stage='optimizer', id=OPTIMIZER, mono=2.),
            dict(common, event='end', stage='optimizer', id=OPTIMIZER, mono=4., input_hash=HASH,
                 lbfgs_result=result, force_stop=force, success=success),
            dict(common, event='end', stage='replan', id=REPLAN, mono=5., success=success)]


def candidate(accepted=True, finished=True, scope=13):
    return dict(schema='delivery_candidate_v1', clock=dict(pid=PID), request_id=REQUEST, scope_id=scope,
                finished=finished, accepted=accepted, failure=None if accepted else dict(reason='latest_prediction_collision'))


def write_json(path, data):
    path.write_text(json.dumps(data), encoding='utf-8')


def write_lines(path, rows):
    payload = ''.join(json.dumps(row) + '\n' for row in rows)
    if path.suffix == '.gz':
        with gzip.open(path, 'wt', encoding='utf-8') as stream:
            stream.write(payload)
    else:
        path.write_text(payload, encoding='utf-8')


def fixture(root, name='on', enabled=True, native=None, trace=None, candidates=None, compressed=False):
    trial = root / name
    trial.mkdir()
    write_json(trial / 'manifest.json', dict(scene='static', build='synthetic-build', build_manifest_sha256='same',
        coupling_experiment=dict(domain_recovery=enabled, seed=11, terminal_solver_failure_fail_closed=True)))
    write_json(trial / 'run_status.json', dict(returncode=0, result_written=True, runner_error=None))
    write_json(trial / 'trial_audit.json', dict(valid_experiment=True, errors=[], navigation_outcome='timeout'))
    write_json(trial / 'result.json', dict(reason='timeout', collision_conservative_projection=False, final_state=[1., 0.]))
    suffix = '.gz' if compressed else ''
    write_lines(trial / ('dual_timeline.jsonl' + suffix), native if native is not None else native_rows(enabled))
    if trace is not False:
        write_lines(trial / ('solver_trace.jsonl' + suffix), trace if trace is not None else trace_rows())
    if candidates is not False:
        directory = trial / 'delivery_diagnostics/candidates'
        directory.mkdir(parents=True)
        for index, row in enumerate(candidates if candidates is not None else [candidate()]):
            write_json(directory / ('%d.json' % index), row)
    return trial


class DomainReportTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def test_exact_identity_success_and_final_hard_gate_are_separate(self):
        result = report.summarize_trial(fixture(self.root, candidates=[candidate(False)]))
        self.assertEqual(result['domain']['rejected_trials'], 2)
        self.assertEqual(result['domain']['accepted_steps_after_rejection'], 1)
        self.assertEqual(result['domain']['certified_successful_solves_after_rejection'], 1)
        self.assertEqual(result['domain']['certified_recovered_hard_accepts'], 0)
        self.assertEqual(result['optimizer_trace']['successful_optimizer_solves'], 1)
        self.assertEqual(result['hard_candidates']['rejected'], 1)
        self.assertEqual(result['outcome']['reason'], 'timeout')
        self.assertIs(result['outcome']['collision_conservative_projection'], False)

    def test_positive_result_and_accepted_step_can_still_timeout(self):
        result = report.summarize_trial(fixture(self.root, trace=trace_rows(force=3, success=False)))
        status = result['solver_status_records'][0]
        self.assertEqual(status['certification'], 'certified')
        self.assertEqual(status['force_stop_name'], 'STOP_FOR_TIMEOUT')
        self.assertFalse(status['actual_successful_solve'])
        self.assertEqual(result['optimizer_trace']['nonnegative_but_timeout'], 1)
        self.assertEqual(result['domain']['certified_successful_solves_after_rejection'], 0)
        self.assertEqual(result['domain']['accepted_steps_after_rejection'], 1)

    def test_negative_after_earlier_recovery_is_not_success(self):
        rows = native_rows(result=-1007)
        rows.insert(-1, dict(pid=PID, request_id=REQUEST, event='domain_terminal_reject', lbfgs_result=-1007, mono=4.1))
        result = report.summarize_trial(fixture(self.root, native=rows, trace=trace_rows(result=-1007, success=False)))
        self.assertEqual(result['domain']['domain_terminal_reject'], 1)
        self.assertEqual(result['domain']['lbfgs_result'], {'-1007': 1})
        self.assertEqual(result['domain']['certified_successful_solves_after_rejection'], 0)

    def test_initial_reject_has_no_solver_status(self):
        rows = [r for r in native_rows() if r['event'] != 'domain_solver_status']
        rows.insert(-1, dict(pid=PID, request_id=REQUEST, event='domain_initial_reject', reason='initial_static_map_domain', mono=3.))
        rows.insert(-1, dict(pid=PID, request_id=REQUEST, event='coupling_objective_error', reason='bad', mono=3.1))
        result = report.summarize_trial(fixture(self.root, native=rows, trace=False, candidates=False))
        self.assertEqual(result['domain']['domain_solver_status'], 0)
        self.assertEqual(result['domain']['domain_initial_reject'], 1)
        self.assertEqual(result['domain']['coupling_objective_error'], 1)
        self.assertEqual(result['domain']['rejected_trials'], 0)

    def test_no_identity_proof_no_time_based_certification(self):
        rows = [r for r in native_rows() if r['event'] != 'resource_link']
        result = report.summarize_trial(fixture(self.root, native=rows))
        self.assertEqual(result['domain']['status_not_certified'], 1)
        self.assertNotIn('force_stop', result['solver_status_records'][0])
        self.assertEqual(result['optimizer_trace']['successful_optimizer_solves'], 1)

    def test_thread_local_id_collision_fails_closed(self):
        rows = trace_rows()
        rows += [dict(r, tid=202, mono=r['mono'] + 10) for r in trace_rows()]
        result = report.summarize_trial(fixture(self.root, trace=rows))
        self.assertEqual(result['domain']['status_not_certified'], 1)

    def test_duplicate_status_link_or_coupling_scope_fails_closed(self):
        for event in ('domain_solver_status', 'resource_link', 'coupling_scope'):
            with self.subTest(event=event):
                rows = native_rows()
                rows.insert(-1, copy.deepcopy(next(r for r in rows if r['event'] == event)))
                result = report.summarize_trial(fixture(self.root, name=event, native=rows))
                self.assertTrue(all(r['certification'] == 'status-not-certified' for r in result['solver_status_records']))

    def test_two_optimizers_in_request_are_ambiguous(self):
        rows = trace_rows()
        second = [dict(r, id=9, mono=r['mono'] + .5) for r in rows[1:3]]
        rows[3:3] = second
        result = report.summarize_trial(fixture(self.root, trace=rows))
        self.assertEqual(result['domain']['status_not_certified'], 1)

    def test_unclosed_misordered_or_backwards_scope_fails_closed(self):
        unclosed = trace_rows()[:-1]
        misordered = trace_rows()
        misordered[2], misordered[3] = misordered[3], misordered[2]
        backwards = trace_rows()
        backwards[2]['mono'] = 1.5
        for index, rows in enumerate((unclosed, misordered, backwards)):
            with self.subTest(case=index):
                result = report.summarize_trial(fixture(self.root, name=str(index), trace=rows))
                self.assertEqual(result['domain']['status_not_certified'], 1)

    def test_wrong_pid_hash_or_scope_does_not_cross_match(self):
        rows = trace_rows()
        rows[2]['input_hash'] += 1
        result = report.summarize_trial(fixture(self.root, name='wrong_hash', trace=rows))
        self.assertEqual(result['domain']['status_not_certified'], 1)
        for name, c in [('wrong_scope', candidate(scope=99)), ('wrong_pid', candidate())]:
            if name == 'wrong_pid':
                c['clock']['pid'] += 1
            result = report.summarize_trial(fixture(self.root, name=name, candidates=[c]))
            self.assertEqual(result['domain']['certified_successful_solves_after_rejection'], 1)
            self.assertEqual(result['domain']['certified_recovered_hard_accepts'], 0)

    def test_missing_fields_never_zero_or_accepted_by_default(self):
        rows = native_rows()
        del rows[3]['rejected_trials']
        c = candidate()
        del c['accepted']
        result = report.summarize_trial(fixture(self.root, native=rows, trace=False, candidates=[c]))
        self.assertIsNone(result['domain']['rejected_trials'])
        self.assertIsNone(result['optimizer_trace']['successful_optimizer_solves'])
        self.assertEqual(result['hard_candidates']['unknown'], 1)
        self.assertEqual(result['hard_candidates']['accepted'], 0)
        self.assertEqual(result['hard_candidates']['rejected'], 0)
        missing = report.summarize_trial(fixture(self.root, name='missing', candidates=False))
        self.assertIsNone(missing['hard_candidates']['accepted'])
        self.assertIsNone(missing['hard_candidates']['observed_count'])
        self.assertIsNone(missing['domain']['certified_recovered_hard_accepts'])

    def test_truncated_trace_does_not_imply_zero_solves(self):
        result = report.summarize_trial(fixture(self.root, trace=trace_rows()[:2]))
        self.assertIsNone(result['optimizer_trace']['successful_optimizer_solves'])
        self.assertFalse(result['optimizer_trace']['scopes_complete'])

    def test_wrong_request_clock_is_not_certified(self):
        rows = native_rows()
        rows[3]['mono'] = 99.
        result = report.summarize_trial(fixture(self.root, native=rows))
        self.assertEqual(result['domain']['status_not_certified'], 1)

    def test_explicit_candidate_finished_and_online_accept(self):
        unfinished = candidate(finished=False)
        alias = candidate()
        del alias['accepted']
        alias['online_accept'] = True
        result = report.summarize_trial(fixture(self.root, candidates=[unfinished, alias]))
        self.assertEqual(result['hard_candidates']['unfinished'], 1)
        self.assertEqual(result['hard_candidates']['accepted'], 1)

    def test_gzip_is_supported_without_double_counting(self):
        trial = fixture(self.root, compressed=True)
        result = report.summarize_trial(trial)
        self.assertEqual(result['domain']['domain_solver_status'], 1)
        self.assertIn('dual_timeline.jsonl.gz', result['input_sha256'])
        write_lines(trial / 'dual_timeline.jsonl', native_rows(rejects=7))
        self.assertEqual(report.summarize_trial(trial)['domain']['rejected_trials'], 7)

    def make_batch(self):
        fixture(self.root, name='off', enabled=False, native=native_rows(False, 0, 0))
        fixture(self.root, name='on', native=native_rows(True, 0, 0))
        write_json(self.root / 'plan.json', dict(jobs=[dict(seed=11, domain_recovery=False), dict(seed=11, domain_recovery=True)]))
        write_json(self.root / 'complete.json', dict(complete=True, completed=[dict(trial='off', valid=True), dict(trial='on', valid=True)]))

    def test_full_batch_no_excitation_conclusion_and_fresh_output(self):
        self.make_batch()
        result = report.build_report(self.root)
        self.assertFalse(result['domain_recovery_exercised'])
        self.assertIn('不能据此证明成功域恢复', result['conclusion'])
        self.assertIn('旧诊断批次不是精确 OFF 对照', report.report_text(result))
        out = self.root / 'new_report'
        with patch.object(report, 'plot_report') as plot:
            self.assertEqual(report.main(['--root', str(self.root), '--out', str(out)]), 0)
            plot.assert_not_called()
        self.assertTrue((out / 'summary.json').is_file())
        self.assertTrue((out / '实验结论.txt').is_file())
        self.assertFalse((out / 'diagnostic_counts.png').exists())
        with self.assertRaises(SystemExit):
            report.main(['--root', str(self.root), '--out', str(out)])

    def test_incomplete_batch_never_produces_report(self):
        write_json(self.root / 'plan.json', {})
        write_json(self.root / 'complete.json', dict(complete=False, completed=[]))
        out = self.root / 'report'
        with self.assertRaises(ValueError):
            report.main(['--root', str(self.root), '--out', str(out)])
        self.assertFalse(out.exists())

    def test_missing_outcome_is_not_silently_filled(self):
        trial = fixture(self.root)
        (trial / 'result.json').unlink()
        with self.assertRaises(FileNotFoundError):
            report.summarize_trial(trial)

    def test_invalid_audit_and_collision_outcome_are_retained(self):
        trial = fixture(self.root)
        write_json(trial / 'trial_audit.json', dict(valid_experiment=False, errors=['synthetic failure']))
        write_json(trial / 'result.json', dict(reason='collision', collision_conservative_projection=True))
        result = report.summarize_trial(trial)
        self.assertFalse(result['audit']['valid_experiment'])
        self.assertTrue(result['outcome']['collision_conservative_projection'])
        self.assertEqual(result['outcome']['reason'], 'collision')


if __name__ == '__main__':
    unittest.main()
