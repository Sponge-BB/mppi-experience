"""Synthetic report contracts only; never starts ROS or reads trial archives."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import report_boundary_admission as report


class ClassificationTests(unittest.TestCase):
    def test_native_strict_epsilon(self):
        for gap, expected in ((0., 'full_margin'), (-5e-9, 'native_tolerance'),
                              (-1e-8, 'native_tolerance'), (-2e-8, 'margin_only')):
            self.assertEqual(report.category(dict(physical=.1, gap=gap)), expected)
        self.assertEqual(report.category(dict(physical=0., gap=-.1)), 'physical_collision')

    def test_missing_is_unknown_not_safe(self):
        for value in (None, {}, dict(physical=.1), dict(physical=float('nan'), gap=0.)):
            self.assertIsNone(report.category(value))


class DeferTests(unittest.TestCase):
    def rows(self, success=False):
        return [dict(pid=42, request_id=1, event=event, **extra) for event, extra in (
            ('request_begin', {}), ('stage_begin', dict(stage='frontend')),
            ('state_contract_manager_input', {}), ('state_contract_frontend_init', {}),
            ('boundary_admission_precheck', dict(deferred=True, would_defer=True, eligible=True)),
            ('boundary_admission_manager_deferred', dict(optimizer_started=False, manager_success=False)),
            ('state_contract_request_end', dict(success=success, capture_sim=1010.)))]

    def analyze(self, rows):
        with patch.object(report, 'precheck_evidence', return_value=None):
            return report.analyze_requests(Path('/unused'), rows, None, None)

    def test_defer_allows_frontend_manager_but_not_success(self):
        result = self.analyze(self.rows())
        self.assertEqual(result['total_requests'], 1)
        self.assertEqual(result['deferred'], 1)
        self.assertEqual(result['backend_solves'], 0)
        self.assertEqual(result['successful_requests'], 0)
        self.assertTrue(result['records'][0]['defer_contract_valid'])
        self.assertEqual(result['defer_contract_violations'], 0)

    def test_defer_claiming_success_never_counted_success(self):
        result = self.analyze(self.rows(success=True))
        self.assertEqual(result['request_success_flags_true'], 1)
        self.assertEqual(result['successful_requests'], 0)
        self.assertEqual(result['defer_contract_violations'], 1)

    def test_optimizer_or_publication_is_contract_violation(self):
        for child in (dict(event='stage_begin', stage='optimizer'),
                      dict(event='state_contract_reference_publish'),
                      dict(event='state_contract_backend_call')):
            rows = self.rows() + [dict(pid=42, request_id=1, **child)]
            self.assertEqual(self.analyze(rows)['defer_contract_violations'], 1)

    def test_missing_end_or_duplicate_marker_fails_closed(self):
        rows = self.rows()
        self.assertIsNot(self.analyze(rows[:-1])['records'][0]['defer_contract_valid'], True)
        rows.append(copy.deepcopy(rows[-2]))
        self.assertIsNot(self.analyze(rows)['records'][0]['defer_contract_valid'], True)

    def test_missing_timeline_not_zero_measurement(self):
        result = self.analyze(None)
        self.assertFalse(result['present'])
        for key in ('total_requests', 'deferred', 'successful_requests', 'backend_solves'):
            self.assertIsNone(result[key])


class ExactEvidenceTests(unittest.TestCase):
    def fixture(self):
        return (dict(boundary_admission_schema=report.SCHEMA, same_native_gap_tolerance=report.EPS,
                     classification='margin_only'),
                dict(independent_fixed_static=dict(physical=.08, gap=-.02),
                     independent_raw_static=dict(physical=.08, gap=-.02),
                     payload_hash_verified=True, raw_online_agreement=True, fixed_online_agreement=True))

    def evidence(self, row, frame, observed=True):
        with patch.object(report, 'snapshot_evidence', return_value=frame), \
             patch.object(report, 'cpp_identity', return_value={}), \
             patch.object(report, 'match_observer', return_value=dict(exact_message_observed=observed)):
            return report.precheck_evidence(Path('/unused'), row, None)

    def test_complete_evidence_and_missing_required_parts(self):
        row, frame = self.fixture()
        self.assertTrue(self.evidence(row, frame)['exact_evidence'])
        for field in ('payload_hash_verified', 'raw_online_agreement', 'fixed_online_agreement',
                      'independent_fixed_static'):
            damaged = dict(frame); damaged.pop(field)
            self.assertIsNot(self.evidence(row, damaged)['exact_evidence'], True)
        for field in ('boundary_admission_schema', 'same_native_gap_tolerance', 'classification'):
            damaged = dict(row); damaged.pop(field)
            self.assertIsNot(self.evidence(damaged, frame)['exact_evidence'], True)
        self.assertIsNot(self.evidence(row, frame, observed=None)['exact_evidence'], True)
        self.assertIsNot(self.evidence(dict(row, same_native_gap_tolerance=0.), frame)['exact_evidence'], True)


class EndToEndTests(unittest.TestCase):
    def run_latency(self, invoked=True):
        source = dict(cycle_id=7, first_kernel_cycle_for_reference=True)
        analyzed = dict(cycle_id=7, online_step_invoked=invoked, unique_cycle_id=True,
                        online_step_entry_ros_time=1014., kernel_code=0,
                        solve=dict(exact_message_observed=True, state_equals_identity=True),
                        reference=dict(complete_exact_reference_chain=True, transaction_fnv64='55', transaction_sha256='abc'))
        native = [dict(event='boundary_admission_precheck', deferred=True, capture_sim=1005.),
                  dict(event='state_contract_reference_publish', capture_sim=1013., transaction_fnv64='55', request_id=4)]
        with patch.object(report, 'analyze_cycles', return_value=dict(rows=[analyzed])), \
             patch.object(report, 'reference_key', return_value=('exact-key',)), \
             patch.object(report, 'sampled_payload', return_value=(dict(payload_sha256_verified=True), None)):
            return report.end_to_end(dict(motion_anchor=0., elapsed=20.), 'static', native, [source], [], [{}], [], [])

    def test_all_defer_frontend_retry_waits_retained(self):
        result = self.run_latency()
        self.assertEqual(result['appearance_ros_time'], 1004.)
        self.assertEqual(result['latency_s'], 10.)
        self.assertFalse(result['right_censored'])
        self.assertEqual(result['request_id'], 4)

    def test_missing_exact_update_is_censored_not_zero(self):
        result = self.run_latency(invoked=False)
        self.assertIsNone(result['latency_s'])
        self.assertTrue(result['right_censored'])
        self.assertEqual(result['censor_lower_bound_s'], 16.)
        unknown = report.end_to_end({}, 'static', None, None, None, None, None, None)
        self.assertIsNone(unknown['right_censored'])
        self.assertIsNone(unknown['latency_s'])


class BatchTests(unittest.TestCase):
    def fixture(self, root):
        names = ['11_off', '11_on', '12_on', '12_off']
        for name in names:
            path = root/name; path.mkdir(); (path/'run_status.json').write_text('{}')
        (root/'plan.json').write_text('{}')
        (root/'complete.json').write_text(json.dumps(dict(complete=True, completed=[dict(trial=n) for n in names])))
        def trial(path):
            seed, mode = path.name.split('_')
            settings = dict(boundary_admission=mode=='on', fixed=True)
            return dict(seed=int(seed), admission=mode=='on', settings=settings,
                        runtime_sha256={'runner.py':'sha'}, build='/frozen', build_manifest_sha256='manifest-sha',
                        scope_checks=dict(static_scene=True))
        return names, trial

    def build(self, root, trial):
        with patch.object(report, 'direction_proof', return_value=dict(passed=True)), \
             patch.object(report, 'assert_plot_quiet'), patch.object(report, 'trial_report', side_effect=trial):
            return report.build_report(root)

    def test_four_jobs_and_treatment_only_pairs(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); _, trial=self.fixture(root)
            result=self.build(root, trial)
            self.assertTrue(all(v is True for pair in result['pair_checks'].values() for v in pair.values()))
            def changed(path):
                data=trial(path)
                if path.name=='11_on':
                    data['runtime_sha256']={'runner.py':'different'}; data['settings']['extra']=True
                return data
            pair=self.build(root, changed)['pair_checks']['11']
            self.assertFalse(pair['runtime_full_hash_equal'])
            self.assertFalse(pair['treatment_only_recorded_flag'])
            (root/'complete.json').write_text(json.dumps(dict(complete=True, completed=[dict(trial='11_off')])))
            with self.assertRaises(ValueError):self.build(root, trial)

    def test_wrong_seed_or_duplicate_trial_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); names, trial=self.fixture(root)
            def wrong(path):
                value=trial(path); value['seed']=11; return value
            with self.assertRaises(ValueError):self.build(root, wrong)
            names[-1]=names[0]
            (root/'complete.json').write_text(json.dumps(dict(complete=True, completed=[dict(trial=n) for n in names])))
            with self.assertRaises(ValueError):self.build(root, trial)

    def test_directed_proof_hash_and_missing_pass_flag(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'proof.json'
            proof=dict(schema='directed_low_speed_verdict_v1', verified_signed_negative_native_chain=True,
                       same_fixture=True, same_runtime=True)
            path.write_text(json.dumps(proof)); plan=dict(direction_proof=str(path), direction_proof_sha256=report.digest(path))
            self.assertTrue(report.direction_proof(plan)['passed'])
            for field in ('same_fixture', 'same_runtime', 'verified_signed_negative_native_chain'):
                bad=dict(proof); bad.pop(field); path.write_text(json.dumps(bad))
                bad_plan=dict(plan,direction_proof_sha256=report.digest(path))
                with self.assertRaises(ValueError):report.direction_proof(bad_plan)
            with self.assertRaises(ValueError):report.direction_proof(plan)


if __name__=='__main__':unittest.main()
