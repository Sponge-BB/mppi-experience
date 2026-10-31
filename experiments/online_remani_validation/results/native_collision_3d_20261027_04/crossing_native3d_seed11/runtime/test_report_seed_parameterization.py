"""Synthetic-only report contracts; no ROS, compiler, archives or plotting."""
import copy
import unittest
from unittest.mock import patch

from report_seed_parameterization import adoption_evidence, pair_checks, phase_for, seed_evidence, summarize_requests


def request(number, fsm=3, pid=42):
    return [dict(event='request_begin', request_id=number, pid=pid, fsm=fsm),
        dict(event='state_contract_request_end', request_id=number, pid=pid, success=False)]


def backend(number, value='123', pid=42):
    base = dict(request_id=number, pid=pid)
    return [dict(base, event='stage_begin', stage='optimizer'),
        dict(base, event='state_contract_backend_fixed', input_hash=value,
            state_contract_backend_id=number, state_contract_backend_id_valid=True),
        dict(base, event='state_contract_backend_scope_link', input_hash=value,
            state_contract_backend_id=number, scope_id=number + 10),
        dict(base, event='coupling_scope', input_hash=value, scope_id=number + 10)]


class RequestTests(unittest.TestCase):
    def test_fsm_not_request_number_defines_initial(self):
        native = request(1, 3) + request(2, 2) + request(3, 2) + request(4, 99)
        answer = summarize_requests(native, [])
        self.assertEqual(answer['initial']['requests'], 2)
        self.assertEqual(answer['replan']['requests'], 1)
        self.assertEqual(answer['unknown_phase']['requests'], 1)
        self.assertEqual(answer['records'][0]['phase'], 'replan')

    def test_missing_or_duplicate_begin_unknown(self):
        self.assertEqual(phase_for([]), 'unknown')
        self.assertEqual(phase_for(request(1) + request(1)), 'unknown')

    def test_native_success_is_not_wrapper_success(self):
        native = request(2)
        native += [dict(event='delivery_search_exit', pid=42, request_id=2,
                        solver='kino_astar', probe_call_id=2, probe_parent_call_id=1,
                        return_code=2, exit_reason='oneshot_reach_end'),
                   dict(event='delivery_search_exit', pid=42, request_id=2,
                        solver='frontend_wrapper', probe_call_id=1,
                        return_code=3, exit_reason='arm_sampling_or_rrt_failed')]
        answer = summarize_requests(native, [])['replan']
        self.assertEqual(answer['wrapper_failures'], 1)
        self.assertEqual(answer['astar_success_then_sampling_failures'], 1)
        self.assertEqual(answer['optimizer_entered_requests'], 0)

    def test_exact_backend_scope_and_unique_hash_required(self):
        native = request(2) + backend(2)
        failure = [dict(kind='solve', label='normal', input_hash='123', reason='budget_timeout', success=False)]
        answer = summarize_requests(native, failure)['replan']
        self.assertEqual(answer['backend_reason_counts'], {'budget_timeout': 1})
        for damaged in (native[:-1], native + request(3) + backend(3),
                        native + [dict(native[-1])]):
            self.assertEqual(summarize_requests(damaged, failure)['replan']['backend_reason_counts'], {})
        self.assertEqual(summarize_requests(native, failure + failure)['replan']['backend_reason_counts'], {})

    def test_normal_failure_only_and_no_publish_inference(self):
        native = request(2) + backend(2)
        failure = [dict(kind='solve', label='diagnostic', input_hash='123', reason='ok', success=True)]
        native += [dict(event='state_contract_solver_return', pid=42, request_id=2, success=True),
                   dict(event='acceptance_reject', pid=42, request_id=2, reason='additional_projected_qualification')]
        answer = summarize_requests(native, failure)['replan']
        self.assertEqual(answer['unknown_backend_links'], 1)
        self.assertEqual(answer['published_requests'], 0)
        self.assertEqual(answer['terminal_rejections'], 1)

    def test_initial_backend_not_in_replan_denominator(self):
        native = request(1, 2) + backend(1) + request(2, 3)
        answer = summarize_requests(native, [])
        self.assertEqual(answer['all']['optimizer_entered_requests'], 1)
        self.assertEqual(answer['initial']['optimizer_entered_requests'], 1)
        self.assertEqual(answer['replan']['optimizer_entered_requests'], 0)
        self.assertEqual(answer['replan']['optimizer_entry_fraction'], 0.)

    def test_missing_log_unknown_not_zero(self):
        answer = summarize_requests(None, None)
        self.assertFalse(answer['present'])
        self.assertIsNone(answer['all']['requests'])
        self.assertIsNone(answer['replan']['optimizer_entered_requests'])


class SeedEvidenceTests(unittest.TestCase):
    def fixture(self, enabled=True):
        event = dict(event='seed_parameterization', seed_parameterization_schema='seed_parameterization_v1',
            enabled=enabled, applied=enabled, valid=True, reason='ok' if enabled else 'legacy_not_repaired',
            input_points=9, output_segments=2, signed_start_speed=-.05, first_gear=-1,
            probe_call_id_available=False)
        wrapper = dict(call_id=4, native_children=[dict(return_code=2)])
        return event, wrapper

    def test_actual_switch_and_success_not_inferred_from_manifest(self):
        event, wrapper = self.fixture()
        answer = seed_evidence([event], [wrapper], True)[0]
        self.assertTrue(answer['evidence_valid']); self.assertTrue(answer['certified_applied'])
        self.assertTrue(answer['repaired_positive_seed'])
        self.assertFalse(seed_evidence([event], [wrapper], False)[0]['evidence_valid'])
        event['valid'] = False; event['reason'] = 'invalid_geometry_or_time_profile'; event['output_segments'] = 0
        answer = seed_evidence([event], [wrapper], True)[0]
        self.assertTrue(answer['certified_applied']); self.assertFalse(answer['repaired_positive_seed'])
        self.assertTrue(answer['astar_success_with_invalid_seed']); self.assertTrue(answer['empty_output_seed'])

    def test_no_probe_id_multi_wrapper_or_event_ambiguous(self):
        event, wrapper = self.fixture()
        for events, wrappers in (([event, event], [wrapper]), ([event], [wrapper, wrapper])):
            answer = seed_evidence(events, wrappers, True)[0]
            self.assertFalse(answer['unique_request_wrapper_association'])
            self.assertFalse(answer['evidence_valid']); self.assertIsNone(answer['linked_wrapper_call_id'])

    def test_off_empty_seed_and_corrupt_schema_fail_closed(self):
        event, wrapper = self.fixture(False)
        event['valid'] = False; event['output_segments'] = 0; event['first_gear'] = 0
        answer = seed_evidence([event], [wrapper], False)[0]
        self.assertTrue(answer['evidence_valid']); self.assertTrue(answer['empty_output_seed'])
        self.assertFalse(answer['certified_applied'])
        for field, value in (('seed_parameterization_schema', 'future'), ('enabled', 1),
                             ('input_points', True), ('signed_start_speed', float('nan')),
                             ('probe_call_id_available', True), ('applied', True), ('valid', True)):
            damaged = dict(event); damaged[field] = value
            self.assertFalse(seed_evidence([damaged], [wrapper], False)[0]['evidence_valid'])


class AdoptionTests(unittest.TestCase):
    def test_missing_every_timeline_safe_and_no_invented_adoption(self):
        self.assertEqual(adoption_evidence(None, None, None, None, None, None), [])

    def fixture(self):
        pub = dict(event='state_contract_reference_publish', pid=42, request_id=2,
            transaction_fnv64='123', capture_sim=1005.)
        cycle = dict(cycle_id=1, first_kernel_cycle_for_reference=True,
            guide_revision=1, guide_epoch=1, message_hashes=['a' * 64],
            reference_array_sha256='b' * 64)
        analyzed = dict(cycle_id=1, online_step_invoked=True, unique_cycle_id=True,
            solve=dict(exact_message_observed=True, state_equals_identity=True),
            reference=dict(complete_exact_reference_chain=True, transaction_fnv64='123'),
            online_step_entry_ros_time=1005.1, actual_validated_command=False)
        consumed = dict(revision=1, epoch=1, message_hashes=['a' * 64], sha256='b' * 64)
        return pub, cycle, analyzed, consumed

    def analyze(self, native, cycles, analyzed, consumed):
        with patch('report_seed_parameterization.analyze_cycles', return_value=dict(rows=analyzed)), \
             patch('report_seed_parameterization.sampled_payload', return_value=(dict(payload_sha256_verified=True), [])):
            return adoption_evidence(native, cycles, [], consumed, [], [])

    def test_adopted_does_not_claim_validated_command(self):
        pub, cycle, analyzed, consumed = self.fixture()
        answer = self.analyze([pub], [cycle], [analyzed], [consumed])[0]
        self.assertTrue(answer['exact_adopted'])
        self.assertFalse(answer['adoption']['actual_validated_command'])

    def test_duplicate_pub_or_cycle_and_missing_odom_fail_closed(self):
        pub, cycle, analyzed, consumed = self.fixture()
        for pubs, cycles, rows in (([pub, pub], [cycle], [analyzed]),
                                  ([pub], [cycle, cycle], [analyzed])):
            self.assertFalse(self.analyze(pubs, cycles, rows, [consumed])[0]['exact_adopted'])
        analyzed['solve']['exact_message_observed'] = False
        self.assertFalse(self.analyze([pub], [cycle], [analyzed], [consumed])[0]['exact_adopted'])

    def test_entry_before_publication_or_wrong_reference_not_adoption(self):
        pub, cycle, analyzed, consumed = self.fixture()
        analyzed['online_step_entry_ros_time'] = 1004.9
        self.assertFalse(self.analyze([pub], [cycle], [analyzed], [consumed])[0]['exact_adopted'])
        analyzed['online_step_entry_ros_time'] = 1005.1
        consumed['sha256'] = 'c' * 64
        self.assertFalse(self.analyze([pub], [cycle], [analyzed], [consumed])[0]['exact_adopted'])


class PairTests(unittest.TestCase):
    def fixture(self):
        return [dict(seed=seed, fix=fix,
            settings=dict(seed=seed, seed_parameterization_fix=fix, execution_weight=0.),
            runtime_sha256={'same.py': 'hash'}, build='same', build_manifest_sha256='samehash',
            runtime_python_verified=True, scope_checks=dict(scope=True),
            shared_manifest=dict(native_safety_chain='retained'))
            for seed in (11, 12) for fix in (False, True)]

    def test_only_flag_and_no_hidden_configuration_drift(self):
        trials = self.fixture()
        self.assertTrue(all(all(checks.values()) for checks in pair_checks(trials).values()))
        for field, value in (('settings', dict(execution_weight=1.)),
                             ('runtime_sha256', {'same.py': 'other'}),
                             ('build_manifest_sha256', 'other'),
                             ('runtime_python_verified', False),
                             ('shared_manifest', dict(native_safety_chain='changed')),
                             ('scope_checks', dict(scope=False))):
            damaged = copy.deepcopy(trials); damaged[1][field] = value
            self.assertFalse(all(pair_checks(damaged)['11'].values()))

    def test_missing_duplicate_pair_rejected(self):
        for trials in (self.fixture()[:-1], self.fixture() + [self.fixture()[0]]):
            with self.assertRaises(ValueError):
                pair_checks(trials)


if __name__ == '__main__':
    unittest.main()
