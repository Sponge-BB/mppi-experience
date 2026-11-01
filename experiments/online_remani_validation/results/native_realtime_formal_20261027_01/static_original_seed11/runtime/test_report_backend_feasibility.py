"""Synthetic report tests only; no ROS, compilation, geometry or archives."""
import copy
import unittest

from report_backend_feasibility import (LABELS, BUDGETS, candidate_key,
    combine_phase, repeat_check, scene_phase, stage_counts)


def fixture(label=LABELS[0], budget=.6):
    summary = dict(input_hash='123', reason='budget_timeout', native_return_success=False,
        budget_s=budget)
    p = dict(label=label, budget_s=budget, summary=summary, complete_records=True,
        input_hash_equal=True, budget_agrees=True, verdict_agrees=True, solver_wall_ms=603.,
        native_failure_frame=dict(iterations=10))
    candidates = [dict(stage=stage, iteration=iteration, candidate_x_hash=str(100 + iteration),
        candidate_evidence_valid=True, solver_elapsed_s=iteration / 10.,
        native_IsTrajSafe_pass=iteration > 0, native_reason='ok' if iteration else 'left_wheel_speed',
        frozen_projected_pass=False, both_pass=False, frozen_projected_20ms=dict(min_static_gap_m=-.01))
        for stage, iteration in (('initial', 0), ('accepted', 1))]
    signature = dict(verified=True, x_hash='100', gradient_hash='200', cost=3.)
    it = dict(phase=label, solve_summary=summary, phase_evidence_valid=True,
        first_signature=signature, candidates=candidates, selected_candidate_checks={},
        first_both_pass=None, first_native_pass=dict(stage='accepted', iteration=1), native_audit_cost={})
    kin = dict(phase=label, input_hash='123', phase_evidence_valid=True,
        first_signature=signature, candidates=[dict(c, evidence_valid=True,
            native_10ms=dict(samples=1, all_wheels_sampled_pass=c['iteration'] > 0),
            diagnostic_10ms_with_boundaries=dict(samples=10, all_wheels_sampled_pass=False,
                peaks=dict(left_speed=dict(ratio_to_final_limit=1.1))),
            initial_k0_values=dict(velocity=[.05, 0.]) if c['iteration'] == 0 else None) for c in candidates])
    return p, it, kin


class CandidateTests(unittest.TestCase):
    def test_initial_x0_and_accepted_not_confused(self):
        self.assertEqual(candidate_key(dict(stage='initial', iteration=0, candidate_x_hash='123')), ('initial', 0, '123'))
        for row in (dict(stage='initial', iteration=1, candidate_x_hash='123'),
                    dict(stage='accepted', iteration=0, candidate_x_hash='123'),
                    dict(stage='accepted', iteration=True, candidate_x_hash='123'),
                    dict(stage='accepted', iteration=1, candidate_x_hash='bad')):
            self.assertIsNone(candidate_key(row))

    def test_join_separates_native_projected_and_wheel_checks(self):
        result = combine_phase(*fixture())
        self.assertTrue(result['evidence_valid'])
        self.assertEqual(result['counts']['initial']['count'], 1)
        self.assertEqual(result['counts']['accepted']['native_IsTrajSafe_pass'], 1)
        self.assertEqual(result['counts']['accepted']['both_pass'], 0)
        self.assertEqual(result['counts']['accepted']['native_grid_wheel_pass'], 1)
        self.assertEqual(result['counts']['accepted']['diagnostic_grid_wheel_pass'], 0)
        self.assertTrue(result['not_online_delivery'])

    def test_native_success_not_full_feasibility(self):
        p, it, kin = fixture(); p['summary']['native_return_success'] = True
        p['summary']['reason'] = 'ok'; p['solver_wall_ms'] = 430.
        result = combine_phase(p, it, kin)
        self.assertTrue(result['evidence_valid']); self.assertTrue(result['native_return_success'])
        self.assertIsNone(result['first_both_pass'])

    def test_unknown_audit_cannot_become_zero_pass(self):
        p, it, kin = fixture()
        for damaged_it, damaged_kin in ((None, kin), (it, None), ({}, {})):
            result = combine_phase(p, damaged_it, damaged_kin)
            self.assertFalse(result['evidence_valid'])
            self.assertIsNone(result['counts']['accepted'])

    def test_duplicate_missing_or_mutated_candidate_identity_rejected(self):
        for mutation in ('duplicate', 'missing', 'hash', 'stage', 'candidate_invalid'):
            p, it, kin = fixture()
            if mutation == 'duplicate':
                kin['candidates'].append(copy.deepcopy(kin['candidates'][0]))
            elif mutation == 'missing':
                kin['candidates'].pop()
            elif mutation == 'hash':
                kin['candidates'][1]['candidate_x_hash'] = '999'
            elif mutation == 'stage':
                kin['candidates'][1]['stage'] = 'initial'
            else:
                kin['candidates'][1]['evidence_valid'] = False
            self.assertFalse(combine_phase(p, it, kin)['evidence_valid'])

    def test_scope_summary_signature_and_status_disagreement_rejected(self):
        p, it, kin = fixture()
        for where, key, value in (('p', 'budget_agrees', False), ('it', 'phase_evidence_valid', False),
                                  ('kin', 'input_hash', '999'), ('kin', 'first_signature', dict(verified=True)),
                                  ('it', 'solve_summary', dict(reason='ok'))):
            copies = dict(p=copy.deepcopy(p), it=copy.deepcopy(it), kin=copy.deepcopy(kin))
            copies[where][key] = value
            self.assertFalse(combine_phase(copies['p'], copies['it'], copies['kin'])['evidence_valid'])
        kin['candidates'][1]['both_pass'] = True
        self.assertFalse(combine_phase(p, it, kin)['evidence_valid'])

    def test_truncation_or_incomplete_parent_not_success(self):
        p, it, kin = fixture(); it['phase_evidence_valid'] = False
        result = combine_phase(p, it, kin)
        self.assertIsNone(result['counts']['accepted']); self.assertFalse(result['evidence_valid'])


class RepeatTests(unittest.TestCase):
    def test_repeated_budget_non_determinism_visible(self):
        a = combine_phase(*fixture())
        p, it, kin = fixture(LABELS[1]); p['solver_wall_ms'] = 610.
        p['summary']['reason'] = 'right_wheel_speed'
        b = combine_phase(p, it, kin)
        result = repeat_check([a, b])
        self.assertTrue(result['valid_same_input_repeated_budget'])
        self.assertFalse(result['same_solver_reason'])
        self.assertEqual(result['lbfgs_wall_ms_difference_repeat_minus_nominal'], 7.)
        self.assertTrue(result['not_independent_repetitions'])

    def test_missing_invalid_duplicate_phase_unknown(self):
        a = combine_phase(*fixture())
        for phases in ([], [a], [a, a]):
            result = repeat_check(phases)
            self.assertFalse(result['valid_same_input_repeated_budget'])
            self.assertIsNone(result['same_solver_reason'])
            self.assertIsNone(result['accepted_count_difference'])


class SceneTests(unittest.TestCase):
    def test_ros_and_elapsed_times_not_mixed(self):
        result = scene_phase(dict(sim=1013.5, frame=dict(now=1012.8)),
            dict(motion_anchor=9.), [dict(kind='motion_anchor', t=9.)])
        self.assertTrue(result['capture_at_or_after_appearance'])
        self.assertEqual(result['capture_elapsed_s'], 13.5)
        self.assertEqual(result['capture_minus_appearance_s'], .5)
        self.assertFalse(result['frozen_frame_at_or_after_appearance'])

    def test_scene_name_alone_or_missing_conflicting_event_unknown(self):
        for outcome, events in ((dict(scene='static'), []), (dict(motion_anchor=9.), []),
                                (dict(motion_anchor=9.), [dict(kind='motion_anchor', t=8.)]),
                                (dict(motion_anchor=9.), [dict(kind='motion_anchor', t=9.)] * 2)):
            result = scene_phase(dict(sim=1013.5), outcome, events)
            self.assertIsNone(result['capture_at_or_after_appearance'])
            self.assertIsNone(result['scene_appearance_ros_time'])

    def test_preappearance_kept_as_preappearance(self):
        result = scene_phase(dict(sim=1012.), dict(motion_anchor=9.), [dict(kind='motion_anchor', t=9.)])
        self.assertFalse(result['capture_at_or_after_appearance'])
        self.assertEqual(result['capture_minus_appearance_s'], -1.)


if __name__ == '__main__':
    unittest.main()
