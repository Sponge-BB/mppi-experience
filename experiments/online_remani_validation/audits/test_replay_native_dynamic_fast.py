"""Small replay-contract tests; no ROS, shared library, or output artifacts."""
import copy
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import replay_native_dynamic_fast as replay


def row(ident, mode=0, moving=False, validation=0, reason='ok'):
    return (ident-1, {'cycle_id': ident}, dict(local_phase={'decision': [mode]},
        obstacle_tracks=int(moving), validation_code=validation, reason=reason))


def record():
    return dict(code=0, action=[.2, .1, .2, .1, 0., 0., 0.], controls=[[.2, .1]],
        diagnostics={'online_debug': [0.]}, validation_recorded=True,
        validation_code=0, validation_report=[1., .1],
        avoidance_debug=[0.]*8, input_sha256={'state': 'actual-hash'}, search_candidates=[
            dict(id=0, feasible=False, selected=False, rows=[[0., 0., 0., 0., 15., -.1, 0., 0.]]),
            dict(id=1, feasible=True, selected=True, rows=[[1., 0., 0., 0., 10., .1, 1., 1.]])])


class ReplayContractTests(unittest.TestCase):
    def test_semantic_selection_covers_intervention_and_infeasibility(self):
        selected = replay.choose_cycles([row(1), row(2, moving=True), row(3, 2, True),
            row(4, 2, True), row(5, 4, True, -1, 'no_feasible_candidate'),
            row(6, 4, True, -1, 'no_feasible_candidate')])
        self.assertEqual([r['cycle_id'] for r in selected], [1, 2, 3, 4, 5, 6])
        self.assertIn('first_intervention', selected[2]['labels'])
        self.assertIn('last_no_feasible_candidate', selected[-1]['labels'])

    def test_duplicate_selection_retains_labels(self):
        selected = replay.choose_cycles([row(1, 4, True, -1, 'no_feasible_candidate')])
        self.assertEqual(len(selected), 1)
        self.assertEqual(len(selected[0]['labels']), 4)

    def test_static_includes_late_rejoin(self):
        selected = replay.choose_cycles([row(1), row(2, 2), row(3, 3), row(4, 3)])
        self.assertEqual(selected[-1]['cycle_id'], 4)
        self.assertIn('late_rejoin_or_cusp', selected[-1]['labels'])

    def test_only_discarded_cost_can_differ(self):
        before = record()
        after = copy.deepcopy(before)
        after['search_candidates'][0]['rows'][0][4] = 11.
        self.assertFalse(replay.compare_records(before, after)['passed'])
        self.assertTrue(replay.compare_records(before, after, allow_pruned_cost=True)['passed'])
        after['search_candidates'][1]['rows'][0][4] = 9.
        self.assertFalse(replay.compare_records(before, after, allow_pruned_cost=True)['passed'])

    def test_actions_controls_validation_and_classification_never_ignored(self):
        for mutate in [lambda r: r['action'].__setitem__(0, .21),
                       lambda r: r['controls'][0].__setitem__(1, .11),
                       lambda r: r.__setitem__('validation_code', 3),
                       lambda r: r['search_candidates'][0].__setitem__('feasible', True),
                       lambda r: r['input_sha256'].__setitem__('state', 'changed')]:
            before = record()
            after = copy.deepcopy(before)
            mutate(after)
            self.assertFalse(replay.compare_records(before, after, allow_pruned_cost=True)['passed'])

    def test_discarded_gap_not_ignored(self):
        before = record()
        after = copy.deepcopy(before)
        after['search_candidates'][0]['rows'][0][5] = -.2
        self.assertFalse(replay.compare_records(before, after, allow_pruned_cost=True)['passed'])

    def test_schedule_always_runs_all_and_varies_start(self):
        orders = [replay.trial_order(i) for i in range(3)]
        for order in orders:
            self.assertEqual(sorted(order), sorted(replay.VARIANTS))
        self.assertGreater(len(set(tuple(order) for order in orders)), 1)

    def test_state_identity_requires_actual_matching_odometry(self):
        cycle = dict(solve_state=[0., 0., 0., .1, 0.],
            solve_odom_identity={'state': [0., 0., 0., .1, 0.]})
        np.testing.assert_array_equal(replay.check_state_identity(cycle, 'solve'), cycle['solve_state'])
        cycle['solve_odom_identity']['state'][3] = 0.
        with self.assertRaisesRegex(ValueError, 'odometry identity'):
            replay.check_state_identity(cycle, 'solve')

    def test_trace_summary_requires_constant_classification(self):
        values = np.array([[0., 0., 0., 0., 1., .1, 1., 0.],
                           [0., .1, .1, 0., 1., .1, 1., 0.]])
        self.assertTrue(replay.summarize_trace(values)[0]['feasible'])
        values[1, 6] = 0.
        with self.assertRaisesRegex(ValueError, 'malformed'):
            replay.summarize_trace(values)


if __name__ == '__main__':
    unittest.main()
