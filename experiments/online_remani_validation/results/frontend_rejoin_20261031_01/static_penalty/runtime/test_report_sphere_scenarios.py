"""Timing/report accounting tests; no ROS, controller replay, or large fixture."""
import unittest

from report_sphere_scenarios import (comparisons, cpu_metrics, cycle_metrics,
                                     loop_metrics, request_metrics, stats,
                                     trial_entries)


class SphereScenarioReportTests(unittest.TestCase):
    def test_statistics_do_not_turn_missing_values_into_zero(self):
        result = stats([None, 10., 30., float('nan')])
        self.assertEqual(result['n'], 2)
        self.assertEqual(result['p50'], 20.)
        self.assertIsNone(stats([])['p95'])

    def test_failed_and_unfinished_requests_are_explicit(self):
        requests = [
            dict(request_id=1, start_sim_s=1., end_sim_s=2., success=True,
                 request_wall_ms=100., stage_wall_ms=dict(search=20.),
                 optimizer_calls=[dict(wall_ms=80., reason='ok')]),
            dict(request_id=2, start_sim_s=3., end_sim_s=4., success=False,
                 request_wall_ms=600., stage_wall_ms=dict(search=50.),
                 optimizer_calls=[dict(wall_ms=550., reason='budget_timeout')]),
            dict(request_id=3, start_sim_s=5., end_sim_s=6., success=False,
                 request_wall_ms=700., stage_wall_ms=dict(search=40.),
                 optimizer_calls=[dict(wall_ms=660., reason='wheel_limit')]),
            dict(request_id=4, start_sim_s=7., end_sim_s=9., success=True,
                 request_wall_ms=200.),
            dict(request_id=5, start_sim_s=20., request_wall_ms=99.)]
        summary = dict(requests=requests, latency=dict(first_native_blocked_sim_s=3.))
        result = request_metrics(summary, 8.)
        self.assertEqual((result['started'], result['completed'], result['pending']), (4, 3, 1))
        self.assertEqual((result['accepted'], result['failed']), (1, 2))
        self.assertEqual(result['completed_request_wall_ms']['p50'], 600.)
        self.assertEqual(result['backend_wall_ms']['n'], 3)
        self.assertEqual(result['backend_outcomes']['budget_timeout'], 1)
        self.assertEqual(result['backend_outcomes']['wheel_limit'], 1)
        self.assertIsNone(result['blocked_to_first_consumed_refresh_s'])
        self.assertEqual(result['observed_refresh_wait_lower_bound_s'], 5.)
        self.assertEqual(result['incomplete_request_ids'], [4])
        self.assertEqual(result['request_groups']['initial']['n'], 1)
        self.assertEqual(result['request_groups']['refresh_failure']['n'], 2)
        self.assertEqual(result['request_groups']['refresh_success']['n'], 0)
        self.assertEqual(result['completed_refresh_wall_ms']['p50'], 650.)
        self.assertEqual(result['completed_request_cumulative_ms'], 1400.)

    def test_reference_wait_is_not_successful_solver_duration(self):
        result = request_metrics(dict(requests=[], latency=dict(
            first_native_blocked_sim_s=2., first_refresh_consume_sim_s=7.)), 10.)
        self.assertEqual(result['blocked_to_first_consumed_refresh_s'], 5.)
        self.assertFalse(result['waiting_for_refresh_at_end'])

    def test_unclosed_requests_from_actual_analyzer_are_counted(self):
        summary = dict(requests=[], clock_origin_ros_s=1000., unmatched=dict(requests=[
            dict(reason='unclosed_begin', event=dict(request_id=4, sim=1007., mono=10.)),
            dict(reason='end_without_begin', event=dict(request_id=2, sim=1002., mono=2.))]))
        result = request_metrics(summary, 8.)
        self.assertEqual(result['started'], 1)
        self.assertEqual(result['pending'], 1)
        self.assertEqual(result['incomplete_request_ids'], [4])
        self.assertIsNone(result['completed_request_wall_ms']['p50'])

    def test_invoked_failures_and_real_update_gaps_remain_in_statistics(self):
        def pair(ident, published, reason, kernel, success, validation=0):
            return (dict(kernel_ms=kernel, callback_ms=kernel+10., validated_proposal=[.1, 0.]),
                    dict(cycle_id=ident, online_step_invoked=True, online_step_succeeded=success,
                         command_publish_after_monotonic=published, reason=reason,
                         validation_code=validation), published)
        pairs = [pair(1, 1., 'ok', 20., True),
                 pair(2, 1.3, 'deadline_miss', 220., True),
                 pair(3, 1.4, 'no_feasible_candidate', 100., False, 3)]
        result = cycle_metrics(pairs)
        self.assertEqual(result['kernel_ms']['n'], 3)
        self.assertEqual(result['kernel_ms']['p50'], 100.)
        self.assertEqual(result['kernel_failures'], 1)
        self.assertEqual(result['callback_over_deadline'], 1)
        self.assertEqual(result['deadline_labels'], 1)
        self.assertEqual(result['validated_nonzero_discarded_late'], 1)
        self.assertEqual(result['publication_intervals_over_lease'], 1)
        self.assertAlmostEqual(result['publication_interval_ms']['p50'], 200.)

    def test_cpu_counts_process_percent_not_divided_by_cores(self):
        samples = [dict(mono=t, processes=[dict(name='base_mppi', pid=7, cpu_pct=120.),
                                          dict(name='remani_planner_node', pid=8, cpu_pct=240.)],
                        cores=[dict(core='cpu', busy_pct=70.)]) for t in (1., 2., 3.)]
        result = cpu_metrics(samples, 1.5, 2.5, {})
        self.assertEqual(result['base_mppi']['n'], 1)
        self.assertEqual(result['base_mppi']['mean'], 120.)
        self.assertEqual(result['remani_planner_node']['p95'], 240.)
        self.assertEqual(result['system_busy_pct']['p50'], 70.)

    def test_validation_per_cycle_sums_every_retry_and_flags_missing_timer(self):
        def pair(ident, attempts):
            return (dict(kernel_ms=10., callback_ms=30.),
                    dict(cycle_id=ident, online_step_invoked=True, validation_invoked=True,
                         command_publish_after_monotonic=float(ident), reason='ok', validation_code=0,
                         validation_attempts=attempts), float(ident))
        result = cycle_metrics([
            pair(1, [dict(wall_ms=2., thread_cpu_ms=1.)]),
            pair(2, [dict(wall_ms=3., thread_cpu_ms=2.), dict(wall_ms=7., thread_cpu_ms=4.)]),
            pair(3, [dict(wall_ms=3.), dict(code=0)])])['validation']
        self.assertEqual(result['attempts'], 5)
        self.assertEqual(result['cycle_wall_ms']['n'], 2)
        self.assertEqual(result['cycle_wall_ms']['p50'], 6.)
        self.assertEqual(result['cycle_main_thread_cpu_ms']['p50'], 3.5)
        self.assertEqual(result['retry_cycles'], 2)
        self.assertEqual(result['extra_retry_attempts'], 2)
        self.assertEqual(result['missing_or_incomplete_timer_cycles'], 1)

    def test_active_loop_excludes_sleep_and_preserves_cpu(self):
        record = dict(columns=['cycle', 'stage', 'mono', 'thread_cpu_s'], rows=[
            [1, 'cycle_start', 10., 1.], [1, 'sleep_enter', 10.3, 1.1],
            [1, 'sleep_exit', 10.5, 1.1], [2, 'cycle_start', 10.5, 1.1]])
        result = loop_metrics(record, {1, 2})
        self.assertAlmostEqual(result['active_loop_wall_ms']['mean'], 300.)
        self.assertAlmostEqual(result['active_loop_main_thread_cpu_ms']['mean'], 100.)
        self.assertEqual(result['active_loop_wall_ms']['n'], 1)

    def test_plan_never_scans_history_and_rejects_duplicates(self):
        good = dict(scene='empty', group='legacy', directory='empty_legacy_seed11')
        self.assertEqual(list(trial_entries(dict(trials=[good])))[0]['directory'], good['directory'])
        with self.assertRaises(ValueError):
            list(trial_entries(dict(trials=[good, good])))
        with self.assertRaises(ValueError):
            list(trial_entries(dict(trials=[dict(good, directory='../history')])))

    def test_invalid_group_is_not_presented_as_valid_comparison(self):
        def row(group, valid, value):
            return dict(scene_key='empty', group=group, valid_experiment=valid,
                mppi=dict(kernel_ms=dict(p50=value, p95=value), callback_ms=dict(p50=value, p95=value),
                          publication_interval_ms=dict(p95=value), solver_phase=dict(intervention=dict(
                              kernel_ms=dict(p95=value), callback_ms=dict(p95=value)))),
                remani=dict(completed_request_wall_ms=dict(p95=value), frontend_wall_ms=dict(p95=value),
                            backend_wall_ms=dict(p95=value)))
        comparison = comparisons([row('legacy', True, 100.), row('b47', False, 50.)])[0]
        self.assertFalse(comparison['both_valid'])
        self.assertEqual(comparison['metrics']['callback_p95']['decrease_pct'], 50.)


if __name__ == '__main__':
    unittest.main()
