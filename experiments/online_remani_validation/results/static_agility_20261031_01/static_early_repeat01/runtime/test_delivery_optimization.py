#!/usr/bin/env python3
import unittest
import xml.etree.ElementTree as ET

from report_delivery_optimization import delivery_metrics
from run_delivery_optimization import GROUPS, SCENES, jobs, set_planner_switches


class DeliveryOptimizationTest(unittest.TestCase):
    def test_prespecified_ab_ba_jobs(self):
        rows = jobs()
        self.assertEqual([(r['scene'], r['group']) for r in rows],
            [('static', 'baseline'), ('static', 'optimized'),
             ('midcross', 'optimized'), ('midcross', 'baseline')])
        self.assertEqual({r['seed'] for r in rows}, {11})
        self.assertEqual(len({r['directory'] for r in rows}), 4)

    def tree(self, replay=''):
        return ET.ElementTree(ET.fromstring('<launch><node name="remani_planner_node">'
            '<env name="DELIVERY_REPLAY_KIND" value="'+replay+'"/>'
            '<env name="DELIVERY_FAST_QUERY" value="old"/>'
            '</node><node name="base_mppi"/></launch>'))

    def test_switches_explicit_and_planner_only(self):
        tree = self.tree()
        for enabled in (False, True):
            set_planner_switches(tree, enabled)
            planner = tree.getroot().find("node[@name='remani_planner_node']")
            for key in ('DELIVERY_FAST_QUERY', 'DELIVERY_ACCEPT_CANDIDATE'):
                self.assertEqual(len(planner.findall("env[@name='%s']" % key)), 1)
                self.assertEqual(planner.find("env[@name='%s']" % key).get('value'), str(int(enabled)))
            self.assertEqual(tree.getroot().find("node[@name='base_mppi']").findall('env'), [])

    def test_replay_not_allowed_online(self):
        with self.assertRaises(ValueError):
            set_planner_switches(self.tree('first_refresh'), True)

    def fixture(self):
        return dict(clock_origin_ros_s=1000.,
            requests=[dict(request_id=2, end_sim_s=5., success=True,
                           optimizer_calls=[dict(resource_optimizer_id=8)])],
            reference_chains=[dict(request_id=2, publish_sim_s=5.01, consume_sim_s=5.02)],
            latency=dict(first_native_blocked_sim_s=3., first_refresh_trigger_sim_s=3.2,
                         first_refresh_publish_sim_s=5.01, first_refresh_consume_sim_s=5.02))

    def test_salvage_native_pass_separate_from_delivery(self):
        analysis = self.fixture()
        resource = dict(event='end', stage='optimizer', id=8, sim=1004.8,
                        salvage_attempt=1, salvage_native_result=1, success=True)
        result = delivery_metrics(analysis, [resource], 6.)
        self.assertEqual(result['salvage_native_pass'], 1)
        self.assertEqual(result['salvage_published'], 1)
        self.assertEqual(result['salvage_consumed'], 1)
        self.assertAlmostEqual(result['first_blocked_to_consume_s'], 2.02)
        # Nothing was delivered by this cut-off, though backend passed.
        result = delivery_metrics(analysis, [resource], 5.)
        self.assertEqual(result['salvage_native_pass'], 1)
        self.assertEqual(result['salvage_published'], 0)
        self.assertIsNone(result['first_blocked_to_consume_s'])

    def test_missing_and_failed_salvage_is_not_success(self):
        analysis = self.fixture()
        # The same request published something, but not this failed candidate.
        resources = [dict(event='end', stage='optimizer', id=8, sim=1004.8,
                          salvage_attempt=1, salvage_native_result=0, success=False),
                     dict(event='end', stage='optimizer', id=9, sim=1004.9, success=True)]
        result = delivery_metrics(analysis, resources, 6.)
        self.assertEqual(result['salvage_attempts'], 1)
        self.assertEqual(result['salvage_native_pass'], 0)
        self.assertEqual(result['salvage_request_success'], 0)
        self.assertEqual(result['salvage_published'], 0)
        self.assertEqual(result['salvage_consumed'], 0)

    def test_timeout_is_not_final_failure(self):
        rows = [dict(event='end', stage='optimizer', id=8, sim=1004.8,
                     force_stop=3, success=True, salvage_attempt=1,
                     salvage_native_result=1, native_validation_ms=12., lbfgs_wall_ms=605.),
                dict(event='end', stage='optimizer', id=9, sim=1004.9,
                     force_stop=3, success=False, lbfgs_wall_ms=603.),
                dict(event='end', stage='optimizer', id=10, sim=1004.9,
                     force_stop=0, success=True, native_validation_ms=10., lbfgs_wall_ms=400.)]
        result = delivery_metrics(self.fixture(), rows, 6.)
        self.assertEqual(result['optimizer_timeout_events'], 2)
        self.assertEqual(result['optimizer_timeout_final_backend_accepted'], 1)
        self.assertEqual(result['optimizer_timeout_final_backend_failed'], 1)
        self.assertEqual(result['native_validation_ms']['n'], 2)
        self.assertEqual(result['native_validation_ms']['p50'], 11.)
        self.assertEqual(result['salvage_consumed'], 1)

    def test_multiple_calls_same_request_not_double_counted(self):
        analysis = self.fixture()
        analysis['requests'][0]['optimizer_calls'].append(dict(resource_optimizer_id=9))
        rows = [dict(event='end', stage='optimizer', id=8, sim=1004.7,
                     success=False, salvage_attempt=1, salvage_native_result=0),
                dict(event='end', stage='optimizer', id=9, sim=1004.9,
                     success=True, salvage_attempt=1, salvage_native_result=1)]
        result = delivery_metrics(analysis, rows, 6.)
        self.assertEqual(result['salvage_attempts'], 2)
        self.assertEqual(result['salvage_native_pass'], 1)
        self.assertEqual(result['salvage_published'], 1)
        self.assertEqual(result['salvage_consumed'], 1)
        self.assertFalse(result['salvages'][0]['published'])


if __name__ == '__main__':
    unittest.main()
