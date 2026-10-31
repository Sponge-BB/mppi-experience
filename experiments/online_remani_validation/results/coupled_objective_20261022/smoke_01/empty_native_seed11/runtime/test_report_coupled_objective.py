"""Offline contracts only: no ROS master, controller or optimization is run."""
import gzip
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from report_coupled_objective import (read, lines, coupling_metrics, measure,
                                      measured_advance, aggregate, report, plot)
from run_coupled_objective import treatment, configure_shared_collision


class CouplingContracts(unittest.TestCase):
    def test_group_weights_and_engineering_are_not_old_factorial_groups(self):
        expected = {'native': (False, 0., 0.), 'engineering': (True, 0., 0.),
                    'execution': (True, 2., 0.), 'handover': (True, 0., 3.),
                    'coupled': (True, 2., 3.)}
        for group, values in expected.items():
            row = treatment(group, 2., 3.)
            self.assertEqual((row['engineering'], row['execution_weight'], row['handover_weight']), values)
        with self.assertRaises(ValueError):
            treatment('A')
        with self.assertRaises(ValueError):
            treatment('coupled', float('nan'), 1.)

    def launch_fixture(self, root):
        (root / 'experiment.launch').write_text('''<launch>
          <node name="base_mppi"><param name="shared_collision" value="true"/></node>
          <node name="shared_collision_model"/>
          <node name="remani_planner_node"/>
        </launch>''')
        (root / 'manifest.json').write_text(json.dumps(dict(shared_collision=True,
            nodes=[dict(name='base_mppi'), dict(name='shared_collision_model'), dict(name='remani_planner_node')])))

    def test_native_disables_shared_node_and_kernel_flag(self):
        with tempfile.TemporaryDirectory(prefix='coupling_contract_') as directory:
            root = Path(directory)
            self.launch_fixture(root)
            configure_shared_collision(root, False)
            tree = ET.parse(root / 'experiment.launch')
            nodes = tree.getroot().findall('node')
            self.assertNotIn('shared_collision_model', [n.get('name') for n in nodes])
            self.assertIn('remani_planner_node', [n.get('name') for n in nodes])
            base = next(n for n in nodes if n.get('name') == 'base_mppi')
            self.assertEqual(base.find('param').get('value'), 'false')
            manifest = read(root / 'manifest.json')
            self.assertIs(manifest['shared_collision'], False)
            self.assertNotIn('shared_collision_model', [n['name'] for n in manifest['nodes']])

    def test_engineering_retains_shared_node_and_flag(self):
        with tempfile.TemporaryDirectory(prefix='coupling_contract_') as directory:
            root = Path(directory)
            self.launch_fixture(root)
            configure_shared_collision(root, True)
            nodes = ET.parse(root / 'experiment.launch').getroot().findall('node')
            self.assertIn('shared_collision_model', [n.get('name') for n in nodes])
            base = next(n for n in nodes if n.get('name') == 'base_mppi')
            self.assertEqual(base.find('param').get('value'), 'true')
            self.assertIs(read(root / 'manifest.json')['shared_collision'], True)

    def test_gzip_is_read_without_temporary_unpack(self):
        with tempfile.TemporaryDirectory(prefix='coupling_contract_') as directory:
            root = Path(directory)
            with gzip.open(root / 'data.json.gz', 'wt') as stream:
                json.dump({'a': 1}, stream)
            with gzip.open(root / 'rows.jsonl.gz', 'wt') as stream:
                stream.write('{"a":1}\n{"a":2}\n')
            self.assertEqual(read(root / 'data.json'), {'a': 1})
            self.assertEqual(lines(root / 'rows.jsonl'), [{'a': 1}, {'a': 2}])
            self.assertFalse((root / 'data.json').exists())
            self.assertIsNone(read(root / 'absent.json'))

    def cost_fixture(self):
        requests = [dict(request_id=2, optimizer_calls=[dict(resource_optimizer_id=7)])]
        resource = [dict(event='begin', stage='optimizer', id=7, mono=10.),
                    dict(event='end', stage='optimizer', id=7, mono=11.)]
        native = [dict(event='coupling_scope', request_id=2, scope_id=3, valid=True,
                       reason='usable', mono=10.1),
                  dict(event='coupling_cost', request_id=2, scope_id=3, phase='initial',
                       execution_cost=2., delivery_cost=1., mono=10.2),
                  dict(event='coupling_cost', request_id=2, scope_id=3, phase='final',
                       execution_cost=.5, delivery_cost=.2, mono=10.9, active=True,
                       callback_count=10, active_callback_count=9)]
        return native, resource, requests

    def test_initial_and_final_cost_events_count_one_optimizer_call(self):
        native, resource, requests = self.cost_fixture()
        row = coupling_metrics(native, resource, requests)
        self.assertEqual(row['refresh_optimizer_calls'], 1)
        self.assertEqual(row['cost_recorded_optimizer_calls'], 1)
        self.assertEqual(row['any_nonzero_optimizer_calls'], 1)
        self.assertEqual(row['execution_nonzero_coverage'], 1.)
        self.assertEqual(row['handover_nonzero_coverage'], 1.)
        self.assertEqual(row['active_objective_callback_rate'], .9)

    def test_enabled_or_missing_fields_are_not_nonzero_cost_evidence(self):
        native, resource, requests = self.cost_fixture()
        native = [dict(event='coupling_scope', request_id=2, scope_id=3, exec_weight=1.,
                       hand_weight=1., reason='not_recorded', mono=10.1)]
        row = coupling_metrics(native, resource, requests)
        self.assertEqual(row['unknown_snapshot_scopes'], 1)
        self.assertEqual(row['cost_recording_coverage'], 0.)
        self.assertEqual(row['any_nonzero_coverage'], 0.)
        empty = coupling_metrics([], [], [])
        self.assertIsNone(empty['any_nonzero_coverage'])
        self.assertIsNone(empty['valid_snapshot_scope_rate'])

    def test_unmatched_cost_does_not_claim_coverage(self):
        native, resource, requests = self.cost_fixture()
        native = [dict(event='coupling_cost', request_id=2, phase='final', scope_id=999,
                       execution_cost=1., delivery_cost=2., mono=50.)]
        row = coupling_metrics(native, resource, requests)
        self.assertEqual(row['unmatched_cost_events'], 1)
        self.assertEqual(row['any_nonzero_coverage'], 0.)

    def test_completed_solver_in_unfinished_request_is_retained(self):
        native, resource, _ = self.cost_fixture()
        native.insert(0, dict(event='request_begin', request_id=2, mono=9.))
        row = coupling_metrics(native, resource, [])
        self.assertEqual(row['refresh_optimizer_calls'], 1)
        self.assertEqual(row['any_nonzero_coverage'], 1.)

    def trial_fixture(self, root, outcome='timeout', delivered=False):
        settings = dict(group='coupled', seed=1, engineering=True, execution_weight=1.,
                        handover_weight=1., legacy_rolling=False, legacy_prefix=False, diagnostic_pause=False)
        (root / 'manifest.json').write_text(json.dumps(dict(scene='static', coupling_experiment=settings)))
        (root / 'run_status.json').write_text('{}')
        (root / 'trial_audit.json').write_text('{"valid_experiment":true}')
        (root / 'result.json').write_text(json.dumps(dict(reason=outcome,
            collision_conservative_projection=outcome == 'collision', final_state=[1., 0., 0.], elapsed=3.)))
        (root / 'state.csv').write_text('t,x,y,yaw\n0,0,0,0\n1,0,0,0\n2,.1,0,0\n3,.2,0,0\n')
        (root / 'analysis').mkdir()
        chains = [dict(publish_sim_s=1.1)]
        if delivered:
            chains.append(dict(publish_sim_s=1.2, consume_sim_s=1.5))
        summary = dict(requests=[], end_sim_s=3., anchor_sim_s=0.,
            latency=dict(first_native_blocked_sim_s=1.), reference_chains=chains)
        (root / 'analysis/summary.json').write_text(json.dumps(summary))

    def test_timeout_and_no_delivery_are_valid_retained_outcomes(self):
        with tempfile.TemporaryDirectory(prefix='coupling_contract_') as directory:
            root = Path(directory)
            self.trial_fixture(root)
            row = measure(root)
            self.assertTrue(row['valid_experiment'])
            self.assertEqual(row['outcome'], 'timeout')
            self.assertEqual(row['delivery_status'], 'no_delivery')
            self.assertIsNone(row['blocked_to_consume_s'])
            self.assertEqual(row['blocked_to_end_lower_bound_s'], 2.)
            self.assertEqual(aggregate([row])['coupled']['navigation_timeouts'], 1)
            self.assertEqual(aggregate([row])['coupled']['no_delivery_trials'], 1)

    def test_later_consumed_reference_counts_if_first_publication_is_not_used(self):
        with tempfile.TemporaryDirectory(prefix='coupling_contract_') as directory:
            root = Path(directory)
            self.trial_fixture(root, delivered=True)
            row = measure(root)
            self.assertEqual(row['delivery_status'], 'delivered')
            self.assertEqual(row['blocked_to_consume_s'], .5)

    def test_scope_solver_and_callback_metrics_are_separate_real_columns(self):
        with tempfile.TemporaryDirectory(prefix='coupling_contract_') as directory:
            root = Path(directory)
            self.trial_fixture(root)
            native, resource, requests = self.cost_fixture()
            summary = read(root / 'analysis/summary.json')
            summary['requests'] = requests
            (root / 'analysis/summary.json').write_text(json.dumps(summary))
            for name, records in [('dual_timeline.jsonl', native), ('solver_trace.jsonl', resource)]:
                (root / name).write_text(''.join(json.dumps(row) + '\n' for row in records))
            row = measure(root)
            self.assertEqual(row['valid_snapshot_scope_rate'], 1.)
            self.assertEqual(row['any_nonzero_optimizer_coverage'], 1.)
            self.assertEqual(row['objective_callback_count'], 10)
            self.assertEqual(row['active_objective_callback_count'], 9)
            self.assertEqual(row['active_objective_callback_rate'], .9)
            total = aggregate([row])['coupled']
            self.assertEqual(total['valid_snapshot_scopes'], 1)
            self.assertEqual(total['any_nonzero_optimizer_calls'], 1)
            self.assertEqual(total['objective_callback_count'], 10)
            self.assertEqual(total['active_objective_callback_count'], 9)

    def test_old_factorial_manifest_is_rejected(self):
        with tempfile.TemporaryDirectory(prefix='coupling_contract_') as directory:
            root = Path(directory)
            (root / 'manifest.json').write_text('{"factorial":{"group":"A"}}')
            with self.assertRaisesRegex(ValueError, 'old A-D'):
                measure(root)

    def test_displacement_is_not_heading_change_or_future_sample(self):
        state = np.array([(0., 0., 0.), (1., .01, 0.), (2., .07, 0.)],
                         dtype=[('t', float), ('x', float), ('y', float)])
        self.assertIsNone(measured_advance(state, None, 2.))
        self.assertIsNone(measured_advance(state, 1., 1.5))
        self.assertEqual(measured_advance(state, 1., 2.), 1.)

    def test_report_retains_collision_and_writes_no_missing_as_zero(self):
        with tempfile.TemporaryDirectory(prefix='coupling_contract_') as directory:
            root = Path(directory)
            batch = root / 'batch'
            batch.mkdir()
            (batch / 'experiment_plan.json').write_text('{"paired":false}')
            (batch / 'batch_complete.json').write_text('{"complete":true}')
            trial = batch / 'trial'
            trial.mkdir()
            self.trial_fixture(trial, outcome='collision')
            data = report([batch], root / 'report', plots=False)
            self.assertEqual(data['groups']['coupled']['collisions'], 1)
            self.assertIsNone(data['groups']['coupled']['blocked_to_consume_s_p50'])
            self.assertTrue((root / 'report/逐次结果.csv').exists())

    def test_small_plot_works_with_host_matplotlib(self):
        with tempfile.TemporaryDirectory(prefix='coupling_contract_') as directory:
            root = Path(directory)
            self.trial_fixture(root)
            rows = [measure(root)]
            plot(rows, aggregate(rows), root)
            self.assertGreater((root / '对比结果.png').stat().st_size, 1000)


if __name__ == '__main__':
    unittest.main()
