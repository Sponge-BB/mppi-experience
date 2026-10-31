"""Offline contracts only: no ROS master, controller or optimization is run."""
import gzip
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
from unittest.mock import patch

import numpy as np

from report_coupled_objective import (read, lines, coupling_metrics, measure,
                                      measured_advance, aggregate, report, plot)
from run_coupled_objective import (treatment, configure_shared_collision, ordered_jobs,
                                   job_name, resume_inventory, sha, RUNTIME_CONTRACT)


class CouplingContracts(unittest.TestCase):
    def resume_fixture(self, root, count=8):
        build, batch = root / 'build', root / 'batch'
        build.mkdir()
        batch.mkdir()
        binary = dict(native_binaries_sha256={}, mppi_library_sha256='frozen', production_sources_sha256={'x': 'y'})
        (build / 'build_manifest.json').write_text(json.dumps(binary))
        plan = dict(build=str(build), build_manifest_sha256=sha(build / 'build_manifest.json'),
                    production_sources_sha256={'x': 'y'}, groups=['native', 'engineering', 'execution', 'handover', 'coupled'],
                    scenes=['static', 'dynamic_head_on_center', 'dynamic_four'], seeds=[11], duration_s=48.,
                    execution_weight=1., handover_weight=1., search=False, paired=False, legacy_rolling=False, legacy_prefix=False)
        (batch / 'experiment_plan.json').write_text(json.dumps(plan))
        jobs = ordered_jobs(plan)
        completed = [dict(scene=s, group=g, seed=n, valid=True) for s, g, n in jobs[:count]]
        (batch / 'batch_stopped.json').write_text(json.dumps(dict(expected_trials=15, completed=completed, reason='disk reserve')))
        for job in jobs[:count]:
            trial = batch / job_name(job)
            trial.mkdir()
            runtime = trial / 'runtime'
            runtime.mkdir()
            digests = {}
            for name in RUNTIME_CONTRACT:
                (runtime / name).write_bytes(b'frozen runtime')
                digests[name] = sha(runtime / name)
            settings = treatment(job[1])
            settings.update(seed=job[2], legacy_rolling=False, legacy_prefix=False, diagnostic_pause=False)
            (trial / 'coupling_parameters.json').write_text(json.dumps(settings))
            manifest = dict(scene=job[0], duration=48., record_search=False, build=str(build),
                coupling_experiment=settings, build_manifest_sha256=plan['build_manifest_sha256'],
                native_binaries_sha256={}, mppi_library_sha256='frozen',
                parameters_sha256=sha(trial / 'coupling_parameters.json'), experiment_sources_sha256=digests)
            (trial / 'manifest.json').write_text(json.dumps(manifest))
            status = dict(returncode=0, result_written=True, graph_audited=True,
                          production_hashes_unchanged=True, build_manifest_unchanged=True)
            (trial / 'run_status.json').write_text(json.dumps(status))
            (trial / 'result.json').write_text('{"reason":"timeout"}')
            (trial / 'analysis').mkdir()
            (trial / 'analysis/summary.json').write_text('{}')
            audit = dict(valid_experiment=True, errors=[], inputs_sha256={name: sha(trial / name)
                for name in ('manifest.json', 'run_status.json', 'result.json')})
            (trial / 'trial_audit.json').write_text(json.dumps(audit))
        return batch, build, jobs

    def test_resume_preserves_original_plan_and_never_repeats_eight_completed_trials(self):
        with tempfile.TemporaryDirectory(prefix='coupling_resume_') as directory:
            batch, build, jobs = self.resume_fixture(Path(directory))
            before = {name: sha(batch / name) for name in ('experiment_plan.json', 'batch_stopped.json')}
            inventory = resume_inventory(batch, build)
            self.assertEqual(inventory['completed'], jobs[:8])
            self.assertEqual(inventory['pending'], jobs[8:])
            self.assertEqual(len(inventory['pending']), 7)
            self.assertEqual(before, {name: sha(batch / name) for name in before})

    def test_resume_rejects_plan_override_invalid_trial_and_missing_completed_group(self):
        for kind in ('override', 'invalid', 'missing'):
            with self.subTest(kind=kind), tempfile.TemporaryDirectory(prefix='coupling_resume_') as directory:
                batch, build, jobs = self.resume_fixture(Path(directory))
                if kind == 'override':
                    with self.assertRaisesRegex(ValueError, 'cannot alter'):
                        resume_inventory(batch, build, dict(seeds=[12]))
                elif kind == 'invalid':
                    (batch / job_name(jobs[0]) / 'trial_audit.json').write_text('{"valid_experiment":false}')
                    with self.assertRaisesRegex(ValueError, 'invalid trial'):
                        resume_inventory(batch, build)
                else:
                    (batch / job_name(jobs[0])).rename(batch / 'unplanned_saved_trial')
                    with self.assertRaises(ValueError):
                        resume_inventory(batch, build)

    def test_resume_rejects_tampered_audited_bytes_and_changed_build(self):
        for kind in ('input', 'build'):
            with self.subTest(kind=kind), tempfile.TemporaryDirectory(prefix='coupling_resume_') as directory:
                batch, build, jobs = self.resume_fixture(Path(directory))
                if kind == 'input':
                    (batch / job_name(jobs[0]) / 'result.json').write_text('{"reason":"goal"}')
                else:
                    (build / 'build_manifest.json').write_text('{}')
                with self.assertRaises(ValueError):
                    resume_inventory(batch, build)

    def test_resume_accepts_archived_audited_bytes_without_restoring_them(self):
        with tempfile.TemporaryDirectory(prefix='coupling_resume_') as directory:
            batch, build, jobs = self.resume_fixture(Path(directory))
            path = batch / job_name(jobs[0]) / 'result.json'
            # result.json is a required status record and intentionally stays
            # plain; representative large audited timeline uses gzip.
            data = path.parent / 'dual_timeline.jsonl'
            blob = b'{"event":"request_begin"}\n'
            with gzip.open(str(data) + '.gz', 'wb') as stream:
                stream.write(blob)
            audit_path = path.parent / 'trial_audit.json'
            audit = read(audit_path)
            import hashlib
            audit['inputs_sha256'][data.name] = hashlib.sha256(blob).hexdigest()
            audit_path.write_text(json.dumps(audit))
            self.assertEqual(len(resume_inventory(batch, build)['completed']), 8)
            self.assertFalse(data.exists())

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
