#!/usr/bin/env python3
"""Graph-free tests: no ROS import, compile, node, model rollout or hardware."""
import ast
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import prepare_directed_low_speed as fixture


class DirectedLowSpeedFixture(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original=(Path(__file__).with_name('simulator.py')).read_text()

    def test_private_transform_preserves_sources_and_control_publishers(self):
        text=fixture.transform_simulator(self.original)
        self.assertIn('self.s=np.array([0.,0.,0.,-.04,0.],dtype=np.float64)',text)
        self.assertIn('goal.pose.position.x=-.8;goal.pose.orientation.w=1.;',text)
        self.assertIn('at_goal=False # This state-only probe never claims physical goal reach.',text)
        self.assertIn("reason='probe_complete'",text)
        self.assertIn('self.t>=14.5',text)
        self.assertIn('self.t>=18.',text)
        self.assertIn('self.directed_first_reference_time+.5',text)
        self.assertEqual(text.count("rospy.Publisher('/odom'"),1)
        self.assertEqual(text.count("rospy.Publisher('/sim/cmd_vel'"),0)
        self.assertEqual(text.count('self.odom_pub.publish(o)'),1)
        self.assertEqual(text.count('self.raw_pub.publish('),1)
        self.assertEqual(text.count('self.joint_pub.publish(j)'),1)
        self.assertIn('self.command=np.array([m.twist.linear.x,m.twist.angular.z])',text)
        self.assertIn('commands_integrated',text)
        ast.parse(text)

    def test_injected_integrate_holds_declared_state_and_does_not_use_commands(self):
        text=fixture.transform_simulator(self.original)
        tree=ast.parse(text)
        method=next(m for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Simulator'
                    for m in n.body if isinstance(m,ast.FunctionDef) and m.name=='integrate')
        # Extract and call only the private 1-line fixture method. The numerical
        # plant, ROS callbacks and native algorithms are not loaded/executed.
        namespace={};exec(compile(ast.Module(body=[method],type_ignores=[]),'fixture_integrate','exec'),namespace)
        state=[10.,20.,30.,40.,50.]
        node=SimpleNamespace(s=state,command=object())
        namespace['integrate'](node,100.)
        self.assertEqual(state,list(fixture.FIXTURE_STATE))

    def test_preparation_geometry_is_safe_but_not_native_map_certificate(self):
        result=fixture.fixture_geometry([[-.125,0.,.28],[.125,0.,.28]],.1)
        self.assertGreater(result['initial_world_physical_m'],.1)
        self.assertGreater(result['goal_world_physical_m'],.1)
        self.assertFalse(result['native_3d_replay_or_safety_certificate'])
        self.assertTrue(result['online_frozen_field_endpoint_validation_required'])
        for body,margin in (([[0.,0.,2.]],.1),([[0.,0.,.2]],.11),([[0.,0.,float('nan')]],.1)):
            with self.assertRaises(ValueError):fixture.fixture_geometry(body,margin)

    def test_source_drift_and_repeat_transform_fail_closed(self):
        with self.assertRaises(ValueError):fixture.transform_simulator('unknown simulator')
        with self.assertRaises(ValueError):fixture.transform_simulator(fixture.transform_simulator(self.original))

    def test_only_new_private_snapshot_changes_and_manifest_hashes_follow(self):
        with tempfile.TemporaryDirectory(prefix='directed_fixture_') as tmp:
            out=Path(tmp);runtime=out/'runtime';runtime.mkdir();path=runtime/'simulator.py'
            path.write_text(self.original)
            before=fixture.sha(path)
            (out/'geometry.csv').write_text('-.125,0.,.28\n.125,0.,.28\n')
            (out/'limits.json').write_text(json.dumps({'margin':.1}))
            manifest=dict(scene='empty',mode='mppi',local_execution_owner=True,
                experiment_sources_sha256={'simulator.py':before},coupling_experiment={'genuine_online_closed_loop':True})
            (out/'manifest.json').write_text(json.dumps(manifest))
            settings={'genuine_online_closed_loop':True}
            self.assertIs(fixture.apply_fixture(out,settings),settings)
            after=json.loads((out/'manifest.json').read_text())
            self.assertEqual(after['experiment_sources_sha256']['simulator.py'],fixture.sha(path))
            self.assertNotEqual(before,fixture.sha(path))
            self.assertEqual(after['parameters_sha256'],fixture.sha(out/'coupling_parameters.json'))
            self.assertFalse(after['coupling_experiment']['genuine_online_closed_loop'])
            self.assertEqual(settings,after['coupling_experiment'])
            self.assertEqual(settings['fixture_goal'],[-.8,0.,0.])
            self.assertEqual(after['directed_low_speed_probe']['original_private_simulator_sha256'],before)
            self.assertFalse(after['directed_low_speed_probe']['paired_same_complete_native_input'])
            with self.assertRaises(ValueError):fixture.patch_prepared_trial(out)
            # Never touched the shared workspace source.
            self.assertEqual((Path(__file__).with_name('simulator.py')).read_text(),self.original)

    def test_changed_or_symlink_snapshot_is_rejected_before_write(self):
        with tempfile.TemporaryDirectory(prefix='directed_fixture_guard_') as tmp:
            out=Path(tmp);runtime=out/'runtime';runtime.mkdir();path=runtime/'simulator.py'
            path.write_text(self.original)
            manifest=dict(scene='empty',mode='mppi',local_execution_owner=True,
                experiment_sources_sha256={'simulator.py':'0'*64})
            (out/'manifest.json').write_text(json.dumps(manifest))
            with self.assertRaises(ValueError):fixture.patch_prepared_trial(out)
            self.assertEqual(path.read_text(),self.original)

    def test_preparer_requires_built_state_contract_and_explicit_dependency(self):
        with tempfile.TemporaryDirectory(prefix='directed_fixture_prepare_') as tmp:
            build=Path(tmp);(build/'build_manifest.json').write_text(json.dumps({'state_contract_experiment':True,'state_contract_prepared_only':False}))
            previous=fixture.ORIGINAL_PREPARER
            try:
                fixture.ORIGINAL_PREPARER=None
                with self.assertRaises(ValueError):fixture.prepare_trial(build/'out',build,'empty','coupled',11,12.)
                with self.assertRaises(ValueError):fixture.set_original_preparer(fixture.prepare_trial)
                with self.assertRaises(ValueError):fixture.prepare_trial(build/'out',build,'static','coupled',11,12.)
                with self.assertRaises(ValueError):fixture.prepare_trial(build/'out',build,'empty','native',11,12.)
                with self.assertRaises(ValueError):fixture.prepare_trial(build/'out',build,'empty','coupled',11,5.)
            finally:fixture.ORIGINAL_PREPARER=previous

    def test_hook_refuses_unknown_config_or_non_dictionary_settings(self):
        with self.assertRaises(ValueError):fixture.apply_fixture('/not-created',None)
        with self.assertRaises(ValueError):fixture.apply_fixture('/not-created',{}, {'speed':-.03})


if __name__=='__main__':unittest.main()
