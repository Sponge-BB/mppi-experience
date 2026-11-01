#!/usr/bin/env python3
"""Read-only parent attestation and bounded child preparation; no ROS launch."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import build_native_collision_3d as builder
import patch_native_collision_planner as planner


class PlannerPatchTests(unittest.TestCase):
    def test_exact_three_file_off_projection(self):
        for name in builder.PLANNER_PATCHED:
            original = (builder.BASE / 'source' / name).read_text()
            changed = planner.transform(original, name)
            self.assertEqual(planner.project_off(changed, name), original)
            self.assertEqual(changed.count(planner.INCLUDE), 1)
            with self.assertRaises(ValueError):
                planner.transform(changed, name)

    def test_native_hard_checks_budgets_and_iterations_not_rewritten(self):
        changed = planner.transform((builder.BASE / 'source/coupling_bridge.hpp').read_text(), 'coupling_bridge.hpp')
        self.assertIn('if(native_collision_export::enabled())return true;', changed)
        optimizer = (builder.BASE / 'source/poly_traj_optimizer.cpp').read_text()
        self.assertIn('nh.param("optimization/max_iterations", max_iterations_, 300)', optimizer)
        self.assertIn('IsTrajSafe(', optimizer)
        self.assertNotIn('max_iterations', ''.join(planner.TRANSFORMS))

    def test_exporter_thread_and_geometry_contract(self):
        text = (builder.HERE / 'native_collision_export.hpp').read_text()
        output = text[text.index('  void output('):text.index('\npublic:')]
        self.assertEqual(output.count('map_->'), 1)
        self.assertIn('map_->getDynamicPointSnapshot(now)', output)
        self.assertNotIn('model_->', output)
        self.assertIn('std::atomic_load(&cache_)', output)
        self.assertIn('getCarPts(Eigen::Vector3d::Zero(),car)', text)
        self.assertIn('model_->getJointTrans(fixed_q_', text)
        self.assertIn('model_->getLinkPoint()', text)
        self.assertIn('map_->getDistance(Eigen::Vector3i(x,y,z))', text)
        self.assertIn('radius+model_->getMapResolution()', text)
        self.assertIn('weight_/5.', text)
        self.assertIn('model_->checkcollision(pose,fixed_q_,false,type)', text)
        self.assertIn('NATIVE_COLLISION_PARITY', text)


class BuilderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.parent = builder.validate(builder.BASE)

    def test_parent_complete_chain_and_no_wrong_parent(self):
        self.assertTrue(self.parent['budget_scaling_experiment'])
        self.assertFalse(self.parent['prepared_only'])
        with self.assertRaisesRegex(ValueError, 'frozen budget_scaling'):
            builder.validate(builder.HERE / 'builds/feasibility_stop_20261026_01')

    def test_selective_recipe(self):
        recipe = builder.recipes(self.parent, builder.DEFAULT_OUT)
        builder.verify_recipes(recipe)
        self.assertEqual([x['source'] for x in recipe['native']['compile_commands']],
            ['poly_traj_optimizer.cpp', 'remani_planner_node.cpp', 'remani_replan_fsm.cpp', 'planner_manager.cpp'])
        self.assertIn(str(builder.DEFAULT_OUT / 'source/online_api.cpp'), recipe['mppi']['compile_command'])
        self.assertEqual(recipe['native']['archive_command'][0:2], ['ar', 'r'])

    def test_prepare_integrity_drift_and_no_cache_copy(self):
        with tempfile.TemporaryDirectory(prefix='native_collision_builder_') as directory:
            out = Path(directory) / 'prepared'
            with mock.patch.object(builder.subprocess, 'run', side_effect=AssertionError('No compiler/process allowed')):
                builder.prepare(builder.BASE, out)
                manifest = builder.check(out)
            self.assertTrue(manifest['prepared_only'])
            self.assertIs(manifest['boundary_admission_experiment'], False)
            self.assertEqual(manifest['native_collision_3d_parent_boundary_experiment'],
                             bool(self.parent.get('boundary_admission_experiment')))
            self.assertEqual(set(p.name for p in (out / 'build').iterdir()), set(builder.PRODUCTS))
            self.assertFalse((out / 'build/CMakeCache.txt').exists())
            self.assertFalse((out / 'build/CMakeFiles').exists())
            self.assertEqual(len(manifest['isolated_source_sha256']), len(self.parent['isolated_source_sha256']) + 2)
            for relative in ('source/native_collision_export.hpp', 'source/planner_manager.cpp',
                             'build/libexp_search.a', 'build/libonline_mppi.so'):
                path = out / relative
                original = path.read_bytes()
                path.write_bytes(original + b'\n')
                with self.subTest(relative=relative), self.assertRaises(ValueError):
                    builder.check(out)
                path.write_bytes(original)
            manifest_path = out / 'build_manifest.json'
            for key, wrong in (('native_collision_3d_parent_source_sha256', {}),
                               ('native_collision_3d_inputs_sha256', {}),
                               ('native_collision_3d_prepared_only', False)):
                changed = copy.deepcopy(manifest)
                changed[key] = wrong
                manifest_path.write_text(json.dumps(changed))
                with self.subTest(key=key), self.assertRaises(ValueError):
                    builder.check(out)
            manifest_path.write_text(json.dumps(manifest))
            self.assertTrue(builder.check(out)['prepared_only'])
            with self.assertRaises(FileExistsError):
                builder.prepare(builder.BASE, out)


if __name__ == '__main__':
    unittest.main()
