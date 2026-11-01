#!/usr/bin/env python3
"""Selective builder contracts; no C++ compile, ROS process or vehicle I/O."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import build_budget_scaling as builder


class BuilderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.parent = builder.validate(builder.BASE)

    def test_actual_parent_and_complete_chain(self):
        self.assertTrue(self.parent['feasibility_stop_experiment'])
        self.assertFalse(self.parent['prepared_only'])
        self.assertEqual(builder.previous.source_inventory(builder.BASE / 'source'),
                         self.parent['isolated_source_sha256'])
        self.assertIn('feasibility_stop_gate.hpp', self.parent['isolated_source_sha256'])
        self.assertIn('domain_lbfgs.hpp', self.parent['isolated_source_sha256'])

    def test_refuse_different_parent(self):
        with self.assertRaisesRegex(ValueError, 'frozen feasibility_stop'):
            builder.validate(builder.HERE / 'builds/seed_parameterization_20261025_01')

    def test_recipe_rebuilds_only_optimizer_and_planner(self):
        parent = Path(self.parent['parent_build'])
        recipe = builder.previous.compile_commands(parent, builder.DEFAULT_OUT)
        builder.previous.validate_recipe(recipe)
        self.assertEqual([r['source'] for r in recipe['compile_commands']],
            ['poly_traj_optimizer.cpp', 'remani_planner_node.cpp',
             'remani_replan_fsm.cpp', 'planner_manager.cpp'])
        self.assertEqual(recipe['archive_command'][0:2], ['ar', 'r'])
        self.assertEqual(recipe['archive_command'][2], str(builder.DEFAULT_OUT / 'build/libexp_opt.a'))
        self.assertEqual(recipe['link_command'].count(str(builder.DEFAULT_OUT / 'build/libexp_search.a')), 1)
        for command in recipe['compile_commands']:
            self.assertIn('-O2', command['command'])
            self.assertNotIn('-ffast-math', command['command'])
            self.assertIn('-I' + str(builder.DEFAULT_OUT / 'source'), command['command'])

    def test_source_inventory_rejects_symlinks(self):
        with tempfile.TemporaryDirectory(prefix='remani_builder_inventory_') as directory:
            folder = Path(directory)
            (folder / 'unsafe').symlink_to(builder.BASE / 'source/domain_lbfgs.hpp')
            with self.assertRaisesRegex(ValueError, 'Symlink'):
                builder.previous.source_inventory(folder)

    def test_prepare_and_check_without_compile_or_cache_copy(self):
        import patch_budget_scaling as patch
        with tempfile.TemporaryDirectory(prefix='remani_budget_builder_') as directory:
            out = Path(directory) / 'prepared'
            with mock.patch.object(builder.subprocess, 'run', side_effect=AssertionError('Compiler/process forbidden')):
                builder.prepare(builder.BASE, out)
                manifest = builder.check(out)
            self.assertTrue(manifest['prepared_only'])
            self.assertTrue(manifest['budget_scaling_prepared_only'])
            self.assertEqual(manifest['budget_scaling_diagnostic_phases'], list(patch.PHASES))
            self.assertEqual(manifest['budget_scaling_matrix']['budgets_s'], list(patch.BUDGETS))
            self.assertEqual({p.name for p in (out / 'build').iterdir()}, set(builder.PRODUCTS))
            self.assertFalse((out / 'build/CMakeFiles').exists())
            self.assertFalse((out / 'build/CMakeCache.txt').exists())
            self.assertEqual(len(manifest['isolated_source_sha256']),
                             len(self.parent['isolated_source_sha256']) + 1)
            for name in ('domain_lbfgs.hpp', 'feasibility_stop_gate.hpp', 'kino_astar.cpp'):
                self.assertEqual(builder.sha(out / 'source' / name),
                                 self.parent['isolated_source_sha256'][name])
            for name in builder.PRODUCTS:
                self.assertEqual(builder.sha(out / 'build' / name), builder.sha(builder.BASE / 'build' / name))
            for name, kind in zip(builder.PATCHED, ('optimizer', 'replay')):
                self.assertEqual(patch.project_off((out / 'source' / name).read_text(), kind=kind),
                                 (builder.BASE / 'source' / name).read_text())
            with self.assertRaises(FileExistsError):
                builder.prepare(builder.BASE, out)

    def test_prepared_metadata_drift_is_rejected(self):
        with tempfile.TemporaryDirectory(prefix='remani_budget_metadata_') as directory:
            out = Path(directory) / 'prepared'
            builder.prepare(builder.BASE, out)
            manifest_path = out / 'build_manifest.json'
            original = json.loads(manifest_path.read_text())
            for key, wrong in (
                    ('budget_scaling_parent_source_sha256', {}),
                    ('budget_scaling_parent_archive_sha256', {}),
                    ('budget_scaling_builder_sha256', '0' * 64),
                    ('budget_scaling_patcher_sha256', '0' * 64),
                    ('budget_scaling_header_sha256', '0' * 64),
                    ('budget_scaling_diagnostic_phases', []),
                    ('budget_scaling_matrix', {}),
                    ('budget_scaling_prepared_only', False)):
                bad = copy.deepcopy(original)
                bad[key] = wrong
                manifest_path.write_text(json.dumps(bad))
                with self.subTest(key=key), self.assertRaises(ValueError):
                    builder.check(out)
            manifest_path.write_text(json.dumps(original))
            self.assertTrue(builder.check(out)['prepared_only'])

    def test_prepared_source_and_reused_product_drift_rejected(self):
        with tempfile.TemporaryDirectory(prefix='remani_budget_drift_') as directory:
            out = Path(directory) / 'prepared'
            builder.prepare(builder.BASE, out)
            for relative in ('source/domain_lbfgs.hpp', 'source/budget_scaling.hpp',
                             'build/libexp_search.a', 'build/libonline_mppi.so',
                             'build/remani_planner_node', 'build/libexp_opt.a'):
                path = out / relative
                original = path.read_bytes()
                path.write_bytes(original + b'\n')
                with self.subTest(path=relative), self.assertRaises(ValueError):
                    builder.check(out)
                path.write_bytes(original)
            self.assertTrue(builder.check(out)['prepared_only'])

    def test_output_symlink_refused_before_copy(self):
        with tempfile.TemporaryDirectory(prefix='remani_budget_symlink_') as directory:
            out = Path(directory) / 'unsafe'
            out.symlink_to(builder.BASE, target_is_directory=True)
            with self.assertRaisesRegex(ValueError, 'symlink'):
                builder.prepare(builder.BASE, out)


if __name__ == '__main__':
    unittest.main()
