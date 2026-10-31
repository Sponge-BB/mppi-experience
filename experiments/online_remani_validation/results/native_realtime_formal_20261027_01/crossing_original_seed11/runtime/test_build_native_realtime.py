#!/usr/bin/env python3
"""Frozen-parent/hash/MPPI-only build contracts; never launch a ROS process."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import build_native_realtime as builder


class ParentAndRecipeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.parent = builder.validate(builder.BASE)

    def test_completed_04_and_no_other_parent(self):
        self.assertTrue(self.parent['native_collision_3d_experiment'])
        self.assertFalse(self.parent['prepared_only'])
        self.assertEqual(builder.previous.native_recipe.source_inventory(builder.BASE / 'source'),
                         self.parent['isolated_source_sha256'])
        with self.assertRaisesRegex(ValueError, 'frozen native_collision_3d_20261027_04'):
            builder.validate(builder.HERE / 'builds/native_collision_3d_20261027_03')

    def test_actual_mppi_only_include_chain(self):
        self.assertEqual(builder.prove_mppi_only_header_dependency(builder.BASE),
            {'native_collision_3d.hpp': ['online_kernel.hpp'], 'online_kernel.hpp': ['online_api.cpp']})

    def test_added_native_header_dependency_fails_closed(self):
        with tempfile.TemporaryDirectory(prefix='realtime_include_') as directory:
            base = Path(directory)
            (base / 'source').mkdir()
            (base / 'source/online_kernel.hpp').write_text('#include "native_collision_3d.hpp"\n')
            (base / 'source/online_api.cpp').write_text('#include "online_kernel.hpp"\n')
            (base / 'source/planner_manager.cpp').write_text('#include "native_collision_3d.hpp"\n')
            with self.assertRaisesRegex(ValueError, 'not isolated'):
                builder.prove_mppi_only_header_dependency(base)

    def test_only_one_translation_unit_no_native_or_archive_recipe(self):
        recipe = builder.recipe(self.parent, builder.BASE, builder.DEFAULT_OUT)
        builder.verify_recipe(recipe)
        self.assertEqual(set(recipe), {'mppi', 'frozen_compile_inputs_sha256',
                                      'unchanged_native_link_dependencies_sha256'})
        command = recipe['mppi']['compile_command']
        self.assertEqual([x for x in command if x.endswith('.cpp')],
                         [str(builder.DEFAULT_OUT / 'source/online_api.cpp')])
        self.assertIn('-O2', command)
        self.assertNotIn('-ffast-math', command)
        self.assertEqual(recipe['frozen_compile_inputs_sha256'],
                         self.parent['native_collision_3d_recipe']['frozen_compile_inputs_sha256'])
        self.assertEqual(len(builder.REUSED), 5)
        self.assertEqual(len(builder.PRODUCTS), 6)

    def test_parent_compiler_baseline_cannot_be_replaced_by_current_recipe(self):
        original = builder._mppi_recipe

        def drifted(manifest, out):
            item = original(manifest, out)
            item['compile_command'] = item['compile_command'] + ['-ffast-math']
            return item

        with mock.patch.object(builder, '_mppi_recipe', side_effect=drifted):
            with self.assertRaisesRegex(ValueError, 'frozen compiler baseline'):
                builder.recipe(self.parent, builder.BASE, builder.DEFAULT_OUT)

    def test_frozen_compiler_input_drift_is_not_silently_rebaselined(self):
        recipe = builder.recipe(self.parent, builder.BASE, builder.DEFAULT_OUT)
        with tempfile.TemporaryDirectory(prefix='realtime_frozen_input_') as directory:
            header = Path(directory) / 'model.h'
            header.write_text('original\n')
            changed = copy.deepcopy(recipe)
            changed['frozen_compile_inputs_sha256'] = {str(header): builder.sha(header)}
            header.write_text('changed\n')
            with self.assertRaisesRegex(ValueError, 'SHA256 mismatch'):
                builder.verify_recipe(changed)


class PreparedChildTests(unittest.TestCase):
    """Run these only after the patch/probe owner explicitly confirms freeze."""

    @classmethod
    def setUpClass(cls):
        cls.parent = builder.validate(builder.BASE)

    def test_exact_declared_transform_scope(self):
        changed = builder.transformed_sources(builder.BASE)
        self.assertEqual(set(changed), set(builder.PATCHED))
        self.assertNotIn('mppi_v12.hpp', changed)
        self.assertNotIn('native_collision_export.hpp', changed)

    def test_prepare_no_process_no_cache_copy_and_all_native_byte_reused(self):
        with tempfile.TemporaryDirectory(prefix='realtime_prepare_') as directory:
            out = Path(directory) / 'prepared'
            with mock.patch.object(builder.subprocess, 'run', side_effect=AssertionError('No process allowed')):
                builder.prepare(builder.BASE, out)
                manifest = builder.check(out)
            self.assertTrue(manifest['prepared_only'])
            self.assertEqual(set(p.name for p in (out / 'build').iterdir()), set(builder.PRODUCTS))
            self.assertFalse((out / 'build/CMakeCache.txt').exists())
            self.assertFalse((out / 'build/CMakeFiles').exists())
            self.assertEqual(len(manifest['isolated_source_sha256']), len(self.parent['isolated_source_sha256']) + 1)
            self.assertEqual(manifest['native_realtime_reused_binary_sha256'], builder.reused_hashes(self.parent))
            self.assertEqual(manifest['native_realtime_switches'], {name: False for name in builder.SWITCHES})
            for name in builder.REUSED:
                self.assertEqual(builder.sha(out / 'build' / name), builder.sha(builder.BASE / 'build' / name))
            for name in ('mppi_v12.hpp', 'poly_traj_optimizer.cpp', 'native_collision_export.hpp',
                         'planner_manager.cpp', 'remani_replan_fsm.cpp'):
                self.assertEqual(builder.sha(out / 'source' / name), self.parent['isolated_source_sha256'][name])
            with self.assertRaises(FileExistsError):
                builder.prepare(builder.BASE, out)

    def test_source_binary_and_manifest_drift_rejected(self):
        with tempfile.TemporaryDirectory(prefix='realtime_drift_') as directory:
            out = Path(directory) / 'prepared'
            builder.prepare(builder.BASE, out)
            original_manifest = builder.check(out)
            for relative in ('source/native_collision_3d.hpp', 'source/native_realtime_probe.hpp',
                             'source/mppi_v12.hpp', 'build/remani_planner_node',
                             'build/libexp_opt.a', 'build/libexp_search.a', 'build/libonline_mppi.so'):
                path = out / relative
                original = path.read_bytes()
                path.write_bytes(original + b'\n')
                with self.subTest(relative=relative), self.assertRaises(ValueError):
                    builder.check(out)
                path.write_bytes(original)
            manifest_path = out / 'build_manifest.json'
            for key, wrong in (
                    ('native_realtime_parent_source_sha256', {}),
                    ('native_realtime_parent_build_inputs_sha256', {}),
                    ('native_realtime_source_contract', {}),
                    ('native_realtime_inputs_sha256', {}),
                    ('native_realtime_reused_binary_sha256', {}),
                    ('native_realtime_switches', {}),
                    ('native_realtime_prepared_only', False),
                    ('native_realtime_include_proof', {})):
                changed = copy.deepcopy(original_manifest)
                changed[key] = wrong
                manifest_path.write_text(json.dumps(changed))
                with self.subTest(key=key), self.assertRaises(ValueError):
                    builder.check(out)
            manifest_path.write_text(json.dumps(original_manifest))
            self.assertTrue(builder.check(out)['prepared_only'])

    def test_compile_dispatch_only_two_mppi_commands_and_refuses_unchanged_library(self):
        with tempfile.TemporaryDirectory(prefix='realtime_dispatch_') as directory:
            out = Path(directory) / 'prepared'
            builder.prepare(builder.BASE, out)
            manifest = builder.check(out)
            with mock.patch.object(builder.subprocess, 'run') as run:
                with self.assertRaisesRegex(ValueError, 'MPPI library unexpectedly unchanged'):
                    builder.compile_prepared(out)
            recipe = manifest['native_realtime_recipe']['mppi']
            self.assertEqual(run.call_args_list, [
                mock.call(recipe['compile_command'], check=True),
                mock.call(recipe['actual_link_command'], cwd=recipe['cwd'], check=True)])
            for name in builder.REUSED:
                self.assertEqual(builder.sha(out / 'build' / name), builder.sha(builder.BASE / 'build' / name))

    def test_output_symlink_refused(self):
        with tempfile.TemporaryDirectory(prefix='realtime_symlink_') as directory:
            out = Path(directory) / 'unsafe'
            out.symlink_to(builder.BASE, target_is_directory=True)
            with self.assertRaisesRegex(ValueError, 'symlink'):
                builder.prepare(builder.BASE, out)


if __name__ == '__main__':
    unittest.main()
