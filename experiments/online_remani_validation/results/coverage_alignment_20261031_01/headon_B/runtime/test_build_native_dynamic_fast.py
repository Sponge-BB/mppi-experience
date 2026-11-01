"""Isolated child/hash/compiler tests; no ROS graph or actuation."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import build_native_dynamic_fast as builder


class BuildContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.parent = builder.validate(builder.BASE)

    def test_rejects_other_parent(self):
        with self.assertRaisesRegex(ValueError, 'frozen native_realtime'):
            builder.validate(builder.HERE / 'builds/native_collision_3d_20261027_04')

    def test_exactly_one_translation_unit_and_same_compiler_flags(self):
        original = self.parent['native_realtime_recipe']['mppi']
        value = builder.recipe(self.parent, builder.BASE, builder.DEFAULT_OUT)
        item = value['mppi']
        self.assertEqual([arg for arg in item['compile_command'] if arg.endswith('.cpp')],
                         [str(builder.DEFAULT_OUT / 'source/online_api.cpp')])
        self.assertEqual(item['compile_command'],
                         [arg.replace(str(builder.BASE), str(builder.DEFAULT_OUT)) for arg in original['compile_command']])
        self.assertIn('-O2', item['compile_command'])
        self.assertNotIn('-ffast-math', item['compile_command'])
        self.assertEqual(item['cwd'], original['cwd'])
        builder.previous.verify_recipe(value)

    def test_prepare_has_no_subprocess_and_parent_unchanged(self):
        before = builder.sha(builder.BASE / 'build_manifest.json')
        with tempfile.TemporaryDirectory(prefix='dynamic_fast_prepare_') as directory:
            out = Path(directory) / 'child'
            with patch.object(builder.subprocess, 'run', side_effect=AssertionError('no compile during prepare')):
                builder.prepare(builder.BASE, out)
                manifest = builder.check(out)
            self.assertTrue(manifest['prepared_only'])
            self.assertFalse(manifest['production_modified'])
            self.assertFalse(manifest['native_dynamic_fast_switch_default'])
            self.assertEqual(len(manifest['native_dynamic_fast_reused_binary_sha256']), 5)
            self.assertEqual(set(manifest['isolated_source_sha256']) - set(self.parent['isolated_source_sha256']),
                             {builder.HEADER})
            self.assertFalse((out / 'build/CMakeCache.txt').exists())
            with self.assertRaisesRegex(ValueError, 'refusing overwrite'):
                builder.prepare(builder.BASE, out)
        self.assertEqual(builder.sha(builder.BASE / 'build_manifest.json'), before)

    def test_tampered_child_fails_closed(self):
        with tempfile.TemporaryDirectory(prefix='dynamic_fast_drift_') as directory:
            out = Path(directory) / 'child'
            builder.prepare(builder.BASE, out)
            target = out / 'source/native_collision_3d.hpp'
            target.write_text(target.read_text() + '\n')
            with self.assertRaisesRegex(ValueError, 'inventory drift'):
                builder.check(out)


if __name__ == '__main__':
    unittest.main()
