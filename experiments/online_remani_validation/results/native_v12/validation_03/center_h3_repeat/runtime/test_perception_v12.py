#!/usr/bin/env python3
"""Compile the isolated tracker transform and its meaningful regressions."""
import argparse
from pathlib import Path
import subprocess
import tempfile
import unittest

from perception_v12 import profile_overrides, transform_core

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PARENT = HERE / 'builds/fixed_arm_01/source/mm_remani'


class TransformTests(unittest.TestCase):
    def test_transform_requires_exact_snapshot(self):
        with self.assertRaises(ValueError):
            transform_core('')
        original = (PARENT / 'src/dynamic_obstacle_tracker_core.cpp').read_text()
        with self.assertRaises(ValueError):
            transform_core(transform_core(original))

    def test_profile_does_not_override_health_or_identity(self):
        self.assertTrue(all(key.rsplit('/', 1)[-1] in (
            'dynamic_enter_speed', 'dynamic_exit_speed', 'velocity_deadband',
            'minimum_dynamic_displacement', 'motion_history_duration')
            for key in profile_overrides()))


def compile_regressions(directory, original=False):
    source = (PARENT / 'src/dynamic_obstacle_tracker_core.cpp').read_text()
    # This is a generated compilation input, never a production source edit.
    generated = directory / ('original_core.cpp' if original else 'v12_core.cpp')
    generated.write_text(source if original else transform_core(source))
    binary = directory / ('original_tests' if original else 'v12_tests')
    subprocess.run([
        '/usr/bin/c++', '-std=c++14', '-O2', '-pthread',
        '-I' + str(PARENT / 'include'), '-I/usr/include/eigen3',
        str(generated), str(PARENT / 'test/test_dynamic_obstacle_tracker_core.cpp'),
        str(HERE / 'test_perception_v12.cpp'), '-lgtest_main', '-lgtest',
        '-o', str(binary)], check=True)
    return binary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--show-original-regression', action='store_true')
    args = parser.parse_args()
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(TransformTests)
    if not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful():
        raise SystemExit(1)
    with tempfile.TemporaryDirectory(prefix='v12_perception_') as temporary:
        directory = Path(temporary)
        if args.show_original_regression:
            baseline = compile_regressions(directory, original=True)
            result = subprocess.run([str(baseline),
                '--gtest_filter=V12Perception.CoherentTranslationCannotBecomeDwellBackground'])
            if result.returncode == 0:
                raise RuntimeError('Expected original dwell regression was not reproduced')
        binary = compile_regressions(directory)
        subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
