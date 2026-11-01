"""Local runtime-copy and execution-invariant checks; never starts ROS."""
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import run_native_dynamic_fast as runner

HERE = Path(__file__).resolve().parent


class RuntimeContractTests(unittest.TestCase):
    def test_only_launch_switch_changes_python_runtime_identical(self):
        before = {name: (HERE / name).read_bytes()
                  for name in ('mppi_node.py', 'local_execution.py', 'simulator.py', 'native_collision_runtime.py')}
        with tempfile.TemporaryDirectory(prefix='dynamic_fast_runtime_') as directory:
            copies = []
            for enabled in (False, True):
                out = Path(directory) / str(enabled)
                (out / 'runtime').mkdir(parents=True)
                for name in ('mppi_node.py', 'local_execution.py', 'simulator.py'):
                    (out / 'runtime' / name).write_bytes(before[name])
                (out / 'experiment.launch').write_text('<launch><node name="base_mppi"/><node name="remani_planner_node"/></launch>')
                (out / 'manifest.json').write_text(json.dumps({'experiment_sources_sha256': {}}))
                runner.patch_trial(out, enabled)
                copies.append({p.name: p.read_bytes() for p in (out / 'runtime').glob('*.py')})
                for node in ET.parse(out / 'experiment.launch').getroot().findall('node'):
                    flags = {e.get('name'): e.get('value') for e in node.findall('env')}
                    self.assertEqual(flags['NATIVE_DYNAMIC_FAST'], str(int(enabled)))
                    self.assertEqual(flags['NATIVE_REALTIME_DEMAND'], '1')
                    self.assertEqual(flags['NATIVE_REALTIME_PROFILE'], '0')
                    self.assertEqual(flags['NATIVE_REALTIME_REPLAY'], '0')
                    self.assertEqual(flags['NATIVE_COLLISION_3D'], '1')
                manifest = json.loads((out / 'manifest.json').read_text())
                self.assertEqual(manifest['native_dynamic_fast']['enabled'], enabled)
                self.assertTrue(manifest['native_dynamic_fast']['geometry_and_search_unchanged'])
                self.assertIn("if computation_ms>50:reason='deadline_miss'", copies[-1]['local_execution.py'].decode())
                self.assertIn('self.command_lease=.075', copies[-1]['simulator.py'].decode())
            self.assertEqual(copies[0], copies[1])
        for name, data in before.items():
            self.assertEqual((HERE / name).read_bytes(), data)

    def test_environment_clears_diagnostic_and_fast_flags_then_restores(self):
        inherited = dict(NATIVE_DYNAMIC_FAST='1', NATIVE_DYNAMIC_PROFILE='1',
                         NATIVE_REALTIME_PROFILE='1', NATIVE_REALTIME_REPLAY='1')
        with patch.dict(os.environ, inherited, clear=True):
            with runner.clean_environment():
                self.assertNotIn('NATIVE_DYNAMIC_FAST', os.environ)
                self.assertNotIn('NATIVE_DYNAMIC_PROFILE', os.environ)
                self.assertEqual(os.environ['NATIVE_REALTIME_PROFILE'], '0')
                self.assertEqual(os.environ['NATIVE_REALTIME_REPLAY'], '0')
            self.assertEqual(dict(os.environ), inherited)

    def test_missing_or_failed_replay_never_starts_ros(self):
        with tempfile.TemporaryDirectory(prefix='dynamic_fast_gate_') as directory:
            out = Path(directory) / 'not_created'
            build = Path(directory) / 'build'
            with patch.object(runner.builder, 'check', return_value={'prepared_only': False}), \
                 patch.object(runner, 'run_trial', side_effect=AssertionError('must not start')):
                with self.assertRaisesRegex(ValueError, 'replay gate'):
                    runner.run_batch(build, out)
                replay = Path(directory) / 'replay.json'
                replay.write_text(json.dumps({'passed': False}))
                with self.assertRaisesRegex(ValueError, 'did not pass'):
                    runner.run_batch(build, out, replay=replay)
            self.assertFalse(out.exists())


if __name__ == '__main__':
    unittest.main()
