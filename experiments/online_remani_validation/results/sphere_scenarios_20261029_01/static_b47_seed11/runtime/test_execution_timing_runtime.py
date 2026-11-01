"""Small source/parameter checks for the isolated timing comparison."""
import ast
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from execution_timing_runtime import patch_trial

HERE = Path(__file__).resolve().parent


class ExecutionTimingRuntimeTests(unittest.TestCase):
    def test_paired_sources_identical_only_launch_timings_differ(self):
        names = ('mppi_node.py', 'local_execution.py', 'simulator.py', 'native_collision_runtime.py')
        original = {name: (HERE / name).read_bytes() for name in names}
        copies = []
        with tempfile.TemporaryDirectory(prefix='execution_timing_') as directory:
            for group, deadline, lease in (('current', 50., .075), ('matched', 200., .250)):
                out = Path(directory) / group
                (out / 'runtime').mkdir(parents=True)
                for name in names[:3]:
                    (out / 'runtime' / name).write_bytes(original[name])
                (out / 'experiment.launch').write_text('<launch><node name="base_mppi"/><node name="remani_planner_node"/><node name="algorithm_simulator"/></launch>')
                (out / 'manifest.json').write_text(json.dumps({'experiment_sources_sha256': {}}))
                patch_trial(out, deadline, lease)
                copies.append({p.name: p.read_bytes() for p in (out / 'runtime').glob('*.py')})
                for text in copies[-1].values():
                    ast.parse(text)
                nodes = {node.get('name'): node for node in ET.parse(out / 'experiment.launch').getroot().findall('node')}
                for name in ('base_mppi', 'algorithm_simulator'):
                    params = {p.get('name'): p.get('value') for p in nodes[name].findall('param')}
                    self.assertEqual(float(params['command_lease_s']), lease)
                    if name == 'base_mppi':
                        self.assertEqual(float(params['execution_deadline_ms']), deadline)
                for name in ('base_mppi', 'remani_planner_node'):
                    flags = {p.get('name'): p.get('value') for p in nodes[name].findall('env')}
                    self.assertEqual(flags['NATIVE_DYNAMIC_FAST'], '1')
                    self.assertEqual(flags['NATIVE_REALTIME_DEMAND'], '1')
                manifest = json.loads((out / 'manifest.json').read_text())
                self.assertEqual(manifest['execution_timing_pair']['group'], group)
                self.assertEqual(manifest['execution_timing_pair']['callback_deadline_s'], deadline / 1000.)
                self.assertEqual(manifest['execution_timing_pair']['deadline_ms'], deadline)
                self.assertEqual(manifest['native_dynamic_fast']['native_and_execution_budgets_unchanged'], group == 'current')
                for name in ('local_execution.py', 'simulator.py'):
                    self.assertEqual(manifest['experiment_sources_sha256'][name], hashlib.sha256(copies[-1][name]).hexdigest())
                execution = copies[-1]['local_execution.py'].decode()
                self.assertIn('lib.online_set_execution_lease(node.handle,node.command_lease_s)', execution)
                self.assertIn('computation_ms>node.execution_deadline_ms', execution)
                self.assertIn('node.execution_deadline_ms-5.', execution)
                self.assertIn("'lease_s':node.command_lease_s", execution)
                self.assertIn("'validated_proposal':validated_proposal", execution)
                self.assertLess(execution.index("validated_proposal=result[:2].tolist() if reason=='ok' else None"),
                    execution.index("if computation_ms>node.execution_deadline_ms:reason='deadline_miss'"))
                self.assertNotIn('computation_ms>50', execution)
                self.assertNotIn('time.monotonic()-wall>.045', execution)
                self.assertIn("rospy.get_param('~command_lease_s'", copies[-1]['simulator.py'].decode())
            self.assertEqual(copies[0], copies[1])
        for name, data in original.items():
            self.assertEqual((HERE / name).read_bytes(), data)


if __name__ == '__main__':
    unittest.main()
