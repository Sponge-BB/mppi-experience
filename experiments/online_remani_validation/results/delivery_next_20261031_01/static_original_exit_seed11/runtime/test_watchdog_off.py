"""Disable only the stack watchdog; keep algorithm and timed-loop AST intact."""
import ast
import json
from pathlib import Path
import shutil
import tempfile
import unittest

from run_watchdog_off import BASELINE, check_same_settings, disable_watchdog


class WatchdogOffTests(unittest.TestCase):
    def test_no_watchdog_and_only_shutdown_hook_changes_control_source(self):
        with tempfile.TemporaryDirectory(prefix='watchdog_off_') as directory:
            out = Path(directory)
            (out / 'runtime').mkdir()
            for name in ('manifest.json', 'parameters.yaml', 'experiment.launch',
                         'limits.json', 'shared_collision_config.json', 'scenario.json', 'geometry.json'):
                text = (BASELINE / name).read_text().replace(str(BASELINE), str(out))
                (out / name).write_text(text)
            for name in ('diagnose_execution_timing.py', 'local_execution.py', 'simulator.py',
                         'mppi_node.py', 'native_reference.py', 'progress.py', 'classified_frame.py',
                         'dynamic_scenarios.py', 'shared_collision_node.py', 'native_collision_runtime.py',
                         'collab_seed_export.py', 'search_recording.py'):
                shutil.copy2(BASELINE / 'runtime' / name, out / 'runtime' / name)
            control_before = (out / 'runtime/local_execution.py').read_text()
            disable_watchdog(out)
            self.assertTrue(check_same_settings(out)['passed'])
            helper = (out / 'runtime/diagnose_execution_timing.py').read_text()
            self.assertNotIn('faulthandler', helper)
            self.assertNotIn('loop_stacks.log', helper)
            self.assertIn("self.mark('cycle_start')", helper)
            hook = '    rospy.on_shutdown(node.timing_probe.close) # exit-only flush, never a loop watchdog\n'
            after = (out / 'runtime/local_execution.py').read_text()
            self.assertEqual(ast.dump(ast.parse(after.replace(hook, ''))), ast.dump(ast.parse(control_before)))
            # No ROS / no real shutdown needed: directly exercise the passive
            # helper and its idempotent exit-only writer.
            namespace = {'__file__': str(out / 'runtime/diagnose_execution_timing.py')}
            exec(compile(helper, namespace['__file__'], 'exec'), namespace)
            probe = namespace['LoopProbe'](out, lambda: 1001.)
            probe.begin(1)
            probe.mark('example')
            probe.close()
            probe.close()
            saved = json.loads((out / 'loop_stages.json').read_text())
            self.assertFalse(saved['watchdog_enabled'])
            self.assertEqual([r[1] for r in saved['rows']], ['cycle_start', 'example'])
            self.assertFalse((out / 'loop_stacks.log').exists())


if __name__ == '__main__':
    unittest.main()
