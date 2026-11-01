"""Check the settings, sensor/truth arena and goal agree before starting ROS."""
import ast
import importlib.util
import json
from pathlib import Path
import shutil
import tempfile
import unittest

import numpy as np
import yaml

from run_long_goal_crossing import configure_trial, register_scene, SCENE

HERE = Path(__file__).resolve().parent
PREVIOUS = HERE / 'results/sim_crop_bypass_20261029_01/head_on_bypass_seed11'


class LongGoalCrossingTests(unittest.TestCase):
    def test_only_declared_parameters_change_and_truth_wall_moves(self):
        register_scene()
        with tempfile.TemporaryDirectory(prefix='long_goal_crossing_') as directory:
            out = Path(directory)
            (out / 'runtime').mkdir()
            for name in ('manifest.json', 'parameters.yaml', 'limits.json', 'shared_collision_config.json'):
                shutil.copy2(PREVIOUS / name, out / name)
            for name in ('simulator.py', 'native_collision_runtime.py', 'dynamic_scenarios.py'):
                shutil.copy2(PREVIOUS / 'runtime' / name, out / 'runtime' / name)
            before = yaml.safe_load((out / 'parameters.yaml').read_text())
            before_limits = json.loads((out / 'limits.json').read_text())
            configure_trial(out)
            after = yaml.safe_load((out / 'parameters.yaml').read_text())
            changed = {key for key in before if before[key] != after[key]}
            self.assertEqual(changed, {
                '/remani_planner_node/mm/mobile_base_max_wheel_omega',
                '/remani_planner_node/dynamic_point_flow/max_prediction_time'})
            limits = json.loads((out / 'limits.json').read_text())
            self.assertEqual(list(limits), list(before_limits))
            self.assertAlmostEqual(limits['wheel_v'], .287)
            self.assertEqual(limits['wheel_a'], before_limits['wheel_a'])
            for source in (out / 'runtime').glob('*.py'):
                ast.parse(source.read_text())
            def load(name):
                spec = importlib.util.spec_from_file_location('new_' + name, out / 'runtime' / (name + '.py'))
                module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(module)
                return module
            scenarios = load('dynamic_scenarios')
            self.assertEqual(scenarios.actors_at(SCENE, 1.99), [])
            actor = scenarios.actors_at(SCENE, 2. + 1.6 / .18)[0]
            self.assertAlmostEqual(actor[0], 3.)
            self.assertAlmostEqual(actor[1], 0.)
            self.assertLess(scenarios.actors_at(SCENE, 2.)[0][1], -1.5)
            body = np.array([[0., 0., .3, .13, .18, 1., 0.]])
            native = load('native_collision_runtime')
            self.assertGreater(native.sampled_truth_clearance(np.array([6., 0., 0.]), body, .05, []), 1.)
            self.assertLess(native.sampled_truth_clearance(np.array([7.49, 0., 0.]), body, .05, []), 0.)
            self.assertGreater(scenarios.body_clearance(np.array([6., 0., 0.]), np.array([[0., 0., .08]]), []), 1.)
            source = (out / 'runtime/simulator.py').read_text()
            self.assertIn('goal.pose.position.x=6.;', source)
            self.assertIn('self.s[:2]-[6.,0.]', source)
            self.assertNotIn('4.5-p[0]', source)
            self.assertIn('for x in (-1.5,7.5) for y in ys', source)


if __name__ == '__main__':
    unittest.main()
