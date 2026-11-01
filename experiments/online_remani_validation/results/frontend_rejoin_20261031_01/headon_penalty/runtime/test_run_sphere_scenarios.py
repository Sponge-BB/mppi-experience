import json
from pathlib import Path
import tempfile
import unittest

import yaml

from run_sphere_scenarios import ACTORS, CASES, configure_comparison, jobs, register_scenes


class SphereScenarioTests(unittest.TestCase):
    def test_paired_order_and_no_goal_occupation(self):
        work = jobs(list(ACTORS))
        self.assertEqual(len(work), 10)
        self.assertEqual([r['group'] for r in work[:4]], ['legacy', 'b47', 'b47', 'legacy'])
        for scene, actors in ACTORS.items():
            for x, y, vx, vy, radius in actors:
                self.assertLess(x+radius, 6.)
                self.assertLess(abs(y)+radius, 2.)

    def test_five_scene_definitions_are_registered(self):
        register_scenes()
        import dynamic_scenarios as scenes
        for name, (_, _, actors) in CASES.items():
            self.assertEqual(len(scenes.scene_definition(name)['actors']), len(actors))

    def test_inherited_hardware_b_rows_cannot_leak_into_legacy(self):
        register_scenes()
        for group in ('legacy', 'b47'):
            with tempfile.TemporaryDirectory() as folder:
                out = Path(folder)
                runtime = out/'runtime'
                runtime.mkdir()
                (runtime/'dynamic_scenarios.py').write_text('CASES={}\nSCENES=()\nAPPEARANCE=2.\n')
                params = {'/remani_planner_node/mm/fixed_collision_spheres': [[0., 0., 0., 1.]],
                          '/mm_controller_node/mm/fixed_collision_spheres_enable': True,
                          '/remani_planner_node/mm/mobile_base_max_wheel_omega': 4.1}
                (out/'parameters.yaml').write_text(yaml.safe_dump(params))
                for name in ('limits.json', 'scenario.json', 'shared_collision_config.json'):
                    (out/name).write_text('{}')
                manifest = dict(experiment_sources_sha256={}, watchdog_off_experiment={},
                    long_goal_crossing_experiment=dict(unchanged=['native 3D collision geometry and soft costs']))
                (out/'manifest.json').write_text(json.dumps(manifest))
                configure_comparison(out, 'headon', group)
                updated = yaml.safe_load((out/'parameters.yaml').read_text())
                self.assertNotIn('/mm_controller_node/mm/fixed_collision_spheres_enable', updated)
                rows = updated.get('/remani_planner_node/mm/fixed_collision_spheres')
                self.assertEqual(None if rows is None else len(rows), None if group == 'legacy' else 47)
                self.assertEqual(updated['/remani_planner_node/mm/mobile_base_max_wheel_omega'], 4.1)
                scenario = json.loads((out/'scenario.json').read_text())
                self.assertEqual(scenario['scene'], 'sphere_timing_headon')


if __name__ == '__main__':
    unittest.main()
