import json
from pathlib import Path
import tempfile
import unittest

import yaml

from run_urdf_spheres_b import MODEL, PARAMETER, configure_spheres, sphere_rows
from validate_urdf_spheres_b import private_parameters


class CandidateBRunnerTests(unittest.TestCase):
    def test_selected_rows_are_ground_frame_and_individual_radius(self):
        model = json.loads(MODEL.read_text())
        rows = sphere_rows(model)
        self.assertEqual(len(rows), 47)
        self.assertEqual(rows[0], [*model['spheres'][0]['center_ground'], model['spheres'][0]['radius']])
        self.assertGreater(len({row[3] for row in rows}), 1)

    def test_does_not_accept_other_candidate(self):
        model = json.loads(MODEL.read_text())
        model['profile'] = 'compact'
        with self.assertRaises(ValueError):
            sphere_rows(model)

    def test_private_model_test_retains_actual_fixed_pose_and_margins(self):
        params = private_parameters()
        self.assertEqual(len(params['mm']['fixed_collision_spheres']), 47)
        self.assertEqual(params['mm']['manipulator_min_pos'], params['mm']['manipulator_max_pos'])
        self.assertEqual(params['optimization']['safe_margin'], .1)

    def test_trial_changes_only_geometry_parameter_not_existing_limits(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder)
            (out/'runtime').mkdir()
            params = {'/remani_planner_node/mm/mobile_base_max_wheel_omega': 4.1,
                      '/remani_planner_node/optimization/safe_margin': .1}
            (out/'parameters.yaml').write_text(yaml.safe_dump(params))
            manifest = dict(long_goal_crossing_experiment=dict(
                unchanged=['native 3D collision geometry and soft costs', 'candidate strategy']),
                watchdog_off_experiment={}, experiment_sources_sha256={})
            (out/'manifest.json').write_text(json.dumps(manifest))
            configure_spheres(out)
            updated = yaml.safe_load((out/'parameters.yaml').read_text())
            self.assertEqual(updated.pop(PARAMETER), sphere_rows(json.loads(MODEL.read_text())))
            self.assertEqual(updated, params)
            meta = json.loads((out/'manifest.json').read_text())['urdf_spheres_b']
            self.assertEqual(meta['expected_exported_total_spheres'], 67)
            self.assertTrue((out/'fixed_structure_spheres.json').is_file())


if __name__ == '__main__':
    unittest.main()
