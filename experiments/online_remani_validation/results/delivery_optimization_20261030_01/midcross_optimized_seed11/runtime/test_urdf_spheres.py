import itertools
from pathlib import Path
import unittest

import numpy as np

from report_urdf_spheres import original_base_spheres, statistics
from urdf_sphere_model import build_model


URDF = Path(__file__).resolve().parents[2]/'src/my_description/robot/mm.urdf'


class GeometryCandidateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.models = {name: build_model(URDF, profile=name)
                      for name in ('coarse', 'compact')}

    def test_counts_and_scope(self):
        self.assertEqual(len(original_base_spheres()), 140)
        for profile, expected in [('coarse', 47), ('compact', 85)]:
            model = self.models[profile]
            self.assertEqual(model['sphere_count'], expected)
            self.assertFalse(model['enabled_in_planner'])
            self.assertEqual(model['safety_margin_change'], 0.)
            self.assertNotIn('link1', model['included_links'])
            self.assertNotIn('link6', model['included_links'])

    def test_every_mesh_cell_has_analytic_volume_cover(self):
        for model in self.models.values():
            for sphere in model['spheres']:
                if sphere['coverage_type'] != 'aabb_cell_circumsphere':
                    continue
                bounds = sphere['coverage_cell_local']
                corners = np.array(list(itertools.product(*zip(bounds['min'], bounds['max']))))
                center = np.array(sphere['center_collision_local'])
                self.assertLessEqual(np.linalg.norm(corners-center, axis=1).max(),
                                     sphere['radius']+1e-12)

    def test_cells_tile_each_mesh_bbox(self):
        for model in self.models.values():
            for component in model['components']:
                if component['geometry_kind'] != 'mesh':
                    continue
                spheres = [s for s in model['spheres'] if s['link'] == component['link']]
                divisions = tuple(component['divisions'])
                self.assertEqual({tuple(s['cell_index']) for s in spheres},
                                 set(np.ndindex(divisions)))
                low = np.array(component['local_bounds']['min'])
                high = np.array(component['local_bounds']['max'])
                step = (high-low)/divisions
                for sphere in spheres:
                    cell = sphere['coverage_cell_local']
                    np.testing.assert_allclose(cell['min'], low+step*sphere['cell_index'], atol=1e-14)
                    np.testing.assert_allclose(cell['max'], low+step*(np.array(sphere['cell_index'])+1), atol=1e-14)

    def test_cylinder_volume_cover_not_unnecessarily_larger_bbox_ball(self):
        for model in self.models.values():
            for component in model['components']:
                if component['geometry_kind'] != 'cylinder':
                    continue
                sphere, = [s for s in model['spheres'] if s['link'] == component['link']]
                r, length = component['cylinder_radius'], component['cylinder_length']
                self.assertAlmostEqual(sphere['radius'], np.hypot(r, length/2))
                self.assertLess(sphere['radius'], np.sqrt(2*r*r+length*length/4))

    def test_transforms_and_ground_offset(self):
        model = self.models['compact']
        for component in model['components']:
            transform = np.array(component['base_link_T_collision'])
            for sphere in model['spheres']:
                if sphere['link'] != component['link']:
                    continue
                center = transform @ np.r_[sphere['center_collision_local'], 1.]
                np.testing.assert_allclose(sphere['center_base_link'], center[:3], atol=1e-12)
                np.testing.assert_allclose(np.array(sphere['center_ground'])-center[:3],
                                           [0., 0., .115], atol=1e-12)
        frame = model['frame_contract']
        self.assertAlmostEqual(frame['urdf_arm_base_height_ground'], .688)
        self.assertAlmostEqual(frame['urdf_minus_planner_arm_height_reference'], .008)

    def test_fallback_is_explicit_not_missing_part_skipped(self):
        component, = [c for c in self.models['compact']['components'] if c['link'] == 'arm_base']
        self.assertIsNotNone(component['fallback'])
        self.assertFalse(component['fallback']['production_path_modified'])
        self.assertTrue(Path(component['source_path']).is_file())
        self.assertEqual(len(component['source_sha256']), 64)

    def test_finer_profile_reduces_bbox_excess(self):
        stats = {name: statistics(model) for name, model in self.models.items()}
        for link in ('base_link', 'arm_support'):
            values = {name: next(c for c in row['components'] if c['link'] == link)
                      for name, row in stats.items()}
            self.assertLess(values['compact']['max_local_axis_bbox_excess_m'],
                            values['coarse']['max_local_axis_bbox_excess_m'])


if __name__ == '__main__':
    unittest.main()
