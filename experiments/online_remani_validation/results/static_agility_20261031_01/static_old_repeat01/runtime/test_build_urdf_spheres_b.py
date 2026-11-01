#!/usr/bin/env python3
"""Read-only recipe checks; no compilation and no ROS."""
import unittest
from pathlib import Path

import build_urdf_spheres_b as build


class RecipeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.recipe = build.recipes(build.DEFAULT_OUT)

    def test_rebuild_all_class_consumers(self):
        rows = self.recipe['compile_commands']
        self.assertEqual(len(rows), 10)
        actual = {Path(row['source']).name for row in rows}
        self.assertEqual(actual, {'mm_config.cpp', 'poly_traj_optimizer.cpp',
            'online_api.cpp', *build.SEARCH, *build.PLANNER})

    def test_child_header_precedes_frozen_model(self):
        expected = ['-I' + str(build.DEFAULT_OUT / 'source/include'),
                    '-I' + str(build.DEFAULT_OUT / 'source')]
        for row in self.recipe['compile_commands']:
            includes = [v for v in row['command'] if v.startswith('-I')]
            self.assertEqual(includes[:2], expected)

    def test_link_has_no_old_model_search_or_optimizer(self):
        link = self.recipe['link_commands']['remani_planner_node']
        for name in build.ARCHIVES:
            matches = [v for v in link if Path(v).name == name]
            self.assertEqual(matches, [str(build.DEFAULT_OUT / 'build' / name)])
        self.assertEqual({Path(v).name for v in link if v.endswith('.o')},
                         {name + '.o' for name in build.PLANNER})

    def test_only_environment_archive_reused(self):
        self.assertEqual(set(self.recipe['reused_link_archives_sha256']),
                         {str(build.FROZEN / 'build/libexp_env.a')})
        build.verify_recipe(self.recipe)


if __name__ == '__main__':
    unittest.main()
