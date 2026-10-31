#!/usr/bin/env python3
"""Read-only production sphere-B wiring/rollback checks; never starts ROS."""
import hashlib
import json
from pathlib import Path
import re
import tarfile
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[2]
LAUNCH = ROOT / 'src/mm_remani/launch'
MODEL = ROOT / 'src/mm_remani_planner/mm_config'
CONFIG = 'src/mm_remani_planner/plan_manage/config/fixed_collision_spheres_b.yaml'
BACKUP = ROOT / 'backups/real_urdf_spheres_b_20261029_01'
ARG = 'use_urdf_collision_b'
ARG_VALUE = '$(arg ' + ARG + ')'
CONFIG_VALUE = '$(find remani_planner)/config/fixed_collision_spheres_b.yaml'


def function_bodies(source, name):
    """Extract bodies of named C++ methods, including overloads."""
    result = []
    for match in re.finditer(r'\b' + re.escape(name) + r'\s*\(', source):
        opening = source.find('{', match.end())
        semicolon = source.find(';', match.end())
        if opening < 0 or (semicolon >= 0 and semicolon < opening):
            continue
        level = 1
        closing = opening + 1
        while level and closing < len(source):
            level += (source[closing] == '{') - (source[closing] == '}')
            closing += 1
        result.append(source[opening:closing])
    return result


class RealSphereBTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.xml = {name: ET.parse(LAUNCH / name).getroot() for name in (
            'exp0_real.launch', 'run_planner_gazebo.launch',
            'piper_advanced_param.xml')}
        cls.model = (MODEL / 'src/mm_config.cpp').read_text()
        cls.header = (MODEL / 'include/mm_config/mm_config.hpp').read_text()
        cls.optimizer = (ROOT / 'src/mm_remani_planner/traj_opt/src/poly_traj_optimizer.cpp').read_text()

    def test_real_enable_and_two_include_hops(self):
        chain = (
            ('exp0_real.launch', 'true', 'run_planner_gazebo.launch'),
            ('run_planner_gazebo.launch', 'false', 'piper_advanced_param.xml'),
            ('piper_advanced_param.xml', 'false', None),
        )
        for name, default, child in chain:
            with self.subTest(launch=name):
                root = self.xml[name]
                declarations = root.findall("./arg[@name='%s']" % ARG)
                self.assertEqual(len(declarations), 1)
                self.assertEqual(declarations[0].get('default'), default)
                if child:
                    includes = [v for v in root.findall('./include')
                                if v.get('file', '').endswith('/' + child)]
                    self.assertEqual(len(includes), 1)
                    forwarded = includes[0].find("./arg[@name='%s']" % ARG)
                    self.assertIsNotNone(forwarded)
                    self.assertEqual(forwarded.get('value'), ARG_VALUE)

    def test_every_model_consumer_has_private_loader_and_override(self):
        root = self.xml['piper_advanced_param.xml']
        self.assertEqual(root.findall('./rosparam[@file="%s"]' % CONFIG_VALUE), [])
        for name in ('remani_planner_node', 'mm_controller_node', 'model_vis'):
            with self.subTest(node=name):
                node = root.find("./node[@name='%s']" % name)
                self.assertIsNotNone(node)
                loaders = [v for v in node.findall('./rosparam')
                           if v.get('file') == CONFIG_VALUE]
                self.assertEqual(len(loaders), 1)
                self.assertEqual(loaders[0].get('if'), ARG_VALUE)
                self.assertIsNone(loaders[0].get('ns'))
                params = node.findall("./param[@name='mm/fixed_collision_spheres_enable']")
                self.assertEqual(len(params), 1)
                self.assertEqual(params[0].get('value'), ARG_VALUE)
                self.assertEqual(params[0].get('type'), 'bool')
                self.assertIsNone(params[0].get('if'))
                self.assertIsNone(params[0].get('unless'))
                self.assertGreater(list(node).index(params[0]), list(node).index(loaders[0]))

    def test_yaml_is_exact_candidate_b_not_uniform_radius(self):
        actual = yaml.safe_load((ROOT / CONFIG).read_text())['mm']['fixed_collision_spheres']
        candidate = json.loads((ROOT / 'experiments/online_remani_validation/results/'
            'urdf_sphere_simplification_20261029_01/candidate_coarse.json').read_text())
        expected = [row['center_ground'] + [row['radius']] for row in candidate['spheres']]
        self.assertEqual(candidate['sphere_count'], 47)
        self.assertEqual(len(actual), 47)
        self.assertEqual(actual, expected)
        self.assertGreater(len({row[3] for row in actual}), 1)
        self.assertTrue(all(len(row) == 4 and row[3] > 0 for row in actual))

    def test_opt_out_defaults_to_legacy_and_opt_in_requires_geometry(self):
        self.assertRegex(self.model, r'bool\s+fixed_collision_spheres_enable\s*=\s*false')
        self.assertRegex(self.model, r'nh\.param\("mm/fixed_collision_spheres_enable",\s*fixed_collision_spheres_enable,\s*false\)')
        self.assertRegex(self.model, r'if\s*\(fixed_collision_spheres_enable\)\s*\{\s*if\s*\(!nh\.getParam\("mm/fixed_collision_spheres",\s*sphere_rows\)\)\s*throw')
        radius = function_bodies(self.header, 'getBaseSphereRadius')[0]
        self.assertIn('fixed_collision_spheres_.empty()', radius)
        self.assertIn('mobile_base_check_radius_', radius)
        self.assertIn('fixed_collision_spheres_.at(i)(3)', radius)

    def test_hard_checks_dynamic_and_self_use_per_sphere_radius(self):
        for method in ('checkCarObsCollision', 'checkCarPlanningObsCollision',
                       'checkCarDynamicObsCollision', 'checkCarPredictedDynamicObsCollision',
                       'checkCarManiCollision', 'getCarObstacleMinSurfaceDistance'):
            with self.subTest(method=method):
                bodies = function_bodies(self.model, 'MMConfig::' + method)
                self.assertTrue(bodies)
                for body in bodies:
                    self.assertIn('getBaseSphereRadius(', body)
                    self.assertNotIn('mobile_base_check_radius_', body)
        predicted = function_bodies(self.model, 'MMConfig::checkPredictedDynamicObstacleCollision')
        self.assertTrue(any('getBaseSphereRadius(' in body for body in predicted))

    def test_soft_cost_and_visuals_use_same_radius(self):
        body = function_bodies(self.optimizer, 'PolyTrajOptimizer::obstacleGradCostforMM')[0]
        self.assertIn('getBaseSphereRadius(i) + safe_margin_ - dist', body)
        self.assertIn('getBaseSphereRadius(m) + self_safe_margin_ - dist', body)
        self.assertNotIn('getBaseCheckRadius()', body)
        visual = function_bodies(self.model, 'MMConfig::visCarCheckBall')[0]
        self.assertIn('2.0 * getBaseSphereRadius(i)', visual)

    def test_rollback_archive_covers_sources_runtime_and_objects(self):
        record = json.loads((BACKUP / 'before.json').read_text())
        self.assertEqual(record['workspace'], str(ROOT))
        self.assertEqual(len(record['source_files']), 7)
        self.assertEqual(len(record['runtime_files']), 11)
        self.assertEqual(len(record['object_dirs']), 9)
        self.assertEqual(record['new_files'], [CONFIG])
        expected_sources = {
            'src/mm_remani/launch/' + name for name in self.xml
        } | {
            'src/mm_remani_planner/mm_config/src/mm_config.cpp',
            'src/mm_remani_planner/mm_config/include/mm_config/mm_config.hpp',
            'src/mm_remani_planner/traj_opt/src/poly_traj_optimizer.cpp',
            'src/mm_remani_planner/plan_manage/src/remani_replan_fsm.cpp',
        }
        self.assertEqual(set(record['source_files']), expected_sources)
        archive = BACKUP / 'before.tar.gz'
        digest = hashlib.sha256()
        with archive.open('rb') as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(block)
        self.assertEqual(digest.hexdigest(), record['archive_sha256'])
        with tarfile.open(archive, 'r:gz') as stream:
            members = {row.name.rstrip('/') for row in stream.getmembers()}
        inventory = set(record['files_sha256'])
        self.assertTrue(inventory.issubset(members))
        for path in record['source_files'] + record['runtime_files']:
            self.assertIn(path, inventory)
        for directory in record['object_dirs']:
            self.assertIn(directory, members)
            self.assertTrue(any(name.startswith(directory + '/') and name.endswith('.o')
                                for name in inventory))


if __name__ == '__main__':
    unittest.main()
