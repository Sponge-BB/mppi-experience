#!/usr/bin/env python3
"""Small offline probe contracts; native full-coordinate tests run explicitly."""
import unittest

import numpy as np

import backend_component_probe as probe


class SourceContracts(unittest.TestCase):
    def test_attested_material_and_exact_dense_branch(self):
        sources,manifest=probe.source_material()
        generated=probe.cpp_probe(sources)
        integral=probe.extract(sources['poly_traj_optimizer.cpp'],
             'void PolyTrajOptimizer::addPVAJGradCost2CT(',True)
        self.assertIn(integral,generated)
        self.assertIn('cost_dense_log / K / dense_sample_resolution_',generated)
        self.assertEqual(8,len(manifest['sources_sha256']))

    def test_latest_context_coordinate_partition(self):
        trial=probe.BASE/'results/feasibility_stop_20261026_01/static_stop_seed11_refresh2'
        context=probe.Context.from_trial(trial)
        self.assertEqual(56,context.nx)
        self.assertEqual(20,len(context.free_indices))
        self.assertEqual(36,len(context.fixed_indices))
        self.assertEqual(set(range(context.nx)),set(context.free_indices)|set(context.fixed_indices))
        self.assertTrue(np.all(context.pieces==[2,5]))
        self.assertEqual(2,len(context.gears))

    def test_no_ros_dependency_in_probe_translation_unit(self):
        material,_=probe.source_material()
        generated=probe.cpp_probe(material)
        self.assertNotIn('#include <ros/',generated)
        self.assertNotIn('ros::',generated)
        self.assertIn('native_obstacle',probe.__doc__.replace('Native obstacle','native_obstacle'))


if __name__=='__main__':unittest.main()
