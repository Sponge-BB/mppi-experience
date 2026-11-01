#!/usr/bin/env python3
"""Reversible source and actual MPPI-header tests; no simulator/robot I/O."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from patch_native_dynamic_fast import PARENT_SHA256, patch_sources, project_off

HERE = Path(__file__).resolve().parent
PARENT = HERE / 'builds/native_realtime_20261027_01/source'
FROZEN_ENV = HERE / 'builds/fixed_arm_01/source/mm_remani_planner/plan_env/include'


class NativeDynamicFastTests(unittest.TestCase):
    def setUp(self):
        self.sources = {name: (PARENT / name).read_text() for name in PARENT_SHA256}

    def test_exact_inverse_and_scope(self):
        changed = patch_sources(self.sources)
        self.assertEqual(set(changed), {'mppi_v12.hpp'})
        self.assertNotEqual(changed['mppi_v12.hpp'], self.sources['mppi_v12.hpp'])
        self.assertEqual(project_off(changed['mppi_v12.hpp'], 'mppi_v12.hpp'), self.sources['mppi_v12.hpp'])

    def test_drift_double_patch_and_wrong_set_fail(self):
        with self.assertRaises(ValueError):
            patch_sources({'mppi_v12.hpp': self.sources['mppi_v12.hpp'] + '\n'})
        with self.assertRaises(ValueError):
            patch_sources(patch_sources(self.sources))
        with self.assertRaises(ValueError):
            patch_sources({})

    def test_no_geometry_horizon_threshold_or_random_order_changes(self):
        old, new = self.sources['mppi_v12.hpp'], patch_sources(self.sources)['mppi_v12.hpp']
        for anchor in (
                'int horizon=std::max(1,std::min(40,int(std::floor(valid/c.rdt))));',
                'if(gap<0)m.feasible=false;',
                'if(gap<.04)m.conflict=std::min(m.conflict,(k+.5*(j+1))*c.rdt);',
                'if(nm.merit.min_gap<.20){d_active=true;d_clear=0;}',
                'for(double scale:{.7,.35,0.})add(following(scale));',
                'int count=std::max(8,24*19/horizon);',
                'for(auto&u:us){noise=.75*noise+V{.012*normal(rng),.09*normal(rng)};u=bound(u+noise,c);}',
                'auto mm=dynamicScore(s,sc,a,mean,future,true,sm.side);',
                'return warm.front();'):
            self.assertEqual(old.count(anchor), 1, anchor)
            self.assertEqual(new.count(anchor), 1, anchor)

    def test_actual_mppi_cpp(self):
        with tempfile.TemporaryDirectory(prefix='native_dynamic_fast_cpp_') as directory:
            out = Path(directory)
            # Only the declared copied header is transformed; native geometry,
            # all LegacyMPPI code and production CV declarations are unchanged.
            for name in ('online_kernel.hpp', 'native_collision_3d.hpp', 'native_realtime_probe.hpp'):
                shutil.copyfile(PARENT / name, out / name)
            (out / 'mppi_v12.hpp').write_text(patch_sources(self.sources)['mppi_v12.hpp'])
            shutil.copyfile(HERE / 'native_dynamic_fast_probe.hpp', out / 'native_dynamic_fast_probe.hpp')
            executable = out / 'test_dynamic_fast'
            command = ['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Wno-misleading-indentation',
                       '-I', str(out), '-I', str(FROZEN_ENV), '-I/opt/ros/noetic/include',
                       '-I/usr/include/eigen3', str(HERE / 'test_native_dynamic_fast.cpp'),
                       '-L/opt/ros/noetic/lib', '-Wl,-rpath,/opt/ros/noetic/lib', '-lrostime',
                       '-o', str(executable)]
            subprocess.run(command, check=True)
            output = subprocess.check_output([str(executable)], text=True)
            self.assertIn('5 full-plan fixtures passed', output)


if __name__ == '__main__':
    unittest.main()
