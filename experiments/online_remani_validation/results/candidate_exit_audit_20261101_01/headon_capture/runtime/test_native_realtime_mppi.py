#!/usr/bin/env python3
"""Pure-transform/probe tests; real-snapshot C ABI replay is a separate audit."""
import subprocess
import tempfile
import unittest
from pathlib import Path
from patch_native_realtime_mppi import PARENT_SHA256, patch_sources, project_off

HERE = Path(__file__).resolve().parent
PARENT = HERE / 'builds/native_collision_3d_20261027_04/source'


class NativeRealtimePatchTests(unittest.TestCase):
    def setUp(self):
        self.sources = {name: (PARENT / name).read_text() for name in PARENT_SHA256}

    def test_exact_inverse_all_three_sources(self):
        patched = patch_sources(self.sources)
        self.assertEqual(set(patched), set(self.sources))
        for name in self.sources:
            self.assertNotEqual(patched[name], self.sources[name])
            self.assertEqual(project_off(patched[name], name), self.sources[name])

    def test_rejects_parent_drift(self):
        self.sources['native_collision_3d.hpp'] += '\n'
        with self.assertRaises(ValueError):
            patch_sources(self.sources)

    def test_rejects_partial_input_and_double_patch(self):
        with self.assertRaises(ValueError):
            patch_sources({'online_kernel.hpp': self.sources['online_kernel.hpp']})
        with self.assertRaises(ValueError):
            patch_sources(patch_sources(self.sources))

    def test_one_policy_branch_only_and_no_geometry_acceleration(self):
        patched = patch_sources(self.sources)
        kernel = patched['online_kernel.hpp']
        self.assertEqual(kernel.count('!native_realtime::skipFreePath(native3d::enabled())'), 1)
        for anchor in (
            'const int samples=std::max(32,c.samples*c.horizon/horizon/proposal_divisor);',
            'if(lateral_needed){\n      auto saved=warm;V5MPPI::plan(s,z,sc);',
            'if(lateral_needed&&best>=0){auto seed=candidates[best];',
            'recovery_debug={};auto previous=warm;V original=V7MPPI::plan(s,z,sc);',
        ):
            self.assertEqual(self.sources['online_kernel.hpp'].count(anchor), 1)
            self.assertEqual(kernel.count(anchor), 1)
        self.assertIn('return native3d::query(s.x,s.y,s.yaw,t,dynamic,true).gap;', kernel)
        geometry = patched['native_collision_3d.hpp']
        for anchor in (
            'for(const auto&body:s.body){',
            'if(dynamic&&s.dynamic_enabled&&s.groups)for(const auto&group:*s.groups){',
            'dyn=std::min(dyn,group.tree.distance(local));',
            'const double error=std::max(0.,body.soft_radius-std::min(stat,dyn));',
        ):
            self.assertEqual(geometry.count(anchor), self.sources['native_collision_3d.hpp'].count(anchor))

    def test_probe_cpp(self):
        with tempfile.TemporaryDirectory(prefix='native-realtime-probe-') as folder:
            executable = Path(folder) / 'test_probe'
            subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-pedantic',
                '-I', str(HERE), str(HERE / 'test_native_realtime_mppi.cpp'), '-o', str(executable)], check=True)
            output = subprocess.check_output([str(executable)], text=True)
            self.assertIn('dual offline gate', output)


if __name__ == '__main__':
    unittest.main()
