import pathlib
import subprocess
import tempfile
import unittest

from mppi_cusp_patch import transform_api,transform_kernel,transform_v12

HERE=pathlib.Path(__file__).resolve().parent
SOURCE=HERE/'builds/native_v12_07/source'


class CuspTests(unittest.TestCase):
    def test_cursor_actual_state_gates(self):
        with tempfile.TemporaryDirectory(prefix='cusp_unit_') as tmp:
            executable=pathlib.Path(tmp)/'cursor_test'
            subprocess.run(['/usr/bin/c++','-std=c++17','-O2','-I'+str(HERE),
                str(HERE/'test_mppi_cusp_cursor.cpp'),'-o',str(executable)],check=True)
            subprocess.run([str(executable)],check=True)

    def test_exact_api_safety_preserved(self):
        old=(SOURCE/'online_api.cpp').read_text();new=transform_api(old)
        for begin,end in (
            ('extern "C" int online_validate_execution','extern "C" int online_execution_candidates'),
            ('extern "C" int online_execution_candidates','extern "C" int online_accept_execution_controls')):
            self.assertEqual(old[old.index(begin):old.index(end)],new[new.index(begin):new.index(end)])
        self.assertEqual(new.count('h.cursor.update('),1)

    def test_physics_and_predictor_preserved(self):
        old=(SOURCE/'online_kernel.hpp').read_text();new=transform_kernel(old)
        for begin,end in (('V wheels(','struct Ref'),('std::vector<O> predict(','struct Constraint')):
            self.assertEqual(old[old.index(begin):old.index(end)],new[new.index(begin):new.index(end)])

    def test_full_native_reference_not_sliced(self):
        api=transform_api((SOURCE/'online_api.cpp').read_text())
        self.assertIn('for(int i=0;i<nr;++i)sc.path.push_back',api)
        self.assertNotIn('sc.path.erase',api)
        self.assertNotIn('sc.path.resize',api)

    def test_phase_and_objective_are_distinct(self):
        old=(SOURCE/'mppi_v12.hpp').read_text()
        phase=transform_v12(old,False);objective=transform_v12(old,True)
        self.assertIn('!finalRange(sc)',phase)
        self.assertIn('sc.path.back()',phase)
        self.assertNotIn('sc.path.back()',objective)
        self.assertIn('sc.path[rangeEnd(sc)]',objective)

    def test_patch_rejects_unknown_source(self):
        with self.assertRaises(ValueError):transform_kernel('not a known kernel')

    def test_gated_phase_does_not_force_blocked_cusp(self):
        api=transform_api((SOURCE/'online_api.cpp').read_text(),True)
        self.assertIn('const bool bind=cusp_free&&',api)
        self.assertIn('const bool near_cusp=association[6]<=.20&&association[8]<=.20;',api)
        self.assertEqual(api.count('h.cursor.update('),1)
        # If the association is inactive the complete native Scene is used.
        self.assertIn('if(bind){sc.directed=true;',api)
        self.assertIn('h.cusp_bound=false;h.m.reset()',api)


if __name__=='__main__':unittest.main()
