"""Offline structural and compiled pure-Eigen tests; no ROS/model invocation."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

import domain_step_control as control


HERE = Path(__file__).resolve().parent
NATIVE = HERE.parents[1] / 'src/mm_remani_planner/traj_opt/include/optimizer/lbfgs.hpp'


class DomainStepTransformation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.native = NATIVE.read_text()

    def test_private_header_and_only_trial_evaluation_is_caught(self):
        patched = control.transform_lbfgs(self.native)
        self.assertIn('#ifndef DOMAIN_STEP_LBFGS_HPP', patched)
        self.assertIn('namespace domain_lbfgs\n{', patched)
        self.assertNotIn('namespace lbfgs\n{', patched)
        self.assertEqual(patched.count('catch (const domain_step::RejectedTrial &)'), 1)
        line_search, optimizer = patched.split('    inline int lbfgs_optimize(', 1)
        self.assertIn('catch (const domain_step::RejectedTrial &)', line_search)
        self.assertNotIn('catch (', optimizer)
        self.assertIn('        fx = cd.proc_evaluate(cd.instance, x, g);', optimizer)
        self.assertNotIn('catch (...)', patched)
        self.assertNotIn('catch (const std::exception', patched)

    def test_defaults_and_existing_limit_checks_remain_unchanged(self):
        patched = control.transform_lbfgs(self.native)
        def parameters(text):
            return text.split('    struct lbfgs_parameter_t', 1)[1].split('    };', 1)[0]
        self.assertEqual(parameters(patched), parameters(self.native))
        for marker in (
            'if (param.max_linesearch <= count)',
            'if (brackt && (nu - mu) < param.machine_prec * nu)',
            'if (stp < stpmin)', 'if (stp > stpmax)',
            'stp = 0.5 * (mu + nu);', 'stp *= 2.0;',
            'if (param.max_iterations != 0 && param.max_iterations <= k)',
        ):
            self.assertEqual(patched.count(marker), self.native.count(marker))
        self.assertEqual(patched.count('++count;'), 1)
        for code in ('INVALID_FUNCVAL', 'MAXIMUMLINESEARCH', 'WIDTHTOOSMALL',
                     'MINIMUMSTEP', 'MAXIMUMSTEP'):
            self.assertIn('return domain_failure(LBFGSERR_' + code + ');', patched)

    def test_every_native_marker_is_required_exactly_once(self):
        for name, marker, _ in control._PATCHES:
            with self.subTest(marker=name, mode='missing'):
                with self.assertRaisesRegex(ValueError, 'expected exactly one match'):
                    control.transform_lbfgs(self.native.replace(marker, '', 1))
            with self.subTest(marker=name, mode='duplicate'):
                with self.assertRaisesRegex(ValueError, 'expected exactly one match'):
                    control.transform_lbfgs(self.native + '\n' + marker)

    def test_repeated_transformation_and_non_text_input_are_rejected(self):
        with self.assertRaises(ValueError):
            control.transform_lbfgs(control.transform_lbfgs(self.native))
        with self.assertRaises(TypeError):
            control.transform_lbfgs(b'not a supplied header string')

    def test_compiled_standalone_contracts(self):
        compiler = shutil.which('c++')
        eigen = Path('/usr/include/eigen3')
        if not compiler or not (eigen / 'Eigen/Eigen').is_file():
            self.skipTest('standalone test requires c++ and /usr/include/eigen3')
        with tempfile.TemporaryDirectory(prefix='domain_step_control_') as directory:
            root = Path(directory)
            (root / 'frozen_native_lbfgs.hpp').write_text(self.native)
            (root / 'domain_lbfgs.hpp').write_text(control.transform_lbfgs(self.native))
            executable = root / 'test_domain_step_control'
            command = [compiler, '-std=c++17', '-O2', '-Wall', '-Wextra', '-pedantic',
                       '-I' + str(root), '-I' + str(HERE), '-I' + str(eigen),
                       str(HERE / 'test_domain_step_control.cpp'), '-o', str(executable)]
            compiled = subprocess.run(command, capture_output=True, text=True, timeout=60)
            self.assertEqual(compiled.returncode, 0, compiled.stdout + compiled.stderr)
            run = subprocess.run([str(executable)], capture_output=True, text=True, timeout=10)
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
            self.assertIn('DOMAIN_STEP_CONTROL_PASSED', run.stdout)


if __name__ == '__main__':
    unittest.main()
