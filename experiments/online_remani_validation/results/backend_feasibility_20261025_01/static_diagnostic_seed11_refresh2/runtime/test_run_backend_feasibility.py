import tempfile
import unittest
from pathlib import Path
from run_backend_feasibility import diagnostic_environment, JOBS


class EnvironmentTests(unittest.TestCase):
    def test_explicit_fixed_seed_and_no_inherited_diagnostics(self):
        source = dict(SEED_PARAMETERIZATION_FIX='0', DELIVERY_REPLAY_REQUEST_ID='99',
            COUPLING_EXEC_WEIGHT='8', FAILURE_OWNED_PID_FILE='/wrong', TOKEN='secret',
            ROS_MASTER_URI='http://unsafe:11311', ROS_PACKAGE_PATH='/known')
        env = diagnostic_environment(Path('/tmp/example_diagnostic'), 12, 2, source)
        self.assertEqual(env['SEED_PARAMETERIZATION_FIX'], '1')
        self.assertEqual(env['COUPLING_EXEC_WEIGHT'], '0')
        self.assertEqual(env['COUPLING_HAND_WEIGHT'], '0')
        self.assertEqual(env['DELIVERY_REPLAY_REFRESH_ORDINAL'], '2')
        self.assertEqual(env['ROS_MASTER_URI'], 'http://127.0.0.1:11441')
        self.assertNotIn('DELIVERY_REPLAY_REQUEST_ID', env)
        self.assertNotIn('TOKEN', env)
        self.assertEqual(env['BOUNDARY_ADMISSION'], '0')
        self.assertEqual(env['COUPLING_FEASIBILITY_RECOVERY'], '0')

    def test_predeclared_order_and_no_outcome_selection(self):
        self.assertEqual([(j['seed'], j['ordinal']) for j in JOBS], [(11,1), (12,2), (11,2), (12,1)])


if __name__ == '__main__':
    unittest.main()
