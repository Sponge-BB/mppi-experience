"""New online runner contracts; no ROS graph is launched."""
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import run_native_collision_3d as runner


class Runner(unittest.TestCase):
    def test_exact_unpaused_environment_restored(self):
        inherited = dict(DELIVERY_REPLAY_KIND='first_refresh', DELIVERY_REPLAY_WAIT='1',
            FEASIBILITY_STOP_GATE='1', BUDGET_OVERRIDE='3', SOLVER_SCALING_ENABLE='1', EXAMPLE='kept')
        with patch.dict(os.environ, inherited, clear=True):
            with runner.clean_online_environment(parity=True):
                self.assertEqual(os.environ['FEASIBILITY_STOP_GATE'], '0')
                self.assertEqual(os.environ['DELIVERY_REPLAY_KIND'], '')
                self.assertEqual(os.environ['NATIVE_COLLISION_PARITY'], '1')
                for key in ('DELIVERY_REPLAY_WAIT', 'BUDGET_OVERRIDE', 'SOLVER_SCALING_ENABLE'):
                    self.assertNotIn(key, os.environ)
            self.assertEqual(dict(os.environ), inherited)

    def test_parent_state_policy_shared_and_signed_enabled(self):
        values = runner.trial_kwargs(dict(state_contract_experiment=True, domain_recovery_experiment=True))
        self.assertTrue(values['signed_low_speed'])
        self.assertTrue(values['state_contract'])
        self.assertTrue(values['diagnostic_capture'])
        self.assertTrue(values['domain_recovery'])
        self.assertFalse(values['clearance_recovery'])

    def test_different_parent_binary_not_valid_common_export_control(self):
        with tempfile.TemporaryDirectory() as temp:
            out = Path(temp)/'never_created'
            with self.assertRaisesRegex(ValueError, 'SAME new build'):
                runner.run_batch(Path(temp)/'parent', Path(temp)/'new', out)
            self.assertFalse(out.exists())


if __name__ == '__main__':
    unittest.main()
