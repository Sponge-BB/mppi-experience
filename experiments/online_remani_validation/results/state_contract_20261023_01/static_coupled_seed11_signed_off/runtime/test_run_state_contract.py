"""Fail-closed family checks must run before graph/runtime preparation."""
import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch

import run_coupled_objective as runner


class StateRunnerGuards(unittest.TestCase):
    def rejects(self, manifest, **kwargs):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            with patch.object(runner,'check_space'), \
                    patch.object(runner,'check_build',return_value=(manifest,{})), \
                    patch.object(runner,'prepare_trial') as prepare, \
                    patch.dict(runner.os.environ,{},clear=True):
                with self.assertRaises(ValueError):
                    runner.run_trial(root/'never_created',root/'build','static','coupled',11,**kwargs)
                prepare.assert_not_called()
                self.assertFalse((root/'never_created').exists())

    def test_signed_rejected_on_old_family(self):
        self.rejects({},signed_low_speed=True,state_contract=True)

    def test_capture_rejected_on_old_family(self):
        self.rejects({},state_contract=True)

    def test_state_family_cannot_silently_disable_capture(self):
        self.rejects(dict(state_contract_experiment=True))

    def test_state_family_requires_diagnostics(self):
        self.rejects(dict(state_contract_experiment=True,domain_recovery_experiment=True),
                     state_contract=True,domain_recovery=True)

    def test_state_family_requires_domain_recovery(self):
        self.rejects(dict(state_contract_experiment=True,domain_recovery_experiment=True),
                     state_contract=True,diagnostic_capture=True)

    def test_state_family_refuses_al(self):
        self.rejects(dict(state_contract_experiment=True,domain_recovery_experiment=True,
                          clearance_recovery_experiment=True),
                     state_contract=True,diagnostic_capture=True,domain_recovery=True,
                     clearance_recovery=True)


if __name__=='__main__':unittest.main()
