import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import yaml
import run_budget_iteration_cap as runner


class CapTests(unittest.TestCase):
    def test_followup_predeclared_two_cap_cases(self):
        self.assertEqual(tuple((j['seed'],j['ordinal']) for j in runner.JOBS),((11,1),(12,1)))
        self.assertEqual(runner.ITERATIONS,1200)

    def test_override_changes_only_solver_iterations(self):
        with tempfile.TemporaryDirectory(prefix='remani_cap_contract_') as tmp:
            out=Path(tmp)
            original={runner.KEY:300,'/remani_planner_node/mm/mobile_base_max_wheel_omega':3.28}
            def prepare(path,build,seed,ordinal):
                (path/'parameters.yaml').write_text(yaml.safe_dump(original))
                (path/'coupling_parameters.json').write_text('{}')
                (path/'manifest.json').write_text('{"parameter_overrides":{}}')
                return {'runtime.py':'frozen'}
            def resolved(*args,**kwargs):
                data=yaml.safe_load((out/'parameters.yaml').read_text())
                return SimpleNamespace(params={k:SimpleNamespace(value=v) for k,v in data.items()})
            fake=SimpleNamespace(config=SimpleNamespace(load_config_default=resolved))
            with patch.object(runner.base,'prepare',side_effect=prepare),patch.dict('sys.modules',{'roslaunch':fake}):
                self.assertEqual(runner.prepare(out,Path('/tmp/immutable'),11,1),{'runtime.py':'frozen'})
            changed=yaml.safe_load((out/'parameters.yaml').read_text())
            self.assertEqual(changed,{**original,runner.KEY:1200})
            manifest=json.loads((out/'manifest.json').read_text())
            self.assertEqual(manifest['parameter_overrides'],{runner.KEY:1200})
            self.assertTrue(manifest['coupling_experiment']['physical_constraints_unchanged'])


if __name__=='__main__':
    unittest.main()
