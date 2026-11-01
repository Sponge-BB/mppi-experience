import json
import tempfile
from pathlib import Path
import unittest

from run_boundary_admission import validate_direction_proof
from run_coupled_objective import sha


class DirectionPrerequisite(unittest.TestCase):
    def fixture(self,root):
        base=root/'base';base.mkdir();(base/'build_manifest.json').write_text('{"frozen":true}')
        trials=[]
        for name in ('off','on'):
            path=root/name;path.mkdir()
            manifest=dict(build=str(base),build_manifest_sha256=sha(base/'build_manifest.json'))
            (path/'manifest.json').write_text(json.dumps(manifest))
            trials.append(dict(source=str(path),manifest_sha256=sha(path/'manifest.json')))
        proof=dict(schema='directed_low_speed_verdict_v1',verified_signed_negative_native_chain=True,
                   same_fixture=True,same_runtime=True,trials=trials)
        output=root/'verification.json';output.write_text(json.dumps(proof))
        return base,output,proof

    def test_valid_prerequisite(self):
        with tempfile.TemporaryDirectory() as tmp:
            base,path,_=self.fixture(Path(tmp));validate_direction_proof(path,base)

    def test_unobserved_direction_or_mixed_runtime_rejected(self):
        for field in ('verified_signed_negative_native_chain','same_fixture','same_runtime'):
            with tempfile.TemporaryDirectory() as tmp:
                base,path,proof=self.fixture(Path(tmp));proof[field]=False;path.write_text(json.dumps(proof))
                with self.assertRaises(ValueError):validate_direction_proof(path,base)

    def test_changed_proof_manifest_or_wrong_build_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            base,path,proof=self.fixture(Path(tmp))
            source=Path(proof['trials'][0]['source'])/'manifest.json';source.write_text('{}')
            with self.assertRaises(ValueError):validate_direction_proof(path,base)
        with tempfile.TemporaryDirectory() as tmp:
            base,path,_=self.fixture(Path(tmp))
            with self.assertRaises(ValueError):validate_direction_proof(path,base.parent/'different')


if __name__=='__main__':unittest.main()
