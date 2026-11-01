#!/usr/bin/env python3
"""Recover the initial screening runner only if its start SHA256 matches.

Later runs archive themselves. The first screening runner was extended while
its already-loaded process ran; this inverse mechanical transform may archive
it only with an EXACT match to that run's recorded start digest.
"""
import hashlib
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent


def archive():
    targets=[HERE/'results/mppi_cusp_20261018'/name for name in ('focus_02','regression_01')]
    if all((out/'runner_snapshot.py').exists() and
           hashlib.sha256((out/'runner_snapshot.py').read_bytes()).hexdigest()==
           json.loads((out/'protocol.json').read_text())['script_sha256'] for out in targets):
        print('EXACT_START_RUNNERS_ALREADY_VERIFIED');return
    text=(HERE/'run_mppi_cusp_adjustment.py').read_text()
    text=text.replace('import shutil\n','')
    text=text.replace("            phase_objective=HERE/'builds/cusp_objective_01',\n            gated_phase=HERE/'builds/cusp_gated_01')",
                      "            phase_objective=HERE/'builds/cusp_objective_01')")
    text=text.replace("        self.bound=getattr(self.lib,'online_cusp_bound',None)\n        if self.bound is not None:\n            self.bound.argtypes=[ct.c_void_p];self.bound.restype=ct.c_int\n",'')
    text=text.replace("        self.gate_gap=getattr(self.lib,'online_cusp_gate_gap',None)\n        if self.gate_gap is not None:\n            self.gate_gap.argtypes=[ct.c_void_p];self.gate_gap.restype=ct.c_double\n",'')
    text=text.replace("        if self.bound is not None:answer['cusp_bound']=bool(self.bound(self.handle))\n",'')
    text=text.replace("        if self.gate_gap is not None:answer['cusp_estimated_gap_m']=float(self.gate_gap(self.handle))\n",'')
    text=text.replace("default=['baseline','gated_phase']","default=list(BUILDS)")
    text=text.replace("    shutil.copy2(__file__,a.out/'runner_snapshot.py')\n    shutil.copy2(HERE/'run_mppi_rejoin_diagnosis.py',a.out/'fixture_runner_snapshot.py')\n",'')
    digest=hashlib.sha256(text.encode()).hexdigest()
    for name in ('focus_02','regression_01'):
        out=HERE/'results/mppi_cusp_20261018'/name
        expected=json.loads((out/'protocol.json').read_text())['script_sha256']
        if digest!=expected:raise ValueError('inverse runner does not match recorded start SHA256: '+name)
        target=out/'runner_snapshot.py'
        if target.exists() and hashlib.sha256(target.read_bytes()).hexdigest()!=digest:
            raise ValueError('refuse overwriting other archived runner')
        if not target.exists():target.write_text(text)
        print('EXACT_START_RUNNER_ARCHIVED',name,digest)


if __name__=='__main__':archive()
