#!/usr/bin/env python3
"""Validate generated artifacts and physical constraints, not navigation success."""
import csv
import hashlib
import json
from pathlib import Path
import sys
import numpy as np


def check(out):
    with (out/'runs.csv').open() as f:rows=list(csv.DictReader(f))
    assert rows, 'no runs'
    for r in rows:
        name=f"{r['scenario']}__{r['mode']}__{r['seed']}"
        d=np.atleast_1d(np.genfromtxt(out/(name+'.csv'),delimiter=',',names=True))
        assert len(d)==int(r['cycles']),name+' cycle count'
        for key in d.dtype.names:assert np.isfinite(d[key]).all(),name+' nonfinite '+key
        assert np.allclose(np.diff(np.r_[0.,d['t']]),.05,atol=1e-7),name+' timestamps'
        assert abs(float(r['elapsed'])-d['t'][-1])<1e-6,name+' elapsed'
        assert np.max(abs(d['v']))<=.30+1e-8 and np.max(abs(d['w']))<=.80+1e-8,name+' velocity'
        assert np.max(abs(np.diff(np.r_[0.,d['v']])))/.05<=.35+1e-6,name+' acceleration'
        assert np.max(abs(np.diff(np.r_[0.,d['w']])))/.05<=1.+1e-6,name+' angular acceleration'
        assert abs(float(r['min_clearance'])-d['clearance'].min())<1e-5,name+' min clearance'
        assert int(r['collision'])==int(np.any(d['clearance']<0)),name+' collision reporting'
        assert int(r['qp_infeasible'])==int(d['infeasible'].sum()),name+' QP reporting'
        assert int(r['deadline_misses'])==int((d['total_ms']>50).sum()),name+' deadline reporting'
        assert np.allclose(d['total_ms'],d['mppi_ms']+d['cbf_ms'],atol=1e-5),name+' timing accounting'
        if int(r['reached']):
            assert np.hypot(d['x'][-1]-3.,d['y'][-1])<.1+1e-8 and abs(d['v'][-1])<.04+1e-8,name+' goal'
        failed=(d['stale']>0)|(d['deadline_miss']>0)|(d['infeasible']>0)
        assert np.all(abs(d['command_v'][failed])<1e-9) and np.all(abs(d['command_w'][failed])<1e-9),name+' fallback'
    g=json.loads((out/'geometry.json').read_text());body=np.array(g['body_disks_xy_r'])
    for x in np.linspace(-g['base_length']/2,g['base_length']/2,51):
        for y in np.linspace(-g['base_width']/2,g['base_width']/2,51):
            assert np.any(np.linalg.norm(body[:2,:2]-[x,y],axis=1)<=body[:2,2]+1e-9),'base coverage'
    points=np.asarray(g['arm_collision_points_xyz'])
    assert np.all(np.linalg.norm(points[:,:2]-body[2,:2],axis=1)+.034<=body[2,2]+1e-9),'arm coverage'
    manifest=json.loads((out/'manifest.json').read_text())
    workspace=Path(__file__).resolve().parents[2]
    for relative,expected in manifest['sources_sha256'].items():
        if relative.startswith('src/'):
            assert hashlib.sha256((workspace/relative).read_bytes()).hexdigest()==expected,'production configuration changed: '+relative
    print(f'PASS: {len(rows)} traces; finite values, timing, limits, collision/goal/failure reporting, geometry and unchanged production configs')


if __name__=='__main__':check(Path(sys.argv[1]).resolve())
