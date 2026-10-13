#!/usr/bin/env python3
"""Check physical bounds and reporting integrity, NOT whether an algorithm wins."""
import csv
import hashlib
import json
from pathlib import Path
import sys
import numpy as np


def verify(out):
    limits = json.loads((out/'limits.json').read_text())
    manifest = json.loads((out/'manifest.json').read_text())
    rows = list(csv.DictReader((out/'runs.csv').open()))
    refs = list(csv.DictReader((out/'references.csv').open()))
    assert rows
    dt = manifest['control_dt']
    for row in rows:
        stem = '__'.join(row[k] for k in ('scenario', 'profile', 'mode', 'seed'))
        d = np.atleast_1d(np.genfromtxt(out/(stem+'.csv'), delimiter=',', names=True))
        for key in d.dtype.names:
            assert np.isfinite(d[key]).all(), (stem, key, 'nonfinite')
        assert len(d) == int(row['cycles'])
        assert np.allclose(np.diff(np.r_[0, d['t']]), dt, atol=1e-7)
        for vkey, wkey in (('v','w'), ('nominal_v','nominal_w'), ('filtered_v','filtered_w'), ('command_v','command_w')):
            v, w = d[vkey], d[wkey]
            assert np.max(abs(v)) <= limits['vmax']+1e-8, (stem, 'linear speed')
            assert np.max(abs(w)) <= limits['wmax']+1e-8, (stem, 'angular speed')
            for sign in (-1, 1):
                wheel = v+sign*.5*limits['track']*w
                assert np.max(abs(wheel)) <= limits['wheel_v']+1e-7, (stem, 'wheel velocity')
                if vkey == 'v':
                    assert np.max(abs(np.diff(np.r_[0, wheel])))/dt <= limits['wheel_a']+1e-6, (stem, 'wheel acceleration')
        assert np.max(abs(np.diff(np.r_[0,d['v']])))/dt <= limits['accel']+1e-6
        assert np.max(abs(np.diff(np.r_[0,d['w']])))/dt <= limits['angular_accel']+1e-6
        assert abs(float(row['min_clearance'])-d['clearance'].min()) < 1e-5
        assert int(row['collision']) == int(np.any(d['clearance']<0))
        assert abs(float(row['elapsed'])-d['t'][-1]) < 1e-6
        for rkey, key in (('qp_failed','qp_failed'),('unverified','unverified'),('guard_cycles','guard'),
                          ('stale_cycles','stale'),('deadline_misses','deadline_miss'),('interventions','intervention'),('tracker_faults','tracker_fault')):
            assert int(row[rkey]) == int(d[key].sum()), (stem, rkey)
        assert np.allclose(d['total_ms'],d['controller_ms']+d['cbf_ms']+d['guard_ms'],atol=1e-7)
        assert np.array_equal(d['deadline_miss']>0, d['total_ms']>dt*1000)
        failed = (d['guard']>0)|(d['stale']>0)|(d['deadline_miss']>0)|(d['qp_failed']>0)
        assert np.all(abs(d['command_v'][failed])<1e-9) and np.all(abs(d['command_w'][failed])<1e-9)
        if int(row['reached']):
            goal = [r for r in refs if r['scenario']==row['scenario']][-1]
            assert np.hypot(d['x'][-1]-float(goal['x']), d['y'][-1]-float(goal['y'])) < limits['goal_pos']+1e-7
            yaw = d['yaw'][-1]-float(goal['yaw'])
            assert abs(np.arctan2(np.sin(yaw),np.cos(yaw))) < limits['goal_yaw']+1e-7
            assert abs(d['v'][-1]) < limits['goal_speed']+1e-8 and abs(d['w'][-1]) < .05+1e-8
    grouped = {}
    for row in rows:
        key = (row['scenario'],row['profile'],row['seed'])
        grouped.setdefault(key,set()).add(row['mode'])
    assert all(modes=={'tracker','mppi','mppi_cbf'} for modes in grouped.values())
    assert len(rows)==len(grouped)*3
    g = json.loads((out/'geometry.json').read_text());body=np.asarray(g['body_disks_xy_r'])
    for x in np.linspace(-g['base_length']/2,g['base_length']/2,51):
        for y in np.linspace(-g['base_width']/2,g['base_width']/2,51):
            assert np.any(np.linalg.norm(body[:2,:2]-[x,y],axis=1)<=body[:2,2]+1e-9)
    points=np.asarray(g['arm_collision_points_xyz'])
    assert np.all(np.linalg.norm(points[:,:2]-body[2,:2],axis=1)+g['manipulator_thickness']<=body[2,2]+1e-9)
    root=Path(__file__).resolve().parents[2]
    for relative,digest in manifest['sources_sha256'].items():
        if relative.startswith('src/'):
            assert hashlib.sha256((root/relative).read_bytes()).hexdigest()==digest, relative+' changed'
    message=f'PASS: {len(rows)} traces; complete 3-way groups, finite values, body/wheel velocity and acceleration, goal/collision/fallback/timing consistency, geometry coverage, unchanged production files'
    (out/'verification.txt').write_text(message+'\n')
    print(message,flush=True)


if __name__=='__main__':
    verify(Path(sys.argv[1]).resolve())
