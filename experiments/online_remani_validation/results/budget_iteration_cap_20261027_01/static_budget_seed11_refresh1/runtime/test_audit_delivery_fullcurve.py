#!/usr/bin/env python3
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

import audit_delivery_fullcurve as a
from test_recheck_delivery_diag import frame,piece


class FullcurveTests(unittest.TestCase):
    def candidate(self,p=None):
        return dict(start_time=1000.,parts=[dict(gear=1,duration=1.,pieces=[p or piece(1.,.8,vx=.1)])])

    def test_global_min_not_first_failure(self):
        c=self.candidate(piece(1.,.4,vx=-.2))
        result=a.scan(c,a.snapshot_arrays(frame()),a.candidate_samples(c,'dense',.02),None)
        self.assertLess(result['worst_static']['gap'],result['first_static_failure']['gap'])
        self.assertAlmostEqual(result['worst_static']['t'],1.)
        self.assertIsNone(result['left_wheel_peak'])
        self.assertFalse(result['continuous_safety_proven'])

    def test_coarse_grid_can_miss_dip(self):
        p=piece(1.,.29+40*.0625**2,vx=-80*.0625)
        p['coeffs'][0][-3]=40.
        c=self.candidate(p);m=a.snapshot_arrays(frame())
        coarse=a.scan(c,m,a.quadrature_samples(c,8),None)
        fine=a.scan(c,m,a.candidate_samples(c,'dense',.005),None)
        self.assertGreater(coarse['min_static_gap_m'],0)
        self.assertLess(fine['min_static_gap_m'],0)

    def test_native_wheel_formula_turn_and_reverse(self):
        p=piece(1.,.8,vx=.2);p['coeffs'][1][-3]=.1
        c=self.candidate(p);s=next(a.candidate_samples(c,'dense',.02))
        params=dict(wheel_base_m=.4,wheel_radius_m=.1)
        left,right=a.wheel_values(c,s,params)
        self.assertAlmostEqual(left,0.)
        self.assertAlmostEqual(right,4.)
        c['parts'][0]['gear']=-1
        s=next(a.candidate_samples(c,'dense',.02))
        left,right=a.wheel_values(c,s,params)
        self.assertAlmostEqual(left,-4.)
        self.assertAlmostEqual(right,0.)

    def test_exact_snapshot_hash_and_path(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);raw=frame();raw['schema']='delivery_snapshot_v1'
            data=json.dumps(raw).encode();(root/'s.json').write_bytes(data)
            ref=dict(snapshot='s.json',snapshot_payload_hash=a.fnv64(data),hash_algorithm='fnv1a64_json')
            _,proof=a.frozen_frame(root,ref)
            self.assertTrue(proof['payload_hash_verified'])
            ref['snapshot_payload_hash']='0'*16
            with self.assertRaises(ValueError):a.frozen_frame(root,ref)
            ref['snapshot']='../s.json'
            with self.assertRaises(ValueError):a.frozen_frame(root,ref)

    def test_identity_rejects_scope_substitution(self):
        candidate=dict(clock=dict(pid=123),request_id=2,scope_id=3,accepted=True)
        rows=[dict(pid=123,request_id=2,event=e) for e in ('request_begin','request_end','state_contract_solver_return')]
        rows[1]['success']=True;rows[2]['success']=True
        rows.extend([dict(pid=123,request_id=2,event='state_contract_backend_fixed',state_contract_backend_id=1,
            state_contract_backend_id_valid=True,input_hash='55'),
            dict(pid=123,request_id=2,event='state_contract_backend_scope_link',state_contract_backend_id=1,scope_id=3,input_hash='55'),
            dict(pid=123,request_id=2,event='coupling_scope',scope_id=3,input_hash=55)])
        self.assertTrue(a.identity(candidate,Path('123_1.json'),rows)['verified'])
        candidate['scope_id']=4
        with self.assertRaises(ValueError):a.identity(candidate,Path('123_1.json'),rows)
        candidate['scope_id']=3
        with self.assertRaises(ValueError):a.identity(candidate,Path('124_1.json'),rows)


if __name__=='__main__':unittest.main()
