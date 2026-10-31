#!/usr/bin/env python3
import copy
import math
from pathlib import Path
import unittest

import numpy as np

import audit_backend_kinematics as a
from audit_delivery_iterates import canonical_candidate,loads_preserving_cpp_zero,vector_hash


PARAMS=dict(wheel_base_m=.4,wheel_radius_m=.1,wheel_speed_limit_rad_s=3.,wheel_acceleration_limit_rad_s2=4.)


def candidate(vx=.1,vy=0.,ax=0.,ay=0.,jx=0.,jy=0.,duration=.2,gear=1):
    coefficients=np.zeros((2,8))
    coefficients[:,6]=[vx,vy];coefficients[:,5]=np.asarray([ax,ay])/2.
    coefficients[:,4]=np.asarray([jx,jy])/6.
    return dict(start_time=100.,parts=[dict(gear=gear,duration=duration,
        pieces=[dict(T=duration,basis='descending',coeffs=coefficients.tolist())])])


def canonical(c):
    return dict(stage='initial',iteration=0,reconstructed=True,x=[1.,-0.],g=[2.,3.],
        native_IsTrajSafe_pass=False,native_reason='left_wheel_speed',solver_elapsed_s=.01,
        canonical_parts=[dict(gear=p['gear'],times=[s['T'] for s in p['pieces']],
          b_rows_ascending_power=np.concatenate([np.asarray(s['coeffs']).T[::-1,:] for s in p['pieces']]).tolist()) for p in c['parts']])


class BackendKinematicsTests(unittest.TestCase):
    def test_exact_native_formula_attestation_and_drift(self):
        path=Path(__file__).resolve().parent/'builds/delivery_stability_20261024_02/source/poly_traj_optimizer.cpp'
        text=path.read_text();self.assertTrue(a.native_formula_evidence(text)['known'])
        self.assertFalse(a.native_formula_evidence(text.replace('double feas_tol_percent_ = 0.05;',
            'double feas_tol_percent_ = 0.06;',1))['known'])

    def test_loaded_scalar_parameters_strict(self):
        self.assertEqual(a.loaded_values('/x: 1.2\n/y: 3.4\n',['/x','/y']),{'/x':1.2,'/y':3.4})
        for text in ('/x: nan\n','/x: -1\n','/x: 1\n/x: 2\n','/x: [1]\n','/y: 1\n'):
            with self.assertRaises(ValueError):a.loaded_values(text,['/x'])

    def test_straight_forward_reverse(self):
        for gear in (-1,1):
            values=a.wheel_values(candidate(vx=.14,gear=gear),0.,PARAMS)
            self.assertAlmostEqual(values['left_speed'],gear*1.4)
            self.assertAlmostEqual(values['right_speed'],gear*1.4)
            self.assertEqual(values['left_acceleration'],0.)
            self.assertEqual(values['right_acceleration'],0.)

    def test_rotation_and_angular_acceleration_exact(self):
        values=a.wheel_values(candidate(vx=1.,ay=2.,jy=3.),0.,PARAMS)
        self.assertEqual(values['omega'],2.);self.assertEqual(values['alpha'],3.)
        self.assertAlmostEqual(values['left_speed'],6.)
        self.assertAlmostEqual(values['right_speed'],14.)
        self.assertAlmostEqual(values['left_acceleration'],-6.)
        self.assertAlmostEqual(values['right_acceleration'],6.)

    def test_mirror_swaps_both_wheel_channels(self):
        original=a.wheel_values(candidate(vx=.13,vy=.05,ax=.1,ay=.3,jx=.2,jy=-.5),.03,PARAMS)
        mirrored=a.wheel_values(candidate(vx=.13,vy=-.05,ax=.1,ay=-.3,jx=.2,jy=.5),.03,PARAMS)
        for suffix in ('speed','acceleration'):
            self.assertAlmostEqual(original['left_'+suffix],mirrored['right_'+suffix])
            self.assertAlmostEqual(original['right_'+suffix],mirrored['left_'+suffix])

    def test_native_singularity_floors(self):
        values=a.wheel_values(candidate(vx=0.,vy=0.,ax=.2),0.,PARAMS)
        self.assertAlmostEqual(values['left_speed'],.001)
        self.assertAlmostEqual(values['right_speed'],.001)
        self.assertEqual(values['left_acceleration'],0.)
        self.assertEqual(values['right_acceleration'],0.)

    def test_all_channels_scan_not_first_failure(self):
        c=candidate(vx=1.,ax=1.)
        scan=a.scan_wheels(c,[0.,.01,.02],PARAMS)
        for channel in a.CHANNELS:
            self.assertEqual(scan['violating_samples'][channel],3)
            self.assertIsNotNone(scan['peaks'][channel]);self.assertIsNotNone(scan['first_violation'][channel])
        self.assertFalse(scan['all_wheels_sampled_pass'])
        self.assertTrue(scan['does_not_early_exit_at_native_first_failure'])

    def test_native_grid_exact_skip_and_endpoint(self):
        times=list(a.native_times(.1));self.assertEqual(len(times),5)
        self.assertEqual(times[0],.05)
        self.assertLess(times[-1],.1)
        expected=.05
        for value in times:self.assertEqual(value,expected);expected+=.01
        self.assertEqual(list(a.native_times(.05)),[])
        self.assertIsNone(a.scan_wheels(candidate(duration=.05),[],PARAMS)['all_wheels_sampled_pass'])

    def test_skip_hides_early_violation_only_in_native_grid(self):
        # p'(t)=.4*(1-t/.08)^6 is high initially, low on .05/.06/.07.
        c=candidate(duration=.08);coefficients=np.zeros((2,8))
        for power in range(7):coefficients[0,7-(power+1)]=.4*math.comb(6,power)*(-1./.08)**power/(power+1)
        c['parts'][0]['pieces'][0]['coeffs']=coefficients.tolist()
        params=dict(PARAMS,wheel_speed_limit_rad_s=.1,wheel_acceleration_limit_rad_s2=1000.)
        native=a.scan_wheels(c,a.native_times(.08),params)
        diagnostic=a.scan_wheels(c,a.diagnostic_times(c),params)
        self.assertTrue(native['all_wheels_sampled_pass'])
        self.assertFalse(diagnostic['all_wheels_sampled_pass'])
        self.assertEqual(diagnostic['first_violation']['left_speed']['t'],0.)

    def test_part_and_piece_equal_boundary_uses_preceding(self):
        c=candidate(duration=.1,gear=-1)
        next_part=copy.deepcopy(c['parts'][0]);next_part['gear']=1;c['parts'].append(next_part)
        self.assertEqual(a.locate(c,.1),(0,0,.1))
        self.assertEqual(a.wheel_values(c,.1,PARAMS)['gear'],-1)
        self.assertEqual(a.wheel_values(c,.10000001,PARAMS)['gear'],1)
        self.assertIn(.1,a.diagnostic_times(c));self.assertIn(.2,a.diagnostic_times(c))
        right=float(np.nextafter(.1,math.inf))
        self.assertIn(right,a.diagnostic_times(c));self.assertEqual(a.wheel_values(c,right,PARAMS)['gear'],1)

    def test_canonical_signed_zero_and_initial_scope(self):
        parsed=loads_preserving_cpp_zero('{"x":[-0,0],"request_id":9007199254740993}')
        self.assertEqual(parsed['request_id'],9007199254740993)
        self.assertLess(math.copysign(1.,parsed['x'][0]),0.)
        self.assertNotEqual(vector_hash(parsed['x']),vector_hash([0.,0.]))
        record=canonical(candidate(vx=.1));prior=dict(stage='initial',iteration=0,
            candidate_x_hash=vector_hash(record['x']),candidate_evidence_valid=True,
            frozen_projected_pass=True,frozen_projected_20ms={'static_all_sampled_clear':True},both_pass=False)
        result=a.analyze_candidate(record,dict(frame=dict(now=100.)),prior,PARAMS)
        self.assertTrue(result['evidence_valid']);self.assertIn('not A* polyline',result['k0_scope'])
        self.assertEqual(result['frozen_projected_20ms'],prior['frozen_projected_20ms'])
        self.assertFalse(result['both_pass'])
        self.assertEqual(result['initial_k0_values']['velocity'],[.1,0.])

    def test_candidate_hash_mismatch_rejected(self):
        record=canonical(candidate());prior=dict(stage='initial',iteration=0,candidate_x_hash='bad')
        with self.assertRaises(ValueError):a.analyze_candidate(record,dict(frame=dict(now=0.)),prior,PARAMS)

    def test_native_pass_wheel_disagreement_invalidates_evidence(self):
        record=canonical(candidate(vx=1.));record['native_IsTrajSafe_pass']=True
        prior=dict(stage='initial',iteration=0,candidate_x_hash=vector_hash(record['x']),candidate_evidence_valid=True)
        result=a.analyze_candidate(record,dict(frame=dict(now=0.)),prior,PARAMS)
        self.assertFalse(result['evidence_valid'])
        self.assertFalse(result['native_pass_implies_sampled_wheel_pass'])

    def test_nonfinite_or_invalid_durations_not_accepted(self):
        c=candidate();c['parts'][0]['pieces'][0]['coeffs'][0][6]=float('nan')
        with self.assertRaises(ValueError):a.wheel_values(c,0.,PARAMS)
        with self.assertRaises(ValueError):list(a.native_times(float('nan')))
        with self.assertRaises(ValueError):list(a.native_times(-1.))
        record=canonical(candidate());record['canonical_parts'][0]['times']=[-1.]
        with self.assertRaises(ValueError):canonical_candidate(record,0.)


if __name__=='__main__':unittest.main()
