#!/usr/bin/env python3
import copy
import ctypes
import json
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

import audit_backend_wheel_gradient as a
from audit_delivery_iterates import loads_preserving_cpp_zero, vector_hash


SOURCE=a.BASE/'builds/seed_parameterization_20261025_01/source/poly_traj_optimizer.cpp'
LIBRARY=a.BASE/'results/wheel_gradient_20261026_01/wheel_gradient_probe.so'
PARAMS=[100.,.2296,.05,.37,.07,3.28,4.25]


def polynomial():
    # x=.15t+.02t^2+.01t^3, y=.03t+.02t^2+.005t^3
    c=np.zeros((2,8));c[:,6]=[.15,.03];c[:,5]=[.02,.02];c[:,4]=[.01,.005]
    return dict(T=.5,coeffs=c.tolist())


class TestWheelGradientAudit(unittest.TestCase):
    def test_native_function_extraction_is_exact_not_transcription(self):
        text=SOURCE.read_text();wheel,smooth,contract=a.source_contract(text)
        self.assertIn(wheel,text);self.assertIn(smooth,text)
        self.assertIn(wheel,a.cpp_probe(text));self.assertIn(smooth,a.cpp_probe(text))
        self.assertEqual(contract['B'],[[0.,-1.],[1.,0.]])
        self.assertTrue(contract['callers_zero_gradients_before_each_call'])
        self.assertIn('cost_mm_feasible = 0;',wheel)

    def test_unknown_contract_or_duplicate_function_fail_closed(self):
        text=SOURCE.read_text()
        with self.assertRaises(ValueError):a.source_contract(text.replace('B_h_ << 0.0, -1.0,','B_h_ << 0.0, 1.0,'))
        with self.assertRaises(ValueError):a.function(text+'\nbool PolyTrajOptimizer::smoothedL1(', 'smoothedL1')
        with self.assertRaises(ValueError):a.function('bool PolyTrajOptimizer::foo(){','foo')

    def test_candidate_selector_is_deterministic_and_deduplicated(self):
        def entry(stage,ratio):
            return dict(stage=stage,diagnostic_10ms_with_boundaries=dict(peaks={k:dict(ratio_to_final_limit=ratio) for k in a.CHANNELS}))
        self.assertEqual(a.select_candidate_indices([]),[])
        self.assertEqual(a.select_candidate_indices([entry('initial',4)]),[0])
        self.assertEqual(a.select_candidate_indices([entry('initial',9),entry('accepted',3),entry('accepted',2),entry('accepted',1)]),[0,1,3])

    def test_native_primary_and_dense_nodes_match_domains(self):
        piece=polynomial();nodes=list(a.quadrature_nodes(piece,16,16))
        primary=[t for t,dense in nodes if not dense]
        self.assertEqual(len(primary),17);self.assertEqual(primary[0],0.)
        self.assertAlmostEqual(primary[-1],piece['T'])
        self.assertFalse(any(dense for t,dense in nodes))
        low=copy.deepcopy(piece);low['coeffs']=np.zeros((2,8)).tolist()
        nodes=list(a.quadrature_nodes(low,16,16));dense=[t for t,d in nodes if d]
        self.assertEqual(len(dense),2*9+15*17)
        self.assertGreaterEqual(min(dense),0.);self.assertLessEqual(max(dense),low['T']+1e-15)

    def test_canonical_negative_zero_and_large_identity_unchanged(self):
        row=loads_preserving_cpp_zero('{"x":[-0,0],"identity":9007199254740993}')
        self.assertEqual(row['identity'],9007199254740993)
        self.assertEqual(math.copysign(1.,row['x'][0]),-1.)
        self.assertNotEqual(vector_hash(row['x']),vector_hash([0.,0.]))

    def test_fixed_protocol_has_no_best_step_selection(self):
        p=a.protocol();self.assertEqual(p['primary_step_index'],1)
        self.assertEqual(p['steps'],list(a.STEPS));self.assertTrue(p['no_solver_runs'])

    def test_five_point_constant_large_cost_has_exact_zero_derivative(self):
        self.assertEqual(a.five_point([1.5274449778607676e16]*4,6e-8),0.)

    @unittest.skipUnless(LIBRARY.exists(),'run the isolated compiled audit first')
    def test_native_gradient_normal_and_clamp_domains(self):
        probe=a.Probe(LIBRARY)
        normal=a.finite_difference(probe,PARAMS,[.15,.04,.1,.3,.2,-.1],1,False)
        self.assertTrue(normal['primary_pass'])
        above=a.finite_difference(probe,PARAMS,[1.1e-4,0.,.1,.2,.3,-.1],1,False)
        self.assertTrue(above['primary_pass'])
        low=a.finite_difference(probe,PARAMS,[5e-5,0.,.1,.2,.3,-.1],1,False)
        self.assertEqual(low['category'],'clamp_interior');self.assertFalse(low['primary_pass'])
        boundary=a.finite_difference(probe,PARAMS,[1e-4,0.,.1,.2,.3,-.1],1,False)
        self.assertEqual(boundary['category'],'clamp_boundary_nondifferentiable')
        self.assertIsNone(boundary['primary_pass'])

    @unittest.skipUnless(LIBRARY.exists(),'run the isolated compiled audit first')
    def test_native_probe_zero_initialization_is_repeatable(self):
        probe=a.Probe(LIBRARY);x=[.15,.04,.1,.3,.2,-.1]
        first=probe(PARAMS,x,-1,True)
        probe(PARAMS,[5e-5,0.,.1,.2,.3,-.1],1,True)
        second=probe(PARAMS,x,-1,True)
        self.assertEqual(first[0],second[0]);np.testing.assert_array_equal(first[1],second[1])

    @unittest.skipUnless(LIBRARY.exists(),'run the isolated compiled audit first')
    def test_fixed_coefficient_primary_time_chain(self):
        row=a.time_chain(a.Probe(LIBRARY),PARAMS,polynomial(),1,16)
        self.assertTrue(row['primary_pass']);self.assertIn('excludes full MINCO',row['scope'])
        self.assertIsInstance(row['primary_pass'],bool)
        json.dumps(row,allow_nan=False)

    @unittest.skipUnless(LIBRARY.exists(),'run the isolated compiled audit first')
    def test_probe_rejects_bad_inputs(self):
        probe=a.Probe(LIBRARY)
        with self.assertRaises(ValueError):probe(PARAMS,[0.]*5)
        with self.assertRaises(ValueError):probe(PARAMS,[0.]*6,0)
        with self.assertRaises(ValueError):probe(PARAMS,[float('nan')]*6)


if __name__=='__main__':unittest.main()
