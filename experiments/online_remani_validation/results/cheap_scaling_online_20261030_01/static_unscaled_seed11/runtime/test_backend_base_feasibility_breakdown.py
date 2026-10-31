#!/usr/bin/env python3
import types
import unittest

import numpy as np

import backend_base_feasibility_breakdown as b


class BreakdownTest(unittest.TestCase):
    def test_attested_source_contract(self):
        p=b.source_contract()
        self.assertEqual(4,len(p['functions_sha256']))
        for name in b.CHANNELS:self.assertIn(name,p['branch_formula_guards'])

    def test_penalty_hand_values(self):
        np.testing.assert_allclose(b.smoothed_l1(np.array([-.1,0,.0025,.005,.01])),
                                   [0,0,.00046875,.0025,.0075],rtol=0,atol=1e-16)
        np.testing.assert_allclose(b.smoothed_log(np.array([-.001,-.0005,0,.0005,.001,.01])),
                                   [0,.09375,.5,.90625,1,1],rtol=0,atol=1e-14)

    def test_straight_high_speed_hand_values(self):
        # weight=2, v=2, max_linear=1, radius=1, maxwheel=.5:
        # 2*(v²-1-.0025)=5.995; each wheel 2*(v²-.25-.0025)=7.495.
        params=[2,1,.05,.37,1,.5,1]
        c=b.point_costs([[2,0]],[[0,0]],[[0,0]],params,1,False)[0]
        np.testing.assert_allclose(c,[0,5.995,7.495,7.495,0,0],rtol=0,atol=1e-13)

    def test_dense_min_speed_hand_value(self):
        # v=.025, v_ns=.05: (mu-.5*.001875)*(.001875/mu)^3*1e6.
        c=b.point_costs([[.025,0]],[[0,0]],[[0,0]],[1,1,.05,.37,1,10,10],1,True)[0]
        np.testing.assert_allclose(c,[214.2333984375,0,0,0,0,0],rtol=0,atol=1e-10)

    def test_zero_curve_actual_dense_measure(self):
        # K=2,D=4: 3 primary anchors, dense counts 3+5+3=11;
        # trapezoid union measure=T=2, native min-speed cost=468.75.
        params=np.array([1,1,.05,.37,1,10,10,0,2,4,1.])
        value=dict(b_parts=[np.zeros((8,8))],times_parts=[np.array([2.])])
        c=b.breakdown(value,types.SimpleNamespace(params=params,gears=np.array([1])))
        self.assertAlmostEqual(c['sum'],937.5,places=10)
        self.assertEqual(3,c['primary_count']);self.assertEqual(3,c['dense_anchor_count'])
        self.assertEqual(11,c['dense_node_count']);self.assertEqual(0,c['minimum_actual_evaluated_speed_m_s'])
        self.assertAlmostEqual(c['costs']['dense_min_speed'],937.5,places=10)


if __name__=='__main__':unittest.main()
