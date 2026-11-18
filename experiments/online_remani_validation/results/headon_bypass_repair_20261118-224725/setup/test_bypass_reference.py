#!/usr/bin/env python3
"""Offline route-family checks with the archived native fixed-sphere model."""
import json
from pathlib import Path
import sys
import unittest

import numpy as np
from scipy.spatial import cKDTree

CASE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(CASE/'mppi/runtime'))
from active_arm_model import PiperArmModel
from active_arm_mppi import Snapshot,WholeBodyMPPI


def fixture(high=False):
    out=CASE/'mppi';cfg=json.loads((out/'active_arm_config.json').read_text())
    model=PiperArmModel.from_parameters(out/'parameters.yaml',out/'model/mm.urdf')
    al=cfg['arm_limits']
    c=WholeBodyMPPI(model,json.loads((out/'limits.json').read_text()),
                   [al[k] for k in ('lower','upper','velocity','acceleration')],
                   cfg['postures'],horizon=20,random_samples=2)
    state=np.r_[np.zeros(5),cfg['q_start'],np.zeros(6)]
    ref=np.c_[np.linspace(0.,4.,81),np.zeros(81),np.zeros(81),np.full(81,.22)]
    zmin,zmax=(3.,3.4) if high else (.1,1.5)
    pts=np.array([[x,y,z] for x in (2.3,2.7) for y in (-.2,.2) for z in (zmin,zmax)])
    h=np.zeros(32);h[5:8]=60;h[8:11]=(-1.,-1.5,-.5);h[11]=.1
    h[16]=2.;h[18]=100.
    snap=Snapshot(h,np.full((60,60,60),10.),[(np.zeros(3),cKDTree(pts))])
    return c,snap,state,ref


def follow(c,state,reference,angle=0.):
    """Expose what a candidate family can express; not a safety validation."""
    z=state.copy();rows=[z.copy()];c.H=1;c.previous=None
    for _ in range(200):
        u,_=c.proposals(z,reference,state[5:11],[(1.,angle)])
        z=c.advance(z,u[0,0],.05);z=c.advance(z,u[0,0],.05)
        rows.append(z.copy())
        if z[0]>3.1:break
    return np.asarray(rows)


class BypassTests(unittest.TestCase):
    def test_true_offset_reaches_side_that_angle_bias_cannot(self):
        c,s,z,r=fixture()
        routes=c.bypass_references(z,r,s,100.1)
        self.assertEqual(len(routes),2)
        side,offset,guide=routes[0]
        self.assertEqual(side,1)
        old=follow(c,z,r,.5);new=follow(c,z,guide)
        self.assertLess(np.max(old[:,1]),.23)
        self.assertGreater(np.max(new[:,1]),offset-.04)
        crossing=new[(new[:,0]>2.2)&(new[:,0]<2.8)]
        self.assertGreater(len(crossing),0)
        fixed=c.spheres(crossing)[:,:47]
        inner=np.min(fixed[:,:,1]-fixed[:,:,3]-s.header[11])
        self.assertGreater(inner,.2)
        print('old max lateral=%.3f m; bypass=%.3f m; observed box edge=.200 m; body inner edge=%.3f m' %
              (np.max(old[:,1]),np.max(new[:,1]),inner))

    def test_high_arm_only_obstacle_does_not_trigger_base_route(self):
        c,s,z,r=fixture(True)
        self.assertEqual(c.bypass_references(z,r,s,100.1),[])
        self.assertEqual(c.bypass_side,0)

    def test_side_and_axis_persist_after_valid_selection_then_clear_when_passed(self):
        c,s,z,r=fixture()
        result,info=c.plan(z,r,z[5:11],s,100.1)
        self.assertIsNotNone(result)
        self.assertTrue(info['bypass_active'])
        side=info['bypass_side'];axis=c.bypass_frame[1].copy()
        z2=z.copy();z2[0]=.05;z2[2]=.15
        result,info=c.plan(z2,r,z2[5:11],s,100.1)
        self.assertIsNotNone(result)
        self.assertEqual(info['bypass_side'],side)
        np.testing.assert_allclose(c.bypass_frame[1],axis)
        _,valid,_,_=c.evaluate(z2,result[0][None],r,z2[5:11],s,100.29)
        self.assertTrue(valid[0])
        z2[0]=3.5
        self.assertEqual(c.bypass_references(z2,r,s,100.1),[])
        self.assertEqual(c.bypass_side,0)


if __name__=='__main__':unittest.main(verbosity=2)
