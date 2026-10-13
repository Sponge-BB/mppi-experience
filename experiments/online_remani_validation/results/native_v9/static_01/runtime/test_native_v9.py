#!/usr/bin/env python3
"""No ROS master: membership, collision and static recovery regression tests."""
import argparse
import ctypes as ct
import json
from pathlib import Path
import unittest
import numpy as np
from classified_frame import collision_frame
from test_mppi_v2 import Controller,DP,ptr

HERE=Path(__file__).resolve().parent

class Membership(unittest.TestCase):
    def test_partition_and_pcd_protection(self):
        # Exact labelled sample, nearby UNKNOWN sample, coincident static PCD.
        points=np.array([[1,0,.4,0,.2,0,1,9,1],[1.1,0,.4,0,0,0,0,0,0]])
        field,obs,stats=collision_frame(points,[[1,0,.4]])
        self.assertEqual(stats['dynamic_points'],1);self.assertEqual(stats['retained_measured_static_points'],1)
        self.assertEqual(field[100,170],0);self.assertEqual(field[100,172],0)
        np.testing.assert_allclose(obs,[[1,0,0,.2,.025,1]])
        field,obs,_=collision_frame(points[:1],[[4,2,.4]])
        self.assertGreater(field[100,170],1);self.assertEqual(len(obs),1)

    def test_all_unknown_retained(self):
        p=np.array([[1,0,.4,0,0,0,0,0,0]])
        field,obs,stats=collision_frame(p,[[4,2,.4]])
        self.assertEqual(len(obs),0);self.assertEqual(field[100,170],0)
        self.assertEqual(stats['retained_measured_static_points'],1)

    def test_bad_membership_fails_closed(self):
        for index,value in [(0,float('nan')),(6,0),(7,0),(8,2)]:
            p=np.array([[1,0,.4,0,.2,0,1,9,1.]]);p[0,index]=value
            with self.assertRaises(ValueError):collision_frame(p,[[4,2,.4]])

    def test_full_shape_and_velocity_consistency(self):
        p=np.array([[1,-.23,.4,0,.2,0,1,9,1],[1,.23,.4,0,.2,0,1,9,1]])
        _,obs,_=collision_frame(p,[[4,2,.4]]);self.assertAlmostEqual(obs[0,4],.255)
        p[1,4]=.4
        with self.assertRaises(ValueError):collision_frame(p,[[4,2,.4]])


def kernel_tests(build):
    c=Controller(build,HERE/'results/final_report/empty_mppi')
    c.lib.online_recovery_debug.argtypes=[ct.c_void_p,DP]
    c.lib.online_avoidance_debug.argtypes=[ct.c_void_p,DP]
    x=np.linspace(0,3,301);ref=np.ascontiguousarray(np.column_stack((x,x*0,x*0,np.minimum(.18,.8*(3-x)))))
    state=np.array([.5,0.,0.,0.,0.]);none=np.empty((0,6))
    c.field=np.zeros((200,300));u=c.step(state,ref,none)
    recovery=np.zeros(10);c.lib.online_recovery_debug(c.handle,ptr(recovery))
    assert recovery[0]==1 and recovery[2]>=100 and recovery[3]==0 and recovery[9]==1,(u,recovery)
    assert np.max(np.abs(u))==0
    c.lib.online_reset(c.handle)
    xx,yy=np.meshgrid(-7.5+.05*np.arange(300),-5.+.05*np.arange(200))
    c.field=np.ascontiguousarray(np.maximum(0,2.-abs(yy)-np.sqrt(2)*.05))
    u=c.step(state,ref,none);assert u[0]>0,u
    c.close()
    return ['static-only fully occupied map runs expanded search, rejects all candidates and reports unverified brake',
            'empty scene still commands forward motion']


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--build',type=Path,required=True);p.add_argument('--out',type=Path,required=True);args=p.parse_args()
    result=unittest.TextTestRunner().run(unittest.defaultTestLoader.loadTestsFromTestCase(Membership))
    if not result.wasSuccessful():raise SystemExit(1)
    checks=kernel_tests(args.build)
    args.out.parent.mkdir(parents=True,exist_ok=True)
    with args.out.open('x') as f:json.dump({'passed':True,'membership_tests':result.testsRun,'kernel_checks':checks},f,indent=2)
    print('PASS',checks)
