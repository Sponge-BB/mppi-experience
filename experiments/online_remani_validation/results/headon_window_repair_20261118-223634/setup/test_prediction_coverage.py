#!/usr/bin/env python3
"""Bounded offline tests of forecast coverage, independent of ROS/robot IO."""
import importlib.util
from pathlib import Path
import sys
import unittest

import numpy as np
from scipy.spatial import cKDTree

KERNEL=Path(__file__).resolve().parents[1]/'mppi/runtime/active_arm_mppi.py'
spec=importlib.util.spec_from_file_location('window_kernel',KERNEL)
kernel=importlib.util.module_from_spec(spec)
sys.modules[spec.name]=kernel
spec.loader.exec_module(kernel)


class SphereModel:
    def __init__(self):self.calls=0

    def body_spheres_batch(self,q,base):
        self.calls+=1
        return np.c_[base[:,:2],np.ones(len(base)),np.full(len(base),.05)][:,None,:]


class Controller(kernel.WholeBodyMPPI):
    def configuration_gap(self,spheres):return np.full(len(spheres),10.)


class CheckedSnapshot(kernel.Snapshot):
    def query(self,spheres,time):
        if self.groups:
            age=np.asarray(time)-self.header[18]
            assert np.all(age>=-.05) and np.all(age<=self.header[16]+1e-9)
        return super().query(spheres,time)


def fixture(dynamic=True):
    limits=dict(vmax=.3,wmax=.8,accel=.35,angular_accel=1.,track=.37,
                wheel_v=.2296,wheel_a=.2975)
    arm=[np.full(6,-3.),np.full(6,3.),np.full(6,.45),np.full(6,.9)]
    controller=Controller(SphereModel(),limits,arm,[],horizon=20,step=.1,
                          random_samples=2,coverage_reserve=.20)
    h=np.zeros(32);h[5:8]=20;h[8:11]=-5.;h[11]=.5
    h[16]=2.;h[18]=100.
    groups=[(np.zeros(3),cKDTree([[4.,4.,4.]]))] if dynamic else []
    snapshot=CheckedSnapshot(h,np.full((20,20,20),10.),groups)
    state=np.zeros(17)
    ref=np.c_[np.linspace(0.,4.,41),np.zeros(41),np.zeros(41),np.full(41,.22)]
    return controller,snapshot,state,ref


class CoverageTests(unittest.TestCase):
    def test_no_tracks_keeps_requested_horizon(self):
        c,s,z,r=fixture(False)
        result,info=c.plan(z,r,z[5:11],s,100.)
        self.assertIsNotNone(result)
        self.assertEqual(len(result[0]),20)
        self.assertEqual(info['effective_horizon_s'],2.)
        self.assertIsNone(info['forecast_remaining_s'])

    def test_distant_track_allows_full_stop_with_delivery_reserve(self):
        c,s,z,r=fixture()
        result,info=c.plan(z,r,z[5:11],s,100.10)
        self.assertIsNotNone(result)
        self.assertGreater(len(result[0]),0)
        self.assertLess(len(result[0]),20)
        self.assertEqual(s.header[16],2.)
        _,valid,_,_=c.evaluate(z,result[0][None],r,z[5:11],s,100.10,.20)
        self.assertTrue(valid[0])
        duration=c.last_evaluation['checked_duration_s'][0]
        self.assertLessEqual(.10+duration+.20,2.+1e-9)
        print('far track H=%d complete_stop=%.2fs age=.10s reserve=.20s' %
              (len(result[0]),duration))

    def test_own_braking_tail_excludes_other_candidate_padding(self):
        c,s,z,r=fixture()
        controls=np.zeros((2,12,8));controls[1,:,0]=.22
        # A stationary candidate stops at 1.25 s; the moving one cannot
        # complete its stop in the 1.5 s available forecast window.
        _,valid,gaps,states=c.evaluate(z,controls,r,z[5:11],s,100.50)
        self.assertEqual(states.shape,(2,13,17))
        self.assertEqual(valid.tolist(),[True,False])
        self.assertGreater(gaps[0],0.)
        self.assertEqual(c.last_evaluation['coverage_valid'].tolist(),[True,False])
        self.assertGreater(c.last_evaluation['stop_steps'][1],c.last_evaluation['stop_steps'][0])

    def test_exhausted_or_expired_window_fails_before_geometry(self):
        for now in (101.90,102.10):
            c,s,z,r=fixture()
            result,info=c.plan(z,r,z[5:11],s,now)
            self.assertIsNone(result)
            self.assertEqual(info['failure_reason'],'prediction_coverage_insufficient')
            self.assertEqual(c.model.calls,0)

    def test_delivery_rechecks_unchanged_complete_candidate(self):
        c,s,z,r=fixture()
        result,_=c.plan(z,r,z[5:11],s,100.10)
        controls=result[0].copy()
        # Delivery inside the reserved interval succeeds without deducting
        # reserve twice. A later delivery must reject, never trim the tail.
        _,valid,_,_=c.evaluate(z,controls[None],r,z[5:11],s,100.29)
        self.assertTrue(valid[0])
        duration=c.last_evaluation['checked_duration_s'][0]
        late=102.-duration+.01
        _,valid,_,_=c.evaluate(z,controls[None],r,z[5:11],s,late)
        self.assertFalse(valid[0])
        self.assertFalse(c.last_evaluation['coverage_valid'][0])
        np.testing.assert_array_equal(controls,result[0])
        accepted,_=c.lease_brake(z,controls,s,101.95)
        self.assertFalse(accepted)
        self.assertFalse(c.last_lease_coverage)


if __name__=='__main__':unittest.main(verbosity=2)
