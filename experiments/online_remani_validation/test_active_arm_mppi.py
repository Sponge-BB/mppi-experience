"""Independent acceptance checks for the experimental whole-body executor."""
import json
from pathlib import Path
import unittest

import numpy as np
from scipy.spatial import cKDTree

from active_arm_model import PiperArmModel
from active_arm_mppi import Snapshot,WholeBodyMPPI
from active_arm_scene import Q_START,Q_RETRACT


def empty_snapshot():
    h=np.zeros(32);h[:2]=[3,1];h[2:8]=[1,1000,1,120,120,50]
    h[8:12]=[-2.,-3.,-.1,.05];h[12]=67;h[14:20]=[1,.35,3.,1000,1000,1];h[20]=1
    return Snapshot(h,np.full((120,120,50),10.),[])


class WholeBodyAcceptance(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model=PiperArmModel.from_defaults()
        cls.limits=json.loads((Path(__file__).parent/'results/short_prefix_rejoin_20261101_01/midcross_B/limits.json').read_text())
        cls.reference=np.c_[np.linspace(0,3,100),np.zeros((100,2)),np.full(100,.2)]

    def kernel(self,enabled=True):
        m=self.model
        return WholeBodyMPPI(m,self.limits,[m.lower,m.upper,m.velocity_limits,m.acceleration_limits],
                             [Q_RETRACT],enabled=enabled,random_samples=2)

    def test_native_half_cell_trilinear_and_unknown(self):
        s=empty_snapshot();h=s.header
        x=h[8]+(np.arange(120)+.5)*h[11]
        s.distance=np.broadcast_to(x[:,None,None]+4.,s.distance.shape)
        spheres=np.array([[[0.,0.,1.,.03]],[[.2,0.,1.,.03]]])
        gap,_=s.query(spheres,np.array([1000.,1000.2]))
        np.testing.assert_allclose(gap,[4.-.03-.05,4.2-.03-.05],atol=1e-12)
        bad=spheres.copy();bad[0,0,0]=-20
        self.assertLess(s.query(bad,1000)[0][0],0.)

    def test_dynamic_queries_use_each_absolute_time(self):
        s=empty_snapshot();s.groups=[(np.array([.1,0,0]),cKDTree(np.array([[0,0,1.]])))]
        spheres=np.array([[[.3,0,1.,.03]],[[.3,0,1.,.03]]])
        gap,_=s.query(spheres,np.array([1000.,1001.]))
        np.testing.assert_allclose(gap,[.22,.12],atol=1e-12)
        self.assertLess(s.query(spheres,np.array([1000.,1004.]))[0][1],0.)

    def test_reused_geometry_matches_independent_configuration_check(self):
        k=self.kernel();q=np.stack([Q_START,Q_RETRACT,self.model.q_nominal])
        z=np.c_[np.zeros((3,5)),q,np.zeros((3,6))]
        z[:,:3]=[[0,0,0],[1,.2,.7],[-.5,.3,-.5]]
        np.testing.assert_allclose(k.configuration_gap(k.spheres(z)),
                                   self.model.configuration_gaps_batch(q),atol=1e-12)

    def test_fixed_arm_group_cannot_sample_joint_motion(self):
        k=self.kernel(False);z=np.r_[np.zeros(5),Q_START,np.zeros(6)]
        planned,info=k.plan(z,self.reference,Q_START,empty_snapshot(),1000.)
        self.assertIsNotNone(planned)
        self.assertFalse(info['intervention'])
        np.testing.assert_array_equal(planned[0][:,2:],np.zeros((k.H,6)))
        np.testing.assert_allclose(planned[1][:,5:11],np.tile(Q_START,(k.H+1,1)))

    def test_initial_collision_is_rejected_even_if_future_moves_out(self):
        k=self.kernel();z=np.r_[np.zeros(5),Q_START,np.zeros(6)]
        s=empty_snapshot();center=k.spheres(z[None])[0,-1,:3]
        s.groups=[(np.array([2.,0,0]),cKDTree(center[None]))]
        us=np.zeros((1,k.H,8));us[:,:,0]=.2
        _,valid,_,_=k.evaluate(z,us,self.reference,Q_START,s,1000.)
        self.assertFalse(valid[0])

    def test_immediate_stop_includes_actual_joint_deceleration(self):
        k=self.kernel();z=np.r_[np.zeros(5),Q_START,np.zeros(6)];z[12]=.4
        good,gap=k.lease_brake(z,None,empty_snapshot(),1000.,lease=0.)
        self.assertTrue(good)
        final=z.copy()
        for _ in range(20):final=k.advance(final,np.zeros(8),.05)
        self.assertGreater(final[6],z[6])
        self.assertAlmostEqual(final[12],0.,places=12)

    def test_lease_holds_first_command_instead_of_assuming_future_reversal(self):
        k=self.kernel();z=np.r_[np.zeros(5),Q_START,np.zeros(6)]
        z[5]=self.model.upper[0]-.13;z[11]=.30
        self.assertTrue(self.model.check(z[5:11],z[11:17])['valid'])
        controls=np.zeros((k.H,8));controls[:,2]=-.45;controls[0,2]=.45
        good,_=k.lease_brake(z,controls,empty_snapshot(),1000.,lease=.25)
        self.assertFalse(good)


if __name__=='__main__':unittest.main()
