#!/usr/bin/env python3
"""Read-only/unit tests; no ROS master or physical command publisher."""
import math
import os
import unittest
from unittest.mock import patch
import numpy as np
from quadrotor_msgs.msg import PolynomialTraj,PolynomialMatrix
from simulator import polynomial,isolation
from progress import project


class Contracts(unittest.TestCase):
    def test_remote_master_rejected(self):
        for uri in ['http://192.168.137.107:11311','http://localhost:11311','']:
            with patch.dict(os.environ,{'ROS_MASTER_URI':uri}):
                with self.assertRaises(RuntimeError):isolation()
        with patch.dict(os.environ,{'ROS_MASTER_URI':'http://127.0.0.1:11441'}):isolation()

    def test_native_polynomial_column_major(self):
        m=PolynomialTraj();m.action=1;m.singul=1;p=PolynomialMatrix();p.num_dim=8;p.num_order=7;p.duration=2.
        coef=np.zeros((8,8));coef[0,-2]=.2;coef[1,-1]=.4;coef[2:,-1]=np.arange(6)*.1;p.data=coef.flatten(order='F').tolist();m.trajectory=[p]
        rows=polynomial(m);np.testing.assert_allclose(rows[:,1],rows[:,0]*.2);np.testing.assert_allclose(rows[:,2],.4);np.testing.assert_allclose(rows[:,4],.2)
        np.testing.assert_allclose(rows[:,5:],np.broadcast_to(np.arange(6)*.1,rows[:,5:].shape));m.singul=-1;rows=polynomial(m)
        np.testing.assert_allclose(abs(rows[:,3]),math.pi);np.testing.assert_allclose(rows[:,4],-.2)

    def test_projection_uses_feedback_not_wall_clock(self):
        times=np.linspace(0,5,101);ref=np.column_stack([times*.2,times*0,times*0,times*0+.2])
        state=np.array([.23,.1,0.,.1,0.]);t,scale,error=project(state,ref,times,0.)
        self.assertAlmostEqual(t,1.15);self.assertAlmostEqual(scale,.5);self.assertAlmostEqual(error,.1)
        state[3]=0;t2,scale,_=project(state,ref,times,t);self.assertAlmostEqual(t2,t);self.assertEqual(scale,0)

    def test_reverse_progress(self):
        times=np.linspace(0,5,101);ref=np.column_stack([-times*.2,times*0,times*0,times*0-.2]);state=np.array([-.23,0,0.,-.1,0.])
        t,scale,error=project(state,ref,times,0.);self.assertAlmostEqual(t,1.15);self.assertAlmostEqual(scale,.5);self.assertAlmostEqual(error,0)

    def test_projection_no_global_jump(self):
        times=np.linspace(0,20,401);ref=np.column_stack([times*.2,times*0,times*0,times*0+.2]);state=np.array([3.,0,0.,.1,0.])
        t,_,error=project(state,ref,times,0.);self.assertLessEqual(t,2.);self.assertGreater(error,2.)


if __name__=='__main__':unittest.main()
