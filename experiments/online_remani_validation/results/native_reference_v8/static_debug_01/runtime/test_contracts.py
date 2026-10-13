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
from mppi_node import execution_gate
from native_reference import NativeReference,sampled_reference,fingerprint
import rospy


class Contracts(unittest.TestCase):
    def message(self, ident=1, start=1001., x=0.):
        m=PolynomialTraj();m.action=1;m.trajectory_id=ident;m.header.stamp=rospy.Time.from_sec(start);m.singul=1
        p=PolynomialMatrix();p.num_dim=8;p.num_order=7;p.duration=2.
        c=np.zeros((8,8));c[0,-2]=.2;c[0,-1]=x;p.data=c.flatten(order='F').tolist();m.trajectory=[p]
        return m

    def test_native_replace_append_cancel(self):
        r=NativeReference();self.assertEqual(r.feed(self.message(),1001),'replace')
        self.assertEqual(r.feed(self.message(2,x=.4),1001.01),'append');self.assertEqual(r.revision,2);self.assertEqual(r.epoch,1)
        self.assertAlmostEqual(r.times[-1][-1],4.)
        self.assertEqual(r.feed(self.message(start=1002,x=.1),1002),'replace');self.assertEqual(len(r.refs),1);self.assertEqual(r.epoch,2)
        m=PolynomialTraj();m.action=2;self.assertEqual(r.feed(m,1003),'cancel');self.assertEqual(r.refs,[])
        self.assertEqual(r.feed(self.message(2,x=.4),1004),'invalid')

    def test_native_invalid_fail_closed(self):
        for change in ('stamp','gap','nan','duration','frame','id'):
            r=NativeReference();r.feed(self.message(),1001);m=self.message(2,x=.4)
            if change=='stamp':m.header.stamp=rospy.Time.from_sec(1002)
            if change=='gap':m.trajectory[0].data[-8]=2.
            if change=='nan':m.trajectory[0].data[0]=float('nan')
            if change=='duration':m.trajectory[0].duration=-1
            if change=='frame':m.header.frame_id='odom'
            if change=='id':m.trajectory_id=3
            self.assertEqual(r.feed(m,1001.01),'invalid',change);self.assertEqual(r.refs,[])

    def test_heading_interpolation_wrap(self):
        refs=[np.array([[0,0,3.13,.2],[.1,0,np.nan,0],[.2,0,-3.13,.2]])]
        r=sampled_reference(refs,0)
        self.assertGreater(abs(r[1,2]),3.1);self.assertEqual(fingerprint(r),fingerprint(r.copy()))

    def test_warning_does_not_cancel(self):
        r=NativeReference();r.feed(self.message(),1001);m=PolynomialTraj();m.action=4
        self.assertEqual(r.feed(m,1002),'notice');self.assertTrue(r.refs);self.assertEqual(r.revision,1)

    def test_shadow_cannot_command_even_if_limit_released(self):
        for limit in (-1.,0.,.2):
            result=np.array([.18,.4,.18,.4,0.,0.,0.,10.])
            reason=execution_gate(result,'ok',limit,False,True)
            np.testing.assert_array_equal(result[:2],[0.,0.]);self.assertEqual(reason,'native_abort_shadow')
            np.testing.assert_array_equal(result[2:4],[.18,.4])

    def test_native_stop_and_bundle_change_remain_authoritative(self):
        for limit,changed in ((0.,False),(-1.,True)):
            result=np.array([.18,.4,.18,.4,0.,0.,0.,10.])
            execution_gate(result,'ok',limit,changed,False)
            np.testing.assert_array_equal(result[:2],[0.,0.])

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
