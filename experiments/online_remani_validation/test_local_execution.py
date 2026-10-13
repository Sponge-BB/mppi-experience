#!/usr/bin/env python3
"""Task/transaction, actuator expiry, and exact local rollout regressions."""
import argparse
import ctypes as ct
import json
import io
from pathlib import Path
import shutil
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch,MagicMock
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped,TwistStamped
from quadrotor_msgs.msg import PolynomialTraj,PolynomialMatrix
from local_execution import ExecutionSession,message_key,configure
from std_msgs.msg import String
from simulator import Simulator
from simulator import polynomial
from native_reference import reference_polynomial
from test_mppi_v2 import Controller,DP,ptr

HERE=Path(__file__).resolve().parent
TEST_BUILD=None

class TaskContracts(unittest.TestCase):
    def goal(self,x=3,stamp=1001):
        m=PoseStamped();m.header.frame_id='world';m.header.stamp=rospy.Time.from_sec(stamp)
        m.pose.position.x=x;m.pose.orientation.w=1.;return m
    def polynomial(self,ident=1,x=0):
        m=PolynomialTraj();m.action=1;m.singul=1;m.trajectory_id=ident;m.header.stamp=rospy.Time.from_sec(1002)
        p=PolynomialMatrix();p.num_dim=8;p.num_order=7;p.duration=2.
        c=np.zeros((8,8));c[0,-2]=.2;c[0,-1]=x;p.data=c.flatten(order='F').tolist();m.trajectory=[p];return m
    def transaction(self,g,messages):return message_key(g)+'\n'+str(len(messages))+'\n'+'\n'.join(message_key(m) for m in messages)
    def active(self):
        s=ExecutionSession();g=self.goal();s.goal(g);s.state(message_key(g)+'\n1\n0')
        s.transaction(self.transaction(g,[self.polynomial()]),1002);return s,g
    def test_old_goal_cannot_supply_reference(self):
        s,g=self.active();s.goal(self.goal(x=-1,stamp=1003));generation=s.generation
        event,_=s.transaction(self.transaction(g,[self.polynomial()]),1004)
        self.assertEqual(event,'other_goal');self.assertIsNone(s.guide);self.assertEqual(s.generation,generation)
    def test_whole_bundle_commits_or_fails(self):
        s,g=self.active();event,_=s.transaction(self.transaction(g,[self.polynomial(),self.polynomial(2,.4)]),1003)
        self.assertEqual(event,'replace');self.assertEqual(len(s.guide.refs),2)
        broken=self.transaction(g,[self.polynomial(),self.polynomial(3,.4)])
        event,_=s.transaction(broken,1004);self.assertEqual(event,'invalid');self.assertIsNone(s.guide)
        self.assertIn('invalid',s.authority())
    def test_fast_sampling_matches_native_scalar_sampling(self):
        rng=np.random.default_rng(13)
        for sign in (-1,1):
            m=self.polynomial();m.singul=sign
            coefficients=rng.normal(0,.01,(8,8));coefficients[:2,-2]=[.2,.05]
            m.trajectory[0].data=coefficients.flatten(order='F').tolist()
            np.testing.assert_array_equal(reference_polynomial(m),polynomial(m,.04)[:,:5])
    def test_actual_abort_stays_hard_until_new_reference(self):
        s,g=self.active();old=s.guide;m=PolynomialTraj();m.action=2
        s.transaction(self.transaction(g,[m]),1003);self.assertIs(s.guide,old);self.assertEqual(s.authority(),'native_abort')
        s.state(message_key(g)+'\n1\n0');self.assertEqual(s.authority(),'native_abort')
        s.transaction(self.transaction(g,[self.polynomial()]),1004)
        s.state(message_key(g)+'\n1\n1');self.assertEqual(s.authority(),'native_hard_stop')
        s.state(message_key(g)+'\n0\n0');self.assertEqual(s.authority(),'no_active_task')
    def test_status_before_goal_matches_exact_task(self):
        s=ExecutionSession();g=self.goal();s.state(message_key(g)+'\n1\n0');s.goal(g)
        self.assertEqual(s.authority(),'waiting_native_trajectory')
        s.state(message_key(self.goal(-1))+'\n1\n0');self.assertTrue(s.enabled)
    def test_abort_discards_already_delayed_reference_updates(self):
        controller=Controller(TEST_BUILD,HERE/'results/final_report/empty_mppi')
        with tempfile.TemporaryDirectory(prefix='remani_execution_test_') as temporary:
            out=Path(temporary);shutil.copy2(HERE/'results/final_report/empty_mppi/geometry.json',out/'geometry.json')
            node=SimpleNamespace(lib=controller.lib,classified=True,out=out,lock=threading.Lock(),reference_events=io.StringIO())
            callbacks={}
            def subscribe(topic,kind,callback,**kw):callbacks[topic]=callback
            with patch('rospy.Publisher',return_value=MagicMock()),patch('rospy.Subscriber',side_effect=subscribe), \
                 patch('rospy.get_param',return_value=3.),patch('rospy.get_time',return_value=1002.):
                configure(node);g=self.goal()
                callbacks['/move_base_simple/goal'](g)
                callbacks['/planning/execution_state'](String(message_key(g)+'\n1\n0'))
                callbacks['/planning/execution_reference'](String(self.transaction(g,[self.polynomial()])))
                callbacks['/planning/execution_reference'](String(self.transaction(g,[self.polynomial()])))
                self.assertEqual(len(node.pending_execution_transactions),1)
                abort=PolynomialTraj();abort.action=2
                callbacks['/planning/execution_reference'](String(self.transaction(g,[abort])))
                self.assertEqual(node.pending_execution_transactions,[])
                self.assertEqual(node.execution.authority(),'native_abort')
            node.execution_log.close()
        controller.close()
    def test_stale_and_nonfinite_commands_rejected(self):
        sim=Simulator.__new__(Simulator);sim.lock=threading.Lock();sim.t=1.2;sim.command_time=1.1
        sim.command=np.zeros(2);sim.command_lease=.075
        m=TwistStamped();m.header.frame_id='world';m.header.stamp=rospy.Time.from_sec(1001.1);m.twist.linear.x=.2
        sim.local_cmd(m);self.assertEqual(sim.command[0],0)
        m.header.stamp=rospy.Time.from_sec(1001.19);sim.local_cmd(m);self.assertEqual(sim.command[0],.2)
        m.header.stamp=rospy.Time.from_sec(1001.2);m.twist.linear.x=float('nan');sim.local_cmd(m)
        self.assertEqual(sim.command[0],.2)
        sim.limits=json.loads((HERE/'results/final_report/empty_mppi/limits.json').read_text())
        sim.s=np.array([0.,0.,0.,.15,0.]);sim.t=1.5
        before=sim.s[3];sim.integrate(.05)
        self.assertLess(sim.s[3],before,'expired local command must brake even if publisher disappears')


def kernel_checks(build):
    c=Controller(build,HERE/'results/final_report/empty_mppi')
    c.lib.online_selected_controls.argtypes=[ct.c_void_p,DP,ct.c_int];c.lib.online_selected_controls.restype=ct.c_int
    c.lib.online_validate_execution.argtypes=[ct.c_void_p,DP,DP,ct.c_int,DP,ct.c_int,ct.c_double,ct.c_double,
        DP,ct.c_int,ct.c_int,ct.c_double,ct.c_double,ct.c_double,DP,ct.c_int,DP]
    x=np.linspace(0,3,301);ref=np.ascontiguousarray(np.column_stack((x,x*0,x*0,x*0+.18)))
    s=np.array([.5,0,0,.15,0.]);obs=np.empty((0,6));nominal=np.zeros(8)
    code=c.lib.online_step(c.handle,ptr(s),ptr(ref),len(ref),ptr(obs),len(obs),.04,
                          ptr(c.field),300,200,-7.5,-5.,.05,0,ptr(nominal))
    assert code==0 and np.isfinite(nominal).all(),(code,nominal)
    count=c.lib.online_selected_controls(c.handle,None,0);us=np.zeros((count,2));c.lib.online_selected_controls(c.handle,ptr(us),count)
    assert count>0 and np.array_equal(us[0],nominal[:2]),'export must precede warm-start feedback'
    def validate(state=s,controls=us,targets=obs,age=.04,state_age=.01):
        out=np.zeros((650,8));report=np.zeros(6)
        code=c.lib.online_validate_execution(c.handle,ptr(state),ptr(controls),len(controls),ptr(targets),len(targets),
            age,state_age,ptr(c.field),300,200,-7.5,-5.,.05,ptr(out),len(out),ptr(report))
        return code,out,report
    code,rows,report=validate();assert code==0 and report[0]==1 and report[5]==1,(code,report)
    assert report[3]==count*10+1
    np.testing.assert_allclose(rows[1,6:8],us[0])
    c.field[:]=0.;code,_,_=validate();assert code!=0,'new obstacle snapshot must veto previously selected trajectory'
    c.field[:]=10.;code,_,_=validate(age=.36);assert code!=0
    bad=us.copy();bad[0,0]=5.;code,_,_=validate(controls=bad);assert code!=0
    # Full fixed-arm/body projection, not just base center, must be checked.
    targets=np.array([[.9,0,0,0,.23,1.]])
    code,_,_=validate(targets=targets);assert code!=0
    bad_state=s.copy();bad_state[0]=float('nan');code,_,_=validate(state=bad_state);assert code!=0
    # A safe, stationary distant track shortens the CV-valid prefix without
    # rejecting the already-selected first command or extending prediction.
    targets=np.array([[4.,2.,0.,.2,.2,1.]])
    code,rows,report=validate(targets=targets,age=.30);assert code==0,(code,report)
    assert rows[int(report[3])-1,0]+.30<=2.,'executed prefix exceeds CV validity'
    targets=np.array([[-.3345,0.,.28,0.,.23,1.]])
    escape=np.ascontiguousarray([[.2296,0.]]*10)
    code,_,report=validate(controls=escape,targets=targets,age=0.,state_age=0.)
    assert code==5 and report[1]>0 and report[2]<0,(code,report)
    c.close();return ['exact selected control sequence verified on fresh snapshot',
        'new occupied snapshot invalidates previous selection','stale CV snapshot rejected',
        'wheel/velocity-infeasible command rejected','full body initial overlap rejected',
        'nonfinite odometry rejected','remaining CV prefix accepted without extrapolation',
        'collision-free proposal rejected when its next-cycle braking fallback collides']


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--build',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    TEST_BUILD=a.build
    result=unittest.TextTestRunner().run(unittest.defaultTestLoader.loadTestsFromTestCase(TaskContracts))
    if not result.wasSuccessful():raise SystemExit(1)
    checks=kernel_checks(a.build);a.out.parent.mkdir(parents=True,exist_ok=True)
    with a.out.open('x') as f:json.dump({'passed':True,'task_tests':result.testsRun,'kernel_checks':checks},f,indent=2)
    print('PASS',checks)
