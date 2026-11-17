"""Plant dynamics, truth separation and atomic command contract tests."""
import json
import threading
from types import SimpleNamespace
import unittest

import numpy as np

from active_arm_model import PiperArmModel, DEFAULT_PARAMETERS
from active_arm_scene import Q_START, Q_RETRACT
from active_arm_simulator import WholeBodyPlant, ActiveArmSimulator


class WholeBodyPlantTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = PiperArmModel.from_defaults()
        cls.limits = json.loads((DEFAULT_PARAMETERS.parent/'limits.json').read_text())

    def plant(self, q=Q_START):
        return WholeBodyPlant(self.model, self.limits, q)

    def test_arm_acceleration_and_actual_position_velocity(self):
        plant = self.plant()
        command = np.array([0.,0.,0.,.3,0.,0.,-.2,0.])
        plant.step(command,.005,-1.)
        np.testing.assert_allclose(plant.state[11:17], [0.,.0045,0.,0.,-.0045,0.], atol=1e-14)
        np.testing.assert_allclose(plant.state[5:11]-Q_START,
                                   [0.,.00001125,0.,0.,-.00001125,0.], atol=1e-14)
        self.assertFalse(plant.collided)
        self.assertFalse(plant.joint_violation)

    def test_whole_body_expiry_braking_preserves_bounds(self):
        plant = self.plant()
        moving = np.array([.2,.1,0.,.3,0.,0.,-.15,0.])
        previous=plant.state.copy()
        for i in range(160):
            command = moving if i<80 else np.zeros(8)
            plant.step(command,.005,-1.)
            acceleration=(plant.state[11:17]-previous[11:17])/.005
            self.assertTrue(np.all(np.abs(acceleration)<=self.model.acceleration_limits+1e-12))
            h=self.limits['track']/2
            self.assertLessEqual(max(abs(plant.state[3]-h*plant.state[4]),abs(plant.state[3]+h*plant.state[4])),self.limits['wheel_v']+1e-12)
            previous=plant.state.copy()
        self.assertLess(np.max(np.abs(plant.state[11:17])),1e-12)
        self.assertLess(abs(plant.state[3]),.02)
        self.assertFalse(plant.collided)
        self.assertEqual(plant.max_substep,.005)

    def test_truth_detects_obstacle_without_changing_command_or_state(self):
        fixed=self.plant(); fixed.state[0]=1.6
        before=fixed.state.copy(); fixed.audit(3.)
        np.testing.assert_array_equal(fixed.state,before)
        self.assertTrue(fixed.collided)
        retract=self.plant(Q_RETRACT); retract.state[0]=1.6; retract.audit(3.)
        self.assertFalse(retract.collided)
        self.assertGreater(retract.external_gap,.05)

    def test_joint_limit_violation_is_logged_not_projected_away(self):
        plant=self.plant(); plant.state[5]=self.model.upper[0]+.01
        plant.audit(-1.,7.)
        self.assertTrue(plant.joint_violation)
        self.assertEqual(plant.first_violation['t'],7.)
        self.assertGreater(plant.state[5],self.model.upper[0])
        with self.assertRaises(ValueError): plant.step(np.zeros(8),.01,-1.)

    def test_fixed_arm_zero_command_stays_fixed_during_base_motion(self):
        plant=self.plant()
        for i in range(20): plant.step(np.r_[.2,0.,np.zeros(6)],.005,-1.)
        np.testing.assert_array_equal(plant.state[5:11],Q_START)
        np.testing.assert_array_equal(plant.state[11:17],np.zeros(6))
        self.assertGreater(plant.state[0],0.)

    def test_atomic_lease_replay_nan_and_format_rejection(self):
        simulator=object.__new__(ActiveArmSimulator)
        simulator.lock=threading.Lock(); simulator.t=1.
        simulator.command_sequence=-1; simulator.command_time=-1e9
        simulator.command_accepted=0; simulator.command_rejected=0
        simulator.whole_command=np.zeros(8); simulator.command=np.zeros(2)
        valid=np.r_[1.,1001.,.25,4.,.1,.2,np.arange(6)*.01]
        simulator.whole_body_command(SimpleNamespace(data=valid))
        np.testing.assert_array_equal(simulator.whole_command,valid[4:])
        self.assertEqual(simulator.command_accepted,1)
        bad=[]
        bad.append(valid.copy()) # replay
        old=valid.copy(); old[1]=1000.; old[3]=5.; bad.append(old)
        long=valid.copy(); long[2]=.26; long[3]=5.; bad.append(long)
        nan=valid.copy(); nan[-1]=np.nan; nan[3]=5.; bad.append(nan)
        bad.append(valid[:-1])
        for value in bad: simulator.whole_body_command(SimpleNamespace(data=value))
        self.assertEqual(simulator.command_rejected,5)
        np.testing.assert_array_equal(simulator.whole_command,valid[4:])


if __name__=='__main__': unittest.main()
