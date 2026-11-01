"""Read-only renderer contracts; no ROS or numerical controller execution."""
import tempfile
import unittest
from pathlib import Path
import json
import numpy as np

from render_dual_timescale import Trial, geometric_lateral, latest, bounds
from types import SimpleNamespace


class ReplayContracts(unittest.TestCase):
    def test_no_future_event(self):
        rows=[dict(t=2,value='old'),dict(t=4,value='new')]
        self.assertIsNone(latest(rows,1.9))
        self.assertEqual(latest(rows,3)['value'],'old')
        self.assertEqual(latest(rows,4)['value'],'new')

    def test_segment_lateral_not_sample_distance(self):
        self.assertAlmostEqual(geometric_lateral([.5,.2],[[0,0],[1,0]]),.2)
        self.assertAlmostEqual(geometric_lateral([.5,-.2],[[0,0],[1,0]]),-.2)
        self.assertAlmostEqual(geometric_lateral([2,0],[[0,0],[1,0]]),0.)
        self.assertAlmostEqual(geometric_lateral([.5,.2],[[1,0],[0,0]]),-.2)

    def fixture(self,path):
        (path/'state.csv').write_text('t,x,y,yaw,v,w\n0,0,0,0,0,0\n1,1,0,0,0,0\n2,2,0,0,0,0\n')
        (path/'geometry.csv').write_text('0,0,0.3\n0.2,0,0.2\n')
        (path/'geometry.json').write_text(json.dumps(dict(body_disks_xy_r=[[0,0,.3],[.2,0,.2]])))
        refs=[dict(t=.5,revision=1,reference=[[0,0],[1,0]]),dict(t=1.5,revision=2,reference=[[1,0],[2,1]])]
        (path/'consumed_references.jsonl').write_text(''.join(json.dumps(v)+'\n' for v in refs))
        events=[dict(t=.5,revision=1,event='replace'),dict(t=1.,revision=2,event='invalid'),dict(t=1.5,revision=2,event='replace')]
        (path/'reference_events.jsonl').write_text(''.join(json.dumps(v)+'\n' for v in events))
        ex=[dict(t=.5,reason='ok',lease_s=.075,rollout=[[0,0,0,0,0,0],[.1,.1,0,0,0,0]])]
        (path/'execution_rollouts.jsonl').write_text(''.join(json.dumps(v)+'\n' for v in ex))

    def test_visibility_and_leases(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp);self.fixture(path);trial=Trial(path)
            self.assertIsNone(trial.reference_at(.49))
            np.testing.assert_equal(trial.reference_at(.5),[[0,0],[1,0]])
            self.assertIsNone(trial.reference_at(1.1))
            np.testing.assert_equal(trial.reference_at(1.5),[[1,0],[2,1]])
            self.assertIsNone(trial.selected_at(.49))
            self.assertIsNotNone(trial.selected_at(.55))
            self.assertIsNone(trial.selected_at(.58))
            self.assertIsNone(trial.selected_at(trial.end))
            trial.verify_unchanged()

    def test_point_fallback_forbidden(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)
            (path/'state.csv').write_text('t,x,y,yaw\n0,0,0,0\n1,1,0,0\n')
            with self.assertRaisesRegex(ValueError,'Full recorded body'):
                Trial(path)

    def test_remote_actor_does_not_shrink_offline_camera(self):
        state=np.array([(0.,0.),(1.,0.)],dtype=[('x',float),('y',float)])
        trial=SimpleNamespace(state=state,refs=[],fixed_ref=np.array([[0.,0.],[1.,0.]]),
            obs=np.empty(0),obs_json=[dict(obstacles=[dict(x=100.,y=100.)])],offline=True)
        low,high=bounds([trial])
        self.assertLess(high[0]-low[0],4.)
        self.assertLess(high[1]-low[1],3.)


if __name__=='__main__':unittest.main()
