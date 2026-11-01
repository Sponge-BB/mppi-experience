"""Pure runtime-copy/early-stop/environment checks; no ROS or robot commands."""
import json
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

import native_realtime_runtime as runtime
import run_native_realtime as runner

HERE = Path(__file__).resolve().parent


def diagnostic(t, **kwargs):
    row = dict(t=t, reference_points=100, reason='deadline_miss', code=0,
        validation_code=0, validation=[1, .2, .2, 301, 0, 1],
        nominal_v=.15, nominal_w=0, applied_v=0, applied_w=0, observed_obstacles=[])
    row.update(kwargs)
    return row


def simulator(t=17.):
    return SimpleNamespace(t=t, s=np.zeros(5), diagnostics=[diagnostic(v) for v in np.arange(10, 17.01, .1)],
        events=[], obstacles=lambda: [], native_truth_collision=False)


class Evidence(unittest.TestCase):
    def test_deadline_zero_requires_validation_and_nonzero_nominal(self):
        self.assertTrue(runtime.deadline_zero_pattern(diagnostic(10)))
        for kwargs in (dict(validation_code=3), dict(nominal_v=0), dict(applied_v=.1),
                       dict(reason='native3d_missing_snapshot'), dict(validation=[1, .2, .2, 301, 0, 0])):
            self.assertFalse(runtime.deadline_zero_pattern(diagnostic(10, **kwargs)))

    def test_minimum17s_preserves_graph_audit_and_common_stop(self):
        sim = simulator(16.)
        sim.diagnostics = [row for row in sim.diagnostics if row['t'] <= 16]
        self.assertFalse(runtime.update_trial_evidence(sim))
        sim.t = 17.
        sim.diagnostics.append(diagnostic(17.))
        # A one-second diagnostic gap is unknown, not continuous evidence.
        self.assertFalse(runtime.update_trial_evidence(sim))
        sim = simulator()
        self.assertTrue(runtime.update_trial_evidence(sim))
        self.assertTrue(sim.realtime_evidence['deadline_zero_deadlock'])

    def test_actual_motion_or_collision_cannot_be_labelled_deadline_zero(self):
        sim = simulator()
        sim.s[3] = .01
        self.assertFalse(runtime.update_trial_evidence(sim))
        sim = simulator()
        sim.native_truth_collision = True
        self.assertFalse(runtime.update_trial_evidence(sim))

    def test_zero_reference_diagnostic_interrupts_continuous_deadlock_window(self):
        sim = simulator()
        index = min(range(len(sim.diagnostics)), key=lambda i: abs(sim.diagnostics[i]['t']-13.))
        sim.diagnostics[index] = diagnostic(13., reference_points=0, reason='waiting_native_trajectory')
        self.assertFalse(runtime.update_trial_evidence(sim))
        self.assertGreater(sim.realtime_evidence['deadline_zero_window_start'], 13.)

    def test_actual_obstacle_and_moving_input_evidence_only_when_present(self):
        sim = simulator()
        sim.obstacles = lambda: [(1., 0., .23, 1, 0, .2)]
        sim.diagnostics[-1]['observed_obstacles'] = [[1, 0, 0, .2, .23, 1]]
        runtime.update_trial_evidence(sim)
        self.assertEqual(sim.realtime_evidence['first_truth_actor_t'], 17.)
        self.assertIsNotNone(sim.realtime_evidence['first_observed_moving_t'])


class RuntimeCopies(unittest.TestCase):
    def test_groups_identical_python_source_only_launch_switch_differs(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            observed = []
            parent_hashes = {name: (HERE/name).read_bytes() for name in ('mppi_node.py', 'local_execution.py', 'simulator.py', 'native_collision_runtime.py')}
            for demand in (False, True):
                out = root/str(demand)
                (out/'runtime').mkdir(parents=True)
                for name in ('mppi_node.py', 'local_execution.py', 'simulator.py'):
                    (out/'runtime'/name).write_bytes(parent_hashes[name])
                (out/'experiment.launch').write_text('<launch><node name="base_mppi"/><node name="remani_planner_node"/></launch>')
                (out/'manifest.json').write_text(json.dumps({'experiment_sources_sha256': {}}))
                runtime.patch_trial(out, demand)
                observed.append({p.name: p.read_bytes() for p in (out/'runtime').glob('*.py')})
                for name, text in observed[-1].items():
                    compile(text, name, 'exec')
                text = (out/'runtime/simulator.py').read_text()
                self.assertIn("reason='deadline_zero_deadlock'", text)
                self.assertNotIn('self.s=np.array([0.,1.5', text)
                launch = (out/'experiment.launch').read_text()
                self.assertIn('name="NATIVE_REALTIME_DEMAND" value="%d"' % demand, launch)
                self.assertIn('name="NATIVE_REALTIME_PROFILE" value="0"', launch)
                self.assertIn('name="NATIVE_COLLISION_3D" value="1"', launch)
            self.assertEqual(observed[0], observed[1])
            for name, content in parent_hashes.items():
                self.assertEqual((HERE/name).read_bytes(), content)

    def test_profile_replay_inheritance_is_cleared_and_restored(self):
        inherited = dict(NATIVE_REALTIME_PROFILE='1', NATIVE_REALTIME_REPLAY='1',
            NATIVE_REALTIME_DEMAND='1', NATIVE_MPPI_PROFILE='1', FEASIBILITY_STOP_GATE='1')
        with patch.dict(os.environ, inherited, clear=True):
            with runner.clean_environment():
                self.assertEqual(os.environ['NATIVE_REALTIME_PROFILE'], '0')
                self.assertEqual(os.environ['NATIVE_REALTIME_REPLAY'], '0')
                self.assertNotIn('NATIVE_REALTIME_DEMAND', os.environ)
                self.assertNotIn('NATIVE_MPPI_PROFILE', os.environ)
                self.assertEqual(os.environ['FEASIBILITY_STOP_GATE'], '0')
            self.assertEqual(dict(os.environ), inherited)


if __name__ == '__main__':
    unittest.main()

