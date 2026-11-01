"""Check the new world/goal and terminal cutoff without starting ROS."""
import unittest
from types import SimpleNamespace

import numpy as np

from report_long_goal_crossing import (measured_rows, sampled_gap, scene_actors,
                                       settings, terminal_settling_start, world_transform)


class LongCrossReportTests(unittest.TestCase):
    def setUp(self):
        self.bounds = [-1.5, 7.5, -2., 2.]
        self.manifest = dict(scenario_start='motion',
            long_goal_crossing_experiment=dict(goal=[6., 0.], bounds=self.bounds),
            scenario=dict(appearance_s=2., actors=[dict(id=1, x=3., y=-1.6, vx=0., vy=.3, r=.2)]))

    def test_goal_and_world_are_recorded_not_old_defaults(self):
        _, goal, bounds = settings(self.manifest)
        self.assertEqual(goal.tolist(), [6., 0.])
        xy, _ = world_transform(bounds, (10, 10, 1010, 500))
        self.assertLess(xy(*goal)[0], xy(7.5, 0.)[0])
        body = np.array([[0., 0., .5, .13, .18, 50000., 0.]])
        gap = sampled_gap(np.array([4.5, 0., 0.]), body, .05, [], bounds)
        self.assertGreater(gap, 1.)
        self.assertAlmostEqual(sampled_gap(np.array([7.5, 0., 0.]), body, .05, [], bounds), -.08)

    def test_actor_uses_recorded_motion_anchor_and_velocity(self):
        result = dict(motion_anchor=10., goal_sent=5.)
        self.assertEqual(scene_actors(self.manifest, result, 11.99), [])
        actor = scene_actors(self.manifest, result, 14.)[0]
        self.assertAlmostEqual(actor[0], 3.)
        self.assertAlmostEqual(actor[1], -1.)

    def test_measurements_exclude_late_state_rows(self):
        state = np.array([(1., .1), (2., .2), (3., .3), (4., .4)], dtype=[('t', float), ('v', float)])
        trial = SimpleNamespace(state=state, anchor=2., end=3.)
        self.assertEqual(measured_rows(trial)['t'].tolist(), [2., 3.])

    def test_goal_settling_uses_six_metre_goal(self):
        state = np.array([(1., 3., 0., 0., 0.), (2., 5.9, 0., 0., .1),
                          (3., 5.98, 0., 0., .01), (4., 5.99, 0., 0., 0.)],
                         dtype=[('t', float), ('x', float), ('y', float), ('yaw', float), ('v', float)])
        limits = dict(goal_pos=.05, goal_yaw=.1, goal_speed=.05)
        self.assertEqual(terminal_settling_start(state, limits, [6., 0.], 4.), 3.)
        self.assertIsNone(terminal_settling_start(state, limits, [3., 0.], 4.))


if __name__ == '__main__':
    unittest.main()
