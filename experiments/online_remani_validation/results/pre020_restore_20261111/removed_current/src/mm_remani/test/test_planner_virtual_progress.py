"""Compile the actual planner phase helpers with ROS time, without a ROS graph."""
from pathlib import Path
import json
import subprocess
import tempfile
import unittest


SOURCE = (Path(__file__).resolve().parents[2] / 'mm_remani_planner' /
          'plan_manage/src/remani_replan_fsm.cpp')


def phase_methods():
    source = SOURCE.read_text()
    methods = []
    for name in ('getCurrentTrajectoryTime', 'getCurrentTrajectoryScale'):
        start = source.index('double REMANIReplanFSM::' + name + '(')
        opening = source.index('{', start)
        depth = 1
        end = opening + 1
        while depth:
            depth += (source[end] == '{') - (source[end] == '}')
            end += 1
        methods.append(source[start:end])
    return '\n'.join(methods)


class PlannerVirtualProgressTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix='remani_phase_test_')
        cls.addClassCleanup(cls.temp.cleanup)
        directory = Path(cls.temp.name)
        cpp = directory / 'phase.cpp'
        cls.binary = directory / 'phase'
        cpp.write_text(r'''
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <ros/time.h>
struct SingulTrajData { double start_time, duration; };
class REMANIReplanFSM {
public:
  bool have_controller_progress_;
  double controller_trajectory_start_time_, controller_trajectory_time_;
  double controller_trajectory_scale_, controller_progress_timeout_;
  ros::Time controller_progress_stamp_;
  double getCurrentTrajectoryTime(const SingulTrajData&, const ros::Time&) const;
  double getCurrentTrajectoryScale(const SingulTrajData&, const ros::Time&) const;
};
''' + phase_methods() + r'''
int main(int argc, char **argv) {
  if (argc != 10) return 2;
  REMANIReplanFSM f;
  const ros::Time now(std::stod(argv[1]));
  f.controller_progress_stamp_.fromSec(std::stod(argv[2]));
  f.controller_trajectory_start_time_ = std::stod(argv[3]);
  f.controller_trajectory_time_ = std::stod(argv[4]);
  f.controller_trajectory_scale_ = std::stod(argv[5]);
  f.have_controller_progress_ = std::stoi(argv[6]);
  SingulTrajData t{std::stod(argv[8]), std::stod(argv[7])};
  f.controller_progress_timeout_ = std::stod(argv[9]);
  std::cout << std::setprecision(17) << "{\"phase\":"
            << f.getCurrentTrajectoryTime(t, now) << ",\"scale\":"
            << f.getCurrentTrajectoryScale(t, now) << "}\n";
}
''')
        subprocess.run(['/usr/bin/c++', '-std=c++14', '-O0',
                        '-I/opt/ros/noetic/include', str(cpp),
                        '-L/opt/ros/noetic/lib', '-Wl,-rpath,/opt/ros/noetic/lib',
                        '-lrostime', '-lcpp_common', '-o', str(cls.binary)],
                       check=True, capture_output=True, text=True)

    def evaluate(self, now=100., stamp=99.9, start=10., phase=.4, scale=.5,
                 have=True, duration=5., trajectory_start=10., timeout=.5):
        args = [now, stamp, start, phase, scale, int(have), duration,
                trajectory_start, timeout]
        return json.loads(subprocess.check_output(
            [str(self.binary)] + [str(value) for value in args], text=True))

    def test_fresh_virtual_phase_does_not_follow_elapsed_wall_time(self):
        self.assertEqual(self.evaluate(), {'phase': .45, 'scale': .5})

    def test_expired_matching_progress_cannot_complete_or_advance_handoff(self):
        result = self.evaluate(stamp=99.4)
        self.assertEqual(result, {'phase': .4, 'scale': 0.})
        self.assertLess(result['phase'], 5. - .01)  # FSM completion threshold.
        self.assertEqual(result['phase'] + .7 * result['scale'], .4)

    def test_long_gap_and_paused_reference_hold_reported_phase(self):
        for now in (100., 1000.):
            for scale in (0., .5, 1.):
                with self.subTest(now=now, scale=scale):
                    self.assertEqual(self.evaluate(now=now, stamp=99.4, scale=scale),
                                     {'phase': .4, 'scale': 0.})

    def test_new_matching_feedback_resumes_without_trajectory_replacement(self):
        self.assertEqual(self.evaluate(stamp=99.4)['scale'], 0.)
        result = self.evaluate(stamp=100., phase=.47, scale=.6)
        self.assertEqual(result, {'phase': .47, 'scale': .6})

    def test_future_progress_does_not_authorize_forward_extrapolation(self):
        for stamp in (100.006, 100.05, 101.):
            with self.subTest(stamp=stamp):
                self.assertEqual(self.evaluate(stamp=stamp), {'phase': .4, 'scale': 0.})
        # One bounded transport/clock tick keeps the existing no-negative-phase policy.
        self.assertEqual(self.evaluate(stamp=100.004), {'phase': .4, 'scale': .5})

    def test_exact_expiry_boundary_and_duration_clamping(self):
        self.assertEqual(self.evaluate(stamp=99.5), {'phase': .65, 'scale': .5})
        self.assertEqual(self.evaluate(stamp=99.499), {'phase': .4, 'scale': 0.})
        self.assertEqual(self.evaluate(stamp=99.4, phase=6.)['phase'], 5.)
        self.assertEqual(self.evaluate(stamp=99.4, phase=-1.)['phase'], 0.)

    def test_missing_or_different_trajectory_preserves_legacy_wall_clock(self):
        for params in ({'have': False}, {'start': 9.}, {'stamp': 0.}):
            with self.subTest(params=params):
                self.assertEqual(self.evaluate(now=11., **params), {'phase': 1., 'scale': 1.})


if __name__ == '__main__':
    unittest.main()
