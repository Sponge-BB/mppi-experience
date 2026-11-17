// Standalone behavior test of the timing policy used by the production FSM.
// No ROS, planner build, or robot is needed.
#include <plan_manage/moving_handoff_timing.hpp>

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

using remani_planner::moving_handoff::Timing;
namespace policy = remani_planner::moving_handoff;

void require(bool value, const char *message) {
  if (!value) throw std::runtime_error(message);
}
void close(double a, double b, const char *message) {
  require(std::fabs(a - b) < 1e-9, message);
}

int main() {
  try {
    // Initial, native, stopped and ordinary replans keep their old behavior.
    for (int flags = 0; flags < 16; ++flags) {
      const bool mppi = flags & 1, dynamic = flags & 2;
      const bool local = flags & 4, stopped = flags & 8;
      require(policy::enabled(mppi, dynamic, local, stopped) ==
              (mppi && dynamic && local && !stopped), "branch scope changed");
    }

    Timing timing{100., 100.7, 6., .4, 12.};
    require(timing.valid(), "valid moving splice rejected");
    require(timing.canPublish(100.2), "early solution lost its activation lead");
    // Solving and validation are two different deadline checks. Neither
    // changes the fixed activation or recomputes now + handoff.
    require(timing.canPublish(100.64), "pre-validation acceptance rejected");
    require(!timing.canPublish(100.66), "validation consumed required lead");
    require(!timing.canPublish(100.71), "late moving solution was accepted");
    close(timing.activation, 100.7, "activation was rebased after validation");
    require(!timing.canPublish(99.9), "clock reversal accepted");
    close(timing.phaseAt(timing.activation), 6.28, "wrong boundary phase");

    std::vector<double> times, phases;
    require(policy::checkBridge(timing, 100.31, .05,
        [&](double phase, double wall) {
          phases.push_back(phase); times.push_back(wall); return true;
        }), "safe old interval rejected");
    close(times.front(), 100.31, "old curve was checked from obsolete solve time");
    close(times.back(), 100.7, "activation endpoint was omitted");
    close(phases.front(), 6.124, "old reference phase was reset at validation");
    close(phases.back(), 6.28, "old endpoint and new boundary disagree");
    for (std::size_t i = 1; i < times.size(); ++i) {
      require(times[i] > times[i-1] && times[i]-times[i-1] <= .05+1e-9,
              "bridge sampling has a hole");
      close(phases[i], 6.+.4*(times[i]-100.), "mixed planning/validation epochs");
    }

    // A clear new starting point cannot authorize an unsafe old interval.
    // Exercise static, live and time-indexed collision callback failures.
    for (int layer = 0; layer < 3; ++layer) {
      int checks = 0;
      const bool clear = policy::checkBridge(timing, 100.31, .05,
          [&](double phase, double wall) {
            ++checks;
            if (layer == 0) return !(phase > 6.17 && phase < 6.23);
            if (layer == 1) return !(phase > 6.21 && phase < 6.25);
            return !(wall > 100.4 && wall < 100.5);
          });
      require(!clear && checks > 1, "unsafe pre-activation interval accepted");
    }

    Timing capped{100., 100.7, 11.9, .4, 12.};
    require(policy::checkBridge(capped, 100.31, .05,
        [](double phase, double) { return phase <= 12.; }), "endpoint hold rejected");
    close(capped.phaseAt(100.7), 12., "old curve extrapolated past endpoint");
    require(!policy::checkBridge(timing, 100.71, .05,
        [](double, double) { return true; }), "expired bridge accepted");
    require(!policy::checkBridge(timing, 100.2, 0.,
        [](double, double) { return true; }), "zero sampling interval accepted");
    require(!policy::checkBridge(timing, 99.99, .05,
        [](double, double) { return true; }), "pre-epoch bridge accepted");
    Timing invalid = timing;
    invalid.scale = std::numeric_limits<double>::quiet_NaN();
    require(!invalid.canPublish(100.2), "invalid clock accepted");
    std::cout << "moving handoff timing behavior: PASS\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
