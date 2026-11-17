#ifndef REMANI_MOVING_HANDOFF_TIMING_HPP
#define REMANI_MOVING_HANDOFF_TIMING_HPP

#include <algorithm>
#include <cmath>

namespace remani_planner {
namespace moving_handoff {

// The MPPI adapter requires a 30 ms quiet interval after the last native
// message. Retain another 20 ms at the publication boundary for transport.
constexpr double minimum_publication_lead = 0.05;

inline bool enabled(bool mppi, bool dynamic_repair, bool have_local,
                    bool guarded_stop) {
  return mppi && dynamic_repair && have_local && !guarded_stop;
}

// One captured epoch defines both the old reference phase and the absolute
// activation of the new reference. This is a reference forecast, not a claim
// that physical MPPI motion follows the old curve exactly.
struct Timing {
  double epoch = 0.0;
  double activation = 0.0;
  double phase = 0.0;
  double scale = 0.0;
  double duration = 0.0;

  bool valid() const {
    return std::isfinite(epoch) && std::isfinite(activation) &&
        std::isfinite(phase) && std::isfinite(scale) &&
        std::isfinite(duration) && epoch > 0.0 && activation > epoch &&
        duration > 0.0 && phase >= 0.0 && phase <= duration &&
        scale >= 0.0 && scale <= 1.0;
  }

  double phaseAt(double wall_time) const {
    return std::max(0.0, std::min(duration,
        phase + scale * (wall_time - epoch)));
  }

  bool canPublish(double now,
                  double lead = minimum_publication_lead) const {
    return valid() && std::isfinite(now) && now >= epoch &&
        std::isfinite(lead) && lead >= 0.0 &&
        activation - now >= lead;
  }
};

// Include both endpoints, even when the interval is not a multiple of step.
// The callback receives an old-reference phase and its absolute query time.
template <typename Check>
bool checkBridge(const Timing &timing, double now, double step, Check check) {
  if (!timing.valid() || !std::isfinite(now) || now < timing.epoch ||
      now > timing.activation || !std::isfinite(step) || step <= 0.0)
    return false;
  double wall = now;
  for (;;) {
    if (!check(timing.phaseAt(wall), wall))
      return false;
    if (wall >= timing.activation)
      return true;
    const double next = std::min(timing.activation, wall + step);
    if (next <= wall)
      return false;
    wall = next;
  }
}

}  // namespace moving_handoff
}  // namespace remani_planner

#endif
