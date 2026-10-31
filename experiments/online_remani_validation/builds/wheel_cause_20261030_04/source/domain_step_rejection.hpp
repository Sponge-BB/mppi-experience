#pragma once
// Isolated SIL optimization signal, never a collision-safety certificate.
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace domain_step {

struct Counters {
  std::size_t rejected_trials = 0;
  std::size_t accepted_steps_after_rejection = 0;
};
inline Counters& counters() {
  static thread_local Counters value;
  return value;
}
// Reset once at each solve boundary, before the initial objective evaluation.
inline void reset() { counters() = Counters{}; }
inline std::size_t rejectedTrials() { return counters().rejected_trials; }
inline std::size_t acceptedStepsAfterRejection() {
  return counters().accepted_steps_after_rejection;
}
// Called by the experimental line search ONLY, not by exception construction,
// objective diagnostics, initial evaluations, or attempted Wolfe evaluations.
inline void recordRejected() { ++counters().rejected_trials; }
inline void recordAcceptedAfterRejection() {
  ++counters().accepted_steps_after_rejection;
}

// Only a finite query outside a well-formed half-open static-map domain may
// construct this recoverable signal. Snapshot health, field validity, native
// arithmetic, and all other prerequisites remain the caller's responsibility.
// All unrelated failures must retain their original fatal exception/status.
class RejectedTrial final : public std::runtime_error {
public:
  const double x, y, lower_x, lower_y, upper_x, upper_y;
  const std::size_t body_index;

  RejectedTrial(double px, double py, double lx, double ly, double ux, double uy,
                std::size_t body = 0)
      : std::runtime_error("finite projected static query outside map domain"),
        x(px), y(py), lower_x(lx), lower_y(ly), upper_x(ux), upper_y(uy),
        body_index(body) {
    if (!std::isfinite(x) || !std::isfinite(y) ||
        !std::isfinite(lower_x) || !std::isfinite(lower_y) ||
        !std::isfinite(upper_x) || !std::isfinite(upper_y) ||
        !(lower_x < upper_x) || !(lower_y < upper_y))
      throw std::invalid_argument("domain rejection requires finite query and valid bounds");
    if (!(x < lower_x || y < lower_y || x >= upper_x || y >= upper_y))
      throw std::invalid_argument("domain rejection requires an out-of-domain query");
  }
};

}  // namespace domain_step
