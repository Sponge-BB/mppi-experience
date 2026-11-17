#pragma once
// Source-stamped early refresh request, never permission to continue motion.
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace mppi_refresh {
struct Event {
  std::array<double, 10> data{};
  double goal() const { return data[2]; }
  double reference() const { return data[3]; }
  double episode() const { return data[9]; }
};
inline bool parse(const std::vector<double>& values, Event& output) {
  output = Event();
  if (values.size() != 10 || values[0] != 1.) return false;
  for (double value : values) if (!std::isfinite(value)) return false;
  for (std::size_t i=1; i<7; ++i) if (values[i] <= 0.) return false;
  if ((values[8] != 0. && values[8] != 1.) ||
      (values[8] == 1. && values[7] <= 0.) ||
      values[9] <= 0. || values[9] > values[1]+.005) return false;
  std::copy(values.begin(), values.end(), output.data.begin());
  return true;
}
inline bool age(double stamp, double now, double maximum) {
  return stamp > 0. && now-stamp >= -.005 && now-stamp <= maximum;
}
inline bool eligible(const Event& event, double now, double goal,
                     double reference, double consumed_goal,
                     double consumed_episode) {
  const auto& d = event.data;
  return std::isfinite(now) && std::abs(d[2]-goal) < 1e-6 &&
      std::abs(d[3]-reference) < 1e-6 && age(d[1], now, .25) &&
      age(d[4], now, .15) && age(d[5], now, .15) &&
      age(d[6], now, .35) && (d[8] == 0. || age(d[7], now, .35)) &&
      !(std::abs(d[2]-consumed_goal) < 1e-6 && event.episode() <= consumed_episode);
}
}  // namespace mppi_refresh
