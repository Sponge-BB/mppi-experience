#pragma once
// Isolated fixed-arm coupling experiment. Geometry and units match Snapshot,
// but future dynamic costs use the queried trajectory time, including dJ/dt.
// This file does not change shared_collision.hpp or any production behavior.
#include "shared_collision.hpp"
#include <limits>

namespace coupling_collision {
using Snapshot = shared_collision::Snapshot;
struct Options {
  bool include_static = true;
  bool include_dynamic = true;
  // C2 soft-cost turn-off over the final bounded prediction interval. This
  // is NOT a safety relaxation: physical/gap are never tapered. A hard
  // validator must still reject unknown dynamic safety beyond the horizon.
  double dynamic_soft_taper_seconds = .25;
};
// No speed/uncertainty-dependent radius is introduced in this experiment.
constexpr double soft_motion_reserve = 0.;
struct Evaluation {
  double cost = 0., gx = 0., gy = 0., gyaw = 0., gt = 0.;
  double physical = std::numeric_limits<double>::infinity();
  double gap = std::numeric_limits<double>::infinity();
  bool valid = true;
};
inline Evaluation invalid() {
  Evaluation value;
  value.valid = false;
  value.cost = 1e12;
  value.physical = value.gap = -1e6;
  return value;
}
struct TemporalWeight { double value = 1., derivative = 0.; };
inline TemporalWeight temporalWeight(double forecast, double horizon, double width) {
  if (width == 0. || forecast <= horizon-width) return {};
  const double u = (forecast-(horizon-width))/width;
  // 1-smoothstep5(u), with zero first/second derivatives at both ends.
  return {1.-u*u*u*(10.+u*(-15.+6.*u)),
          -30.*u*u*(1.-u)*(1.-u)/width};
}

// forecast = absolute trajectory-query time - observation stamp, in seconds.
// It is NOT time since the optimizer began and must include observation age
// plus trajectory offset. One immutable Snapshot should cover each solve.
// Snapshot.healthy/freshness is checked at the caller's actual wall/ROS time;
// this pure kernel cannot infer that time from forecast.
inline Evaluation evaluate(const Snapshot& m, double x, double y, double yaw,
                           double forecast, const Options& options = {}) {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) ||
      !std::isfinite(forecast) || forecast < 0. || m.body.empty() ||
      !std::isfinite(m.margin) || m.margin < 0. ||
      !std::isfinite(m.comfort) || m.comfort < m.margin ||
      !std::isfinite(m.weight) || m.weight <= 0. ||
      !std::isfinite(options.dynamic_soft_taper_seconds) ||
      options.dynamic_soft_taper_seconds < 0. ||
      (!options.include_static && !options.include_dynamic)) return invalid();
  if (options.include_static &&
      (m.nx < 2 || m.ny < 2 || !std::isfinite(m.res) || m.res <= 0. ||
       !std::isfinite(m.gx) || !std::isfinite(m.gy) ||
       m.field.size() != size_t(m.nx)*size_t(m.ny))) return invalid();
  const bool has_predictions = options.include_dynamic && !m.obstacles.empty();
  if (has_predictions &&
      (!std::isfinite(m.max_prediction) || m.max_prediction <= 0. ||
       forecast > m.max_prediction ||
       options.dynamic_soft_taper_seconds > m.max_prediction)) return invalid();
  const auto temporal = has_predictions
    ? temporalWeight(forecast, m.max_prediction, options.dynamic_soft_taper_seconds)
    : TemporalWeight{};
  const double co = std::cos(yaw), si = std::sin(yaw);
  Evaluation out;
  for (const auto& body : m.body) {
    if (!std::isfinite(body.x) || !std::isfinite(body.y) ||
        !std::isfinite(body.r) || body.r <= 0.) return invalid();
    const double px = x+co*body.x-si*body.y;
    const double py = y+si*body.x+co*body.y;
    const double yaw_x = -si*body.x-co*body.y;
    const double yaw_y = co*body.x-si*body.y;
    auto charge = [&](double surface, double dx, double dy, double vx, double vy,
                      TemporalWeight weight) {
      const double physical = surface-body.r;
      out.physical = std::min(out.physical, physical);
      out.gap = std::min(out.gap, physical-m.margin);
      const double error = std::max(0., m.comfort-physical);
      const double raw_cost = m.weight*error*error*error;
      const double factor = -3.*m.weight*error*error;
      const double raw_gx = factor*dx, raw_gy = factor*dy;
      out.cost += weight.value*raw_cost;
      out.gx += weight.value*raw_gx;
      out.gy += weight.value*raw_gy;
      out.gyaw += weight.value*(raw_gx*yaw_x+raw_gy*yaw_y);
      out.gt += -weight.value*(raw_gx*vx+raw_gy*vy)
                +weight.derivative*raw_cost;
    };
    if (options.include_static) {
      const double fx = (px-m.gx)/m.res, fy = (py-m.gy)/m.res;
      // Do not cast huge/unbounded coordinates to int or silently return
      // free space outside the observed map. Last grid nodes lack a full cell.
      if (!std::isfinite(fx) || !std::isfinite(fy) || fx < 0. || fy < 0. ||
          fx >= m.nx-1 || fy >= m.ny-1) return invalid();
      const int ix = int(std::floor(fx)), iy = int(std::floor(fy));
      const double q[] = {m.field[iy*m.nx+ix], m.field[iy*m.nx+ix+1],
        m.field[(iy+1)*m.nx+ix], m.field[(iy+1)*m.nx+ix+1]};
      for (double d : q) if (!std::isfinite(d) || d < 0.) return invalid();
      const auto distance = shared_collision::mapDistance(m, px, py);
      charge(distance.d, distance.dx, distance.dy, 0., 0., {});
    }
    if (options.include_dynamic) for (const auto& obstacle : m.obstacles) {
      if (!std::isfinite(obstacle.x) || !std::isfinite(obstacle.y) ||
          !std::isfinite(obstacle.vx) || !std::isfinite(obstacle.vy) ||
          !std::isfinite(obstacle.r) || obstacle.r <= 0. ||
          (obstacle.moving != 0. && obstacle.moving != 1.)) return invalid();
      const double vx = obstacle.vx*obstacle.moving;
      const double vy = obstacle.vy*obstacle.moving;
      const double dx = px-obstacle.x-forecast*vx;
      const double dy = py-obstacle.y-forecast*vy;
      const double distance = std::hypot(dx, dy);
      if (!std::isfinite(distance)) return invalid();
      // Norm has no unique derivative at zero. Zero is a bounded symmetric
      // subgradient; the overlapping geometry remains unsafe through gap.
      charge(distance-obstacle.r, distance > 1e-12 ? dx/distance : 0.,
             distance > 1e-12 ? dy/distance : 0., vx, vy, temporal);
    }
  }
  if (!std::isfinite(out.cost) || !std::isfinite(out.gx) ||
      !std::isfinite(out.gy) || !std::isfinite(out.gyaw) ||
      !std::isfinite(out.gt)) return invalid();
  return out;
}
}  // namespace coupling_collision
