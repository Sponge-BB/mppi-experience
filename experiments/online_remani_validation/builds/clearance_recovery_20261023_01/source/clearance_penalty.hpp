#pragma once
// Isolated static-clearance recovery objective. Never changes geometry, native
// costs, hard safety margins, or acceptance. Finite rho is NOT a certificate.
#include "coupling_collision.hpp"

namespace clearance_penalty {
using Snapshot = coupling_collision::Snapshot;
using Evaluation = coupling_collision::Evaluation;
// Predeclared before the recovery trials; unrelated to Snapshot::weight.
constexpr double fixed_rho = 1.0e5;

// lambda==nullptr is J=.5*rho*sum_body max(0,margin-physical_body)^2.
// Optional immutable nonnegative per-body multipliers give one shifted AL
// penalty: ((max(0,lambda+rho*h))^2-lambda^2)/(2*rho), h=margin-physical.
// The original static evaluator remains authoritative for validity/domain
// semantics and the full-projection physical/gap minima. Its aggregate soft
// gx/gy/gyaw must NOT be mistaken for derivatives of the minimum hard gap.
inline Evaluation evaluate(const Snapshot& frame, double x, double y, double yaw,
                           double rho = fixed_rho,const double* lambda=nullptr,
                           size_t lambda_count=0) {
  if (!std::isfinite(rho) || rho <= 0.) return coupling_collision::invalid();
  if ((lambda && lambda_count!=frame.body.size()) || (!lambda && lambda_count))
    return coupling_collision::invalid();
  if (lambda) for(size_t i=0;i<lambda_count;++i)
    if(!std::isfinite(lambda[i]) || lambda[i]<0.) return coupling_collision::invalid();
  coupling_collision::Options options;
  options.include_dynamic = false;
  options.dynamic_soft_taper_seconds = 0.;
  auto out = coupling_collision::evaluate(frame, x, y, yaw, 0., options);
  if (!out.valid) return out;
  out.cost = out.gx = out.gy = out.gyaw = out.gt = 0.;
  const double co = std::cos(yaw), si = std::sin(yaw);
  size_t body_index=0;
  for (const auto& body : frame.body) {
    const double px = x + co*body.x - si*body.y;
    const double py = y + si*body.x + co*body.y;
    // evaluate() has validated every queried bilinear cell and body already.
    const auto distance = shared_collision::mapDistance(frame, px, py);
    const double h = frame.margin - (distance.d-body.r);
    const double multiplier=lambda ? lambda[body_index] : 0.;
    const double shifted=multiplier+rho*h;
    if(!std::isfinite(shifted)) return coupling_collision::invalid();
    const double factor = -std::max(0.,shifted);
    const double gx = factor*distance.dx, gy = factor*distance.dy;
    // This algebra avoids cancellation between shifted^2 and lambda^2.
    out.cost += shifted>0. ? h*(.5*rho*h+multiplier)
                          : -.5*multiplier*(multiplier/rho);
    out.gx += gx;
    out.gy += gy;
    out.gyaw += gx*(-si*body.x-co*body.y) + gy*(co*body.x-si*body.y);
    ++body_index;
  }
  if (!std::isfinite(out.cost) || !std::isfinite(out.gx) ||
      !std::isfinite(out.gy) || !std::isfinite(out.gyaw))
    return coupling_collision::invalid();
  return out;
}
}  // namespace clearance_penalty
